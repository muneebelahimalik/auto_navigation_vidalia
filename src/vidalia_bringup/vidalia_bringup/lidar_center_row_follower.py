#!/usr/bin/env python3
"""
lidar_center_row_follower.py — Online LiDAR center-row follower for the Amiga.

The Amiga straddles the crop row: wheels travel in clear tire tracks while
the robot frame passes over the planted center band.  This node detects the
CENTER crop row directly under the robot from VLP-16 data and steers to keep
the robot centerline aligned over it.

Detection pipeline (per PointCloud2 at ~10 Hz):
  1. Filter points to the center-crop ROI (forward strip, crop height band).
  2. Project to Bird's-Eye View (x–y ground plane).
  3. Fit dominant row direction via PCA on BEV points.
  4. Compute RANSAC inlier ratio to score line quality.
  5. Score row confidence from point count, PCA elongation, inlier ratio.
  6. Smooth heading and lateral-offset estimates with EMA across frames.
  7. Generate /cmd_vel only when autonomous_mode:=true.

State machine:
  ACQUIRING     — Searching for the center row (startup / after obstacle).
  FOLLOWING     — Confident row locked; controller active in autonomous mode.
  ROW_END       — Row confidence lost long enough past minimum travel distance.
  OBSTACLE_WAIT — Forward safety zone or tire path blocked; zero velocity.

Coordinate frame (velodyne, with default zero-yaw mount):
  x = forward  (same axis as base_link +x)
  y = left     (same axis as base_link +y)
  z = up
  lidar_x_offset=0.959 m forward from base_link origin
  lidar_z_offset=0.80 m above ground (2026-07 re-mount)

Usage:
  # Perception-only (inspect RViz before enabling motion):
  ros2 launch vidalia_bringup row_follower.launch.py autonomous_mode:=false

  # Autonomous (slow):
  ros2 launch vidalia_bringup row_follower.launch.py autonomous_mode:=true max_linear_speed:=0.20

RViz topics to add:
  MarkerArray  /row/markers        — crop points, row line, target, confidence text
  String       /row/state
  Float64      /row/confidence
  Float64      /row/lateral_offset (metres, + = row is to the left)
  Float64      /row/heading_error  (radians, + = row tilts left → steer left)
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as BTime
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────────────────
# PointCloud2 parsing  (no sensor_msgs_py dependency)
# ─────────────────────────────────────────────────────────────────────────────

def _extract_xyz(cloud: PointCloud2) -> Optional[np.ndarray]:
    """Return Nx3 float32 array (x, y, z) from a PointCloud2, or None."""
    field_map = {f.name: f.offset for f in cloud.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return None
    step = cloud.point_step
    raw = np.frombuffer(bytes(cloud.data), dtype=np.uint8)
    n = raw.shape[0] // step
    if n == 0:
        return None
    pts = raw[:n * step].reshape(n, step)
    ox, oy, oz = field_map['x'], field_map['y'], field_map['z']
    x = pts[:, ox:ox + 4].copy().view(np.float32).ravel()
    y = pts[:, oy:oy + 4].copy().view(np.float32).ravel()
    z = pts[:, oz:oz + 4].copy().view(np.float32).ravel()
    xyz = np.column_stack([x, y, z]).astype(np.float64)
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid] if valid.any() else None


# ─────────────────────────────────────────────────────────────────────────────
# Zone filtering
# ─────────────────────────────────────────────────────────────────────────────

def _box(
    xyz: np.ndarray,
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    z_min: float, z_max: float,
) -> np.ndarray:
    """Return points inside an axis-aligned box."""
    m = (
        (xyz[:, 0] >= x_min) & (xyz[:, 0] <= x_max) &
        (xyz[:, 1] >= y_min) & (xyz[:, 1] <= y_max) &
        (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    )
    return xyz[m]


# ─────────────────────────────────────────────────────────────────────────────
# Row geometry  (all inputs: Nx2 BEV, x=forward, y=left in velodyne frame)
# ─────────────────────────────────────────────────────────────────────────────

def _pca_row(
    pts_xy: np.ndarray,
) -> Tuple[float, float, float]:
    """
    PCA on Nx2 BEV points.

    Returns
    -------
    heading_err : float  — angle of primary axis from +x (forward), rad.
                           Positive = row tilts toward left → steer left.
    lateral_off : float  — y-coordinate of centroid, metres.
                           Positive = row centre is to the left → steer left.
    elongation  : float  — largest-eigenvalue fraction [0, 1].
                           Near 1 = linear (row-like), near 0 = isotropic (blob).
    """
    centroid = pts_xy.mean(axis=0)
    centered = pts_xy - centroid
    cov = centered.T @ centered
    eigvals, eigvecs = np.linalg.eigh(cov)   # ascending order
    principal = eigvecs[:, -1]
    if principal[0] < 0:   # enforce forward-pointing convention
        principal = -principal
    heading_err = float(math.atan2(principal[1], principal[0]))
    lateral_off = float(centroid[1])
    total = float(eigvals.sum())
    elongation = float(eigvals[-1] / total) if total > 1e-9 else 0.0
    return heading_err, lateral_off, elongation


def _ransac_inlier_ratio(
    pts_xy: np.ndarray,
    n_iter: int = 40,
    dist_thresh: float = 0.12,
) -> float:
    """Fraction of points within dist_thresh of the best-fit RANSAC line."""
    n = len(pts_xy)
    if n < 4:
        return 0.0
    rng = np.random.default_rng(seed=42)
    best = 0
    for _ in range(n_iter):
        i, j = rng.choice(n, 2, replace=False)
        d = pts_xy[j] - pts_xy[i]
        length = math.hypot(d[0], d[1])
        if length < 0.05:
            continue
        normal = np.array([-d[1], d[0]]) / length
        dists = np.abs((pts_xy - pts_xy[i]) @ normal)
        count = int((dists < dist_thresh).sum())
        if count > best:
            best = count
    return best / n


def _row_confidence(
    n_pts: int,
    elongation: float,
    inlier_ratio: float,
    min_pts: int,
    good_pts: int,
) -> float:
    """Combine count, elongation, inlier ratio into a [0, 1] confidence."""
    if n_pts < min_pts:
        return 0.0
    density  = min(1.0, (n_pts - min_pts) / max(1, good_pts - min_pts))
    # Elongation ≥ 0.55 = clearly linear; < 0.45 = blob. Ramp between.
    linear   = float(np.clip((elongation - 0.45) / 0.30, 0.0, 1.0))
    return float(density * linear * inlier_ratio)


# ─────────────────────────────────────────────────────────────────────────────
# Temporal smoother
# ─────────────────────────────────────────────────────────────────────────────

class _RowSmoother:
    """Exponential moving average for heading, lateral offset, and confidence."""

    def __init__(self, alpha: float = 0.35, history: int = 6) -> None:
        self._alpha = alpha
        self._heading: Optional[float] = None
        self._lateral: Optional[float] = None
        self._conf_buf: Deque[float] = deque(maxlen=history)

    def update(self, heading: float, lateral: float, conf: float) -> None:
        self._conf_buf.append(conf)
        if self._heading is None:
            self._heading, self._lateral = heading, lateral
            return
        diff = (heading - self._heading + math.pi) % (2 * math.pi) - math.pi
        self._heading += self._alpha * diff
        self._lateral += self._alpha * (lateral - self._lateral)

    def reset(self) -> None:
        self._heading = self._lateral = None
        self._conf_buf.clear()

    @property
    def heading(self) -> float:
        return self._heading if self._heading is not None else 0.0

    @property
    def lateral(self) -> float:
        return self._lateral if self._lateral is not None else 0.0

    @property
    def smoothed_confidence(self) -> float:
        return float(np.mean(self._conf_buf)) if self._conf_buf else 0.0

    @property
    def initialized(self) -> bool:
        return self._heading is not None


# ─────────────────────────────────────────────────────────────────────────────
# State constants
# ─────────────────────────────────────────────────────────────────────────────

class _S:
    ACQUIRING     = 'ACQUIRING'
    FOLLOWING     = 'FOLLOWING'
    ROW_END       = 'ROW_END'
    OBSTACLE_WAIT = 'OBSTACLE_WAIT'


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 node
# ─────────────────────────────────────────────────────────────────────────────

class LidarCenterRowFollower(Node):
    """
    Subscribes to /velodyne_points, detects the center crop row, and
    publishes steering commands + RViz markers.

    Start with autonomous_mode:=false to validate detection visually in
    RViz before enabling /cmd_vel output.
    """

    def __init__(self) -> None:
        super().__init__('lidar_center_row_follower')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('autonomous_mode',        False)
        self.declare_parameter('max_linear_speed',       0.30)   # m/s
        self.declare_parameter('max_angular_speed',      0.40)   # rad/s
        self.declare_parameter('k_heading',              1.20)   # P gain, heading error
        self.declare_parameter('k_lateral',              0.80)   # P gain, lateral offset
        self.declare_parameter('speed_heading_scale',    0.60)   # reduce speed when misaligned

        # Sensor geometry (matches amiga_min.urdf)
        self.declare_parameter('lidar_x_offset',         0.959)  # m fwd from base_link
        self.declare_parameter('lidar_z_offset',         0.80)   # m above ground (2026-07 re-mount)

        # Center crop ROI (base_link frame)
        self.declare_parameter('roi_forward_min',        0.30)   # m
        self.declare_parameter('roi_forward_max',        4.00)   # m
        self.declare_parameter('roi_lateral_half',       0.35)   # ±m from centreline

        # Crop height above ground (cardboard ~15 cm, onion ~30 cm)
        self.declare_parameter('crop_height_min',        0.05)   # m
        self.declare_parameter('crop_height_max',        0.45)   # m

        # Tire-path clearance zones (base_link frame, symmetric left/right)
        self.declare_parameter('tire_forward_min',       0.30)
        self.declare_parameter('tire_forward_max',       2.50)
        self.declare_parameter('tire_lateral_inner',     0.42)   # inner edge
        self.declare_parameter('tire_lateral_outer',     0.80)   # outer edge
        self.declare_parameter('tire_height_min',        0.05)
        self.declare_parameter('tire_height_max',        1.50)
        self.declare_parameter('tire_obstacle_thresh',   3)      # pts → obstacle

        # Forward safety zone (base_link frame)
        self.declare_parameter('safety_forward_min',     0.15)
        self.declare_parameter('safety_forward_max',     1.50)
        self.declare_parameter('safety_lateral_half',    0.55)
        self.declare_parameter('safety_height_min',      0.10)
        self.declare_parameter('safety_height_max',      2.00)
        self.declare_parameter('safety_obstacle_thresh', 3)
        self.declare_parameter('obstacle_clear_secs',    3.0)

        # Row detection thresholds
        self.declare_parameter('min_crop_points',        8)
        self.declare_parameter('good_crop_points',       30)
        self.declare_parameter('confidence_threshold',   0.35)
        self.declare_parameter('confidence_loss_frames', 6)      # → ROW_END
        # Minimum scans driven before ROW_END can trigger.
        # Proxy for distance at ~10 Hz / 0.30 m·s⁻¹: 1 scan ≈ 0.03 m
        # Default 100 scans ≈ 3 m.  Set to 0 to disable.
        self.declare_parameter('row_end_min_scans',      100)

        # ── Internal state ─────────────────────────────────────────────────
        self._state = _S.ACQUIRING
        self._smoother = _RowSmoother(alpha=0.35, history=6)
        self._low_conf_frames = 0
        self._following_scans = 0
        self._obstacle_clear_since: Optional[float] = None

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_state   = self.create_publisher(String,      '/row/state',          1)
        self._pub_conf    = self.create_publisher(Float64,     '/row/confidence',     1)
        self._pub_lat     = self.create_publisher(Float64,     '/row/lateral_offset', 1)
        self._pub_hdg     = self.create_publisher(Float64,     '/row/heading_error',  1)
        self._pub_markers = self.create_publisher(MarkerArray, '/row/markers',        1)
        self._pub_cmd     = self.create_publisher(Twist,       '/cmd_vel',            1)

        # ── Subscriber ─────────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/velodyne_points', self._cloud_cb, 10
        )

        self.get_logger().info(
            'LidarCenterRowFollower ready. '
            f'autonomous_mode={self._p("autonomous_mode")}  '
            'Set autonomous_mode:=true to enable /cmd_vel output.'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _vel_bounds(self, xyz: np.ndarray, x_bl_min, x_bl_max,
                    y_min, y_max, z_gnd_min, z_gnd_max) -> np.ndarray:
        """Filter to box; coords in base_link; convert to velodyne via offsets."""
        lx = self._p('lidar_x_offset')
        lz = self._p('lidar_z_offset')
        return _box(xyz,
                    x_bl_min - lx, x_bl_max - lx,
                    y_min, y_max,
                    z_gnd_min - lz, z_gnd_max - lz)

    def _crop_roi(self, xyz: np.ndarray) -> np.ndarray:
        yh = self._p('roi_lateral_half')
        return self._vel_bounds(
            xyz,
            self._p('roi_forward_min'),  self._p('roi_forward_max'),
            -yh, yh,
            self._p('crop_height_min'),  self._p('crop_height_max'),
        )

    def _tire_zone(self, xyz: np.ndarray, sign: float) -> np.ndarray:
        """sign=+1 → left tire zone (y>0), sign=-1 → right (y<0)."""
        yi = self._p('tire_lateral_inner')
        yo = self._p('tire_lateral_outer')
        return self._vel_bounds(
            xyz,
            self._p('tire_forward_min'), self._p('tire_forward_max'),
            sign * yo if sign < 0 else yi,
            sign * yi if sign < 0 else yo,
            self._p('tire_height_min'),  self._p('tire_height_max'),
        )

    def _safety_zone(self, xyz: np.ndarray) -> np.ndarray:
        yh = self._p('safety_lateral_half')
        return self._vel_bounds(
            xyz,
            self._p('safety_forward_min'), self._p('safety_forward_max'),
            -yh, yh,
            self._p('safety_height_min'),  self._p('safety_height_max'),
        )

    # ── Main callback ─────────────────────────────────────────────────────────

    def _cloud_cb(self, cloud: PointCloud2) -> None:
        xyz = _extract_xyz(cloud)
        if xyz is None:
            return

        # ── 1. Safety checks ─────────────────────────────────────────────
        n_safety   = len(self._safety_zone(xyz))
        n_tire_l   = len(self._tire_zone(xyz, +1.0))
        n_tire_r   = len(self._tire_zone(xyz, -1.0))
        blocked    = (n_safety >= self._p('safety_obstacle_thresh') or
                      n_tire_l >= self._p('tire_obstacle_thresh') or
                      n_tire_r >= self._p('tire_obstacle_thresh'))

        # ── 2. Center crop detection ──────────────────────────────────────
        crop_pts = self._crop_roi(xyz)
        n_crop   = len(crop_pts)
        min_pts  = self._p('min_crop_points')

        if n_crop >= min_pts:
            bev = crop_pts[:, :2]   # (x=forward, y=left)
            hdg, lat, elong = _pca_row(bev)
            inlier          = _ransac_inlier_ratio(bev)
            raw_conf = _row_confidence(
                n_crop, elong, inlier, min_pts, self._p('good_crop_points'))
            self._smoother.update(hdg, lat, raw_conf)
        else:
            raw_conf = 0.0
            self._smoother.update(0.0, 0.0, 0.0)

        conf       = self._smoother.smoothed_confidence
        conf_th    = self._p('confidence_threshold')
        loss_limit = self._p('confidence_loss_frames')
        end_scans  = self._p('row_end_min_scans')
        now        = time.monotonic()

        # ── 3. State machine ──────────────────────────────────────────────
        if blocked:
            if self._state != _S.OBSTACLE_WAIT:
                self._obstacle_clear_since = None
                self.get_logger().warn(
                    f'[{self._state}→OBSTACLE_WAIT] '
                    f'safety={n_safety} tire_L={n_tire_l} tire_R={n_tire_r}')
            self._state = _S.OBSTACLE_WAIT

        elif self._state == _S.OBSTACLE_WAIT:
            if self._obstacle_clear_since is None:
                self._obstacle_clear_since = now
            elif now - self._obstacle_clear_since >= self._p('obstacle_clear_secs'):
                self.get_logger().info('[OBSTACLE_WAIT→ACQUIRING] zone clear')
                self._state = _S.ACQUIRING
                self._obstacle_clear_since = None
                self._smoother.reset()

        elif self._state == _S.ACQUIRING:
            if conf >= conf_th:
                self.get_logger().info(
                    f'[ACQUIRING→FOLLOWING] conf={conf:.2f}  '
                    f'lat={self._smoother.lateral:+.3f} m  '
                    f'hdg={math.degrees(self._smoother.heading):+.1f}°')
                self._state = _S.FOLLOWING
                self._low_conf_frames = 0
                self._following_scans = 0

        elif self._state == _S.FOLLOWING:
            self._following_scans += 1
            if raw_conf < conf_th:
                self._low_conf_frames += 1
                if (self._low_conf_frames >= loss_limit and
                        self._following_scans >= end_scans):
                    self.get_logger().info(
                        f'[FOLLOWING→ROW_END] '
                        f'low_conf={self._low_conf_frames} frames  '
                        f'scans={self._following_scans}')
                    self._state = _S.ROW_END
            else:
                self._low_conf_frames = 0

        # ── 4. Velocity command ───────────────────────────────────────────
        twist = Twist()
        if (self._p('autonomous_mode') and
                self._state == _S.FOLLOWING and
                self._smoother.initialized):
            h = self._smoother.heading
            l = self._smoother.lateral
            # Slow down proportionally to heading misalignment (max slow at 45°)
            speed_factor = max(
                0.0, 1.0 - self._p('speed_heading_scale') * abs(h) / (math.pi / 4))
            v = self._p('max_linear_speed') * speed_factor
            omega = (self._p('k_heading') * h +
                     self._p('k_lateral') * l)
            max_w = self._p('max_angular_speed')
            twist.linear.x  = float(v)
            twist.angular.z = float(max(-max_w, min(max_w, omega)))

        self._pub_cmd.publish(twist)

        # ── 5. Diagnostics ────────────────────────────────────────────────
        self._pub_state.publish(String(data=self._state))
        self._pub_conf.publish(Float64(data=conf))
        self._pub_lat.publish(Float64(data=self._smoother.lateral))
        self._pub_hdg.publish(Float64(data=self._smoother.heading))
        self._publish_markers(cloud, crop_pts, xyz)

        self.get_logger().debug(
            f'[{self._state}] conf={conf:.2f} '
            f'lat={self._smoother.lateral:+.3f} m '
            f'hdg={math.degrees(self._smoother.heading):+.1f}° '
            f'n_crop={n_crop} n_safe={n_safety}')

    # ── RViz markers ──────────────────────────────────────────────────────────

    def _publish_markers(
        self,
        cloud: PointCloud2,
        crop_pts: np.ndarray,
        xyz: np.ndarray,
    ) -> None:
        frame = cloud.header.frame_id
        stamp = cloud.header.stamp
        lx    = self._p('lidar_x_offset')
        lz    = self._p('lidar_z_offset')
        ma    = MarkerArray()

        def _m(mid: int, mtype: int, ns: str) -> Marker:
            mk = Marker()
            mk.header.frame_id = frame
            mk.header.stamp    = stamp
            mk.ns   = ns
            mk.id   = mid
            mk.type = mtype
            mk.action = Marker.ADD
            mk.scale.x = mk.scale.y = mk.scale.z = 0.05
            mk.color.a = 1.0
            mk.pose.orientation.w = 1.0
            return mk

        # Crop-ROI points — green dots
        if len(crop_pts) > 0:
            m = _m(0, Marker.POINTS, 'crop_pts')
            m.scale.x = m.scale.y = 0.04
            m.color.g = 1.0
            for p in crop_pts[::max(1, len(crop_pts) // 200)]:
                pt = Point(); pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
                m.points.append(pt)
            ma.markers.append(m)

        # Row centreline — blue line
        if self._smoother.initialized and self._smoother.smoothed_confidence > 0.1:
            h   = self._smoother.heading
            lat = self._smoother.lateral
            ch, sh = math.cos(h), math.sin(h)
            ground_z = float(-lz + 0.12)   # 12 cm above ground in velodyne frame
            m = _m(1, Marker.LINE_STRIP, 'row_line')
            m.scale.x = 0.06
            m.color.b = 1.0; m.color.g = 0.3
            for t in [-2.5, 2.5]:
                pt = Point()
                pt.x = float(-lx + t * ch)
                pt.y = float(lat + t * sh)
                pt.z = ground_z
                m.points.append(pt)
            ma.markers.append(m)

        # Lookahead target — red sphere
        if self._smoother.initialized:
            h = self._smoother.heading
            m = _m(2, Marker.SPHERE, 'target_pt')
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.r = 1.0; m.color.g = 0.2
            lookahead = 1.5
            m.pose.position.x = float(-lx + lookahead * math.cos(h))
            m.pose.position.y = float(self._smoother.lateral + lookahead * math.sin(h))
            m.pose.position.z = float(-lz + 0.20)
            ma.markers.append(m)

        # State / confidence text — floats between red (low) and green (high)
        m = _m(3, Marker.TEXT_VIEW_FACING, 'conf_text')
        m.scale.z = 0.20
        m.pose.position.x = float(-lx + 1.0)
        m.pose.position.z = float(-lz + 1.0)
        c = self._smoother.smoothed_confidence
        m.color.r = float(1.0 - c); m.color.g = float(c)
        m.text = (
            f'{self._state}\n'
            f'conf={c:.2f}\n'
            f'lat={self._smoother.lateral:+.2f} m\n'
            f'hdg={math.degrees(self._smoother.heading):+.1f}°'
        )
        ma.markers.append(m)

        # Zone outlines — thin white lines (safety + crop ROI) for orientation
        self._add_zone_marker(ma, frame, stamp, lx, lz)

        self._pub_markers.publish(ma)

    def _add_zone_marker(self, ma, frame, stamp, lx, lz):
        """Add wireframe outlines for the crop ROI and safety zone."""
        def _rect(mid, ns, x_bl_min, x_bl_max, y_min, y_max, r, g, b):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp    = stamp
            m.ns = ns; m.id = mid; m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.02
            m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.6
            m.pose.orientation.w = 1.0
            z = float(-lz + 0.05)
            corners = [
                (x_bl_min - lx, y_min), (x_bl_max - lx, y_min),
                (x_bl_max - lx, y_max), (x_bl_min - lx, y_max),
                (x_bl_min - lx, y_min),   # close
            ]
            for cx, cy in corners:
                pt = Point(); pt.x = float(cx); pt.y = float(cy); pt.z = z
                m.points.append(pt)
            ma.markers.append(m)

        yh  = self._p('roi_lateral_half')
        _rect(10, 'roi_box',
              self._p('roi_forward_min'), self._p('roi_forward_max'),
              -yh, yh, 0.0, 1.0, 0.5)

        ysh = self._p('safety_lateral_half')
        _rect(11, 'safety_box',
              self._p('safety_forward_min'), self._p('safety_forward_max'),
              -ysh, ysh, 1.0, 0.4, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarCenterRowFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
