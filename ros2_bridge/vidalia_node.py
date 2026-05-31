#!/usr/bin/env python3
"""
vidalia_node.py — ROS2 Foxy bridge for the Vidalia autonomous navigation stack.

Reads scan data and navigation status written by row_follow.py to /dev/shm/,
then publishes standard ROS2 topics so RViz2 can visualise the live robot state
during field operation.

Topics published
----------------
/velodyne_points      sensor_msgs/PointCloud2         Full scan @ 10 Hz
/tf_static            (StaticTransformBroadcaster)    base_link → velodyne
/row_viz              visualization_msgs/MarkerArray   Row estimate overlay
                        id 10 : ARROW  — row direction arrow
                        id 11 : LINE   — lateral offset line (robot → row centre)
                        id 12 : LINE   — row centreline  ±7 m along row direction
                        id 13,14: LINE — EKF ±2σ uncertainty band
/safety_viz           visualization_msgs/MarkerArray   Safety zones + robot footprint
                        id 0–2  : safety zone rectangles (forward + L/R tire)
                        id 30   : robot footprint rectangle
/nav_status_viz       visualization_msgs/MarkerArray   Nav state indicators
                        id 20 : CYLINDER  — state-coloured ground disc
                        id 21 : TEXT      — multi-line nav state summary
/cmd_vel              geometry_msgs/Twist              Navigation commands

Shared-memory protocol (written by row_follow.py --ros2-bridge)
---------------------------------------------------------------
/dev/shm/vidalia_pts.bin    : 4-byte little-endian int32 N, then Nx3 float32
/dev/shm/vidalia_status.json: JSON nav status (see row_navigator.py)

Run inside the Docker container started by start.sh (host networking).
"""

from __future__ import annotations

import json
import math
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Image as RosImage, PointCloud2, PointField
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------
# Shared-memory paths (same as row_follow.py --ros2-bridge writes)
# ---------------------------------------------------------------------------
SHM_PTS       = "/dev/shm/vidalia_pts.bin"
SHM_LIDAR_PTS = "/dev/shm/vidalia_lidar_pts.bin"
SHM_CAM_PTS   = "/dev/shm/vidalia_cam_pts.bin"
SHM_STATUS    = "/dev/shm/vidalia_status.json"
SHM_IMG_LEFT  = "/dev/shm/vidalia_img_left.bin"
SHM_IMG_RIGHT = "/dev/shm/vidalia_img_right.bin"

# ---------------------------------------------------------------------------
# Geometry constants (must match CLAUDE.md)
# ---------------------------------------------------------------------------
LIDAR_MOUNT_HEIGHT = 0.699   # m — z offset base_link → velodyne
LIDAR_MOUNT_X      = 0.959   # m — forward offset base_link → velodyne
LIDAR_TILT_DEG     = 15.0    # nose-down tilt applied at mount time

# Safety zone geometry (matches row_safety.py defaults)
FWD_HALF_WIDTH = 0.95
FWD_DIST       = 2.5
TIRE_TRACK     = 0.915
TIRE_HALF_W    = 0.25
TIRE_DIST      = 2.5

# Amiga robot footprint in base_link frame (approximate)
ROBOT_FRONT =  0.85
ROBOT_REAR  = -0.85
ROBOT_LEFT  = -TIRE_TRACK
ROBOT_RIGHT =  TIRE_TRACK

# Height band thresholds for point-cloud colouring in RViz (ground-relative)
H_GROUND  = 0.0
H_CROP_HI = 0.60
H_OBS     = 0.75

# ---------------------------------------------------------------------------
# Colour lookup — one entry per navigator state
# ---------------------------------------------------------------------------
_STATE_COLOR: dict[str, tuple] = {
    "ACQUIRE":       (1.0, 0.80, 0.0,  0.9),   # amber
    "FOLLOW":        (0.1, 0.95, 0.1,  0.9),   # green
    "OBSTACLE_WAIT": (1.0, 0.15, 0.15, 0.9),   # red
    "ROW_END":       (0.2, 0.55, 1.0,  0.9),   # blue
    "DONE":          (0.7, 0.7,  0.7,  0.9),   # grey
}
_DEFAULT_COLOR = (0.7, 0.7, 0.7, 0.9)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ros_time(node: Node) -> RosTime:
    sec, nsec = divmod(int(node.get_clock().now().nanoseconds), 10 ** 9)
    t = RosTime()
    t.sec = sec
    t.nanosec = nsec
    return t


def _header(node: Node, frame: str) -> Header:
    h = Header()
    h.stamp = _ros_time(node)
    h.frame_id = frame
    return h


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _make_pc2(pts: np.ndarray, node: Node, intensity_override: float = -1.0) -> PointCloud2:
    """Convert Nx3 float64 (x,y,z) array to a PointCloud2 in base_link frame.

    intensity_override >= 0: sets every point to a fixed intensity value (used
    for the camera cloud so RViz can give it a flat colour distinct from LiDAR).
    intensity_override < 0 (default): height-relative colour (blue→red).
    """
    n = len(pts)
    h = (pts[:, 2] + LIDAR_MOUNT_HEIGHT).astype(np.float32)

    if intensity_override >= 0.0:
        h_norm = np.full(n, intensity_override, dtype=np.float32)
    else:
        h_norm = np.clip(h / H_OBS, 0.0, 1.0).astype(np.float32)

    xyz  = pts.astype(np.float32)
    data = np.hstack([xyz, h_norm[:, None]])   # Nx4 float32
    raw  = data.tobytes()

    fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg = PointCloud2()
    msg.header       = _header(node, "base_link")
    msg.height       = 1
    msg.width        = n
    msg.fields       = fields
    msg.is_bigendian = False
    msg.point_step   = 16
    msg.row_step     = 16 * n
    msg.data         = raw
    msg.is_dense     = True
    return msg


def _line_marker(
    ns: str, marker_id: int,
    pts: list, frame: str,
    r: float, g: float, b: float, a: float,
    width: float,
    lifetime_ns: int,
    node: Node,
) -> Marker:
    m = Marker()
    m.header           = _header(node, frame)
    m.ns               = ns
    m.id               = marker_id
    m.type             = Marker.LINE_STRIP
    m.action           = Marker.ADD
    m.scale.x          = width
    m.color            = ColorRGBA(r=r, g=g, b=b, a=a)
    m.lifetime.nanosec = lifetime_ns
    m.points           = pts
    return m


def _box_rect_pts(
    mid_x: float, mid_y: float,
    half_x: float, half_y: float,
    z: float,
) -> list:
    """5 RosPoints forming a closed rectangle (for LINE_STRIP)."""
    x0, x1 = mid_x - half_x, mid_x + half_x
    y0, y1 = mid_y - half_y, mid_y + half_y
    return [
        RosPoint(x=x0, y=y0, z=z),
        RosPoint(x=x1, y=y0, z=z),
        RosPoint(x=x1, y=y1, z=z),
        RosPoint(x=x0, y=y1, z=z),
        RosPoint(x=x0, y=y0, z=z),
    ]


def _read_shm_bgr(path: str):
    """Read a BGR image written by row_navigator._write_shm_bgr.
    Returns HxWx3 uint8 numpy array or None on any error.
    """
    try:
        with open(path, "rb") as f:
            raw = f.read()
    except OSError:
        return None
    if len(raw) < 8:
        return None
    h, w = struct.unpack_from("<ii", raw, 0)
    expected = 8 + h * w * 3
    if len(raw) < expected or h <= 0 or w <= 0:
        return None
    arr = np.frombuffer(raw, dtype=np.uint8, count=h * w * 3, offset=8)
    return arr.reshape(h, w, 3)


def _bgr_to_ros_image(bgr: np.ndarray, node: Node) -> RosImage:
    """Convert BGR HxWx3 uint8 numpy array to sensor_msgs/Image."""
    msg = RosImage()
    msg.header = _header(node, "base_link")
    msg.height = bgr.shape[0]
    msg.width  = bgr.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = bgr.shape[1] * 3
    msg.data = bgr.tobytes()
    return msg


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class VidaliaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vidalia_bridge")

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pc_pub       = self.create_publisher(PointCloud2, "/velodyne_points",   reliable_qos)
        self._lidar_pub    = self.create_publisher(PointCloud2, "/lidar_points",     reliable_qos)
        self._cam_pub      = self.create_publisher(PointCloud2, "/camera_points",    reliable_qos)
        self._row_pub      = self.create_publisher(MarkerArray, "/row_viz",          reliable_qos)
        self._safety_pub   = self.create_publisher(MarkerArray, "/safety_viz",       reliable_qos)
        self._nstatus_pub  = self.create_publisher(MarkerArray, "/nav_status_viz",   reliable_qos)
        self._vel_pub      = self.create_publisher(Twist,       "/cmd_vel",          reliable_qos)
        self._img_left_pub = self.create_publisher(RosImage,    "/camera_left/image",  reliable_qos)
        self._img_right_pub= self.create_publisher(RosImage,    "/camera_right/image", reliable_qos)

        self._tf_static  = StaticTransformBroadcaster(self)
        self._publish_tf_static()

        self._timer          = self.create_timer(1.0 / 12.0, self._poll_shm)
        self._last_pts_t     = 0.0
        self._last_lidar_t   = 0.0
        self._last_cam_t     = 0.0
        self._last_stat_t    = 0.0
        self._last_img_left_t  = 0.0
        self._last_img_right_t = 0.0

        self.get_logger().info(
            "Vidalia ROS2 bridge started — waiting for /dev/shm/ data …\n"
            "  LiDAR  : /lidar_points (LiDAR only)  /velodyne_points (fused)\n"
            "  Camera : /camera_points (depth 3D)  /camera_left/image  /camera_right/image\n"
            "  Nav    : /row_viz  /safety_viz  /nav_status_viz  /cmd_vel"
        )

    # ------------------------------------------------------------------
    def _publish_tf_static(self) -> None:
        ts = TransformStamped()
        ts.header.stamp    = _ros_time(self)
        ts.header.frame_id = "base_link"
        ts.child_frame_id  = "velodyne"
        ts.transform.translation.x = LIDAR_MOUNT_X
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = LIDAR_MOUNT_HEIGHT
        tilt_rad = math.radians(-LIDAR_TILT_DEG)
        ts.transform.rotation = _quat_from_rpy(0.0, tilt_rad, 0.0)
        self._tf_static.sendTransform(ts)

    # ------------------------------------------------------------------
    def _poll_shm(self) -> None:
        self._try_publish_pts()
        self._try_publish_lidar_pts()
        self._try_publish_cam_pts()
        self._try_publish_cam_images()
        self._try_publish_status()

    # ------------------------------------------------------------------
    def _try_publish_pts(self) -> None:
        try:
            mtime = os.path.getmtime(SHM_PTS)
        except OSError:
            return
        if mtime <= self._last_pts_t:
            return
        self._last_pts_t = mtime

        try:
            with open(SHM_PTS, "rb") as f:
                raw = f.read()
        except OSError:
            return

        if len(raw) < 4:
            return
        n = struct.unpack_from("<i", raw, 0)[0]
        expected = 4 + n * 3 * 4
        if len(raw) < expected or n <= 0:
            return

        pts = np.frombuffer(raw, dtype=np.float32, count=n * 3, offset=4).reshape(n, 3)
        self._pc_pub.publish(_make_pc2(pts.astype(np.float64), self))

    # ------------------------------------------------------------------
    def _try_publish_lidar_pts(self) -> None:
        try:
            mtime = os.path.getmtime(SHM_LIDAR_PTS)
        except OSError:
            return
        if mtime <= self._last_lidar_t:
            return
        self._last_lidar_t = mtime
        try:
            with open(SHM_LIDAR_PTS, "rb") as f:
                raw = f.read()
        except OSError:
            return
        if len(raw) < 4:
            return
        n = struct.unpack_from("<i", raw, 0)[0]
        expected = 4 + n * 3 * 4
        if len(raw) < expected or n <= 0:
            return
        pts = np.frombuffer(raw, dtype=np.float32, count=n * 3, offset=4).reshape(n, 3)
        self._lidar_pub.publish(_make_pc2(pts.astype(np.float64), self))

    # ------------------------------------------------------------------
    def _try_publish_cam_pts(self) -> None:
        try:
            mtime = os.path.getmtime(SHM_CAM_PTS)
        except OSError:
            return
        if mtime <= self._last_cam_t:
            return
        self._last_cam_t = mtime

        try:
            with open(SHM_CAM_PTS, "rb") as f:
                raw = f.read()
        except OSError:
            return

        if len(raw) < 4:
            return
        n = struct.unpack_from("<i", raw, 0)[0]
        if n <= 0:
            return
        expected = 4 + n * 3 * 4
        if len(raw) < expected:
            return

        pts = np.frombuffer(raw, dtype=np.float32, count=n * 3, offset=4).reshape(n, 3)
        # intensity_override=2.0 → saturated value above the normal 0–1 height range,
        # so RViz rainbow colormap renders these points as a visually distinct orange.
        self._cam_pub.publish(_make_pc2(pts.astype(np.float64), self, intensity_override=2.0))

    # ------------------------------------------------------------------
    def _try_publish_cam_images(self) -> None:
        for shm_path, pub, attr in (
            (SHM_IMG_LEFT,  self._img_left_pub,  "_last_img_left_t"),
            (SHM_IMG_RIGHT, self._img_right_pub, "_last_img_right_t"),
        ):
            try:
                mtime = os.path.getmtime(shm_path)
            except OSError:
                continue
            if mtime <= getattr(self, attr):
                continue
            setattr(self, attr, mtime)
            bgr = _read_shm_bgr(shm_path)
            if bgr is not None:
                pub.publish(_bgr_to_ros_image(bgr, self))

    # ------------------------------------------------------------------
    def _try_publish_status(self) -> None:
        try:
            mtime = os.path.getmtime(SHM_STATUS)
        except OSError:
            return
        if mtime <= self._last_stat_t:
            return
        self._last_stat_t = mtime

        try:
            with open(SHM_STATUS) as f:
                st = json.load(f)
        except (OSError, json.JSONDecodeError):
            return

        self._publish_safety_markers(st)
        self._publish_row_marker(st)
        self._publish_nav_status_markers(st)
        self._publish_cmd_vel(st)

    # ------------------------------------------------------------------
    def _publish_safety_markers(self, st: dict) -> None:
        markers  = MarkerArray()
        state    = st.get("state", "ACQUIRE")
        fwd_blk  = st.get("forward_blocked",    False)
        left_blk = st.get("left_tire_blocked",  False)
        rgt_blk  = st.get("right_tire_blocked", False)

        safe  = (0.2, 1.0, 0.2, 0.5)
        block = (1.0, 0.2, 0.2, 0.8)
        LT    = 250_000_000   # 0.25 s marker lifetime

        # Forward zone
        fc = block if fwd_blk else safe
        markers.markers.append(_line_marker(
            "safety", 0,
            _box_rect_pts(0.0, FWD_DIST / 2, FWD_HALF_WIDTH, FWD_DIST / 2, 0.85),
            "base_link", *fc, 0.04, LT, self,
        ))
        # Left tire zone
        lc = block if left_blk else safe
        markers.markers.append(_line_marker(
            "safety", 1,
            _box_rect_pts(-TIRE_TRACK, TIRE_DIST / 2, TIRE_HALF_W, TIRE_DIST / 2, 0.85),
            "base_link", *lc, 0.04, LT, self,
        ))
        # Right tire zone
        rc = block if rgt_blk else safe
        markers.markers.append(_line_marker(
            "safety", 2,
            _box_rect_pts(TIRE_TRACK, TIRE_DIST / 2, TIRE_HALF_W, TIRE_DIST / 2, 0.85),
            "base_link", *rc, 0.04, LT, self,
        ))

        # Robot footprint — colored by nav state
        sr, sg, sb, sa = _STATE_COLOR.get(state, _DEFAULT_COLOR)
        markers.markers.append(_line_marker(
            "robot", 30,
            _box_rect_pts(
                (ROBOT_LEFT + ROBOT_RIGHT) / 2,
                (ROBOT_FRONT + ROBOT_REAR) / 2,
                abs(ROBOT_RIGHT - ROBOT_LEFT) / 2,
                abs(ROBOT_FRONT - ROBOT_REAR) / 2,
                0.10,
            ),
            "base_link", sr, sg, sb, 0.85, 0.06, LT, self,
        ))

        self._safety_pub.publish(markers)

    # ------------------------------------------------------------------
    def _publish_row_marker(self, st: dict) -> None:
        offset   = st.get("lateral_offset", 0.0)
        heading  = st.get("heading_error",  0.0)
        conf     = st.get("confidence",     0.0)
        ekf_std  = st.get("ekf_std_lat",    0.0)

        if conf < 0.10:
            return

        sh, ch   = math.sin(heading), math.cos(heading)
        K_BACK   = -2.0
        K_FWD    =  7.0
        LT       = 350_000_000   # 0.35 s
        markers  = MarkerArray()

        # Direction arrow (short — 3 m ahead)
        arr = Marker()
        arr.header           = _header(self, "base_link")
        arr.ns               = "row"
        arr.id               = 10
        arr.type             = Marker.ARROW
        arr.action           = Marker.ADD
        arr.scale.x          = 0.06
        arr.scale.y          = 0.14
        arr.scale.z          = 0.0
        arr.color            = ColorRGBA(r=0.0, g=0.6, b=1.0, a=min(1.0, conf + 0.3))
        arr.lifetime.nanosec = LT
        arr.points = [
            RosPoint(x=0.0, y=0.0, z=0.35),
            RosPoint(x=sh * 3.0 + offset, y=ch * 3.0, z=0.35),
        ]
        markers.markers.append(arr)

        # Lateral offset line (robot origin → row centre at Y=0)
        markers.markers.append(_line_marker(
            "row", 11,
            [RosPoint(x=0.0, y=0.0, z=0.30), RosPoint(x=offset, y=0.0, z=0.30)],
            "base_link", 1.0, 0.85, 0.0, 0.9, 0.05, LT, self,
        ))

        # Row centreline: extends K_BACK..K_FWD metres along detected row
        markers.markers.append(_line_marker(
            "row", 12,
            [
                RosPoint(x=offset + K_BACK * sh, y=K_BACK * ch, z=0.18),
                RosPoint(x=offset + K_FWD  * sh, y=K_FWD  * ch, z=0.18),
            ],
            "base_link", 0.2, 1.0, 0.2, 0.75, 0.035, LT, self,
        ))

        # EKF ±2σ uncertainty band (only when EKF is active and std is meaningful)
        if ekf_std > 0.003:
            for sign, mid in [(+1, 13), (-1, 14)]:
                band_off = offset + sign * 2.0 * ekf_std
                markers.markers.append(_line_marker(
                    "row", mid,
                    [
                        RosPoint(x=band_off + K_BACK * sh, y=K_BACK * ch, z=0.18),
                        RosPoint(x=band_off + K_FWD  * sh, y=K_FWD  * ch, z=0.18),
                    ],
                    "base_link", 0.2, 1.0, 0.2, 0.30, 0.015, LT, self,
                ))

        self._row_pub.publish(markers)

    # ------------------------------------------------------------------
    def _publish_nav_status_markers(self, st: dict) -> None:
        """State-coloured ground disc + multi-line text overlay above the robot."""
        state    = st.get("state",           "ACQUIRE")
        conf     = st.get("confidence",      0.0)
        off      = st.get("lateral_offset",  0.0)
        hdg      = math.degrees(st.get("heading_error", 0.0))
        n        = st.get("n_points",        0)
        d        = st.get("row_dist",        0.0)
        rows_done = st.get("rows_done",      0)
        rows_tot  = st.get("rows_total",     1)
        ekf_std  = st.get("ekf_std_lat",     0.0)
        ekf_conv = st.get("ekf_converged",   False)
        acq_n    = st.get("acq_count",       0)
        acq_f    = st.get("acquire_frames",  5)
        fwd_blk  = st.get("forward_blocked", False)
        cam_blk  = st.get("cam_blocked",     False)
        nearest  = st.get("nearest_forward", 99.0)

        r, g, b, a = _STATE_COLOR.get(state, _DEFAULT_COLOR)
        LT = 400_000_000   # 0.4 s

        markers = MarkerArray()

        # Ground disc (shows state colour at a glance)
        disc = Marker()
        disc.header           = _header(self, "base_link")
        disc.ns               = "nav_state"
        disc.id               = 20
        disc.type             = Marker.CYLINDER
        disc.action           = Marker.ADD
        disc.pose.position.z  = 0.02
        disc.pose.orientation.w = 1.0
        disc.scale.x          = 0.60
        disc.scale.y          = 0.60
        disc.scale.z          = 0.04
        disc.color            = ColorRGBA(r=r, g=g, b=b, a=a)
        disc.lifetime.nanosec = LT
        markers.markers.append(disc)

        # Text overlay — displayed 1.8 m above and 1.5 m behind robot
        # so it doesn't obscure the forward view
        if ekf_std > 0.001:
            ekf_line = f"σ={ekf_std:.3f}m" + ("  CONV" if ekf_conv else f"  acq={acq_n}/{acq_f}")
        else:
            ekf_line = f"acq={acq_n}/{acq_f}"

        obs_line = ""
        if fwd_blk:
            obs_line = f"\nFWD-BLOCKED  {nearest:.1f}m"
        elif cam_blk:
            obs_line = "\nCAM-BLOCKED"

        text_content = (
            f"{state}\n"
            f"conf={conf:.2f}  off={off:+.2f}m\n"
            f"hdg={hdg:+.1f}°  n={n}\n"
            f"{ekf_line}\n"
            f"d={d:.1f}m  rows={rows_done}/{rows_tot}"
            f"{obs_line}"
        )

        txt = Marker()
        txt.header              = _header(self, "base_link")
        txt.ns                  = "nav_state"
        txt.id                  = 21
        txt.type                = Marker.TEXT_VIEW_FACING
        txt.action              = Marker.ADD
        txt.pose.position.x     = 0.0
        txt.pose.position.y     = -1.8   # behind robot
        txt.pose.position.z     = 1.8
        txt.pose.orientation.w  = 1.0
        txt.scale.z             = 0.20   # text height in metres
        txt.color               = ColorRGBA(r=r, g=g, b=b, a=1.0)
        txt.text                = text_content
        txt.lifetime.nanosec    = LT
        markers.markers.append(txt)

        self._nstatus_pub.publish(markers)

    # ------------------------------------------------------------------
    def _publish_cmd_vel(self, st: dict) -> None:
        t = Twist()
        t.linear.x  = float(st.get("linear_vel",  0.0))
        t.angular.z = float(st.get("angular_vel", 0.0))
        self._vel_pub.publish(t)


# ---------------------------------------------------------------------------
def main() -> None:
    rclpy.init()
    node = VidaliaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
