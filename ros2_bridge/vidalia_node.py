#!/usr/bin/env python3
"""
vidalia_node.py — ROS2 Foxy bridge for the Vidalia autonomous navigation stack.

Reads scan data and navigation status written by row_follow.py to /dev/shm/,
then publishes standard ROS2 topics so RViz2 on a laptop can visualise the
live robot state during field operation.

Topics published
----------------
/velodyne_points      sensor_msgs/PointCloud2       Full scan @ 10 Hz
/tf_static            (StaticTransformBroadcaster)  base_link → velodyne
/row_viz              visualization_msgs/MarkerArray  Row estimate overlay
/safety_viz           visualization_msgs/MarkerArray  Safety zone outlines
/cmd_vel              geometry_msgs/Twist             Navigation commands

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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, ColorRGBA
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# ---------------------------------------------------------------------------
# Shared-memory paths (same as row_follow.py --ros2-bridge writes)
# ---------------------------------------------------------------------------
SHM_PTS    = "/dev/shm/vidalia_pts.bin"
SHM_STATUS = "/dev/shm/vidalia_status.json"

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

# Height band thresholds for point-cloud colouring in RViz (ground-relative)
H_GROUND  = 0.0
H_CROP_LO = 0.05
H_CROP_HI = 0.60
H_OBS     = 0.75


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
    """Convert roll/pitch/yaw (rad) to a geometry_msgs Quaternion."""
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _make_pc2(pts: np.ndarray, node: Node) -> PointCloud2:
    """Convert Nx3 float64 (x,y,z) array to a PointCloud2 in velodyne frame."""
    n = len(pts)
    # Add height-relative intensity for colour-by-height in RViz
    h = (pts[:, 2] + LIDAR_MOUNT_HEIGHT).astype(np.float32)
    h_norm = np.clip((h - H_GROUND) / (H_OBS - H_GROUND), 0.0, 1.0).astype(np.float32)

    xyz = pts.astype(np.float32)
    data = np.hstack([xyz, h_norm[:, None]])        # Nx4 float32
    raw  = data.tobytes()

    fields = [
        PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg = PointCloud2()
    msg.header        = _header(node, "velodyne")
    msg.height        = 1
    msg.width         = n
    msg.fields        = fields
    msg.is_bigendian  = False
    msg.point_step    = 16
    msg.row_step      = 16 * n
    msg.data          = raw
    msg.is_dense      = True
    return msg


def _make_box_marker(
    mid_x: float, mid_y: float, half_x: float, half_y: float,
    h_lo: float, h_hi: float,
    r: float, g: float, b: float, a: float,
    marker_id: int,
    node: Node,
) -> Marker:
    """Return a LINE_STRIP marker outlining a rectangular zone in base_link frame."""
    m = Marker()
    m.header      = _header(node, "base_link")
    m.ns          = "safety"
    m.id          = marker_id
    m.type        = Marker.LINE_STRIP
    m.action      = Marker.ADD
    m.scale.x     = 0.03
    m.color       = ColorRGBA(r=r, g=g, b=b, a=a)
    m.lifetime.sec = 0
    m.lifetime.nanosec = 200_000_000   # 0.2 s

    # Draw a box at mid height
    mid_z = (h_lo + h_hi) / 2.0
    corners = [
        (mid_x - half_x, mid_y - half_y, mid_z),
        (mid_x + half_x, mid_y - half_y, mid_z),
        (mid_x + half_x, mid_y + half_y, mid_z),
        (mid_x - half_x, mid_y + half_y, mid_z),
        (mid_x - half_x, mid_y - half_y, mid_z),  # close
    ]
    from geometry_msgs.msg import Point as RosPoint
    m.points = [RosPoint(x=cx, y=cy, z=cz) for cx, cy, cz in corners]
    return m


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class VidaliaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vidalia_bridge")

        # QoS: reliable for latched topics, best-effort for high-frequency data
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pc_pub     = self.create_publisher(PointCloud2, "/velodyne_points", best_effort_qos)
        self._row_pub    = self.create_publisher(MarkerArray, "/row_viz",          10)
        self._safety_pub = self.create_publisher(MarkerArray, "/safety_viz",       10)
        self._vel_pub    = self.create_publisher(Twist,       "/cmd_vel",          10)

        self._tf_static  = StaticTransformBroadcaster(self)
        self._publish_tf_static()

        # Poll /dev/shm/ at 12 Hz — slightly faster than LiDAR 10 Hz so no scan is missed
        self._timer      = self.create_timer(1.0 / 12.0, self._poll_shm)
        self._last_pts_t = 0.0
        self._last_stat_t = 0.0

        self.get_logger().info(
            "Vidalia ROS2 bridge started — waiting for data in /dev/shm/ …\n"
            f"  pts:    {SHM_PTS}\n"
            f"  status: {SHM_STATUS}\n"
            "  Topics: /velodyne_points /row_viz /safety_viz /cmd_vel /tf_static"
        )

    # ------------------------------------------------------------------
    def _publish_tf_static(self) -> None:
        """Broadcast static transform: base_link → velodyne (15° tilt, 0.959 m fwd, 0.699 m up)."""
        ts = TransformStamped()
        ts.header.stamp    = _ros_time(self)
        ts.header.frame_id = "base_link"
        ts.child_frame_id  = "velodyne"

        ts.transform.translation.x = LIDAR_MOUNT_X
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = LIDAR_MOUNT_HEIGHT

        # 15° nose-down pitch (rotation around Y in ROS frame = around X in sensor frame)
        tilt_rad = math.radians(-LIDAR_TILT_DEG)   # negative = nose down
        ts.transform.rotation = _quat_from_rpy(0.0, tilt_rad, 0.0)

        self._tf_static.sendTransform(ts)
        self.get_logger().info(
            f"TF: base_link → velodyne  x={LIDAR_MOUNT_X}m  z={LIDAR_MOUNT_HEIGHT}m  "
            f"pitch={-LIDAR_TILT_DEG}°"
        )

    # ------------------------------------------------------------------
    def _poll_shm(self) -> None:
        self._try_publish_pts()
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
        self._publish_cmd_vel(st)

    # ------------------------------------------------------------------
    def _publish_safety_markers(self, st: dict) -> None:
        markers = MarkerArray()

        fwd_blocked   = st.get("forward_blocked", False)
        left_blocked  = st.get("left_tire_blocked", False)
        right_blocked = st.get("right_tire_blocked", False)
        cam_blocked   = st.get("cam_blocked", False)

        any_blocked = fwd_blocked or left_blocked or right_blocked or cam_blocked
        safe_col  = (0.2, 1.0, 0.2, 0.5)   # green
        block_col = (1.0, 0.2, 0.2, 0.8)   # red

        # Forward zone: centred on robot
        fc = block_col if fwd_blocked else safe_col
        markers.markers.append(
            _make_box_marker(
                0.0, FWD_DIST / 2, FWD_HALF_WIDTH, FWD_DIST / 2,
                0.5, 1.2, *fc, 0, self
            )
        )
        # Left tire zone
        lc = block_col if left_blocked else safe_col
        markers.markers.append(
            _make_box_marker(
                -(TIRE_TRACK), TIRE_DIST / 2, TIRE_HALF_W, TIRE_DIST / 2,
                0.3, 1.0, *lc, 1, self
            )
        )
        # Right tire zone
        rc = block_col if right_blocked else safe_col
        markers.markers.append(
            _make_box_marker(
                TIRE_TRACK, TIRE_DIST / 2, TIRE_HALF_W, TIRE_DIST / 2,
                0.3, 1.0, *rc, 2, self
            )
        )
        self._safety_pub.publish(markers)

    # ------------------------------------------------------------------
    def _publish_row_marker(self, st: dict) -> None:
        offset  = st.get("lateral_offset", 0.0)
        heading = st.get("heading_error", 0.0)
        conf    = st.get("confidence", 0.0)

        if conf < 0.10:
            return

        markers = MarkerArray()

        # Arrow showing the detected row direction
        m = Marker()
        m.header    = _header(self, "base_link")
        m.ns        = "row"
        m.id        = 10
        m.type      = Marker.ARROW
        m.action    = Marker.ADD
        m.scale.x   = 0.06
        m.scale.y   = 0.12
        m.scale.z   = 0.0
        m.color     = ColorRGBA(r=0.0, g=0.5, b=1.0, a=min(1.0, conf + 0.3))
        m.lifetime.nanosec = 300_000_000

        from geometry_msgs.msg import Point as RosPoint
        # Arrow from robot origin → 3 m ahead along detected row direction
        m.points = [
            RosPoint(x=0.0, y=0.0, z=0.3),
            RosPoint(x=math.sin(heading) * 3.0 + offset,
                     y=math.cos(heading) * 3.0,
                     z=0.3),
        ]
        markers.markers.append(m)

        # Lateral offset line at crop height
        m2 = Marker()
        m2.header   = _header(self, "base_link")
        m2.ns       = "row"
        m2.id       = 11
        m2.type     = Marker.LINE_STRIP
        m2.action   = Marker.ADD
        m2.scale.x  = 0.04
        m2.color    = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
        m2.lifetime.nanosec = 300_000_000
        m2.points   = [
            RosPoint(x=0.0,    y=0.0, z=0.3),
            RosPoint(x=offset, y=0.0, z=0.3),
        ]
        markers.markers.append(m2)

        self._row_pub.publish(markers)

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
