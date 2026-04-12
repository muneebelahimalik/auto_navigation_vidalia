"""
autonomous_row_coverage.py — Autonomous boustrophedon row coverage with live SLAM,
LiDAR-based row-end detection, and obstacle stop-and-wait safety.

Place the robot at the START of the first onion row, facing along the row.
Call ~/start.  The robot drives itself.

  Row end detection (LiDAR)
  ─────────────────────────
  The VLP-16 is sampled every scan for a "crop density" zone — a wide box
  ahead of the robot at crop height.  Inside the row, plant returns fill
  this zone.  At the headland (end of row), returns drop sharply.
  When the median density over the last N scans falls below a threshold
  the row end is declared, the robot halts, and the end-of-row maneuver
  begins.

  Obstacle stop-and-wait (LiDAR)
  ──────────────────────────────
  A tight safety box directly ahead of the robot (within robot width, up to
  obstacle_stop_distance metres forward) is monitored continuously.
  Any points in that zone cancel the active Nav2 goal and publish zero
  velocity.  When the zone is clear for obstacle_clear_secs, the remaining
  waypoints are re-issued to Nav2.

  End-of-row maneuver (from farm-ng end_of_row_maneuver.py pattern)
  ─────────────────────────────────────────────────────────────────
    ════ row i ═══════════════► ──► buf
    row i                           ↓ row_spacing (lateral left of heading)
    ◄═══ row i+1 ═════════════ ◄── buf

  Three waypoints per maneuver:
    1. buffer_distance past the row end  (clear crop edge)
    2. row_spacing across, heading reversed  (now aligned with next row)
    3. buffer_distance back toward the field  (at next row start edge)

ROS 2 interface
───────────────
Services:
  ~/start          Trigger — begin coverage from current pose
  ~/stop           Trigger — cancel immediately
  ~/mark_row_end   Trigger — manually mark current position as row end (LEARNING mode only)

Topics published:
  ~/coverage_path  nav_msgs/Path  — full planned route (updates after learning)
  ~/current_row    std_msgs/Int32 — index of row being driven

Parameters
──────────
  row_length            float  Expected row length (m). 0 = auto-detect via LiDAR.
  num_rows              int    Number of rows to cover. Default 20.
  row_spacing           float  Distance between row centres (m). Default 0.45.
  buffer_distance       float  Distance past crop to drive before turning (m). Default 1.5.

  # LiDAR row-end detection  (only used when row_length == 0)
  row_end_threshold     int    Points/scan below which end-of-row is declared. Default 5.
  row_end_confirm_scans int    Consecutive low-density scans to confirm row end. Default 4.
  row_end_min_dist      float  Min metres driven before row-end detector activates. Default 3.0.

  # LiDAR obstacle safety  (always active during navigation)
  obstacle_stop_dist    float  Depth of forward safety zone (m). Default 1.5.
  obstacle_clear_secs   float  Seconds zone must be clear before resuming. Default 3.0.
  obstacle_threshold    int    Points in safety zone counted as obstacle. Default 3.

  nav_frame             str    TF frame for waypoints. Default: map.
  base_frame            str    Robot base frame. Default: base_link.

  # Physical calibration — must match amiga_min.urdf
  lidar_x_offset        float  Velodyne X offset from base_link (m). Default 1.13.
  lidar_z_offset        float  Velodyne Z offset from base_link (m). Default 0.80.

Usage
─────
  # Known row length:
  ros2 launch vidalia_bringup autonomous_coverage.launch.py \\
      row_length:=45.0 num_rows:=22 row_spacing:=0.45

  # Auto-detect row length (robot stops itself at end of row):
  ros2 launch vidalia_bringup autonomous_coverage.launch.py \\
      row_length:=0.0 num_rows:=22

  ros2 service call /autonomous_row_coverage/start std_srvs/srv/Trigger {}
  ros2 service call /autonomous_row_coverage/stop  std_srvs/srv/Trigger {}
"""

import math
from collections import deque

import numpy as np
import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException


# ──────────────────────────────────────────────────────────────────────────────
# LiDAR zone counting  (fast numpy — no ROS dependency beyond sensor_msgs)
# ──────────────────────────────────────────────────────────────────────────────

def _count_in_box(cloud_msg: PointCloud2,
                  x_min: float, x_max: float,
                  y_min: float, y_max: float,
                  z_min: float, z_max: float,
                  lidar_x: float, lidar_z: float) -> int:
    """
    Count PointCloud2 points that fall inside an axis-aligned box.

    The cloud is in the velodyne frame.  The box is specified in the
    base_link frame.  The transform velodyne→base_link is a pure translation:
        base_x = vel_x + lidar_x
        base_y = vel_y
        base_z = vel_z + lidar_z
    so we shift the box bounds into velodyne frame instead.

    Uses numpy for speed (~1 ms for a full VLP-16 scan of ~28 k points).
    NaN points (invalid returns) naturally fail all comparisons and are ignored.
    """
    field_map = {f.name: f.offset for f in cloud_msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return 0

    step = cloud_msg.point_step
    raw = np.frombuffer(bytes(cloud_msg.data), dtype=np.uint8)
    n = raw.shape[0] // step
    if n == 0:
        return 0

    pts = raw[:n * step].reshape(n, step)

    # Shift box bounds to velodyne frame
    xn, xx = x_min - lidar_x, x_max - lidar_x
    yn, yx = y_min,           y_max
    zn, zx = z_min - lidar_z, z_max - lidar_z

    # Extract float32 coordinates using the field offsets from the message
    ox = field_map['x']
    oy = field_map['y']
    oz = field_map['z']

    x = pts[:, ox:ox + 4].copy().view(np.float32).ravel()
    y = pts[:, oy:oy + 4].copy().view(np.float32).ravel()
    z = pts[:, oz:oz + 4].copy().view(np.float32).ravel()

    mask = (x >= xn) & (x <= xx) & (y >= yn) & (y <= yx) & (z >= zn) & (z <= zx)
    return int(np.sum(mask))


# ──────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ──────────────────────────────────────────────────────────────────────────────

def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _make_pose(x: float, y: float, yaw: float, frame: str, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.header.stamp = stamp
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    ps.pose.orientation = _quat_from_yaw(yaw)
    return ps


# ──────────────────────────────────────────────────────────────────────────────
# Boustrophedon waypoint generation
# ──────────────────────────────────────────────────────────────────────────────

def _end_of_row_maneuver(x, y, heading, row_spacing, buf, lat_x0, lat_y0, frame, stamp):
    """
    Three waypoints that move the robot from the end of one row to the start
    of the next, stacking rows to the LEFT of the INITIAL heading (yaw0).

        [row end] ──► buf ──► cross row_spacing ──► buf back = [next row start]

    lat_x0, lat_y0: lateral unit vector computed from yaw0 (NOT from heading).
    This is critical for odd rows where heading = yaw0+π — computing lat from
    heading would point AWAY from the next row instead of toward it.

    The incoming heading reverses after the maneuver (boustrophedon).
    """
    fwd_x = math.cos(heading)
    fwd_y = math.sin(heading)

    # 1. Drive past the crop edge
    bx = x + buf * fwd_x
    by = y + buf * fwd_y

    # 2. Cross to the next row using the fixed lateral direction from yaw0
    cx = bx + row_spacing * lat_x0
    cy = by + row_spacing * lat_y0
    next_heading = heading + math.pi

    # 3. Drive back to the field edge in the new (reversed) direction
    nx = cx + buf * math.cos(next_heading)   # = cx - buf * fwd_x
    ny = cy + buf * math.sin(next_heading)

    return [
        _make_pose(bx, by, heading,      frame, stamp),
        _make_pose(cx, cy, next_heading, frame, stamp),
        _make_pose(nx, ny, next_heading, frame, stamp),
    ]


def generate_coverage_waypoints(x0, y0, yaw0,
                                 row_length, row_spacing, num_rows,
                                 buf, frame, clock):
    """
    Full boustrophedon waypoint list starting from (x0, y0, yaw0).

    The caller is already AT the row-0 start; the first waypoint is therefore
    the END of row 0.  Nav2's goal tolerance (30 cm) means that if the robot
    is already at/near the first waypoint, it skips ahead automatically.
    """
    now = clock.now().to_msg()
    waypoints = []

    fwd_x = math.cos(yaw0)
    fwd_y = math.sin(yaw0)
    lat_x = -math.sin(yaw0)   # lateral-left direction (rows stack here)
    lat_y =  math.cos(yaw0)

    for i in range(num_rows):
        # Row i origin (lateral offset from row 0 origin)
        ox = x0 + i * row_spacing * lat_x
        oy = y0 + i * row_spacing * lat_y

        if i % 2 == 0:
            # Even rows: drive in yaw0 direction (forward)
            heading = yaw0
            end_x = ox + row_length * fwd_x
            end_y = oy + row_length * fwd_y
        else:
            # Odd rows: start at far end, drive in reverse
            heading = yaw0 + math.pi
            ox += row_length * fwd_x   # shift origin to far end
            oy += row_length * fwd_y
            end_x = x0 + i * row_spacing * lat_x   # back to near end
            end_y = y0 + i * row_spacing * lat_y

        waypoints.append(_make_pose(end_x, end_y, heading, frame, now))

        if i < num_rows - 1:
            waypoints.extend(
                _end_of_row_maneuver(
                    end_x, end_y, heading, row_spacing, buf,
                    lat_x, lat_y,   # fixed lateral direction from yaw0
                    frame, now,
                )
            )

    return waypoints


# ──────────────────────────────────────────────────────────────────────────────
# State constants
# ──────────────────────────────────────────────────────────────────────────────

class _State:
    IDLE           = 'IDLE'
    LEARNING       = 'LEARNING'        # driving row 0, detecting end via LiDAR
    NAVIGATING     = 'NAVIGATING'      # FollowWaypoints active
    OBSTACLE_WAIT  = 'OBSTACLE_WAIT'   # paused for safety — waiting for clear
    DONE           = 'DONE'


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────

class AutonomousRowCoverageNode(Node):

    def __init__(self):
        super().__init__('autonomous_row_coverage')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('row_length',            0.0)
        self.declare_parameter('num_rows',              20)
        self.declare_parameter('row_spacing',           0.45)
        self.declare_parameter('buffer_distance',       1.5)
        # row-end detector
        self.declare_parameter('row_end_threshold',     5)
        self.declare_parameter('row_end_confirm_scans', 4)
        self.declare_parameter('row_end_min_dist',      3.0)
        # obstacle safety
        self.declare_parameter('obstacle_stop_dist',    1.5)
        self.declare_parameter('obstacle_clear_secs',   3.0)
        self.declare_parameter('obstacle_threshold',    3)
        # frames / calibration
        self.declare_parameter('nav_frame',             'map')
        self.declare_parameter('base_frame',            'base_link')
        self.declare_parameter('lidar_x_offset',        1.13)
        self.declare_parameter('lidar_z_offset',        0.80)

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Nav2 action clients ──────────────────────────────────────────────
        self._wp_client  = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._nav_client = ActionClient(self, NavigateToPose,  '/navigate_to_pose')

        # ── Publishers ───────────────────────────────────────────────────────
        self._path_pub = self.create_publisher(Path,   '~/coverage_path', 10)
        self._row_pub  = self.create_publisher(Int32,  '~/current_row',   10)
        self._vel_pub  = self.create_publisher(Twist,  '/cmd_vel',        10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, '~/start',        self._start_cb)
        self.create_service(Trigger, '~/stop',         self._stop_cb)
        self.create_service(Trigger, '~/mark_row_end', self._mark_row_end_cb)

        # ── LiDAR subscription ───────────────────────────────────────────────
        self.create_subscription(
            PointCloud2, '/velodyne_points', self._cloud_cb, 5
        )

        # ── Tick timer (10 Hz) ────────────────────────────────────────────────
        self.create_timer(0.10, self._tick)

        # ── Internal state ────────────────────────────────────────────────────
        self._state             = _State.IDLE
        self._goal_handle       = None    # FollowWaypoints goal
        self._nav_goal_handle   = None    # NavigateToPose goal (LEARNING only)

        self._learn_start       = None    # (x, y, yaw) recorded at ~/start
        self._all_waypoints     = []      # full waypoint list for current run
        self._wp_goal_offset    = 0       # _all_waypoints idx == index 0 in current goal
        self._last_wp_fb_idx    = 0       # latest feedback index from current goal

        # LiDAR measurement buffers (updated in cloud callback, read in tick)
        self._row_density_buf   = deque(maxlen=10)   # recent point counts in crop zone
        self._obstacle_present  = False              # true if safety zone is occupied
        self._obstacle_clr_secs = 0.0               # seconds the safety zone has been clear

        self.get_logger().info(
            f'\nAutonomousRowCoverage ready.'
            f'\n  row_length   = {self.get_parameter("row_length").value:.1f} m  (0 = auto-detect via LiDAR)'
        )

    # ──────────────────────────────────────────────────────────────────────────
    # TF helper
    # ──────────────────────────────────────────────────────────────────────────

    def _get_pose(self):
        """Return (x, y, yaw) in nav_frame, or None on failure."""
        nav   = self.get_parameter('nav_frame').value
        base  = self.get_parameter('base_frame').value
        try:
            tf = self.tf_buffer.lookup_transform(nav, base, rclpy.time.Time())
            q   = tf.transform.rotation
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
                2.0 * math.atan2(q.z, q.w),
            )
        except TransformException as e:
            self.get_logger().error(f'TF {nav}→{base}: {e}', throttle_duration_sec=5.0)
            return None

    # ──────────────────────────────────────────────────────────────────────────
    # LiDAR callback — runs in the subscription thread
    # ──────────────────────────────────────────────────────────────────────────

    def _cloud_cb(self, cloud: PointCloud2):
        lx = self.get_parameter('lidar_x_offset').value
        lz = self.get_parameter('lidar_z_offset').value

        # ── Safety zone: anything directly in the robot's path ────────────────
        # Box in base_link frame: 0.15–stop_dist ahead, ±half robot width, 0.10–2.0 m tall
        stop_dist = self.get_parameter('obstacle_stop_dist').value
        obs_count = _count_in_box(
            cloud,
            x_min=0.15,       x_max=stop_dist,
            y_min=-0.55,      y_max=0.55,    # slightly wider than robot half-width (0.465 m)
            z_min=0.10,       z_max=2.00,
            lidar_x=lx, lidar_z=lz,
        )
        self._obstacle_present = obs_count >= self.get_parameter('obstacle_threshold').value

        # ── Crop density zone: forward-wide at crop height ─────────────────────
        # Used to detect end of row: inside row = many returns, headland = very few
        density = _count_in_box(
            cloud,
            x_min=1.00,  x_max=4.00,   # 1–4 m ahead (avoids immediate-proximity noise)
            y_min=-2.00, y_max=2.00,    # ±2 m catches crops on both sides of the robot
            z_min=0.00,  z_max=1.50,    # ground to above canopy height
            lidar_x=lx, lidar_z=lz,
        )
        self._row_density_buf.append(density)

    # ──────────────────────────────────────────────────────────────────────────
    # Tick timer — 10 Hz state-machine driver
    # ──────────────────────────────────────────────────────────────────────────

    def _tick(self):
        state = self._state

        # ── Obstacle handling (active in NAVIGATING and LEARNING) ─────────────
        if state in (_State.NAVIGATING, _State.LEARNING):
            if self._obstacle_present:
                self._on_obstacle_detected()
                return

        if state == _State.OBSTACLE_WAIT:
            self._tick_obstacle_wait()
            return

        # ── Row-end detection (LEARNING only) ─────────────────────────────────
        if state == _State.LEARNING:
            self._tick_learning()

    def _tick_learning(self):
        """Check LiDAR density to auto-detect row end."""
        min_dist = self.get_parameter('row_end_min_dist').value
        threshold = self.get_parameter('row_end_threshold').value
        confirm   = self.get_parameter('row_end_confirm_scans').value

        # Need enough history
        if len(self._row_density_buf) < confirm:
            return

        # Check minimum distance driven
        pose = self._get_pose()
        if pose is None or self._learn_start is None:
            return
        dist = math.hypot(pose[0] - self._learn_start[0],
                          pose[1] - self._learn_start[1])
        if dist < min_dist:
            return

        # Confirm row end: last N scans all below threshold
        recent = list(self._row_density_buf)[-confirm:]
        if all(d <= threshold for d in recent):
            self.get_logger().info(
                f'Row end detected at distance {dist:.2f} m '
                f'(density: {recent})'
            )
            self._on_row_end_detected(pose, dist)

    def _tick_obstacle_wait(self):
        """Wait for the safety zone to be clear, then resume."""
        if self._obstacle_present:
            self._obstacle_clr_secs = 0.0
            # Keep publishing zero velocity so the robot stays still
            self._vel_pub.publish(Twist())
            return

        self._obstacle_clr_secs += 0.10   # timer period
        wait_needed = self.get_parameter('obstacle_clear_secs').value

        self.get_logger().info(
            f'Obstacle cleared — resuming in {wait_needed - self._obstacle_clr_secs:.1f} s',
            throttle_duration_sec=1.0,
        )

        if self._obstacle_clr_secs >= wait_needed:
            self.get_logger().info('Safety zone clear — resuming coverage.')
            self._obstacle_clr_secs = 0.0
            self._resume_after_obstacle()

    # ──────────────────────────────────────────────────────────────────────────
    # Service callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _start_cb(self, request, response):
        if self._state != _State.IDLE:
            response.success = False
            response.message = f'Already active (state={self._state}). Call ~/stop first.'
            return response

        pose = self._get_pose()
        if pose is None:
            response.success = False
            response.message = 'Cannot read pose from TF. Is SLAM running?'
            return response

        x0, y0, yaw0 = pose
        row_length = float(self.get_parameter('row_length').value)

        if row_length <= 0.0:
            # LEARNING: drive straight, LiDAR detects the row end automatically
            self._learn_start = (x0, y0, yaw0)
            self._row_density_buf.clear()
            self._state = _State.LEARNING
            self._drive_open_ended(x0, y0, yaw0)
            self.get_logger().info(
                f'LEARNING: driving from ({x0:.2f}, {y0:.2f}) heading '
                f'{math.degrees(yaw0):.1f}°. '
                'LiDAR will detect the row end automatically.'
            )
            response.success = True
            response.message = 'LEARNING mode: robot driving row 0, will stop at row end automatically.'
        else:
            # Known length: compute all waypoints now and go
            self._state = _State.NAVIGATING
            self._start_coverage(x0, y0, yaw0, row_length)
            response.success = True
            response.message = (
                f'Coverage started: row_length={row_length:.1f} m, '
                f'{self.get_parameter("num_rows").value} rows.'
            )
        return response

    def _stop_cb(self, request, response):
        if self._state == _State.IDLE:
            response.success = False
            response.message = 'Nothing active.'
            return response
        self._cancel_all_goals()
        self._state = _State.IDLE
        response.success = True
        response.message = 'Stopped.'
        return response

    def _mark_row_end_cb(self, request, response):
        """
        Manual row-end marker for LEARNING mode.

        Call this service when you are standing at the end of row 0 to teach
        the system the row length without relying on LiDAR density detection.
        The robot will cancel its open-ended drive and begin the full coverage
        plan using the measured distance as row_length.
        """
        if self._state != _State.LEARNING:
            response.success = False
            response.message = (
                f'mark_row_end only valid in LEARNING state (current: {self._state}). '
                'Call ~/start first with row_length:=0.'
            )
            return response

        pose = self._get_pose()
        if pose is None or self._learn_start is None:
            response.success = False
            response.message = 'Cannot read robot pose from TF.'
            return response

        learned_dist = math.hypot(
            pose[0] - self._learn_start[0],
            pose[1] - self._learn_start[1],
        )
        min_dist = self.get_parameter('row_end_min_dist').value
        if learned_dist < min_dist:
            response.success = False
            response.message = (
                f'Only {learned_dist:.2f} m from start — too short '
                f'(minimum row_end_min_dist = {min_dist:.1f} m). '
                'Drive further before marking row end.'
            )
            return response

        self.get_logger().info(
            f'Row end marked manually at {learned_dist:.2f} m from start.'
        )
        self._on_row_end_detected(pose, learned_dist)
        response.success = True
        response.message = f'Row end marked. Row length = {learned_dist:.2f} m. Starting coverage.'
        return response

    # ──────────────────────────────────────────────────────────────────────────
    # Navigation helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _drive_open_ended(self, x0, y0, yaw0):
        """
        NavigateToPose to a point 300 m away (effectively 'drive until stopped').
        Row-end detection in the tick timer will cancel this when the end is found.
        """
        nav_frame = self.get_parameter('nav_frame').value
        now = self.get_clock().now().to_msg()
        far_x = x0 + 300.0 * math.cos(yaw0)
        far_y = y0 + 300.0 * math.sin(yaw0)

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(far_x, far_y, yaw0, nav_frame, now)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/navigate_to_pose server not available.')
            self._state = _State.IDLE
            return

        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._learning_goal_accepted_cb)

    def _learning_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Learning drive rejected by Nav2.')
            self._state = _State.IDLE
            return
        self._nav_goal_handle = handle
        self.get_logger().info('Learning drive accepted — watching LiDAR for row end.')

    def _on_row_end_detected(self, current_pose, learned_dist):
        """Called by _tick_learning when row end is confirmed by LiDAR."""
        # Cancel the open-ended drive
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        x0, y0, yaw0 = self._learn_start
        row_length = learned_dist

        self.get_logger().info(
            f'Row length learned: {row_length:.2f} m. '
            'Generating coverage waypoints.'
        )
        self._state = _State.NAVIGATING
        self._start_coverage(x0, y0, yaw0, row_length)

    def _start_coverage(self, x0, y0, yaw0, row_length):
        """Generate all waypoints and issue the FollowWaypoints goal."""
        num_rows   = self.get_parameter('num_rows').value
        row_space  = self.get_parameter('row_spacing').value
        buf        = self.get_parameter('buffer_distance').value
        nav_frame  = self.get_parameter('nav_frame').value

        waypoints = generate_coverage_waypoints(
            x0, y0, yaw0, row_length, row_space, num_rows, buf, nav_frame, self.get_clock()
        )
        self._all_waypoints  = waypoints
        self._wp_goal_offset = 0
        self._last_wp_fb_idx = 0

        self.get_logger().info(
            f'Coverage plan: {len(waypoints)} waypoints '
            f'({num_rows} rows × ~{4} wps/row)'
        )
        self._publish_path(waypoints, nav_frame)
        self._issue_waypoints(0)

    def _issue_waypoints(self, from_idx: int):
        """Send self._all_waypoints[from_idx:] to /follow_waypoints."""
        remaining = self._all_waypoints[from_idx:]
        if not remaining:
            self.get_logger().info('No remaining waypoints — coverage done.')
            self._state = _State.DONE
            return

        self._wp_goal_offset = from_idx
        self._last_wp_fb_idx = 0

        if not self._wp_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/follow_waypoints server not available.')
            self._state = _State.IDLE
            return

        goal = FollowWaypoints.Goal()
        goal.poses = remaining
        fut = self._wp_client.send_goal_async(
            goal, feedback_callback=self._wp_feedback_cb
        )
        fut.add_done_callback(self._wp_accepted_cb)

    def _wp_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected.')
            self._state = _State.IDLE
            return
        self._goal_handle = handle
        self.get_logger().info('Coverage goal accepted.')
        handle.get_result_async().add_done_callback(self._wp_result_cb)

    def _wp_feedback_cb(self, msg):
        self._last_wp_fb_idx = msg.feedback.current_waypoint
        row = (self._wp_goal_offset + self._last_wp_fb_idx) // 4
        m = Int32()
        m.data = row
        self._row_pub.publish(m)
        self.get_logger().info(
            f'Row {row} — global wp {self._wp_goal_offset + self._last_wp_fb_idx}',
            throttle_duration_sec=10.0,
        )

    def _wp_result_cb(self, future):
        result  = future.result().result
        missed  = list(result.missed_waypoints)
        self._goal_handle = None
        if self._state == _State.NAVIGATING:   # not cancelled for obstacle/stop
            self._state = _State.DONE
            if missed:
                self.get_logger().warn(
                    f'Coverage done. Missed {len(missed)} waypoints: {missed}. '
                    'Consider increasing goal tolerance or reducing speed.'
                )
            else:
                self.get_logger().info(
                    'All rows complete. '
                    'Save the SLAM map with: bash scripts/save_map.sh ~/maps/field'
                )

    # ──────────────────────────────────────────────────────────────────────────
    # Obstacle stop-and-wait
    # ──────────────────────────────────────────────────────────────────────────

    def _on_obstacle_detected(self):
        """Cancel Nav2 goal and transition to OBSTACLE_WAIT."""
        self.get_logger().warn(
            'Obstacle in safety zone — stopping robot.',
            throttle_duration_sec=2.0,
        )
        self._cancel_active_goal()        # cancel whichever goal is running
        self._obstacle_clr_secs = 0.0
        self._state = _State.OBSTACLE_WAIT
        self._vel_pub.publish(Twist())    # explicit zero velocity

    def _resume_after_obstacle(self):
        """Re-issue remaining waypoints after obstacle is cleared."""
        resume_from = self._wp_goal_offset + self._last_wp_fb_idx
        self.get_logger().info(f'Resuming from global waypoint {resume_from}.')
        self._state = _State.NAVIGATING
        self._issue_waypoints(resume_from)

    # ──────────────────────────────────────────────────────────────────────────
    # Goal cancellation
    # ──────────────────────────────────────────────────────────────────────────

    def _cancel_active_goal(self):
        """Cancel only the currently active Nav2 goal (follow or navigate)."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def _cancel_all_goals(self):
        self._cancel_active_goal()

    # ──────────────────────────────────────────────────────────────────────────
    # Visualisation
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_path(self, waypoints, frame):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = waypoints
        self._path_pub.publish(path)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousRowCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
