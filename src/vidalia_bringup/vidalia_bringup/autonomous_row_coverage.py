"""
autonomous_row_coverage.py -- Autonomous boustrophedon row coverage with live SLAM,
LiDAR-based row-end detection, and obstacle stop-and-wait safety.

Place the robot at the START of the first onion row, facing along the row.
Call ~/start.  The robot drives itself.

  Row end detection (LiDAR)
  -------------------------
  The VLP-16 is sampled every scan for a "crop density" zone.
  At the headland (end of row), returns drop sharply.
  When the median density over the last N scans falls below a threshold
  the row end is declared, the robot halts, and the end-of-row maneuver begins.

  Obstacle stop-and-wait (LiDAR)
  --------------------------------
  A tight safety box directly ahead of the robot is monitored continuously.
  Any points in that zone cancel the active Nav2 goal and publish zero velocity.
  When the zone is clear for obstacle_clear_secs, the remaining waypoints are re-issued.

ROS 2 interface
---------------
Services:
  ~/start          Trigger -- begin coverage from current pose
  ~/stop           Trigger -- cancel immediately
  ~/mark_row_end   Trigger -- manually mark current position as row end (LEARNING mode only)

Topics published:
  ~/coverage_path  nav_msgs/Path  -- full planned route
  ~/current_row    std_msgs/Int32 -- index of row being driven

Parameters
----------
  row_length            float  Expected row length (m). 0 = auto-detect via LiDAR.
  num_rows              int    Number of rows to cover. Default 20.
  row_spacing           float  Distance between row centres (m). Default 0.45.
  buffer_distance       float  Distance past crop to drive before turning (m). Default 1.5.
  row_end_threshold     int    Points/scan below which end-of-row is declared. Default 5.
  row_end_confirm_scans int    Consecutive low-density scans to confirm row end. Default 4.
  row_end_min_dist      float  Min metres driven before row-end detector activates. Default 3.0.
  obstacle_stop_dist    float  Depth of forward safety zone (m). Default 1.5.
  obstacle_clear_secs   float  Seconds zone must be clear before resuming. Default 3.0.
  obstacle_threshold    int    Points in safety zone counted as obstacle. Default 3.
  nav_frame             str    TF frame for waypoints. Default: map.
  base_frame            str    Robot base frame. Default: base_link.
  lidar_x_offset        float  Velodyne X offset from base_link (m). Default 1.13.
  lidar_z_offset        float  Velodyne Z offset from base_link (m). Default 0.80.

Usage
-----
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


# -------------------------------------------------------------------------
# LiDAR zone counting  (fast numpy)
# -------------------------------------------------------------------------

def _count_in_box(cloud_msg: PointCloud2,
                  x_min: float, x_max: float,
                  y_min: float, y_max: float,
                  z_min: float, z_max: float,
                  lidar_x: float, lidar_z: float) -> int:
    """
    Count PointCloud2 points that fall inside an axis-aligned box.

    The cloud is in the velodyne frame.  The box is specified in the
    base_link frame.  The transform velodyne->base_link is a pure translation:
        base_x = vel_x + lidar_x
        base_y = vel_y
        base_z = vel_z + lidar_z
    so we shift the box bounds into velodyne frame instead.
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

    xn, xx = x_min - lidar_x, x_max - lidar_x
    yn, yx = y_min,           y_max
    zn, zx = z_min - lidar_z, z_max - lidar_z

    ox = field_map['x']
    oy = field_map['y']
    oz = field_map['z']

    x = pts[:, ox:ox + 4].copy().view(np.float32).ravel()
    y = pts[:, oy:oy + 4].copy().view(np.float32).ravel()
    z = pts[:, oz:oz + 4].copy().view(np.float32).ravel()

    mask = (x >= xn) & (x <= xx) & (y >= yn) & (y <= yx) & (z >= zn) & (z <= zx)
    return int(np.sum(mask))


# -------------------------------------------------------------------------
# Geometry helpers
# -------------------------------------------------------------------------

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


# -------------------------------------------------------------------------
# Boustrophedon waypoint generation
# -------------------------------------------------------------------------

def _end_of_row_maneuver(x, y, heading, row_spacing, buf, lat_x0, lat_y0, frame, stamp):
    """
    Three waypoints that move the robot from the end of one row to the start
    of the next, stacking rows to the LEFT of the INITIAL heading (yaw0).
    """
    fwd_x = math.cos(heading)
    fwd_y = math.sin(heading)

    bx = x + buf * fwd_x
    by = y + buf * fwd_y

    cx = bx + row_spacing * lat_x0
    cy = by + row_spacing * lat_y0
    next_heading = heading + math.pi

    nx = cx + buf * math.cos(next_heading)
    ny = cy + buf * math.sin(next_heading)

    return [
        _make_pose(bx, by, heading,      frame, stamp),
        _make_pose(cx, cy, next_heading, frame, stamp),
        _make_pose(nx, ny, next_heading, frame, stamp),
    ]


def generate_coverage_waypoints(x0, y0, yaw0,
                                 row_length, row_spacing, num_rows,
                                 buf, frame, clock):
    """Full boustrophedon waypoint list starting from (x0, y0, yaw0)."""
    now = clock.now().to_msg()
    waypoints = []

    fwd_x = math.cos(yaw0)
    fwd_y = math.sin(yaw0)
    lat_x = -math.sin(yaw0)
    lat_y =  math.cos(yaw0)

    for i in range(num_rows):
        ox = x0 + i * row_spacing * lat_x
        oy = y0 + i * row_spacing * lat_y

        if i % 2 == 0:
            heading = yaw0
            end_x = ox + row_length * fwd_x
            end_y = oy + row_length * fwd_y
        else:
            heading = yaw0 + math.pi
            ox += row_length * fwd_x
            oy += row_length * fwd_y
            end_x = x0 + i * row_spacing * lat_x
            end_y = y0 + i * row_spacing * lat_y

        waypoints.append(_make_pose(end_x, end_y, heading, frame, now))

        if i < num_rows - 1:
            waypoints.extend(
                _end_of_row_maneuver(
                    end_x, end_y, heading, row_spacing, buf,
                    lat_x, lat_y,
                    frame, now,
                )
            )

    return waypoints


# -------------------------------------------------------------------------
# State constants
# -------------------------------------------------------------------------

class _State:
    IDLE           = 'IDLE'
    LEARNING       = 'LEARNING'
    NAVIGATING     = 'NAVIGATING'
    OBSTACLE_WAIT  = 'OBSTACLE_WAIT'
    DONE           = 'DONE'


# -------------------------------------------------------------------------
# Node
# -------------------------------------------------------------------------

class AutonomousRowCoverageNode(Node):

    def __init__(self):
        super().__init__('autonomous_row_coverage')

        self.declare_parameter('row_length',            0.0)
        self.declare_parameter('num_rows',              20)
        self.declare_parameter('row_spacing',           0.45)
        self.declare_parameter('buffer_distance',       1.5)
        self.declare_parameter('row_end_threshold',     5)
        self.declare_parameter('row_end_confirm_scans', 4)
        self.declare_parameter('row_end_min_dist',      3.0)
        self.declare_parameter('obstacle_stop_dist',    1.5)
        self.declare_parameter('obstacle_clear_secs',   3.0)
        self.declare_parameter('obstacle_threshold',    3)
        self.declare_parameter('nav_frame',             'map')
        self.declare_parameter('base_frame',            'base_link')
        self.declare_parameter('lidar_x_offset',        1.13)
        self.declare_parameter('lidar_z_offset',        0.80)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._wp_client  = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._nav_client = ActionClient(self, NavigateToPose,  '/navigate_to_pose')

        self._path_pub = self.create_publisher(Path,   '~/coverage_path', 10)
        self._row_pub  = self.create_publisher(Int32,  '~/current_row',   10)
        self._vel_pub  = self.create_publisher(Twist,  '/cmd_vel',        10)

        self.create_service(Trigger, '~/start',        self._start_cb)
        self.create_service(Trigger, '~/stop',         self._stop_cb)
        self.create_service(Trigger, '~/mark_row_end', self._mark_row_end_cb)

        self.create_subscription(
            PointCloud2, '/velodyne_points', self._cloud_cb, 5
        )

        self.create_timer(0.10, self._tick)

        self._state             = _State.IDLE
        self._goal_handle       = None
        self._nav_goal_handle   = None

        self._learn_start       = None
        self._all_waypoints     = []
        self._wp_goal_offset    = 0
        self._last_wp_fb_idx    = 0

        self._row_density_buf   = deque(maxlen=10)
        self._obstacle_present  = False
        self._obstacle_clr_secs = 0.0

        self.get_logger().info(
            f'\nAutonomousRowCoverage ready.'
            f'\n  row_length   = {self.get_parameter("row_length").value:.1f} m  (0 = auto-detect via LiDAR)'
        )

    # -------------------------------------------------------------------
    # TF helper
    # -------------------------------------------------------------------

    def _get_pose(self):
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
            self.get_logger().error(f'TF {nav}->{base}: {e}', throttle_duration_sec=5.0)
            return None

    # -------------------------------------------------------------------
    # LiDAR callback
    # -------------------------------------------------------------------

    def _cloud_cb(self, cloud: PointCloud2):
        lx = self.get_parameter('lidar_x_offset').value
        lz = self.get_parameter('lidar_z_offset').value

        stop_dist = self.get_parameter('obstacle_stop_dist').value
        obs_count = _count_in_box(
            cloud,
            x_min=0.15,       x_max=stop_dist,
            y_min=-0.55,      y_max=0.55,
            z_min=0.10,       z_max=2.00,
            lidar_x=lx, lidar_z=lz,
        )
        self._obstacle_present = obs_count >= self.get_parameter('obstacle_threshold').value

        density = _count_in_box(
            cloud,
            x_min=1.00,  x_max=4.00,
            y_min=-2.00, y_max=2.00,
            z_min=0.00,  z_max=1.50,
            lidar_x=lx, lidar_z=lz,
        )
        self._row_density_buf.append(density)

    # -------------------------------------------------------------------
    # Tick timer -- 10 Hz state-machine driver
    # -------------------------------------------------------------------

    def _tick(self):
        state = self._state

        if state in (_State.NAVIGATING, _State.LEARNING):
            if self._obstacle_present:
                self._on_obstacle_detected()
                return

        if state == _State.OBSTACLE_WAIT:
            self._tick_obstacle_wait()
            return

        if state == _State.LEARNING:
            self._tick_learning()

    def _tick_learning(self):
        min_dist  = self.get_parameter('row_end_min_dist').value
        threshold = self.get_parameter('row_end_threshold').value
        confirm   = self.get_parameter('row_end_confirm_scans').value

        if len(self._row_density_buf) < confirm:
            return

        pose = self._get_pose()
        if pose is None or self._learn_start is None:
            return
        dist = math.hypot(pose[0] - self._learn_start[0],
                          pose[1] - self._learn_start[1])
        if dist < min_dist:
            return

        recent = list(self._row_density_buf)[-confirm:]
        if all(d <= threshold for d in recent):
            self.get_logger().info(
                f'Row end detected at distance {dist:.2f} m (density: {recent})'
            )
            self._on_row_end_detected(pose, dist)

    def _tick_obstacle_wait(self):
        if self._obstacle_present:
            self._obstacle_clr_secs = 0.0
            self._vel_pub.publish(Twist())
            return

        self._obstacle_clr_secs += 0.10
        wait_needed = self.get_parameter('obstacle_clear_secs').value

        self.get_logger().info(
            f'Obstacle cleared -- resuming in {wait_needed - self._obstacle_clr_secs:.1f} s',
            throttle_duration_sec=1.0,
        )

        if self._obstacle_clr_secs >= wait_needed:
            self.get_logger().info('Safety zone clear -- resuming coverage.')
            self._obstacle_clr_secs = 0.0
            self._resume_after_obstacle()

    # -------------------------------------------------------------------
    # Service callbacks
    # -------------------------------------------------------------------

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
            self._learn_start = (x0, y0, yaw0)
            self._row_density_buf.clear()
            self._state = _State.LEARNING
            self._drive_open_ended(x0, y0, yaw0)
            self.get_logger().info(
                f'LEARNING: driving from ({x0:.2f}, {y0:.2f}) heading '
                f'{math.degrees(yaw0):.1f} deg. '
                'LiDAR will detect the row end automatically.'
            )
            response.success = True
            response.message = 'LEARNING mode: robot driving row 0, will stop at row end automatically.'
        else:
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
                f'Only {learned_dist:.2f} m from start -- too short '
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

    # -------------------------------------------------------------------
    # Navigation helpers
    # -------------------------------------------------------------------

    def _drive_open_ended(self, x0, y0, yaw0):
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
        self.get_logger().info('Learning drive accepted -- watching LiDAR for row end.')

    def _on_row_end_detected(self, current_pose, learned_dist):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        x0, y0, yaw0 = self._learn_start
        row_length = learned_dist

        self.get_logger().info(
            f'Row length learned: {row_length:.2f} m. Generating coverage waypoints.'
        )
        self._state = _State.NAVIGATING
        self._start_coverage(x0, y0, yaw0, row_length)

    def _start_coverage(self, x0, y0, yaw0, row_length):
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
            f'({num_rows} rows x ~4 wps/row)'
        )
        self._publish_path(waypoints, nav_frame)
        self._issue_waypoints(0)

    def _issue_waypoints(self, from_idx: int):
        remaining = self._all_waypoints[from_idx:]
        if not remaining:
            self.get_logger().info('No remaining waypoints -- coverage done.')
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
            f'Row {row} -- global wp {self._wp_goal_offset + self._last_wp_fb_idx}',
            throttle_duration_sec=10.0,
        )

    def _wp_result_cb(self, future):
        result  = future.result().result
        missed  = list(result.missed_waypoints)
        self._goal_handle = None
        if self._state == _State.NAVIGATING:
            self._state = _State.DONE
            if missed:
                self.get_logger().warn(
                    f'Coverage done. Missed {len(missed)} waypoints: {missed}.'
                )
            else:
                self.get_logger().info(
                    'All rows complete. '
                    'Save the SLAM map with: bash scripts/save_map.sh ~/maps/field'
                )

    # -------------------------------------------------------------------
    # Obstacle stop-and-wait
    # -------------------------------------------------------------------

    def _on_obstacle_detected(self):
        self.get_logger().warn(
            'Obstacle in safety zone -- stopping robot.',
            throttle_duration_sec=2.0,
        )
        self._cancel_active_goal()
        self._obstacle_clr_secs = 0.0
        self._state = _State.OBSTACLE_WAIT
        self._vel_pub.publish(Twist())

    def _resume_after_obstacle(self):
        resume_from = self._wp_goal_offset + self._last_wp_fb_idx
        self.get_logger().info(f'Resuming from global waypoint {resume_from}.')
        self._state = _State.NAVIGATING
        self._issue_waypoints(resume_from)

    # -------------------------------------------------------------------
    # Goal cancellation
    # -------------------------------------------------------------------

    def _cancel_active_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    def _cancel_all_goals(self):
        self._cancel_active_goal()

    # -------------------------------------------------------------------
    # Visualisation
    # -------------------------------------------------------------------

    def _publish_path(self, waypoints, frame):
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = waypoints
        self._path_pub.publish(path)


# -------------------------------------------------------------------------
# Entry point
# -------------------------------------------------------------------------

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
