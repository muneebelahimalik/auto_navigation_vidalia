"""
field_coverage_planner.py -- Boustrophedon field coverage path planner for autonomous weed control.

Generates a back-and-forth (lawn-mower) waypoint pattern across a field polygon and
dispatches it to Nav2's /follow_waypoints action.

ROS 2 interface
---------------
Services:
  ~/start_coverage  (std_srvs/Trigger) -- compute and send waypoints now
  ~/stop_coverage   (std_srvs/Trigger) -- cancel the active goal

Topics published:
  ~/coverage_path   (nav_msgs/Path) -- the planned waypoints for RViz visualisation

Parameters (see config/field_coverage.yaml):
  field_corners   [[x,y],...] -- polygon vertices in map frame (CCW order)
  row_spacing     float       -- distance between passes (m), default 0.45
  row_heading     float       -- row travel direction in degrees from map +X (default 0)
  start_margin    float       -- pull row endpoints this many metres inside the polygon
  nav_frame       str         -- coordinate frame for waypoints (default: map)

Typical usage
-------------
  # 1. Edit config/field_coverage.yaml: set field_corners to match your field
  # 2. Launch localize_nav.launch.py (or slam_nav.launch.py)
  # 3. In a second terminal:
  ros2 launch vidalia_bringup field_coverage.launch.py
  # 4. Trigger coverage:
  ros2 service call /field_coverage_planner/start_coverage std_srvs/srv/Trigger {}
  # 5. Cancel at any time:
  ros2 service call /field_coverage_planner/stop_coverage std_srvs/srv/Trigger {}
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _quat_from_yaw(yaw: float) -> Quaternion:
    """Return a geometry_msgs/Quaternion for a pure Z rotation."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _rotate(x: float, y: float, angle_rad: float):
    """Rotate (x, y) by angle_rad CCW around the origin."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return c * x - s * y, s * x + c * y


def _polygon_x_range_at_y(vertices, y: float):
    """
    Return (x_min, x_max) where a horizontal line at the given y intersects
    the polygon defined by vertices (list of (x, y) tuples, closed).
    Returns None if the line does not intersect the polygon.
    """
    n = len(vertices)
    xs = []
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))
    if len(xs) < 2:
        return None
    return min(xs), max(xs)


def generate_boustrophedon(corners, row_spacing: float, heading_deg: float,
                           start_margin: float, nav_frame: str, node_clock):
    """
    Generate a boustrophedon (back-and-forth) list of PoseStamped waypoints.

    Parameters
    ----------
    corners       : list of [x, y] floats -- field polygon in map frame (CCW)
    row_spacing   : metres between parallel passes
    heading_deg   : direction robot travels *along each row*, degrees from map +X
    start_margin  : metres to pull row endpoints inward from the polygon edges
    nav_frame     : TF frame for the PoseStamped messages
    node_clock    : rclpy clock (for stamp)
    """
    heading_rad = math.radians(heading_deg)
    perp_rad = heading_rad + math.pi / 2.0

    rot_corners = [_rotate(x, y, -heading_rad) for x, y in corners]

    rx_vals = [p[0] for p in rot_corners]
    ry_vals = [p[1] for p in rot_corners]
    ry_min, ry_max = min(ry_vals), max(ry_vals)

    y_start = ry_min + row_spacing / 2.0
    row_ys = []
    y = y_start
    while y < ry_max:
        row_ys.append(y)
        y += row_spacing

    waypoints = []
    now = node_clock.now().to_msg()

    for row_idx, ry in enumerate(row_ys):
        result = _polygon_x_range_at_y(rot_corners, ry)
        if result is None:
            continue
        rx_min, rx_max = result

        rx_start = rx_min + start_margin
        rx_end = rx_max - start_margin
        if rx_start >= rx_end:
            continue

        if row_idx % 2 == 0:
            row_points_rot = [(rx_start, ry), (rx_end, ry)]
            travel_yaw = 0.0
        else:
            row_points_rot = [(rx_end, ry), (rx_start, ry)]
            travel_yaw = math.pi

        map_travel_yaw = heading_rad + travel_yaw
        q = _quat_from_yaw(map_travel_yaw)

        for rx, ry_pt in row_points_rot:
            mx, my = _rotate(rx, ry_pt, heading_rad)
            ps = PoseStamped()
            ps.header.frame_id = nav_frame
            ps.header.stamp = now
            ps.pose.position.x = mx
            ps.pose.position.y = my
            ps.pose.position.z = 0.0
            ps.pose.orientation = q
            waypoints.append(ps)

    return waypoints


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class FieldCoveragePlannerNode(Node):

    def __init__(self):
        super().__init__('field_coverage_planner')

        # --- Parameters ---
        self.declare_parameter(
            'field_corners',
            [0.0, 0.0,  20.0, 0.0,  20.0, 10.0,  0.0, 10.0],
        )
        self.declare_parameter('row_spacing', 0.45)
        self.declare_parameter('row_heading', 0.0)
        self.declare_parameter('start_margin', 0.5)
        self.declare_parameter('nav_frame', 'map')

        # --- Nav2 action client ---
        self._waypoint_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._goal_handle = None

        # --- Path publisher (for RViz visualisation) ---
        self._path_pub = self.create_publisher(Path, '~/coverage_path', 10)

        # --- Services ---
        self.create_service(Trigger, '~/start_coverage', self._start_cb)
        self.create_service(Trigger, '~/stop_coverage', self._stop_cb)

        self.get_logger().info(
            'FieldCoveragePlanner ready. '
            'Call ~/start_coverage to begin autonomous field coverage.'
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _start_cb(self, request, response):
        waypoints = self._build_waypoints()
        if not waypoints:
            response.success = False
            response.message = 'No waypoints generated -- check field_corners parameter.'
            return response

        self.get_logger().info(
            f'Coverage plan: {len(waypoints)} waypoints, '
            f'row_spacing={self.get_parameter("row_spacing").value} m, '
            f'heading={self.get_parameter("row_heading").value} deg'
        )
        self._publish_path(waypoints)
        self._send_waypoints(waypoints)
        response.success = True
        response.message = f'Coverage started: {len(waypoints)} waypoints.'
        return response

    def _stop_cb(self, request, response):
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling active coverage goal.')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            response.success = True
            response.message = 'Coverage cancelled.'
        else:
            response.success = False
            response.message = 'No active coverage goal to cancel.'
        return response

    # ------------------------------------------------------------------
    # Waypoint generation
    # ------------------------------------------------------------------

    def _build_waypoints(self):
        flat = self.get_parameter('field_corners').value
        if len(flat) < 6 or len(flat) % 2 != 0:
            self.get_logger().error(
                f'field_corners must be a flat list of x,y pairs (got {len(flat)} values).'
            )
            return []

        corners = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        row_spacing = self.get_parameter('row_spacing').value
        heading_deg = self.get_parameter('row_heading').value
        start_margin = self.get_parameter('start_margin').value
        nav_frame = self.get_parameter('nav_frame').value

        waypoints = generate_boustrophedon(
            corners, row_spacing, heading_deg, start_margin, nav_frame, self.get_clock()
        )
        return waypoints

    # ------------------------------------------------------------------
    # Nav2 dispatch
    # ------------------------------------------------------------------

    def _send_waypoints(self, waypoints):
        if not self._waypoint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/follow_waypoints action server not available.')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = waypoints

        send_future = self._waypoint_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Waypoint goal rejected by Nav2.')
            return
        self._goal_handle = handle
        self.get_logger().info('Coverage goal accepted by Nav2.')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Navigating to waypoint {fb.current_waypoint + 1}',
            throttle_duration_sec=5.0,
        )

    def _result_cb(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)
        self._goal_handle = None
        if missed:
            self.get_logger().warn(
                f'Coverage complete. Missed waypoints: {missed}'
            )
        else:
            self.get_logger().info('Coverage complete -- all waypoints reached.')

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def _publish_path(self, waypoints):
        path = Path()
        path.header.frame_id = self.get_parameter('nav_frame').value
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = waypoints
        self._path_pub.publish(path)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = FieldCoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
