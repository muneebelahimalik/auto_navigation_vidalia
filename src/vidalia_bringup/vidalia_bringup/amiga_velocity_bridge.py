#!/usr/bin/env python3
"""
amiga_velocity_bridge.py — velocity command relay for the farm-ng Amiga (ROS 2 Humble).

This node bridges the gap between the standard ROS 2 navigation command topic
(/cmd_vel, used by Nav2 and teleop tools) and the Amiga-specific topic that the
amiga_ros_bridge expects (/amiga/cmd_vel).

Subscribes
----------
/cmd_vel  (geometry_msgs/Twist)
    Velocity commands from Nav2, keyboard teleop, or any standard ROS 2 tool.
    Only linear.x (forward, m/s) and angular.z (yaw rate, rad/s) are forwarded
    to the differential-drive Amiga; all other components are zeroed.

Publishes
---------
/amiga/cmd_vel  (geometry_msgs/Twist)
    Velocity command consumed by amiga_ros_bridge → Amiga gRPC canbus service.

Parameters
----------
cmd_vel_in   (str, default '/cmd_vel')
    Source topic (from Nav2 / teleop).

cmd_vel_out  (str, default '/amiga/cmd_vel')
    Destination topic (consumed by amiga_ros_bridge).

max_linear   (float, default 1.5)
    Hard clamp on |linear.x| (m/s).  Amiga safe speed in field conditions.

max_angular  (float, default 1.0)
    Hard clamp on |angular.z| (rad/s).  ~57 degrees/s max turn rate.

watchdog_timeout (float, default 0.5)
    If no /cmd_vel message is received for this many seconds, publish a zero
    Twist to stop the robot safely.

Usage
-----
When using Nav2 for autonomous navigation, Nav2 publishes to /cmd_vel by
default.  This node re-publishes those commands to /amiga/cmd_vel so the
Amiga drives.

For manual teleop with keyboard:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # (publishes to /cmd_vel by default)

For the amiga_ros_bridge (ROS 1) to receive these commands you need either:
  1. ros1_bridge running to forward /amiga/cmd_vel from ROS 2 → ROS 1
  2. A native ROS 2 gRPC client for the Amiga canbus service
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist


class AmigaVelocityBridge(Node):

    def __init__(self):
        super().__init__('amiga_velocity_bridge')

        # ---- parameters ----
        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('cmd_vel_out', '/amiga/cmd_vel')
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('watchdog_timeout', 0.5)

        in_topic = self.get_parameter('cmd_vel_in').value
        out_topic = self.get_parameter('cmd_vel_out').value
        self._max_lin = float(self.get_parameter('max_linear').value)
        self._max_ang = float(self.get_parameter('max_angular').value)
        timeout = float(self.get_parameter('watchdog_timeout').value)

        # ---- publisher ----
        self._pub = self.create_publisher(Twist, out_topic, 10)

        # ---- subscriber ----
        self.create_subscription(Twist, in_topic, self._cmd_cb, 10)

        # ---- watchdog timer ----
        self._last_cmd_time = self.get_clock().now()
        self._timeout = Duration(seconds=timeout)
        self._watchdog = self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info(
            f'amiga_velocity_bridge: {in_topic} → {out_topic} '
            f'(max_lin={self._max_lin} m/s, max_ang={self._max_ang} rad/s, '
            f'watchdog={timeout} s)'
        )

    def _clamp(self, value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd_time = self.get_clock().now()

        out = Twist()
        out.linear.x = self._clamp(msg.linear.x, self._max_lin)
        out.angular.z = self._clamp(msg.angular.z, self._max_ang)
        # Differential drive: lateral/vertical velocities are always zero
        self._pub.publish(out)

    def _watchdog_cb(self) -> None:
        """Publish zero velocity if commands stop arriving (safety stop)."""
        elapsed = self.get_clock().now() - self._last_cmd_time
        if elapsed > self._timeout:
            self._pub.publish(Twist())  # all zeros = stop


def main(args=None):
    rclpy.init(args=args)
    node = AmigaVelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
