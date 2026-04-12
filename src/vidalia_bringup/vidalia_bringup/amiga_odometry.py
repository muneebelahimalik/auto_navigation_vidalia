#!/usr/bin/env python3
"""
amiga_odometry.py — wheel odometry node for the farm-ng Amiga (ROS 2 Humble).

Subscribes
----------
/amiga/vel  (geometry_msgs/TwistStamped)
    Measured velocity published by the amiga_ros_bridge (or a native ROS 2
    Amiga driver).  On a differential-drive robot only linear.x (forward
    speed, m/s) and angular.z (yaw rate, rad/s) are meaningful.

Publishes
---------
/wheel_odom  (nav_msgs/Odometry)
    Dead-reckoned position/velocity in the odom frame.  Frame ids:
      header.frame_id = "odom"
      child_frame_id  = "base_link"

    NOTE: The topic name is /wheel_odom (not /odom) so it does not collide
    with the ICP odometry already published by rtabmap_odom.  The two
    odometry streams can be fused later with robot_localization EKF
    (see config/ekf.yaml).

Parameters
----------
publish_tf  (bool, default false)
    When true, also broadcasts odom→base_link TF.  Keep false while
    icp_odometry is running (it owns that TF); set true only if you want
    to run wheel odometry *instead of* ICP.

vel_topic   (str, default '/amiga/vel')
    TwistStamped topic to subscribe to.

odom_topic  (str, default '/wheel_odom')
    Odometry topic to publish.

Notes on covariance
-------------------
Covariance values are intentionally conservative for soft agricultural soil:
  • position_sigma  = 0.10 m   (wheel slip in field conditions)
  • heading_sigma   = 0.05 rad (yaw drift between ICP updates)
  • linear_v_sigma  = 0.05 m/s (encoder noise + slip)
  • angular_v_sigma = 0.02 rad/s
These will be scaled by robot_localization EKF and do not need to be exact.

ROS 1 bridge note
-----------------
The amiga_ros_bridge is a ROS 1 Noetic package.  To get /amiga/vel into
ROS 2 Humble either:
  1. Run ros1_bridge (recommended for initial testing)
  2. Write / use a native ROS 2 gRPC client for the Amiga canbus service
  3. Use the farm-ng Python SDK in a ROS 2 node (future work)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class AmigaOdometry(Node):

    def __init__(self):
        super().__init__('amiga_odometry')

        # ---- parameters ----
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('vel_topic', '/amiga/vel')
        self.declare_parameter('odom_topic', '/wheel_odom')

        publish_tf = self.get_parameter('publish_tf').value
        vel_topic = self.get_parameter('vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        # ---- state ----
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_stamp = None  # rclpy.time.Time

        # ---- covariance constants ----
        pos_var = 0.10 ** 2       # 10 cm position sigma
        head_var = 0.05 ** 2      # 3 deg heading sigma
        lin_v_var = 0.05 ** 2     # linear velocity variance
        ang_v_var = 0.02 ** 2     # angular velocity variance

        # Flat 6×6 row-major: [x, y, z, roll, pitch, yaw]
        self._pose_cov = [0.0] * 36
        self._pose_cov[0] = pos_var    # xx
        self._pose_cov[7] = pos_var    # yy
        self._pose_cov[14] = 1e-9      # zz (planar — very small)
        self._pose_cov[21] = 1e-9      # roll (planar)
        self._pose_cov[28] = 1e-9      # pitch (planar)
        self._pose_cov[35] = head_var  # yaw

        self._twist_cov = [0.0] * 36
        self._twist_cov[0] = lin_v_var    # vx
        self._twist_cov[7] = 1e-9         # vy (differential drive: no lateral)
        self._twist_cov[14] = 1e-9        # vz
        self._twist_cov[21] = 1e-9        # vroll
        self._twist_cov[28] = 1e-9        # vpitch
        self._twist_cov[35] = ang_v_var   # vyaw

        # ---- publisher ----
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # ---- optional TF broadcaster ----
        self._tf_pub = TransformBroadcaster(self) if publish_tf else None

        # ---- subscriber ----
        self.create_subscription(TwistStamped, vel_topic, self._vel_cb, 10)

        self.get_logger().info(
            f'amiga_odometry started: {vel_topic} → {odom_topic}'
            + (' + TF' if publish_tf else '')
        )

    def _vel_cb(self, msg: TwistStamped) -> None:
        now = rclpy.time.Time.from_msg(msg.header.stamp)

        if self._last_stamp is None:
            self._last_stamp = now
            return

        dt = (now - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = now

        if dt <= 0.0 or dt > 1.0:
            # Skip stale or future messages
            return

        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        # Midpoint (2nd-order Runge-Kutta) integration for better accuracy
        # on curved paths (headland turns).
        theta_mid = self._theta + omega * dt * 0.5
        self._x += v * math.cos(theta_mid) * dt
        self._y += v * math.sin(theta_mid) * dt
        self._theta += omega * dt

        self._publish(msg.header.stamp, v, omega)

    def _publish(self, stamp, v: float, omega: float) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw only
        odom.pose.pose.orientation.z = math.sin(self._theta * 0.5)
        odom.pose.pose.orientation.w = math.cos(self._theta * 0.5)
        odom.pose.covariance = self._pose_cov

        # Twist (body frame)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        odom.twist.covariance = self._twist_cov

        self._odom_pub.publish(odom)

        if self._tf_pub is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z = math.sin(self._theta * 0.5)
            tf.transform.rotation.w = math.cos(self._theta * 0.5)
            self._tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = AmigaOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
