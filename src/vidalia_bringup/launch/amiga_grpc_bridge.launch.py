"""
amiga_grpc_bridge.launch.py -- start the native ROS 2 / farm-ng gRPC bridge.

This replaces the ROS 1 amiga_ros_bridge + ros1_bridge combination.
It connects directly to the Amiga's gRPC services over Tailscale (or LAN).

Published ROS 2 topics
----------------------
  /amiga/vel   (geometry_msgs/TwistStamped) -- from canbus service
  /amiga/pose  (nav_msgs/Odometry)          -- from filter service (GPS)

Subscribed ROS 2 topics
-----------------------
  /cmd_vel     (geometry_msgs/Twist)        -- forwarded to Amiga canbus

Usage
-----
  # Default (uses host from parameter)
  ros2 launch vidalia_bringup amiga_grpc_bridge.launch.py

  # Override host and ports
  ros2 launch vidalia_bringup amiga_grpc_bridge.launch.py \\
      host:=camphor-clone.tail0be07.ts.net \\
      canbus_port:=6001 \\
      filter_port:=20001

  # Also publish filter pose even when GPS has not converged (lab use)
  ros2 launch vidalia_bringup amiga_grpc_bridge.launch.py publish_unconverged_filter:=true

After this is running, you should see:
  ros2 topic echo /amiga/vel          # wheel velocity
  ros2 topic echo /amiga/pose         # GPS pose (empty in lab until GPS locks)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration('host')
    canbus_port = LaunchConfiguration('canbus_port')
    filter_port = LaunchConfiguration('filter_port')
    publish_unconverged_filter = LaunchConfiguration('publish_unconverged_filter')

    bridge = Node(
        package='vidalia_bringup',
        executable='amiga_ros2_bridge',
        name='amiga_ros2_bridge',
        output='screen',
        parameters=[{
            'host': host,
            'canbus_port': canbus_port,
            'filter_port': filter_port,
            'max_linear': 1.5,
            'max_angular': 1.0,
            'publish_unconverged_filter': publish_unconverged_filter,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='camphor-clone.tail0be07.ts.net',
            description='Amiga Tailscale hostname or IP address',
        ),
        DeclareLaunchArgument(
            'canbus_port',
            default_value='6001',
            description='Amiga canbus gRPC service port',
        ),
        DeclareLaunchArgument(
            'filter_port',
            default_value='20001',
            description='Amiga filter (state estimation) gRPC service port',
        ),
        DeclareLaunchArgument(
            'publish_unconverged_filter',
            default_value='false',
            description='Publish /amiga/pose even when GPS has not converged (lab debug)',
        ),
        bridge,
    ])
