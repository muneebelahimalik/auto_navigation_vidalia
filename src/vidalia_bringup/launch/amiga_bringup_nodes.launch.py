"""
amiga_bringup_nodes.launch.py -- Amiga robot interface nodes for ROS 2 Foxy.

Launches:
  1. amiga_odometry      -- converts /amiga/vel -> /wheel_odom (dead-reckoning)
  2. amiga_velocity_bridge -- relays /cmd_vel -> /amiga/cmd_vel  (Nav2 -> Amiga)
  3. ekf_filter_node     -- (optional) fuses /odom + /wheel_odom into
                           /odometry/filtered (requires robot_localization)

Prerequisites
-------------
/amiga/vel must be available in ROS 2.  Options:
  A. Use amiga_grpc_bridge.launch.py (native ROS 2 gRPC client for the Amiga canbus service).
  B. Use ros1_bridge to bridge amiga_ros_bridge topics from ROS 1 Noetic.

EKF fusion (use_ekf:=true)
---------------------------
When enabled, the EKF fuses ICP odometry with wheel odometry for more
robust pose estimation on slippery agricultural soil.

Usage
-----
  # Amiga nodes only (sensors + SLAM running separately):
  ros2 launch vidalia_bringup amiga_bringup_nodes.launch.py

  # With EKF fusion:
  ros2 launch vidalia_bringup amiga_bringup_nodes.launch.py use_ekf:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ekf = LaunchConfiguration('use_ekf')

    # ---- Wheel odometry ----
    amiga_odom = Node(
        package='vidalia_bringup',
        executable='amiga_odometry',
        name='amiga_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_tf': False,
            'vel_topic': '/amiga/vel',
            'odom_topic': '/wheel_odom',
        }],
    )

    # ---- Velocity bridge (Nav2 /cmd_vel -> Amiga /amiga/cmd_vel) ----
    vel_bridge = Node(
        package='vidalia_bringup',
        executable='amiga_velocity_bridge',
        name='amiga_velocity_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cmd_vel_in': '/cmd_vel',
            'cmd_vel_out': '/amiga/cmd_vel',
            'max_linear': 1.5,
            'max_angular': 1.0,
            'watchdog_timeout': 0.5,
        }],
    )

    # ---- EKF fusion (optional, requires robot_localization package) ----
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(use_ekf),
        parameters=[
            os.path.join(pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description=(
                'Set true to fuse ICP odometry and wheel odometry with EKF. '
                'Requires robot_localization package and that icp_odometry '
                'has publish_tf:=false to avoid TF conflicts.'
            )),
        amiga_odom,
        vel_bridge,
        ekf_node,
    ])
