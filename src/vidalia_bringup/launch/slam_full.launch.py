"""
slam_full.launch.py — complete live stack: VLP-16 + Amiga gRPC bridge + SLAM.

This is the all-in-one launch file for running on the physical Amiga robot.
It brings up all components in the correct order:

  1. amiga_ros2_bridge   — connects to Amiga gRPC (Tailscale) →
                           /amiga/vel (wheel velocity) + /amiga/pose (GPS)
  2. amiga_odometry      — /amiga/vel → /wheel_odom (dead-reckoning)
  3. Velodyne VLP-16     — UDP → /velodyne_packets → /velodyne_points
  4. robot_state_publisher — URDF → base_link → velodyne TF
  5. icp_odometry        — /velodyne_points → /odom  +  odom→base_link TF
  6. rtabmap             — builds 3D map, publishes map→odom TF
  7. rtabmap_viz         — 3D visualisation of SLAM output
  8. RViz2               — optional full GUI visualisation

Prerequisites
-------------
• Amiga brain reachable over Tailscale (or LAN) — test with filter_client.py
• VLP-16 reachable at 192.168.1.201:2368 (same LAN as this PC)
• farm-ng Python SDK installed:  pip install farm-ng-amiga

Usage
-----
  ros2 launch vidalia_bringup slam_full.launch.py
  ros2 launch vidalia_bringup slam_full.launch.py rviz:=true
  ros2 launch vidalia_bringup slam_full.launch.py database_path:=~/maps/field.db

  # Override Amiga Tailscale host (if different):
  ros2 launch vidalia_bringup slam_full.launch.py \
      amiga_host:=camphor-clone.tail0be07.ts.net
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    amiga_host = LaunchConfiguration('amiga_host')
    canbus_port = LaunchConfiguration('canbus_port')
    filter_port = LaunchConfiguration('filter_port')

    # ---- 1. Native gRPC bridge (replaces ROS 1 amiga_ros_bridge) ----
    grpc_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'amiga_grpc_bridge.launch.py')
        ),
        launch_arguments={
            'host': amiga_host,
            'canbus_port': canbus_port,
            'filter_port': filter_port,
            'publish_unconverged_filter': 'false',
        }.items(),
    )

    # ---- 2. Wheel odometry (amiga/vel → /wheel_odom) ----
    amiga_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'vidalia_bringup_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ekf': 'false',  # ICP odometry owns odom→base_link TF
        }.items(),
    )

    # ---- 3-7. VLP-16 + SLAM ----
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_rtabmap_lidar3d.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'cloud_topic': '/velodyne_points',
            'database_path': database_path,
            'rviz': rviz,
        }.items(),
    )

    # ---- VLP-16 sensor driver ----
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 for visualisation'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database path (map persisted here)'),
        DeclareLaunchArgument(
            'amiga_host',
            default_value='camphor-clone.tail0be07.ts.net',
            description='Amiga Tailscale hostname or IP'),
        DeclareLaunchArgument(
            'canbus_port', default_value='6001',
            description='Amiga canbus gRPC service port'),
        DeclareLaunchArgument(
            'filter_port', default_value='20001',
            description='Amiga filter gRPC service port'),
        grpc_bridge,
        amiga_odom,
        sensors,
        slam,
    ])
