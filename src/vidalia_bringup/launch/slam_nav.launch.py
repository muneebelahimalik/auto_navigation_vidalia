"""
slam_nav.launch.py — SLAM mapping + Nav2 autonomous navigation together.

Use this when you need to build the initial field map while the robot is
navigating autonomously (e.g. first field survey run).

Stack launched
--------------
  1. amiga_ros2_bridge  — gRPC → /amiga/vel + /amiga/pose + cmd_vel relay
  2. amiga_odometry     — /amiga/vel → /wheel_odom
  3. Velodyne VLP-16    — /velodyne_points
  4. robot_state_publisher — URDF → base_link→velodyne TF
  5. icp_odometry       — /velodyne_points → /odom  +  odom→base_link TF
  6. rtabmap            — active mapping; publishes /map + map→odom TF
  7. Nav2               — controller, planner, behavior, bt_navigator,
                          waypoint_follower, lifecycle_manager
  8. RViz2              — optional (rviz:=true)

Workflow
--------
  1. Drive the robot to the field start position.
  2. Launch this file — the map starts empty and grows as the robot moves.
  3. Send nav goals via RViz2 or the /follow_waypoints action.
  4. When done, save the map:
       bash scripts/save_map.sh ~/maps/field
  5. For future autonomous runs, use localize_nav.launch.py instead.

Usage
-----
  ros2 launch vidalia_bringup slam_nav.launch.py
  ros2 launch vidalia_bringup slam_nav.launch.py rviz:=true
  ros2 launch vidalia_bringup slam_nav.launch.py \
      database_path:=~/maps/field_in_progress.db rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')
    amiga_host = LaunchConfiguration('amiga_host')
    canbus_port = LaunchConfiguration('canbus_port')
    filter_port = LaunchConfiguration('filter_port')
    params_file = LaunchConfiguration('params_file')

    # ---- 1–2. Amiga gRPC bridge + wheel odometry ----
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

    amiga_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'vidalia_bringup_nodes.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ekf': 'false',
        }.items(),
    )

    # ---- 3–6. VLP-16 + active SLAM ----
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

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # ---- 7. Nav2 navigation stack ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database path (map written here)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2',
        ),
        DeclareLaunchArgument(
            'amiga_host',
            default_value='camphor-clone.tail0be07.ts.net',
            description='Amiga Tailscale hostname or IP',
        ),
        DeclareLaunchArgument(
            'canbus_port', default_value='6001',
            description='Amiga canbus gRPC service port',
        ),
        DeclareLaunchArgument(
            'filter_port', default_value='20001',
            description='Amiga filter gRPC service port',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('vidalia_bringup'),
                'config', 'nav2_params.yaml',
            ),
            description='Nav2 params file',
        ),
        grpc_bridge,
        amiga_odom,
        sensors,
        slam,
        nav2,
    ])
