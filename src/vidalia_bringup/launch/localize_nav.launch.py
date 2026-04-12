"""
localize_nav.launch.py — full production stack for autonomous field runs.

Use this when the field map has already been built.  Localizes against the
saved RTAB-Map database then runs Nav2 for autonomous navigation.

Stack launched
--------------
  1. amiga_ros2_bridge  — gRPC → /amiga/vel + /amiga/pose + cmd_vel relay
  2. amiga_odometry     — /amiga/vel → /wheel_odom
  3. Velodyne VLP-16    — /velodyne_points
  4. robot_state_publisher — URDF → base_link→velodyne TF
  5. icp_odometry       — /velodyne_points → /odom  +  odom→base_link TF
  6. rtabmap            — localization-only (Mem/IncrementalMemory=false)
                          publishes map→odom TF, /map OccupancyGrid
  7. Nav2               — controller, planner, behavior, bt_navigator,
                          waypoint_follower, lifecycle_manager
  8. RViz2              — optional (rviz:=true)

Prerequisites
-------------
  • Map built with slam_full.launch.py or slam_nav.launch.py and saved:
        bash scripts/save_map.sh ~/maps/field
  • Amiga reachable over Tailscale
  • VLP-16 reachable at 192.168.1.201:2368

Usage
-----
  ros2 launch vidalia_bringup localize_nav.launch.py \
      database_path:=~/maps/field.db

  ros2 launch vidalia_bringup localize_nav.launch.py \
      database_path:=~/maps/field.db rviz:=true

  # Send autonomous goal (once stack is running):
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}}"

  # Follow a list of waypoints (field row waypoints):
  ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
      "{poses: [...]}"
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

    # ---- 3. VLP-16 sensor driver ----
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # ---- 4–6. RTAB-Map localization (no new mapping) ----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'cloud_topic': '/velodyne_points',
            'database_path': database_path,
            'rviz': rviz,
        }.items(),
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
            description='Path to RTAB-Map database to localize against',
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
        localization,
        nav2,
    ])
