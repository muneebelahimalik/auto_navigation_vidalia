"""
nav2.launch.py — Nav2 navigation stack for the Amiga robot.

Launches all Nav2 nodes against a running localization/SLAM stack.
RTAB-Map must already be running and publishing:
  - /map            (nav_msgs/OccupancyGrid)
  - map→odom TF     (from rtabmap)
  - odom→base_link TF  (from icp_odometry or EKF)

This file launches:
  • controller_server   — local costmap + DWB path follower
  • planner_server      — global costmap + NavFn path planner
  • behavior_server     — spin / back-up / wait recoveries
  • bt_navigator        — Navigate-to-Pose / Navigate-Through-Poses
  • waypoint_follower   — ordered waypoint execution
  • lifecycle_manager   — activates all Nav2 lifecycle nodes

Usage
-----
  # With slam_full.launch.py already running in another terminal:
  ros2 launch vidalia_bringup nav2.launch.py

  # Custom params:
  ros2 launch vidalia_bringup nav2.launch.py \
      params_file:=/path/to/nav2_params.yaml

  # Send a single goal from RViz2 (Nav2 Goal tool), or via CLI:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0}}}}"

  # Send a list of waypoints:
  ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
      "{poses: [{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0}}},
                {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Nav2 lifecycle nodes ----

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ---- Lifecycle manager: activates all Nav2 nodes automatically ----
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to Nav2 params YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (true for bag replay)',
        ),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
