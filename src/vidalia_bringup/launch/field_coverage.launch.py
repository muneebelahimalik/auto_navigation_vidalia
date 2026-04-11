"""
field_coverage.launch.py -- Launch the boustrophedon field coverage planner.

This node attaches to a running Nav2 stack (localize_nav.launch.py or
slam_nav.launch.py) and waits to be triggered.

Prerequisites
-------------
  * localize_nav.launch.py (or slam_nav.launch.py) running in another terminal
  * field_corners configured in config/field_coverage.yaml to match the actual field

Usage
-----
  # 1. Start the full autonomous stack:
  ros2 launch vidalia_bringup localize_nav.launch.py database_path:=~/maps/field.db

  # 2. In a second terminal, start the coverage planner:
  ros2 launch vidalia_bringup field_coverage.launch.py

  # 3. Trigger autonomous coverage:
  ros2 service call /field_coverage_planner/start_coverage std_srvs/srv/Trigger {}

  # 4. Watch the planned path in RViz2 (topic: /field_coverage_planner/coverage_path)
  #    Add a Path display subscribed to /field_coverage_planner/coverage_path

  # 5. Cancel at any time:
  ros2 service call /field_coverage_planner/stop_coverage std_srvs/srv/Trigger {}

  # Override field params on the command line (for quick testing):
  ros2 launch vidalia_bringup field_coverage.launch.py \\
      row_spacing:=0.45 row_heading:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')
    default_params = os.path.join(pkg, 'config', 'field_coverage.yaml')

    params_file = LaunchConfiguration('params_file')
    row_spacing = LaunchConfiguration('row_spacing')
    row_heading = LaunchConfiguration('row_heading')

    coverage_node = Node(
        package='vidalia_bringup',
        executable='field_coverage_planner',
        name='field_coverage_planner',
        output='screen',
        parameters=[
            params_file,
            {
                'row_spacing': row_spacing,
                'row_heading': row_heading,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to field_coverage.yaml params file',
        ),
        DeclareLaunchArgument(
            'row_spacing',
            default_value='0.45',
            description='Distance between parallel passes (m)',
        ),
        DeclareLaunchArgument(
            'row_heading',
            default_value='0.0',
            description='Row travel direction in degrees from map +X axis',
        ),
        coverage_node,
    ])
