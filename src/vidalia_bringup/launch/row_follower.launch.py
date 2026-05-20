"""
row_follower.launch.py — Sensors + LiDAR center-row follower (no Nav2, no SLAM).

Use this for warehouse validation before full-field deployment.
The robot drives reactively: live LiDAR row detection → /cmd_vel.
No pre-built map required.

What runs
---------
  1. Velodyne VLP-16 driver  — /velodyne_points
  2. Static TF base_link → velodyne
  3. robot_state_publisher   — URDF joint states / TF
  4. lidar_center_row_follower — row detection + optional /cmd_vel

How to run
----------
  # Perception-only (inspect RViz before enabling motion):
  ros2 launch vidalia_bringup row_follower.launch.py

  # Enable autonomous motion (slow, for warehouse validation):
  ros2 launch vidalia_bringup row_follower.launch.py autonomous_mode:=true

  # With RViz2:
  ros2 launch vidalia_bringup row_follower.launch.py rviz:=true

  # Tune the crop-height band for your cardboard height (~15 cm):
  ros2 launch vidalia_bringup row_follower.launch.py \\
      crop_height_min:=0.05 crop_height_max:=0.25

RViz displays to add
---------------------
  MarkerArray  /row/markers       — crop points, row line, target, confidence text
  String       /row/state         — ACQUIRING / FOLLOWING / ROW_END / OBSTACLE_WAIT
  Float64      /row/confidence    — 0.0 (nothing) … 1.0 (confident row)
  Float64      /row/lateral_offset — metres, + = row is to the left
  Float64      /row/heading_error  — degrees recommended; + = steer left

Emergency stop
--------------
  Ctrl-C  OR  publish a zero-velocity /cmd_vel from another terminal:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{}' --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')

    # ── Declare arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('autonomous_mode',        default_value='false',
                              description='Enable /cmd_vel output (false = perception-only)'),
        DeclareLaunchArgument('max_linear_speed',       default_value='0.25',
                              description='Max forward speed in autonomous mode (m/s)'),
        DeclareLaunchArgument('max_angular_speed',      default_value='0.40',
                              description='Max yaw rate (rad/s)'),
        DeclareLaunchArgument('k_heading',              default_value='1.20',
                              description='Heading-error P gain'),
        DeclareLaunchArgument('k_lateral',              default_value='0.80',
                              description='Lateral-offset P gain'),

        DeclareLaunchArgument('roi_forward_min',        default_value='0.30',
                              description='Crop ROI start distance (m, from robot centre)'),
        DeclareLaunchArgument('roi_forward_max',        default_value='4.00',
                              description='Crop ROI end distance (m, from robot centre)'),
        DeclareLaunchArgument('roi_lateral_half',       default_value='0.35',
                              description='Half-width of center crop zone (m)'),

        DeclareLaunchArgument('crop_height_min',        default_value='0.05',
                              description='Minimum crop/cardboard height above ground (m)'),
        DeclareLaunchArgument('crop_height_max',        default_value='0.45',
                              description='Maximum crop/cardboard height above ground (m)'),

        DeclareLaunchArgument('confidence_threshold',   default_value='0.35',
                              description='Row confidence [0,1] required to enter FOLLOWING'),
        DeclareLaunchArgument('row_end_min_scans',      default_value='100',
                              description='Scans in FOLLOWING before ROW_END can trigger (~3 m at 0.25 m/s)'),

        DeclareLaunchArgument('rviz',                   default_value='false',
                              description='Launch RViz2 for monitoring'),
    ]

    # ── 1. VLP-16 driver ──────────────────────────────────────────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # ── 2. Static TF: base_link → velodyne ───────────────────────────────
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'tf_static_base_to_velodyne.launch.py')
        )
    )

    # ── 3. robot_state_publisher (URDF TF) ────────────────────────────────
    urdf_path = os.path.join(pkg, 'urdf', 'amiga_min.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # ── 4. LiDAR center-row follower ──────────────────────────────────────
    row_follower = Node(
        package='vidalia_bringup',
        executable='lidar_center_row_follower',
        name='lidar_center_row_follower',
        output='screen',
        parameters=[{
            'autonomous_mode':       LaunchConfiguration('autonomous_mode'),
            'max_linear_speed':      LaunchConfiguration('max_linear_speed'),
            'max_angular_speed':     LaunchConfiguration('max_angular_speed'),
            'k_heading':             LaunchConfiguration('k_heading'),
            'k_lateral':             LaunchConfiguration('k_lateral'),
            'roi_forward_min':       LaunchConfiguration('roi_forward_min'),
            'roi_forward_max':       LaunchConfiguration('roi_forward_max'),
            'roi_lateral_half':      LaunchConfiguration('roi_lateral_half'),
            'crop_height_min':       LaunchConfiguration('crop_height_min'),
            'crop_height_max':       LaunchConfiguration('crop_height_max'),
            'confidence_threshold':  LaunchConfiguration('confidence_threshold'),
            'row_end_min_scans':     LaunchConfiguration('row_end_min_scans'),
        }],
    )

    # ── 5. Optional RViz2 ─────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'slam_lidar.rviz')],
    )

    return LaunchDescription(args + [sensors, tf_static, rsp, row_follower, rviz_node])
