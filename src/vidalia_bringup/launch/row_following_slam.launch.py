"""
row_following_slam.launch.py — Locally reactive row follower + background SLAM.

The LiDAR center-row follower drives the robot reactively (no pre-built map).
RTAB-Map runs in the background, building the field map while the robot moves.
Nav2 is NOT used — the row follower owns /cmd_vel directly.

This is the intended architecture for autonomous onion-field operation:
  "locally reactive and globally aware"

What runs
---------
  1. amiga_ros2_bridge   — live gRPC connection to Amiga (vel + cmd_vel relay)
  2. amiga_odometry      — /amiga/vel → /wheel_odom
  3. Velodyne VLP-16     — /velodyne_points
  4. robot_state_publisher — URDF TF
  5. icp_odometry        — /velodyne_points → /odom + odom→base_link TF
  6. rtabmap             — active SLAM; map→odom TF; map grows while driving
  7. lidar_center_row_follower — row detection; publishes /cmd_vel

How to run
----------
  # Perception-only first (inspect RViz markers before enabling motion):
  ros2 launch vidalia_bringup row_following_slam.launch.py rviz:=true

  # Enable autonomous motion:
  ros2 launch vidalia_bringup row_following_slam.launch.py \\
      autonomous_mode:=true rviz:=true

  # Specify Amiga hostname (if not using default):
  ros2 launch vidalia_bringup row_following_slam.launch.py \\
      amiga_host:=camphor-clone.tail0be07.ts.net autonomous_mode:=true

  # Tune crop height for your field:
  ros2 launch vidalia_bringup row_following_slam.launch.py \\
      crop_height_min:=0.10 crop_height_max:=0.55 autonomous_mode:=true

Workflow
--------
  1. Place robot at start of first onion row, pointing along the row.
  2. Launch this file — wait for "[ACQUIRING→FOLLOWING]" in the console.
  3. Robot drives the row.  Watch /row/markers in RViz.
  4. At ROW_END, robot stops.  Execute headland turn manually or extend
     with a headland-turn module (future work).
  5. Save map when done:
       bash scripts/save_map.sh ~/maps/field

Emergency stop
--------------
  Ctrl-C  OR:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{}' --once

Recovery after obstacle
-----------------------
  The robot automatically resumes after the safety zone clears for
  obstacle_clear_secs (default 3 s).  Override the wait time:
    ros2 launch ... obstacle_clear_secs:=1.0
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
        # Amiga connectivity
        DeclareLaunchArgument('amiga_host',        default_value='camphor-clone.tail0be07.ts.net'),
        DeclareLaunchArgument('canbus_port',        default_value='6001'),
        DeclareLaunchArgument('filter_port',        default_value='20001'),

        # SLAM
        DeclareLaunchArgument('database_path',      default_value='',
                              description='RTAB-Map database path (empty = in-memory)'),

        # Row follower — motion
        DeclareLaunchArgument('autonomous_mode',    default_value='false',
                              description='Enable /cmd_vel output'),
        DeclareLaunchArgument('max_linear_speed',   default_value='0.25'),
        DeclareLaunchArgument('max_angular_speed',  default_value='0.40'),
        DeclareLaunchArgument('k_heading',          default_value='1.20'),
        DeclareLaunchArgument('k_lateral',          default_value='0.80'),

        # Row follower — perception
        DeclareLaunchArgument('roi_forward_min',    default_value='0.30'),
        DeclareLaunchArgument('roi_forward_max',    default_value='4.00'),
        DeclareLaunchArgument('roi_lateral_half',   default_value='0.35'),
        DeclareLaunchArgument('crop_height_min',    default_value='0.05'),
        DeclareLaunchArgument('crop_height_max',    default_value='0.45'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('row_end_min_scans',  default_value='100'),
        DeclareLaunchArgument('obstacle_clear_secs', default_value='3.0'),

        DeclareLaunchArgument('rviz',               default_value='false'),
    ]

    # ── 1–2. Amiga gRPC bridge + wheel odometry ───────────────────────────
    grpc_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'amiga_grpc_bridge.launch.py')
        ),
        launch_arguments={
            'host':        LaunchConfiguration('amiga_host'),
            'canbus_port': LaunchConfiguration('canbus_port'),
            'filter_port': LaunchConfiguration('filter_port'),
            'publish_unconverged_filter': 'false',
        }.items(),
    )

    # ── 3–4. VLP-16 + static TF + robot_state_publisher ──────────────────
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'tf_static_base_to_velodyne.launch.py')
        )
    )

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

    # ── 5. SLAM stack (ICP odometry + RTAB-Map) ───────────────────────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_rtabmap_lidar3d.launch.py')
        ),
        launch_arguments={
            'use_sim_time':   'false',
            'cloud_topic':    '/velodyne_points',
            'database_path':  LaunchConfiguration('database_path'),
        }.items(),
    )

    # ── 6. LiDAR center-row follower ──────────────────────────────────────
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
            'obstacle_clear_secs':   LaunchConfiguration('obstacle_clear_secs'),
        }],
    )

    # ── 7. Optional RViz2 ─────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg, 'rviz', 'slam_lidar.rviz')],
    )

    return LaunchDescription(
        args + [grpc_bridge, sensors, tf_static, rsp, slam, row_follower, rviz_node]
    )
