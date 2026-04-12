"""
autonomous_coverage.launch.py — All-in-one: sensors + SLAM + Nav2 + row coverage.

Use this for the first time in a new field (or any time you want to build a
map simultaneously with the weed-control pass).

What runs
---------
  1. amiga_ros2_bridge  — live gRPC connection to Amiga (velocity + cmd_vel)
  2. amiga_odometry     — /amiga/vel → /wheel_odom
  3. Velodyne VLP-16    — /velodyne_points
  4. robot_state_publisher — URDF TF
  5. icp_odometry       — /velodyne_points → /odom + odom→base_link TF
  6. rtabmap            — active SLAM; map grows as robot moves
  7. Nav2               — local planner, global planner, bt_navigator, etc.
  8. autonomous_row_coverage — waits for ~/start service call
  9. RViz2              — optional (rviz:=true)

How to run
----------
  # Known row length (most common — measure it once with a tape):
  ros2 launch vidalia_bringup autonomous_coverage.launch.py \
      row_length:=45.0 num_rows:=22 row_spacing:=0.45

  # Unknown row length — robot learns it on the first pass:
  ros2 launch vidalia_bringup autonomous_coverage.launch.py \
      row_length:=0.0 num_rows:=22

  # With RViz2 for monitoring:
  ros2 launch vidalia_bringup autonomous_coverage.launch.py \
      row_length:=45.0 num_rows:=22 rviz:=true

Workflow
--------
  Step 1  Drive the Amiga to the very start of row 0, pointing along the row.
  Step 2  Launch this file — wait until you see "AutonomousRowCoverage ready."
  Step 3  Trigger:
            ros2 service call /autonomous_row_coverage/start std_srvs/srv/Trigger {}
  Step 4  If row_length:=0, walk to the row end and call:
            ros2 service call /autonomous_row_coverage/mark_row_end std_srvs/srv/Trigger {}
  Step 5  Robot drives all rows autonomously. Watch progress in RViz2.
  Step 6  When done, save the map:
            bash scripts/save_map.sh ~/maps/field

  Emergency stop at any time:
    ros2 service call /autonomous_row_coverage/stop std_srvs/srv/Trigger {}

RViz2 displays to add
---------------------
  • Path   topic: /autonomous_row_coverage/coverage_path    — planned route
  • Int32  topic: /autonomous_row_coverage/current_row      — live row counter
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')

    # --- Coverage parameters (user-facing) ---
    row_length       = LaunchConfiguration('row_length')
    num_rows         = LaunchConfiguration('num_rows')
    row_spacing      = LaunchConfiguration('row_spacing')
    buffer_distance  = LaunchConfiguration('buffer_distance')

    # --- LiDAR detection tuning ---
    row_end_threshold     = LaunchConfiguration('row_end_threshold')
    row_end_confirm_scans = LaunchConfiguration('row_end_confirm_scans')
    row_end_min_dist      = LaunchConfiguration('row_end_min_dist')
    obstacle_stop_dist    = LaunchConfiguration('obstacle_stop_dist')
    obstacle_clear_secs   = LaunchConfiguration('obstacle_clear_secs')
    obstacle_threshold    = LaunchConfiguration('obstacle_threshold')

    # --- Common parameters ---
    rviz             = LaunchConfiguration('rviz')
    database_path    = LaunchConfiguration('database_path')
    amiga_host       = LaunchConfiguration('amiga_host')
    canbus_port      = LaunchConfiguration('canbus_port')
    filter_port      = LaunchConfiguration('filter_port')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

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
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

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

    # ---- 7. Nav2 ----
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
        }.items(),
    )

    # ---- 8. Autonomous row coverage node ----
    coverage_node = Node(
        package='vidalia_bringup',
        executable='autonomous_row_coverage',
        name='autonomous_row_coverage',
        output='screen',
        parameters=[{
            'row_length':             row_length,
            'num_rows':               num_rows,
            'row_spacing':            row_spacing,
            'buffer_distance':        buffer_distance,
            'row_end_threshold':      row_end_threshold,
            'row_end_confirm_scans':  row_end_confirm_scans,
            'row_end_min_dist':       row_end_min_dist,
            'obstacle_stop_dist':     obstacle_stop_dist,
            'obstacle_clear_secs':    obstacle_clear_secs,
            'obstacle_threshold':     obstacle_threshold,
            'nav_frame':              'map',
            'base_frame':             'base_link',
        }],
    )

    return LaunchDescription([
        # Coverage parameters
        DeclareLaunchArgument(
            'row_length', default_value='0.0',
            description='Row length in metres. 0 = learn from ~/mark_row_end service.',
        ),
        DeclareLaunchArgument(
            'num_rows', default_value='20',
            description='Number of rows to cover.',
        ),
        DeclareLaunchArgument(
            'row_spacing', default_value='0.45',
            description='Distance between row centres (m). Default: 0.45 (2× onion row pitch).',
        ),
        DeclareLaunchArgument(
            'buffer_distance', default_value='1.5',
            description='Metres past crop edge to drive before turning.',
        ),
        # LiDAR detection tuning
        DeclareLaunchArgument(
            'row_end_threshold', default_value='5',
            description='LiDAR returns/scan below this threshold declares row end.',
        ),
        DeclareLaunchArgument(
            'row_end_confirm_scans', default_value='4',
            description='Consecutive low-density scans required to confirm row end.',
        ),
        DeclareLaunchArgument(
            'row_end_min_dist', default_value='3.0',
            description='Min metres driven before row-end detector activates.',
        ),
        DeclareLaunchArgument(
            'obstacle_stop_dist', default_value='1.5',
            description='Depth of forward safety zone (m) — robot stops when occupied.',
        ),
        DeclareLaunchArgument(
            'obstacle_clear_secs', default_value='3.0',
            description='Seconds safety zone must be clear before resuming after obstacle.',
        ),
        DeclareLaunchArgument(
            'obstacle_threshold', default_value='3',
            description='LiDAR points in safety zone counted as obstacle present.',
        ),
        # Stack parameters
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz2.',
        ),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='RTAB-Map database path (map written here).',
        ),
        DeclareLaunchArgument(
            'amiga_host', default_value='camphor-clone.tail0be07.ts.net',
            description='Amiga Tailscale hostname or IP.',
        ),
        DeclareLaunchArgument(
            'canbus_port', default_value='6001',
            description='Amiga canbus gRPC port.',
        ),
        DeclareLaunchArgument(
            'filter_port', default_value='20001',
            description='Amiga filter gRPC port.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                get_package_share_directory('vidalia_bringup'),
                'config', 'nav2_params.yaml',
            ),
            description='Nav2 params file.',
        ),
        # Nodes
        grpc_bridge,
        amiga_odom,
        sensors,
        slam,
        nav2,
        coverage_node,
    ])
