"""
slam_localization.launch.py -- localise against a previously built RTAB-Map database.

Usage:
    ros2 launch vidalia_bringup slam_localization.launch.py \\
        database_path:=/path/to/map.db

This launches the full sensor stack (VLP-16 + static TF) plus RTAB-Map in
localization-only mode (Mem/IncrementalMemory=false).  The map is NOT extended;
only the robot's pose within the saved map is estimated.

To build a map first, run:
    ros2 launch vidalia_bringup slam_live.launch.py
Then save the map with:
    bash ~/auto_navigation_vidalia/scripts/save_map.sh

ROS 2 Foxy note: RTAB-Map packages are all under rtabmap_ros
  (not the split rtabmap_slam / rtabmap_odom / rtabmap_viz used in Humble).
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    database_path = LaunchConfiguration('database_path')

    urdf_file = os.path.join(pkg, 'urdf', 'amiga_min.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ---- Sensors ----
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # ---- Robot state publisher (URDF TF) ----
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ---- ICP Odometry (ROS 2 Foxy: package=rtabmap_ros) ----
    icp_odom = Node(
        package='rtabmap_ros',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.2',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/MaxTranslation': '2.0',
            'Icp/MaxRotation': '0.78',
            'Icp/PM': 'true',
            'Icp/PMOutlierRatio': '0.85',
            'Icp/CorrespondenceRatio': '0.01',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '5',
            'Icp/PointToPlaneRadius': '0.0',
            'Odom/Strategy': '0',
            'OdomF2M/MaxSize': '20000',
        }],
        remappings=[('scan_cloud', cloud_topic)],
    )

    # ---- RTAB-Map in localization mode (ROS 2 Foxy: package=rtabmap_ros) ----
    rtabmap = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            'subscribe_scan_cloud': True,
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_stereo': False,
            'database_path': database_path,
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.3',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/MaxTranslation': '3.0',
            'Icp/MaxRotation': '1.57',
            'Icp/PM': 'true',
            'Icp/PMOutlierRatio': '0.85',
            'Icp/CorrespondenceRatio': '0.01',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '5',
            'Icp/PointToPlaneRadius': '0.0',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '2',
            'Kp/MaxFeatures': '400',
            'Vis/MinInliers': '5',
            'Grid/CellSize': '0.1',
            'Grid/RangeMin': '0.5',
            'Grid/RangeMax': '20.0',
            'Grid/3D': 'true',
            'Grid/NormalsSegmentation': 'true',
            'Grid/MaxObstacleHeight': '2.0',
            'Grid/GroundIsObstacle': 'false',
            'Optimizer/Strategy': '1',
            'Optimizer/Robust': 'true',
        }],
        remappings=[('scan_cloud', cloud_topic)],
    )

    # ---- rtabmap_viz (ROS 2 Foxy: package=rtabmap_ros, executable=rtabmapviz) ----
    rtabmap_viz = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_scan_cloud': False,
            'subscribe_rgb': False,
            'subscribe_depth': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Path to saved RTAB-Map database to localise against'),
        sensors,
        robot_state_pub,
        icp_odom,
        rtabmap,
        rtabmap_viz,
    ])
