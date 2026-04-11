import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    database_path = LaunchConfiguration('database_path')
    rviz = LaunchConfiguration('rviz')

    pkg = get_package_share_directory('vidalia_bringup')
    urdf_file = os.path.join(pkg, 'urdf', 'amiga_min.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    rviz_cfg = os.path.join(pkg, 'rviz', 'slam_lidar.rviz')

    # ---------------------------------------------------------------------------
    # robot_state_publisher -- broadcasts fixed joint TFs from URDF
    # ---------------------------------------------------------------------------
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

    # ---------------------------------------------------------------------------
    # ICP Odometry -- estimates odom -> base_link from consecutive LiDAR scans.
    # ROS 2 Foxy: package is rtabmap_ros (not rtabmap_odom)
    # ---------------------------------------------------------------------------
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

    # ---------------------------------------------------------------------------
    # RTAB-Map SLAM (mapping mode)
    # ROS 2 Foxy: package is rtabmap_ros (not rtabmap_slam)
    # ---------------------------------------------------------------------------
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
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '2',
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
            'Mem/STMSize': '30',
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'false',
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

    # ---------------------------------------------------------------------------
    # rtabmap_viz -- visualises rtabmap output only
    # ROS 2 Foxy: package is rtabmap_ros (not rtabmap_viz)
    # ---------------------------------------------------------------------------
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database (map is saved here on exit)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Set true to launch RViz2 with the SLAM display config'),
        robot_state_pub,
        icp_odom,
        rtabmap,
        rtabmap_viz,
        rviz_node,
    ])
