from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')

    bag_path = LaunchConfiguration('bag_path')
    playback_rate = LaunchConfiguration('playback_rate')

    # Static TF: base_link -> velodyne
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'tf_static_base_to_velodyne.launch.py')
        )
    )

    database_path = LaunchConfiguration('database_path')

    # SLAM stack (use_sim_time=true to consume /clock from bag)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_rtabmap_lidar3d.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'cloud_topic': '/velodyne_points',
            'database_path': database_path,
        }.items(),
    )

    # Bag playback — delay slightly so SLAM nodes are ready before messages arrive
    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play',
                    bag_path,
                    '--clock',
                    '--rate', playback_rate,
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=os.path.join(
                os.path.expanduser('~'), 'amiga_slam', 'data', 'bags', 'vlp16_test'
            ),
            description='Absolute path to the ROS2 bag directory',
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Bag playback speed multiplier',
        ),
        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database (map saved here on exit)'),
        tf_static,
        slam,
        bag_play,
    ])
