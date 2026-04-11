from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('vidalia_bringup')

    # VLP-16 driver + pointcloud transform
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'velodyne_vlp16.launch.py')
        )
    )

    # Static TF: base_link -> velodyne
    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'tf_static_base_to_velodyne.launch.py')
        )
    )

    # SLAM stack (use_sim_time=false for live data)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'slam_rtabmap_lidar3d.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'cloud_topic': '/velodyne_points',
        }.items(),
    )

    return LaunchDescription([sensors, tf_static, slam])
