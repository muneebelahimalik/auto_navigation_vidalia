from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vidalia_bringup')
    transform_params = os.path.join(pkg_share, 'config', 'velodyne_transform.yaml')

    driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[{
            'device_ip': '192.168.1.201',
            'udp_port': 2368,
            'frame_id': 'velodyne',
            'model': 'VLP16',
        }]
    )

    transform = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        name='velodyne_transform_node',
        output='screen',
        parameters=[transform_params]
    )

    return LaunchDescription([driver, transform])
