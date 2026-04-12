from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO: replace xyz/rpy with measured LiDAR mount w.r.t base_link
    base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([base_to_velodyne])
