from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # TODO: replace xyz/rpy with measured LiDAR mount w.r.t base_link
    # ROS 2 Foxy: positional args  x y z yaw pitch roll frame_id child_frame_id
    base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([base_to_velodyne])
