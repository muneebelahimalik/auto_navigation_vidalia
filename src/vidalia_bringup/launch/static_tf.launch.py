from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Physically measured 2025: x=0.959 m (37.75 in forward), z=0.699 m (27.5 in up)
    # Matches amiga_min.urdf and tf_static_base_to_velodyne.launch.py
    base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=['0.959', '0', '0.699', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([base_to_velodyne])
