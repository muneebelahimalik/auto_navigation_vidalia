from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 2026-07 forward-facing re-mount: x=0.85 m (1 ft ahead of front-tire centres,
    # ~0.55 m ahead of base_link), z=0.800 m (tape, robot on wheels).
    # See tf_static_base_to_velodyne.launch.py — for this RAW-cloud ROS stack the
    # ~15° nose-down mount pitch must go in this transform, and the velodyne
    # driver needs the VLP-16 Puck Hi-Res calibration (unit is Hi-Res).
    base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=['0.85', '0', '0.800', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([base_to_velodyne])
