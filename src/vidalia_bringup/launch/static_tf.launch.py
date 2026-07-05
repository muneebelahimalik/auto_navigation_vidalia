from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 2026-07 forward-facing re-mount: z=0.800 m (tape, robot on wheels).
    # See tf_static_base_to_velodyne.launch.py — for this RAW-cloud ROS stack the
    # ~15° nose-down mount pitch must go in this transform, and the velodyne
    # driver needs the VLP-16 Puck Hi-Res calibration (unit is Hi-Res). The
    # forward x offset changed with the re-mount and must be re-measured.
    base_to_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_velodyne_tf',
        output='screen',
        arguments=['0.959', '0', '0.800', '0', '0', '0', 'base_link', 'velodyne']
    )

    return LaunchDescription([base_to_velodyne])
