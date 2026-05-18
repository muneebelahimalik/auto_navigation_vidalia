from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Physically measured 2025: 37.75 in forward, 27.5 in up
    # Matches amiga_min.urdf and tf_static_base_to_velodyne.launch.py
    x = 0.959
    y = 0.0
    z = 0.699
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # ROS 2 Foxy: positional args  x y z yaw pitch roll frame_id child_frame_id
    # (named flags --x, --frame-id, etc. were added in Galactic/Humble)
    base_to_velodyne = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_velodyne_tf",
        output="screen",
        arguments=[
            str(x), str(y), str(z), str(yaw), str(pitch), str(roll),
            'base_link', 'velodyne',
        ],
    )

    return LaunchDescription([base_to_velodyne])
