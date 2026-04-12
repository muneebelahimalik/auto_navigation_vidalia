from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Update these once you physically mount the VLP-16 (meters, radians)
    x = 0.0
    y = 0.0
    z = 0.0
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
