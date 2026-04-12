from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Physically measured mount position (2025-03-05).
    # Matches the joint origin in urdf/amiga_min.urdf.
    #   x = 1.130 m  (44.5 in from wheelbase midpoint to LiDAR spin axis, forward)
    #   y = 0.000 m  (centered on robot centreline)
    #   z = 0.800 m  (LiDAR spin axis height above ground)
    #   roll = pitch = yaw = 0  (mount is level, LiDAR front aligns with robot +x)
    #
    # ROS 2 Foxy: positional args  x y z yaw pitch roll frame_id child_frame_id
    # (named flags --x, --frame-id, etc. were added in Galactic/Humble)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_velodyne',
        output='screen',
        arguments=['1.130', '0', '0.80', '0', '0', '0', 'base_link', 'velodyne']
    )
    return LaunchDescription([static_tf])
