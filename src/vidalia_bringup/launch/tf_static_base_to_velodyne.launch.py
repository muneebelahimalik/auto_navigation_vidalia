from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Physically measured mount position.
#   x = 0.959 m  (37.75 in from robot body center to LiDAR spin axis, forward)
#   y = 0.000 m  (centered on robot centreline)
#   z = 0.699 m  (27.5 in LiDAR spin axis height above ground)
#   roll = pitch = yaw = 0  (if mount is level and aligned with robot forward)
    #
    # ROS 2 Foxy: positional args  x y z yaw pitch roll frame_id child_frame_id
    # (named flags --x, --frame-id, etc. were added in Galactic/Humble)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_velodyne',
        output='screen',
        arguments=['0.959', '0', '0.699', '0', '0', '0', 'base_link', 'velodyne']
    )
    return LaunchDescription([static_tf])
