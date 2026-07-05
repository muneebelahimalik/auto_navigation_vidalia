from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Physically measured mount position (2026-07 forward-facing re-mount).
#   y = 0.000 m  (centered on robot centreline)
#   z = 0.800 m  (LiDAR spin-axis height above ground; tape, robot on wheels)
    #
    # NOTE: this dev-PC ROS 2 stack consumes the RAW velodyne cloud, so — unlike
    # the native brain stack, where row_navigator corrects the cloud — the mount
    # PITCH must live in THIS transform.  The mount is ~15° nose-down
    # (pitch ≈ -0.262 rad); set the pitch arg accordingly AND configure the
    # velodyne_pointcloud driver for the *VLP-16 Puck Hi-Res* calibration (this
    # unit is Hi-Res, product-ID 0x24 — see lidar/lidar_driver.py).  The forward
    # x offset changed with the re-mount and must be re-measured (was 0.959 m).
    #
    # ROS 2 Foxy: positional args  x y z yaw pitch roll frame_id child_frame_id
    # (named flags --x, --frame-id, etc. were added in Galactic/Humble)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_velodyne',
        output='screen',
        arguments=['0.959', '0', '0.800', '0', '0', '0', 'base_link', 'velodyne']
    )
    return LaunchDescription([static_tf])
