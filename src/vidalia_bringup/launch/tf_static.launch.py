from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 2026-07 forward-facing re-mount: x = 0.85 m (1 ft ahead of the front-tire
    # centres, ~0.55 m ahead of base_link = half the ~1.10 m wheelbase),
    # z = 0.80 m (tape, robot on wheels).
    # NOTE: this RAW-cloud ROS stack needs the ~15° nose-down mount pitch in this
    # transform (pitch ≈ -0.262 rad) AND the velodyne driver set to VLP-16 Puck
    # Hi-Res calibration (this unit is Hi-Res). See tf_static_base_to_velodyne.launch.py.
    x = 0.85
    y = 0.0
    z = 0.80
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
