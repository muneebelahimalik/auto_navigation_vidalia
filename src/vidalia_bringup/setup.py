from setuptools import setup
import os
from glob import glob

package_name = 'vidalia_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muneebelahimalik',
    maintainer_email='59524535+muneebelahimalik@users.noreply.github.com',
    description='Bringup for Farm-ng Amiga SLAM stack (ROS2 Foxy)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'amiga_odometry = vidalia_bringup.amiga_odometry:main',
            'amiga_velocity_bridge = vidalia_bringup.amiga_velocity_bridge:main',
            'amiga_ros2_bridge = vidalia_bringup.amiga_ros2_bridge:main',
            'field_coverage_planner = vidalia_bringup.field_coverage_planner:main',
            'autonomous_row_coverage = vidalia_bringup.autonomous_row_coverage:main',
        ],
    },
)
