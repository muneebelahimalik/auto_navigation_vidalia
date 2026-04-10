#!/usr/bin/env bash
set -e
cd ~/auto_navigation_vidalia
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch vidalia_bringup sensors_live.launch.py
