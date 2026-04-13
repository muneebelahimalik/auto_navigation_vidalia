#!/usr/bin/env bash
# run_live.sh — quick launch of sensors_live.launch.py (dev PC only).
#
# NOTE: ROS 2 Foxy is NOT installed on the Amiga brain.
#       Run this script only on a development PC with ROS 2 Foxy installed.

set -e

if [ ! -f /opt/ros/foxy/setup.bash ]; then
    echo "[run_live.sh] ERROR: ROS 2 Foxy not found."
    echo "              This script requires a dev PC with ROS 2 Foxy installed."
    echo "              On the Amiga brain, run:  python3 main.py"
    exit 1
fi

cd ~/auto_navigation_vidalia
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch vidalia_bringup sensors_live.launch.py
