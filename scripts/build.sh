#!/usr/bin/env bash
# build.sh — build the vidalia_bringup ROS 2 workspace (development PC only).
#
# NOTE: ROS 2 Foxy is NOT installed on the Amiga brain (camphor-clone).
#       The overlay filesystem resets /opt/ros/ on every reboot.
#       Run this script only on a development PC with ROS 2 Foxy installed.
#
#       On the Amiga brain, use the native Python stack instead:
#           bash scripts/install_farmng.sh   # one-time setup
#           python3 main.py                  # run

set -e

cd ~/auto_navigation_vidalia

if [ ! -f /opt/ros/foxy/setup.bash ]; then
    echo "[build.sh] ERROR: ROS 2 Foxy not found at /opt/ros/foxy/setup.bash"
    echo "           This script is intended for a development PC with ROS 2 Foxy installed."
    echo ""
    echo "           On the Amiga brain, use the native Python stack:"
    echo "               bash scripts/install_farmng.sh"
    echo "               python3 main.py"
    exit 1
fi

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "[build.sh] Built and sourced workspace."
