#!/usr/bin/env bash
set -e
cd ~/auto_navigation_vidalia
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
echo "Built and sourced workspace."
