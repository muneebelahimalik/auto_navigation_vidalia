#!/usr/bin/env bash
# env.sh — source the correct environment for the auto_navigation_vidalia workspace.
#
# On the Amiga brain (camphor-clone) ROS 2 Foxy is NOT installed on the base
# image and is wiped on reboot (overlay filesystem).  This script handles both:
#   1. Native farm-ng Python stack  (always available on the brain)
#   2. ROS 2 Foxy workspace          (only if /opt/ros/foxy exists — dev PC)
#
# Usage:
#   source scripts/env.sh

# ---- farm-ng venv (Amiga brain) -------------------------------------------
if [ -f /farm_ng_image/venv/bin/activate ]; then
    source /farm_ng_image/venv/bin/activate
    echo "[env] Activated farm-ng venv: /farm_ng_image/venv"
fi

# ---- ROS 2 Foxy (development PC — not present on Amiga brain) --------------
if [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
    # Source the colcon workspace overlay if it has been built
    if [ -f ~/auto_navigation_vidalia/install/setup.bash ]; then
        source ~/auto_navigation_vidalia/install/setup.bash
        echo "[env] Sourced ROS 2 Foxy + workspace overlay"
    else
        echo "[env] Sourced ROS 2 Foxy (workspace not yet built — run scripts/build.sh)"
    fi
else
    echo "[env] ROS 2 Foxy not found — using native farm-ng Python stack only"
    echo "      To run the native stack:  python3 main.py"
fi
