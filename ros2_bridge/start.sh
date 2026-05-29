#!/usr/bin/env bash
# start.sh — Build and launch the Vidalia ROS2 Foxy + RViz2 bridge on the Amiga brain.
#
# WHY DOCKER (not native apt install)
# ------------------------------------
# The Amiga OS uses an overlay filesystem.  /opt/ros/ is wiped on every reboot.
# /var/lib/docker is mounted on the NVMe and PERSISTS across reboots.
# Building this image once means it is always available — no reinstall needed.
#
# BASE IMAGE: dustynv/ros:foxy-desktop-l4t-r35.2.1
# -------------------------------------------------
# L4T-optimised build for Jetson Xavier NX (L4T R35.2.1 / Jetpack 5.1).
# "desktop" variant includes RViz2, rqt, and all visualisation plugins.
# Matches the Amiga Brain kernel (linux-tegra 5.10.104, aarch64, Ubuntu 20.04).
# Uses --runtime nvidia to enable the Jetson NVIDIA container runtime.
# (Same approach as farm-ng's official amiga-ros-bridge Docker setup.)
#
# WHAT RUNS INSIDE
# ----------------
#   vidalia_node.py    reads /dev/shm/ written by row_follow.py --ros2-bridge,
#                      publishes /velodyne_points, /tf_static, /row_viz,
#                      /safety_viz, /cmd_vel at 12 Hz
#   RViz2              reads all topics, renders to virtual display (Xvfb)
#   noVNC              serves RViz2 in browser on port 6080
#
# VISUALISATION (zero install — just a browser tab)
# -------------------------------------------------
#   VS Code SSH automatically port-forwards 6080 to your laptop.
#   Open:  http://localhost:6080/vnc.html
#   Or via Tailscale: http://100.66.121.56:6080/vnc.html
#
# USAGE
# -----
#   Terminal 1 (navigation):  python3 scripts/row_follow.py --auto --tire-height 0.85 --ros2-bridge
#   Terminal 2 (this script): bash ros2_bridge/start.sh
#   Laptop browser:           http://localhost:6080/vnc.html

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="vidalia-ros2-foxy"
CONTAINER_NAME="vidalia_bridge"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
VNC_WEB_PORT=6080

echo "======================================================"
echo "  Vidalia ROS2 Bridge + RViz2 (noVNC browser access)"
echo "======================================================"
echo "  repo          : $REPO_ROOT"
echo "  image         : $IMAGE_NAME  (persists in /var/lib/docker on NVMe)"
echo "  ROS_DOMAIN_ID : $ROS_DOMAIN_ID"
echo "  Browser port  : $VNC_WEB_PORT"
echo ""

# --------------------------------------------------------------------------
# 1. Build the Docker image (cached — only rebuilds if Dockerfile changed)
# --------------------------------------------------------------------------
echo "[1/3] Building Docker image $IMAGE_NAME …"
docker build -t "$IMAGE_NAME" "$REPO_ROOT/ros2_bridge/"
echo "      Image built and stored in /var/lib/docker (NVMe — survives reboot)."

# --------------------------------------------------------------------------
# 2. Stop any previous bridge container
# --------------------------------------------------------------------------
echo "[2/3] Stopping any previous bridge container …"
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# --------------------------------------------------------------------------
# 3. Run the bridge
# --------------------------------------------------------------------------
echo "[3/3] Starting bridge …"
echo ""
echo "  >>> VS Code SSH automatically forwards port $VNC_WEB_PORT to your laptop."
echo "      Open in laptop browser: http://localhost:$VNC_WEB_PORT/vnc.html"
echo "      (Or via Tailscale)    : http://100.66.121.56:$VNC_WEB_PORT/vnc.html"
echo ""
echo "  Press Ctrl+C to stop."
echo ""

docker run --rm \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --net=host \
    -v /dev/shm:/dev/shm \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "$IMAGE_NAME"
