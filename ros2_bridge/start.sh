#!/usr/bin/env bash
# start.sh — Build and launch the Vidalia ROS2 Foxy bridge on the Amiga brain.
#
# WHY DOCKER (not native apt install)
# ------------------------------------
# The Amiga OS uses an overlay filesystem.  /opt/ros/ is wiped on every reboot.
# /var/lib/docker is mounted on the NVMe and PERSISTS across reboots.
# Building this image once means it is always available — no reinstall needed.
#
# BASE IMAGE: dustynv/ros:foxy-ros-base-l4t-r35.2.1
# --------------------------------------------------
# L4T-optimised build for Jetson Xavier NX (L4T R35.2.1 / Jetpack 5.1).
# Matches the Amiga Brain kernel (linux-tegra 5.10.104, aarch64, Ubuntu 20.04).
# Uses --runtime nvidia to enable the Jetson NVIDIA container runtime.
# (Same approach as farm-ng's official amiga-ros-bridge Docker setup.)
#
# WHAT RUNS INSIDE
# ----------------
#   vidalia_node.py    reads /dev/shm/ written by row_follow.py --ros2-bridge,
#                      publishes /velodyne_points, /tf_static, /row_viz,
#                      /safety_viz, /cmd_vel at 10 Hz
#   foxglove_bridge    bridges all topics to WebSocket port 8765
#
# VISUALISATION (zero install on any other device)
# -------------------------------------------------
#   1. Open  https://app.foxglove.dev  in any browser
#   2. Click "Open connection" → "Rosbridge WebSocket"
#   3. URL:  ws://<amiga-tailscale-ip>:8765
#      e.g.  ws://100.66.121.56:8765
#
# USAGE
# -----
#   Terminal 1 (navigation):  python3 scripts/row_follow.py --auto --tire-height 0.85 --ros2-bridge
#   Terminal 2 (this script): bash ros2_bridge/start.sh
#   Browser:                  https://app.foxglove.dev  → ws://100.66.121.56:8765

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="vidalia-ros2-foxy"
CONTAINER_NAME="vidalia_bridge"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
FOXGLOVE_PORT=8765

echo "======================================================"
echo "  Vidalia ROS2 Bridge + Foxglove WebSocket Server"
echo "======================================================"
echo "  repo           : $REPO_ROOT"
echo "  image          : $IMAGE_NAME  (persists in /var/lib/docker on NVMe)"
echo "  ROS_DOMAIN_ID  : $ROS_DOMAIN_ID"
echo "  Foxglove port  : $FOXGLOVE_PORT"
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
echo "  >>> Open in any browser (no install on laptop/phone/tablet):"
echo "      https://app.foxglove.dev"
echo "      Connection type : Rosbridge WebSocket"
echo "      URL             : ws://$(hostname -I 2>/dev/null | awk '{print $1}' || echo '100.66.121.56'):${FOXGLOVE_PORT}"
echo ""
echo "  Press Ctrl+C to stop."
echo ""

docker run --rm \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --net=host \
    -v /dev/shm:/dev/shm \
    -p "${FOXGLOVE_PORT}:${FOXGLOVE_PORT}" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "$IMAGE_NAME"
