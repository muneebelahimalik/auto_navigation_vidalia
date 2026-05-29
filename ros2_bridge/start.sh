#!/usr/bin/env bash
# start.sh — Build and launch the Vidalia ROS2 Foxy bridge container on the Amiga brain.
#
# The bridge reads scan data written by row_follow.py --ros2-bridge, then
# publishes /velodyne_points, /row_viz, /safety_viz, /cmd_vel, and /tf_static.
#
# On the LAPTOP, run:
#   export ROS_DOMAIN_ID=42
#   rviz2 -d ~/auto_navigation_vidalia/ros2_bridge/rviz/vidalia.rviz
#
# Or install ROS2 Humble on Ubuntu 22.04 dev PC; set same ROS_DOMAIN_ID.
#
# Prerequisites:
#   - docker is installed and running    (docker --version)
#   - row_follow.py is running with      --ros2-bridge
#   - /dev/shm/vidalia_pts.bin exists    (created by --ros2-bridge flag)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_NAME="vidalia-ros2-foxy"
CONTAINER_NAME="vidalia_bridge"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

echo "=== Vidalia ROS2 Bridge ==="
echo "  repo: $REPO_ROOT"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# --------------------------------------------------------------------------
# Build the Docker image (only rebuilds if Dockerfile changed)
# --------------------------------------------------------------------------
echo "[1/3] Building Docker image $IMAGE_NAME …"
docker build -t "$IMAGE_NAME" "$REPO_ROOT/ros2_bridge/"
echo "      Done."

# --------------------------------------------------------------------------
# Stop any previous bridge container
# --------------------------------------------------------------------------
echo "[2/3] Stopping any previous bridge container …"
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

# --------------------------------------------------------------------------
# Run the bridge
# --------------------------------------------------------------------------
echo "[3/3] Starting bridge container (host networking, /dev/shm shared) …"
echo ""
echo "  Topics will appear on ROS_DOMAIN_ID=$ROS_DOMAIN_ID:"
echo "    /velodyne_points    PointCloud2 @ 10 Hz"
echo "    /tf_static          base_link → velodyne (15° tilt, 0.959m fwd, 0.699m up)"
echo "    /row_viz            MarkerArray (row direction + lateral offset)"
echo "    /safety_viz         MarkerArray (forward + tire zones, green/red)"
echo "    /cmd_vel            Twist (commanded velocity)"
echo ""
echo "  Press Ctrl+C to stop."
echo ""

docker run --rm \
    --name "$CONTAINER_NAME" \
    --net=host \
    -v /dev/shm:/dev/shm \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "$IMAGE_NAME"
