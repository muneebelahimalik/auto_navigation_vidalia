#!/usr/bin/env bash
# entrypoint.sh — start the Vidalia ROS2 bridge + Foxglove WebSocket server.
#
# Two processes run inside the container:
#   1. vidalia_node.py   — reads /dev/shm/, publishes ROS2 topics
#   2. foxglove_bridge   — bridges all ROS2 topics to WebSocket port 8765
#
# Connect from any browser: https://app.foxglove.dev
#   → Open connection → Rosbridge WebSocket → ws://<amiga-ip>:8765

set -euo pipefail

source /opt/ros/foxy/setup.bash

# Start our navigation bridge node in the background
python3 /workspace/vidalia_node.py &
BRIDGE_PID=$!

# Give vidalia_node a moment to initialise before foxglove connects
sleep 1

# Start Foxglove WebSocket bridge on port 8765, accessible on all interfaces
ros2 launch foxglove_bridge foxglove_bridge.launch.xml \
    port:=8765 \
    address:=0.0.0.0 \
    tls:=false &
FOXGLOVE_PID=$!

echo ""
echo "=== Vidalia ROS2 bridge running ==="
echo "  vidalia_node pid    : $BRIDGE_PID"
echo "  foxglove_bridge pid : $FOXGLOVE_PID"
echo ""
echo "  Visualise from any browser (no install):"
echo "    1. Open  https://app.foxglove.dev"
echo "    2. Click 'Open connection' → 'Rosbridge WebSocket'"
echo "    3. URL:  ws://$(hostname -I | awk '{print $1}'):8765"
echo "  or over Tailscale: ws://100.66.121.56:8765"
echo ""

# Wait for either process to exit (Ctrl+C kills both via trap)
trap "kill $BRIDGE_PID $FOXGLOVE_PID 2>/dev/null" SIGTERM SIGINT
wait $BRIDGE_PID $FOXGLOVE_PID
