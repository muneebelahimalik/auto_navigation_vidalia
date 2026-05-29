#!/usr/bin/env bash
# entrypoint.sh — start the Vidalia ROS2 bridge + RViz2 via virtual display.
#
# Stack inside the container:
#   1. Xvfb        — virtual framebuffer display :99
#   2. vidalia_node.py  — reads /dev/shm/, publishes ROS2 topics
#   3. RViz2       — reads all topics, renders 3-D view on :99
#   4. x11vnc      — serves display :99 as VNC on port 5900
#   5. websockify  — wraps VNC in WebSocket; noVNC serves browser UI on 6080
#
# Access from laptop (VS Code SSH auto-forwards port 6080):
#   http://localhost:6080/vnc.html

set -eo pipefail

# ROS setup scripts reference optional env vars (e.g. AMENT_TRACE_SETUP_FILES)
# that are not always exported; -u (unbound variable) would abort the script.
source /opt/ros/foxy/setup.bash

export DISPLAY=:99
# Software rendering — RViz2 in Xvfb has no GPU, must use llvmpipe/Mesa.
export LIBGL_ALWAYS_SOFTWARE=1

echo "[1/5] Starting virtual display (Xvfb :99, 1920x1080x24)..."
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
XVFB_PID=$!
sleep 1

echo "[2/5] Starting vidalia_node (reads /dev/shm/, publishes ROS2 topics)..."
python3 /workspace/vidalia_node.py &
NODE_PID=$!
sleep 2

echo "[3/5] Starting RViz2..."
rviz2 -d /workspace/vidalia.rviz &
RVIZ_PID=$!
sleep 2

echo "[4/5] Starting VNC server (x11vnc :99 → port 5900)..."
x11vnc -display :99 -forever -nopw -rfbport 5900 -quiet &
VNC_PID=$!
sleep 1

echo "[5/5] Starting noVNC web server on port 6080..."
websockify --web=/usr/share/novnc/ --wrap-mode=ignore 6080 localhost:5900 &
NOVNC_PID=$!

echo ""
echo "=== Vidalia ROS2 + RViz2 running ==="
echo "  vidalia_node pid : $NODE_PID"
echo "  rviz2 pid        : $RVIZ_PID"
echo ""
echo "  Open in laptop browser (VS Code SSH forwards port 6080 automatically):"
echo "    http://localhost:6080/vnc.html"
echo ""
echo "  Or via Tailscale: http://100.66.121.56:6080/vnc.html"
echo ""

trap "kill $XVFB_PID $NODE_PID $RVIZ_PID $VNC_PID $NOVNC_PID 2>/dev/null; exit 0" SIGTERM SIGINT
wait $NODE_PID $RVIZ_PID
