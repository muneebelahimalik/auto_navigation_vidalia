#!/usr/bin/env bash
# entrypoint.sh — start the Vidalia ROS2 bridge + RViz2 via virtual display.
#
# Stack inside the container:
#   1. Xvfb        — virtual framebuffer display :99
#   2. vidalia_node.py  — reads /dev/shm/, publishes ROS2 topics
#   3. RViz2       — reads all topics, renders 3-D view on :99
#   4. x11vnc      — serves display :99 as VNC on port 5910
#   5. websockify  — wraps VNC in WebSocket; noVNC serves browser UI on 6080
#
# Access from laptop (VS Code SSH auto-forwards port 6080):
#   http://localhost:6080/vnc.html

set -eo pipefail

# Source ROS2 setup — path differs between dustynv image variants.
# Dockerfile saves the found path to /ros_setup_path.txt at build time.
ROS_SETUP=""
if [ -s /ros_setup_path.txt ]; then
    ROS_SETUP=$(cat /ros_setup_path.txt)
fi

if [ -z "$ROS_SETUP" ] || [ ! -f "$ROS_SETUP" ]; then
    for candidate in \
        /opt/ros/foxy/setup.bash \
        /opt/ros/foxy/install/setup.bash \
        /ros2_ws/install/setup.bash \
        /ros_ws/install/setup.bash \
        /opt/ros2_ws/install/setup.bash; do
        if [ -f "$candidate" ]; then
            ROS_SETUP="$candidate"
            break
        fi
    done
fi

if [ -n "$ROS_SETUP" ] && [ -f "$ROS_SETUP" ]; then
    echo "[ROS] Sourcing $ROS_SETUP"
    source "$ROS_SETUP"
else
    echo "[ERROR] ROS2 setup.bash not found. Listing candidates:"
    find / -name "setup.bash" -maxdepth 12 2>/dev/null | grep -i ros | head -10 || true
    exit 1
fi

export DISPLAY=:99
# Software rendering — RViz2 in Xvfb has no GPU, must use llvmpipe/Mesa.
export LIBGL_ALWAYS_SOFTWARE=1

# Use port 5910 for VNC — with --net=host the container shares the host's
# ports, and 5900 (the VNC default) is often already occupied on the Amiga.
VNC_PORT=5910

echo "[1/5] Starting virtual display (Xvfb :99, 1920x1080x24)..."
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp &
XVFB_PID=$!

# Wait for Xvfb to be fully ready before x11vnc tries to connect (up to 10 s).
for i in $(seq 1 20); do
    DISPLAY=:99 xdpyinfo >/dev/null 2>&1 && break
    sleep 0.5
done

echo "[2/5] Starting vidalia_node (reads /dev/shm/, publishes ROS2 topics)..."
python3 /workspace/vidalia_node.py &
NODE_PID=$!
sleep 2

echo "[3/5] Starting RViz2..."
rviz2 -d /workspace/vidalia.rviz &
RVIZ_PID=$!
sleep 2

echo "[4/5] Starting VNC server (x11vnc :99 → port $VNC_PORT)..."
x11vnc -display :99 -forever -nopw -rfbport $VNC_PORT \
       -noxdamage -noxrecord &
VNC_PID=$!

# Wait until x11vnc is accepting TCP connections (up to 15 s).
# Uses bash /dev/tcp — no nc/netcat dependency.
VNC_READY=0
for i in $(seq 1 30); do
    (echo >/dev/tcp/localhost/$VNC_PORT) 2>/dev/null && { VNC_READY=1; break; }
    sleep 0.5
done
if [ $VNC_READY -eq 0 ]; then
    echo "[ERROR] x11vnc failed to bind to port $VNC_PORT after 15 s — exiting."
    exit 1
fi
echo "  x11vnc ready."

echo "[5/5] Starting noVNC web server on port 6080..."
# --web-index redirects bare '/' to vnc.html so the user doesn't see a directory listing.
websockify --web=/usr/share/novnc/ --wrap-mode=ignore \
           --index-file=vnc.html 6080 localhost:$VNC_PORT &
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
