#!/usr/bin/env bash
# install_farmng.sh — install farm-ng Python SDK in the system Python
# that ROS 2 Foxy uses.
#
# Run this ONCE before building the auto_navigation_vidalia workspace.
# The farm-ng SDK must be in the SAME Python environment as ROS 2,
# NOT inside a venv (ROS 2 nodes cannot see venv-installed packages).
#
# Usage:
#   bash scripts/install_farmng.sh
#   bash scripts/install_farmng.sh --upgrade   # upgrade existing install

set -e

UPGRADE_FLAG=""
if [[ "$1" == "--upgrade" ]]; then
    UPGRADE_FLAG="--upgrade"
fi

echo "[install_farmng] Detecting ROS 2 Python executable..."

# Prefer the Python pointed to by the active ROS 2 setup
if command -v python3 &>/dev/null; then
    PY=$(command -v python3)
else
    echo "[install_farmng] ERROR: python3 not found on PATH." >&2
    exit 1
fi

echo "[install_farmng] Using: $PY  ($(${PY} --version))"

# Check if farm_ng is already installed
if ${PY} -c "import farm_ng" 2>/dev/null; then
    INSTALLED=$(${PY} -c "import farm_ng_amiga; print(farm_ng_amiga.__version__)" 2>/dev/null || echo "unknown")
    if [[ -z "$UPGRADE_FLAG" ]]; then
        echo "[install_farmng] farm-ng already installed (version: $INSTALLED)."
        echo "                 Run with --upgrade to force reinstall."
        exit 0
    fi
fi

echo "[install_farmng] Installing farm-ng-amiga and farm-ng-core..."

# Install into the system Python (user install to avoid needing sudo).
# protobuf>=3.20 required for farm-ng (system apt protobuf 3.12.4 is too old).
${PY} -m pip install ${UPGRADE_FLAG} --user "protobuf>=3.20,<=5.27.5"
${PY} -m pip install ${UPGRADE_FLAG} farm-ng-amiga farm-ng-core

echo ""
echo "[install_farmng] Verifying install..."
${PY} -c "
from farm_ng.canbus.packet import AmigaTpdo1
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri
print('  farm_ng.canbus.packet.AmigaTpdo1  OK')
print('  farm_ng.canbus.canbus_pb2.Twist2d OK')
print('  farm_ng.core.event_client         OK')

# Check payload_to_protobuf location
try:
    from farm_ng.core.events_file_reader import payload_to_protobuf
    print('  payload_to_protobuf               OK (events_file_reader)')
except ImportError:
    try:
        from farm_ng.core.event_client import payload_to_protobuf
        print('  payload_to_protobuf               OK (event_client)')
    except ImportError:
        print('  payload_to_protobuf               MISSING — canbus decode may fail')
"

echo ""
echo "[install_farmng] Done."
echo ""
echo "[install_farmng] Run the native Python stack (Amiga brain):"
echo "    python3 main.py"
echo ""
echo "[install_farmng] If also using the ROS 2 workspace (dev PC only):"
echo "    cd ~/auto_navigation_vidalia && colcon build --symlink-install"
echo "    source install/setup.bash"
echo "    ros2 launch vidalia_bringup amiga_grpc_bridge.launch.py"
echo ""
echo "    NOTE: ROS 2 Foxy is NOT installed on the Amiga brain."
echo "          colcon build only works on a dev PC with ROS 2 Foxy."
