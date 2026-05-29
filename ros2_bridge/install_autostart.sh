#!/usr/bin/env bash
# install_autostart.sh — Install a persistent systemd user service so the
# ROS2 Foxy bridge starts automatically on every Amiga reboot.
#
# Uses a SYSTEMD USER SERVICE (no sudo needed).  The service file lives
# under ~/.config/systemd/user/ which is on the NVMe and persists across
# reboots.
#
# Usage:
#   bash ros2_bridge/install_autostart.sh          install + enable
#   bash ros2_bridge/install_autostart.sh remove   uninstall

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SERVICE_DIR="$HOME/.config/systemd/user"
SERVICE_FILE="$SERVICE_DIR/vidalia-ros2-bridge.service"

install_service() {
    mkdir -p "$SERVICE_DIR"

    cat > "$SERVICE_FILE" <<EOF
[Unit]
Description=Vidalia ROS2 Foxy bridge + Foxglove WebSocket (port 8765)
After=network.target docker.service
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=${REPO_ROOT}
ExecStartPre=/usr/bin/docker rm -f vidalia_bridge || true
ExecStart=/usr/bin/docker run --rm \\
    --name vidalia_bridge \\
    --runtime nvidia \\
    --net=host \\
    -v /dev/shm:/dev/shm \\
    -e ROS_DOMAIN_ID=42 \\
    vidalia-ros2-foxy
ExecStop=/usr/bin/docker rm -f vidalia_bridge || true
Restart=on-failure
RestartSec=5

[Install]
WantedBy=default.target
EOF

    systemctl --user daemon-reload
    systemctl --user enable vidalia-ros2-bridge.service
    systemctl --user start  vidalia-ros2-bridge.service

    echo ""
    echo "=== Autostart installed ==="
    echo "  Service file : $SERVICE_FILE"
    echo "  Status       : $(systemctl --user is-active vidalia-ros2-bridge.service)"
    echo ""
    echo "  Commands:"
    echo "    systemctl --user status  vidalia-ros2-bridge"
    echo "    systemctl --user stop    vidalia-ros2-bridge"
    echo "    systemctl --user restart vidalia-ros2-bridge"
    echo "    journalctl --user -u vidalia-ros2-bridge -f"
    echo ""
    echo "  After next reboot, bridge starts automatically."
    echo "  Foxglove: https://app.foxglove.dev → ws://100.66.121.56:8765"
}

remove_service() {
    systemctl --user stop    vidalia-ros2-bridge.service 2>/dev/null || true
    systemctl --user disable vidalia-ros2-bridge.service 2>/dev/null || true
    rm -f "$SERVICE_FILE"
    systemctl --user daemon-reload
    echo "Service removed."
}

case "${1:-install}" in
    remove|uninstall) remove_service ;;
    *)                install_service ;;
esac
