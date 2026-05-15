#!/usr/bin/env bash
# run.sh — One-command launcher for the Vidalia native Python navigation stack.
#
# This is the PRIMARY way to start autonomous navigation on the Amiga brain
# after reboot.  It handles:
#   1. Activating the farm-ng Python venv
#   2. Verifying the LiDAR network IP (192.168.1.100) is on eth0
#   3. Running python3 main.py from the correct directory
#
# Usage:
#   bash /mnt/data/auto_navigation_vidalia/run.sh
#   bash /mnt/data/auto_navigation_vidalia/run.sh --config /path/to/config.json

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Activate farm-ng venv
if [ -f /farm_ng_image/venv/bin/activate ]; then
    source /farm_ng_image/venv/bin/activate
    echo "[run] Activated /farm_ng_image/venv"
else
    echo "[run] WARNING: farm-ng venv not found at /farm_ng_image/venv"
    echo "              Run:  bash scripts/install_farmng.sh"
fi

# Ensure LiDAR secondary IP is present (idempotent)
if ! ip addr show dev eth0 | grep -q "inet 192.168.1.100/"; then
    echo "[run] Adding 192.168.1.100/24 to eth0 for VLP-16 …"
    sudo ip addr add 192.168.1.100/24 dev eth0 2>/dev/null || true
fi

echo "[run] Starting navigation stack from $REPO_DIR"
cd "$REPO_DIR"
exec python3 main.py "$@"
