#!/usr/bin/env bash
# setup_lidar_network.sh — Add the secondary 192.168.1.x IP to eth0 so the
# Amiga can communicate with the Velodyne VLP-16 (IP: 192.168.1.201).
#
# Background:
#   The Amiga brain's eth0 is configured by farmng_network_config.sh to
#   10.95.76.1/24 (internal Amiga bus).  The VLP-16 lives on 192.168.1.x.
#   This script adds a secondary alias 192.168.1.100/24 on eth0 so both
#   networks coexist on the same interface.
#
#   The VLP-16 web interface (http://192.168.1.201) must have its
#   "Destination IP" set to 192.168.1.100 for UDP data to flow to this host.
#
# Usage:
#   sudo bash scripts/setup_lidar_network.sh        # add the IP (idempotent)
#   bash scripts/setup_lidar_network.sh --check     # check only, no changes
#
# This script is also called by vidalia-lidar-network.service at boot.

set -euo pipefail

LIDAR_IP="192.168.1.201"
HOST_IP="192.168.1.100"
PREFIX="24"
IFACE="eth0"
CHECK_ONLY="${1:-}"

echo "[lidar-net] Checking ${IFACE} for ${HOST_IP}/${PREFIX} …"

if ip addr show dev "$IFACE" | grep -q "inet ${HOST_IP}/"; then
    echo "[lidar-net] ${HOST_IP}/${PREFIX} already present on ${IFACE} — nothing to do."
else
    if [[ "$CHECK_ONLY" == "--check" ]]; then
        echo "[lidar-net] MISSING: ${HOST_IP}/${PREFIX} not on ${IFACE}."
        echo "            Run:  sudo bash scripts/setup_lidar_network.sh"
        exit 1
    fi
    echo "[lidar-net] Adding ${HOST_IP}/${PREFIX} to ${IFACE} …"
    ip addr add "${HOST_IP}/${PREFIX}" dev "$IFACE" 2>/dev/null || true
    echo "[lidar-net] Done."
fi

echo "[lidar-net] Current eth0 addresses:"
ip -4 addr show dev "$IFACE" | grep inet

echo "[lidar-net] Pinging VLP-16 at ${LIDAR_IP} …"
if ping -c 1 -W 2 "$LIDAR_IP" &>/dev/null; then
    echo "[lidar-net] VLP-16 reachable at ${LIDAR_IP}"
else
    echo "[lidar-net] WARNING: VLP-16 at ${LIDAR_IP} not responding."
    echo "            Check:"
    echo "              * VLP-16 powered on and Ethernet cable connected?"
    echo "              * VLP-16 web interface: http://${LIDAR_IP}"
    echo "              * VLP-16 'Host (Destination) IP' must be set to ${HOST_IP}"
fi
