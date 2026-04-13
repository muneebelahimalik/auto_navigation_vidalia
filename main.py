#!/usr/bin/env python3
"""
main.py — Entry point for the native farm-ng navigation stack.

This is the primary way to run autonomous navigation on the Amiga brain
(camphor-clone) WITHOUT ROS 2.  ROS 2 Foxy is not installed on the brain's
overlay filesystem and is wiped on every reboot.

Architecture:
    VLP-16 LiDAR (UDP :2368)
          ↓  lidar/lidar_driver.py
    navigation/nav_logic.py
          ↓  canbus/canbus_interface.py
    Amiga canbus service (gRPC :6001 via Tailscale)
          ↓  Twist2d via request_reply("/twist")
    Amiga wheels

Prerequisites (one-time setup on the Amiga brain):
    # Option A — activate the pre-installed farm-ng venv:
    source /farm_ng_image/venv/bin/activate

    # Option B — install into user Python:
    bash scripts/install_farmng.sh

Usage:
    python3 main.py
    python3 main.py --config /path/to/service_config.json

Tailscale must be active and camphor-clone reachable (100.66.121.56).
The VLP-16 must be connected to the same network interface and reachable
at 192.168.1.201 on UDP port 2368.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Validate farm-ng SDK before importing project modules
# ---------------------------------------------------------------------------
try:
    from google.protobuf import json_format
    from farm_ng.core.event_service_pb2 import EventServiceConfigList
except ImportError:
    print(
        "ERROR: farm-ng Python SDK not found.\n"
        "\n"
        "On the Amiga brain, activate the farm-ng venv:\n"
        "    source /farm_ng_image/venv/bin/activate\n"
        "\n"
        "Or install into user Python:\n"
        "    bash scripts/install_farmng.sh\n"
        "\n"
        "On a dev PC:\n"
        "    pip install -r requirements.txt",
        file=sys.stderr,
    )
    sys.exit(1)

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from navigation.nav_logic import NavLogic

DEFAULT_CONFIG = Path(__file__).parent / "service_config.json"


def _load_service_configs(config_path: Path) -> dict:
    """
    Load service_config.json and return a dict mapping name → EventServiceConfig.

    The JSON file follows the EventServiceConfigList protobuf format:
        {"configs": [{"name": "canbus", "host": ..., "port": ..., ...}, ...]}
    """
    with open(config_path) as f:
        raw = json.load(f)

    config_list = json_format.ParseDict(raw, EventServiceConfigList())
    return {cfg.name: cfg for cfg in config_list.configs}


async def _main(args: argparse.Namespace) -> None:
    config_path = Path(args.config)
    if not config_path.exists():
        print(
            f"ERROR: service_config.json not found at {config_path}\n"
            f"       Expected:  {DEFAULT_CONFIG}",
            file=sys.stderr,
        )
        sys.exit(1)

    configs = _load_service_configs(config_path)

    if "canbus" not in configs:
        print(
            "ERROR: 'canbus' service missing from service_config.json.\n"
            "       Expected an entry with name='canbus'.",
            file=sys.stderr,
        )
        sys.exit(1)

    canbus_cfg = configs["canbus"]
    print(
        f"[main] Connecting to Amiga canbus service: "
        f"{canbus_cfg.host}:{canbus_cfg.port}"
    )

    canbus = CanbusInterface(canbus_cfg)
    nav = NavLogic(canbus)

    print("[main] Starting VLP-16 LiDAR receiver (UDP :2368) …")
    print("[main] Press Ctrl+C to stop.\n")

    async with LidarDriver() as lidar:
        try:
            await nav.run(lidar)
        except KeyboardInterrupt:
            print("\n[main] Keyboard interrupt — stopping robot …")
            await nav.stop()
            print("[main] Robot stopped.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Vidalia — native farm-ng autonomous navigation (no ROS 2 required)"
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        metavar="PATH",
        help=f"Path to service_config.json  (default: {DEFAULT_CONFIG})",
    )
    args = parser.parse_args()
    asyncio.run(_main(args))


if __name__ == "__main__":
    main()
