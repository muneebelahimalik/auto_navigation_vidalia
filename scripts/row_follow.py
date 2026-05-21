#!/usr/bin/env -S python3 -u
"""
row_follow.py — Autonomous onion-row following on the Amiga brain (no ROS 2).

Runs entirely in the native Python stack: VLP-16 UDP -> LiDAR row
perception -> pure-pursuit control -> Twist2d over the Amiga canbus.

The robot detects the centre crop row directly ahead of it in real time
and drives along it.  No pre-built map and no survey are required — this
is online, perception-driven row following.  An occupancy-grid SLAM map
can be built in the background (``--slam``) for global awareness and
later coverage tracking, but it does NOT drive the robot.

Usage (on the Amiga brain):
    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia

    # 1. PERCEPTION-ONLY — robot does not move; verify detection first:
    python3 scripts/row_follow.py

    # 2. AUTONOMOUS — robot follows the row (start slow, supervise!):
    python3 scripts/row_follow.py --auto

    # multi-row with headland turns + background SLAM map:
    python3 scripts/row_follow.py --auto --rows 8 --headland --slam

Press Ctrl+C at any time to stop the robot immediately.

Key flags:
    --auto            enable motion (default: perception-only, no motion)
    --rows N          number of rows to cover (default: 1)
    --headland        perform open-loop headland turns between rows
    --slam            build an occupancy-grid map in the background
    --speed M         max forward speed m/s (default: 0.30)
    --no-validate     skip the LiDAR startup health check
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent.parent))

# Auto-add the farm-ng venv site-packages so the script works without
# needing "source /farm_ng_image/venv/bin/activate" first.
_FARMNG_SITE = Path("/farm_ng_image/venv/lib/python3.8/site-packages")
if _FARMNG_SITE.exists() and str(_FARMNG_SITE) not in sys.path:
    sys.path.insert(0, str(_FARMNG_SITE))

from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import validate_lidar_startup
from navigation.row_controller import PurePursuitController
from navigation.row_navigator import RowNavigator
from navigation.row_perception import RowDetector
from navigation.row_safety import SafetyMonitor


# ---------------------------------------------------------------------------
# Canbus config
# ---------------------------------------------------------------------------

def _load_canbus_interface():
    """
    Load the canbus EventServiceConfig and build a CanbusInterface.

    Returns None if service_config.json or the farm-ng SDK is unavailable —
    the script then runs perception-only (it cannot command the robot).
    """
    config_path = Path(__file__).parent.parent / "service_config.json"
    if not config_path.exists():
        print(f" [row_follow] service_config.json not found at {config_path}")
        return None
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList
        from canbus.canbus_interface import CanbusInterface

        with open(config_path) as f:
            raw = json.load(f)
        config_list = json_format.ParseDict(raw, EventServiceConfigList())
        for cfg in config_list.configs:
            if cfg.name == "canbus":
                return CanbusInterface(cfg)
        print(" [row_follow] no 'canbus' service in service_config.json")
    except Exception as exc:
        print(f" [row_follow] could not load canbus config: {exc}")
    return None


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def _run(args: argparse.Namespace, nav_ref: list) -> None:
    canbus = _load_canbus_interface()

    if args.auto and canbus is None:
        print("\n[row_follow] --auto requested but no canbus available — "
              "falling back to perception-only mode.\n")
        args.auto = False

    slam = None
    if args.slam:
        from slam.slam_engine import SlamEngine
        slam = SlamEngine()

    detector = RowDetector()
    safety = SafetyMonitor()
    controller = PurePursuitController(max_linear=args.speed)
    navigator = RowNavigator(
        canbus, detector, safety, controller,
        auto=args.auto, rows=args.rows, headland=args.headland, slam=slam,
    )
    nav_ref.append(navigator)

    print()
    print("=" * 68)
    print("  Autonomous onion-row follower")
    print(f"  mode    : {'AUTONOMOUS — robot WILL move' if args.auto else 'perception-only (no motion)'}")
    print(f"  rows    : {args.rows}   headland turns: {'on' if args.headland else 'off'}")
    print(f"  speed   : {args.speed:.2f} m/s max   SLAM map: {'on' if args.slam else 'off'}")
    print("  Press Ctrl+C to stop the robot immediately.")
    print("=" * 68)
    print()

    async with LidarDriver() as lidar:
        if not args.no_validate:
            healthy = await validate_lidar_startup(lidar)
            if not healthy:
                print("[row_follow] LiDAR health check failed — aborting.")
                return

        # Confirm the canbus link with a harmless zero-velocity command.
        if canbus is not None:
            try:
                await canbus.stop()
                print(" [row_follow] canbus link OK (sent zero twist).")
            except Exception as exc:
                print(f" [row_follow] canbus unreachable: {exc}")
                if args.auto:
                    print(" [row_follow] cannot drive — switching to perception-only.")
                    navigator.auto = False

        print(" Following the centre crop row — watch the status line.\n")
        await navigator.run(lidar)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Autonomous onion-row following (native stack, no ROS 2)"
    )
    parser.add_argument("--auto", action="store_true",
                        help="Enable motion (default: perception-only)")
    parser.add_argument("--rows", type=int, default=1, metavar="N",
                        help="Number of rows to cover (default: 1)")
    parser.add_argument("--headland", action="store_true",
                        help="Perform open-loop headland turns between rows")
    parser.add_argument("--slam", action="store_true",
                        help="Build an occupancy-grid map in the background")
    parser.add_argument("--speed", type=float, default=0.30, metavar="M",
                        help="Max forward speed in m/s (default: 0.30)")
    parser.add_argument("--save-dir", default="", metavar="PATH",
                        help="Map output directory (with --slam)")
    parser.add_argument("--no-validate", action="store_true",
                        help="Skip the LiDAR startup health check")
    args = parser.parse_args()

    nav_ref: list = []
    try:
        asyncio.run(_run(args, nav_ref))
    except KeyboardInterrupt:
        pass

    print("\n[row_follow] stopping robot …")
    if nav_ref:
        navigator = nav_ref[0]
        try:
            asyncio.run(navigator.stop())
        except Exception:
            pass

        if args.slam and navigator.slam is not None:
            from slam.map_io import save_map
            state = navigator.slam.get_state()
            if state.scan_count > 0:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = Path(args.save_dir) if args.save_dir else Path("maps") / ts
                out = save_map(navigator.slam._grid, state.trajectory, save_dir)
                print(f"[row_follow] map saved: {out / 'map.png'}")
    print("[row_follow] done.")


if __name__ == "__main__":
    main()
