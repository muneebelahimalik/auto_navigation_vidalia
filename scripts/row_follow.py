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
# Diagnostic loop
# ---------------------------------------------------------------------------

async def _debug_loop(lidar: LidarDriver, args: argparse.Namespace) -> None:
    """Stream a per-scan LiDAR height profile to diagnose row detection."""
    import time

    import numpy as np

    from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT

    print(f" DEBUG — LiDAR height profile (assumed mount height "
          f"{LIDAR_MOUNT_HEIGHT:.3f} m)")
    print(f" forward strip: y 1.5-7.0 m, |x| <= {args.roi_x:.2f} m, "
          f"self-filter < {args.self_radius:.2f} m")
    print(" A healthy run shows steady 'pts' and 'hz' near 10.  The tallest")
    print(" h-hist bin is the floor — the crop band must sit ABOVE it.\n")

    h_edges = np.arange(-0.30, 1.05, 0.10)
    z_edges = np.arange(-1.10, 0.30, 0.05)
    t0 = time.monotonic()
    count = 0
    hz = 0.0
    shown = 0

    async for scan in lidar.scan_stream():
        count += 1
        now = time.monotonic()
        if now - t0 >= 1.0:
            hz = count / (now - t0)
            count = 0
            t0 = now
        if not scan:
            continue
        shown += 1
        if shown % 3 != 0:
            continue

        pts = np.array([(p.x, p.y, p.z) for p in scan], dtype=np.float64)
        rng = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[rng >= args.self_radius]
        strip = pts[(pts[:, 1] >= 1.5) & (pts[:, 1] <= 7.0)
                    & (np.abs(pts[:, 0]) <= args.roi_x)]
        if len(strip) == 0:
            print(f"pts={len(scan):6d} hz={hz:4.1f} | forward strip EMPTY")
            continue

        h = strip[:, 2] + LIDAR_MOUNT_HEIGHT
        zh, _ = np.histogram(strip[:, 2], bins=z_edges)
        floor_z = float(z_edges[int(np.argmax(zh))] + 0.025)
        hh, _ = np.histogram(h, bins=h_edges)
        band = int(((h >= args.crop_min) & (h <= args.crop_max)).sum())

        bars = " ".join(
            f"{h_edges[i]:+.2f}:{hh[i]:<4d}" for i in range(len(hh)) if hh[i] > 0
        )
        print(f"pts={len(scan):6d} hz={hz:4.1f} strip={len(strip):5d} "
              f"floor_z={floor_z:+.2f} (mount~{-floor_z:.2f}m) "
              f"crop[{args.crop_min:.2f}-{args.crop_max:.2f}]={band}")
        print(f"   h-hist: {bars}")


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def _run(args: argparse.Namespace, nav_ref: list) -> None:
    if args.debug:
        async with LidarDriver() as lidar:
            await _debug_loop(lidar, args)
        return

    canbus = _load_canbus_interface()

    if args.auto and canbus is None:
        print("\n[row_follow] --auto requested but no canbus available — "
              "falling back to perception-only mode.\n")
        args.auto = False

    slam = None
    if args.slam:
        from slam.slam_engine import SlamEngine
        slam = SlamEngine()

    detector = RowDetector(
        roi_x_half=args.roi_x,
        crop_h_min=args.crop_min,
        crop_h_max=args.crop_max,
    )
    safety = SafetyMonitor()
    controller = PurePursuitController(max_linear=args.speed)
    navigator = RowNavigator(
        canbus, detector, safety, controller,
        auto=args.auto, rows=args.rows, headland=args.headland, slam=slam,
        self_radius=args.self_radius,
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
    parser.add_argument("--roi-x", type=float, default=0.80, metavar="M",
                        help="Half-width of the centre-row search band (default: 0.80)")
    parser.add_argument("--crop-min", type=float, default=0.05, metavar="M",
                        help="Min crop height above ground to keep (default: 0.05)")
    parser.add_argument("--crop-max", type=float, default=0.60, metavar="M",
                        help="Max crop height above ground to keep (default: 0.60)")
    parser.add_argument("--self-radius", type=float, default=1.0, metavar="M",
                        help="Discard LiDAR returns within this radius — the "
                             "robot's own frame (default: 1.0)")
    parser.add_argument("--debug", action="store_true",
                        help="Stream a LiDAR height profile instead of navigating")
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
