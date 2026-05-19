#!/usr/bin/env -S python3 -u
"""
slam_mapper.py — Run 2-D SLAM while driving manually; save map on exit.

Usage (on the Amiga brain):
    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia
    python3 scripts/slam_mapper.py

Press Ctrl+C to stop.  The script will immediately save:
    maps/<timestamp>/map.npz   — raw occupancy grid (reloadable)
    maps/<timestamp>/map.png   — rendered PNG (viewable / shareable)

Wheel odometry (optional):
    If service_config.json is present and the Amiga canbus service is
    reachable, the script automatically subscribes to AmigaTpdo1 and
    fuses wheel encoder data with LiDAR ICP.  The status line shows
    "odom=<N>" counting scans that used encoder warm-start.  If the
    canbus is unavailable the script falls back to constant-velocity
    prediction silently.

Optional flags:
    --save-dir  PATH    override output directory
    --autosave  N       also auto-save every N seconds (default 60)
    --no-autosave       disable auto-save
    --icp-points N      ICP subsample count (default 400)
    --map-every  N      update occupancy grid every N scans (default 1)
    --no-odom           disable wheel odometry even if canbus is available
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
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
from slam.slam_engine import SlamEngine
from slam.wheel_odometry import WheelOdometry
from slam.map_io import save_map


# ---------------------------------------------------------------------------
# Canbus config loader
# ---------------------------------------------------------------------------

def _load_canbus_config():
    """
    Load the canbus EventServiceConfig from service_config.json using the
    same EventServiceConfigList pattern as main.py.
    Returns the config object or None if unavailable / farm-ng SDK not found.
    """
    config_path = Path(__file__).parent.parent / "service_config.json"
    if not config_path.exists():
        return None
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList
        with open(config_path) as f:
            raw = json.load(f)
        config_list = json_format.ParseDict(raw, EventServiceConfigList())
        for cfg in config_list.configs:
            if cfg.name == "canbus":
                return cfg
    except Exception as exc:
        print(f" [slam_mapper] Warning: could not load canbus config: {exc}")
    return None


# ---------------------------------------------------------------------------
# Canbus subscriber task
# ---------------------------------------------------------------------------

async def _canbus_loop(config, odom: WheelOdometry) -> None:
    """
    Subscribe to AmigaTpdo1 from the canbus service and feed encoder data
    into WheelOdometry.  Runs as a background asyncio task.
    """
    try:
        from farm_ng.core.event_client import EventClient
        from farm_ng.canbus.canbus_pb2 import AmigaTpdo1
        client = EventClient(config)
        first = True
        async for _event, message in client.subscribe(
            config.subscriptions[0], decode=True
        ):
            if first:
                print(f"\n [canbus] Connected — receiving AmigaTpdo1")
                first = False
            if isinstance(message, AmigaTpdo1):
                odom.update(
                    message.meas_speed_x,
                    message.meas_ang_rate,
                    time.monotonic(),
                )
            elif first:
                # Log unexpected message type once to aid debugging
                print(f"\n [canbus] Unexpected message type: {type(message).__name__}")
    except asyncio.CancelledError:
        pass
    except Exception as exc:
        print(f"\n [canbus] Connection failed: {exc}"
              f"\n          Falling back to constant-velocity prediction.")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _status_line(state, cell_count: int, hz: float) -> str:
    p = state.pose
    deg = math.degrees(p.theta)
    odom_str = f"odom={state.odom_scans}" if state.odom_scans > 0 else "odom=off"
    return (
        f"\rscan={state.scan_count:5d} | "
        f"x={p.x:+6.2f} y={p.y:+6.2f} θ={deg:+6.1f}° | "
        f"map={cell_count:,} cells | "
        f"icp={state.last_icp_error:.3f} m | "
        f"lc={state.loop_closures} | "
        f"{odom_str} | "
        f"{hz:.1f} Hz   "
    )


def _do_save(engine: SlamEngine, save_dir: Path) -> Path:
    state = engine.get_state()
    return save_map(engine._grid, state.trajectory, save_dir)


def _print_save_result(engine: SlamEngine, out: Path) -> None:
    print(f"\n[slam_mapper] Saved:")
    print(f"  {out / 'map.npz'}")
    print(f"  {out / 'map.png'}")
    print()
    print("Copy PNG to your laptop:")
    print(f"  scp farm-ng-user-laserweeding@100.66.121.56:"
          f"~/auto_navigation_vidalia/{out / 'map.png'} .")
    print()


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def _run(args: argparse.Namespace, engine_ref: list) -> None:
    engine = SlamEngine(
        icp_points=args.icp_points,
        map_update_every=args.map_every,
    )
    engine_ref.append(engine)

    odom = WheelOdometry()
    canbus_task: Optional[asyncio.Task] = None

    # Try to start canbus odometry in background
    if not args.no_odom:
        canbus_cfg = _load_canbus_config()
        if canbus_cfg is not None:
            canbus_task = asyncio.create_task(_canbus_loop(canbus_cfg, odom))

    autosave_interval = args.autosave if not args.no_autosave else None
    t_autosave = time.monotonic()
    autosave_count = 0
    t0 = time.monotonic()
    scans_since = 0
    hz = 0.0

    print()
    print("=" * 64)
    print("  SLAM mapper — drive the robot to build a map")
    print("  Press Ctrl+C to stop and save the map.")
    print("=" * 64)
    print()

    try:
        async with LidarDriver() as lidar:
            print(" Waiting for LiDAR packets on UDP port 2368 …")
            print(" (If this hangs, check: ping 192.168.1.201  and "
                  " sudo tcpdump -i any udp port 2368 -c 5 -q)")
            print()
            first = True
            async for scan in lidar.scan_stream():
                if first:
                    odom_status = "with wheel odometry" if canbus_task else "without wheel odometry"
                    print(f" First scan received — {len(scan):,} points. "
                          f"Mapping started ({odom_status}).")
                    first = False

                # Grab odometry delta accumulated since last scan
                delta = odom.get_delta_and_reset() if odom.available else None

                engine.process_scan(scan, odom_delta=delta)
                scans_since += 1

                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= 1.0:
                    hz = scans_since / elapsed
                    scans_since = 0
                    t0 = now

                state = engine.get_state()
                sys.stdout.write(_status_line(state, engine.cell_count, hz))
                sys.stdout.flush()

                if autosave_interval and (now - t_autosave) >= autosave_interval:
                    autosave_count += 1
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    adir = (Path(args.save_dir) if args.save_dir else Path("maps")) \
                           / f"autosave_{ts}"
                    _do_save(engine, adir)
                    sys.stdout.write(f"\n[autosave #{autosave_count}] → {adir}\n")
                    t_autosave = now
    finally:
        if canbus_task is not None:
            canbus_task.cancel()
            try:
                await canbus_task
            except asyncio.CancelledError:
                pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run 2-D SLAM and save map on exit"
    )
    parser.add_argument("--save-dir",    default="",  metavar="PATH",
                        help="Override output directory (default: maps/<timestamp>)")
    parser.add_argument("--autosave",    type=int, default=60, metavar="N",
                        help="Auto-save every N seconds (default: 60)")
    parser.add_argument("--no-autosave", action="store_true",
                        help="Disable periodic auto-save")
    parser.add_argument("--icp-points",  type=int, default=400, metavar="N",
                        help="Points used per ICP iteration (default: 400)")
    parser.add_argument("--map-every",   type=int, default=1,  metavar="N",
                        help="Update occupancy grid every N scans (default: 1)")
    parser.add_argument("--no-odom",     action="store_true",
                        help="Disable wheel odometry (use constant-velocity only)")
    args = parser.parse_args()

    engine_ref: list = []

    try:
        asyncio.run(_run(args, engine_ref))
    except KeyboardInterrupt:
        pass

    sys.stdout.write("\n")
    if not engine_ref:
        print("[slam_mapper] No engine started — nothing to save.")
        return

    engine = engine_ref[0]
    state = engine.get_state()
    if state.scan_count == 0:
        print("[slam_mapper] No scans recorded — nothing to save.")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = Path(args.save_dir) if args.save_dir else Path("maps") / ts
    print(f"[slam_mapper] Saving map ({state.scan_count} scans, "
          f"{engine.cell_count:,} cells) …")
    out = _do_save(engine, save_dir)
    _print_save_result(engine, out)


if __name__ == "__main__":
    main()
