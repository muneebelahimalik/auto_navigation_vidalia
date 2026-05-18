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

Optional flags:
    --save-dir  PATH    override output directory
    --autosave  N       also auto-save every N seconds (default 60)
    --no-autosave       disable auto-save
    --icp-points N      ICP subsample count (default 400)
    --map-every  N      update occupancy grid every N scans (default 1)
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from lidar.lidar_driver import LidarDriver
from slam.slam_engine import SlamEngine
from slam.map_io import save_map


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _status_line(state, cell_count: int, hz: float) -> str:
    p = state.pose
    deg = math.degrees(p.theta)
    return (
        f"\rscan={state.scan_count:5d} | "
        f"x={p.x:+6.2f} y={p.y:+6.2f} θ={deg:+6.1f}° | "
        f"map={cell_count:,} cells | "
        f"icp={state.last_icp_error:.3f} m | "
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

async def run(args: argparse.Namespace) -> None:
    engine = SlamEngine(
        icp_points=args.icp_points,
        map_update_every=args.map_every,
    )

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

    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            engine.process_scan(scan)
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
    args = parser.parse_args()

    engine_ref: list[SlamEngine] = []

    async def _run_and_capture() -> None:
        e = SlamEngine(
            icp_points=args.icp_points,
            map_update_every=args.map_every,
        )
        engine_ref.append(e)

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

        async with LidarDriver() as lidar:
            async for scan in lidar.scan_stream():
                e.process_scan(scan)
                scans_since += 1

                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= 1.0:
                    hz = scans_since / elapsed
                    scans_since = 0
                    t0 = now

                state = e.get_state()
                sys.stdout.write(_status_line(state, e.cell_count, hz))
                sys.stdout.flush()

                if autosave_interval and (now - t_autosave) >= autosave_interval:
                    autosave_count += 1
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    adir = (Path(args.save_dir) if args.save_dir else Path("maps")) \
                           / f"autosave_{ts}"
                    _do_save(e, adir)
                    sys.stdout.write(f"\n[autosave #{autosave_count}] → {adir}\n")
                    t_autosave = now

    try:
        asyncio.run(_run_and_capture())
    except KeyboardInterrupt:
        pass

    # Save after the event loop exits cleanly
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
