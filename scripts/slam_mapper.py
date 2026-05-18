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
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from lidar.lidar_driver import LidarDriver
from slam.slam_engine import SlamEngine
from slam.map_io import save_map


# ---------------------------------------------------------------------------
# Terminal status line
# ---------------------------------------------------------------------------

def _status(state, hz: float) -> str:
    p = state.pose
    deg = math.degrees(p.theta)
    return (
        f"\rscan={state.scan_count:5d} | "
        f"x={p.x:+6.2f} y={p.y:+6.2f} θ={deg:+6.1f}° | "
        f"map={state.scan_count and 0:,} cells | "   # placeholder — filled below
        f"icp={state.last_icp_error:.3f} m | "
        f"{hz:.1f} Hz   "
    )


def _status_line(state, cell_count: int, hz: float) -> str:
    import math
    p = state.pose
    deg = math.degrees(p.theta)
    return (
        f"\rscan={state.scan_count:5d} | "
        f"x={p.x:+6.2f} y={p.y:+6.2f} θ={deg:+6.1f}° | "
        f"map={cell_count:,} cells | "
        f"icp={state.last_icp_error:.3f} m | "
        f"{hz:.1f} Hz   "
    )


# ---------------------------------------------------------------------------
# Save helper
# ---------------------------------------------------------------------------

def _do_save(engine: SlamEngine, save_dir: Path) -> Path:
    state = engine.get_state()
    out = save_map(engine._grid, state.trajectory, save_dir)
    return out


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def run(args: argparse.Namespace) -> None:
    import math

    engine = SlamEngine(
        icp_points=args.icp_points,
        map_update_every=args.map_every,
    )

    autosave_interval = args.autosave if not args.no_autosave else None
    t_autosave = time.monotonic()
    autosave_count = 0

    # --- timing for Hz estimate ---
    t0 = time.monotonic()
    scans_since = 0
    hz = 0.0

    # --- graceful shutdown flag ---
    stop_event = asyncio.Event()

    def _sigint(_sig, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    print()
    print("=" * 64)
    print("  SLAM mapper — drive the robot to build a map")
    print("  Press Ctrl+C to stop and save the map.")
    print("=" * 64)
    print()

    try:
        async with LidarDriver() as lidar:
            async for scan in lidar.scan_stream():
                if stop_event.is_set():
                    break

                engine.process_scan(scan)
                scans_since += 1

                now = time.monotonic()
                elapsed = now - t0
                if elapsed >= 1.0:
                    hz = scans_since / elapsed
                    scans_since = 0
                    t0 = now

                state = engine.get_state()
                cells = engine.cell_count
                line = _status_line(state, cells, hz)
                sys.stdout.write(line)
                sys.stdout.flush()

                # Auto-save
                if autosave_interval and (now - t_autosave) >= autosave_interval:
                    autosave_count += 1
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    adir = Path(args.save_dir) / f"autosave_{ts}" if args.save_dir \
                        else Path("maps") / f"autosave_{ts}"
                    _do_save(engine, adir)
                    sys.stdout.write(f"\n[autosave #{autosave_count}] → {adir}\n")
                    t_autosave = now

    except Exception as exc:
        sys.stdout.write(f"\n[slam_mapper] ERROR: {exc}\n")
        raise
    finally:
        # Always save on exit
        sys.stdout.write("\n")
        state = engine.get_state()
        if state.scan_count == 0:
            print("[slam_mapper] No scans recorded — nothing to save.")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.save_dir:
            save_dir = Path(args.save_dir)
        else:
            save_dir = Path("maps") / ts

        print(f"[slam_mapper] Saving map ({state.scan_count} scans, "
              f"{engine.cell_count:,} cells) …")
        out = _do_save(engine, save_dir)
        npz = out / "map.npz"
        png = out / "map.png"
        print(f"[slam_mapper] Saved:")
        print(f"  {npz}")
        print(f"  {png}")
        print()
        print("To view the PNG, copy it to your laptop:")
        print(f"  scp farm-ng-user-laserweeding@100.66.121.56:"
              f"~/auto_navigation_vidalia/{png} .")
        print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run 2-D SLAM and save map on exit"
    )
    parser.add_argument("--save-dir",   default="",  metavar="PATH",
                        help="Override output directory (default: maps/<timestamp>)")
    parser.add_argument("--autosave",   type=int, default=60, metavar="N",
                        help="Auto-save every N seconds (default: 60)")
    parser.add_argument("--no-autosave", action="store_true",
                        help="Disable periodic auto-save")
    parser.add_argument("--icp-points", type=int, default=400, metavar="N",
                        help="Points used per ICP iteration (default: 400)")
    parser.add_argument("--map-every",  type=int, default=1,  metavar="N",
                        help="Update occupancy grid every N scans (default: 1)")
    args = parser.parse_args()

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
