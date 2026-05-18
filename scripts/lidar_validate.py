#!/usr/bin/env python3
"""
lidar_validate.py — Standalone LiDAR health and orientation check.

No robot motion is commanded.  Use this to verify the VLP-16 is streaming
correctly and that its mounting orientation matches expectations before
running any drive test.

Checks performed for each scan:
  - Total point count  (expect ~25,000–32,000 per 360° scan)
  - Ring coverage      (expect all 16 channels)
  - Per-sector density (forward / left / right / rear, 60°-wide each)
  - Nearest return per sector
  - Ground-relative obstacle count in the forward safety corridor

Usage:
    python3 scripts/lidar_validate.py
    python3 scripts/lidar_validate.py --scans 10
    python3 scripts/lidar_validate.py --duration 30   # run for 30 seconds
"""

import argparse
import asyncio
import math
import sys
import time
from typing import Dict, List

# Allow running directly from workspace root without pip install
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import (
    LIDAR_MOUNT_HEIGHT,
    LOOK_AHEAD_DIST,
    MIN_OBSTACLE_GROUND_HEIGHT,
    ROBOT_HALF_WIDTH,
    front_zone_points,
    nearest_in_sector,
    sector_counts,
)


def _bar(n: int, max_n: int, width: int = 20) -> str:
    filled = round(width * n / max_n) if max_n > 0 else 0
    return "[" + "#" * filled + "-" * (width - filled) + f"] {n:5d}"


def _polar_display(sct: Dict[str, int]) -> str:
    """Compact 4-direction density display."""
    max_v = max(sct.values()) or 1
    return "\n".join([
        f"           FWD  {_bar(sct['fwd'], max_v)}",
        f"  LEFT  {_bar(sct['left'], max_v)}    RIGHT  {_bar(sct['right'], max_v)}",
        f"           REAR {_bar(sct['rear'], max_v)}",
    ])


async def run_validate(args: argparse.Namespace) -> None:
    print("=" * 70)
    print("LIDAR VALIDATION — no robot motion")
    print(f"  LiDAR mount height        : {LIDAR_MOUNT_HEIGHT:.3f} m above ground")
    print(f"  Min obstacle ground height: {MIN_OBSTACLE_GROUND_HEIGHT:.3f} m  "
          f"(sensor-frame z > {MIN_OBSTACLE_GROUND_HEIGHT - LIDAR_MOUNT_HEIGHT:.3f} m)")
    print(f"  Forward corridor          : y ∈ (0, {LOOK_AHEAD_DIST:.1f} m], "
          f"|x| < {ROBOT_HALF_WIDTH:.2f} m")
    if args.duration > 0:
        print(f"  Running for               : {args.duration} s")
    else:
        print(f"  Running for               : {args.scans} scans")
    print("=" * 70)

    total_scans = 0
    sum_pts = 0
    sum_rings = 0
    warn_count = 0
    t_start = time.monotonic()

    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            total_scans += 1
            elapsed = time.monotonic() - t_start

            if args.duration > 0 and elapsed >= args.duration:
                break
            if args.duration == 0 and total_scans > args.scans:
                break

            rings_seen = len({p.ring for p in scan})
            sct = sector_counts(scan)
            fwd_zone = front_zone_points(scan)
            n_fwd = nearest_in_sector(scan, 0.0)
            n_left = nearest_in_sector(scan, 270.0)
            n_right = nearest_in_sector(scan, 90.0)
            n_rear = nearest_in_sector(scan, 180.0)

            sum_pts += len(scan)
            sum_rings += rings_seen

            def _fmt(v: float) -> str:
                return f"{v:5.2f} m" if v < math.inf else " open "

            warn = ""
            if len(scan) < 5000:
                warn = "  *** LOW POINT COUNT ***"
                warn_count += 1
            if rings_seen < 14:
                warn += f"  *** ONLY {rings_seen}/16 RINGS ***"
                warn_count += 1

            print(f"\n--- scan {total_scans:03d}  ({elapsed:.1f} s) ---{warn}")
            print(f"  total pts : {len(scan):6d}   rings: {rings_seen}/16")
            print(
                f"  nearest   : fwd={_fmt(n_fwd)}  left={_fmt(n_left)}  "
                f"right={_fmt(n_right)}  rear={_fmt(n_rear)}"
            )
            print(f"  fwd-zone obstacle pts (ground-relative): {len(fwd_zone)}")
            print(_polar_display(sct))

            avg_lat = (sct["left"] + sct["right"]) / 2
            if avg_lat > 300 and sct["fwd"] < avg_lat * 0.1:
                print(
                    "  !! ORIENTATION WARNING: forward sector sparse relative to "
                    "lateral — verify mount yaw is 0°"
                )
                warn_count += 1

    if total_scans == 0:
        print("No scans received — check UDP connection (192.168.1.100, port 2368).")
        return

    avg_pts = sum_pts / total_scans
    avg_rings = sum_rings / total_scans
    print("\n" + "=" * 70)
    print(f"SUMMARY — {total_scans} scans")
    print(f"  avg pts/scan : {avg_pts:.0f}")
    print(f"  avg rings    : {avg_rings:.1f} / 16")
    print(f"  warnings     : {warn_count}")
    print("  RESULT:", "PASS" if warn_count == 0 else "WARN — review output above before driving")
    print("=" * 70)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scans", type=int, default=8,
        help="Number of scans to collect (used when --duration is 0)",
    )
    parser.add_argument(
        "--duration", type=float, default=0,
        help="Run for this many seconds (0 = use --scans)",
    )
    args = parser.parse_args()
    asyncio.run(run_validate(args))


if __name__ == "__main__":
    main()
