#!/usr/bin/env python3
"""
diag_alignment.py — LiDAR yaw alignment diagnostic.

Systematically measures where the sensor ACTUALLY sees objects placed at
known positions around the robot.  Answers: is the sensor's Y+ axis (what
the code calls "forward") really pointing forward, or is it rotated?

PROCEDURE
---------
Run the script. It will prompt you through three placements:

  1. No object in any direction (baseline — background subtraction)
  2. Bucket DIRECTLY IN FRONT of the robot
  3. Bucket DIRECTLY TO THE RIGHT of the robot

For each placement it captures scans and finds the nearest cluster of
returns that appeared since the previous step.  It reports:
  - median (x, y) in sensor frame
  - planar range
  - azimuth in sensor frame
  - which named sector (fwd / right / rear / left)

Expected if sensor is correctly aligned:
  Front bucket → y ≈ +range m, x ≈ 0.0 m, azimuth ≈ 0°, sector=fwd
  Right bucket → x ≈ +range m, y ≈ 0.0 m, azimuth ≈ 90°, sector=right

Actual offsets reveal the mount yaw error so it can be corrected in code.
"""

from __future__ import annotations

import asyncio
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT, tilt_correct_pts


async def ainput(prompt: str = "") -> str:
    """Non-blocking input — keeps the event loop alive while waiting for Enter."""
    loop = asyncio.get_running_loop()
    return await loop.run_in_executor(None, input, prompt)


def _sector_name(az_deg: float) -> str:
    az = az_deg % 360.0
    for name, centre in [("fwd", 0.0), ("right", 90.0), ("rear", 180.0), ("left", 270.0)]:
        diff = abs((az - centre + 180) % 360 - 180)
        if diff <= 45:
            return name
    return "other"


async def scan_producer(lidar: LidarDriver, queue: asyncio.Queue,
                         tilt_rad: float, self_r: float) -> None:
    """Single persistent scan loop — runs until task is cancelled."""
    count = 0
    try:
        async for pts in lidar.scan_stream_np():
            if len(pts):
                rng = np.hypot(pts[:, 0], pts[:, 1])
                pts = pts[rng >= self_r]
                if tilt_rad:
                    pts = tilt_correct_pts(pts, tilt_rad)
            await queue.put(np.asarray(pts, dtype=np.float32))
            count += 1
    except asyncio.CancelledError:
        raise
    except Exception as e:
        print(f"\n  [producer CRASHED after {count} scans]: {type(e).__name__}: {e}",
              flush=True)
        raise


async def flush_and_collect(queue: asyncio.Queue, n: int) -> list[np.ndarray]:
    """
    Discard all scans currently in the queue (stale — from before user
    pressed Enter), wait for 3 fresh scans to arrive, then collect n.
    """
    # Drain everything already buffered
    drained = 0
    while True:
        try:
            queue.get_nowait()
            drained += 1
        except asyncio.QueueEmpty:
            break
    if drained:
        print(f"    [flushed {drained} stale scans]", flush=True)

    # Wait for 3 fresh scans (guarantees we're seeing live data)
    for i in range(3):
        try:
            await asyncio.wait_for(queue.get(), timeout=8.0)
        except asyncio.TimeoutError:
            print(f"  *** TIMEOUT waiting for warmup scan {i+1}/3 "
                  f"(queue size={queue.qsize()}) ***", flush=True)
            return []

    # Collect n scans
    scans = []
    for i in range(n):
        try:
            scans.append(await asyncio.wait_for(queue.get(), timeout=8.0))
        except asyncio.TimeoutError:
            print(f"  *** TIMEOUT waiting for scan {i+1}/{n} "
                  f"(queue size={queue.qsize()}) ***", flush=True)
            break
    return scans


def find_new_cluster(with_pts: np.ndarray, baseline_pts: np.ndarray,
                     expect_range: float = 1.0, range_tol: float = 0.40) -> dict | None:
    """
    Find points that are in with_pts but absent from baseline (the bucket).

    Strategy: find all points within [expect_range±range_tol] of the sensor
    origin that have NO neighbours in the baseline at the same location.
    Returns median (x, y, z) and derived geometry.
    """
    rng_w = np.hypot(with_pts[:, 0], with_pts[:, 1])
    in_band = (rng_w >= expect_range - range_tol) & (rng_w <= expect_range + range_tol)
    candidates = with_pts[in_band]
    if len(candidates) < 3:
        return None

    novel = []
    for p in candidates:
        dists = np.hypot(baseline_pts[:, 0] - p[0], baseline_pts[:, 1] - p[1])
        if dists.min() > 0.10:
            novel.append(p)

    if len(novel) < 3:
        novel = candidates

    pts = np.array(novel)
    mx, my = float(np.median(pts[:, 0])), float(np.median(pts[:, 1]))
    mz = float(np.median(pts[:, 2]))
    mh = mz + LIDAR_MOUNT_HEIGHT
    r = math.hypot(mx, my)
    az = math.degrees(math.atan2(mx, my)) % 360.0  # 0°=Y+ forward, 90°=X+ right

    return {
        "x": mx, "y": my, "z": mz, "h": mh,
        "range": r, "azimuth": az,
        "sector": _sector_name(az),
        "n_pts": len(novel),
    }


def print_result(label: str, r: dict | None) -> None:
    if r is None:
        print(f"  {label}: *** NO CLUSTER FOUND — try wider range_tol or more scans ***")
        return
    print(f"  {label}:")
    print(f"    sensor (x={r['x']:+.3f}, y={r['y']:+.3f}) m   "
          f"range={r['range']:.3f} m   h={r['h']:+.3f} m")
    print(f"    azimuth={r['azimuth']:.1f}°   sector={r['sector']}   n_pts={r['n_pts']}")


def stacked(scans: list[np.ndarray]) -> np.ndarray:
    return np.vstack(scans) if scans else np.empty((0, 3))


async def main() -> None:
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scans", type=int, default=10, help="scans per step")
    ap.add_argument("--lidar-tilt", type=float, default=0.0)
    ap.add_argument("--self-radius", type=float, default=1.5)
    ap.add_argument("--expect-range", type=float, default=2.0,
                    help="Expected range of the placed object (m)")
    ap.add_argument("--range-tol", type=float, default=0.50,
                    help="Search band width around expect-range (m)")
    args = ap.parse_args()
    tilt_rad = math.radians(args.lidar_tilt)

    print("\n====================================================================")
    print("  LiDAR Yaw Alignment Diagnostic")
    print(f"  tilt={args.lidar_tilt}°  self_r={args.self_radius}m  "
          f"object_range={args.expect_range}±{args.range_tol}m")
    print("====================================================================")

    queue: asyncio.Queue[np.ndarray] = asyncio.Queue()

    async with LidarDriver() as lidar:
        # Single persistent scan loop runs as a background task throughout
        producer = asyncio.create_task(
            scan_producer(lidar, queue, tilt_rad, args.self_radius)
        )

        try:
            # ---- Step 1: baseline --------------------------------------------
            await ainput("\nStep 1: Remove ALL objects from around the robot.\n"
                         "  Press Enter to capture baseline …")
            print(f"  Capturing {args.scans} baseline scans …", flush=True)
            baseline_scans = await flush_and_collect(queue, args.scans)
            baseline = stacked(baseline_scans)
            print(f"  Baseline: {len(baseline)} total points captured.")

            # ---- Step 2: object DIRECTLY IN FRONT ----------------------------
            await ainput(f"\nStep 2: Place the bucket at exactly {args.expect_range:.1f} m "
                         "DIRECTLY IN FRONT of the robot (on the Y+ axis).\n"
                         "  Press Enter to capture …")
            print(f"  Capturing {args.scans} scans …", flush=True)
            front_scans = await flush_and_collect(queue, args.scans)
            front_pts = stacked(front_scans)
            front_result = find_new_cluster(front_pts, baseline,
                                            args.expect_range, args.range_tol)

            # ---- Step 3: object DIRECTLY TO THE RIGHT ------------------------
            await ainput(f"\nStep 3: Move the bucket to {args.expect_range:.1f} m "
                         "DIRECTLY TO THE RIGHT of the robot (on the X+ axis).\n"
                         "  Press Enter to capture …")
            print(f"  Capturing {args.scans} scans …", flush=True)
            right_scans = await flush_and_collect(queue, args.scans)
            right_pts = stacked(right_scans)
            right_result = find_new_cluster(right_pts, baseline,
                                            args.expect_range, args.range_tol)

        finally:
            producer.cancel()
            try:
                await producer
            except (asyncio.CancelledError, BaseException):
                pass

    # ---- Report ---------------------------------------------------------------
    print("\n====================================================================")
    print("  RESULTS")
    print("====================================================================")
    print_result(f"FRONT bucket (expect: y=+{args.expect_range:.1f}m, x=0, az≈0°, sector=fwd)",
                 front_result)
    print_result(f"RIGHT bucket (expect: x=+{args.expect_range:.1f}m, y=0, az≈90°, sector=right)",
                 right_result)

    if front_result and right_result:
        print("\n  INTERPRETATION:")
        az_front = front_result["azimuth"]
        if az_front > 180:
            az_front -= 360
        print(f"  Front bucket azimuth = {front_result['azimuth']:.1f}°  "
              f"(expected 0° — offset = {az_front:+.1f}°)")

        az_right = right_result["azimuth"]
        if az_right > 180:
            az_right -= 360
        az_right_error = az_right - 90.0
        if az_right_error > 180:
            az_right_error -= 360
        print(f"  Right bucket azimuth = {right_result['azimuth']:.1f}°  "
              f"(expected 90° — offset = {az_right_error:+.1f}°)")

        yaw_err = 0.5 * (az_front + az_right_error)
        print(f"  Estimated sensor yaw error ≈ {yaw_err:+.1f}°")
        if abs(yaw_err) < 10:
            print("  → Sensor yaw is CORRECT (within ±10°). No code change needed.")
        else:
            print(f"  → Sensor yaw is OFF by ~{yaw_err:.0f}°.")
            print(f"     The driver must apply a yaw rotation of {-yaw_err:.0f}° to the raw points.")
            print(f"     Add --lidar-yaw {-yaw_err:.0f} to the run command.")

    print("\n  RAW NEAREST per SECTOR (front-bucket scans):")
    for az_centre, name in [(0.0, "fwd"), (90.0, "right"), (180.0, "rear"), (270.0, "left")]:
        if len(front_pts) == 0:
            continue
        az_pts = np.degrees(np.arctan2(front_pts[:, 0], front_pts[:, 1])) % 360.0
        diff = np.abs(((az_pts - az_centre + 180) % 360) - 180)
        in_sector = front_pts[diff <= 30]
        if len(in_sector):
            rng = np.hypot(in_sector[:, 0], in_sector[:, 1])
            print(f"    {name:5s}: n={len(in_sector):5d}  nearest={rng.min():.3f} m  "
                  f"median_rng={np.median(rng):.3f} m")
        else:
            print(f"    {name:5s}: n=0  (no returns)")

    print()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
