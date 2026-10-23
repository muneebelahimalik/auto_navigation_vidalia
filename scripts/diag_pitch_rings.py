#!/usr/bin/env -S python3 -u
"""
diag_pitch_rings.py — Measure LiDAR pitch & height from RAW ring geometry.

This bypasses the ENTIRE correction pipeline (no yaw, no tilt_correct, no
ground_ramp_slope).  It reads raw VLP-16 returns and uses pure trigonometry on
the 16 known channel elevation angles to solve for the sensor's nose-down pitch
θ and height H above a level floor.

Geometry (forward beam, channel elevation α, level ground H below the sensor):
    a beam on channel α, with the sensor pitched nose-down by θ, points
    (θ − α) below true horizontal, so it reaches the ground at range
        R = H / sin(θ − α).
Every downward channel that sees the floor straight ahead gives one such
equation.  With ≥2 channels the pair (θ, H) is over-determined:
  * per-channel: assuming H, θ_i = α_i + asin(H / R_i)  — should agree across i.
  * joint fit:   R_i·sin(θ − α_i) = H  → for each θ, H(θ)=mean(R_i sin(θ−α_i));
                 the θ that minimises the spread of those H is the pitch.

Run on the brain with a CLEAR, flat floor straight ahead for ~8 m — move the
calibration box out of the way first (the forward ±3° window is exactly where
the box would sit, and rays hitting it read short):

    python3 scripts/diag_pitch_rings.py --height 0.99
    python3 scripts/diag_pitch_rings.py --height 0.99 --az-window 4 --scans 8
"""
from __future__ import annotations

import argparse
import asyncio
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lidar.lidar_driver import LidarDriver, VLP16_VERTICAL_ANGLES
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT


async def collect(scans: int, az_window: float, rmin: float, rmax: float):
    """Median forward-beam range per channel (ring), over `scans` revolutions.

    Returns {ring: (alpha_deg, median_range_m, n)} for rings that see the floor
    straight ahead (|azimuth|<az_window, planar range in [rmin,rmax])."""
    per_ring: dict[int, list[float]] = {r: [] for r in range(16)}
    got = 0
    async with LidarDriver() as lidar:
        async for scan in lidar.scan_stream():
            for p in scan:
                az = p.azimuth % 360.0
                if not (az <= az_window or az >= 360.0 - az_window):
                    continue
                rng = math.hypot(p.x, p.y)
                if rng < rmin or rng > rmax:
                    continue
                if p.y <= 0:            # forward only
                    continue
                # full 3-D range from the sensor origin
                R = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
                per_ring[p.ring].append(R)
            got += 1
            if got >= scans:
                break
    out = {}
    for r, vals in per_ring.items():
        if len(vals) >= 5:
            out[r] = (float(VLP16_VERTICAL_ANGLES[r]), float(np.median(vals)), len(vals))
    return out


def solve(rings: dict, H_known: float):
    """Per-channel pitch (given H) and a joint (θ,H) least-squares fit."""
    print(f"\n  Per-channel pitch (assuming H = {H_known:.3f} m):")
    print("    ring  α(deg)   R(m)    n     θ = α + asin(H/R)")
    alphas, Rs = [], []
    per_theta = []
    for r in sorted(rings):
        a, R, n = rings[r]
        arad = math.radians(a)
        # only downward-looking channels that actually reach the floor
        if R <= H_known:
            continue
        ratio = H_known / R
        if not (-1.0 <= ratio <= 1.0):
            continue
        theta = a + math.degrees(math.asin(ratio))
        # keep physically-forward ground hits (θ−α in (0,90))
        if not (0 < theta - a < 90):
            continue
        per_theta.append(theta)
        alphas.append(arad); Rs.append(R)
        print(f"    {r:3d}   {a:+6.1f}  {R:6.2f}  {n:4d}      θ = {theta:5.1f}°")
    if len(per_theta) >= 2:
        print(f"\n  Per-channel θ: median = {np.median(per_theta):.1f}°  "
              f"mean = {np.mean(per_theta):.1f}°  spread(std) = {np.std(per_theta):.1f}°")

    # Joint fit for (θ, H): minimise spread of R_i·sin(θ−α_i)
    if len(alphas) >= 2:
        alphas = np.asarray(alphas); Rs = np.asarray(Rs)
        best = None
        for tdeg in np.arange(5.0, 30.01, 0.1):
            t = math.radians(tdeg)
            Hs = Rs * np.sin(t - alphas)
            if np.any(Hs <= 0):
                continue
            spread = float(np.std(Hs))
            if best is None or spread < best[2]:
                best = (tdeg, float(np.mean(Hs)), spread)
        if best:
            print(f"\n  >>> Joint raw-geometry fit (no correction code):")
            print(f"      pitch  θ = {best[0]:.1f}°     (independent of H)")
            print(f"      height H = {best[1]:.3f} m   (independent of the tape)")
            print(f"      residual spread = {best[2]*100:.1f} cm "
                  f"({'tight — trustworthy' if best[2] < 0.03 else 'loose — noisy floor/box'})")
            print(f"\n  Compare to: tape H = {H_known:.2f} m (the --height you passed).")
            if best[2] < 0.03:
                print(f"  >>> Use: --lidar-tilt {best[0]:.1f}  and  --lidar-height {best[1]:.2f}")
                print(f"      (or set LIDAR_MOUNT_HEIGHT = {best[1]:.2f} in lidar/obstacle_filter.py).")
                print( "      Cross-check the tilt with diag_birdseye.py --tilt-sweep; the two "
                       "should agree within ~0.5°.")
            if abs(best[1] - H_known) > 0.05:
                print(f"  ! Fitted height differs from the tape by {100 * (best[1] - H_known):+.0f} cm"
                      " — re-measure ground → drum centre (robot on its wheels, level floor).")

async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=LIDAR_MOUNT_HEIGHT, metavar="M",
                    help="Tape-measured LiDAR height above the floor "
                         f"(default: LIDAR_MOUNT_HEIGHT = {LIDAR_MOUNT_HEIGHT:.2f}).")
    ap.add_argument("--az-window", type=float, default=3.0, metavar="DEG",
                    help="Forward azimuth half-window for 'straight ahead' (default 3°).")
    ap.add_argument("--rmin", type=float, default=1.0, help="min planar range (m)")
    ap.add_argument("--rmax", type=float, default=8.0, help="max planar range (m)")
    ap.add_argument("--scans", type=int, default=8)
    args = ap.parse_args()

    print(f"[pitch-rings] collecting {args.scans} scans, forward |az|<{args.az_window}° …")
    rings = await collect(args.scans, args.az_window, args.rmin, args.rmax)
    if len(rings) < 2:
        print("  Too few forward ground returns — clear the space straight ahead, "
              "lower --rmin, or widen --az-window.")
        return
    solve(rings, args.height)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
