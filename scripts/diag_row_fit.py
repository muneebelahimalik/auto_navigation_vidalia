#!/usr/bin/env python3
"""
diag_row_fit.py — Offline-analyzable diagnostic for the dual-row LiDAR fit.

Captures N scans (robot stays still — no canbus), applies EXACTLY the same
preprocessing as RowNavigator (self-filter, optional tilt correction), then
for every scan prints the internals of the dual-row fit:

  * crop-band point count and left/right split
  * whole-cloud PCA heading (the seed)
  * cross-row histogram peaks
  * per-cluster mass, centroid, y-extent and per-cluster OWN PCA heading
  * pooled cluster-centred heading (the production path)
  * midpoint lateral + spacing factor

It also saves every raw scan to ONE compressed .npz so the clouds can be
re-analyzed on another machine:

    python3 scripts/diag_row_fit.py --scans 20 --out /tmp/row_diag.npz

Copy /tmp/row_diag.npz off the brain (scp) or `git add` it for remote
debugging.  No robot motion, no camera needed.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT, tilt_correct_pts, yaw_correct_pts
from navigation.row_perception import (
    RowDetector,
    _cluster_centred_pca,
    _weighted_pca_dir,
    find_row_midpoint,
    histogram_peaks,
)


def deg(rad: float) -> float:
    return math.degrees(rad)


def heading_of(direction: np.ndarray) -> float:
    return math.atan2(direction[0], direction[1])


def analyze_scan(i: int, pts: np.ndarray, det: RowDetector) -> None:
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    h = z + LIDAR_MOUNT_HEIGHT
    roi = (
        (y >= det.roi_y_min) & (y <= det.roi_y_max)
        & (np.abs(x) <= det.roi_x_half)
        & (h >= det.crop_h_min) & (h <= det.crop_h_max)
    )
    P = np.column_stack((x[roi], y[roi]))
    n = len(P)
    print(f"\n--- scan {i}: raw={len(pts)}  crop ROI n={n} ---")
    if n < det.min_points:
        print("    (below min_points — production fit would decay)")
        return

    n_left = int((P[:, 0] < 0).sum())
    print(f"    left/right split (x<0 / x>=0): {n_left} / {n - n_left}")

    # Seed: whole-cloud PCA (the production seed)
    direction, linearity = _weighted_pca_dir(P)
    print(f"    whole-cloud PCA : heading={deg(heading_of(direction)):+6.2f}°  linearity={linearity:.3f}")

    w = np.ones(n)
    cross = P @ np.array([direction[1], -direction[0]])
    for it in range(2):
        peaks = histogram_peaks(cross, det.roi_x_half, det.bin_width)
        peak_str = ", ".join(f"{p:+.3f}" for p in peaks)
        print(f"    pass {it+1} peaks   : [{peak_str}]")
        # per-cluster diagnostics
        pk = np.asarray(peaks)
        dists = np.abs(cross[:, None] - pk[None, :])
        assign = np.argmin(dists, axis=1)
        near = dists[np.arange(n), assign] <= 0.25
        for k, p in enumerate(peaks):
            sel = (assign == k) & near
            m = int(sel.sum())
            if m < 5:
                print(f"      cluster @{p:+.3f}: n={m} (skipped)")
                continue
            C = P[sel]
            d_k, lin_k = _weighted_pca_dir(C)
            print(f"      cluster @{p:+.3f}: n={m:4d}  y=[{C[:,1].min():.2f},{C[:,1].max():.2f}]"
                  f"  x_mean={C[:,0].mean():+.3f}  own-heading={deg(heading_of(d_k)):+6.2f}°  lin={lin_k:.3f}")
        n_far = int((~near).sum())
        if n_far:
            far = P[~near]
            print(f"      excluded (>0.25m from peaks): n={n_far}  x_mean={far[:,0].mean():+.3f}")
        direction, linearity = _cluster_centred_pca(
            P, w, cross, peaks, fallback=(direction, linearity))
        cross = P @ np.array([direction[1], -direction[0]])
        print(f"    pass {it+1} pooled  : heading={deg(heading_of(direction)):+6.2f}°  linearity={linearity:.3f}")

    lateral, sf = find_row_midpoint(cross, det.roi_x_half, det.bin_width, det.row_spacing)
    print(f"    midpoint lateral={lateral:+.3f} m  spacing_factor={sf:.2f}")

    est = det.update(pts)
    print(f"    PRODUCTION est  : hdg={deg(est.heading_error):+6.2f}°  off={est.lateral_offset:+.3f} m"
          f"  conf={est.confidence:.2f}  n={est.n_points}")


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scans", type=int, default=20)
    ap.add_argument("--out", default="/tmp/row_diag.npz")
    ap.add_argument("--self-radius", type=float, default=1.5)
    ap.add_argument("--lidar-tilt", type=float, default=0.0)
    ap.add_argument("--lidar-yaw", type=float, default=0.0,
                    help="Mount yaw correction (CCW positive degrees). Use 71 for this robot.")
    ap.add_argument("--roi-x", type=float, default=0.80)
    ap.add_argument("--row-spacing", type=float, default=0.76)
    args = ap.parse_args()

    det = RowDetector(dual_row=True, row_spacing=args.row_spacing,
                      roi_x_half=args.roi_x)
    tilt_rad = math.radians(args.lidar_tilt)
    yaw_rad  = math.radians(args.lidar_yaw)

    scans = []
    print(f"[diag] capturing {args.scans} scans …")
    async with LidarDriver() as lidar:
        i = 0
        async for pts in lidar.scan_stream_np():
            if len(pts):
                rng = np.hypot(pts[:, 0], pts[:, 1])
                pts = pts[rng >= args.self_radius]
                if yaw_rad != 0.0:
                    pts = yaw_correct_pts(pts, yaw_rad)
                if tilt_rad != 0.0:
                    pts = tilt_correct_pts(pts, tilt_rad)
            scans.append(np.asarray(pts, dtype=np.float32))
            analyze_scan(i, pts, det)
            i += 1
            if i >= args.scans:
                break

    np.savez_compressed(args.out, **{f"scan_{j:03d}": s for j, s in enumerate(scans)})
    print(f"\n[diag] saved {len(scans)} scans to {args.out}")
    print("[diag] copy off the brain with e.g.:")
    print(f"         scp farm-ng-user-laserweeding@100.66.121.56:{args.out} .")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
