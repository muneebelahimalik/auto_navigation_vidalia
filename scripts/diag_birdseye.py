#!/usr/bin/env python3
"""
diag_birdseye.py — Capture LiDAR scans and save a bird's-eye PNG.

Shows the top-down (X-Y) point cloud coloured by ground-relative height,
with sector lines and the self-filter circle overlaid.  Run with different
--lidar-yaw / --lidar-tilt combinations to verify corrections visually.

Copy the PNG off the robot with:
    scp farm-ng-user-laserweeding@100.66.121.56:/tmp/birdseye.png .

Usage examples:
    # Default: field-calibrated yaw=71° + tilt=21.5° applied, forward view:
    python3 scripts/diag_birdseye.py

    # Re-derive the pitch that flattens the ground (yaw applied first):
    python3 scripts/diag_birdseye.py --tilt-sweep 0:26:1

    # Raw sensor frame (no corrections — shows 71° rotated view):
    python3 scripts/diag_birdseye.py --lidar-yaw 0 --lidar-tilt 0

    # Wider range, more scans:
    python3 scripts/diag_birdseye.py --range 8 --scans 10
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


async def collect_raw(n: int, self_r: float) -> np.ndarray:
    """Collect n self-filtered scans WITHOUT any yaw/tilt correction applied."""
    queue: asyncio.Queue[np.ndarray] = asyncio.Queue()

    async def producer() -> None:
        async for pts in lidar.scan_stream_np():
            await queue.put(pts)

    async with LidarDriver() as lidar:
        task = asyncio.create_task(producer())
        scans = []
        # discard first 3 to clear buffered packets
        for _ in range(3):
            await queue.get()
        for _ in range(n):
            pts = await queue.get()
            if len(pts):
                rng = np.hypot(pts[:, 0], pts[:, 1])
                pts = pts[rng >= self_r]
            scans.append(pts)
        task.cancel()
        try:
            await task
        except (asyncio.CancelledError, BaseException):
            pass

    return np.vstack(scans) if any(len(s) for s in scans) else np.empty((0, 3))


def correct(pts: np.ndarray, yaw_rad: float, tilt_rad: float) -> np.ndarray:
    """Apply mount corrections in the SAME order as navigation/row_navigator.py:
    yaw FIRST (align to robot frame), then tilt (pitch about robot X)."""
    if not len(pts):
        return pts
    if yaw_rad:
        pts = yaw_correct_pts(pts, yaw_rad)
    if tilt_rad:
        pts = tilt_correct_pts(pts, tilt_rad)
    return pts


def ground_ramp_slope(pts: np.ndarray) -> tuple[float, float]:
    """Robust ground-flatness metric for the forward row-following ROI.

    Returns (slope, ground_level):
      * slope        — m of height per m of forward distance; ~0 = flat ground.
      * ground_level — fitted ground height (m) at 3.75 m forward; should be ~0
                       at the correct tilt (large negative = over-rotation).

    Population-stable: in each 0.5 m forward range-bin the GROUND is taken as
    the 15th-percentile height (robust to crop/obstacles above it), so the fit
    tracks the true ground surface regardless of where the |h| window would
    fall — unlike a fixed |h|<band filter, which slides with tilt and biases
    the "flattest" answer toward over-tilting.
    """
    if len(pts) < 50:
        return float("nan"), float("nan")
    roi = pts[(pts[:, 1] > 1.5) & (pts[:, 1] < 6.0) & (np.abs(pts[:, 0]) < 1.0)]
    if len(roi) < 50:
        return float("nan"), float("nan")
    h = roi[:, 2] + LIDAR_MOUNT_HEIGHT
    y = roi[:, 1]
    edges = np.arange(1.5, 6.01, 0.5)
    idx = np.digitize(y, edges)
    yc, gc = [], []
    for b in range(1, len(edges)):
        sel = idx == b
        if sel.sum() < 10:
            continue
        yc.append(0.5 * (edges[b - 1] + edges[b]))
        gc.append(np.percentile(h[sel], 15))   # ground = low percentile
    if len(yc) < 3:
        return float("nan"), float("nan")
    slope, intercept = np.polyfit(np.array(yc), np.array(gc), 1)
    return float(slope), float(slope * 3.75 + intercept)


def save_png(pts: np.ndarray, out: str, max_range: float,
             yaw_deg: float, tilt_deg: float) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable

    fig, axes = plt.subplots(1, 2, figsize=(16, 8))

    # ---- Left: bird's-eye (X-Y) forward half, coloured by height -----------
    ax = axes[0]
    # Only show points in the forward half (y > 0) for a clean forward view
    fwd_mask = pts[:, 1] > 0 if len(pts) else np.array([], dtype=bool)
    fwd_pts = pts[fwd_mask] if len(pts) else pts
    if len(fwd_pts):
        h = fwd_pts[:, 2] + LIDAR_MOUNT_HEIGHT
        norm = Normalize(vmin=-0.2, vmax=1.5)
        sc = ax.scatter(fwd_pts[:, 0], fwd_pts[:, 1], c=h, cmap="RdYlGn_r",
                        norm=norm, s=0.5, alpha=0.7)
        plt.colorbar(sc, ax=ax, label="Height above ground (m)")

    # sector lines (forward half only: FWD, LEFT-FWD, RIGHT-FWD)
    for angle_deg, label, col in [(0, "FWD", "blue"),
                                   (45, "FWD-R", "orange"),
                                   (-45, "FWD-L", "purple")]:
        a = math.radians(angle_deg)
        dx, dy = math.sin(a), math.cos(a)
        ax.annotate("", xy=(dx * max_range * 0.90, dy * max_range * 0.90),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="->", color=col, lw=1.5))
        ax.text(dx * max_range * 0.95, dy * max_range * 0.95, label,
                ha="center", va="center", color=col, fontsize=8)

    # self-filter arc and range arc (forward half only)
    for r, style, lbl in [(1.5, "--", "self-filter"), (max_range, ":", f"{max_range}m")]:
        theta = np.linspace(0, math.pi, 200)   # 0=right, π=left (forward half)
        ax.plot(np.cos(theta) * r, np.sin(theta) * r, style, color="gray",
                lw=0.8, label=lbl)

    ax.set_aspect("equal")
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(0, max_range)
    ax.set_xlabel("X → RIGHT (m)")
    ax.set_ylabel("Y → FORWARD (m)")
    ax.set_title(f"Forward bird's-eye  yaw={yaw_deg:.0f}°  tilt={tilt_deg:.0f}°\n"
                 f"n_fwd={len(fwd_pts):,}  n_total={len(pts):,}")
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(True, alpha=0.3)
    ax.plot(0, 0, "r+", markersize=12)   # sensor origin (bottom centre)

    # ---- Right: height histogram -------------------------------------------
    ax2 = axes[1]
    if len(pts):
        h = pts[:, 2] + LIDAR_MOUNT_HEIGHT
        bins = np.arange(-0.30, 2.01, 0.05)
        counts, edges = np.histogram(h, bins=bins)
        centres = 0.5 * (edges[:-1] + edges[1:])
        ax2.barh(centres, counts, height=0.045,
                 color=["green" if 0.03 <= c <= 0.30 else
                        "red"   if c > 0.50 else
                        "steelblue" for c in centres])
        # reference lines
        for yval, label, col in [(0.03, "crop min (0.03m)", "green"),
                                  (0.30, "crop max (0.30m)", "green"),
                                  (0.50, "obstacle thr (0.50m)", "red"),
                                  (0.0, "ground", "gray")]:
            ax2.axhline(yval, color=col, lw=1.0, ls="--", alpha=0.8, label=label)
        ax2.set_xlabel("Point count per 5 cm bin")
        ax2.set_ylabel("Ground-relative height (m)")
        ax2.set_title("Height distribution")
        ax2.set_ylim(-0.35, 2.05)
        ax2.legend(fontsize=7)
        ax2.grid(True, alpha=0.3, axis="x")

        # print crop-band and obstacle counts
        crop_pts = int(((h >= 0.03) & (h <= 0.30)).sum())
        obs_pts  = int((h > 0.50).sum())
        print(f"  crop band [0.03–0.30m]: {crop_pts} pts")
        print(f"  obstacle  [>0.50m]    : {obs_pts} pts")
        print(f"  below ground [<0m]    : {int((h < 0.0).sum())} pts")

    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"  Saved → {out}")


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scans", type=int, default=5)
    ap.add_argument("--lidar-yaw", type=float, default=71.0, metavar="DEG",
                    help="Mount yaw correction (CCW positive). Default 71 (confirmed for this robot).")
    ap.add_argument("--lidar-tilt", type=float, default=21.5, metavar="DEG",
                    help="Nose-down pitch correction (degrees), applied AFTER yaw. "
                         "Default 21.5 (field-calibrated). Use --tilt-sweep to re-derive.")
    ap.add_argument("--tilt-sweep", default="", metavar="LO:HI:STEP",
                    help="Sweep tilt (after yaw) and report the ground-ramp slope "
                         "per angle to find the value that flattens the ground, "
                         "e.g. --tilt-sweep 0:20:1")
    ap.add_argument("--self-radius", type=float, default=1.5)
    ap.add_argument("--range", type=float, default=6.0,
                    help="Plot radius (m)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "birdseye.png"))
    args = ap.parse_args()

    yaw_rad  = math.radians(args.lidar_yaw)
    tilt_rad = math.radians(args.lidar_tilt)

    print(f"\n[birdseye] collecting {args.scans} scans "
          f"(yaw={args.lidar_yaw}°  tilt={args.lidar_tilt}°) …", flush=True)
    raw = await collect_raw(args.scans, args.self_radius)
    print(f"  total points after self-filter: {len(raw):,}")

    # --- Optional tilt sweep: find the angle that flattens the ground --------
    if args.tilt_sweep:
        lo, hi, step = (float(v) for v in args.tilt_sweep.split(":"))
        print(f"\n  Tilt sweep (yaw={args.lidar_yaw}° fixed, applied FIRST):")
        print("    tilt(deg)   ground-slope (m/m, ~0=flat)   ground@3.75m (m, ~0=correct)")
        best = None
        t = lo
        while t <= hi + 1e-6:
            p = correct(raw, yaw_rad, math.radians(t))
            slope, glevel = ground_ramp_slope(p)
            if not math.isnan(slope) and (best is None or abs(slope) < abs(best[1])):
                best = (t, slope, glevel)
            print(f"    {t:7.1f}        {slope:+.4f}                  {glevel:+.3f}")
            t += step
        if best is not None:
            print(f"\n  >>> Flattest ground at --lidar-tilt {best[0]:.1f} "
                  f"(slope {best[1]:+.4f}, ground@3.75m {best[2]:+.3f} m).")
            print("      SLOPE≈0 is the pitch (mount-height-independent) — trust it.")
            print("      Ground level is a SECONDARY check: in a bedded field the "
                  "15th-pct tracks the FURROW bottoms, so a negative level "
                  "(~ -0.3 m = furrow depth) is EXPECTED, not over-rotation. "
                  "Only suspect over-rotation if the level keeps dropping well "
                  "past the slope-zero tilt.")
        tilt_rad = math.radians(best[0]) if best else tilt_rad
        args.lidar_tilt = best[0] if best else args.lidar_tilt

    pts = correct(raw, yaw_rad, tilt_rad)

    # per-sector nearest range
    print("\n  Nearest range per sector:")
    for az_centre, name in [(0.0, "fwd"), (90.0, "right"), (180.0, "rear"), (270.0, "left")]:
        if not len(pts):
            break
        az_pts = np.degrees(np.arctan2(pts[:, 0], pts[:, 1])) % 360.0
        diff = np.abs(((az_pts - az_centre + 180) % 360) - 180)
        in_s = pts[diff <= 30]
        if len(in_s):
            rng = np.hypot(in_s[:, 0], in_s[:, 1])
            print(f"    {name:5s}: n={len(in_s):5d}  nearest={rng.min():.3f}m  "
                  f"median={np.median(rng):.3f}m")
        else:
            print(f"    {name:5s}: (no returns)")

    save_png(pts, args.out, args.range, args.lidar_yaw, args.lidar_tilt)
    print(f"\n  Copy off the robot:")
    print(f"    scp farm-ng-user-laserweeding@100.66.121.56:~/auto_navigation_vidalia/birdseye.png .")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
