#!/usr/bin/env -S python3 -u
"""
replay_scans.py — Offline dense-canopy perception bench.

Replays a saved raw-scan time series (``runs/<ts>/scans/``, written by
``row_follow.py --save-scans``) through ``RowDetector`` OFFLINE, so a perception
change can be scored on REAL field frames — especially the degenerate dense-
canopy frames that cause the weave / false row-ends — BEFORE it ever goes back to
the field.  This is the measurement harness for the dense-vegetation work: turns
"did it help?" from a field guess into a number.

It also A/B's two configurations in one pass (e.g. baseline vs a candidate, or
``--accumulate 1`` vs ``--accumulate 6``) on the identical scan sequence, and
reports the metrics that matter in dense canopy:

  * heading weave    — std of the per-scan detected heading (deg)
  * lateral weave    — std of the detected lateral offset (m)
  * false row-ends   — scans reading row_end_conf ≥ 0.70 while the ROI is FULL of
                       crop (n ≥ --full-n): each such scan would push a spurious
                       HEADLAND turn.  This is the mid-row-turn failure, counted.
  * low-reliability  — fraction of dense scans whose arbiter self-consistency
                       fell below the hold floor (how degenerate the run was)
  * mode split       — density vs canopy expert usage

Usage:
    # single config over a scan folder:
    python3 scripts/replay_scans.py --scans runs/run_pursuit_<ts>/scans

    # A/B accumulation (needs a run saved with --save-scans, i.e. EVERY scan):
    python3 scripts/replay_scans.py --scans <dir> --accumulate 1 --accumulate-b 6

    # A/B two crop configs / restrict to a window:
    python3 scripts/replay_scans.py --scans <dir> --from 760 --to 800 --plot out.png

The scans are already yaw/tilt-corrected robot-frame Nx3 clouds (that is what
``--save-scans`` writes), so they feed ``RowDetector.update`` directly, exactly
as the live navigator does.
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.row_perception import RowDetector           # noqa: E402
from navigation.scan_accumulator import ScanAccumulator     # noqa: E402


def _scan_index(f: str) -> int:
    return int(os.path.basename(f).split("_")[1].split(".")[0])


def load_scans(scans_dir: str, lo: int, hi: int):
    files = sorted(glob.glob(os.path.join(scans_dir, "scan_*.npy")), key=_scan_index)
    files = [f for f in files if lo <= _scan_index(f) <= hi]
    idx_path = os.path.join(scans_dir, "index.csv")
    index = {}
    if os.path.exists(idx_path):
        for r in csv.DictReader(open(idx_path)):
            index[r.get("file")] = r
    return files, index


def make_detector(args) -> RowDetector:
    return RowDetector(
        dual_row=True, roi_x_half=args.roi_x,
        crop_h_min=args.crop_min, crop_h_max=args.crop_max,
        canopy_tall_h=args.canopy_tall_h, dense_canopy_frac=args.dense_frac,
        row_spacing=args.row_spacing,
        reliability_floor=args.reliability_floor,
        row_end_veto_density=args.row_end_veto)


def _intensity_contrast(P, lateral, spacing, args):
    """Mean row-intensity minus mean strip-intensity for one Nx4 scan.

    In the 903 nm band green canopy is expected BRIGHTER than the dry residue
    strip, so a positive, consistent contrast means intensity separates the strip
    from the rows even when the height signal is a plateau (the ② discriminator
    test).  Returns None for Nx3 scans (no intensity captured)."""
    if P.shape[1] < 4:
        return None
    x, y, inten = P[:, 0], P[:, 1], P[:, 3]
    roi = (y >= args.crop_min_y) & (y <= args.roi_y_max) & (np.abs(x) <= args.roi_x)
    hs = 0.5 * spacing
    strip = roi & (np.abs(x - lateral) <= args.strip_half)
    rows = roi & ((np.abs(x - (lateral - hs)) <= args.strip_half)
                  | (np.abs(x - (lateral + hs)) <= args.strip_half))
    if strip.sum() < 5 or rows.sum() < 5:
        return None
    return float(inten[rows].mean() - inten[strip].mean())


def replay(files, args, n_accum: int):
    det = make_detector(args)
    acc = ScanAccumulator(n_accum)
    H, L, C, E, N, R, DENSE, MODE, ICON = ([] for _ in range(9))
    for f in files:
        P = np.load(f).astype(np.float64)
        # Approx inter-scan motion: saved scans may be subsampled, so use the
        # nominal field speed × the save interval only when accumulating.
        d_fwd = args.speed * args.scan_dt if n_accum > 1 else 0.0
        # Accumulation is geometry-only (drops intensity); fit on xyz.
        Pfit = acc.update(P[:, :3], d_fwd=d_fwd, d_theta=0.0)
        e = det.update(Pfit)
        H.append(math.degrees(e.heading_error)); L.append(e.lateral_offset)
        C.append(e.confidence); E.append(e.row_end_confidence); N.append(e.n_points)
        R.append(det.last_reliability); DENSE.append(det.last_dense)
        MODE.append(det.last_mode)
        # Intensity discriminator diagnostic on the RAW scan (Nx4 only).
        ic = _intensity_contrast(P, e.lateral_offset, det._spacing_est, args)
        ICON.append(ic if ic is not None else np.nan)
    return dict(H=np.array(H), L=np.array(L), C=np.array(C), E=np.array(E),
                N=np.array(N), R=np.array(R), DENSE=np.array(DENSE), MODE=MODE,
                ICON=np.array(ICON))


def summarize(name, m, full_n):
    H, L, E, N, R, DENSE = m["H"], m["L"], m["E"], m["N"], m["R"], m["DENSE"]
    false_end = int(((E >= 0.70) & (N >= full_n)).sum())
    dense = DENSE.astype(bool)
    lowrel = float((R[dense] < 0.35).mean()) if dense.any() else 0.0
    ncan = sum(1 for x in m["MODE"] if x == "canopy")
    print(f"  {name}")
    print(f"    heading weave (std)   {H.std():6.2f} deg     max |{np.abs(H).max():.1f}|")
    print(f"    lateral weave (std)   {L.std():6.3f} m")
    print(f"    FALSE row-ends        {false_end:4d}   (row_end_conf>=0.70 with n>={full_n} crop pts)")
    print(f"    dense-regime scans    {int(dense.sum()):4d}/{len(H)}   low-reliability {lowrel*100:.0f}%")
    print(f"    mode  density/canopy  {len(H)-ncan}/{ncan}")
    icon = m.get("ICON")
    if icon is not None and np.isfinite(icon).any():
        v = icon[np.isfinite(icon)]
        same_sign = max(np.mean(v > 0), np.mean(v < 0))     # sign consistency
        usable = abs(v.mean()) > 5.0 and same_sign > 0.70   # strong AND consistent
        print(f"    intensity contrast    {v.mean():+6.1f} (row−strip, 0–255)   "
              f"sign-consistent {100*same_sign:.0f}% of {len(v)} scans   "
              f"→ {'USABLE strip discriminator' if usable else 'weak / inconclusive'}")
    return dict(false_end=false_end, hstd=H.std(), lstd=L.std())


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline dense-canopy perception bench")
    ap.add_argument("--scans", required=True, help="a run's scans/ folder")
    ap.add_argument("--from", dest="lo", type=int, default=0)
    ap.add_argument("--to", dest="hi", type=int, default=10**9)
    ap.add_argument("--accumulate", type=int, default=1, help="config A: scan accumulation")
    ap.add_argument("--accumulate-b", type=int, default=None,
                    help="config B: accumulation to A/B against A (else single run)")
    ap.add_argument("--full-n", type=int, default=200,
                    help="crop-point count above which a row-end is 'false' (ROI full)")
    ap.add_argument("--speed", type=float, default=0.15, help="assumed m/s for accumulation motion")
    ap.add_argument("--scan-dt", type=float, default=0.1, help="seconds between SAVED scans")
    # detector config (match the field run)
    ap.add_argument("--roi-x", type=float, default=0.80)
    ap.add_argument("--crop-min", type=float, default=0.10)
    ap.add_argument("--crop-max", type=float, default=0.60)
    ap.add_argument("--canopy-tall-h", type=float, default=0.30)
    ap.add_argument("--dense-frac", type=float, default=0.30)
    ap.add_argument("--row-spacing", type=float, default=0.76)
    ap.add_argument("--reliability-floor", type=float, default=0.35)
    ap.add_argument("--row-end-veto", type=float, default=200.0)
    # intensity-discriminator diagnostic (Nx4 scans only)
    ap.add_argument("--roi-y-max", type=float, default=7.0)
    ap.add_argument("--crop-min-y", type=float, default=1.5)
    ap.add_argument("--strip-half", type=float, default=0.15,
                    help="cross-row half-width (m) sampled at the strip centre and each flanking row")
    ap.add_argument("--plot", default="", help="write a heading/row-end A/B figure here")
    args = ap.parse_args()

    files, index = load_scans(args.scans, args.lo, args.hi)
    if not files:
        print(f"no scan_*.npy in {args.scans} within [{args.lo},{args.hi}]"); return
    print(f"replaying {len(files)} real scans from {args.scans}"
          f"  (index {_scan_index(files[0])}..{_scan_index(files[-1])})\n")

    mA = replay(files, args, args.accumulate)
    print(f"CONFIG A  (accumulate={args.accumulate})")
    sA = summarize(f"accumulate={args.accumulate}", mA, args.full_n)
    mB = None
    if args.accumulate_b is not None:
        mB = replay(files, args, args.accumulate_b)
        print(f"\nCONFIG B  (accumulate={args.accumulate_b})")
        sB = summarize(f"accumulate={args.accumulate_b}", mB, args.full_n)
        print(f"\n  Δ (B−A): heading weave {sB['hstd']-sA['hstd']:+.2f} deg   "
              f"lateral weave {sB['lstd']-sA['lstd']:+.3f} m   "
              f"false row-ends {sB['false_end']-sA['false_end']:+d}")

    if args.plot:
        _plot(files, mA, mB, args)


def _plot(files, mA, mB, args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    x = [_scan_index(f) for f in files]
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    ax[0].plot(x, mA["H"], "-o", ms=3, color="#1f77b4", label=f"heading A (accum={args.accumulate})")
    if mB is not None:
        ax[0].plot(x, mB["H"], "-o", ms=3, color="#2ca02c", label=f"heading B (accum={args.accumulate_b})")
    ax[0].set_ylabel("detected heading (deg)"); ax[0].legend(fontsize=8); ax[0].axhline(0, color="#ccc", lw=0.5)
    ax[0].set_title(f"Dense-canopy replay bench — {os.path.basename(os.path.dirname(args.scans+'/'))}", fontweight="bold")
    ax[1].plot(x, mA["E"], "-o", ms=3, color="#1f77b4", label="row_end_conf A")
    if mB is not None:
        ax[1].plot(x, mB["E"], "-o", ms=3, color="#2ca02c", label="row_end_conf B")
    ax[1].axhline(0.70, ls="--", color="#888", lw=1, label="row-end threshold")
    ax[1].set_ylabel("row-end confidence"); ax[1].set_xlabel("scan index"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(args.plot, dpi=140, bbox_inches="tight")
    print(f"\nwrote {args.plot}")


if __name__ == "__main__":
    main()
