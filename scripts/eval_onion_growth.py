#!/usr/bin/env python3
"""
eval_onion_growth.py — run the REAL onion bed detector through ray-cast scans
of every growth stage (sim/onion_field.py) and report how well it tracks.

For each stage and each start condition (lateral offset, yaw) the robot drives
down the bed (2 cm per scan ≈ 0.2 m/s at 10 Hz) while the detector runs exactly
as configured by ``row_follow.py --preset onion``.  Reported per case:

  lat_err   mean / std / max |error| of the lateral estimate (cm, after settling)
  hdg_err   mean / max |error| of the heading estimate (deg)
  conf      mean confidence; ``low%`` = scans below the 0.35 FOLLOW threshold
  stops     scans the navigator would treat as lost (conf < 0.35) after settling
  rowend%   scans with row-end confidence ≥ 0.70 (a false row end mid-bed)
  modes     which perception path produced the estimate

With ``--bed-end`` each stage also drives into a bed END (grass headland) and
reports where confidence dropped (m before the end) and whether the grass was
ever followed afterwards.

Usage:
    python3 scripts/eval_onion_growth.py                    # all stages, mid-bed
    python3 scripts/eval_onion_growth.py --stages mature lodged --bed-end
"""
from __future__ import annotations

import argparse
import collections
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from navigation.row_perception import RowDetector            # noqa: E402
from sim.onion_field import STAGES, OnionField, SensorConfig, stage_config  # noqa: E402

CASES = [(0.0, 0.0), (0.12, 0.0), (-0.20, 0.0), (0.05, 4.0), (-0.08, -6.0)]


def onion_detector() -> RowDetector:
    """The detector exactly as ``row_follow.py --preset onion`` builds it."""
    return RowDetector(roi_x_half=0.80, crop_h_min=0.05, crop_h_max=0.60,
                       dual_row=True, row_spacing=0.2794, bed_rows=4,
                       bed_shape=True, bed_pitch=1.83)


def run_case(stage: str, d: float, yaw: float, n: int = 60, settle: int = 20,
             seed: int = 0, field_kw=None, sensor: SensorConfig = None,
             yaw_fn=None, drop_sector: float = 0.0, det: RowDetector = None):
    """``yaw_fn(i)`` overrides the yaw per scan (weaving robot); ``drop_sector``
    = probability per scan of losing a random 40° azimuth sector (VLP-16 UDP
    packet loss); ``det`` lets a caller reuse a detector (e.g. after reset())."""
    f = OnionField(stage_config(stage, **(field_kw or {})), seed=seed)
    det = det or onion_detector()
    rng = np.random.default_rng(seed + 100)
    out = collections.defaultdict(list)
    for i in range(n):
        yw = yaw_fn(i) if yaw_fn else yaw
        P, tr = f.scan(xr=d, yr=0.02 * i, yaw_deg=yw, sensor=sensor, rng=rng, return_truth=True)
        if drop_sector and rng.random() < drop_sector:
            az = np.degrees(np.arctan2(P[:, 0], P[:, 1]))
            a0 = rng.uniform(-60, 20)
            P = P[(az < a0) | (az > a0 + 40)]
        e = det.update(P)
        if i < settle:
            continue
        out["lat"].append(e.lateral_offset - tr["lateral"])
        out["hdg"].append(math.degrees(e.heading_error - tr["heading"]))
        out["conf"].append(e.confidence)
        out["rowend"].append(e.row_end_confidence)
        out["mode"].append(det.last_mode)
        out["present"].append(det.last_bed_present)
    return {k: (np.asarray(v) if k not in ("mode",) else v) for k, v in out.items()}


def run_bed_end(stage: str, d: float = 0.05, seed: int = 0, y_end: float = 12.0,
                field_kw=None):
    """Drive toward a bed end at world y_end.  Returns (dist_before_end_when_lost,
    max confidence after it was lost, modes after)."""
    f = OnionField(stage_config(stage, y_end=y_end, **(field_kw or {})), seed=seed)
    det = onion_detector()
    rng = np.random.default_rng(seed + 200)
    lost_at, after = None, []
    yr = 0.0
    while yr < y_end + 2.0:
        e = det.update(f.scan(xr=d, yr=yr, rng=rng))
        if lost_at is None and yr > 2.0 and e.confidence < 0.35:
            lost_at = y_end - yr
        elif lost_at is not None:
            after.append(e.confidence)
        yr += 0.04
    return lost_at, (max(after) if after else 0.0), det.last_mode


def robustness_cases():
    """(label, stage, run_case kwargs) — field realities on top of the stages."""
    tilt = SensorConfig(tilt_err_deg=1.5)
    weave = lambda i: 3.0 * math.sin(2 * math.pi * i / 40.0)     # ±3°, 4 s period
    C = []
    for st in ("bare", "young", "mid", "mature", "lodged"):
        C += [
            ("weeds", st, dict(d=0.05, yaw=0.0, field_kw=dict(weeds_per_m2=1.5))),
            ("stand gaps 30%", st, dict(d=0.05, yaw=0.0, field_kw=dict(gap_prob=0.3))),
            ("low bed 4 cm", st, dict(d=0.10, yaw=0.0, field_kw=dict(bed_h=0.04))),
            ("tilt error 1.5°", st, dict(d=0.05, yaw=0.0, sensor=tilt)),
            ("sector dropouts", st, dict(d=0.05, yaw=0.0, drop_sector=0.3)),
            ("weaving ±3°", st, dict(d=0.05, yaw=0.0, yaw_fn=weave)),
        ]
    for st in ("young", "mid"):
        C.append(("missing row", st, dict(d=0.05, yaw=0.0, field_kw=dict(rows=(1, 2, 3)))))
    return C


def run_reentry(stage: str, seed: int = 0):
    """After a U-turn the detector is reset and the robot enters the next bed
    off-centre and angled.  Returns the summary over scans 15–40 after reset."""
    det = onion_detector()
    run_case(stage, 0.0, 0.0, n=25, settle=25, seed=seed, det=det)   # previous row
    det.reset()
    rs = []
    for d, yaw in ((0.30, 10.0), (-0.35, -8.0), (0.20, -12.0)):
        d2 = onion_detector()
        d2._floor_lvl = det._floor_lvl            # the tracked bed top survives the turn
        rs.append(((d, yaw), run_case(stage, d, yaw, n=40, settle=15, seed=seed, det=d2)))
    return rs


def summarise(r) -> str:
    lat, hdg, conf = r["lat"] * 100, r["hdg"], r["conf"]
    modes = collections.Counter(r["mode"])
    mtxt = " ".join(f"{k}:{v}" for k, v in modes.most_common())
    return (f"lat {lat.mean():+5.1f}±{lat.std():4.1f} max {np.abs(lat).max():4.1f}cm  "
            f"hdg {hdg.mean():+4.1f} max {np.abs(hdg).max():4.1f}°  "
            f"conf {conf.mean():.2f} low {100 * np.mean(conf < 0.35):3.0f}%  "
            f"rowend {100 * np.mean(r['rowend'] >= 0.7):3.0f}%  "
            f"nobed {100 * np.mean(~r['present']):3.0f}%  [{mtxt}]")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--stages", nargs="*", default=list(STAGES))
    ap.add_argument("--scans", type=int, default=60)
    ap.add_argument("--bed-end", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--robust", action="store_true",
                    help="run the field-robustness suite (weeds, gaps, low beds, tilt error, "
                         "dropouts, weaving, missing row, re-entry after a U-turn)")
    args = ap.parse_args()
    if args.robust:
        for label, st, kw in robustness_cases():
            if st not in args.stages:
                continue
            d, yaw = kw.pop("d"), kw.pop("yaw")
            r = run_case(st, d, yaw, n=args.scans, seed=args.seed, **kw)
            print(f"   {st:7s} {label:16s} {summarise(r)}")
        for st in args.stages:
            for (d, yaw), r in run_reentry(st, seed=args.seed):
                print(f"   {st:7s} re-entry {d:+.2f}/{yaw:+.0f}° {summarise(r)}")
        return
    for st in args.stages:
        print(f"== {st}")
        for d, yaw in CASES:
            r = run_case(st, d, yaw, n=args.scans, seed=args.seed)
            print(f"   off {d:+.2f} yaw {yaw:+4.1f}°  {summarise(r)}")
        if args.bed_end:
            lost, after, mode = run_bed_end(st, seed=args.seed)
            ltxt = "never" if lost is None else f"{lost:+.1f} m before end"
            print(f"   bed end: lost {ltxt}; max conf after {after:.2f}; final mode {mode}")


if __name__ == "__main__":
    main()
