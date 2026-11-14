#!/usr/bin/env python3
"""
calibrate_sim_from_run.py — system-identify the RL simulator from a field run.

The RL policy is trained in ``sim/row_follow_env.py``; it is only as good as the
disturbance model it learned against.  This script fits that model to a REAL
pure-pursuit field run: it computes the closed-loop statistics of the logged
perception estimates (what the controller actually saw) and searches the sim's
noise parameters so that pure pursuit IN THE SIM reproduces them:

  * lateral-estimate std, scan-to-scan change std, autocorrelation at 1/10/50
  * heading-estimate std, scan-to-scan change std, autocorrelation at 10
  * mean |steering command|

Fitted parameters: white lateral/heading noise, and the std / AR(1) coefficient
of a slowly wandering lateral / heading measurement bias (bed meander, shoulder
irregularity, fit drift).  Speed, dropout rate and the detector EMA are taken
from the run directly.

Usage:
    python3 scripts/calibrate_sim_from_run.py runs/run_pursuit_<ts>/telemetry.jsonl
    python3 scripts/calibrate_sim_from_run.py telemetry.jsonl --iters 300 --seed 1
    python3 scripts/calibrate_sim_from_run.py run_a/telemetry.jsonl run_b/telemetry.jsonl   # pooled, every bed

Prints the field stats, the best-fit parameters (paste into
``sim.row_follow_env.ONION_FIT`` or a new profile) and the fitted sim's stats.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.evaluate import pursuit_act_fn                 # noqa: E402
from sim.row_follow_env import EnvConfig, RowFollowEnv  # noqa: E402

KEYS = ["lat_std", "dlat_std", "lat_ac1", "lat_ac10", "lat_ac50",
        "hdg_std", "dhdg_std", "hdg_ac10", "w_abs"]
WEIGHTS = dict(lat_std=2.0, dlat_std=2.0, lat_ac1=1.0, lat_ac10=1.0, lat_ac50=0.5,
               hdg_std=2.0, dhdg_std=1.0, hdg_ac10=1.0, w_abs=1.0)


def _ac(x: np.ndarray, k: int) -> float:
    x = x - x.mean()
    v = float(x.var())
    if v <= 0 or len(x) <= k:
        return 0.0
    return float((x[:-k] * x[k:]).mean() / v)


def series_stats(lat, hdg, w) -> dict:
    lat, hdg, w = map(np.asarray, (lat, hdg, w))
    return dict(lat_std=float(lat.std()), dlat_std=float(np.diff(lat).std()),
                lat_ac1=_ac(lat, 1), lat_ac10=_ac(lat, 10), lat_ac50=_ac(lat, 50),
                hdg_std=float(hdg.std()), dhdg_std=float(np.diff(hdg).std()),
                hdg_ac10=_ac(hdg, 10), w_abs=float(np.abs(w).mean()))


def _bed_segments(rows, skip: int = 100, all_beds: bool = True):
    """Steady down-row FOLLOW stretches, one per bed: drop the first ``skip``
    scans of each bed (start-up / post-turn settling) and anything after the
    bed end.  Bed-relief runs also stop where the relief fades (15 consecutive
    scans below 0.6× the bed's median) — older runs had no bed-end gate and
    kept "following" the grass headland, which is not steady row following.
    ``all_beds=False`` keeps only the first bed (the original behaviour)."""
    beds = {}
    for r in rows:
        if r.get("state") == "FOLLOW" and r.get("mode") != "no-bed":
            beds.setdefault(int(r.get("rows_done", 0)), []).append(r)
    segs = []
    for k in sorted(beds):
        body = beds[k][skip:]
        rel = np.array([r.get("bed_contrast_m") or 0.0 for r in body])
        med = float(np.median(rel[rel > 0])) if (rel > 0).any() else 0.0
        if med > 0:
            low = 0
            for i, r in enumerate(body):
                low = low + 1 if (r.get("bed_contrast_m") or 0.0) < 0.6 * med else 0
                # Only a fade in the last 20 % is a bed end (relief swings
                # mid-bed under a canopy; current runs end FOLLOW at the gate).
                if low >= 15 and i >= 0.8 * len(body):
                    body = body[:i - 14]
                    break
        if len(body) > 300:
            segs.append(body)
        if not all_beds:
            break
    return segs


def field_stats(paths, skip: int = 100, all_beds: bool = True):
    """Length-weighted closed-loop statistics over every steady bed of every
    run in ``paths`` (one path or a list)."""
    if isinstance(paths, (str, Path)):
        paths = [paths]
    segs = []
    for path in paths:
        rows = [json.loads(l) for l in open(path) if l.strip()]
        segs += _bed_segments(rows, skip, all_beds)
    if not segs:
        raise SystemExit("no steady FOLLOW stretch found")
    stats = [series_stats([r["lateral"] for r in b],
                          [math.radians(r["heading_deg"]) for r in b],
                          [r["ang_cmd"] for r in b]) for b in segs]
    n = np.array([len(b) for b in segs], dtype=float)
    field = {k: float(np.sum([s[k] for s in stats] * n) / n.sum()) for k in KEYS}
    body = [r for b in segs for r in b]
    v = float(np.mean([r["lin_cmd"] for r in body]))
    dropout = float(np.mean([r["conf"] < 0.35 for r in body]))
    return field, v, dropout, len(body), len(segs)


def sim_stats(cfg: EnvConfig, n_ep: int = 6, steps: int = 1200, seed: int = 0) -> dict:
    cfg.max_steps = steps
    env = RowFollowEnv(cfg)
    act = pursuit_act_fn()
    L, H, W = [], [], []
    for ep in range(n_ep):
        obs = env.reset(seed=seed + ep)
        lat, hdg, w = [], [], []
        for _ in range(steps):
            a = act(obs)
            obs, _, done, info = env.step(a)
            lat.append(obs[0]); hdg.append(obs[1]); w.append(a * cfg.w_max)
            if done:
                break
        if len(lat) > 60:
            L.append(lat[50:]); H.append(hdg[50:]); W.append(w[50:])
    st = [series_stats(l, h, w) for l, h, w in zip(L, H, W)]
    return {k: float(np.mean([s[k] for s in st])) for k in KEYS}


def loss(sim: dict, field: dict) -> float:
    tot = 0.0
    for k in KEYS:
        scale = max(abs(field[k]), 0.05 if "ac" in k else 1e-3)
        tot += WEIGHTS[k] * ((sim[k] - field[k]) / scale) ** 2
    return tot


def make_cfg(p: dict, v: float, dropout: float, obs_ema: float = 0.35) -> EnvConfig:
    p = dict(p)
    ema = p.pop("obs_ema", obs_ema)
    return EnvConfig(max_linear=v / 0.98, grade_drift_std=0.004, theta_noise_grade_k=0.0,
                     dropout_p=max(dropout, 0.002), obs_ema=ema, **p)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("telemetry", nargs="+", help="one or more pursuit telemetry.jsonl")
    ap.add_argument("--first-bed-only", action="store_true",
                    help="original behaviour: only the first bed of each run")
    ap.add_argument("--iters", type=int, default=200, help="random-search candidates")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--obs-ema", type=float, default=0.35,
                    help="detector EMA assumed in the sim (also searched with --fit-ema)")
    ap.add_argument("--fit-ema", action="store_true", help="also fit the detector EMA (0.15-0.6)")
    args = ap.parse_args()

    field, v, dropout, n, nseg = field_stats(args.telemetry, all_beds=not args.first_bed_only)
    print(f"field: {nseg} beds from {len(args.telemetry)} run(s), {n} steady FOLLOW scans, "
          f"speed {v:.3f} m/s, dropout {dropout*100:.1f} %")
    print("  " + "  ".join(f"{k}={field[k]:.4f}" for k in KEYS))

    rng = np.random.default_rng(args.seed)
    space = dict(e_noise=(0.0, 0.15), theta_noise=(0.0, 0.06),
                 e_bias_std=(0.0, 0.10), e_bias_rho=(0.80, 0.995),
                 th_bias_std=(0.0, 0.08), th_bias_rho=(0.95, 0.998))
    if args.fit_ema:
        space["obs_ema"] = (0.15, 0.6)
    best = None
    for i in range(args.iters):
        if best is None or i < args.iters // 2:
            p = {k: float(rng.uniform(*b)) for k, b in space.items()}
        else:                               # local refinement around the best
            p = {k: float(np.clip(best[1][k] + rng.normal(0, 0.15 * (b[1] - b[0])), *b))
                 for k, b in space.items()}
        s = sim_stats(make_cfg(p, v, dropout, args.obs_ema), seed=1000)
        l = loss(s, field)
        if best is None or l < best[0]:
            best = (l, p, s)
            print(f"  [{i:3d}] loss {l:7.3f}  " + " ".join(f"{k}={p[k]:.4f}" for k in p))
    l, p, s = best
    print("\nbest fit (paste into sim.row_follow_env.ONION_FIT):")
    print("ONION_FIT = dict(" + ", ".join(f"{k}={p[k]:.4f}" for k in p) + ")")
    print("\n          " + "  ".join(f"{k:>9s}" for k in KEYS))
    print("field     " + "  ".join(f"{field[k]:9.4f}" for k in KEYS))
    print("sim fit   " + "  ".join(f"{s[k]:9.4f}" for k in KEYS))


if __name__ == "__main__":
    main()
