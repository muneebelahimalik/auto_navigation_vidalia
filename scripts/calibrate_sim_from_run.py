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


def field_stats(path: str, skip: int = 100):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    # Steady down-row FOLLOW only: drop the start-up and anything after the
    # first row end / headland (the turn is out of the steering task's scope).
    # Bed-relief runs: also stop where the relief fades (15 consecutive scans
    # below 0.6× its median) — older runs had no bed-end gate and kept
    # "following" the grass headland, which is not steady row following.
    rel = np.array([r.get("bed_contrast_m") or 0.0 for r in rows])
    med = float(np.median(rel[rel > 0])) if (rel > 0).any() else 0.0
    body, low = [], 0
    for r in rows[skip:]:
        if r.get("state") in ("ROW_END", "HEADLAND", "DONE") or r.get("mode") == "no-bed":
            break
        if med > 0:
            low = low + 1 if (r.get("bed_contrast_m") or 0.0) < 0.6 * med else 0
            if low >= 15:
                body = body[:-14]
                break
        if r.get("state") == "FOLLOW":
            body.append(r)
    lat = [r["lateral"] for r in body]
    hdg = [math.radians(r["heading_deg"]) for r in body]
    w = [r["ang_cmd"] for r in body]
    v = float(np.mean([r["lin_cmd"] for r in body]))
    dropout = float(np.mean([r["conf"] < 0.35 for r in body]))
    return series_stats(lat, hdg, w), v, dropout, len(body)


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


def make_cfg(p: dict, v: float, dropout: float) -> EnvConfig:
    return EnvConfig(max_linear=v / 0.98, grade_drift_std=0.004, theta_noise_grade_k=0.0,
                     dropout_p=max(dropout, 0.002), obs_ema=0.35, **p)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("telemetry")
    ap.add_argument("--iters", type=int, default=200, help="random-search candidates")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    field, v, dropout, n = field_stats(args.telemetry)
    print(f"field run: {n} steady FOLLOW scans, speed {v:.3f} m/s, dropout {dropout*100:.1f} %")
    print("  " + "  ".join(f"{k}={field[k]:.4f}" for k in KEYS))

    rng = np.random.default_rng(args.seed)
    space = dict(e_noise=(0.0, 0.15), theta_noise=(0.0, 0.06),
                 e_bias_std=(0.0, 0.10), e_bias_rho=(0.90, 0.995),
                 th_bias_std=(0.0, 0.08), th_bias_rho=(0.95, 0.998))
    best = None
    for i in range(args.iters):
        if best is None or i < args.iters // 2:
            p = {k: float(rng.uniform(*b)) for k, b in space.items()}
        else:                               # local refinement around the best
            p = {k: float(np.clip(best[1][k] + rng.normal(0, 0.15 * (b[1] - b[0])), *b))
                 for k, b in space.items()}
        s = sim_stats(make_cfg(p, v, dropout), seed=1000)
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
