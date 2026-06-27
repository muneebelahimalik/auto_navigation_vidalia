#!/usr/bin/env -S python3 -u
"""
eval_controller.py — Compare a learned steering policy against pure-pursuit.

Runs both controllers on the SAME held-out episodes (identical slip, cross-slope
drift, sensor noise and dropout per seed) and prints the metrics side by side,
including a breakdown by cross-slope severity — the regime where a learned
policy is most likely to help.  This is the honest "does RL improve it?" test;
it reports the numbers either way.

Usage:
    python3 scripts/eval_controller.py --policy policies/follow.npz --episodes 200
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.rl_policy import MLPPolicy
from sim.evaluate import evaluate, policy_act_fn, pursuit_act_fn
from sim.row_follow_env import EnvConfig


def _fmt(agg, key, scale=1.0, unit=""):
    m, s = agg[key]
    return f"{m*scale:7.2f}{unit} ± {s*scale:5.2f}"


def _row(name, agg):
    return (f"  {name:14s} "
            f"return {agg['return'][0]:+7.2f} | "
            f"xtrack {agg['xtrack_rmse_m'][0]*100:5.1f} cm | "
            f"heading {agg['heading_rmse_deg'][0]:4.1f}° | "
            f"jerk {agg['control_jerk'][0]:.3f} | "
            f"success {agg['success_rate']*100:5.1f}%")


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare RL steering vs pure-pursuit")
    ap.add_argument("--policy", default="", help="trained policy .npz (omit = baseline only)")
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed-base", type=int, default=50_000)
    args = ap.parse_args()

    seeds = list(range(args.seed_base, args.seed_base + args.episodes))
    pursuit = pursuit_act_fn()

    print(f"Held-out evaluation over {len(seeds)} episodes (identical per-seed disturbances)\n")
    base = evaluate(pursuit, seeds)
    print(_row("pure-pursuit", base))

    if args.policy and Path(args.policy).exists():
        pol = MLPPolicy.load(args.policy)
        rl = evaluate(policy_act_fn(pol), seeds)
        print(_row("RL policy", rl))
        d = lambda k, sc=1.0: (rl[k][0] - base[k][0]) * sc
        print("\n  delta (RL - pursuit):  "
              f"xtrack {d('xtrack_rmse_m', 100):+.2f} cm   "
              f"heading {d('heading_rmse_deg'):+.2f}°   "
              f"jerk {d('control_jerk'):+.4f}   "
              f"success {(rl['success_rate']-base['success_rate'])*100:+.1f}%   "
              f"return {d('return'):+.2f}")

        # Robustness by cross-slope severity (where RL should earn its keep).
        print("\n  by cross-slope drift magnitude:")
        for lo, hi in [(0.0, 0.03), (0.03, 0.06), (0.06, 1.0)]:
            cfg = EnvConfig()
            # restrict to seeds whose drawn grade falls in [lo,hi) — re-derive via env
            sel = []
            from sim.row_follow_env import RowFollowEnv
            probe = RowFollowEnv(cfg)
            for s in seeds:
                probe.reset(seed=s)
                if lo <= abs(probe._grade) < hi:
                    sel.append(s)
            if len(sel) < 5:
                continue
            b = evaluate(pursuit, sel); r = evaluate(policy_act_fn(pol), sel)
            print(f"    |drift| {lo:.2f}-{hi:.2f} m/s (n={len(sel):3d}): "
                  f"xtrack pursuit {b['xtrack_rmse_m'][0]*100:4.1f} cm  "
                  f"RL {r['xtrack_rmse_m'][0]*100:4.1f} cm   "
                  f"success {b['success_rate']*100:3.0f}% -> {r['success_rate']*100:3.0f}%")
    elif args.policy:
        print(f"\n  (no policy file at {args.policy} — baseline only)")


if __name__ == "__main__":
    main()
