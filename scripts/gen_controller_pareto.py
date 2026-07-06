#!/usr/bin/env python3
"""
gen_controller_pareto.py — Regenerate results/controller_pareto.csv.

Evaluates pure-pursuit, each RL jerk-sweep policy, and (optionally) MPC on ONE
fixed held-out seed set on the current (field-calibrated) sim, and writes the
Pareto table used for the accuracy-vs-smoothness figure.  Reproducible: same
seeds for every controller, so the rows are directly comparable.

    python3 scripts/gen_controller_pareto.py                 # default sweep + MPC
    python3 scripts/gen_controller_pareto.py --episodes 400  # tighter estimates
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.rl_policy import MLPPolicy
from sim.evaluate import (evaluate, mpc_act_fn, policy_act_fn, pursuit_act_fn,
                          residual_act_fn)


def _row(name, agg):
    return {
        "controller": name,
        "xtrack_rmse_cm": round(agg["xtrack_rmse_m"][0] * 100, 2),
        "heading_rmse_deg": round(agg["heading_rmse_deg"][0], 2),
        "control_jerk": round(agg["control_jerk"][0], 4),
        "success_rate": round(agg["success_rate"], 3),
        "mean_return": round(agg["return"][0], 2),
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Regenerate the controller Pareto CSV")
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed-base", type=int, default=50_000)
    ap.add_argument("--out", default="results/controller_pareto.csv")
    ap.add_argument("--no-mpc", action="store_true", help="skip the MPC row (slow)")
    ap.add_argument("--policies", nargs="+", default=[
        "policies/follow.npz:rl_jerk0.3",
        "policies/follow_jerk1.0.npz:rl_jerk1.0",
        "policies/follow_jerk3.0.npz:rl_jerk3.0",
        "policies/follow_jerk8.0.npz:rl_jerk8.0",
    ], help="standalone RL policies — each entry PATH:label")
    ap.add_argument("--residual-policies", nargs="+", default=[
        "policies/follow_residual.npz:rl_residual_jerk0.3",
        "policies/follow_residual_jerk1.0.npz:rl_residual_jerk1.0",
        "policies/follow_residual_jerk3.0.npz:rl_residual_jerk3.0",
        "policies/follow_residual_jerk8.0.npz:rl_residual_jerk8.0",
    ], help="RESIDUAL RL policies (evaluated on top of the baseline) — PATH:label")
    ap.add_argument("--residual-scale", type=float, default=0.5)
    args = ap.parse_args()

    seeds = list(range(args.seed_base, args.seed_base + args.episodes))
    rows = [_row("pursuit", evaluate(pursuit_act_fn(), seeds))]
    print(f"pursuit: {rows[-1]}")

    for spec in args.policies:
        path, _, label = spec.partition(":")
        label = label or Path(path).stem
        if not Path(path).exists():
            print(f"  skip {label}: {path} not found")
            continue
        pol = MLPPolicy.load(path)
        rows.append(_row(label, evaluate(policy_act_fn(pol), seeds)))
        print(f"{label}: {rows[-1]}")

    for spec in args.residual_policies:
        path, _, label = spec.partition(":")
        label = label or Path(path).stem
        if not Path(path).exists():
            print(f"  skip {label}: {path} not found")
            continue
        pol = MLPPolicy.load(path)
        rows.append(_row(label, evaluate(
            residual_act_fn(pol, scale=args.residual_scale), seeds)))
        print(f"{label}: {rows[-1]}")

    if not args.no_mpc:
        rows.append(_row("mpc", evaluate(mpc_act_fn(), seeds)))
        print(f"mpc: {rows[-1]}")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {out}  ({len(rows)} rows, {args.episodes} episodes each)")


if __name__ == "__main__":
    main()
