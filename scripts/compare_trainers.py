#!/usr/bin/env -S python3 -u
"""
compare_trainers.py — Test-set comparison of the steering policies trained by
Evolution Strategies (``train_rl.py``), PPO and SAC (``train_sb3.py``), several
training seeds each, against pure pursuit and MPC, plus learning curves.

Every controller runs the SAME held-out test episodes (seeds 50000–50199,
identical per-seed slip / cross-slope drift / noise / dropouts) — the protocol
of ``eval_controller.py``.  Policies are the exported NumPy files the robot
loads, so what is tested is what would be deployed.

Inputs (default layout written by ``scripts/run_trainer_study.sh``):
    policies/trainers/{es,ppo,sac}_<profile>_s<seed>.npz
    results/trainers/{es,ppo,sac}_<profile>_s<seed>_curve.csv
Outputs:
    results/trainers/test_<profile>.csv          one row per policy / controller
    results/trainers/summary_<profile>.txt       mean ± SD across training seeds
    results/trainers/learning_curves.png|.svg    validation return vs simulator steps

Usage:
    python3 scripts/compare_trainers.py --profile soybean --profile onion
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.rl_policy import MLPPolicy                           # noqa: E402
from sim.evaluate import evaluate, mpc_act_fn, policy_act_fn, pursuit_act_fn  # noqa: E402
from sim.gym_env import make_config                                  # noqa: E402

ALGOS = ("es", "ppo", "sac", "sac1m")
LABEL = {"es": "ES (NumPy)", "ppo": "PPO (SB3)", "sac": "SAC (SB3) 0.3 M",
         "sac1m": "SAC (SB3) 1 M"}
DEPLOYED = {"soybean": "policies/follow_field.npz", "onion": "policies/follow_onion.npz"}
# MPC settings used on the robot for each profile (see --preset onion in row_follow.py).
MPC_KW = {"soybean": {}, "onion": dict(noise_sigma=0.20, temperature=0.3, drift_gain=0.05)}
VALIDATION_SEEDS = list(range(10_000, 10_064))   # model-selection seeds
METRICS = ("xtrack_cm", "heading_deg", "jerk", "effort", "success_pct", "return")


def _metrics(agg) -> dict:
    return {"xtrack_cm": agg["xtrack_rmse_m"][0] * 100, "heading_deg": agg["heading_rmse_deg"][0],
            "jerk": agg["control_jerk"][0], "effort": agg["control_effort"][0],
            "success_pct": agg["success_rate"] * 100, "return": agg["return"][0]}


def _curve_steps(path: Path) -> float:
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return float(rows[-1]["timesteps"]) if rows else float("nan")


def compare(profile: str, pol_dir: Path, res_dir: Path, seeds, weights, algos=ALGOS) -> None:
    cfg = make_config(profile, **weights)
    rows = []
    print(f"\n=== profile {profile}: {len(seeds)} test episodes (seeds {seeds[0]}–{seeds[-1]}) ===")
    rows.append(dict(controller="pure pursuit", algo="pursuit", seed="",
                     **_metrics(evaluate(pursuit_act_fn(), seeds, cfg))))
    rows.append(dict(controller="MPC (MPPI)", algo="mpc", seed="",
                     **_metrics(evaluate(mpc_act_fn(**MPC_KW[profile]), seeds, cfg))))
    dep = Path(DEPLOYED[profile])
    if dep.exists():
        rows.append(dict(controller=f"deployed ES ({dep.name})", algo="deployed", seed="",
                         **_metrics(evaluate(policy_act_fn(MLPPolicy.load(str(dep))), seeds, cfg))))
    for algo in algos:
        for p in sorted(pol_dir.glob(f"{algo}_{profile}_s*.npz")):
            seed = int(re.search(r"_s(\d+)\.npz$", p.name).group(1))
            curve = res_dir / f"{p.stem}_curve.csv"
            m = _metrics(evaluate(policy_act_fn(MLPPolicy.load(str(p))), seeds, cfg))
            rows.append(dict(controller=LABEL[algo], algo=algo, seed=seed,
                             train_steps=_curve_steps(curve) if curve.exists() else "", **m))

    cols = ["controller", "algo", "seed", "train_steps", *METRICS]
    with open(res_dir / f"test_{profile}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})

    groups = defaultdict(list)
    for r in rows:
        groups[r["controller"]].append(r)
    lines = [f"Steering controllers — {profile} profile, {len(seeds)} held-out test episodes "
             f"(seeds {seeds[0]}–{seeds[-1]}); reward weights {weights}.",
             "Trained policies: mean ± SD across training seeds (n = seeds); every policy is the "
             "exported NumPy file the robot loads.", "",
             f"{'controller':30s} {'n':>2s}  {'cross-track cm':>15s}  {'heading °':>12s}  "
             f"{'jerk':>13s}  {'|ω| effort':>13s}  {'success %':>12s}  {'train steps':>11s}"]
    for name, rs in groups.items():
        def ms(k, fmt):
            v = np.array([r[k] for r in rs], dtype=float)
            return (f"{v.mean():{fmt}} ± {v.std(ddof=1):{fmt}}" if len(v) > 1
                    else f"{v.mean():{fmt}}        ")
        steps = [r.get("train_steps") for r in rs if r.get("train_steps") not in ("", None)]
        st = f"{np.mean(steps)/1e6:9.2f} M" if steps else ""
        lines.append(f"{name:30s} {len(rs):2d}  {ms('xtrack_cm', '6.2f'):>15s}  "
                     f"{ms('heading_deg', '5.2f'):>12s}  {ms('jerk', '.4f'):>13s}  "
                     f"{ms('effort', '.4f'):>13s}  {ms('success_pct', '5.1f'):>12s}  {st:>11s}")
    txt = "\n".join(lines)
    print(txt)
    (res_dir / f"summary_{profile}.txt").write_text(txt + "\n")


def plot_curves(profiles, res_dir: Path, weights, algos=ALGOS) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                         # pragma: no cover
        print(f"[skip figure: matplotlib unavailable — {exc}]")
        return
    colors = {"es": "#2a6fdb", "ppo": "#d1495b", "sac": "#2e9e6a", "sac1m": "#8a5cc2"}
    fig, axes = plt.subplots(1, len(profiles), figsize=(5.2 * len(profiles), 3.8), squeeze=False)
    for ax, profile in zip(axes[0], profiles):
        # Pure-pursuit reference on the same 64 validation episodes and reward.
        ref = evaluate(pursuit_act_fn(), VALIDATION_SEEDS, make_config(profile, **weights))
        ax.axhline(ref["return"][0], color="0.35", ls="--", lw=1, label="pure pursuit")
        for algo in algos:
            curves = []
            for p in sorted(res_dir.glob(f"{algo}_{profile}_s*_curve.csv")):
                with open(p) as f:
                    rs = [r for r in csv.DictReader(f) if float(r["timesteps"]) > 0]
                if not rs:
                    continue
                x = np.array([float(r["timesteps"]) for r in rs])
                y = np.array([float(r["return"]) for r in rs])
                curves.append((x, y))
                ax.plot(x, y, color=colors[algo], alpha=0.25, lw=0.8)
            if not curves:
                continue
            lo = max(c[0][0] for c in curves)
            hi = min(c[0][-1] for c in curves)
            grid = np.geomspace(lo, hi, 60)
            ys = np.array([np.interp(np.log(grid), np.log(x), y) for x, y in curves])
            ax.plot(grid, ys.mean(0), color=colors[algo], lw=2,
                    label=f"{LABEL[algo]} (mean of {len(curves)} seeds)")
        ax.set_xscale("log")
        ax.set_xlabel("simulator steps used for training")
        ax.set_ylabel("validation return (64 episodes)")
        ax.set_title(f"{profile} profile")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    for ext in ("png", "svg"):
        fig.savefig(res_dir / f"learning_curves.{ext}", dpi=200)
    print(f"figure -> {res_dir / 'learning_curves.png'}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare ES / PPO / SAC steering policies")
    ap.add_argument("--profile", action="append", choices=["soybean", "onion"])
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed-base", type=int, default=50_000)
    ap.add_argument("--policies", default="policies/trainers")
    ap.add_argument("--results", default="results/trainers")
    ap.add_argument("--algos", nargs="+", choices=ALGOS, default=list(ALGOS))
    ap.add_argument("--c-u", type=float, default=0.8)
    ap.add_argument("--c-du", type=float, default=8.0)
    args = ap.parse_args()
    profiles = args.profile or ["soybean", "onion"]
    seeds = list(range(args.seed_base, args.seed_base + args.episodes))
    res = Path(args.results)
    res.mkdir(parents=True, exist_ok=True)
    for prof in profiles:
        compare(prof, Path(args.policies), res, seeds, dict(c_u=args.c_u, c_du=args.c_du),
                args.algos)
    plot_curves(profiles, res, dict(c_u=args.c_u, c_du=args.c_du), args.algos)


if __name__ == "__main__":
    main()
