#!/usr/bin/env python3
"""
replay_controllers.py — feed a REAL run's perception estimates to every
steering controller and compare what each would have commanded.

Open-loop on identical inputs: the logged per-scan estimate (lateral, heading,
confidence from ``telemetry.jsonl``) is replayed through pure pursuit, MPC and
any number of RL policies.  The robot's actual path is NOT re-simulated — this
measures each controller's RESPONSE to real field perception (steering effort,
jerk, peak commands, how much it reacts to estimate noise), which is exactly
where a sim-trained policy shows its sim-to-real gap (e.g. the soybean RL
policy steering 6.7× harder than pursuit on the same field estimates).

Usage:
    python3 scripts/replay_controllers.py runs/run_pursuit_<ts>/telemetry.jsonl \\
        --policy policies/follow_field.npz --policy policies/follow_onion.npz
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from navigation.rl_controller import RLController            # noqa: E402
from navigation.rl_policy import MLPPolicy                   # noqa: E402
from navigation.row_controller import PurePursuitController  # noqa: E402
from navigation.row_mpc_controller import RowMPCController   # noqa: E402
from navigation.row_perception import RowEstimate            # noqa: E402


def load_follow(path: str, skip: int = 100):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    rel = np.array([r.get("bed_contrast_m") or 0.0 for r in rows])
    med = float(np.median(rel[rel > 0])) if (rel > 0).any() else 0.0
    out, low = [], 0
    for r in rows[skip:]:
        if r.get("state") in ("ROW_END", "HEADLAND", "DONE") or r.get("mode") == "no-bed":
            break
        if med > 0:                        # stop at the bed end (older runs had no gate)
            low = low + 1 if (r.get("bed_contrast_m") or 0.0) < 0.6 * med else 0
            if low >= 15:
                out = out[:-14]
                break
        if r.get("state") == "FOLLOW":
            out.append(r)
    return out


def replay(ctrl, rows):
    if hasattr(ctrl, "reset"):
        ctrl.reset()
    W = []
    for r in rows:
        est = RowEstimate(heading_error=math.radians(r["heading_deg"]),
                          lateral_offset=r["lateral"], confidence=r["conf"],
                          row_end_confidence=r.get("row_end_conf", 0.0),
                          n_points=r.get("n", 0), valid=True)
        _, w = ctrl.compute(est)
        W.append(w)
    return np.asarray(W)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("telemetry")
    ap.add_argument("--policy", action="append", default=[], help="RL policy .npz (repeatable)")
    ap.add_argument("--speed", type=float, default=0.20, help="max linear speed used in the run")
    ap.add_argument("--no-mpc", action="store_true")
    ap.add_argument("--mpc-onion", action="store_true",
                    help="also replay MPC with the onion-preset tuning "
                         "(noise 0.20, temperature 0.3, drift gain 0.05)")
    args = ap.parse_args()

    rows = load_follow(args.telemetry)
    logged = np.array([r["ang_cmd"] for r in rows])
    print(f"{len(rows)} steady FOLLOW scans replayed (identical real estimates for every controller)\n")
    ctrls = [("pursuit", PurePursuitController(max_linear=args.speed))]
    if not args.no_mpc:
        ctrls.append(("mpc", RowMPCController(max_linear=args.speed)))
    if args.mpc_onion:
        ctrls.append(("mpc (onion-tuned)", RowMPCController(
            max_linear=args.speed, noise_sigma=0.20, temperature=0.3, drift_gain=0.05)))
    for p in args.policy:
        ctrls.append((Path(p).stem, RLController(policy=MLPPolicy.load(p), max_linear=args.speed)))

    print(f"  {'controller':22s} {'mean|ω|':>8s} {'p95|ω|':>8s} {'max|ω|':>8s} "
          f"{'jerk':>8s} {'×pursuit':>8s}")
    print(f"  {'(logged command)':22s} {np.abs(logged).mean():8.4f} "
          f"{np.percentile(np.abs(logged), 95):8.4f} {np.abs(logged).max():8.4f} "
          f"{np.abs(np.diff(logged)).mean():8.4f}")
    base = None
    for name, c in ctrls:
        W = replay(c, rows)
        eff = float(np.abs(W).mean())
        base = eff if base is None else base
        print(f"  {name:22s} {eff:8.4f} {np.percentile(np.abs(W), 95):8.4f} "
              f"{np.abs(W).max():8.4f} {np.abs(np.diff(W)).mean():8.4f} {eff / base:8.2f}")


if __name__ == "__main__":
    main()
