#!/usr/bin/env python3
"""
select_onion_policy.py — choose the onion RL policy on data it was not fitted to.

Every candidate (pure pursuit, MPC at the field setting, the field RL policy,
and every policies/trainers/*_onion_nov_s*.npz) is scored two ways:
  * sim: 200 held-out test episodes on the 'onion_nov' profile;
  * real: the logged FOLLOW estimates of held-out field runs (the MPC runs,
    which were NOT used to fit 'onion_nov') replayed open-loop through each
    controller — steering effort, jerk and peak relative to pure pursuit on
    the identical inputs.

    python3 scripts/select_onion_policy.py runs/run_mpc_A/telemetry.jsonl runs/run_mpc_B/telemetry.jsonl
"""
import json, math, sys, glob
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from navigation.rl_controller import RLController
from navigation.rl_policy import MLPPolicy
from navigation.row_controller import PurePursuitController
from navigation.row_mpc_controller import RowMPCController
from navigation.row_perception import RowEstimate
from sim.evaluate import evaluate, mpc_act_fn, policy_act_fn, pursuit_act_fn
from sim.row_follow_env import profile_config

HELDOUT = sys.argv[1:]
if not HELDOUT:
    raise SystemExit(__doc__)
MPC = dict(noise_sigma=0.10, temperature=0.1, drift_gain=0.05)

def segs(path):
    R = [json.loads(l) for l in open(path)]
    out, cur = [], []
    for r in R:
        if r["state"] == "FOLLOW": cur.append(r)
        elif cur: out.append(cur); cur = []
    if cur: out.append(cur)
    return [s for s in out if len(s) > 200]
SEGS = [s for p in HELDOUT for s in segs(p)]

def replay(ctrl):
    W = []
    for s in SEGS:
        if hasattr(ctrl, "reset"): ctrl.reset()
        for r in s:
            W.append(ctrl.compute(RowEstimate(heading_error=math.radians(r["heading_deg"]), lateral_offset=r["lateral"],
                     confidence=r["conf"], row_end_confidence=r.get("row_end_conf", 0.0), n_points=r.get("n", 0), valid=True))[1])
    return np.array(W)

kw = dict(max_linear=0.20, speed_conf_full=0.65)
cands = [("pure pursuit", PurePursuitController(**kw), pursuit_act_fn()),
         ("MPC (field setting)", RowMPCController(**MPC, **kw), mpc_act_fn(**MPC))]
pols = ["policies/trainers/ppo_onion_s1.npz", "policies/follow_onion.npz"] + sorted(glob.glob(str(ROOT / "policies/trainers/*_onion_nov_s*.npz")))
for p in pols:
    p = p if p.startswith("/") else str(ROOT / p)
    pol = MLPPolicy.load(p)
    cands.append((Path(p).stem, RLController(policy=pol, max_angular_slew=0.12, **kw), policy_act_fn(pol)))

cfg = profile_config("onion_nov", c_u=0.8, c_du=8.0)
seeds = list(range(50000, 50200))
print(f"held-out real replay: {sum(len(s) for s in SEGS)} FOLLOW scans from {len(SEGS)} beds of {len(HELDOUT)} run(s)")
print(f"{'controller':24s} | {'sim xtrack cm':>13s} {'sim hdg°':>8s} {'sim jerk':>8s} {'succ%':>5s} | {'real |w| xPP':>12s} {'real jerk xPP':>13s} {'real peak':>9s}")
base = None
for name, ctrl, fn in cands:
    W = replay(ctrl)
    if base is None: base = (np.abs(W).mean(), np.abs(np.diff(W)).mean())
    m = evaluate(fn, seeds, cfg)
    print(f"{name:24s} | {m['xtrack_rmse_m'][0]*100:13.2f} {m['heading_rmse_deg'][0]:8.2f} {m['control_jerk'][0]:8.4f} {m['success_rate']*100:5.1f} | "
          f"{np.abs(W).mean()/base[0]:12.2f} {np.abs(np.diff(W)).mean()/base[1]:13.2f} {np.abs(W).max():9.3f}", flush=True)
