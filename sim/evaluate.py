#!/usr/bin/env python3
"""
evaluate.py — Shared rollout + metrics for the row-follow steering task.

Both the trainer (fitness) and the comparison harness use these so a learned
policy and the pure-pursuit baseline are measured on IDENTICAL episodes (same
seeds → same slip, grade, noise and dropout draws).
"""

from __future__ import annotations

import math

import numpy as np

from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowEstimate
from sim.row_follow_env import EnvConfig, RowFollowEnv


def pursuit_act_fn(max_angular: float = 0.40):
    """Baseline steering: the real pure-pursuit controller as an env policy."""
    ctrl = PurePursuitController(max_angular=max_angular)

    def act(raw_obs) -> float:
        e, theta, conf = raw_obs[0], raw_obs[1], raw_obs[2]
        est = RowEstimate(heading_error=float(theta), lateral_offset=float(e),
                          confidence=float(conf), valid=True)
        _v, w = ctrl.compute(est)
        return float(np.clip(w / max_angular, -1.0, 1.0)) if max_angular else 0.0

    act.reset = ctrl.reset            # clear the integral between episodes
    return act


def policy_act_fn(policy):
    """A trained MLPPolicy as an env policy (carries prev-action via the obs)."""
    return lambda raw_obs: policy.act(raw_obs)


def mpc_act_fn(max_angular: float = 0.40, **mpc_kwargs):
    """The MPPI controller as an env policy (stateful: warm-start + observers).

    Returns a callable with a ``.reset()`` method so the rollout can clear the
    controller's plan/observers between episodes (`rollout` calls it if present).
    """
    from navigation.row_mpc_controller import RowMPCController

    ctrl = RowMPCController(max_angular=max_angular, **mpc_kwargs)

    def act(raw_obs) -> float:
        e, theta, conf = raw_obs[0], raw_obs[1], raw_obs[2]
        est = RowEstimate(heading_error=float(theta), lateral_offset=float(e),
                          confidence=float(conf), valid=True)
        _v, w = ctrl.compute(est)
        return float(np.clip(w / max_angular, -1.0, 1.0)) if max_angular else 0.0

    act.reset = ctrl.reset
    return act


def rollout(env: RowFollowEnv, act_fn, seed: int) -> dict:
    """One episode; returns per-episode metrics from the TRUE hidden state."""
    if hasattr(act_fn, "reset"):
        act_fn.reset()   # stateful controllers (MPC) start each episode clean
    obs = env.reset(seed=seed)
    ret = 0.0
    es, ths, acts = [], [], []
    prev_a = 0.0
    done = False
    while not done:
        a = float(np.clip(act_fn(obs), -1.0, 1.0))
        obs, r, done, info = env.step(a)
        ret += r
        es.append(info["e"]); ths.append(info["theta"]); acts.append(info["a"])
        prev_a = a
    es = np.asarray(es); ths = np.asarray(ths); acts = np.asarray(acts)
    jerk = float(np.mean(np.abs(np.diff(acts)))) if len(acts) > 1 else 0.0
    return {
        "return": ret,
        "xtrack_rmse_m": float(np.sqrt(np.mean(es ** 2))),
        "heading_rmse_deg": float(math.degrees(np.sqrt(np.mean(ths ** 2)))),
        "control_jerk": jerk,
        "success": not bool(info["off_road"]),
        "steps": len(es),
        "grade_drift": float(abs(info["grade"])),   # per-episode cross-slope (m/s)
        "slip": float(info["slip"]),                # per-episode achieved-rate fraction
    }


def per_episode(act_fn, seeds, config: EnvConfig | None = None) -> list:
    """Per-episode metric rows (one dict per seed) for CSV export / figures."""
    env = RowFollowEnv(config)
    return [dict(seed=int(s), **rollout(env, act_fn, int(s))) for s in seeds]


def evaluate(act_fn, seeds, config: EnvConfig | None = None) -> dict:
    """Aggregate metrics over a fixed seed set (mean ± std)."""
    env = RowFollowEnv(config)
    rows = [rollout(env, act_fn, int(s)) for s in seeds]
    keys = ["return", "xtrack_rmse_m", "heading_rmse_deg", "control_jerk", "steps"]
    agg = {k: (float(np.mean([r[k] for r in rows])),
               float(np.std([r[k] for r in rows]))) for k in keys}
    agg["success_rate"] = float(np.mean([r["success"] for r in rows]))
    agg["n"] = len(rows)
    return agg
