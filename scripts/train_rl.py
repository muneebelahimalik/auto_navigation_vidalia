#!/usr/bin/env -S python3 -u
"""
train_rl.py — Train the FOLLOW steering policy with Evolution Strategies (ES).

ES (Salimans et al. 2017, "Evolution Strategies as a Scalable Alternative to
Reinforcement Learning") is a gradient-free, embarrassingly-parallel RL method:
it estimates a search gradient by perturbing the policy parameters with Gaussian
noise and weighting the perturbations by the episode return they produce.  It
needs only numpy (no autograd / no torch), which matches the deployment target,
and is robust on the low-dimensional, noisy control task here.

This trains ONLY the in-row steering policy; the state machine, safety monitor
and headland turn are untouched.  The pure-pursuit baseline is reported every
few iterations so improvement (or the lack of it) is honest and visible.

Usage:
    python3 scripts/train_rl.py --iters 300 --out policies/follow.npz
    python3 scripts/eval_controller.py --policy policies/follow.npz   # compare
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.rl_policy import MLPPolicy
from sim.evaluate import evaluate, policy_act_fn, pursuit_act_fn, rollout
from sim.row_follow_env import EnvConfig, RowFollowEnv


def fitness(policy: MLPPolicy, flat, env: RowFollowEnv, seeds) -> float:
    """Mean episode return of the policy(flat) over the given eval seeds."""
    policy.set_params(flat)
    act = policy_act_fn(policy)
    return float(np.mean([rollout(env, act, int(s))["return"] for s in seeds]))


def rank_normalise(returns: np.ndarray) -> np.ndarray:
    """Map returns to centred ranks in [-0.5, 0.5] (robust to reward scale)."""
    ranks = np.empty_like(returns, dtype=np.float64)
    ranks[np.argsort(returns)] = np.arange(len(returns))
    return ranks / (len(returns) - 1) - 0.5


def main() -> None:
    ap = argparse.ArgumentParser(description="Train FOLLOW steering policy (ES)")
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--pop", type=int, default=40, help="perturbations (antithetic pairs = pop/2)")
    ap.add_argument("--sigma", type=float, default=0.10, help="perturbation std")
    ap.add_argument("--lr", type=float, default=0.05, help="step size")
    ap.add_argument("--episodes", type=int, default=8, help="eval episodes per candidate")
    ap.add_argument("--hidden", type=int, nargs="+", default=[16, 16])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default="policies/follow.npz")
    ap.add_argument("--eval-seeds", type=int, default=64, help="held-out seeds for the report")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    env = RowFollowEnv(EnvConfig())
    policy = MLPPolicy(obs_dim=RowFollowEnv.OBS_DIM, hidden=tuple(args.hidden), seed=args.seed)
    theta = policy.get_params()
    n = theta.size
    half = max(1, args.pop // 2)

    # Held-out seeds (never used to compute the ES gradient) for honest reporting.
    held = list(range(10_000, 10_000 + args.eval_seeds))
    base = evaluate(pursuit_act_fn(), held)
    print(f"params={n}  pop={2*half}  sigma={args.sigma}  lr={args.lr}")
    print(f"[baseline pure-pursuit] return={base['return'][0]:+.2f}  "
          f"xtrack_rmse={base['xtrack_rmse_m'][0]*100:.1f}cm  "
          f"heading_rmse={base['heading_rmse_deg'][0]:.1f}deg  "
          f"jerk={base['control_jerk'][0]:.3f}  success={base['success_rate']*100:.0f}%")

    best_theta, best_score = theta.copy(), -1e18
    for it in range(1, args.iters + 1):
        # Common random numbers: all candidates this iter share the eval seeds.
        seeds = rng.integers(0, 1_000_000, size=args.episodes)
        eps = rng.normal(0.0, 1.0, size=(half, n))
        scores = np.empty(2 * half)
        for i in range(half):
            scores[2 * i]     = fitness(policy, theta + args.sigma * eps[i], env, seeds)
            scores[2 * i + 1] = fitness(policy, theta - args.sigma * eps[i], env, seeds)
        # Antithetic ES gradient with rank-normalised utilities.
        util = rank_normalise(scores)
        grad = np.zeros(n)
        for i in range(half):
            grad += (util[2 * i] - util[2 * i + 1]) * eps[i]
        grad /= (2 * half * args.sigma)
        theta = theta + args.lr * grad

        if it % 10 == 0 or it == 1:
            cur = evaluate(policy_act_fn(policy.set_params(theta)), held)
            score = cur["return"][0]
            if score > best_score:
                best_score, best_theta = score, theta.copy()
            print(f"iter {it:4d}  return={score:+.2f}  "
                  f"xtrack={cur['xtrack_rmse_m'][0]*100:.1f}cm  "
                  f"heading={cur['heading_rmse_deg'][0]:.1f}deg  "
                  f"jerk={cur['control_jerk'][0]:.3f}  success={cur['success_rate']*100:.0f}%")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    policy.set_params(best_theta).save(str(out))
    print(f"\nsaved best policy -> {out}  (held-out return {best_score:+.2f} "
          f"vs baseline {base['return'][0]:+.2f})")
    print("compare in detail with:  python3 scripts/eval_controller.py "
          f"--policy {out}")


if __name__ == "__main__":
    main()
