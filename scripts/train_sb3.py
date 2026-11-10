#!/usr/bin/env -S python3 -u
"""
train_sb3.py — Train the steering policy with standard deep-RL algorithms
(Stable-Baselines3 PPO or SAC, PyTorch) on the Gymnasium version of the
steering simulator, and export it to the robot's NumPy policy format.

Same task, reward, network and selection protocol as the Evolution-Strategies
trainer (``scripts/train_rl.py``), so the two are directly comparable:

  * environment  ``sim.gym_env.RowFollowGymEnv`` (identical dynamics/reward,
                 ``--profile soybean|onion``, reward weights ``--c-u`` etc.)
  * actor        5 → 16 → 16 → 1, tanh hidden units — exactly the deployed
                 ``MLPPolicy`` (385 parameters); the critic(s) are separate
                 networks used only during training
  * selection    every ``--eval-every`` steps the actor is exported to NumPy
                 and evaluated on the validation seeds 10000–10063 (never
                 used for training); the best export is kept
  * test         ``scripts/eval_controller.py --profile … --policy <out>``
                 on seeds 50000+ (identical per-seed disturbances for every
                 controller)

Export is exact: the deterministic PPO action is the Gaussian mean clipped to
[-1, 1] (``out_act="clip"``); the deterministic SAC action is tanh(mean)
(``out_act="tanh"``).  ``tests/test_gym_sb3.py`` checks the NumPy export
reproduces the PyTorch actor to ~1e-6.  The robot needs no PyTorch.

Usage:
    python3 scripts/train_sb3.py --algo ppo --profile soybean --c-u 0.8 --c-du 8.0 \\
        --seed 0 --timesteps 1000000 --out policies/sb3/ppo_soybean_s0.npz
Writes <out> and <out stem>_curve.csv (timesteps, wall-clock s, validation
return / cross-track / heading / jerk / success).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np                                                  # noqa: E402
import torch                                                        # noqa: E402
from stable_baselines3 import PPO, SAC                              # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback         # noqa: E402
from stable_baselines3.common.env_util import make_vec_env          # noqa: E402

from navigation.rl_policy import MLPPolicy                          # noqa: E402
from sim.evaluate import evaluate, policy_act_fn                    # noqa: E402
from sim.gym_env import RowFollowGymEnv, make_config                # noqa: E402
from sim.row_follow_env import RowFollowEnv                         # noqa: E402

VALIDATION_SEEDS = list(range(10_000, 10_064))   # same as train_rl.py
HIDDEN = (16, 16)


def _linears(seq) -> list:
    return [m for m in seq if isinstance(m, torch.nn.Linear)]


def export_actor(model) -> MLPPolicy:
    """Copy the SB3 actor's weights into the robot's NumPy MLPPolicy."""
    if isinstance(model, PPO):
        hidden = _linears(model.policy.mlp_extractor.policy_net)
        out = model.policy.action_net
        out_act = "clip"
    elif isinstance(model, SAC):
        hidden = _linears(model.actor.latent_pi)
        out = model.actor.mu
        out_act = "tanh"
    else:
        raise TypeError(type(model))
    layers = hidden + [out]
    pol = MLPPolicy(obs_dim=RowFollowEnv.OBS_DIM, hidden=HIDDEN, out_act=out_act)
    if len(layers) != len(pol._W):
        raise ValueError("actor architecture does not match MLPPolicy")
    for k, lin in enumerate(layers):
        pol._W[k] = lin.weight.detach().cpu().numpy().astype(np.float64).T.copy()
        pol._b[k] = lin.bias.detach().cpu().numpy().astype(np.float64).copy()
    return pol


def validate(pol: MLPPolicy, cfg) -> dict:
    agg = evaluate(policy_act_fn(pol), VALIDATION_SEEDS, cfg)
    return {"return": agg["return"][0], "xtrack_cm": agg["xtrack_rmse_m"][0] * 100,
            "heading_deg": agg["heading_rmse_deg"][0], "jerk": agg["control_jerk"][0],
            "effort": agg["control_effort"][0], "success": agg["success_rate"]}


class ValidateAndKeepBest(BaseCallback):
    def __init__(self, cfg, every: int, out: Path, curve: Path) -> None:
        super().__init__()
        self.cfg, self.every, self.out, self.curve = cfg, every, out, curve
        self.best = -1e18
        self.t0 = time.time()
        self._next = 0
        self.rows = []

    def _check(self) -> None:
        pol = export_actor(self.model)
        m = validate(pol, self.cfg)
        m.update(timesteps=int(self.num_timesteps), wall_s=round(time.time() - self.t0, 1))
        self.rows.append(m)
        star = ""
        if m["return"] > self.best:
            self.best = m["return"]
            pol.save(str(self.out))
            star = "  *best"
        print(f"  {m['timesteps']:>9d} steps {m['wall_s']:7.0f}s | val return {m['return']:+7.2f}"
              f" | xtrack {m['xtrack_cm']:5.1f} cm | heading {m['heading_deg']:4.1f}°"
              f" | jerk {m['jerk']:.3f} | success {m['success']*100:5.1f}%{star}")
        with open(self.curve, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)

    def _on_training_start(self) -> None:
        self._check()                       # untrained policy = step 0 of the curve
        self._next = self.every

    def _on_step(self) -> bool:
        if self.num_timesteps >= self._next:
            self._check()
            self._next += self.every
        return True

    def _on_training_end(self) -> None:
        if not self.rows or self.rows[-1]["timesteps"] != self.num_timesteps:
            self._check()


def build(algo: str, env, seed: int, explore: str = "gsde"):
    """``explore`` (PPO): ``"gsde"`` = generalized State-Dependent Exploration
    (Raffin et al., 2022) with initial std e^-2 — smooth, temporally correlated
    exploration noise; ``"default"`` = SB3's independent Gaussian noise, std 1.
    With the deployed reward (jerk cost 8·Δa² per step vs a +0.5 alive bonus)
    the default noise alone costs ~6 per step, so PPO learned that leaving the
    row early pays — validation return fell (kept in
    results/trainers/ppo_default_noise_failed/)."""
    if algo == "ppo":
        sde = dict(use_sde=True, sde_sample_freq=4) if explore == "gsde" else {}
        return PPO("MlpPolicy", env, seed=seed, device="cpu", verbose=0,
                   n_steps=1024, batch_size=256, n_epochs=10, gamma=0.99,
                   gae_lambda=0.95, learning_rate=3e-4, ent_coef=0.0, clip_range=0.2,
                   **sde,
                   policy_kwargs=dict(net_arch=dict(pi=list(HIDDEN), vf=[64, 64]),
                                      activation_fn=torch.nn.Tanh,
                                      log_std_init=-2.0 if explore == "gsde" else 0.0))
    return SAC("MlpPolicy", env, seed=seed, device="cpu", verbose=0,
               buffer_size=300_000, learning_starts=5_000, batch_size=256,
               gamma=0.99, tau=0.005, learning_rate=3e-4, train_freq=1,
               gradient_steps=1, ent_coef="auto",
               policy_kwargs=dict(net_arch=dict(pi=list(HIDDEN), qf=[64, 64]),
                                  activation_fn=torch.nn.Tanh))


def main() -> None:
    ap = argparse.ArgumentParser(description="Train steering with Stable-Baselines3 PPO/SAC")
    ap.add_argument("--algo", choices=["ppo", "sac"], required=True)
    ap.add_argument("--profile", choices=["soybean", "onion"], default="soybean")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--timesteps", type=int, default=1_000_000)
    ap.add_argument("--n-envs", type=int, default=8, help="parallel envs (PPO only)")
    ap.add_argument("--eval-every", type=int, default=50_000)
    ap.add_argument("--explore", choices=["gsde", "default"], default="gsde",
                    help="PPO exploration noise (see build()); SAC uses its default")
    ap.add_argument("--c-e", type=float, default=2.0)
    ap.add_argument("--c-theta", type=float, default=0.5)
    ap.add_argument("--c-u", type=float, default=0.05)
    ap.add_argument("--c-du", type=float, default=0.30)
    ap.add_argument("--out", required=True)
    ap.add_argument("--curve-csv", default="", help="default: <out stem>_curve.csv")
    args = ap.parse_args()

    torch.set_num_threads(1)
    weights = dict(c_e=args.c_e, c_theta=args.c_theta, c_u=args.c_u, c_du=args.c_du)
    cfg = make_config(args.profile, **weights)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    curve = Path(args.curve_csv) if args.curve_csv else out.with_name(out.stem + "_curve.csv")
    curve.parent.mkdir(parents=True, exist_ok=True)

    n_envs = args.n_envs if args.algo == "ppo" else 1
    env = make_vec_env(RowFollowGymEnv, n_envs=n_envs, seed=args.seed,
                       env_kwargs=dict(profile=args.profile, **weights))
    model = build(args.algo, env, args.seed, args.explore)
    print(f"[{args.algo.upper()} | profile {args.profile} | seed {args.seed} | "
          f"{args.timesteps} steps | reward {weights} | explore {args.explore}]")
    cb = ValidateAndKeepBest(cfg, args.eval_every, out, curve)
    model.learn(total_timesteps=args.timesteps, callback=cb)
    print(f"\nsaved best policy -> {out}  (validation return {cb.best:+.2f}); curve -> {curve}")
    print(f"Test it: python3 scripts/eval_controller.py --profile {args.profile} "
          f"--policy {out} --mpc")


if __name__ == "__main__":
    main()
