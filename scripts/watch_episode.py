#!/usr/bin/env python3
"""
watch_episode.py — SEE the steering controllers drive one simulated row.

Runs ONE held-out test episode (same seed = identical slip, cross-slope drift,
sensor noise and dropouts for every controller) with pure pursuit, MPC and
the learned policies (ES / PPO / SAC exported .npz files), and plots, from the
TRUE hidden state:
  * top-down path: lateral offset from the row centre vs distance along the row
    (the shaded band is the off-row limit — touching it ends the episode),
  * heading error, and
  * the steering command each controller sent (smooth vs twitchy is visible).
With --gif it also writes an animation of the robots driving down the row.

    python3 scripts/watch_episode.py                       # soybean, test seed 50000
    python3 scripts/watch_episode.py --profile onion --seed 50017
    python3 scripts/watch_episode.py --hardest              # steepest cross-slope test seed
    python3 scripts/watch_episode.py --hardest --gif        # + animation
    python3 scripts/watch_episode.py --show                 # open a window (dev PC)

Needs numpy + matplotlib (the learned policies are NumPy; no PyTorch needed).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.rl_policy import MLPPolicy                              # noqa: E402
from sim.evaluate import mpc_act_fn, policy_act_fn, pursuit_act_fn      # noqa: E402
from sim.gym_env import make_config                                     # noqa: E402
from sim.row_follow_env import RowFollowEnv                             # noqa: E402

DEPLOYED = {"soybean": "policies/follow_field.npz", "onion": "policies/follow_onion.npz"}
MPC_KW = {"soybean": {}, "onion": dict(noise_sigma=0.20, temperature=0.3, drift_gain=0.05)}
COLORS = {"pure pursuit": "#555555", "MPC": "#e08a1e", "ES (deployed)": "#2a6fdb",
          "PPO": "#d1495b", "SAC": "#2e9e6a"}


def run(env: RowFollowEnv, act_fn, seed: int) -> dict:
    if hasattr(act_fn, "reset"):
        act_fn.reset()
    obs = env.reset(seed=seed)
    tr = {k: [] for k in ("s", "e", "theta", "a", "e_meas")}
    s, done, info = 0.0, False, {}
    tr["s"].append(0.0); tr["e"].append(env._e); tr["theta"].append(env._theta)
    tr["a"].append(0.0); tr["e_meas"].append(float(obs[0]))
    while not done:
        a = float(np.clip(act_fn(obs), -1.0, 1.0))
        obs, _r, done, info = env.step(a)
        s += info["v"] * np.cos(info["theta"]) * env.cfg.dt
        tr["s"].append(s); tr["e"].append(info["e"]); tr["theta"].append(info["theta"])
        tr["a"].append(info["a"]); tr["e_meas"].append(float(obs[0]))
    out = {k: np.array(v) for k, v in tr.items()}
    out["off_road"] = bool(info.get("off_road"))
    out["grade"], out["slip"] = info.get("grade", 0.0), info.get("slip", 1.0)
    return out


def controllers(profile: str, pol_dir: Path, train_seed: int) -> dict:
    ctl = {"pure pursuit": pursuit_act_fn(), "MPC": mpc_act_fn(**MPC_KW[profile])}
    dep = Path(DEPLOYED[profile])
    if dep.exists():
        ctl["ES (deployed)"] = policy_act_fn(MLPPolicy.load(str(dep)))
    for algo, name in (("ppo", "PPO"), ("sac", "SAC")):
        p = pol_dir / f"{algo}_{profile}_s{train_seed}.npz"
        if p.exists():
            ctl[name] = policy_act_fn(MLPPolicy.load(str(p)))
    return ctl


def main() -> None:
    ap = argparse.ArgumentParser(description="Watch controllers drive one simulated row")
    ap.add_argument("--profile", choices=["soybean", "onion"], default="soybean")
    ap.add_argument("--seed", type=int, default=50_000, help="test episode seed")
    ap.add_argument("--hardest", action="store_true",
                    help="pick the test seed (50000–50199) with the steepest cross-slope drift")
    ap.add_argument("--train-seed", type=int, default=0, help="which PPO/SAC training seed")
    ap.add_argument("--policies", default="policies/trainers")
    ap.add_argument("--out", default="results/watch")
    ap.add_argument("--gif", action="store_true", help="also write an animation")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    args = ap.parse_args()

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = make_config(args.profile, c_u=0.8, c_du=8.0)
    env = RowFollowEnv(cfg)
    ctl = controllers(args.profile, Path(args.policies), args.train_seed)
    seed = args.seed
    if args.hardest:
        # Steepest cross-slope that at least one controller can still drive
        # (the very steepest test seeds are lost by every controller).
        grades = []
        for s in range(50_000, 50_200):
            env.reset(seed=s)
            grades.append((abs(env._grade), s))
        for _g, s in sorted(grades, reverse=True):
            if any(not run(env, fn, s)["off_road"] for fn in ctl.values()):
                seed = s
                break

    runs = {name: run(env, fn, seed) for name, fn in ctl.items()}
    first = next(iter(runs.values()))
    title = (f"{args.profile} profile, test episode {seed}: cross-slope drift "
             f"{first['grade'] * 100:+.1f} cm/s, wheel slip {first['slip']:.2f}")
    print(title)
    for name, r in runs.items():
        print(f"  {name:14s} cross-track RMSE {np.sqrt(np.mean(r['e'] ** 2)) * 100:5.1f} cm"
              f" | max {np.max(np.abs(r['e'])) * 100:5.1f} cm"
              f" | steering jerk {np.mean(np.diff(r['a']) ** 2):.4f}"
              f" | {'LEFT THE ROW' if r['off_road'] else 'stayed on row'}")

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    lim = cfg.e_limit * 100
    fig, ax = plt.subplots(3, 1, figsize=(10, 8.5), sharex=True,
                           gridspec_kw={"height_ratios": [2.2, 1, 1]})
    ax[0].axhspan(lim, lim * 1.4, color="#f3c4c4", lw=0)
    ax[0].axhspan(-lim * 1.4, -lim, color="#f3c4c4", lw=0)
    ax[0].axhline(0, color="k", lw=0.8)
    for name, r in runs.items():
        c = COLORS.get(name, None)
        ax[0].plot(r["s"], r["e"] * 100, color=c, lw=1.6, label=name)
        ax[1].plot(r["s"], np.degrees(r["theta"]), color=c, lw=1.0)
        ax[2].plot(r["s"], r["a"] * cfg.w_max, color=c, lw=1.0)
    ax[0].plot(first["s"], first["e_meas"] * 100, color="0.6", lw=0.5, alpha=0.6,
               label="what the sensor reported (pursuit run)")
    ax[0].set_ylim(-lim * 1.4, lim * 1.4)
    ax[0].set_ylabel("offset from row centre (cm)\n+ = right")
    ax[1].set_ylabel("heading error (°)")
    ax[2].set_ylabel("steering ω (rad/s)")
    ax[2].set_xlabel("distance along the row (m)")
    ax[0].set_title(title, fontsize=10)
    ax[0].legend(fontsize=8, ncol=3, loc="upper right")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    png = out / f"episode_{args.profile}_{seed}.png"
    fig.savefig(png, dpi=150)
    print(f"figure -> {png}")

    if args.gif:
        from matplotlib.animation import FuncAnimation, PillowWriter
        n = max(len(r["s"]) for r in runs.values())
        step = 3
        fig2, axg = plt.subplots(figsize=(10, 4))
        axg.axhspan(lim, lim * 1.4, color="#f3c4c4", lw=0)
        axg.axhspan(-lim * 1.4, -lim, color="#f3c4c4", lw=0)
        axg.axhline(0, color="k", lw=0.8, ls="--")
        axg.set_ylim(-lim * 1.4, lim * 1.4)
        axg.set_xlim(0, max(r["s"][-1] for r in runs.values()))
        axg.set_xlabel("distance along the row (m)")
        axg.set_ylabel("offset (cm)")
        axg.set_title(title, fontsize=9)
        lines, dots = {}, {}
        for name in runs:
            c = COLORS.get(name, None)
            lines[name], = axg.plot([], [], color=c, lw=1.4, label=name)
            dots[name], = axg.plot([], [], "o", color=c, ms=7)
        axg.legend(fontsize=8, ncol=5, loc="upper right")

        def frame(i):
            for name, r in runs.items():
                k = min(i * step, len(r["s"]) - 1)
                lines[name].set_data(r["s"][:k + 1], r["e"][:k + 1] * 100)
                dots[name].set_data([r["s"][k]], [r["e"][k] * 100])
            return [*lines.values(), *dots.values()]

        anim = FuncAnimation(fig2, frame, frames=n // step + 1, blit=True)
        gif = out / f"episode_{args.profile}_{seed}.gif"
        anim.save(gif, writer=PillowWriter(fps=20))
        print(f"animation -> {gif}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
