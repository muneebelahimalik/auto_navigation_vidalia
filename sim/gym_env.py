#!/usr/bin/env python3
"""
gym_env.py — the steering simulator as a standard Gymnasium environment.

A thin wrapper around ``sim.row_follow_env.RowFollowEnv``: identical dynamics,
disturbance profiles and reward, exposed through the Gymnasium API so standard
RL libraries (Stable-Baselines3 PPO / SAC, ``scripts/train_sb3.py``) can train
on it and ``gymnasium.utils.env_checker`` can verify it.

  observation  5-vector [lateral, heading, confidence, prev_action, drift_integral],
               pre-scaled with the SAME encoding the deployed policy uses
               (``navigation.rl_policy.encode_obs``), so a trained network maps
               1:1 onto ``MLPPolicy`` for the robot (no PyTorch on the Jetson).
  action       1-vector in [-1, 1] = steering command / max_angular (0.40 rad/s).
  terminated   left the row (|lateral| > 0.60 m);  truncated: 20 s elapsed.

Episode disturbances (slip, cross-slope drift, noise, dropouts) are drawn from
an episode seed taken from the Gymnasium RNG, so ``reset(seed=…)`` makes runs
reproducible; ``reset(options={"episode_seed": s})`` replays exactly the
episode ``RowFollowEnv.reset(seed=s)`` would — the held-out evaluation seeds
used for every controller in the paper.
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from navigation.rl_policy import encode_obs
from sim.row_follow_env import EnvConfig, RowFollowEnv, onion_bed_config

PROFILES = ("soybean", "onion")
OBS_BOUND = 10.0


def make_config(profile: str = "soybean", **reward_weights) -> EnvConfig:
    """Disturbance profile + optional reward-weight overrides (c_e, c_theta, c_u, c_du)."""
    if profile not in PROFILES:
        raise ValueError(f"profile must be one of {PROFILES}")
    return onion_bed_config(**reward_weights) if profile == "onion" else EnvConfig(**reward_weights)


class RowFollowGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, profile: str = "soybean", **reward_weights) -> None:
        super().__init__()
        self.profile = profile
        self.core = RowFollowEnv(make_config(profile, **reward_weights))
        # Scaled inputs are ~[-1.5, 1.5] in practice; ±OBS_BOUND is a no-op clip.
        self.observation_space = spaces.Box(-OBS_BOUND, OBS_BOUND,
                                            shape=(RowFollowEnv.OBS_DIM,), dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(1,), dtype=np.float32)

    @staticmethod
    def _enc(obs: np.ndarray) -> np.ndarray:
        return np.clip(encode_obs(*obs), -OBS_BOUND, OBS_BOUND).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options and "episode_seed" in options:
            ep = int(options["episode_seed"])
        else:
            ep = int(self.np_random.integers(0, 2 ** 31 - 1))
        obs = self.core.reset(seed=ep)
        return self._enc(obs), {"episode_seed": ep}

    def step(self, action):
        obs, reward, done, info = self.core.step(np.asarray(action, dtype=np.float64))
        terminated = bool(info["off_road"])
        truncated = bool(done and not terminated)
        return self._enc(obs), float(reward), terminated, truncated, info
