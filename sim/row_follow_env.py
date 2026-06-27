#!/usr/bin/env python3
"""
row_follow_env.py — Gym-style simulator for the FOLLOW steering task.

This environment models ONLY the part of the stack a learned policy is allowed
to touch: the in-row steering command (angular velocity) during FOLLOW.  The
state machine, safety monitor and headland turn are out of scope.

Dynamics (row centreline = world +Y; signs match navigation.RowEstimate):
    theta = heading_error  (+ve: row angled to the robot's right)
    e     = lateral_offset (+ve: row is to the robot's right)
    e_dot     = v * sin(theta) + grade_drift          # lateral closes via heading
    theta_dot = w_cmd * slip + process_noise          # skid-steer UNDER-rotates

The realism that makes a learned policy potentially useful (and the sim honest):
  * skid-steer slip   — commanded angular rate is only partly achieved (per
    episode `slip` < 1, matching the field U-turn finding);
  * cross-slope drift — a per-episode lateral `grade_drift` the controller must
    actively cancel (the source of the slope weaving seen in the logs);
  * sensor noise + dropout — the observation is a noisy `(e, theta)` and on some
    steps the row fix drops (low confidence, observation frozen), like the empty
    VLP-16 crop-ROI scans.

The forward speed is the SAME formula as PurePursuitController, so a policy only
ever learns steering and is compared against pure-pursuit on identical dynamics.

Reward (per step): forward progress along the row, minus quadratic costs on
lateral error, heading error, control effort and control *jerk* (the jerk term
directly targets smoothness / weaving).  Going off-row terminates with a
penalty.

API mirrors Gym without depending on it::
    obs = env.reset(seed=0)
    obs, reward, done, info = env.step(action)   # action in [-1, 1]
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# Forward-speed constants — identical to PurePursuitController so steering is
# the only difference between a learned policy and the baseline.
_MAX_LINEAR = 0.30
_MIN_LINEAR = 0.08


def speed_from(conf: float, theta_meas: float) -> float:
    """The pure-pursuit forward-speed law (shared with the real controller)."""
    speed_factor = conf * max(0.25, 1.0 - abs(theta_meas) / 1.0)
    return max(_MIN_LINEAR, _MAX_LINEAR * speed_factor)


@dataclass
class EnvConfig:
    dt: float = 0.1
    max_steps: int = 200            # 20 s episodes
    w_max: float = 0.40             # rad/s command bound (matches controller)
    e_limit: float = 0.60           # |lateral| beyond this = off-row (terminate)
    # disturbances (per-episode draws)
    slip_mean: float = 0.75         # achieved angular rate fraction
    slip_std: float = 0.12
    grade_drift_std: float = 0.035  # m/s cross-slope lateral drift (1-sigma)
    # sensor model
    e_noise: float = 0.02           # m
    theta_noise: float = 0.02       # rad
    dropout_p: float = 0.08         # prob the row fix drops this step
    conf_hi: float = 0.92
    conf_lo: float = 0.20
    # reward weights.  A per-step ALIVE bonus dominates the tracking costs, so
    # the only way to maximise return is to stay on the row for the whole
    # episode AND minimise offset/heading/jerk — leaving off-road (which forfeits
    # all remaining alive bonuses) is never beneficial.  Without this, the agent
    # reward-hacks by driving off-row early to stop paying per-step costs.
    alive_bonus: float = 0.5
    c_e: float = 2.0
    c_theta: float = 0.5
    c_u: float = 0.05
    c_du: float = 0.30
    k_progress: float = 2.0
    offroad_penalty: float = 5.0


class RowFollowEnv:
    OBS_DIM = 4   # [lateral, heading, confidence, prev_action]
    ACT_DIM = 1   # angular command in [-1, 1]

    def __init__(self, config: EnvConfig | None = None) -> None:
        self.cfg = config or EnvConfig()
        self._rng = np.random.default_rng(0)
        self._reset_state()

    # ------------------------------------------------------------------
    def _reset_state(self) -> None:
        self._e = 0.0
        self._theta = 0.0
        self._prev_a = 0.0
        self._step = 0
        self._slip = self.cfg.slip_mean
        self._grade = 0.0
        self._frozen_obs = None

    # ------------------------------------------------------------------
    def reset(self, seed: int | None = None) -> np.ndarray:
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        c = self.cfg
        self._e = float(self._rng.uniform(-0.30, 0.30))
        self._theta = float(self._rng.uniform(-0.20, 0.20))
        self._prev_a = 0.0
        self._step = 0
        self._slip = float(np.clip(self._rng.normal(c.slip_mean, c.slip_std), 0.4, 1.0))
        self._grade = float(self._rng.normal(0.0, c.grade_drift_std))
        self._frozen_obs = None
        return self._observe(conf=c.conf_hi)

    # ------------------------------------------------------------------
    def _observe(self, conf: float) -> np.ndarray:
        c = self.cfg
        e_m = self._e + self._rng.normal(0.0, c.e_noise)
        th_m = self._theta + self._rng.normal(0.0, c.theta_noise)
        return np.array([e_m, th_m, conf, self._prev_a], dtype=np.float64)

    # ------------------------------------------------------------------
    def step(self, action) -> tuple[np.ndarray, float, bool, dict]:
        c = self.cfg
        a = float(np.clip(np.asarray(action).reshape(-1)[0], -1.0, 1.0))
        w_cmd = a * c.w_max

        # Sensor: occasional dropout (row fix lost → low conf, stale obs).
        dropout = self._rng.random() < c.dropout_p
        conf = c.conf_lo if dropout else c.conf_hi
        if dropout and self._frozen_obs is not None:
            obs_for_speed = self._frozen_obs
        else:
            obs_for_speed = self._observe(conf=conf)
            self._frozen_obs = obs_for_speed
        theta_meas = obs_for_speed[1]

        v = speed_from(conf, theta_meas)

        # Dynamics (true state).
        w_actual = w_cmd * self._slip + self._rng.normal(0.0, 0.02)
        self._theta += w_actual * c.dt
        self._e += (v * np.sin(self._theta) + self._grade) * c.dt
        self._step += 1

        # Reward: stay-alive bonus + forward progress − quadratic tracking costs
        # (incl. control jerk for smoothness).
        progress = c.k_progress * v * np.cos(self._theta) * c.dt
        cost = (c.c_e * self._e ** 2 + c.c_theta * self._theta ** 2
                + c.c_u * a ** 2 + c.c_du * (a - self._prev_a) ** 2)
        reward = c.alive_bonus + progress - cost

        off_road = bool(abs(self._e) > c.e_limit)
        if off_road:
            reward -= c.offroad_penalty
        done = bool(off_road or self._step >= c.max_steps)

        self._prev_a = a
        obs = obs_for_speed if dropout else self._observe(conf=conf)
        info = {"e": float(self._e), "theta": float(self._theta), "v": float(v),
                "a": a, "off_road": off_road, "slip": self._slip, "grade": self._grade}
        return obs, float(reward), bool(done), info
