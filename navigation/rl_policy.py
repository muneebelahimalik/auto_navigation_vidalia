#!/usr/bin/env python3
"""
rl_policy.py — Tiny numpy MLP steering policy (no torch / no autograd).

A deliberately small fully-connected network maps the FOLLOW observation
``[lateral_offset, heading_error, confidence, prev_action]`` to a single steering
action in ``[-1, 1]`` (scaled to the angular-velocity bound by the controller).

Pure numpy so it deploys on the Jetson with zero extra dependencies and runs the
forward pass in microseconds.  Parameters flatten to / from a single vector so
gradient-free trainers (evolution strategies) can optimise them; weights save to
a small ``.npz``.

Observation normalisation lives here (single source of truth) so training and
deployment encode the observation identically.
"""

from __future__ import annotations

import numpy as np

# Fixed input scaling so the raw observation lands in ~[-1, 1] for the net.
#                      lateral(m)  heading(rad)  conf  prev_action  drift_integral
_OBS_SCALE = np.array([2.0,        2.5,          1.0,  1.0,         2.0], dtype=np.float64)

# Leaky drift integrator — the RL analogue of the MPC disturbance observer.
# A memoryless [e, theta, conf, prev] policy cannot cancel a STEADY cross-slope
# drift (a constant disturbance gives constant steady-state error — it always
# sags downhill).  Feeding a slow leaky integral of the lateral offset gives the
# policy a signal proportional to accumulated drift, so it can learn a steady
# counter-steer.  The SAME update runs in the sim, the eval, and the deployed
# controller, so training == deployment.
#   eint <- clip(EINT_LEAK * eint + lateral * EINT_DT, ±EINT_CLIP)
EINT_LEAK = 0.97
EINT_DT = 0.1
EINT_CLIP = 0.5


def update_eint(eint: float, lateral: float) -> float:
    """Advance the leaky drift integrator by one step (shared sim/deploy rule)."""
    v = EINT_LEAK * float(eint) + float(lateral) * EINT_DT
    return float(np.clip(v, -EINT_CLIP, EINT_CLIP))


def encode_obs(lateral: float, heading: float, conf: float, prev_action: float,
               drift_integral: float = 0.0) -> np.ndarray:
    """Raw FOLLOW observation → normalised network input (training == deploy).

    ``drift_integral`` is optional so a 4-input (pre-integrator) policy encodes
    identically to before; the 5th slot is simply dropped by ``act`` when the
    loaded network has obs_dim 4."""
    full = np.array([lateral, heading, conf, prev_action, drift_integral],
                    dtype=np.float64) * _OBS_SCALE
    return full


class MLPPolicy:
    def __init__(self, obs_dim: int = 4, hidden=(16, 16), seed: int = 0) -> None:
        self.obs_dim = obs_dim
        self.hidden = tuple(hidden)
        self.sizes = (obs_dim, *self.hidden, 1)
        rng = np.random.default_rng(seed)
        self._W = []
        self._b = []
        for nin, nout in zip(self.sizes[:-1], self.sizes[1:]):
            # Small init keeps the initial policy near zero steering (safe-ish).
            self._W.append(rng.normal(0.0, 0.3, size=(nin, nout)) * np.sqrt(1.0 / nin))
            self._b.append(np.zeros(nout))

    # ------------------------------------------------------------------
    def act(self, raw_obs) -> float:
        """Raw observation → steering action in [-1, 1].

        Accepts a 4- or 5-vector; the scaling is sliced to the network's input
        width so a 4-input (pre-integrator) policy and a 5-input policy both
        encode correctly from the same helper."""
        ro = np.asarray(raw_obs, dtype=np.float64).reshape(-1)[:self.obs_dim]
        x = ro * _OBS_SCALE[:self.obs_dim]
        for W, b in zip(self._W[:-1], self._b[:-1]):
            x = np.tanh(x @ W + b)
        x = np.tanh(x @ self._W[-1] + self._b[-1])   # bounded output
        return float(x[0])

    # ------------------------------------------------------------------
    @property
    def n_params(self) -> int:
        return sum(W.size + b.size for W, b in zip(self._W, self._b))

    def get_params(self) -> np.ndarray:
        return np.concatenate([W.ravel() for W in self._W]
                              + [b.ravel() for b in self._b])

    def set_params(self, flat) -> "MLPPolicy":
        flat = np.asarray(flat, dtype=np.float64).ravel()
        i = 0
        for k, W in enumerate(self._W):
            n = W.size
            self._W[k] = flat[i:i + n].reshape(W.shape); i += n
        for k, b in enumerate(self._b):
            n = b.size
            self._b[k] = flat[i:i + n].reshape(b.shape); i += n
        return self

    # ------------------------------------------------------------------
    def save(self, path: str) -> None:
        np.savez(path, sizes=np.array(self.sizes), params=self.get_params())

    @classmethod
    def load(cls, path: str) -> "MLPPolicy":
        data = np.load(path)
        sizes = [int(s) for s in data["sizes"]]
        pol = cls(obs_dim=sizes[0], hidden=tuple(sizes[1:-1]))
        pol.set_params(data["params"])
        return pol
