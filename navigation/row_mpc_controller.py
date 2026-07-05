#!/usr/bin/env python3
"""
row_mpc_controller.py — Sampling-based Model-Predictive Control (MPPI) steering,
drop-in for PurePursuitController, with an online disturbance observer.

Why this is the state-of-the-art upgrade over pure pursuit
----------------------------------------------------------
Pure pursuit is fundamentally a geometric law.  The baseline now carries a
bounded leaky-integral term (see row_controller.py) that cancels much of a
steady cross-slope drift, but it has no model of the robot and no PREVIEW: it
still reacts one step at a time and its integral authority is deliberately
capped for stability.

This controller goes further on both counts:

  1. **MPPI** (Model-Predictive Path Integral control; Williams et al. 2017,
     "Information-Theoretic MPC") — a sampling-based MPC.  Each step it samples
     K candidate steering sequences over an H-step horizon, rolls each through a
     kinematic row-tracking model, scores them by a quadratic tracking cost, and
     returns the cost-weighted (soft-argmin) average first action.  It is
     gradient-free, needs only numpy, and — unlike pure pursuit — it *previews*
     the consequences of steering over the horizon and balances offset, heading
     and control effort/jerk in one optimisation.

  2. **Online disturbance observer** — a scalar Luenberger estimate ``d_hat`` of
     the persistent cross-track drift (slope, mis-calibration), updated from the
     residual between the model-predicted lateral motion and the measured one.
     MPPI rolls its predictions WITH ``d_hat``, so the optimiser actively steers
     to cancel the slope drift instead of lagging behind it — the integral
     action pure pursuit lacks.  A gentle slip estimate on the heading channel
     does the same for skid-steer under-rotation.

Safety / authenticity by construction (identical contract to RLController):
  * Forward speed and the low-confidence behaviour come straight from
    PurePursuitController — MPC only ever sets the STEERING.
  * Below ``min_confidence`` (or invalid fix) it falls back to pure pursuit and
    holds the observers, so MPC never acts on a bad observation.
  * The steering output is hard-clamped to ``max_angular``.
  * It is a bounded refinement on a known-good geometric controller, opt-in via
    ``--controller mpc``; the default remains pure pursuit.

Interface: ``compute(est) -> (linear, angular)`` plus ``reset()`` and a
``min_confidence`` attribute, so the navigator and the sim use it unchanged.
"""

from __future__ import annotations

import numpy as np

from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowEstimate


class RowMPCController:
    def __init__(
        self,
        *,
        max_linear: float = 0.30,
        min_linear: float = 0.08,
        max_angular: float = 0.40,
        lookahead: float = 2.0,
        min_confidence: float = 0.35,
        # MPPI hyper-parameters
        horizon: int = 20,            # H steps of preview
        n_samples: int = 200,         # K sampled control sequences
        dt: float = 0.1,              # model step (matches sim / scan period)
        temperature: float = 0.6,     # MPPI lambda (lower = greedier soft-argmin)
        noise_sigma: float = 0.35,    # exploration std on the normalised action
        # cost weights (on the predicted row-tracking state)
        q_e: float = 6.0,             # lateral-offset cost
        q_theta: float = 1.5,         # heading-error cost
        r_u: float = 0.05,            # control-effort cost
        r_du: float = 0.30,           # control-jerk cost (smoothness)
        # online observers
        slip_nominal: float = 0.75,   # prior achieved-rate fraction
        drift_gain: float = 0.20,     # disturbance-observer learning rate
        slip_gain: float = 0.05,      # slip-observer learning rate
        seed: int = 0,
    ) -> None:
        # Reused for forward speed + as the low-confidence / startup fallback.
        self.pursuit = PurePursuitController(
            max_linear=max_linear, min_linear=min_linear, max_angular=max_angular,
            lookahead=lookahead, min_confidence=min_confidence)
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.min_confidence = min_confidence
        self.lookahead = lookahead

        self.H = int(horizon)
        self.K = int(n_samples)
        self.dt = float(dt)
        self.lam = float(temperature)
        self.sigma = float(noise_sigma)
        self.q_e = float(q_e)
        self.q_theta = float(q_theta)
        self.r_u = float(r_u)
        self.r_du = float(r_du)
        self.slip_nominal = float(slip_nominal)
        self.drift_gain = float(drift_gain)
        self.slip_gain = float(slip_gain)

        self._rng = np.random.default_rng(seed)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear the warm-start plan and the online estimates (new row/run)."""
        self._u_nom = np.zeros(self.H)     # warm-started nominal action sequence
        self._prev_a = 0.0
        self._d_hat = 0.0                  # estimated cross-slope drift (m/s)
        self._slip_hat = self.slip_nominal  # estimated achieved-rate fraction
        self._last = None                  # (e, theta, w_cmd) for the observers
        self.pursuit.reset()               # clear the baseline's integral too

    # ------------------------------------------------------------------
    def _update_observers(self, e: float, theta: float) -> None:
        """Luenberger updates for the drift and slip from the last step's residual."""
        if self._last is None:
            return
        e_prev, th_prev, w_cmd, v_prev = self._last
        # Lateral channel: measured Δe vs model Δe (without drift) → drift residual.
        de_meas = e - e_prev
        de_model = v_prev * np.sin(th_prev) * self.dt
        self._d_hat += self.drift_gain * (de_meas - de_model - self._d_hat * self.dt) / max(self.dt, 1e-3)
        self._d_hat = float(np.clip(self._d_hat, -0.20, 0.20))
        # Heading channel: measured Δθ vs commanded w·slip → slip residual.
        # (row-relative heading is small, so no angle wrapping is needed.)
        if abs(w_cmd) > 1e-3:
            dth_meas = theta - th_prev
            slip_meas = float(np.clip(dth_meas / (w_cmd * self.dt), 0.2, 1.2))
            self._slip_hat += self.slip_gain * (slip_meas - self._slip_hat)
            self._slip_hat = float(np.clip(self._slip_hat, 0.3, 1.1))

    # ------------------------------------------------------------------
    def _mppi(self, e0: float, theta0: float, v: float) -> float:
        """Return the MPPI-optimal first normalised action in [-1, 1]."""
        K, H, dt = self.K, self.H, self.dt
        wmax = self.max_angular
        slip = self._slip_hat
        d = self._d_hat

        # Sample K action-sequence perturbations around the warm-started nominal.
        noise = self._rng.normal(0.0, self.sigma, size=(K, H))
        u = np.clip(self._u_nom[None, :] + noise, -1.0, 1.0)   # (K, H)

        # Roll out the kinematic row-tracking model for all samples in parallel.
        e = np.full(K, e0)
        th = np.full(K, theta0)
        prev = np.full(K, self._prev_a)
        cost = np.zeros(K)
        for t in range(H):
            a_t = u[:, t]
            w_act = a_t * wmax * slip
            th = th + w_act * dt
            e = e + (v * np.sin(th) + d) * dt
            cost += (self.q_e * e * e + self.q_theta * th * th
                     + self.r_u * a_t * a_t + self.r_du * (a_t - prev) ** 2)
            prev = a_t

        # Path-integral (soft-argmin) weighting → updated nominal sequence.
        beta = cost.min()
        wts = np.exp(-(cost - beta) / max(self.lam, 1e-6))
        wts /= wts.sum() + 1e-9
        self._u_nom = np.clip((wts[:, None] * u).sum(axis=0), -1.0, 1.0)
        return float(self._u_nom[0])

    # ------------------------------------------------------------------
    def compute(self, est: RowEstimate) -> tuple[float, float]:
        v, w_pp = self.pursuit.compute(est)
        # Weak / invalid fix → pure-pursuit; hold the plan and observers steady.
        if est.confidence < self.min_confidence or not est.valid:
            self._prev_a = (w_pp / self.max_angular) if self.max_angular else 0.0
            self._last = None
            return v, w_pp

        e0 = float(est.lateral_offset)
        theta0 = float(est.heading_error)
        self._update_observers(e0, theta0)

        a = self._mppi(e0, theta0, max(v, 1e-3))
        a = float(np.clip(a, -1.0, 1.0))
        w = float(np.clip(a * self.max_angular, -self.max_angular, self.max_angular))

        # Warm-start: shift the nominal plan forward one step for the next call.
        self._u_nom = np.concatenate([self._u_nom[1:], self._u_nom[-1:]])
        self._last = (e0, theta0, a * self.max_angular, max(v, 1e-3))
        self._prev_a = a
        return v, w
