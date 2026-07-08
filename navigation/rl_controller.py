#!/usr/bin/env python3
"""
rl_controller.py — Optional learned steering, drop-in for PurePursuitController.

``RLController.compute(est)`` has the SAME signature as the pure-pursuit
controller, so the navigator uses it without any change to the state machine,
safety monitor or headland logic.

Safety/authenticity by construction:
  * The forward speed and the low-confidence behaviour come straight from the
    pure-pursuit controller — the policy only ever sets the STEERING.
  * Below ``min_confidence`` (weak/lost row fix) it falls back to pure-pursuit,
    so the learned policy never acts on a bad observation.
  * The steering output is hard-clamped to ``max_angular``.
  * With no policy loaded it IS pure-pursuit — a missing/corrupt policy file can
    never disable the robot.

The learned policy is therefore a bounded refinement on top of a known-good
geometric controller, never a replacement for the safety architecture.
"""

from __future__ import annotations

from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowEstimate
from navigation.rl_policy import MLPPolicy, update_eint


class RLController:
    def __init__(
        self,
        policy: MLPPolicy | None = None,
        *,
        max_linear: float = 0.30,
        min_linear: float = 0.08,
        max_angular: float = 0.40,
        lookahead: float = 2.0,
        min_confidence: float = 0.35,
        residual: bool = False,
        residual_scale: float = 0.5,
        max_angular_slew: float = 0.0,
    ) -> None:
        # Reused for forward speed + as the low-confidence / no-policy fallback,
        # AND (in residual mode) as the base steering the policy corrects on top of.
        self.pursuit = PurePursuitController(
            max_linear=max_linear, min_linear=min_linear, max_angular=max_angular,
            lookahead=lookahead, min_confidence=min_confidence)
        self.policy = policy
        self.max_angular = max_angular
        self.min_confidence = min_confidence
        self.lookahead = lookahead
        # Residual mode: the policy outputs a DELTA added to the geometric+integral
        # baseline steering (bounded, strictly safer), instead of replacing it.
        self.residual = residual
        self.residual_scale = residual_scale
        # Angular SLEW-RATE limit (rad/s of change per control step, ~0.1 s).
        # Caps how fast the steering command can move between scans, so the
        # policy physically CANNOT produce the violent left-right thrash seen in
        # the field (a learned policy reacts fastest, so it amplified the
        # dense-canopy heading runaway into an over-correction that stalled the
        # robot).  This is a hard field-safety bound independent of the policy
        # and its input — even a bad policy or a noisy heading can only turn the
        # wheels so fast.  0 = disabled (default; preserves existing behaviour).
        # Applied to the FINAL output on every path (fallback included) so the
        # pursuit→policy handoff is also smooth.
        self.max_angular_slew = max_angular_slew
        self._prev_w = 0.0
        self._prev_a = 0.0
        self._eint = 0.0
        # Whether the loaded policy takes the 5th (drift-integrator) input.
        self._use_eint = policy is not None and getattr(policy, "obs_dim", 4) >= 5

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear per-row controller state (call on a new row / after a U-turn)."""
        self._prev_a = 0.0
        self._prev_w = 0.0
        self._eint = 0.0
        self.pursuit.reset()          # clear the baseline's integral too

    # ------------------------------------------------------------------
    def _slew(self, w: float) -> float:
        """Clamp the per-step change in the angular command (field-safety bound)."""
        if self.max_angular_slew > 0.0:
            lo, hi = self._prev_w - self.max_angular_slew, self._prev_w + self.max_angular_slew
            w = max(lo, min(hi, w))
        self._prev_w = w
        return w

    # ------------------------------------------------------------------
    def compute(self, est: RowEstimate) -> tuple[float, float]:
        v, w_pp = self.pursuit.compute(est)
        # Keep the drift integrator warm every call (same as the sim, which
        # integrates every step) so it is correct the moment the policy re-engages
        # after a low-confidence stretch.
        self._eint = update_eint(self._eint, est.lateral_offset)
        # Fall back to pure-pursuit when the fix is weak or no policy is loaded —
        # the learned policy never acts on a low-confidence observation.
        if self.policy is None or est.confidence < self.min_confidence or not est.valid:
            self._prev_a = (w_pp / self.max_angular) if self.max_angular else 0.0
            return v, self._slew(w_pp)

        obs = [est.lateral_offset, est.heading_error, est.confidence, self._prev_a]
        if self._use_eint:
            obs.append(self._eint)
        delta = max(-1.0, min(1.0, self.policy.act(obs)))
        if self.residual:
            # Total action = baseline (with integral) + bounded policy correction.
            a_pp = w_pp / self.max_angular if self.max_angular else 0.0
            a = max(-1.0, min(1.0, a_pp + self.residual_scale * delta))
        else:
            a = delta                                  # policy replaces the steering
        self._prev_a = a
        w = max(-self.max_angular, min(self.max_angular, a * self.max_angular))
        return v, self._slew(w)
