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
from navigation.rl_policy import MLPPolicy


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
    ) -> None:
        # Reused for forward speed + as the low-confidence / no-policy fallback.
        self.pursuit = PurePursuitController(
            max_linear=max_linear, min_linear=min_linear, max_angular=max_angular,
            lookahead=lookahead, min_confidence=min_confidence)
        self.policy = policy
        self.max_angular = max_angular
        self.min_confidence = min_confidence
        self.lookahead = lookahead
        self._prev_a = 0.0

    # ------------------------------------------------------------------
    def compute(self, est: RowEstimate) -> tuple[float, float]:
        v, w_pp = self.pursuit.compute(est)
        # Fall back to pure-pursuit when the fix is weak or no policy is loaded —
        # the learned policy never acts on a low-confidence observation.
        if self.policy is None or est.confidence < self.min_confidence or not est.valid:
            self._prev_a = (w_pp / self.max_angular) if self.max_angular else 0.0
            return v, w_pp

        a = self.policy.act([
            est.lateral_offset, est.heading_error, est.confidence, self._prev_a,
        ])
        a = max(-1.0, min(1.0, a))
        self._prev_a = a
        w = max(-self.max_angular, min(self.max_angular, a * self.max_angular))
        return v, w
