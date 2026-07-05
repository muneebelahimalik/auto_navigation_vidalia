#!/usr/bin/env python3
"""
row_controller.py — Pure-pursuit steering for centre-row following.

Given a RowEstimate (heading error, lateral offset, confidence) the
controller places a look-ahead target on the crop-row centreline and
produces a (linear, angular) velocity command that drives the robot
centreline onto the row.

Sign convention (matches CanbusInterface):
  linear  > 0  — forward
  angular > 0  — counter-clockwise (left turn)

Pure pursuit naturally folds heading error and lateral offset into a
single curvature, so no separate gains are needed.  Speed is throttled
by row confidence and heading error so the robot slows down — rather than
swerving — whenever the row fix is weak.

Integral (disturbance-cancelling) action
-----------------------------------------
Plain pure pursuit is a *memoryless geometric* law: on a cross-slope it reaches
a steady state with a standing cross-track error, because the geometric feedback
needs a non-zero offset to produce the counter-steer that balances a constant
drift — it can chase the drift but never cancel it.  That standing error is the
field slope-weave, and it is why the MPC (disturbance observer) and the RL policy
(drift integrator) were added on top.

The baseline itself now carries the same idea, at its simplest and safest: a
**leaky integral of the cross-track offset** adds a counter-steer so a persistent
drift is cancelled and the steady-state offset goes to ~0.  It is:
  * **leaky** (``i_leak`` < 1) — the integral forgets, so it never runs away and
    it empties on a disturbance-free row (nominal behaviour is unchanged);
  * **anti-windup** (the steering contribution is clamped to ``i_clamp`` and the
    accumulator to the value that would saturate it) — it cannot build a huge
    hidden term that overshoots when the disturbance clears;
  * **confidence-gated** (only integrates above ``i_min_conf``; leaks toward 0 on
    a weak/lost fix) — it never winds up on a bad observation;
  * **opt-outable** — ``ki = 0`` restores the exact original pure-geometric law.
``reset()`` clears it on a new row / after a U-turn.
"""

from __future__ import annotations

import math

from navigation.row_perception import RowEstimate


class PurePursuitController:
    """Look-ahead pure-pursuit controller for crop-row following, with a leaky
    integral (disturbance-cancelling) term on the cross-track offset."""

    def __init__(
        self,
        max_linear: float = 0.30,
        min_linear: float = 0.08,
        max_angular: float = 0.40,
        lookahead: float = 2.0,
        min_confidence: float = 0.35,
        *,
        ki: float = 0.15,            # integral gain on the leaky cross-track integral
        i_leak: float = 0.94,        # per-step leak (<1); ~1.6 s memory at 10 Hz (damped, no limit cycle)
        i_clamp: float = 0.15,       # max |integral steering contribution| (rad/s)
        i_min_conf: float = 0.45,    # only integrate above this confidence
        i_rate: float = 0.20,        # only integrate when |Δoffset| < i_rate·dt (persistent, not transient)
        dt: float = 0.1,             # control period (s); matches the 10 Hz scan rate
    ) -> None:
        self.max_linear = max_linear
        self.min_linear = min_linear
        self.max_angular = max_angular
        self.lookahead = lookahead
        self.min_confidence = min_confidence
        self.ki = ki
        self.i_leak = i_leak
        self.i_clamp = i_clamp
        self.i_min_conf = i_min_conf
        self.i_rate = i_rate
        self.dt = dt
        # Accumulator bound (m·s) so the integral's steering term never exceeds
        # i_clamp — anti-windup at the state level.
        self._e_max = (i_clamp / ki) if ki > 0 else 0.0
        self._eint = 0.0
        self._prev_off = 0.0

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Clear the integral (call on a new row / after a U-turn)."""
        self._eint = 0.0
        self._prev_off = 0.0

    # ------------------------------------------------------------------
    def compute(self, est: RowEstimate) -> tuple[float, float]:
        """
        Return (linear, angular) for the given row estimate.

        Returns (0, 0) when confidence is below threshold — the caller's
        state machine is responsible for re-acquiring the row.
        """
        if est.confidence < self.min_confidence:
            self._eint *= self.i_leak      # bleed the integral on a lost fix
            return 0.0, 0.0

        theta = est.heading_error
        offset = est.lateral_offset
        L = self.lookahead

        # Look-ahead target on the row centreline, at forward distance ~L.
        # centreline: point = offset * perp + s * dir
        #   dir  = (sinθ, cosθ)   perp = (cosθ, -sinθ)
        # solve forward coord y = L  ->  s, then read target X.
        cos_t = max(math.cos(theta), 0.10)
        s = (L + offset * math.sin(theta)) / cos_t
        target_x = offset * math.cos(theta) + s * math.sin(theta)
        target_y = max(-offset * math.sin(theta) + s * math.cos(theta), 0.5)

        # Speed: scale down with weak confidence and large heading error.
        # Divisor 1.0 rad keeps the robot near half-speed at a typical 22° entry
        # angle; the old 0.60 bottomed out at 35% (~0.10 m/s) which cut angular
        # authority so much that lateral correction took 5+ seconds to converge.
        speed_factor = est.confidence * max(0.25, 1.0 - abs(theta) / 1.0)
        linear = max(self.min_linear, self.max_linear * speed_factor)

        # Pure-pursuit curvature; +target_x (row to the right) -> turn right.
        dist_sq = target_x * target_x + target_y * target_y
        curvature = 2.0 * target_x / dist_sq
        angular = -linear * curvature

        # Leaky integral (disturbance-cancelling) term.  Accumulate only when the
        # fix is confident AND the offset is PERSISTENT (|Δoffset| small) — a
        # steady drift leaves a standing offset that barely changes step-to-step,
        # whereas a transient approach changes fast.  Gating on the RATE (not the
        # magnitude) lets the integral engage on a large-but-steady slope error
        # (where it matters most) while still refusing to wind up on a fast
        # transient and overshoot.  Sign matches the curvature: offset>0 → row
        # right → steer right (w<0).
        if self.ki > 0.0:
            steady = abs(offset - self._prev_off) <= self.i_rate * self.dt
            if est.confidence >= self.i_min_conf and steady:
                self._eint = self.i_leak * self._eint + offset * self.dt
            else:
                self._eint *= self.i_leak
            if self._e_max:                                  # anti-windup on the state
                self._eint = max(-self._e_max, min(self._e_max, self._eint))
            w_i = -self.ki * self._eint
            w_i = max(-self.i_clamp, min(self.i_clamp, w_i))
            angular += w_i
        self._prev_off = offset

        angular = max(-self.max_angular, min(self.max_angular, angular))
        return linear, angular
