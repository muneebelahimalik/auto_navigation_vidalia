#!/usr/bin/env python3
"""
headland.py — Closed-loop headland U-turn onto the adjacent row.

The previous headland manoeuvre was open-loop dead-reckoning: each phase ran for
a fixed number of seconds at the commanded speed, so wheel slip, control latency
and speed ramp-up all accumulated into large heading and position errors — the
robot routinely finished the turn pointing several degrees off the next row.

``HeadlandTurn`` instead closes the loop on wheel odometry (measured wheel speed
when the canbus telemetry is available, commanded-velocity dead-reckoning as a
fallback).  Each phase advances only when the measured distance or heading
change reaches its target, so the turn is accurate regardless of slip or latency.

U-turn geometry (right-hand turn shown; left mirrors the angular sign):

        row N (just finished)          row N+1 (next)
              │                              │
              │  ▲ heading in                ▼ heading out
              │  │                           │
        ──────┘  EXIT ──► TURN_A ──► SHIFT ──► TURN_B ──►
                 (clear     (pivot     (cross    (pivot
                  row end)   90°)       to next   90°)
                                        strip)

Phases
------
EXIT    drive straight ``exit_dist`` m to carry the body past the last plants.
TURN_A  pivot 90° toward the next row (in place; linear = 0).
SHIFT   drive straight ``row_spacing`` m across to the adjacent strip.
TURN_B  pivot 90° the same direction → now aligned down the next row.
DONE    manoeuvre complete; hand control back to ACQUIRE.

The serpentine (boustrophedon) coverage pattern alternates turn direction every
row: right U-turn, then left, then right …  The caller passes ``turn_sign`` to
``begin()`` (+1 = right / clockwise, -1 = left / counter-clockwise).
"""

from __future__ import annotations

import math


def _ang_norm(a: float) -> float:
    """Normalise an angle to (-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HeadlandTurn:
    """Odometry-closed-loop U-turn driver.

    Parameters
    ----------
    odometry : object
        Provides ``.distance`` (cumulative metres) and ``.theta`` (heading rad,
        CCW positive).  ``WheelOdometry`` satisfies this with measured wheel
        speed when available, commanded-velocity dead-reckoning otherwise.
    row_spacing : float
        Lateral distance (m) to the adjacent strip the robot should straddle next.
    exit_dist : float
        Straight distance (m) driven before the first pivot, to clear the row end.
    speed : float
        Forward speed (m/s) during the straight EXIT and SHIFT phases.
    turn_rate : float
        Pivot rate (rad/s) during the two 90° turns.
    angle_tol : float
        Heading tolerance (rad) for completing each pivot.
    """

    PHASES = ("EXIT", "TURN_A", "SHIFT", "TURN_B", "DONE")

    def __init__(
        self,
        odometry,
        *,
        row_spacing: float = 0.76,
        exit_dist: float = 1.0,
        speed: float = 0.15,
        turn_rate: float = 0.35,
        angle_tol: float = 0.05,
        heading_source=None,
    ) -> None:
        self.odometry = odometry
        self.row_spacing = row_spacing
        self.exit_dist = exit_dist
        self.speed = speed
        self.turn_rate = turn_rate
        self.angle_tol = angle_tol
        # Optional absolute-heading source (FilterHeading): when fresh+converged
        # the 90° pivots close on it instead of the slip-prone wheel-integrated
        # heading.  The choice is latched at begin() so a pivot never mixes two
        # heading references mid-turn.
        self.heading_source = heading_source

        self._phase = "DONE"
        self._sign = 1.0          # +1 = right (CW), -1 = left (CCW)
        self._d0 = 0.0            # odometry distance at current phase start
        self._th0 = 0.0           # heading at current phase start
        self._use_filter = False  # latched per turn in begin()

    # ------------------------------------------------------------------
    def begin(self, turn_sign: float) -> None:
        """Start a new U-turn.  ``turn_sign``: +1 = right, -1 = left."""
        self._sign = 1.0 if turn_sign >= 0 else -1.0
        # Latch the pivot heading source for the whole turn: prefer the filter
        # (IMU/GPS absolute heading) when it is usable, else wheel odometry.
        self._use_filter = bool(
            self.heading_source is not None and getattr(self.heading_source, "usable", False)
        )
        self._phase = "EXIT"
        self._mark()

    @property
    def heading_source_name(self) -> str:
        """Which heading reference the current turn is using (for status/logs)."""
        return "filter" if self._use_filter else "wheel"

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def done(self) -> bool:
        return self._phase == "DONE"

    @property
    def turn_sign(self) -> float:
        return self._sign

    # ------------------------------------------------------------------
    def step(self, dt: float) -> tuple[float, float]:
        """Advance the manoeuvre one tick; return (linear, angular) command.

        ``dt`` is accepted for interface symmetry; progress is measured from
        odometry, not integrated here.
        """
        # Pivot toward the next row: a right (CW) turn is negative angular
        # velocity in the CCW-positive convention used by the canbus.
        pivot = -self._sign * self.turn_rate
        target_pivot = (math.pi / 2.0) - self.angle_tol

        # Re-evaluate after each phase change so a completed phase does not
        # waste a zero-command frame.
        for _ in range(len(self.PHASES)):
            phase = self._phase
            if phase == "EXIT":
                if self._dist_since() >= self.exit_dist:
                    self._advance("TURN_A")
                    continue
                return self.speed, 0.0
            if phase == "TURN_A":
                if self._angle_since() >= target_pivot:
                    self._advance("SHIFT")
                    continue
                return 0.0, pivot
            if phase == "SHIFT":
                if self._dist_since() >= self.row_spacing:
                    self._advance("TURN_B")
                    continue
                return self.speed, 0.0
            if phase == "TURN_B":
                if self._angle_since() >= target_pivot:
                    self._advance("DONE")
                    continue
                return 0.0, pivot
            return 0.0, 0.0  # DONE
        return 0.0, 0.0

    # ------------------------------------------------------------------
    def _heading(self) -> float:
        """Current heading (rad) from the latched source for this turn."""
        if self._use_filter:
            return float(self.heading_source.heading)
        return float(self.odometry.theta)

    def _mark(self) -> None:
        self._d0 = float(self.odometry.distance)
        self._th0 = self._heading()

    def _advance(self, phase: str) -> None:
        self._phase = phase
        self._mark()

    def _dist_since(self) -> float:
        return abs(float(self.odometry.distance) - self._d0)

    def _angle_since(self) -> float:
        return abs(_ang_norm(self._heading() - self._th0))
