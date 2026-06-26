#!/usr/bin/env python3
"""
headland.py — Closed-loop headland U-turn onto the adjacent row.

The first design dead-reckoned each phase for a fixed time; the second closed
the loop on odometry but turned with **in-place pivots** (linear = 0, spin in
place).  On a 4-wheel skid-steer an in-place pivot is the worst case for
heading feedback: every wheel scrubs sideways, so the wheel-derived
``measured_angular_rate`` over-reports the body rotation badly.  In the field a
commanded 90° pivot finished closer to ~45°, and the two pivots summed to ~90°
instead of 180° — the robot ended up pointing *across* the rows and drove off
the field.

``HeadlandTurn`` now drives a **smooth constant-radius arc** instead.  Rolling
the wheels through an arc scrubs far less than pivoting in place, so the
heading estimate is much more accurate, and the largest radius that still lands
the robot on the next strip is a **semicircle of radius = shift / 2** (a tighter
radius would need a straight cross-segment; a wider one would overshoot the
strip).  This is the "bigger radius" turn — maximally gentle for the geometry.

U-turn geometry (right-hand turn shown; left mirrors the angular sign):

        row N (just finished)        row N+1 (next strip, `shift` over)
              │                              │
              │  ▲ heading in                ▼ heading out
              │  │      ____arc____          │
        ──────┘  EXIT ─/           \─────────┘
                 (clear   semicircle, radius shift/2,
                  row end)  turning a full 180°

Phases
------
EXIT  drive straight ``exit_dist`` m to carry the body past the last plants
      and give the LiDAR (which is blind inside ~1.5 m) room to confirm the row
      really ended.
ARC   drive a constant-radius arc (linear = radius·turn_rate, angular =
      ±turn_rate) until the heading has swept a full 180° → now aligned back
      down the next row.
DONE  manoeuvre complete; hand control to APPROACH.

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
    """Odometry-closed-loop arc U-turn driver.

    Parameters
    ----------
    odometry : object
        Provides ``.distance`` (cumulative metres) and ``.theta`` (heading rad,
        CCW positive).  ``WheelOdometry`` satisfies this with measured wheel
        speed when available, commanded-velocity dead-reckoning otherwise.
    row_spacing : float
        Centre-to-centre distance (m) to the adjacent strip the robot should
        straddle next (the headland *shift*).  Sets the arc radius to half this
        so a 180° semicircle lands exactly on the next strip, unless
        ``turn_radius`` is given explicitly.
    exit_dist : float
        Straight distance (m) driven before the arc, to clear the row end.
    speed : float
        Forward speed (m/s) during the straight EXIT phase.
    turn_rate : float
        Angular rate (rad/s) during the arc.  The arc's forward speed is
        ``radius · turn_rate`` so the path is a true circle of that radius.
    angle_tol : float
        Heading tolerance (rad) for completing the 180° arc.
    turn_radius : float, optional
        Explicit arc radius (m).  Defaults to ``row_spacing / 2`` (the
        maximum-radius semicircle that lands on the next strip).
    heading_source : object, optional
        Absolute-heading source (FilterHeading): when fresh+converged the arc
        closes on it instead of the slip-prone wheel-integrated heading.  The
        choice is latched at ``begin()`` so a turn never mixes two references.
    """

    PHASES = ("EXIT", "ARC", "DONE")

    def __init__(
        self,
        odometry,
        *,
        row_spacing: float = 0.76,
        exit_dist: float = 1.0,
        speed: float = 0.15,
        turn_rate: float = 0.35,
        angle_tol: float = 0.05,
        turn_radius: float | None = None,
        heading_source=None,
    ) -> None:
        self.odometry = odometry
        self.row_spacing = row_spacing
        self.exit_dist = exit_dist
        self.speed = speed
        self.turn_rate = turn_rate
        self.angle_tol = angle_tol
        # Maximum-radius semicircle by default: R = shift/2 lands a 180° arc
        # exactly on the next strip.  A caller may override for a tighter or
        # wider turn (wider needs more headland room and overshoots the strip,
        # which APPROACH + pure-pursuit then corrects laterally).
        self.turn_radius = (
            float(turn_radius) if turn_radius is not None else 0.5 * float(row_spacing)
        )
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
        # Latch the heading source for the whole turn: prefer the filter
        # (IMU/GPS absolute heading) when usable, else wheel odometry.
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
        # Arc command: a right (CW) turn is negative angular velocity in the
        # CCW-positive canbus convention.  Forward speed = radius·rate so the
        # path is a circle of `turn_radius`.
        arc_w = -self._sign * self.turn_rate
        arc_v = self.turn_radius * self.turn_rate
        # Full 180° U-turn (minus a small tolerance so we stop on time).
        target_arc = math.pi - self.angle_tol

        # Re-evaluate after each phase change so a completed phase does not
        # waste a zero-command frame.
        for _ in range(len(self.PHASES)):
            phase = self._phase
            if phase == "EXIT":
                if self._dist_since() >= self.exit_dist:
                    self._advance("ARC")
                    continue
                return self.speed, 0.0
            if phase == "ARC":
                if self._angle_since() >= target_arc:
                    self._advance("DONE")
                    continue
                return arc_v, arc_w
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
