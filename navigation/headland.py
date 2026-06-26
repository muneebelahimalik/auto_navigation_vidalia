#!/usr/bin/env python3
"""
headland.py — Perception-closed headland U-turn onto the next row.

Three designs preceded this one and each failed in the field:

  1. open-loop timed phases — wheel slip/latency accumulated into big errors;
  2. two in-place 90° pivots closed on wheel heading — a 4-wheel skid-steer
     SCRUBS when pivoting in place, so the wheel-derived ``measured_angular_rate``
     over-reports rotation ~2×; each "90°" finished ~45°, the pair summed to ~90°
     and the robot drove off across the rows;
  3. a single 180° arc closed on wheel heading — gentler, but still scrubbed
     enough that "180° of wheel heading" was only ~90° of real rotation, so it
     handed off to APPROACH pointing across the field and got stuck.

The lesson: **wheel-odometry heading cannot be trusted to know when the turn is
done** on this platform.  So the turn no longer closes on heading at all.  It
drives a smooth, *large-radius* arc and **keeps arcing until the LiDAR actually
sees the next row lined up ahead** (perception-closed), at which point the
navigator hands straight to FOLLOW.  Odometry is used only for coarse
arc-LENGTH guards (forward distance from ``measured_speed`` is reliable even when
the angular rate is not):

  * don't start looking for the next row until ~``min_turn_frac`` of a nominal
    semicircle has been driven (so the sweep is past the row just left and near
    the next strip);
  * give up and stop after ~``max_turn_frac`` semicircles (field edge / no next
    row) rather than spinning forever.

A larger radius is deliberately gentle: it scrubs less AND sweeps the next row
through the field of view more slowly, so the alignment lock is easier to catch.

U-turn geometry (right turn; left mirrors the sign):

        row N (just finished)        row N+1 (next strip)
              │                              │
              │  ▲ heading in                ▼ heading out
              │  │      __ big arc __        │
        ──────┘  EXIT ─/             \───────┘
                 (clear   keep arcing until the row
                  row end) is detected aligned ahead → FOLLOW

The serpentine pattern alternates turn direction each row; the caller passes
``turn_sign`` to ``begin()`` (+1 = right / clockwise, -1 = left / CCW).
"""

from __future__ import annotations

import math


class HeadlandTurn:
    """Perception-closed large-radius arc U-turn driver.

    The driver only generates the arc command and tracks how far it has driven;
    the decision to *finish* the turn is made by the navigator when perception
    re-locks the next row (``finish()``), or here when the arc-length safety cap
    is reached (``capped``).

    Parameters
    ----------
    odometry : object
        Provides ``.distance`` (cumulative metres).  Only forward distance is
        used (reliable); the heading is intentionally NOT used to end the turn.
    row_spacing : float
        Centre-to-centre distance (m) to the next strip — sets the auto arc
        radius when ``turn_radius`` is not given.
    exit_dist : float
        Straight distance (m) driven before the arc, to clear the row end.
    speed : float
        Forward speed (m/s) during the straight EXIT phase.
    turn_rate : float
        Arc angular rate (rad/s); arc forward speed = ``turn_radius · turn_rate``.
    turn_radius : float, optional
        Arc radius (m).  Default 1.0 m — a gentle, low-scrub turn (overridable).
    min_turn_frac, max_turn_frac : float
        Arc-length guards as fractions of a nominal semicircle (π·radius):
        earliest re-acquire and the give-up cap.
    heading_source : object, optional
        Kept only for the status label (filter vs wheel availability); not used
        to end the turn.
    """

    PHASES = ("EXIT", "ARC", "DONE")

    def __init__(
        self,
        odometry,
        *,
        row_spacing: float = 1.52,
        exit_dist: float = 1.0,
        speed: float = 0.15,
        turn_rate: float = 0.30,
        turn_radius: float | None = None,
        min_turn_frac: float = 0.55,
        max_turn_frac: float = 2.2,
        heading_source=None,
    ) -> None:
        self.odometry = odometry
        self.row_spacing = row_spacing
        self.exit_dist = exit_dist
        self.speed = speed
        self.turn_rate = turn_rate
        # Default to a gentle 1.0 m radius (bigger than the shift/2 semicircle):
        # less scrub, and the next row sweeps through view slowly so the
        # perception lock is easy to catch.
        self.turn_radius = float(turn_radius) if turn_radius and turn_radius > 0.0 else 1.0
        self.min_turn_frac = min_turn_frac
        self.max_turn_frac = max_turn_frac
        self.heading_source = heading_source

        self._phase = "DONE"
        self._sign = 1.0
        self._d0 = 0.0            # odometry distance at current phase start
        self._capped = False
        self._use_filter = False

    # ------------------------------------------------------------------
    def begin(self, turn_sign: float) -> None:
        """Start a new U-turn.  ``turn_sign``: +1 = right, -1 = left."""
        self._sign = 1.0 if turn_sign >= 0 else -1.0
        self._capped = False
        self._use_filter = bool(
            self.heading_source is not None and getattr(self.heading_source, "usable", False)
        )
        self._phase = "EXIT"
        self._d0 = float(self.odometry.distance)

    @property
    def heading_source_name(self) -> str:
        return "filter" if self._use_filter else "wheel"

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def done(self) -> bool:
        return self._phase == "DONE"

    @property
    def capped(self) -> bool:
        """True if the turn ended by hitting the arc-length cap (no row found)."""
        return self._capped

    @property
    def turn_sign(self) -> float:
        return self._sign

    @property
    def _nominal_arc(self) -> float:
        return math.pi * self.turn_radius

    @property
    def arc_len(self) -> float:
        """Distance driven since the ARC phase began (m)."""
        if self._phase not in ("ARC", "DONE"):
            return 0.0
        return abs(float(self.odometry.distance) - self._d0)

    @property
    def ready_to_reacquire(self) -> bool:
        """True once enough arc has been driven to start looking for the next row."""
        return self._phase == "ARC" and self.arc_len >= self.min_turn_frac * self._nominal_arc

    # ------------------------------------------------------------------
    def finish(self) -> None:
        """End the turn because perception re-locked the next row."""
        self._phase = "DONE"
        self._capped = False

    # ------------------------------------------------------------------
    def step(self, dt: float) -> tuple[float, float]:
        """Advance the manoeuvre one tick; return (linear, angular).

        Progress is measured from odometry distance, not integrated here.
        """
        arc_w = -self._sign * self.turn_rate
        arc_v = self.turn_radius * self.turn_rate

        if self._phase == "EXIT":
            if abs(float(self.odometry.distance) - self._d0) >= self.exit_dist:
                self._phase = "ARC"
                self._d0 = float(self.odometry.distance)
                return arc_v, arc_w
            return self.speed, 0.0

        if self._phase == "ARC":
            # Safety cap on arc LENGTH (reliable) — give up if no row is found.
            if self.arc_len >= self.max_turn_frac * self._nominal_arc:
                self._phase = "DONE"
                self._capped = True
                return 0.0, 0.0
            return arc_v, arc_w

        return 0.0, 0.0  # DONE
