#!/usr/bin/env python3
"""
headland.py — IMU/perception headland U-turn onto the next row.

Four designs preceded this one and each failed in the field, all for the same
underlying reason: **nothing reliably measured how far the robot had actually
rotated.**

  1. open-loop timed phases — wheel slip/latency accumulated into big errors;
  2. two in-place 90° pivots closed on WHEEL heading — a 4-wheel skid-steer
     scrubs when pivoting, so the wheel-derived ``measured_angular_rate``
     over-reports rotation ~2×; "180°" was ~90° and the robot drove off;
  3. a single arc closed on WHEEL heading — same over-report, same ~90°;
  4. a perception-closed arc (keep arcing until the row is seen ahead) — the
     open-loop arc under-rotated (skid-steer slip makes the *actual* radius far
     wider than commanded, so a given arc length is much less rotation), and the
     dual-row detector locked onto **grass** at ~90° (it cannot tell grass from
     soybean rows), ending the turn early on the wrong feature.

The fix is to measure rotation with the one sensor that does not depend on wheel
contact: the **IMU in the filter service** (``FilterState`` heading).  The turn
only needs the *relative* heading CHANGE over ~10 s, and IMU yaw is locally
accurate even before the GPS filter globally converges — so we use the filter
heading's cumulative change regardless of ``has_converged`` (the previous code
wrongly required convergence and thus fell back to the unreliable wheel heading).

``HeadlandTurn`` drives EXIT → ARC and accumulates the absolute IMU heading
change since the arc began.  The navigator (``row_navigator._step_headland``)
uses that rotation to:
  * keep arcing until ~180° of REAL rotation (robust to slip);
  * only allow a perception lock to end the turn AFTER ≥ ~150° of rotation, so a
    grass glimpse at ~90° can no longer end it early;
  * complete on heading alone at ~175° (then APPROACH creeps in to acquire the
    actual row) if perception hasn't locked.
If the IMU heading is unavailable, it falls back to a long arc-length window plus
a strict perception lock.  Odometry forward distance (reliable) bounds the turn.

The serpentine pattern alternates direction each row; the caller passes
``turn_sign`` to ``begin()`` (+1 = right / clockwise, -1 = left / CCW).
"""

from __future__ import annotations

import math


def _ang_norm(a: float) -> float:
    """Normalise an angle to (-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class HeadlandTurn:
    """IMU-measured large-radius arc U-turn driver.

    Parameters
    ----------
    odometry : object
        Provides ``.distance`` (cumulative metres) — used only for arc-length
        guards (forward distance is reliable; the heading is not).
    row_spacing : float
        Centre-to-centre distance (m) to the next strip (status/info only now).
    exit_dist : float
        Straight distance (m) driven before the arc, to clear the row end.
    speed : float
        Forward speed (m/s) during the straight EXIT phase.
    turn_rate : float
        Arc angular rate (rad/s); arc forward speed = ``turn_radius · turn_rate``.
    turn_radius : float, optional
        Arc radius (m).  Default 1.0 m.
    max_turn_frac : float
        Hard arc-length cap as a multiple of a nominal semicircle (π·radius);
        stops the turn if no completion signal ever arrives (field edge).
    heading_source : object, optional
        Exposes ``.heading`` (rad) and ``.fresh`` (bool).  When fresh, its
        cumulative change measures the real rotation.
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
        max_turn_frac: float = 3.0,
        heading_source=None,
        scrub_comp: float = 0.6,
        ramp_dist: float = 0.6,
    ) -> None:
        self.odometry = odometry
        self.row_spacing = row_spacing
        self.exit_dist = exit_dist
        self.speed = speed
        self.turn_rate = turn_rate
        self.turn_radius = float(turn_radius) if turn_radius and turn_radius > 0.0 else 1.0
        self.max_turn_frac = max_turn_frac
        self.heading_source = heading_source
        # Wheel-odometry heading over-reports body rotation on a skid-steer arc;
        # multiply its cumulative change by scrub_comp to estimate the REAL
        # rotation (calibrate against the on-screen rot= readout).  Only used
        # when the IMU/filter heading is not live.
        self.scrub_comp = scrub_comp
        # Ease the arc angular rate in over the first ramp_dist metres of arc so
        # the skid-steer doesn't lurch (smoother, less initial scrub).
        self.ramp_dist = max(1e-3, ramp_dist)

        self._phase = "DONE"
        self._sign = 1.0
        self._d0 = 0.0            # odometry distance at current phase start
        self._capped = False
        # Rotation accumulation during ARC (cumulative |Δheading|, rad)
        self._cum_imu = 0.0
        self._cum_wheel = 0.0
        self._imu_prev = None
        self._wheel_prev = None
        self._imu_ok = False      # filter heading was fresh during the arc

    # ------------------------------------------------------------------
    def begin(self, turn_sign: float) -> None:
        self._sign = 1.0 if turn_sign >= 0 else -1.0
        self._capped = False
        self._cum_imu = 0.0
        self._cum_wheel = 0.0
        self._imu_prev = None
        self._wheel_prev = None
        self._imu_ok = False
        self._phase = "EXIT"
        self._d0 = float(self.odometry.distance)

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def done(self) -> bool:
        return self._phase == "DONE"

    @property
    def capped(self) -> bool:
        return self._capped

    @property
    def turn_sign(self) -> float:
        return self._sign

    @property
    def _nominal_arc(self) -> float:
        return math.pi * self.turn_radius

    @property
    def arc_len(self) -> float:
        if self._phase not in ("ARC", "DONE"):
            return 0.0
        return abs(float(self.odometry.distance) - self._d0)

    @property
    def heading_rotation(self) -> float:
        """Best estimate of REAL rotation since the arc began (rad).

        Prefers the IMU/filter heading (true rotation); otherwise uses the
        wheel-odometry heading scaled by ``scrub_comp`` (the skid-steer arc
        over-reports, so scrub_comp < 1 brings it down to the real rotation).
        """
        if self._imu_ok:
            return self._cum_imu
        return self._cum_wheel * self.scrub_comp

    @property
    def heading_tracking(self) -> bool:
        """True when a live rotation estimate (IMU or wheel) is available."""
        return self._imu_ok or self._cum_wheel > 0.0

    @property
    def heading_source_name(self) -> str:
        return "imu" if self._imu_ok else "wheel"

    # ------------------------------------------------------------------
    def finish(self) -> None:
        """End the turn (navigator: perception locked / heading reached 180°)."""
        self._phase = "DONE"
        self._capped = False

    # ------------------------------------------------------------------
    def _accumulate_heading(self) -> None:
        # Wheel-odometry heading (always available; over-reports).
        wcur = float(self.odometry.theta)
        if self._wheel_prev is not None:
            self._cum_wheel += abs(_ang_norm(wcur - self._wheel_prev))
        self._wheel_prev = wcur
        # IMU/filter heading (preferred when fresh).
        src = self.heading_source
        if src is not None and getattr(src, "fresh", False):
            icur = float(src.heading)
            if self._imu_prev is not None:
                self._cum_imu += abs(_ang_norm(icur - self._imu_prev))
            self._imu_prev = icur
            self._imu_ok = True

    # ------------------------------------------------------------------
    def _arc_command(self) -> tuple[float, float]:
        # Ease the angular rate in over the first ramp_dist of arc for a smooth,
        # low-scrub entry; forward speed tracks it so the path stays a circle.
        frac = min(1.0, self.arc_len / self.ramp_dist)
        ease = frac * frac * (3.0 - 2.0 * frac)   # smoothstep
        rate = self.turn_rate * (0.3 + 0.7 * ease)
        return self.turn_radius * rate, -self._sign * rate

    # ------------------------------------------------------------------
    def step(self, dt: float) -> tuple[float, float]:
        # Re-evaluate after a phase change so a completed phase does not waste a
        # zero/straight-command frame.
        for _ in range(len(self.PHASES)):
            if self._phase == "EXIT":
                if abs(float(self.odometry.distance) - self._d0) >= self.exit_dist:
                    self._phase = "ARC"
                    self._d0 = float(self.odometry.distance)
                    self._imu_prev = None    # start heading accumulation fresh
                    self._wheel_prev = None
                    continue
                return self.speed, 0.0

            if self._phase == "ARC":
                self._accumulate_heading()
                # Hard arc-length safety cap (reliable): give up if nothing ever
                # completes the turn (field edge / no next row).
                if self.arc_len >= self.max_turn_frac * self._nominal_arc:
                    self._phase = "DONE"
                    self._capped = True
                    return 0.0, 0.0
                return self._arc_command()

            return 0.0, 0.0  # DONE
        return 0.0, 0.0
