#!/usr/bin/env python3
"""
wheel_odometry.py — Integrate Amiga wheel-encoder data between LiDAR scans.

The Amiga canbus service streams AmigaTpdo1 at ~50 Hz with:
    meas_speed_x  — forward speed (m/s)
    meas_ang_rate — yaw rate (rad/s)

WheelOdometry accumulates these into arc parameters (ds, dtheta).
Call get_delta_and_reset() once per LiDAR scan to obtain the motion
since the previous scan; slam_engine converts that arc to a world-frame
pose delta for the ICP warm start.

Thread-safe: update() is called from an async canbus task; get_delta_and_reset()
is called from the LiDAR processing loop.
"""

from __future__ import annotations

import math
import threading
from typing import Optional, Tuple


class WheelOdometry:
    """
    Accumulates Amiga encoder readings between LiDAR scans.

    Usage::

        odom = WheelOdometry()
        # in canbus loop (high-rate):
        odom.update(msg.meas_speed_x, msg.meas_ang_rate, time.monotonic())
        # once per LiDAR scan:
        ds, dtheta = odom.get_delta_and_reset()
    """

    # Ignore encoder updates with dt outside this window (stale / burst gaps)
    _DT_MIN = 1e-4   # 0.1 ms
    _DT_MAX = 0.50   # 500 ms

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._ds: float = 0.0       # accumulated forward arc (m)
        self._dtheta: float = 0.0   # accumulated heading change (rad)
        self._t_last: Optional[float] = None
        self.available: bool = False  # True after first valid update

    # ------------------------------------------------------------------
    def update(self, speed_x: float, ang_rate: float, t: float) -> None:
        """
        Integrate one encoder reading.

        Parameters
        ----------
        speed_x  : forward speed in m/s (from AmigaTpdo1.meas_speed_x)
        ang_rate : yaw rate in rad/s  (from AmigaTpdo1.meas_ang_rate)
        t        : monotonic timestamp in seconds (time.monotonic())
        """
        with self._lock:
            if self._t_last is None:
                self._t_last = t
                self.available = True
                return
            dt = t - self._t_last
            self._t_last = t
            if not (self._DT_MIN < dt < self._DT_MAX):
                return
            self._ds     += speed_x  * dt
            self._dtheta += ang_rate * dt

    # ------------------------------------------------------------------
    def get_delta_and_reset(self) -> Tuple[float, float]:
        """
        Return ``(ds, dtheta)`` accumulated since the last call and
        reset the accumulators to zero.

        ds     : forward arc distance in metres (positive = forward)
        dtheta : heading change in radians, wrapped to [-pi, pi]
                 (positive = CCW / left turn)
        """
        with self._lock:
            ds = self._ds
            dtheta = (self._dtheta + math.pi) % (2.0 * math.pi) - math.pi
            self._ds = 0.0
            self._dtheta = 0.0
            return ds, dtheta
