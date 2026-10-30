#!/usr/bin/env python3
"""
slam_runner.py — Run SLAM mapping alongside row-follow WITHOUT touching control.

Row-follow owns the only VLP-16 reader (a second UDP socket on port 2368 would
split the packet stream — Linux load-balances unicast UDP, it does not
duplicate).  So SLAM is fed the same scan the navigator already has.

To guarantee SLAM can never slow or destabilise the control loop, the heavy
work (ICP + map update) runs in a **separate daemon thread**.  The navigator
calls :meth:`submit` once per scan — that is O(1): it just stores the latest
scan under a lock.  The worker thread processes the most recent scan it finds
and drops any it could not keep up with (fine for mapping).  If anything in the
SLAM pipeline raises, it is swallowed here so mapping can never crash driving.

Mapping only — this never commands the wheels.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np

from slam.slam_engine import SlamEngine


class SlamRunner:
    def __init__(self, engine: SlamEngine, *, idle_sleep: float = 0.01) -> None:
        self.engine = engine
        self._idle_sleep = idle_sleep
        self._latest: Optional[np.ndarray] = None
        self._latest_odom: Optional[tuple] = None   # odometry (x, y, θ) at submit
        self._last_odom: Optional[tuple] = None     # … at the last PROCESSED scan
        self._lock = threading.Lock()
        self._stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True, name="slam")
        self.scans_submitted = 0
        self.scans_processed = 0
        self.errors = 0

    # ------------------------------------------------------------------
    def start(self) -> None:
        self._thread.start()

    def submit(self, pts: np.ndarray, odom_pose: Optional[tuple] = None) -> None:
        """Hand the latest scan (Nx3, sensor frame) to the worker. Non-blocking.

        ``odom_pose`` = the wheel-odometry pose ``(x, y, θ)`` (x along θ = 0,
        CCW θ) when the scan was taken.  The worker turns the change between the
        scans it actually PROCESSES (it drops the ones it cannot keep up with)
        into the ``(ds, dθ)`` warm start — without it ICP alone collapsed the
        along-row motion in an onion field (175 m mapped as 7.9 m)."""
        if pts is None or len(pts) == 0:
            return
        with self._lock:
            self._latest = pts            # keep only the most recent
            self._latest_odom = odom_pose
            self.scans_submitted += 1

    def stop(self, timeout: float = 2.0) -> None:
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=timeout)

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop:
            with self._lock:
                pts = self._latest
                odom = self._latest_odom
                self._latest = None
            if pts is None:
                time.sleep(self._idle_sleep)
                continue
            delta = None
            if odom is not None and self._last_odom is not None:
                x0, y0, t0 = self._last_odom
                ds = (odom[0] - x0) * math.cos(t0) + (odom[1] - y0) * math.sin(t0)
                dth = (odom[2] - t0 + math.pi) % (2 * math.pi) - math.pi
                delta = (ds, dth)
            self._last_odom = odom
            try:
                self.engine.process_scan(pts, odom_delta=delta)
                self.scans_processed += 1
            except Exception:           # never let mapping crash driving
                self.errors += 1
