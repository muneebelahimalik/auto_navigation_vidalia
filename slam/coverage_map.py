#!/usr/bin/env python3
"""
coverage_map.py — Field coverage accounting from the drift-corrected SLAM pose.

This is the operational payoff of running SLAM on a row-follower: a single LiDAR
scan cannot tell you whether the robot covered the field, only what is around it
right now.  Stamping the robot's WORKING SWATH along the SLAM pose trajectory
(which is drift-corrected by scan-matching + loop closure — raw wheel odometry
would smear after a few rows) produces a map of exactly which ground was
serviced, where the gaps are, and how much the passes overlapped.

The grid shares the OccupancyGrid coordinate convention (origin at array centre,
``gx = round(x/res)+origin``) so the coverage mask overlays the structure map
pixel-for-pixel.

Mapping/accounting only — never touches control.
"""

from __future__ import annotations

import numpy as np


class CoverageGrid:
    """Boolean "serviced ground" mask accumulated along the robot path.

    Each pose stamps a disc of radius ``swath_m / 2`` (the half working width).
    Consecutive poses are interpolated so a fast move or a dropped scan can not
    leave a false gap in the swath.
    """

    def __init__(self, size_m: float = 150.0, resolution: float = 0.10,
                 swath_m: float = 1.92) -> None:
        self.res = float(resolution)
        self.n = int(size_m / resolution)
        self.origin = self.n // 2
        self.swath_m = float(swath_m)
        self._covered = np.zeros((self.n, self.n), dtype=bool)
        self._path_len = 0.0
        self._last_xy: tuple[float, float] | None = None

        # Pre-computed disc footprint (cell offsets within the swath half-width).
        r = max(0, int(round((self.swath_m / 2.0) / self.res)))
        if r == 0:
            self._disc = np.zeros((1, 2), dtype=np.int64)   # single cell
        else:
            ys, xs = np.mgrid[-r:r + 1, -r:r + 1]
            m = (ys * ys + xs * xs) <= r * r
            self._disc = np.column_stack([ys[m].ravel(), xs[m].ravel()]).astype(np.int64)
        self._stamp_step = max(self.res, (r * self.res) * 0.5)  # ≤ half-swath spacing

    # ------------------------------------------------------------------
    def _stamp(self, x: float, y: float) -> None:
        """Mark the swath disc centred at world (x, y) as covered."""
        cy = int(round(y / self.res)) + self.origin
        cx = int(round(x / self.res)) + self.origin
        gy = self._disc[:, 0] + cy
        gx = self._disc[:, 1] + cx
        ok = (gy >= 0) & (gy < self.n) & (gx >= 0) & (gx < self.n)
        self._covered[gy[ok], gx[ok]] = True

    # ------------------------------------------------------------------
    def add_pose(self, x: float, y: float) -> None:
        """Extend the covered swath to the new robot position (world frame)."""
        if self._last_xy is None:
            self._stamp(x, y)
            self._last_xy = (x, y)
            return
        x0, y0 = self._last_xy
        dist = float(np.hypot(x - x0, y - y0))
        self._path_len += dist
        # Interpolate stamps so the swath stays continuous across the segment.
        n_sub = max(1, int(np.ceil(dist / self._stamp_step)))
        for i in range(1, n_sub + 1):
            t = i / n_sub
            self._stamp(x0 + (x - x0) * t, y0 + (y - y0) * t)
        self._last_xy = (x, y)

    # ------------------------------------------------------------------
    @property
    def covered_mask(self) -> np.ndarray:
        return self._covered

    @property
    def covered_cells(self) -> int:
        return int(self._covered.sum())

    @property
    def covered_area_m2(self) -> float:
        return self.covered_cells * self.res * self.res

    @property
    def path_length_m(self) -> float:
        return float(self._path_len)

    @property
    def redundancy(self) -> float:
        """Swept area (path × swath) ÷ unique covered area.

        1.0 = no overlap between passes; >1 = passes overlapped / double-driven;
        a value well above 1 with visible gaps means uneven row spacing.
        """
        area = self.covered_area_m2
        if area <= 0.0:
            return 0.0
        return (self._path_len * self.swath_m) / area

    def stats(self) -> dict:
        return {
            "covered_area_m2": round(self.covered_area_m2, 2),
            "path_length_m": round(self._path_len, 2),
            "swath_m": self.swath_m,
            "redundancy": round(self.redundancy, 3),
        }
