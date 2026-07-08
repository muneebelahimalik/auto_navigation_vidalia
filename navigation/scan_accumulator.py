#!/usr/bin/env python3
"""
scan_accumulator.py — Motion-compensated multi-scan accumulation for the row
detector (dense-vegetation signal restoration).

WHY.  The dual-row LiDAR fit keys off the ground/furrow gap between the two crop
rows.  When the canopy closes over (dense soybean), that gap fills in and a
single 10 Hz VLP-16 scan sees the residue strip only as a faint canopy-height
VALLEY between two ridges — low signal-to-noise, so the per-scan PCA heading is
ill-conditioned (the field weave / heading runaway).  But the row is STATIC and
the robot crawls (~1.5 cm per scan at 0.15 m/s), so several consecutive scans
are near-identical views of the same structure.  Registering the last ``n``
scans into the CURRENT robot frame (motion-compensated by odometry) multiplies
the point density ~n×, which averages out the per-scan plateau noise and
re-conditions the fit — the standard scan-aggregation move in ag-robotics.

SCOPE.  This densifies the cloud fed to the ROW DETECTOR only.  It must NOT feed
the safety monitor: obstacles can move, and smearing several scans of a moving
person across the frame would blur the very thing the safety zones must see
crisply.  The caller keeps the raw single scan for safety / bridge / capture and
passes the accumulated cloud only to ``RowDetector.update``.

FRAME.  Robot frame is x=right, y=forward, z=height-above-ground (already
yaw/tilt corrected upstream).  z is invariant under the planar (3-DOF) motion of
a ground robot, so only the (x, y) of each buffered scan is transformed; the
height that the crop-band / dense-canopy logic reads is unchanged.

Registration uses the commanded (or measured) motion since the previous scan —
forward distance ``d_fwd`` (m, ≥0) and heading change ``d_theta`` (rad, +CCW /
left).  Over a short window (n ≈ 5–10 scans, < 1 s, < 15 cm of travel) odometry
drift is negligible, so no ICP is needed; the accumulator just needs approximate
registration to densify, not metric mapping.
"""
from __future__ import annotations

import math

import numpy as np


class ScanAccumulator:
    def __init__(self, n_scans: int = 1, max_points: int = 0) -> None:
        # n_scans = 1 → pass-through (no accumulation, byte-identical to before).
        self.n = max(1, int(n_scans))
        # Optional cap on the merged cloud size (0 = unlimited); a random
        # subsample keeps a very dense merge from slowing the O(N) fit.
        self.max_points = int(max_points)
        self._rng = np.random.default_rng(0)
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the buffered scans and reset the local odometry pose.

        Call on a new row / after a U-turn (same lifecycle as
        ``RowDetector.reset``): the ROI sweeps across the headland during a turn,
        so stale scans from the previous row must not be merged into the next.
        """
        self._x = 0.0
        self._y = 0.0
        self._phi = 0.0                 # heading vs the anchor frame (rad, CCW)
        self._buf: list[np.ndarray] = []   # buffered scans, in the anchor world frame

    # ------------------------------------------------------------------
    def update(self, pts: np.ndarray, d_fwd: float = 0.0,
               d_theta: float = 0.0) -> np.ndarray:
        """Advance the pose by the motion since the previous scan, append this
        scan, and return the last ``n`` scans merged into the CURRENT frame.

        ``pts`` is the corrected robot-frame Nx3 cloud for this scan.  With
        ``n_scans == 1`` this is a pass-through (returns ``pts`` unchanged).
        """
        # Integrate the robot pose in a fixed anchor frame (robot starts at the
        # origin heading +Y).  Forward motion is along the robot's current
        # heading; world displacement = Rot(phi)·[0, d_fwd].
        self._phi += float(d_theta)
        self._x += -math.sin(self._phi) * float(d_fwd)
        self._y += math.cos(self._phi) * float(d_fwd)

        if self.n <= 1 or pts is None or len(pts) == 0:
            # Pass-through, but still track the pose so a later n>1 call (or the
            # very next scan) has a consistent origin.
            return pts

        c, s = math.cos(self._phi), math.sin(self._phi)
        P = np.asarray(pts, dtype=np.float64)
        # this scan → anchor world frame:  world = [x,y] + Rot(phi)·[rx,ry]
        wx = self._x + c * P[:, 0] - s * P[:, 1]
        wy = self._y + s * P[:, 0] + c * P[:, 1]
        self._buf.append(np.column_stack((wx, wy, P[:, 2])))
        if len(self._buf) > self.n:
            self._buf.pop(0)

        # all buffered world points → CURRENT robot frame:
        #   robot = Rot(phi)^T · (world − [x,y])
        W = np.vstack(self._buf)
        dx = W[:, 0] - self._x
        dy = W[:, 1] - self._y
        rx = c * dx + s * dy
        ry = -s * dx + c * dy
        merged = np.column_stack((rx, ry, W[:, 2]))

        if self.max_points and len(merged) > self.max_points:
            idx = self._rng.choice(len(merged), self.max_points, replace=False)
            merged = merged[idx]
        return merged
