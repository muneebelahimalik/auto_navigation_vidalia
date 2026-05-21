#!/usr/bin/env python3
"""
row_perception.py — Online centre crop-row detection from VLP-16 scans.

The Amiga straddles a crop bed: the robot body passes over the planted
centre rows while the wheels run in the furrows either side.  This module
detects the centre crop row directly ahead of the robot in real time, with
NO prior map, and reports how the robot sits relative to it.

Pipeline (per scan, sensor frame X=right, Y=forward, Z=up):
  1. Crop a forward region of interest (ROI) ahead of the robot.
  2. Keep only points inside the crop-height band above ground, using
     ground-relative height  h = z + LIDAR_MOUNT_HEIGHT.  Bare-soil furrow
     returns (h ~ 0) drop out; raised crop rows survive.
  3. Project to a 2-D bird's-eye view (X, Y).
  4. PCA over the whole ROI -> dominant *row direction*.  All crop rows are
     parallel, so the elongation axis is the row heading.
  5. Project points onto the cross-row axis and locate the centre row
     (the cluster nearest the robot centreline) -> lateral offset.
  6. Exponential smoothing over time so the controller follows the row
     trend, not every plant-to-plant wobble.

Output: a RowEstimate the controller and state machine consume directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT


@dataclass
class RowEstimate:
    """Robot pose relative to the centre crop row."""
    heading_error: float = 0.0       # rad; +ve = row angled to robot's right
    lateral_offset: float = 0.0      # m;   +ve = centre row is to the right
    confidence: float = 0.0          # 0..1 row-detection quality
    row_end_confidence: float = 0.0  # 0..1; high when crop ahead disappears
    n_points: int = 0                # crop-band points used in the fit
    valid: bool = False              # True when a row was detected this scan


class RowDetector:
    """
    Stateful centre-row detector.  Call update() once per LiDAR scan.

    All distances are metres in the sensor frame (X=right, Y=forward).
    """

    def __init__(
        self,
        roi_y_min: float = 1.5,
        roi_y_max: float = 7.0,
        roi_x_half: float = 1.20,
        crop_h_min: float = 0.04,
        crop_h_max: float = 0.55,
        min_points: int = 40,
        full_points: int = 220,
        refine_window: float = 0.18,
        bin_width: float = 0.10,
        row_end_density: float = 70.0,
        ema_alpha: float = 0.35,
    ) -> None:
        self.roi_y_min = roi_y_min
        self.roi_y_max = roi_y_max
        self.roi_x_half = roi_x_half
        self.crop_h_min = crop_h_min
        self.crop_h_max = crop_h_max
        self.min_points = min_points
        self.full_points = full_points
        self.refine_window = refine_window
        self.bin_width = bin_width
        self.row_end_density = row_end_density
        self.ema_alpha = ema_alpha
        self._est = RowEstimate()

    # ------------------------------------------------------------------
    def update(self, pts: np.ndarray) -> RowEstimate:
        """
        Detect the centre row from an Nx3 (x, y, z) sensor-frame point array.

        Returns a smoothed RowEstimate.  When no row is found the previous
        estimate is decayed (confidence halved, row-end confidence raised)
        so the state machine can react instead of acting on a stale fix.
        """
        if pts is None or len(pts) == 0:
            return self._decay()

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        h = z + LIDAR_MOUNT_HEIGHT
        roi = (
            (y >= self.roi_y_min) & (y <= self.roi_y_max)
            & (np.abs(x) <= self.roi_x_half)
            & (h >= self.crop_h_min) & (h <= self.crop_h_max)
        )
        cx, cy = x[roi], y[roi]
        n = int(cx.shape[0])

        row_end_conf = 1.0 - min(1.0, n / self.row_end_density)
        if n < self.min_points:
            return self._decay(row_end_conf=row_end_conf)

        P = np.column_stack((cx, cy))

        # --- PCA: dominant axis = row direction (rows are parallel) ---
        mean = P.mean(axis=0)
        Q = P - mean
        cov = (Q.T @ Q) / len(Q)
        evals, evecs = np.linalg.eigh(cov)          # ascending
        direction = evecs[:, 1]
        if direction[1] < 0.0:                       # forward-positive
            direction = -direction
        lam_major, lam_minor = float(evals[1]), float(evals[0])
        linearity = (lam_major - lam_minor) / (lam_major + lam_minor + 1e-9)
        heading = math.atan2(direction[0], direction[1])

        # --- locate the centre row on the cross-row axis ---
        # Histogram the cross-row coordinate, pick the peak nearest the
        # robot centreline, then refine with a tight window around it so
        # adjacent rows are never merged into the estimate.
        perp = np.array([direction[1], -direction[0]])   # unit, +X-ish
        cross = P @ perp
        peak = self._nearest_peak(cross)
        near = np.abs(cross - peak) <= self.refine_window
        lateral = float(cross[near].mean()) if int(near.sum()) >= 8 else peak

        density = min(1.0, n / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
        confidence = density * linear_factor

        return self._smooth(RowEstimate(
            heading_error=heading,
            lateral_offset=lateral,
            confidence=confidence,
            row_end_confidence=row_end_conf,
            n_points=n,
            valid=True,
        ))

    # ------------------------------------------------------------------
    def _nearest_peak(self, cross: np.ndarray) -> float:
        """Histogram the cross-row axis; return the peak nearest centreline."""
        lo, hi = -self.roi_x_half, self.roi_x_half
        edges = np.arange(lo, hi + self.bin_width, self.bin_width)
        hist, edges = np.histogram(cross, bins=edges)
        centres = 0.5 * (edges[:-1] + edges[1:])
        smooth = np.convolve(hist.astype(float), [0.25, 0.5, 0.25], mode="same")
        thresh = max(3.0, 0.25 * float(smooth.max()))
        peaks = [
            i for i in range(1, len(smooth) - 1)
            if smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]
            and smooth[i] >= thresh
        ]
        if not peaks:
            peaks = [int(np.argmax(smooth))]
        best = min(peaks, key=lambda i: abs(centres[i]))
        return float(centres[best])

    # ------------------------------------------------------------------
    def _smooth(self, fresh: RowEstimate) -> RowEstimate:
        """Exponentially blend a fresh estimate into the running state."""
        a = self.ema_alpha
        prev = self._est
        self._est = RowEstimate(
            heading_error=a * fresh.heading_error + (1 - a) * prev.heading_error,
            lateral_offset=a * fresh.lateral_offset + (1 - a) * prev.lateral_offset,
            confidence=a * fresh.confidence + (1 - a) * prev.confidence,
            row_end_confidence=fresh.row_end_confidence,
            n_points=fresh.n_points,
            valid=True,
        )
        return self._est

    # ------------------------------------------------------------------
    def _decay(self, row_end_conf: float = 1.0) -> RowEstimate:
        """No detection this scan — keep geometry, drop confidence."""
        prev = self._est
        self._est = RowEstimate(
            heading_error=prev.heading_error,
            lateral_offset=prev.lateral_offset,
            confidence=prev.confidence * 0.5,
            row_end_confidence=max(prev.row_end_confidence, row_end_conf),
            n_points=0,
            valid=False,
        )
        return self._est
