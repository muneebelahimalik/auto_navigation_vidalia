#!/usr/bin/env python3
"""
row_perception.py — Online centre crop-row detection from VLP-16 scans.

The Amiga straddles a centre residue strip (soybean field) or a raised crop
bed (onion field): the robot body passes over the centre while the wheels run
in the furrows on either side.  This module detects the centre line ahead of
the robot in real time with NO prior map and reports how the robot sits
relative to it.

Pipeline (per scan, sensor frame X=right, Y=forward, Z=up):
  1. Crop a forward region of interest (ROI) ahead of the robot.
  2. Keep only points inside the crop-height band above ground, using
     ground-relative height  h = z + LIDAR_MOUNT_HEIGHT.  Bare-soil furrow
     returns (h ~ 0) drop out; raised crop rows survive.
  3. Project to a 2-D bird's-eye view (X, Y).
  4. PCA over the whole ROI -> dominant *row direction*.  All crop rows are
     parallel, so the elongation axis is the row heading.
  5. Locate the lateral centre:
       Single-row mode (default): histogram peak nearest the robot centreline.
       Dual-row mode (--dual-row): midpoint between the nearest left and right
         peaks — used in soybean fields where the robot straddles a dark centre
         residue strip flanked by soybean rows on each side; the midpoint is
         the centre of the residue strip, not either soybean row.
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
    """Robot pose relative to the centre crop row / residue strip."""
    heading_error: float = 0.0       # rad; +ve = row angled to robot's right
    lateral_offset: float = 0.0      # m;   +ve = centre row is to the right
    confidence: float = 0.0          # 0..1 row-detection quality
    row_end_confidence: float = 0.0  # 0..1; high when crop ahead disappears
    n_points: int = 0                # crop-band points used in the fit
    valid: bool = False              # True when a row was detected this scan


class RowDetector:
    """
    Stateful centre-row / centre-residue detector.  Call update() once per
    LiDAR scan.

    All distances are metres in the sensor frame (X=right, Y=forward).

    dual_row=False (default): single crop row under the robot (onion field).
    dual_row=True: robot straddles a centre residue strip flanked by crop rows
      on each side (soybean field).  Lateral offset = midpoint between the
      nearest left and right crop-band histogram peaks.
    """

    def __init__(
        self,
        roi_y_min: float = 1.5,
        roi_y_max: float = 7.0,
        roi_x_half: float = 0.80,
        crop_h_min: float = 0.03,
        crop_h_max: float = 0.30,
        min_points: int = 40,
        full_points: int = 130,
        refine_window: float = 0.18,
        bin_width: float = 0.10,
        row_end_density: float = 70.0,
        ema_alpha: float = 0.35,
        dual_row: bool = False,
        row_spacing: float = 0.76,
        max_lateral_jump: float = 0.30,
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
        self.dual_row = dual_row
        self.row_spacing = row_spacing
        self.max_lateral_jump = max_lateral_jump
        self._spacing_factor = 1.0   # confidence penalty when peak pair spacing is off
        self._est = RowEstimate()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the smoothed estimate (call when starting a new row).

        During a headland turn the ROI sweeps across the headland and the
        EMA accumulates garbage geometry; without a reset the lateral and
        heading outlier gates then fight the first genuine detections of
        the next row.
        """
        self._est = RowEstimate()
        self._spacing_factor = 1.0

    # ------------------------------------------------------------------
    def update(self, pts: np.ndarray) -> RowEstimate:
        """
        Detect the centre row/residue from an Nx3 (x, y, z) sensor-frame
        point array.

        Returns a smoothed RowEstimate.  When no row is found the previous
        estimate is decayed (confidence reduced, row-end confidence raised)
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

        # --- locate the centre on the cross-row axis ---
        perp = np.array([direction[1], -direction[0]])   # unit, +X-ish
        cross = P @ perp

        if self.dual_row:
            # Soybean / centre-residue mode: find the midpoint between the
            # nearest left and right crop-row peaks.  The midpoint is the
            # centre of the residue strip, which is the tracking target.
            lateral = self._midpoint_peaks(cross)
        else:
            # Single-row mode: histogram peak nearest the robot centreline,
            # then refine with a tight window so adjacent rows are not merged.
            peak = self._nearest_peak(cross)
            near = np.abs(cross - peak) <= self.refine_window
            lateral = float(cross[near].mean()) if int(near.sum()) >= 8 else peak

        density = min(1.0, n / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
        confidence = density * linear_factor
        if self.dual_row:
            confidence *= self._spacing_factor

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
    def _midpoint_peaks(self, cross: np.ndarray) -> float:
        """Return the midpoint between the left and right histogram peaks
        (dual-row / soybean centre-residue mode).

        The robot straddles the dark residue strip; the soybean rows flank it
        symmetrically on each side.  Their histogram peaks are at roughly
        ±(row_spacing/2) in the robot frame.  The midpoint of those two peaks
        is the centre of the residue strip — the tracking target.

        Pair selection uses a row-spacing prior: among all left/right peak
        combinations, the pair whose separation is closest to ``row_spacing``
        wins.  Picking the *innermost* pair instead (the old behaviour) locks
        onto weed clumps or residue clutter near the centreline whenever they
        produce a histogram peak, dragging the midpoint off the true centre.
        A pair whose separation is far from the expected spacing also lowers
        confidence via ``_spacing_factor``.

        When only one side is visible (robot far off-centre, or one row
        occluded), the residue centre is half a row spacing INWARD from the
        visible row — mirroring DualCameraRowTracker's single-side fallback.
        Returning the peak itself (the old behaviour) made the robot steer
        directly onto the visible soybean row.
        """
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

        # Separate into left (x < -0.05 m) and right (x > +0.05 m) peaks.
        # The small dead-band avoids treating noise exactly at centre as a peak.
        left_peaks = [p for p in peaks if centres[p] < -0.05]
        right_peaks = [p for p in peaks if centres[p] > 0.05]

        self._spacing_factor = 1.0
        half_spacing = 0.5 * self.row_spacing

        if left_peaks and right_peaks:
            # Row-spacing prior: choose the L/R pair whose separation best
            # matches the known flanking-row spacing.
            best_pair = None
            best_err = float("inf")
            for li in left_peaks:
                for ri in right_peaks:
                    sep = centres[ri] - centres[li]
                    err = abs(sep - self.row_spacing)
                    if err < best_err:
                        best_err = err
                        best_pair = (li, ri)
            li, ri = best_pair
            # Penalise confidence when even the best pair's separation is far
            # from the expected spacing (clutter posing as a row).
            self._spacing_factor = max(0.5, 1.0 - best_err / self.row_spacing)
            return 0.5 * (centres[li] + centres[ri])

        if left_peaks:
            # Only the left row visible — centre is half a spacing to its right.
            inner = max(left_peaks, key=lambda i: centres[i])
            self._spacing_factor = 0.7
            return float(centres[inner]) + half_spacing
        if right_peaks:
            inner = min(right_peaks, key=lambda i: centres[i])
            self._spacing_factor = 0.7
            return float(centres[inner]) - half_spacing

        # All peaks inside the dead-band (clutter on the residue strip itself):
        # treat as "approximately centred" rather than steering at the clutter.
        self._spacing_factor = 0.5
        return float(centres[min(peaks, key=lambda i: abs(centres[i]))])

    # ------------------------------------------------------------------
    def _smooth(self, fresh: RowEstimate) -> RowEstimate:
        """Exponentially blend a fresh estimate into the running state."""
        a = self.ema_alpha
        prev = self._est

        # Heading outlier gate: if the fresh PCA heading jumps more than 30°
        # from the current smoothed heading, clamp it. Sparse or nearly-round
        # crop clusters (row ends, cardboard, thin canopy) give PCA directions
        # that can flip ±90°; without the gate the heading EMA oscillates ±50°
        # and pure-pursuit produces diverging steering corrections.
        # Threshold 0.15 (was 0.30): n=0 scans decay confidence ×0.75, so
        # starting from conf≈0.45 it falls to 0.34→0.25. With 0.30 the gate
        # disabled at 0.25 and allowed unclamped jumps, drifting heading +5°/scan.
        hdg_fresh = fresh.heading_error
        lat_fresh = fresh.lateral_offset
        if prev.confidence > 0.15:
            delta = hdg_fresh - prev.heading_error
            max_delta = math.radians(30.0)
            if abs(delta) > max_delta:
                hdg_fresh = prev.heading_error + math.copysign(max_delta, delta)
            # Lateral outlier gate: the dual-row midpoint can snap by half a
            # row spacing in one scan when peak pairing changes (one row
            # momentarily occluded).  The robot cannot physically translate
            # that fast at 10 Hz, so clamp the per-scan jump.
            d_lat = lat_fresh - prev.lateral_offset
            if abs(d_lat) > self.max_lateral_jump:
                lat_fresh = prev.lateral_offset + math.copysign(self.max_lateral_jump, d_lat)

        self._est = RowEstimate(
            heading_error=a * hdg_fresh + (1 - a) * prev.heading_error,
            lateral_offset=a * lat_fresh + (1 - a) * prev.lateral_offset,
            confidence=a * fresh.confidence + (1 - a) * prev.confidence,
            row_end_confidence=fresh.row_end_confidence,
            n_points=fresh.n_points,
            valid=True,
        )
        return self._est

    # ------------------------------------------------------------------
    def _decay(self, row_end_conf: float = 1.0) -> RowEstimate:
        """No detection this scan — keep geometry, drop confidence.

        Factor 0.75 (was 0.5) — at 10 Hz a VLP-16 can produce 1–3 empty crop
        ROI scans due to beam angle variation between rotations; 0.5 caused
        confidence to drop below the 0.35 FOLLOW threshold after just 2 empty
        scans (~200 ms), triggering spurious FOLLOW→ACQUIRE→FOLLOW cycles.
        With 0.75 it takes ~5 consecutive empty scans (~500 ms) to cross the
        threshold, which is combined with the navigator's follow_miss_thresh
        counter for a second layer of debounce.
        """
        prev = self._est
        self._est = RowEstimate(
            heading_error=prev.heading_error,
            lateral_offset=prev.lateral_offset,
            confidence=prev.confidence * 0.75,
            row_end_confidence=max(prev.row_end_confidence, row_end_conf),
            n_points=0,
            valid=False,
        )
        return self._est
