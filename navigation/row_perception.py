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
from typing import Optional

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


def find_row_midpoint(
    cross: np.ndarray,
    roi_x_half: float,
    bin_width: float,
    row_spacing: float,
    weights: "Optional[np.ndarray]" = None,
) -> "tuple[float, float]":
    """Locate the residue-strip centre on the cross-row axis.

    Shared by the LiDAR ``RowDetector`` (dual-row mode) and the camera
    ``DualCameraRowTracker`` — both sensors observe the same two flanking
    crop rows, so the centre-finding logic is identical once their
    observations are expressed as cross-row coordinates in the robot frame.

    ``weights`` (optional) assigns a per-point mass to the histogram —
    used by point-level fusion to mix LiDAR points (mass 1 each) with
    camera ground points (down-weighted so the camera cannot outvote the
    LiDAR when they disagree).

    Returns ``(lateral, spacing_factor)`` where ``spacing_factor`` in
    (0, 1] penalises confidence when the detected peak geometry deviates
    from the expected row spacing.

    Pair selection uses a row-spacing prior: among all left/right peak
    combinations, the pair whose separation is closest to ``row_spacing``
    wins.  Picking the *innermost* pair instead locks onto weed clumps or
    residue clutter near the centreline whenever they produce a histogram
    peak, dragging the midpoint off the true centre.

    When only one side is visible (robot far off-centre, or one row
    occluded), the residue centre is half a row spacing INWARD from the
    visible row.  Returning the peak itself would steer the robot directly
    onto the visible crop row.
    """
    peaks = histogram_peaks(cross, roi_x_half, bin_width, weights)

    # Separate into left (x < -0.05 m) and right (x > +0.05 m) peaks.
    # The small dead-band avoids treating noise exactly at centre as a peak.
    left_peaks = [p for p in peaks if p < -0.05]
    right_peaks = [p for p in peaks if p > 0.05]

    half_spacing = 0.5 * row_spacing

    if left_peaks and right_peaks:
        best_pair = None
        best_err = float("inf")
        for pl in left_peaks:
            for pr in right_peaks:
                err = abs((pr - pl) - row_spacing)
                if err < best_err:
                    best_err = err
                    best_pair = (pl, pr)
        pl, pr = best_pair
        # Penalise confidence when even the best pair's separation is far
        # from the expected spacing (clutter posing as a row).
        spacing_factor = max(0.5, 1.0 - best_err / row_spacing)
        return 0.5 * (pl + pr), spacing_factor

    if left_peaks:
        # Only the left row visible — centre is half a spacing to its right.
        return max(left_peaks) + half_spacing, 0.7
    if right_peaks:
        return min(right_peaks) - half_spacing, 0.7

    # All peaks inside the dead-band (clutter on the residue strip itself):
    # treat as "approximately centred" rather than steering at the clutter.
    return min(peaks, key=abs), 0.5


def histogram_peaks(
    cross: np.ndarray,
    roi_x_half: float,
    bin_width: float,
    weights: "Optional[np.ndarray]" = None,
) -> "list[float]":
    """Smoothed-histogram peak positions (m) on the cross-row axis.

    Single source of the peak-detection logic used by both the midpoint
    pairing (``find_row_midpoint``) and the peak-cluster-centred PCA.
    """
    lo, hi = -roi_x_half, roi_x_half
    edges = np.arange(lo, hi + bin_width, bin_width)
    hist, edges = np.histogram(cross, bins=edges, weights=weights)
    centres = 0.5 * (edges[:-1] + edges[1:])
    smooth = np.convolve(hist.astype(float), [0.25, 0.5, 0.25], mode="same")
    thresh = max(3.0, 0.25 * float(smooth.max()))
    idx = [
        i for i in range(1, len(smooth) - 1)
        if smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]
        and smooth[i] >= thresh
    ]
    if not idx:
        idx = [int(np.argmax(smooth))]
    return [float(centres[i]) for i in idx]


def _weighted_pca_dir(P: np.ndarray, w: Optional[np.ndarray] = None) -> "tuple[np.ndarray, float]":
    """Principal direction (forward-positive) and linearity of 2D points.

    With ``w=None`` this is byte-identical to the original unweighted PCA;
    with weights it computes the weighted mean/covariance so down-weighted
    camera points influence the fit proportionally to their mass.
    """
    if w is None:
        mean = P.mean(axis=0)
        Q = P - mean
        cov = (Q.T @ Q) / len(Q)
    else:
        W = float(w.sum())
        mean = (w[:, None] * P).sum(axis=0) / W
        Q = P - mean
        cov = (Q.T @ (w[:, None] * Q)) / W
    evals, evecs = np.linalg.eigh(cov)          # ascending
    direction = evecs[:, 1]
    if direction[1] < 0.0:                       # forward-positive
        direction = -direction
    lam_major, lam_minor = float(evals[1]), float(evals[0])
    linearity = (lam_major - lam_minor) / (lam_major + lam_minor + 1e-9)
    return direction, linearity


def _cluster_centred_pca(
    P: np.ndarray,
    w: np.ndarray,
    cross: np.ndarray,
    peak_positions: "list[float]",
    fallback: "tuple[np.ndarray, float]",
) -> "tuple[np.ndarray, float]":
    """Within-row PCA: assign every point to its nearest histogram peak,
    centre each peak cluster on its own (weighted) centroid, and fit the
    pooled centred points.

    Removes the between-stripe covariance that tilts a whole-cloud PCA when
    the stripes have unequal extents (different sensor coverage) — and,
    because clustering is per PEAK rather than a two-way split, stripes
    contributed by a mis-calibrated sensor form their own clusters and
    cannot tilt the heading either.  Returns ``fallback`` when no usable
    cluster remains.
    """
    peaks = np.asarray(peak_positions, dtype=float)
    if len(peaks) == 0:
        return fallback
    assign = np.argmin(np.abs(cross[:, None] - peaks[None, :]), axis=1)
    total = float(w.sum())
    parts_p, parts_w = [], []
    for k in range(len(peaks)):
        sel = assign == k
        if not sel.any():
            continue
        ws = w[sel]
        mass = float(ws.sum())
        if mass < 0.05 * total:
            continue                      # degenerate sliver — skip
        C = P[sel]
        centroid = (ws[:, None] * C).sum(axis=0) / mass
        parts_p.append(C - centroid)
        parts_w.append(ws)
    if not parts_p:
        return fallback
    return _weighted_pca_dir(np.vstack(parts_p), np.concatenate(parts_w))


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
        bin_width: float = 0.05,
        row_end_density: float = 70.0,
        ema_alpha: float = 0.35,
        dual_row: bool = False,
        row_spacing: float = 0.76,
        max_lateral_jump: float = 0.20,
        aux_y_min: float = 0.5,
        aux_mass_ratio: float = 0.5,
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
        # Point-level fusion of camera ground points (aux_xy in update()):
        # aux_y_min lets near-field camera points (LiDAR self-filter blind
        # zone) into the fit; aux_mass_ratio caps the camera's total
        # histogram mass at this fraction of full_points so the camera can
        # never outvote a healthy LiDAR scan.
        self.aux_y_min = aux_y_min
        self.aux_mass_ratio = aux_mass_ratio
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
    def update(self, pts: np.ndarray, aux_xy: Optional[np.ndarray] = None) -> RowEstimate:
        """
        Detect the centre row/residue from an Nx3 (x, y, z) sensor-frame
        point array.

        ``aux_xy`` (optional, point-level fusion): Mx2 (x, y) metric ground
        points in the same robot frame — e.g. the camera tracker's
        ground-projected green pixels.  When provided, both sensors'
        evidence is pooled into ONE weighted row fit instead of fusing two
        independently fitted estimates.  Camera points are down-weighted so
        their total histogram mass never exceeds ``aux_mass_ratio ×
        full_points`` — a healthy LiDAR scan always outvotes the camera,
        while an empty LiDAR scan can still be carried by camera evidence
        at proportionally reduced confidence.

        Returns a smoothed RowEstimate.  When no row is found the previous
        estimate is decayed (confidence reduced, row-end confidence raised)
        so the state machine can react instead of acting on a stale fix.
        """
        # --- LiDAR crop-band points ---
        if pts is not None and len(pts) > 0:
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
            h = z + LIDAR_MOUNT_HEIGHT
            roi = (
                (y >= self.roi_y_min) & (y <= self.roi_y_max)
                & (np.abs(x) <= self.roi_x_half)
                & (h >= self.crop_h_min) & (h <= self.crop_h_max)
            )
            cx, cy = x[roi], y[roi]
            n = int(cx.shape[0])
        else:
            cx = cy = np.empty(0)
            n = 0

        # Row-end confidence stays LiDAR-only: the camera's contribution to
        # row-end detection is the separate green-fraction veto, and camera
        # mass must not be able to mask a genuinely sparse crop band ahead.
        row_end_conf = 1.0 - min(1.0, n / self.row_end_density)

        # --- auxiliary camera ground points (same robot frame) ---
        A = None
        if aux_xy is not None and len(aux_xy):
            ax, ay = aux_xy[:, 0], aux_xy[:, 1]
            keep = (
                (ay >= self.aux_y_min) & (ay <= self.roi_y_max)
                & (np.abs(ax) <= self.roi_x_half)
            )
            A = aux_xy[keep]
            if len(A) == 0:
                A = None

        cam_mass = 0.0
        if A is not None:
            cam_mass = min(float(len(A)), self.aux_mass_ratio * self.full_points)

        total_mass = n + cam_mass
        if total_mass < self.min_points:
            return self._decay(row_end_conf=row_end_conf)

        # --- pooled point set + per-point weights ---
        P = np.column_stack((cx, cy))
        if A is not None:
            w = np.concatenate([np.ones(n), np.full(len(A), cam_mass / len(A))])
            P = np.vstack([P, A]) if n else np.asarray(A, dtype=float)
        else:
            w = None   # pure-LiDAR path: identical to the original unweighted fit

        # --- PCA: dominant axis = row direction (rows are parallel) ---
        # Heading seed for mixed-sensor fits comes from the LiDAR points
        # alone when the LiDAR is healthy: its symmetric two-stripe view
        # gives an unbiased axis, whereas pooling a (possibly mis-calibrated)
        # camera before the stripes are clustered can tilt the axis enough
        # to smear the cross-row histogram and merge adjacent peaks.
        if w is not None and n >= self.min_points:
            direction, linearity = _weighted_pca_dir(P[:n])
        else:
            direction, linearity = _weighted_pca_dir(P, w)
        perp = np.array([direction[1], -direction[0]])   # unit, +X-ish
        cross = P @ perp

        if self.dual_row:
            if w is not None:
                # Two-pass peak-cluster-centred PCA (mixed-sensor fits only):
                # LiDAR and camera cover different y ranges, so the pooled
                # cloud has stripes with unequal extents and whole-cloud PCA
                # picks up a between-stripe covariance tilt; a mis-calibrated
                # camera adds extra stripes that tilt it further.  Assigning
                # every point to its nearest histogram peak and centring each
                # peak cluster on its own centroid leaves only the within-row
                # direction.  The pure-LiDAR path keeps the original
                # single-pass fit (field-validated symmetric view; behaviour
                # unchanged).
                peaks = histogram_peaks(cross, self.roi_x_half,
                                        self.bin_width, weights=w)
                direction, linearity = _cluster_centred_pca(
                    P, w, cross, peaks, fallback=(direction, linearity))
                perp = np.array([direction[1], -direction[0]])
                cross = P @ perp
            # Soybean / centre-residue mode: find the midpoint between the
            # nearest left and right crop-row peaks.  The midpoint is the
            # centre of the residue strip, which is the tracking target.
            lateral, self._spacing_factor = find_row_midpoint(
                cross, self.roi_x_half, self.bin_width, self.row_spacing,
                weights=w)
        else:
            # Single-row mode: histogram peak nearest the robot centreline,
            # then refine with a tight window so adjacent rows are not merged.
            peak = self._nearest_peak(cross, weights=w)
            near = np.abs(cross - peak) <= self.refine_window
            if int(near.sum()) >= 8:
                lateral = float(np.average(cross[near],
                                           weights=None if w is None else w[near]))
            else:
                lateral = peak

        heading = math.atan2(direction[0], direction[1])
        density = min(1.0, total_mass / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
        confidence = density * linear_factor
        if self.dual_row:
            confidence *= self._spacing_factor

        return self._smooth(RowEstimate(
            heading_error=heading,
            lateral_offset=lateral,
            confidence=confidence,
            row_end_confidence=row_end_conf,
            n_points=int(round(total_mass)),
            valid=True,
        ))

    # ------------------------------------------------------------------
    def _nearest_peak(self, cross: np.ndarray, weights: Optional[np.ndarray] = None) -> float:
        """Histogram the cross-row axis; return the peak nearest centreline."""
        lo, hi = -self.roi_x_half, self.roi_x_half
        edges = np.arange(lo, hi + self.bin_width, self.bin_width)
        hist, edges = np.histogram(cross, bins=edges, weights=weights)
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
        """Dual-row / soybean centre-residue mode: locate the residue-strip
        centre via the shared ``find_row_midpoint`` (row-spacing prior +
        single-side half-spacing fallback)."""
        lateral, self._spacing_factor = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, self.row_spacing)
        return lateral

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
