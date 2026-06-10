#!/usr/bin/env python3
"""
soybean_row_tracker.py — Dual-camera flanking-row tracker for centre-residue
following in soybean fields.

Why two cameras?
----------------
The left and right OAK-D cameras are NOT a redundant stereo pair pointed at the
same scene.  Each is assigned to the soybean row on its OWN side of the central
residue strip the robot straddles:

    left  OAK-D  →  tracks the LEFT  soybean row
    right OAK-D  →  tracks the RIGHT soybean row

The robot must keep the dark residue strip centred between those two rows, so
the control target is the MIDPOINT of the two detected rows:

    centre_offset = (left_row_x + right_row_x) / 2          (robot frame, +X = right)

This differential measurement is far more robust and precise than a single
forward-looking camera:

  * Common-mode rejection — exposure / white-balance / lighting errors shared by
    both rows cancel in the midpoint.
  * Direct centring signal — drifting right brings the right row nearer and the
    left row farther, so the midpoint shifts immediately and proportionally.
  * Graceful degradation — if one row is occluded (weed, gap, shadow) the tracker
    falls back to the visible row (offset by half the row spacing) and flags
    reduced confidence instead of failing outright.
  * Heading averaging — both rows are parallel, so two independent heading
    estimates of the same direction are averaged, cutting heading noise by ~1/√2.

The output ``DualRowEstimate`` is attribute-compatible with the LiDAR
``RowEstimate`` consumers (``lateral_offset``, ``heading_error``, ``confidence``,
``green_fraction``) so it is a drop-in replacement for ``VisualRowDetector`` in
both the LiDAR+camera fusion path and the camera-only navigator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[soybean_row_tracker] opencv not installed — dual-camera tracking disabled")


@dataclass
class DualRowEstimate:
    """Residue-strip centre estimate fused from the two flanking-row cameras."""
    lateral_offset: float = 0.0     # m; +ve = residue-strip centre is to robot's right
    heading_error: float = 0.0      # rad; +ve = rows angled to robot's right
    confidence: float = 0.0         # 0..1 (camera-only range capped at ~0.5)
    green_fraction: float = 0.0     # max green fraction across both cameras (row-end cue)
    left_valid: bool = False        # left soybean row detected this frame
    right_valid: bool = False       # right soybean row detected this frame
    left_offset: float = math.nan   # detected left-row lateral position (m, robot frame)
    right_offset: float = math.nan  # detected right-row lateral position (m, robot frame)


@dataclass
class _SideResult:
    """Per-camera detection of one flanking soybean row."""
    valid: bool = False
    world_x: float = 0.0       # row lateral position in robot frame (m, +X = right)
    heading: float = 0.0       # row heading error (rad)
    green_fraction: float = 0.0
    strength: float = 0.0      # peak column green density (used to weight fusion)


class DualCameraRowTracker:
    """Track the two soybean rows flanking the centre residue strip.

    Each camera independently locks onto the single dominant crop-row line in
    its view; the residue-strip centre is the midpoint of the two rows.

    Drop-in for ``VisualRowDetector``: identical ``update()`` signature, and the
    returned estimate exposes the same attributes the navigator/EKF consume.
    """

    def __init__(
        self,
        cam_x_left: float = -0.88,
        cam_x_right: float = 0.88,
        row_spacing: float = 0.76,
        cam_hfov_deg: float = 73.0,
        cam_vfov_deg: float = 54.0,
        green_h_lo: int = 35,
        green_h_hi: int = 85,
        green_s_lo: int = 40,
        green_v_lo: int = 40,
        roi_row_lo_frac: float = 0.35,
        min_green_fraction: float = 0.05,
        min_band_pixels: int = 20,
        ema_alpha: float = 0.30,
    ) -> None:
        self.cam_x_left = cam_x_left
        self.cam_x_right = cam_x_right
        self.row_spacing = row_spacing
        self.cam_hfov_rad = math.radians(cam_hfov_deg)
        self.cam_vfov_rad = math.radians(cam_vfov_deg)
        self.green_h_lo = green_h_lo
        self.green_h_hi = green_h_hi
        self.green_s_lo = green_s_lo
        self.green_v_lo = green_v_lo
        self.roi_row_lo_frac = roi_row_lo_frac
        self.min_green_fraction = min_green_fraction
        self.min_band_pixels = min_band_pixels
        self.ema_alpha = ema_alpha
        self._est = DualRowEstimate()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the smoothed estimate (call when starting a new row)."""
        self._est = DualRowEstimate()

    # ------------------------------------------------------------------
    def update(
        self,
        rgb_left: Optional[np.ndarray],
        depth_left: Optional[np.ndarray],
        rgb_right: Optional[np.ndarray],
        depth_right: Optional[np.ndarray],
    ) -> DualRowEstimate:
        """Detect both flanking rows and fuse them into a residue-strip estimate."""
        if not _CV2_OK:
            return DualRowEstimate()

        left = self._process_side(rgb_left, depth_left, self.cam_x_left)
        right = self._process_side(rgb_right, depth_right, self.cam_x_right)
        return self._fuse(left, right)

    # ------------------------------------------------------------------
    def _fuse(self, left: _SideResult, right: _SideResult) -> DualRowEstimate:
        """Combine the two per-camera row detections into a centre estimate.

        Separated from ``update()`` (which does the OpenCV image processing) so
        the fusion geometry can be unit-tested without an OpenCV dependency.
        """
        green = max(left.green_fraction, right.green_fraction)

        if left.valid and right.valid:
            centre = 0.5 * (left.world_x + right.world_x)
            heading = 0.5 * (left.heading + right.heading)
            # Confidence rises when the two rows are detected with similar
            # strength (a balanced, well-centred view); a lopsided detection
            # (one row much stronger) is down-weighted.
            denom = max(left.strength, right.strength) + 1e-9
            similarity = 1.0 - min(1.0, abs(left.strength - right.strength) / denom)
            confidence = 0.30 + 0.20 * similarity            # 0.30 .. 0.50
        elif left.valid:
            # Only the left row visible — residue centre is half a row-spacing
            # to its right.  Keeps the correction sign correct while one side
            # is occluded, at reduced confidence.
            centre = left.world_x + 0.5 * self.row_spacing
            heading = left.heading
            confidence = 0.22
        elif right.valid:
            centre = right.world_x - 0.5 * self.row_spacing
            heading = right.heading
            confidence = 0.22
        else:
            return self._decay()

        return self._smooth(DualRowEstimate(
            lateral_offset=centre,
            heading_error=heading,
            confidence=confidence,
            green_fraction=green,
            left_valid=left.valid,
            right_valid=right.valid,
            left_offset=left.world_x if left.valid else math.nan,
            right_offset=right.world_x if right.valid else math.nan,
        ))

    # ------------------------------------------------------------------
    def _process_side(
        self,
        rgb: Optional[np.ndarray],
        depth: Optional[np.ndarray],
        cam_x: float,
    ) -> _SideResult:
        """Lock onto the dominant crop-row line in one camera's lower view."""
        if rgb is None:
            return _SideResult()

        h, w = rgb.shape[:2]
        row_lo = int(h * self.roi_row_lo_frac)
        strip = rgb[row_lo:h, :]

        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        lower = np.array([self.green_h_lo, self.green_s_lo, self.green_v_lo], dtype=np.uint8)
        upper = np.array([self.green_h_hi, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        frac = int(mask.sum() // 255) / max(mask.size, 1)
        if frac < self.min_green_fraction:
            return _SideResult(green_fraction=frac)

        # Column histogram of green → dominant vertical row band.  Locking onto
        # the single strongest column band (rather than the centroid of ALL
        # green) keeps each camera on ONE row line and prevents two adjacent
        # rows from being averaged into a phantom mid-row.
        col_density = (mask > 0).sum(axis=0).astype(np.float64)
        ksize = max(3, (w // 40) | 1)
        kernel = np.ones(ksize, dtype=np.float64) / ksize
        col_smooth = np.convolve(col_density, kernel, mode="same")

        # The flanking soybean row always appears in the INNER half of each
        # camera's image (the half facing toward the residue strip centre),
        # because the row is between the camera and the strip.  Restricting
        # the peak search to that half prevents locking onto outer rows
        # (farther from centre) which are often denser and would otherwise
        # dominate the argmax.
        #   right camera (cam_x > 0): inner half = LEFT columns (0 … w*0.6)
        #   left  camera (cam_x < 0): inner half = RIGHT columns (w*0.4 … w)
        # The 60/40 split (not 50/50) gives ±0.5 m of robot offset headroom.
        if cam_x > 0:
            search_hi = int(w * 0.60)
            peak_col = int(np.argmax(col_smooth[:search_hi]))
        else:
            search_lo = int(w * 0.40)
            peak_col = int(np.argmax(col_smooth[search_lo:])) + search_lo

        band = max(8, w // 12)
        lo = max(0, peak_col - band)
        hi = min(w, peak_col + band + 1)
        band_mask = mask[:, lo:hi]
        ys, xs_local = np.where(band_mask > 0)
        if len(xs_local) < self.min_band_pixels:
            return _SideResult(green_fraction=frac)

        xs = xs_local.astype(np.float64) + lo
        centroid_col = float(xs.mean())
        px_offset = centroid_col - (w / 2.0)
        median_depth = self._median_depth(depth)
        m_per_px = 2.0 * median_depth * math.tan(self.cam_hfov_rad / 2.0) / w
        world_x = cam_x + px_offset * m_per_px

        heading = self._heading_from_pca(xs, ys.astype(np.float64) + row_lo, w, h)

        return _SideResult(
            valid=True,
            world_x=world_x,
            heading=heading,
            green_fraction=frac,
            strength=float(col_smooth[peak_col]),
        )

    # ------------------------------------------------------------------
    def _heading_from_pca(self, xs: np.ndarray, ys: np.ndarray, w: int, h: int) -> float:
        """Row heading from the principal axis of the band's green pixels.

        Returns 0.0 for round / non-elongated blobs (row-end, isolated leaf),
        whose principal direction is arbitrary and flips ±90° between frames.
        """
        if len(xs) < self.min_band_pixels:
            return 0.0
        coords = np.stack([xs, ys], axis=1)
        centered = coords - coords.mean(axis=0)
        cov = (centered.T @ centered) / max(len(centered) - 1, 1)
        vals, vecs = np.linalg.eigh(cov)
        if vals[-1] <= 1e-6:
            return 0.0
        linearity = (vals[-1] - vals[0]) / (vals[-1] + 1e-6)
        if linearity < 0.30:
            return 0.0
        pc = vecs[:, -1]
        if pc[1] > 0:                       # orient toward top of image (forward)
            pc = -pc
        slope_px = pc[0] / max(-pc[1], 0.1)
        h_rad_per_px = 2.0 * math.tan(self.cam_hfov_rad / 2.0) / w
        v_rad_per_px = 2.0 * math.tan(self.cam_vfov_rad / 2.0) / h
        return math.atan(slope_px * h_rad_per_px / v_rad_per_px)

    # ------------------------------------------------------------------
    def _median_depth(self, depth: Optional[np.ndarray]) -> float:
        """Estimate depth to the flanking crop row.

        Using the full-frame median gives 3–4 m (dominated by far ground),
        which is 2–3× the actual plant distance (~1–1.5 m) and amplifies
        the world_x error proportionally.  Instead take the 25th percentile
        of the lower 40 % of the depth frame (close-range ground + plants).
        """
        if depth is None:
            return 1.5
        d = depth.astype(np.float32) / 1000.0
        roi = d[int(d.shape[0] * 0.60):, :]   # lower 40 % = nearest scene
        valid = roi[(roi >= 0.3) & (roi <= 4.0)]
        if len(valid) == 0:
            return 1.5
        return float(np.percentile(valid, 25))  # 25th pct ≈ nearest objects

    # ------------------------------------------------------------------
    def _smooth(self, fresh: DualRowEstimate) -> DualRowEstimate:
        """EMA blend with a heading outlier gate (mirrors RowDetector._smooth)."""
        a = self.ema_alpha
        prev = self._est

        hdg = fresh.heading_error
        if prev.confidence > 0.15:
            delta = hdg - prev.heading_error
            max_delta = math.radians(30.0)
            if abs(delta) > max_delta:
                hdg = prev.heading_error + math.copysign(max_delta, delta)

        self._est = DualRowEstimate(
            lateral_offset=a * fresh.lateral_offset + (1.0 - a) * prev.lateral_offset,
            heading_error=a * hdg + (1.0 - a) * prev.heading_error,
            confidence=a * fresh.confidence + (1.0 - a) * prev.confidence,
            green_fraction=fresh.green_fraction,
            left_valid=fresh.left_valid,
            right_valid=fresh.right_valid,
            left_offset=fresh.left_offset,
            right_offset=fresh.right_offset,
        )
        return self._est

    # ------------------------------------------------------------------
    def _decay(self) -> DualRowEstimate:
        """No row detected this frame — keep geometry, decay confidence."""
        prev = self._est
        self._est = DualRowEstimate(
            lateral_offset=prev.lateral_offset,
            heading_error=prev.heading_error,
            confidence=prev.confidence * 0.6,
            green_fraction=0.0,
            left_valid=False,
            right_valid=False,
        )
        return self._est
