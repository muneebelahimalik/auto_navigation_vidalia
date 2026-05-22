from __future__ import annotations

import math
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[row_detector_depth_edge] opencv not installed — depth-edge row detection disabled")

from camera.row_detector_visual import VisualRowEstimate


class DepthEdgeRowDetector:
    """Geometry-based row detector using Canny/Hough heading and stereo-depth gap lateral offset."""

    def __init__(
        self,
        cam_x_left: float = -0.915,
        cam_x_right: float = 0.915,
        cam_hfov_deg: float = 73.0,
        cam_vfov_deg: float = 54.0,
        img_width: int = 640,
        img_height: int = 400,
        canny_lo: int = 30,
        canny_hi: int = 100,
        hough_threshold: int = 25,
        hough_min_length: int = 40,
        hough_max_gap: int = 20,
        min_lines: int = 3,
        depth_band_frac: float = 0.40,
        depth_gap_smooth: int = 25,
        min_depth_contrast: float = 0.12,
        ema_alpha: float = 0.35,
    ) -> None:
        self.cam_x_left = cam_x_left
        self.cam_x_right = cam_x_right
        self.cam_hfov_rad = math.radians(cam_hfov_deg)
        self.cam_vfov_rad = math.radians(cam_vfov_deg)
        self.img_width = img_width
        self.img_height = img_height
        self.canny_lo = canny_lo
        self.canny_hi = canny_hi
        self.hough_threshold = hough_threshold
        self.hough_min_length = hough_min_length
        self.hough_max_gap = hough_max_gap
        self.min_lines = min_lines
        self.depth_band_frac = depth_band_frac
        self.depth_gap_smooth = depth_gap_smooth
        self.min_depth_contrast = min_depth_contrast
        self.ema_alpha = ema_alpha
        self._est = VisualRowEstimate()

    def update(
        self,
        rgb_left: Optional[np.ndarray],
        depth_left: Optional[np.ndarray],
        rgb_right: Optional[np.ndarray],
        depth_right: Optional[np.ndarray],
    ) -> VisualRowEstimate:
        if not _CV2_OK:
            return VisualRowEstimate()

        row_frac_l, offset_l, heading_l, valid_l = self._process_side(
            rgb_left, depth_left, self.cam_x_left
        )
        row_frac_r, offset_r, heading_r, valid_r = self._process_side(
            rgb_right, depth_right, self.cam_x_right
        )

        if not valid_l and not valid_r:
            return self._decay()

        max_frac = max(row_frac_l if valid_l else 0.0, row_frac_r if valid_r else 0.0)

        if valid_l and valid_r:
            similarity = 1.0 - min(
                1.0,
                abs(row_frac_l - row_frac_r) / (max(row_frac_l, row_frac_r) + 1e-9),
            )
            confidence = 0.35 * similarity + 0.15
            lateral = (offset_l + offset_r) * 0.5
            heading = (heading_l + heading_r) * 0.5
        elif valid_l:
            confidence = 0.25
            lateral = offset_l
            heading = heading_l
        else:
            confidence = 0.25
            lateral = offset_r
            heading = heading_r

        return self._smooth(VisualRowEstimate(
            lateral_offset=lateral,
            heading_error=heading,
            confidence=confidence,
            green_fraction=max_frac,
        ))

    def _process_side(
        self,
        rgb: Optional[np.ndarray],
        depth: Optional[np.ndarray],
        cam_x: float,
    ):
        """Return (row_fraction, lateral_offset_m, heading_rad, valid)."""
        line_conf, heading = self._heading_from_edges(rgb)
        depth_contrast, lateral = self._lateral_from_depth(depth, cam_x)

        depth_conf = 0.0
        if depth_contrast > self.min_depth_contrast:
            depth_conf = min(1.0, depth_contrast / 0.25)

        valid = line_conf > 0.0 or depth_conf > 0.0
        if not valid:
            return 0.0, 0.0, 0.0, False

        # Cap single-camera contribution at 0.50 per spec.
        confidence = min(0.50, 0.55 * line_conf + 0.35 * depth_conf + 0.10)
        row_fraction = max(line_conf, depth_conf)

        return row_fraction, lateral, heading, True

    def _heading_from_edges(self, rgb: Optional[np.ndarray]):
        """Return (line_conf, heading_rad). line_conf=0 when fewer than min_lines survive."""
        if rgb is None:
            return 0.0, 0.0

        h, w = rgb.shape[:2]
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_lo, self.canny_hi)

        roi_top = h // 3
        roi = edges[roi_top:h, :]

        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_length,
            maxLineGap=self.hough_max_gap,
        )

        if lines is None:
            return 0.0, 0.0

        slopes = []
        for x1, y1, x2, y2 in lines[:, 0]:
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            # Keep only lines that run primarily in the vertical (row) direction.
            if abs(dy) <= abs(dx):
                continue
            # slope_px: px-right per px-down; negate for "toward top" convention
            slopes.append(-dx / abs(dy))

        if len(slopes) < self.min_lines:
            return 0.0, 0.0

        slopes = np.array(slopes, dtype=np.float32)
        median_slope = float(np.median(slopes))
        inliers = slopes[np.abs(slopes - median_slope) <= 0.5]

        if len(inliers) < self.min_lines:
            return 0.0, 0.0

        mean_slope = float(inliers.mean())
        h_rad_per_px = 2.0 * math.tan(self.cam_hfov_rad / 2.0) / w
        v_rad_per_px = 2.0 * math.tan(self.cam_vfov_rad / 2.0) / h
        slope_world = mean_slope * h_rad_per_px / v_rad_per_px
        heading = math.atan(slope_world)

        line_conf = min(1.0, len(inliers) / 8.0)
        return line_conf, heading

    def _lateral_from_depth(self, depth: Optional[np.ndarray], cam_x: float):
        """Return (depth_contrast, lateral_offset_m)."""
        if depth is None:
            return 0.0, cam_x

        h, w = depth.shape[:2] if depth.ndim == 2 else (depth.shape[0], depth.shape[1])
        band_top = h // 3
        band_bottom = band_top + int(h * self.depth_band_frac)
        band = depth[band_top:band_bottom, :].astype(np.float32) / 1000.0

        # Mask out invalid depth readings.
        band[(band < 0.3) | (band > 8.0)] = np.nan

        col_means = np.nanmean(band, axis=0)

        global_mean = float(np.nanmean(col_means))
        if np.isnan(global_mean):
            return 0.0, cam_x

        nan_mask = np.isnan(col_means)
        col_means[nan_mask] = global_mean

        # GaussianBlur requires a 2-D array even for 1-D horizontal smoothing.
        col_row = col_means.reshape(1, -1).astype(np.float32)
        # kernel size must be odd
        k = self.depth_gap_smooth if self.depth_gap_smooth % 2 == 1 else self.depth_gap_smooth + 1
        col_smooth = cv2.GaussianBlur(col_row, (k, 1), 0).reshape(-1)

        gap_col = int(np.argmax(col_smooth))
        gap_depth = float(col_smooth[gap_col])
        mean_smooth = float(np.mean(col_smooth))
        depth_contrast = (gap_depth - mean_smooth) / (mean_smooth + 1e-6)

        if depth_contrast <= self.min_depth_contrast:
            return depth_contrast, cam_x

        m_per_px = 2.0 * gap_depth * math.tan(self.cam_hfov_rad / 2.0) / w
        lateral = cam_x + (gap_col - w / 2.0) * m_per_px
        return depth_contrast, lateral

    def _smooth(self, fresh: VisualRowEstimate) -> VisualRowEstimate:
        a = self.ema_alpha
        prev = self._est
        self._est = VisualRowEstimate(
            lateral_offset=a * fresh.lateral_offset + (1.0 - a) * prev.lateral_offset,
            heading_error=a * fresh.heading_error + (1.0 - a) * prev.heading_error,
            confidence=a * fresh.confidence + (1.0 - a) * prev.confidence,
            green_fraction=fresh.green_fraction,
        )
        return self._est

    def _decay(self) -> VisualRowEstimate:
        prev = self._est
        self._est = VisualRowEstimate(
            lateral_offset=prev.lateral_offset,
            heading_error=prev.heading_error,
            confidence=prev.confidence * 0.5,
            green_fraction=0.0,
        )
        return self._est
