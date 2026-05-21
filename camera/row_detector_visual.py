from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[row_detector_visual] opencv not installed — visual row detection disabled")


@dataclass
class VisualRowEstimate:
    """Lateral offset and confidence boost from visual green detection."""
    lateral_offset: float = 0.0
    confidence: float = 0.0


class VisualRowDetector:
    """Detect crop green in OAK-D RGB images to supplement LiDAR row estimates."""

    def __init__(
        self,
        cam_x_left: float = -0.915,
        cam_x_right: float = 0.915,
        cam_hfov_deg: float = 73.0,
        img_width: int = 640,
        green_h_lo: int = 35,
        green_h_hi: int = 85,
        green_s_lo: int = 40,
        green_v_lo: int = 40,
        min_green_fraction: float = 0.08,
        ema_alpha: float = 0.30,
    ) -> None:
        self.cam_x_left = cam_x_left
        self.cam_x_right = cam_x_right
        self.cam_hfov_rad = math.radians(cam_hfov_deg)
        self.img_width = img_width
        self.green_h_lo = green_h_lo
        self.green_h_hi = green_h_hi
        self.green_s_lo = green_s_lo
        self.green_v_lo = green_v_lo
        self.min_green_fraction = min_green_fraction
        self.ema_alpha = ema_alpha
        self._est = VisualRowEstimate()

    def update(
        self,
        rgb_left: Optional[np.ndarray],
        depth_left: Optional[np.ndarray],
        rgb_right: Optional[np.ndarray],
        depth_right: Optional[np.ndarray],
    ) -> VisualRowEstimate:
        """Estimate lateral offset and confidence from both camera images."""
        if not _CV2_OK:
            return VisualRowEstimate()

        result_left = self._process_side(rgb_left, depth_left, self.cam_x_left)
        result_right = self._process_side(rgb_right, depth_right, self.cam_x_right)

        frac_l, offset_l, valid_l = result_left
        frac_r, offset_r, valid_r = result_right

        if not valid_l and not valid_r:
            return self._decay()

        # Confidence is moderate when both sides see green with similar fractions.
        if valid_l and valid_r:
            similarity = 1.0 - min(1.0, abs(frac_l - frac_r) / (max(frac_l, frac_r) + 1e-9))
            confidence = 0.35 * similarity + 0.15
            lateral = (offset_l + offset_r) * 0.5
        elif valid_l:
            confidence = 0.20
            lateral = offset_l
        else:
            confidence = 0.20
            lateral = offset_r

        return self._smooth(VisualRowEstimate(lateral_offset=lateral, confidence=confidence))

    def _process_side(
        self,
        rgb: Optional[np.ndarray],
        depth: Optional[np.ndarray],
        cam_x: float,
    ):
        """Return (green_fraction, lateral_offset_m, valid) for one camera."""
        if rgb is None:
            return 0.0, 0.0, False

        h, w = rgb.shape[:2]
        # Crop: middle third horizontally, lower two-thirds vertically.
        col_lo = w // 3
        col_hi = w - w // 3
        row_lo = h // 3
        strip = rgb[row_lo:h, col_lo:col_hi]

        hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
        lower = np.array([self.green_h_lo, self.green_s_lo, self.green_v_lo], dtype=np.uint8)
        upper = np.array([self.green_h_hi, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        total = mask.size
        n_green = int(mask.sum() // 255)
        frac = n_green / max(total, 1)

        if frac < self.min_green_fraction:
            return frac, 0.0, False

        # Centroid of green pixels in the full-width coordinate of the strip.
        cols_green = np.where(mask > 0)[1] + col_lo
        centroid_px = float(cols_green.mean())

        # Pixel offset from image centre, positive = right.
        px_offset = centroid_px - (w / 2.0)
        # Convert pixel offset to metres using hfov and a median depth estimate.
        median_depth = self._median_depth(depth)
        m_per_px = 2.0 * median_depth * math.tan(self.cam_hfov_rad / 2.0) / w
        offset_from_cam = px_offset * m_per_px

        # The row centre should appear on the inward side of each camera.
        # Left camera (cam_x < 0): row is to the right of centre (+offset_from_cam
        # means row is further right, so robot is to the left of the row -> +ve lateral).
        # Right camera (cam_x > 0): row is to the left of centre.
        # We express lateral_offset as: row is to the right of robot centreline = +ve.
        if cam_x < 0:
            lateral = cam_x + offset_from_cam
        else:
            lateral = cam_x + offset_from_cam

        return frac, lateral, True

    def _median_depth(self, depth: Optional[np.ndarray]) -> float:
        """Return median valid depth in metres, defaulting to 1.0 m."""
        if depth is None:
            return 1.0
        d = depth.astype(np.float32) / 1000.0
        valid = d[(d >= 0.3) & (d <= 10.0)]
        if len(valid) == 0:
            return 1.0
        return float(np.median(valid))

    def _smooth(self, fresh: VisualRowEstimate) -> VisualRowEstimate:
        a = self.ema_alpha
        prev = self._est
        self._est = VisualRowEstimate(
            lateral_offset=a * fresh.lateral_offset + (1.0 - a) * prev.lateral_offset,
            confidence=a * fresh.confidence + (1.0 - a) * prev.confidence,
        )
        return self._est

    def _decay(self) -> VisualRowEstimate:
        prev = self._est
        self._est = VisualRowEstimate(
            lateral_offset=prev.lateral_offset,
            confidence=prev.confidence * 0.5,
        )
        return self._est
