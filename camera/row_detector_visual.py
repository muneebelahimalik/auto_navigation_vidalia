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
    """Lateral offset, heading, and confidence from visual green detection."""
    lateral_offset: float = 0.0
    heading_error: float = 0.0   # rad; +ve = row angled to robot's right
    confidence: float = 0.0
    green_fraction: float = 0.0  # max green fraction seen (for row-end detection)


class VisualRowDetector:
    """Detect crop green in OAK-D RGB images to supplement LiDAR row estimates."""

    def __init__(
        self,
        cam_x_left: float = -0.915,
        cam_x_right: float = 0.915,
        cam_hfov_deg: float = 73.0,
        cam_vfov_deg: float = 54.0,
        img_width: int = 640,
        img_height: int = 400,
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
        self.cam_vfov_rad = math.radians(cam_vfov_deg)
        self.img_width = img_width
        self.img_height = img_height
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
        """Estimate lateral offset, heading, and confidence from both cameras."""
        if not _CV2_OK:
            return VisualRowEstimate()

        frac_l, offset_l, heading_l, valid_l = self._process_side(
            rgb_left, depth_left, self.cam_x_left
        )
        frac_r, offset_r, heading_r, valid_r = self._process_side(
            rgb_right, depth_right, self.cam_x_right
        )

        if not valid_l and not valid_r:
            return self._decay()

        max_frac = max(frac_l if valid_l else 0.0, frac_r if valid_r else 0.0)

        if valid_l and valid_r:
            similarity = 1.0 - min(1.0, abs(frac_l - frac_r) / (max(frac_l, frac_r) + 1e-9))
            confidence = 0.35 * similarity + 0.15
            lateral = (offset_l + offset_r) * 0.5
            heading = (heading_l + heading_r) * 0.5
        elif valid_l:
            # 0.25 not 0.20: EMA (alpha=0.30) converges to 0.25 > acquire_conf(0.20)
            # so the threshold can actually be crossed in finite frames.
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
        """Return (green_fraction, lateral_offset_m, heading_rad, valid)."""
        if rgb is None:
            return 0.0, 0.0, 0.0, False

        h, w = rgb.shape[:2]
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
            return frac, 0.0, 0.0, False

        # Centroid → lateral offset
        cols_green = np.where(mask > 0)[1] + col_lo
        rows_green = np.where(mask > 0)[0]
        centroid_px = float(cols_green.mean())
        px_offset = centroid_px - (w / 2.0)
        median_depth = self._median_depth(depth)
        m_per_px = 2.0 * median_depth * math.tan(self.cam_hfov_rad / 2.0) / w
        offset_from_cam = px_offset * m_per_px
        lateral = cam_x + offset_from_cam

        # PCA heading: fit the principal axis of green pixels in image space.
        # Image convention: col → X (right), row → Y (down = near).
        # We want the component pointing "upward" (toward top of image = forward).
        heading = 0.0
        if n_green >= 20:
            xs = cols_green.astype(np.float32)
            ys = rows_green.astype(np.float32)
            coords = np.stack([xs, ys], axis=1)
            centered = coords - coords.mean(axis=0)
            cov = (centered.T @ centered) / max(len(centered) - 1, 1)
            vals, vecs = np.linalg.eigh(cov)
            pc = vecs[:, -1]           # (dx_px, dy_px) — principal direction
            if pc[1] > 0:              # ensure PC points toward top of image (forward)
                pc = -pc
            # Only use PCA heading when the green region is elongated.
            # A round blob (row-end or isolated leaf) has nearly equal eigenvalues
            # and the principal direction is arbitrary — it flips ±90° between frames.
            linearity = (vals[-1] - vals[0]) / (vals[-1] + 1e-6) if vals[-1] > 1e-6 else 0.0
            if linearity >= 0.30:
                # Pixel slope: columns-right per row-forward (row toward smaller index)
                slope_px = pc[0] / max(-pc[1], 0.1)
                # Convert pixel slope to world heading via FOV scale factors
                h_rad_per_px = 2.0 * math.tan(self.cam_hfov_rad / 2.0) / w
                v_rad_per_px = 2.0 * math.tan(self.cam_vfov_rad / 2.0) / h
                slope_world = slope_px * h_rad_per_px / v_rad_per_px
                heading = math.atan(slope_world)

        return frac, lateral, heading, True

    def _median_depth(self, depth: Optional[np.ndarray]) -> float:
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
