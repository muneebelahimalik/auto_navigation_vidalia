from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class DepthObstacleStatus:
    """Result of one depth-frame obstacle check."""
    blocked: bool = False
    nearest_m: float = math.inf
    side: str = ""

    def reason(self) -> str:
        if self.blocked:
            return f"CAM-{self.side.upper()}@{self.nearest_m:.1f}m"
        return "clear"


class DepthObstacleDetector:
    """Detect close obstacles in the blind zone using OAK-D depth frames."""

    def __init__(
        self,
        stop_dist_m: float = 1.2,
        min_dist_m: float = 0.30,
        fwd_band_frac: float = 0.40,
        fwd_half_frac: float = 0.25,
        col_centre_frac: float = 0.5,
        min_pixels: int = 50,
        img_height: int = 400,
        img_width: int = 640,
    ) -> None:
        self.stop_dist_m = stop_dist_m
        self.min_dist_m = min_dist_m
        self.min_pixels = min_pixels

        # Precompute the centre-strip slice indices.
        # col_centre_frac shifts the horizontal centre of the strip:
        #   0.5 = image centre (default, valid for forward-facing cameras)
        #   0.80 = inner edge for left side-camera (robot path at ~col 480)
        #   0.20 = inner edge for right side-camera (robot path at ~col 160)
        row_half = int(img_height * fwd_band_frac / 2)
        col_half = int(img_width * fwd_half_frac / 2)
        r_mid = img_height // 2
        c_mid = int(img_width * col_centre_frac)
        self._row_sl = slice(r_mid - row_half, r_mid + row_half)
        self._col_sl = slice(
            max(0, c_mid - col_half),
            min(img_width, c_mid + col_half),
        )

    def check(self, depth: Optional[np.ndarray], side: str) -> DepthObstacleStatus:
        """Return obstacle status for one depth frame (uint16 mm)."""
        if depth is None:
            return DepthObstacleStatus(side=side)

        strip = depth[self._row_sl, self._col_sl].astype(np.float32) / 1000.0
        valid = (strip >= self.min_dist_m) & (strip <= 50.0)
        close = valid & (strip < self.stop_dist_m)
        n_close = int(close.sum())

        nearest = math.inf
        if n_close:
            nearest = float(strip[close].min())

        return DepthObstacleStatus(
            blocked=n_close >= self.min_pixels,
            nearest_m=nearest,
            side=side,
        )
