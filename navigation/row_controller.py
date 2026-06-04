#!/usr/bin/env python3
"""
row_controller.py — Pure-pursuit steering for centre-row following.

Given a RowEstimate (heading error, lateral offset, confidence) the
controller places a look-ahead target on the crop-row centreline and
produces a (linear, angular) velocity command that drives the robot
centreline onto the row.

Sign convention (matches CanbusInterface):
  linear  > 0  — forward
  angular > 0  — counter-clockwise (left turn)

Pure pursuit naturally folds heading error and lateral offset into a
single curvature, so no separate gains are needed.  Speed is throttled
by row confidence and heading error so the robot slows down — rather than
swerving — whenever the row fix is weak.
"""

from __future__ import annotations

import math

from navigation.row_perception import RowEstimate


class PurePursuitController:
    """Look-ahead pure-pursuit controller for crop-row following."""

    def __init__(
        self,
        max_linear: float = 0.30,
        min_linear: float = 0.08,
        max_angular: float = 0.40,
        lookahead: float = 2.0,
        min_confidence: float = 0.35,
    ) -> None:
        self.max_linear = max_linear
        self.min_linear = min_linear
        self.max_angular = max_angular
        self.lookahead = lookahead
        self.min_confidence = min_confidence

    # ------------------------------------------------------------------
    def compute(self, est: RowEstimate) -> tuple[float, float]:
        """
        Return (linear, angular) for the given row estimate.

        Returns (0, 0) when confidence is below threshold — the caller's
        state machine is responsible for re-acquiring the row.
        """
        if est.confidence < self.min_confidence:
            return 0.0, 0.0

        theta = est.heading_error
        offset = est.lateral_offset
        L = self.lookahead

        # Look-ahead target on the row centreline, at forward distance ~L.
        # centreline: point = offset * perp + s * dir
        #   dir  = (sinθ, cosθ)   perp = (cosθ, -sinθ)
        # solve forward coord y = L  ->  s, then read target X.
        cos_t = max(math.cos(theta), 0.10)
        s = (L + offset * math.sin(theta)) / cos_t
        target_x = offset * math.cos(theta) + s * math.sin(theta)
        target_y = max(-offset * math.sin(theta) + s * math.cos(theta), 0.5)

        # Speed: scale down with weak confidence and large heading error.
        # Divisor 1.0 rad keeps the robot near half-speed at a typical 22° entry
        # angle; the old 0.60 bottomed out at 35% (~0.10 m/s) which cut angular
        # authority so much that lateral correction took 5+ seconds to converge.
        speed_factor = est.confidence * max(0.25, 1.0 - abs(theta) / 1.0)
        linear = max(self.min_linear, self.max_linear * speed_factor)

        # Pure-pursuit curvature; +target_x (row to the right) -> turn right.
        dist_sq = target_x * target_x + target_y * target_y
        curvature = 2.0 * target_x / dist_sq
        angular = -linear * curvature
        angular = max(-self.max_angular, min(self.max_angular, angular))

        return linear, angular
