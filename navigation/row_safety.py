#!/usr/bin/env python3
"""
row_safety.py — Forward and wheel-track obstacle monitoring for row following.

The Amiga straddles the crop, so the crop itself passes harmlessly under the
robot body — it is NOT an obstacle.  Three zones are watched instead:

  * Forward zone  — the robot's path ahead.  Only returns ABOVE crop height
    count, so onion plants do not trip a stop but a person, post or animal
    does.
  * Left / right tire zones — the wheel corridors, which extend wider than
    the body zone.  Like the forward zone they only count returns ABOVE
    crop-canopy height, so adjacent crop rows are ignored while a post,
    animal or piece of equipment in the wheel path triggers a stop.

Any occupied zone makes the state machine halt and wait.  Pure geometry,
sensor frame X=right, Y=forward, Z=up; ground-relative height
h = z + LIDAR_MOUNT_HEIGHT.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT


@dataclass
class SafetyStatus:
    """Result of one safety scan."""
    forward_blocked: bool = False
    left_tire_blocked: bool = False
    right_tire_blocked: bool = False
    nearest_forward: float = math.inf
    fwd_points: int = 0
    left_points: int = 0
    right_points: int = 0
    cam_blocked: bool = False
    cam_reason: str = ""

    @property
    def blocked(self) -> bool:
        return self.forward_blocked or self.left_tire_blocked or self.right_tire_blocked or self.cam_blocked

    def reason(self) -> str:
        parts = []
        if self.forward_blocked:
            parts.append(f"FWD@{self.nearest_forward:.1f}m(n={self.fwd_points})")
        if self.left_tire_blocked:
            parts.append(f"L-TIRE(n={self.left_points})")
        if self.right_tire_blocked:
            parts.append(f"R-TIRE(n={self.right_points})")
        if self.cam_blocked:
            parts.append(self.cam_reason if self.cam_reason else "CAM")
        return ",".join(parts) if parts else "clear"


class SafetyMonitor:
    """Per-scan obstacle check for the forward path and both wheel tracks."""

    def __init__(
        self,
        forward_dist: float = 2.5,
        forward_half_width: float = 0.60,
        obstacle_height: float = 0.75,
        tire_track: float = 0.915,
        tire_half_width: float = 0.25,
        tire_dist: float = 2.5,
        tire_obstacle_height: float = 0.75,
        near: float = 0.20,
        forward_min_points: int = 4,
        tire_min_points: int = 4,
    ) -> None:
        self.forward_dist = forward_dist
        self.forward_half_width = forward_half_width
        self.obstacle_height = obstacle_height
        self.tire_track = tire_track
        self.tire_half_width = tire_half_width
        self.tire_dist = tire_dist
        self.tire_obstacle_height = tire_obstacle_height
        self.near = near
        self.forward_min_points = forward_min_points
        self.tire_min_points = tire_min_points

    # ------------------------------------------------------------------
    def check(self, pts: np.ndarray) -> SafetyStatus:
        """Classify obstacles in the three zones from an Nx3 point array."""
        if pts is None or len(pts) == 0:
            return SafetyStatus()

        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        h = z + LIDAR_MOUNT_HEIGHT
        ahead = y > self.near

        fwd = (
            ahead & (y <= self.forward_dist)
            & (np.abs(x) <= self.forward_half_width)
            & (h >= self.obstacle_height)
        )
        fwd_n = int(fwd.sum())
        nearest = math.inf
        if fwd_n:
            nearest = float(np.min(np.hypot(x[fwd], y[fwd])))

        tire = ahead & (y <= self.tire_dist) & (h >= self.tire_obstacle_height)
        lo, hi = self.tire_track - self.tire_half_width, self.tire_track + self.tire_half_width
        left_n = int((tire & (x <= -lo) & (x >= -hi)).sum())
        right_n = int((tire & (x >= lo) & (x <= hi)).sum())

        return SafetyStatus(
            forward_blocked=fwd_n >= self.forward_min_points,
            left_tire_blocked=left_n >= self.tire_min_points,
            right_tire_blocked=right_n >= self.tire_min_points,
            nearest_forward=nearest,
            fwd_points=fwd_n,
            left_points=left_n,
            right_points=right_n,
        )
