#!/usr/bin/env python3
"""
nav_logic.py — Compute Twist2d velocity commands from VLP-16 point cloud data.

Data flow:
    LidarDriver  →  NavLogic.run()  →  CanbusInterface  →  Amiga wheels

This implements a reactive obstacle-avoidance behaviour as a foundation for
the weed-control navigation task.  The robot drives forward at up to MAX_LINEAR
m/s and slows/stops when the LiDAR detects an obstacle within the forward
safety corridor.

For full field coverage with boustrophedon row patterns, see:
    src/vidalia_bringup/vidalia_bringup/autonomous_row_coverage.py  (ROS 2)
    src/vidalia_bringup/vidalia_bringup/field_coverage_planner.py   (ROS 2)

These modules can be ported to the native Python stack once ROS 2 is available,
or adapted to publish waypoints via EventServiceGrpc.
"""

from __future__ import annotations

import asyncio
import math
from typing import List

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver, VelodynePoint

# ---------------------------------------------------------------------------
# Safety geometry (metres)
# ---------------------------------------------------------------------------
# Amiga body width = 0.93 m; half-width + 0.135 m clearance = 0.60 m
ROBOT_HALF_WIDTH = 0.60

# Distance ahead to monitor for obstacles
LOOK_AHEAD_DIST = 2.0

# Hard-stop threshold — stop immediately if an obstacle is within this range
STOP_DISTANCE = 0.80

# Minimum height above ground to count as an obstacle (filter spurious ground returns)
MIN_OBSTACLE_HEIGHT = 0.10   # metres

# Maximum speed for field operation (conservative)
MAX_LINEAR = 0.8   # m/s


def _front_zone_points(
    points: List[VelodynePoint],
    look_ahead: float = LOOK_AHEAD_DIST,
    half_width: float = ROBOT_HALF_WIDTH,
    min_height: float = MIN_OBSTACLE_HEIGHT,
) -> List[VelodynePoint]:
    """
    Return only LiDAR points inside the forward safety corridor.

    The corridor is a rectangular prism:
        Y in (0, look_ahead]   — directly ahead of the robot
        |X| < half_width       — within robot width + clearance
        Z > min_height         — above ground (filters road surface returns)
    """
    return [
        p for p in points
        if 0.0 < p.y <= look_ahead
        and abs(p.x) < half_width
        and p.z > min_height
    ]


def _nearest_range(zone_points: List[VelodynePoint]) -> float:
    """Return the planar range to the closest point in the zone, or inf."""
    if not zone_points:
        return math.inf
    return min(math.hypot(p.x, p.y) for p in zone_points)


class NavLogic:
    """
    Reactive forward navigation with LiDAR obstacle avoidance.

    Speed control:
        nearest > LOOK_AHEAD_DIST  → MAX_LINEAR m/s (clear path)
        STOP_DISTANCE < nearest ≤ LOOK_AHEAD_DIST → linearly scaled speed
        nearest ≤ STOP_DISTANCE   → 0 m/s (stop)

    Usage::

        canbus = CanbusInterface(configs["canbus"])
        nav = NavLogic(canbus)
        async with LidarDriver() as lidar:
            await nav.run(lidar)
    """

    def __init__(self, canbus: CanbusInterface) -> None:
        self._canbus = canbus
        self._running = False

    async def run(self, lidar: LidarDriver) -> None:
        """Main control loop.  Runs until stop() is called or lidar closes."""
        self._running = True
        async for scan in lidar.scan_stream():
            if not self._running:
                break

            front = _front_zone_points(scan)
            nearest = _nearest_range(front)

            if nearest <= STOP_DISTANCE:
                await self._canbus.stop()
            elif nearest <= LOOK_AHEAD_DIST:
                # Scale speed linearly: 0 at STOP_DISTANCE → MAX_LINEAR at LOOK_AHEAD_DIST
                ratio = (nearest - STOP_DISTANCE) / (LOOK_AHEAD_DIST - STOP_DISTANCE)
                await self._canbus.send_twist(MAX_LINEAR * ratio, 0.0)
            else:
                await self._canbus.send_twist(MAX_LINEAR, 0.0)

    async def stop(self) -> None:
        """Signal the run loop to exit and send a final stop command."""
        self._running = False
        await self._canbus.stop()
