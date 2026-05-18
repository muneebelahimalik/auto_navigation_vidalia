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
"""

from __future__ import annotations

import asyncio

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import (  # noqa: F401 (re-exported for callers)
    LIDAR_MOUNT_HEIGHT,
    LOOK_AHEAD_DIST,
    MIN_OBSTACLE_GROUND_HEIGHT,
    ROBOT_HALF_WIDTH,
    front_zone_points,
    nearest_range,
    nearest_in_sector,
    sector_counts,
    validate_lidar_startup,
)

# Maximum speed for field operation
MAX_LINEAR = 0.8    # m/s
STOP_DISTANCE = 0.80  # metres — hard-stop threshold


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

    async def run(self, lidar: LidarDriver, validate: bool = True) -> None:
        """
        Main control loop.  Runs until stop() is called or lidar closes.

        If validate=True, calls validate_lidar_startup() before motion and
        aborts if the sensor looks unhealthy.
        """
        if validate:
            healthy = await validate_lidar_startup(lidar)
            if not healthy:
                print("[NavLogic] aborting — LiDAR validation failed")
                return

        self._running = True
        async for scan in lidar.scan_stream():
            if not self._running:
                break

            front = front_zone_points(scan)
            nearest = nearest_range(front)

            if nearest <= STOP_DISTANCE:
                await self._canbus.stop()
            elif nearest <= LOOK_AHEAD_DIST:
                ratio = (nearest - STOP_DISTANCE) / (LOOK_AHEAD_DIST - STOP_DISTANCE)
                await self._canbus.send_twist(MAX_LINEAR * ratio, 0.0)
            else:
                await self._canbus.send_twist(MAX_LINEAR, 0.0)

    async def stop(self) -> None:
        """Signal the run loop to exit and send a final stop command."""
        self._running = False
        await self._canbus.stop()
