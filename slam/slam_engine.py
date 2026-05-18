#!/usr/bin/env python3
"""
slam_engine.py — Scan-to-scan ICP odometry + occupancy grid SLAM.

Pipeline per scan:
  1. Extract 2D horizontal slice from 3D VLP-16 scan.
  2. Downsample to a fixed point count.
  3. Run ICP against the previous reference scan (in world frame)
     to estimate the robot's new pose.
  4. Accept the ICP result only if mean correspondence error < threshold.
  5. Transform the full 2D slice to world frame and update the occupancy grid.

Thread safety: process_scan() and get_state() / get_map() may be called
from different threads.  All shared state is protected by a Lock.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from lidar.lidar_driver import VelodynePoint
from slam.scan_matcher import (
    Pose2D,
    downsample,
    extract_2d_slice,
    icp_2d,
    sensor_to_world,
)
from slam.occupancy_grid import OccupancyGrid

# ICP result is rejected if mean error exceeds this (metres).
# At 0.4 m the match is likely to have diverged.
ICP_REJECT_THRESHOLD = 0.40


@dataclass
class SlamState:
    pose: Pose2D = field(default_factory=Pose2D)
    trajectory: List[List[float]] = field(default_factory=list)
    scan_count: int = 0
    last_icp_error: float = 0.0


class SlamEngine:
    """
    Maintains pose, trajectory, and occupancy grid from a stream of VLP-16 scans.

    Usage::

        engine = SlamEngine()
        async for scan in lidar.scan_stream():
            engine.process_scan(scan)
            state = engine.get_state()
            map_pts = engine.get_map()
    """

    def __init__(
        self,
        grid_size_m: float = 150.0,
        grid_resolution: float = 0.10,
        icp_points: int = 400,
        map_update_every: int = 1,
    ) -> None:
        self._grid = OccupancyGrid(grid_size_m, grid_resolution)
        self._icp_points = icp_points
        self._map_update_every = map_update_every

        self._pose = Pose2D()
        self._trajectory: List[List[float]] = [[0.0, 0.0]]
        self._scan_count = 0
        self._last_icp_err = 0.0
        self._ref_scan_world: Optional[np.ndarray] = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def process_scan(self, scan: List[VelodynePoint]) -> None:
        """Process one full 360° VLP-16 scan.  Thread-safe."""
        pts_local = extract_2d_slice(scan)
        if len(pts_local) < 20:
            return

        pts_ds = downsample(pts_local, self._icp_points)

        with self._lock:
            pose_est = Pose2D(self._pose.x, self._pose.y, self._pose.theta)
            ref = self._ref_scan_world

        # ICP scan matching
        icp_err = 0.0
        if ref is not None and len(ref) >= 10:
            refined, icp_err = icp_2d(pts_ds, ref, pose_est)
            if icp_err < ICP_REJECT_THRESHOLD:
                pose_est = refined

        # Transform the full slice to world frame for the map
        pts_world = sensor_to_world(pts_local, pose_est)

        with self._lock:
            self._pose = pose_est
            self._trajectory.append([pose_est.x, pose_est.y])
            if len(self._trajectory) > 2000:
                self._trajectory = self._trajectory[-2000:]
            self._scan_count += 1
            self._last_icp_err = icp_err
            # Use downsampled world-frame scan as next reference
            self._ref_scan_world = sensor_to_world(pts_ds, pose_est)

        if self._scan_count % self._map_update_every == 0:
            self._grid.mark_occupied(pts_world)

    # ------------------------------------------------------------------
    def get_state(self) -> SlamState:
        """Return a snapshot of the current SLAM state (thread-safe)."""
        with self._lock:
            return SlamState(
                pose=Pose2D(self._pose.x, self._pose.y, self._pose.theta),
                trajectory=list(self._trajectory),
                scan_count=self._scan_count,
                last_icp_error=self._last_icp_err,
            )

    def get_map(self) -> np.ndarray:
        """Return Nx2 world-frame coordinates of all occupied map cells."""
        return self._grid.get_occupied_world()

    @property
    def cell_count(self) -> int:
        return self._grid.cell_count
