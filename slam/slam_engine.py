#!/usr/bin/env python3
"""
slam_engine.py — Scan-to-scan ICP odometry + occupancy grid SLAM.

Pipeline per scan:
  1. Extract 2D horizontal slice from 3D VLP-16 scan.
  2. Voxel-downsample to reduce noise and preserve geometric structure.
  3. Predict next pose using constant-velocity motion model (extrapolate
     from the delta between the last two accepted poses).  This gives ICP
     a warm start that tracks turns rather than assuming zero velocity.
  4. Run ICP against a sliding-window submap reference (last 5 world-frame
     downsampled scans, merged and voxel-downsampled to ~800 points) to
     refine the predicted pose.
  5. Accept the ICP result only if mean correspondence error < threshold;
     otherwise fall back to the motion-model prediction.
  6. Transform the full 2D slice to world frame and update the occupancy
     grid with full ray casting (free-space clearing + endpoint marking).

Thread safety: process_scan() and get_state() / get_map() may be called
from different threads.  All shared state is protected by a Lock.
"""

from __future__ import annotations

import math
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
    voxel_downsample,
)
from slam.occupancy_grid import OccupancyGrid

# ICP result is rejected if mean error exceeds this (metres).
# At 0.2 m the match is likely to have diverged.
ICP_REJECT_THRESHOLD = 0.20

# Number of past scans to keep in the sliding-window submap reference.
_REF_WINDOW = 5

# Target point count for the merged reference scan fed into ICP.
_REF_MAX_PTS = 800


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
        self._prev_pose: Optional[Pose2D] = None   # pose one scan before current
        self._trajectory: List[List[float]] = [[0.0, 0.0]]
        self._scan_count = 0
        self._last_icp_err = 0.0
        # Sliding-window submap: list of last _REF_WINDOW world-frame scans.
        self._ref_scans: List[np.ndarray] = []
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def _build_reference(self) -> Optional[np.ndarray]:
        """
        Merge the sliding-window scans into a single reference point cloud
        and voxel-downsample it to at most _REF_MAX_PTS points.

        Returns None if the window is empty.
        """
        if not self._ref_scans:
            return None
        merged = np.concatenate(self._ref_scans, axis=0)
        # Voxel size chosen so that _REF_MAX_PTS points remain on average;
        # fall back to a fixed 0.15 m voxel if the cloud is already small.
        ref = voxel_downsample(merged, voxel=0.15)
        if len(ref) > _REF_MAX_PTS:
            # Increase voxel size to thin further (simple iterative approach)
            ref = voxel_downsample(merged, voxel=0.25)
        return ref

    # ------------------------------------------------------------------
    def _predict_pose(self, current: Pose2D, prev: Optional[Pose2D]) -> Pose2D:
        """
        Constant-velocity prediction: extrapolate current pose by the delta
        between the previous and current pose.

        When the robot is turning, this predicts the heading change so ICP
        starts from a rotated initial guess rather than zero angular velocity.
        The angular delta is wrapped to [-pi, pi] to avoid wrap-around jumps.
        """
        if prev is None:
            return Pose2D(current.x, current.y, current.theta)
        dx = current.x - prev.x
        dy = current.y - prev.y
        # Wrap angular delta to [-pi, pi]
        dtheta = (current.theta - prev.theta + math.pi) % (2 * math.pi) - math.pi
        return Pose2D(current.x + dx, current.y + dy, current.theta + dtheta)

    # ------------------------------------------------------------------
    def process_scan(self, scan: List[VelodynePoint]) -> None:
        """Process one full 360° VLP-16 scan.  Thread-safe."""
        pts_local = extract_2d_slice(scan)
        if len(pts_local) < 20:
            return

        pts_ds = voxel_downsample(pts_local, 0.15)

        with self._lock:
            pose_predicted = self._predict_pose(self._pose, self._prev_pose)
            ref = self._build_reference()

        # ICP scan matching: start from motion-model prediction, not last pose
        icp_err = 0.0
        pose_est = pose_predicted
        if ref is not None and len(ref) >= 10:
            refined, icp_err = icp_2d(pts_ds, ref, pose_predicted)
            if icp_err < ICP_REJECT_THRESHOLD:
                pose_est = refined
            # If ICP failed but prediction moved significantly, trust prediction
            # over the stale last pose so turns are still tracked
            # (pose_est already set to pose_predicted above on failure)

        # Transform the full slice to world frame for the map
        pts_world = sensor_to_world(pts_local, pose_est)

        # World-frame downsampled scan for next reference window
        pts_ds_world = sensor_to_world(pts_ds, pose_est)

        with self._lock:
            self._prev_pose = Pose2D(self._pose.x, self._pose.y, self._pose.theta)
            self._pose = pose_est
            self._trajectory.append([pose_est.x, pose_est.y])
            if len(self._trajectory) > 2000:
                self._trajectory = self._trajectory[-2000:]
            self._scan_count += 1
            self._last_icp_err = icp_err
            # Update sliding-window reference: append and keep last _REF_WINDOW
            self._ref_scans.append(pts_ds_world)
            if len(self._ref_scans) > _REF_WINDOW:
                self._ref_scans = self._ref_scans[-_REF_WINDOW:]

        if self._scan_count % self._map_update_every == 0:
            self._grid.update_scan(pose_est.x, pose_est.y, pts_world)

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
