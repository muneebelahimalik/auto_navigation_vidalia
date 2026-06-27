#!/usr/bin/env python3
"""
slam_engine.py — Scan-to-scan ICP odometry + occupancy grid SLAM.

Pipeline per scan:
  1. Extract 2D horizontal slice from 3D VLP-16 scan.
  2. Voxel-downsample + statistical outlier removal.
  3. Constant-velocity motion prediction for the ICP warm start.
  4. ICP against a sliding-window submap reference (last 5 scans).
  5. Sanity-check: reject any ICP result that moves the robot faster
     than physically possible (velocity cap + rotation cap).
  6. Periodic scan-to-map loop closure: every _LC_EVERY_N_SCANS scans,
     ICP the current scan against occupied map cells within _LC_RADIUS_M
     of the current position.  A tight error threshold (_LC_ACCEPT_ERR)
     guards against false closures.  When accepted, the pose snaps to the
     map-consistent estimate, correcting accumulated drift.
  7. Update the occupancy grid with ray-cast free-space clearing.

Thread safety: process_scan() and get_state() / get_map() may be called
from different threads.  All shared state is protected by a Lock.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from lidar.lidar_driver import VelodynePoint
from slam.scan_matcher import (
    Pose2D,
    SLICE_MAX_GND,
    SLICE_MIN_GND,
    correct_scan,
    deskew_scan,
    downsample,
    extract_2d_slice,
    icp_2d,
    remove_outliers,
    robot_xyz_to_world,
    scan_to_xyz,
    sensor_to_world,
    voxel_downsample,
    voxel_downsample_3d,
)
from slam.coverage_map import CoverageGrid
from slam.occupancy_grid import OccupancyGrid
from slam.voxel_map import VoxelMap

# ---------- LiDAR mount correction (field-calibrated, matches row-follow) -----
# The VLP-16 is mounted yawed ~66° and pitched ~21.5° nose-down.  SLAM must
# correct the raw cloud into the robot frame before slicing — see
# scan_matcher.correct_scan and CLAUDE.md (LiDAR yaw/tilt calibration).
_DEFAULT_YAW_DEG = 66.0
_DEFAULT_TILT_DEG = 21.5

# ---------- ICP / submap parameters -----------------------------------------
# ICP result is rejected if mean error exceeds this (metres).
# Keep tight: in sparse open environments, bad ICP (0.10–0.18 m error) is
# WORSE than pure wheel-odometry prediction (~0.01 m/scan drift).  Only
# accept ICP when it's genuinely confident; fall back to odom otherwise.
ICP_REJECT_THRESHOLD = 0.08

# Number of past scans to keep in the sliding-window submap reference.
_REF_WINDOW = 8

# Target point count for the merged reference scan fed into ICP.
_REF_MAX_PTS = 1200

# ---------- Physical motion caps ---------------------------------------------
# Robot max speed 0.8 m/s; LiDAR worst-case ~1.5 Hz → 0.53 m/scan.
# Cap is generous to handle LiDAR rate variation without clipping real motion.
_MAX_TRANS_PER_SCAN = 0.70       # metres
_MAX_ROT_PER_SCAN   = math.radians(50)  # radians (~100°/s at 2 Hz)

# ---------- Loop closure parameters -----------------------------------------
_LC_EVERY_N_SCANS = 8    # attempt loop closure every N scans
_LC_MIN_SCANS     = 25   # don't attempt until the map has enough structure
_LC_RADIUS_M      = 15.0 # radius around current pose to sample map cells
_LC_ACCEPT_ERR    = 0.18 # accept closure only if ICP error is below this
                          # (0.18 m is realistic for sparse agricultural fields)


@dataclass
class SlamState:
    pose: Pose2D = field(default_factory=Pose2D)
    trajectory: List[List[float]] = field(default_factory=list)
    scan_count: int = 0
    last_icp_error: float = 0.0
    loop_closures: int = 0
    odom_scans: int = 0   # scans where wheel odometry was used as warm start
    map3d_voxels: int = 0  # occupied voxels in the 3-D map (0 if disabled)
    covered_area_m2: float = 0.0   # serviced ground area (0 if coverage off)
    path_length_m: float = 0.0     # total driven path length
    coverage_redundancy: float = 0.0  # swept ÷ unique covered area (1 = no overlap)


class SlamEngine:
    """
    Maintains pose, trajectory, and occupancy grid from a stream of VLP-16 scans.

    Usage::

        engine = SlamEngine()
        async for scan in lidar.scan_stream():
            engine.process_scan(scan)
            state = engine.get_state()
    """

    def __init__(
        self,
        grid_size_m: float = 150.0,
        grid_resolution: float = 0.10,
        icp_points: int = 400,
        map_update_every: int = 1,
        yaw_deg: float = _DEFAULT_YAW_DEG,
        tilt_deg: float = _DEFAULT_TILT_DEG,
        slice_min: float = SLICE_MIN_GND,
        slice_max: float = SLICE_MAX_GND,
        build_3d: bool = False,
        voxel_3d: float = 0.15,
        map3d_z_min: float = -1.0,
        map3d_z_max: float = 5.0,
        track_coverage: bool = True,
        swath_m: float = 1.92,
    ) -> None:
        self._grid = OccupancyGrid(grid_size_m, grid_resolution)
        self._icp_points = icp_points
        self._map_update_every = map_update_every
        self._yaw_rad = math.radians(yaw_deg)
        self._tilt_rad = math.radians(tilt_deg)
        self._slice_min = slice_min
        self._slice_max = slice_max

        # Optional 3-D voxel map, registered with the same 2-D pose used for the
        # occupancy grid (2.5-D mapping — see slam/voxel_map.py).
        self._voxel_3d = voxel_3d
        self._map3d: Optional[VoxelMap] = (
            VoxelMap(voxel=voxel_3d, z_min=map3d_z_min, z_max=map3d_z_max)
            if build_3d else None
        )

        # Field coverage accounting (the serviced-swath map) — mapping only.
        self._coverage: Optional[CoverageGrid] = (
            CoverageGrid(grid_size_m, grid_resolution, swath_m)
            if track_coverage else None
        )

        self._pose = Pose2D()
        self._prev_pose: Optional[Pose2D] = None
        self._trajectory: List[List[float]] = [[0.0, 0.0]]
        # Full (uncapped) pose log for the coverage PNG + trajectory CSV;
        # the rendered self._trajectory is capped, this is not (≈0.9 MB/hour).
        self._full_traj: List[List[float]] = [[0.0, 0.0, 0.0]]
        self._scan_count = 0
        self._last_icp_err = 0.0
        self._loop_closures = 0
        self._odom_scans = 0
        self._ref_scans: List[np.ndarray] = []
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    def _build_reference(self) -> Optional[np.ndarray]:
        """Merge the sliding-window scans, voxel-downsample to _REF_MAX_PTS."""
        if not self._ref_scans:
            return None
        merged = np.concatenate(self._ref_scans, axis=0)
        ref = voxel_downsample(merged, voxel=0.15)
        if len(ref) > _REF_MAX_PTS:
            ref = voxel_downsample(merged, voxel=0.25)
        return ref

    # ------------------------------------------------------------------
    def _predict_pose(self, current: Pose2D, prev: Optional[Pose2D]) -> Pose2D:
        """Constant-velocity prediction from the last two accepted poses."""
        if prev is None:
            return Pose2D(current.x, current.y, current.theta)
        dx = current.x - prev.x
        dy = current.y - prev.y
        dtheta = (current.theta - prev.theta + math.pi) % (2 * math.pi) - math.pi
        return Pose2D(current.x + dx, current.y + dy, current.theta + dtheta)

    # ------------------------------------------------------------------
    def _predict_pose_odom(
        self, current: Pose2D, ds: float, dtheta: float
    ) -> Pose2D:
        """
        Wheel-odometry prediction: apply arc (ds, dtheta) to current pose.

        Uses the midpoint heading so the arc is correctly curved rather
        than approximated as a straight-line step.

        ds     : forward arc distance in metres
        dtheta : heading change in radians (CCW positive)
        """
        mid_theta = current.theta + dtheta / 2.0
        return Pose2D(
            current.x + ds * math.cos(mid_theta),
            current.y + ds * math.sin(mid_theta),
            current.theta + dtheta,
        )

    # ------------------------------------------------------------------
    def _apply_motion_caps(self, new_pose: Pose2D, ref_pose: Pose2D) -> Pose2D:
        """
        Clamp the pose change from ref_pose to new_pose to physically
        plausible limits.  Prevents runaway ICP drift on featureless
        straight sections where correspondence quality is low.
        """
        dx = new_pose.x - ref_pose.x
        dy = new_pose.y - ref_pose.y
        dist = math.sqrt(dx * dx + dy * dy)
        dtheta = (new_pose.theta - ref_pose.theta + math.pi) % (2 * math.pi) - math.pi

        if dist > _MAX_TRANS_PER_SCAN:
            scale = _MAX_TRANS_PER_SCAN / dist
            dx *= scale
            dy *= scale

        if abs(dtheta) > _MAX_ROT_PER_SCAN:
            dtheta = math.copysign(_MAX_ROT_PER_SCAN, dtheta)

        return Pose2D(ref_pose.x + dx, ref_pose.y + dy, ref_pose.theta + dtheta)

    # ------------------------------------------------------------------
    def _try_loop_closure(
        self, pts_ds: np.ndarray, pose: Pose2D
    ) -> Optional[Pose2D]:
        """
        Scan-to-map ICP loop closure.

        Samples occupied map cells within _LC_RADIUS_M of the current
        pose and runs ICP against them.  The map contains all previously
        accepted scans, so when the robot revisits a known area the
        ICP will snap the pose to the map-consistent estimate, correcting
        accumulated drift without requiring an explicit keyframe database.

        Returns the corrected pose if ICP error < _LC_ACCEPT_ERR, else None.
        """
        all_occ = self._grid.get_occupied_world()  # Nx2 world-frame
        if len(all_occ) < 30:
            return None

        dx = all_occ[:, 0] - pose.x
        dy = all_occ[:, 1] - pose.y
        nearby = all_occ[(dx * dx + dy * dy) < _LC_RADIUS_M ** 2]
        if len(nearby) < 20:
            return None

        map_ref = voxel_downsample(nearby.astype(np.float64), 0.20)

        refined, err = icp_2d(
            pts_ds, map_ref, pose,
            max_correspondence_dist=0.40,
            trim_ratio=0.75,
        )
        if err < _LC_ACCEPT_ERR:
            return refined
        return None

    # ------------------------------------------------------------------
    def process_scan(
        self,
        scan: List[VelodynePoint],
        odom_delta: Optional[Tuple[float, float]] = None,
        fwd_speed: float = 0.0,
    ) -> None:
        """
        Process one full 360° VLP-16 scan.  Thread-safe.

        Parameters
        ----------
        scan       : raw VLP-16 point list from LidarDriver
        odom_delta : optional ``(ds, dtheta)`` from WheelOdometry.get_delta_and_reset().
                     When provided, wheel-odometry is used for the ICP warm
                     start instead of the constant-velocity fallback.
        fwd_speed  : forward speed in m/s from the most recent encoder reading.
                     Used to deskew the point cloud for motion distortion caused
                     by the robot moving during the 100 ms VLP-16 scan window.
        """
        pts_local = extract_2d_slice(
            scan,
            min_gnd=self._slice_min,
            max_gnd=self._slice_max,
            yaw_rad=self._yaw_rad,
            tilt_rad=self._tilt_rad,
        )
        if len(pts_local) < 20:
            return

        pts_local = deskew_scan(pts_local, fwd_speed)
        pts_ds = remove_outliers(voxel_downsample(pts_local, 0.15))

        # Full corrected 3-D cloud (robot frame) for the optional 3-D map.
        # Built once here; registered with the final pose at the end of the scan.
        full_robot_3d = None
        if self._map3d is not None:
            full_robot_3d = voxel_downsample_3d(
                correct_scan(scan_to_xyz(scan), self._yaw_rad, self._tilt_rad),
                voxel=max(0.05, 0.5 * self._voxel_3d),
            )

        with self._lock:
            if odom_delta is not None:
                ds, dtheta = odom_delta
                pose_predicted = self._predict_pose_odom(self._pose, ds, dtheta)
            else:
                pose_predicted = self._predict_pose(self._pose, self._prev_pose)
            ref = self._build_reference()
            scan_count_now = self._scan_count

        # --- 1. Submap ICP (warm-started from motion model) ---
        icp_err = 0.0
        pose_est = pose_predicted
        if ref is not None and len(ref) >= 10:
            refined, icp_err = icp_2d(pts_ds, ref, pose_predicted)
            if icp_err < ICP_REJECT_THRESHOLD:
                pose_est = refined

        # --- 2. Velocity / rotation caps ---
        with self._lock:
            last_pose = Pose2D(self._pose.x, self._pose.y, self._pose.theta)
        pose_est = self._apply_motion_caps(pose_est, last_pose)

        # --- 3. Loop closure (scan-to-map ICP, periodic) ---
        lc_accepted = False
        if scan_count_now >= _LC_MIN_SCANS and scan_count_now % _LC_EVERY_N_SCANS == 0:
            lc_pose = self._try_loop_closure(pts_ds, pose_est)
            if lc_pose is not None:
                pose_est = self._apply_motion_caps(lc_pose, last_pose)
                lc_accepted = True

        # World-frame filtered cloud for map update and reference window
        pts_ds_world = sensor_to_world(pts_ds, pose_est)

        with self._lock:
            self._prev_pose = Pose2D(self._pose.x, self._pose.y, self._pose.theta)
            self._pose = pose_est
            self._trajectory.append([pose_est.x, pose_est.y])
            if len(self._trajectory) > 2000:
                self._trajectory = self._trajectory[-2000:]
            self._full_traj.append([pose_est.x, pose_est.y, pose_est.theta])
            if self._coverage is not None:
                self._coverage.add_pose(pose_est.x, pose_est.y)
            self._scan_count += 1
            self._last_icp_err = icp_err
            if lc_accepted:
                self._loop_closures += 1
            if odom_delta is not None:
                self._odom_scans += 1
            self._ref_scans.append(pts_ds_world)
            if len(self._ref_scans) > _REF_WINDOW:
                self._ref_scans = self._ref_scans[-_REF_WINDOW:]

        if self._scan_count % self._map_update_every == 0:
            self._grid.update_scan(pose_est.x, pose_est.y, pts_ds_world)

        # 3-D map: register the full corrected cloud with the final 2-D pose.
        if self._map3d is not None and full_robot_3d is not None:
            world3d = robot_xyz_to_world(full_robot_3d, pose_est)
            with self._lock:
                self._map3d.add_points(world3d)

    # ------------------------------------------------------------------
    def get_state(self) -> SlamState:
        """Return a snapshot of the current SLAM state (thread-safe)."""
        with self._lock:
            return SlamState(
                pose=Pose2D(self._pose.x, self._pose.y, self._pose.theta),
                trajectory=list(self._trajectory),
                scan_count=self._scan_count,
                last_icp_error=self._last_icp_err,
                loop_closures=self._loop_closures,
                odom_scans=self._odom_scans,
                map3d_voxels=self._map3d.count if self._map3d is not None else 0,
                covered_area_m2=(self._coverage.covered_area_m2
                                 if self._coverage is not None else 0.0),
                path_length_m=(self._coverage.path_length_m
                               if self._coverage is not None else 0.0),
                coverage_redundancy=(self._coverage.redundancy
                                     if self._coverage is not None else 0.0),
            )

    def get_map(self) -> np.ndarray:
        """Return Nx2 world-frame coordinates of all occupied map cells."""
        return self._grid.get_occupied_world()

    def get_coverage(self) -> Optional[CoverageGrid]:
        """Return the coverage grid (None if coverage tracking is disabled)."""
        return self._coverage

    def get_full_trajectory(self) -> np.ndarray:
        """Return the uncapped Nx3 (x, y, heading) pose log for CSV / coverage."""
        with self._lock:
            return np.asarray(self._full_traj, dtype=np.float64)

    def get_3d_points(self) -> np.ndarray:
        """Return the Nx3 world-frame 3-D map points (empty if 3-D is off)."""
        if self._map3d is None:
            return np.zeros((0, 3), dtype=np.float32)
        with self._lock:
            return self._map3d.to_points()

    @property
    def cell_count(self) -> int:
        return self._grid.cell_count

    @property
    def map3d_count(self) -> int:
        return self._map3d.count if self._map3d is not None else 0
