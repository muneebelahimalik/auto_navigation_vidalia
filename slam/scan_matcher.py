#!/usr/bin/env python3
"""
scan_matcher.py — 2D ICP scan matcher for VLP-16 horizontal slices.

Coordinate convention (shared with lidar_driver.py):
  Sensor frame: X = right, Y = forward, Z = up from spin axis
  Map frame:    same axes; robot starts at (0, 0), theta = 0
  theta:        CCW radians measured from the world +X axis
                (standard robotics convention)

  Transform sensor → world:
      world_pts = sensor_pts @ R(theta).T + [x, y]
  where R(theta) = [[cos, -sin], [sin, cos]]  (standard CCW rotation)

  Check: theta=pi/2, forward point (0, 2) → [-2, 0]  (robot facing west,
         so 2 m ahead ends up at world (-2, 0)) ✓
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from lidar.lidar_driver import VelodynePoint
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT

# Height band for the 2D slice (ground-relative metres)
SLICE_MIN_GND = 0.20    # above ground
SLICE_MAX_GND = 1.50    # above ground


@dataclass
class Pose2D:
    """2D robot pose: (x, y) metres in map frame, theta CCW radians from +X."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def _R(theta: float) -> np.ndarray:
    """Standard 2×2 CCW rotation matrix."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def sensor_to_world(pts: np.ndarray, pose: Pose2D) -> np.ndarray:
    """Transform Nx2 sensor-frame points to world frame using pose."""
    if len(pts) == 0:
        return pts
    return pts @ _R(pose.theta).T + np.array([pose.x, pose.y])


def extract_2d_slice(
    scan: List[VelodynePoint],
    min_gnd: float = SLICE_MIN_GND,
    max_gnd: float = SLICE_MAX_GND,
) -> np.ndarray:
    """
    Project a 3D scan to 2D by keeping only the horizontal band
    [min_gnd, max_gnd] metres above ground.

    Returns Nx2 float64 array of (x, y) sensor-frame coords.
    """
    pts = [
        (p.x, p.y)
        for p in scan
        if min_gnd <= (p.z + LIDAR_MOUNT_HEIGHT) <= max_gnd
    ]
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 2), dtype=np.float64)


def voxel_downsample(pts: np.ndarray, voxel: float = 0.15) -> np.ndarray:
    """Keep one point per voxel cell (grid-based, preserves structure)."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(keys[:, 0] * 100003 + keys[:, 1], return_index=True)
    return pts[idx]


def downsample(pts: np.ndarray, n: int = 400) -> np.ndarray:
    """Voxel-grid downsample (replaces random sampling to preserve structure)."""
    return voxel_downsample(pts, 0.15)


def remove_outliers(pts: np.ndarray, k: int = 8, std_mult: float = 2.0) -> np.ndarray:
    """
    Statistical outlier removal: discard points whose mean distance to their
    k nearest neighbours exceeds (global_mean + std_mult * global_std).

    Removes isolated noise returns (wind, insects, dust) without touching
    real structure.  O(N²) — fast for N ≤ 600 after voxel downsampling.
    """
    if len(pts) <= k + 1:
        return pts
    diff = pts[:, None, :] - pts[None, :, :]        # N × N × 2
    sq = np.einsum("ijk,ijk->ij", diff, diff)         # N × N
    # k nearest excluding self (self-distance = 0 is always the minimum)
    knn_sq = np.partition(sq, k + 1, axis=1)[:, 1:k + 1]
    mean_dists = np.sqrt(knn_sq).mean(axis=1)         # N
    threshold = mean_dists.mean() + std_mult * mean_dists.std()
    return pts[mean_dists < threshold]


def icp_2d(
    source_local: np.ndarray,
    target_world: np.ndarray,
    init_pose: Pose2D,
    max_iter: int = 25,
    tol: float = 1e-4,
    max_correspondence_dist: float = 0.50,
    trim_ratio: float = 0.75,
) -> Tuple[Pose2D, float]:
    """
    Trimmed point-to-point 2D ICP.

    Each iteration keeps only the closest trim_ratio fraction of valid
    correspondences (those within max_correspondence_dist).  Trimming
    makes the SVD step robust to residual outliers that survive the hard
    distance gate — particularly useful in vegetated open fields.

    Returns (refined_pose, mean_error_metres).
    On failure (too few correspondences) returns (init_pose, inf).
    """
    if len(source_local) < 10 or len(target_world) < 10:
        return init_pose, float("inf")

    R = _R(init_pose.theta)
    t = np.array([init_pose.x, init_pose.y], dtype=np.float64)
    src = source_local @ R.T + t

    prev_err = float("inf")

    for _ in range(max_iter):
        # --- nearest-neighbour correspondences ---
        diff = src[:, None, :] - target_world[None, :, :]   # N × M × 2
        sq = np.einsum("ijk,ijk->ij", diff, diff)            # N × M
        nn_idx = np.argmin(sq, axis=1)
        nn_d = np.sqrt(sq[np.arange(len(src)), nn_idx])

        # Hard gate: discard obviously wrong matches
        valid = nn_d < max_correspondence_dist
        if valid.sum() < 6:
            break

        # Trimmed: keep only the closest trim_ratio of the valid set
        cutoff = float(np.percentile(nn_d[valid], trim_ratio * 100.0))
        mask = valid & (nn_d <= cutoff)
        if mask.sum() < 6:
            mask = valid   # fall back to all valid if trimming is too aggressive

        err = float(np.mean(nn_d[mask]))
        if abs(prev_err - err) < tol:
            break
        prev_err = err

        s = src[mask]
        tgt = target_world[nn_idx[mask]]

        # --- SVD-based optimal rotation ---
        mu_s, mu_t = s.mean(0), tgt.mean(0)
        H = (s - mu_s).T @ (tgt - mu_t)
        U, _, Vt = np.linalg.svd(H)
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[-1] *= -1
            dR = Vt.T @ U.T
        dt = mu_t - dR @ mu_s

        src = src @ dR.T + dt
        t = dR @ t + dt
        R = dR @ R

    theta = float(math.atan2(R[1, 0], R[0, 0]))
    return Pose2D(float(t[0]), float(t[1]), theta), prev_err
