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


def downsample(pts: np.ndarray, n: int = 400) -> np.ndarray:
    """Uniform random downsample to at most n points."""
    if len(pts) <= n:
        return pts
    idx = np.random.choice(len(pts), n, replace=False)
    return pts[idx]


def icp_2d(
    source_local: np.ndarray,
    target_world: np.ndarray,
    init_pose: Pose2D,
    max_iter: int = 25,
    tol: float = 1e-4,
    max_correspondence_dist: float = 0.8,
) -> Tuple[Pose2D, float]:
    """
    Point-to-point 2D ICP.

    Finds the Pose2D that best aligns source_local (sensor frame)
    with target_world (world frame) by iterating SVD-based registration.

    Returns (refined_pose, mean_error_metres).
    On failure (too few correspondences) returns (init_pose, inf).
    """
    if len(source_local) < 10 or len(target_world) < 10:
        return init_pose, float("inf")

    # Initialise running transform from init_pose
    R = _R(init_pose.theta)
    t = np.array([init_pose.x, init_pose.y], dtype=np.float64)
    src = source_local @ R.T + t   # source in world frame (initial estimate)

    prev_err = float("inf")

    for _ in range(max_iter):
        # --- nearest-neighbour correspondences (brute force O(NM)) ---
        diff = src[:, None, :] - target_world[None, :, :]   # N × M × 2
        sq = np.einsum("ijk,ijk->ij", diff, diff)            # N × M
        nn_idx = np.argmin(sq, axis=1)                       # N
        nn_d = np.sqrt(sq[np.arange(len(src)), nn_idx])

        mask = nn_d < max_correspondence_dist
        if mask.sum() < 6:
            break

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
        # Ensure proper rotation (det = +1, not a reflection)
        if np.linalg.det(dR) < 0:
            Vt[-1] *= -1
            dR = Vt.T @ U.T
        dt = mu_t - dR @ mu_s

        # Apply delta transform
        src = src @ dR.T + dt
        t = dR @ t + dt
        R = dR @ R   # compose: both are standard rotation matrices

    theta = float(math.atan2(R[1, 0], R[0, 0]))
    return Pose2D(float(t[0]), float(t[1]), theta), prev_err
