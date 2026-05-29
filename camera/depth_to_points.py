#!/usr/bin/env python3
"""
depth_to_points.py — Convert an OAK-D depth image to a 3D obstacle cloud
in the LiDAR sensor-frame convention (X=right, Y=forward, Z sensor-relative,
where z + LIDAR_MOUNT_HEIGHT = ground-relative height).

This fills the LiDAR self-filter blind zone (< 1.5 m) with obstacle-relevant
points from side-mounted OAK-D cameras.  The output array is drop-in
compatible with SafetyMonitor.check() — merge it directly with the LiDAR
cloud before the safety call.

Coordinate transforms
---------------------
Camera optical frame (OpenCV): x=right, y=down, z=into-scene
Robot base_link frame:         X=right, Y=forward, Z=up (absolute height)
LiDAR sensor convention:       same XY, but  z_sensor = z_robot − LIDAR_MOUNT_HEIGHT
                                so that z_sensor + LIDAR_MOUNT_HEIGHT = height above ground

Camera mount geometry (all units: metres / radians, in robot base_link frame):
  cam_x      : signed lateral offset  (negative = left side)
  cam_y_fwd  : forward offset along robot Y axis
  cam_z      : height above ground
  cam_yaw    : signed rotation around robot Z (CCW positive).
               Left camera looking inward → negative yaw.
               Right camera looking inward → positive yaw.
  cam_pitch  : downward pitch around robot X.  Positive = nose down.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT

# OAK-D stereo intrinsics at 640×400 resolution.
# Consistent with oak_driver.py FOCAL_PX = 452.
_FX = 452.0
_FY = 452.0
_CX = 320.0
_CY = 200.0

# Depth validity window (metres in robot Y / forward direction).
_DEPTH_MIN_M = 0.10
_DEPTH_MAX_M = 4.5

# Rotation matrix: camera optical frame → robot body frame (zero yaw/pitch).
#   cam_x_optical  → robot +X (right)
#   cam_y_optical  → robot −Z (down → −up)
#   cam_z_optical  → robot +Y (forward)
_R_OPT2BODY = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  0.0,  1.0],
    [0.0, -1.0,  0.0],
], dtype=np.float64)


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rx(pitch: float) -> np.ndarray:
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


class DepthToPoints:
    """Convert one OAK-D depth frame to a 3D obstacle cloud compatible with SafetyMonitor.

    Parameters
    ----------
    cam_x : float
        Signed lateral offset of the camera from robot centreline (m).
        Negative = left camera, positive = right camera.
    cam_y_fwd : float
        Forward offset along robot Y axis (m).  Positive = in front of axle.
    cam_z : float
        Camera height above ground (m).  Typical OAK-D mount: ~0.55 m.
    cam_yaw : float
        Signed yaw rotation (rad, CCW positive).
        Left camera pointing inward: negative (e.g. -0.61 rad ≈ -35°).
        Right camera pointing inward: positive (e.g. +0.61 rad ≈ +35°).
    cam_pitch : float
        Downward pitch (rad, positive = nose down).  Typical: 0.0–0.17 rad.
    subsample : int
        Take every Nth row and column.  subsample=4 → 160×100 = 16 000 px.
    y_max : float
        Discard points beyond this forward distance (m).  Keep ≤ 3.0 m
        to focus on the blind zone ahead rather than duplicating LiDAR.
    """

    def __init__(
        self,
        cam_x: float,
        cam_y_fwd: float = 0.30,
        cam_z: float = 0.55,
        cam_yaw: float = 0.0,
        cam_pitch: float = 0.0,
        subsample: int = 4,
        y_max: float = 3.0,
    ) -> None:
        self.cam_x = cam_x
        self.cam_y_fwd = cam_y_fwd
        self.cam_z = cam_z
        self.subsample = max(1, subsample)
        self.y_max = y_max

        # Full rotation: Rz(yaw) @ Rx(-pitch) @ R_opt2body
        # Rx(-pitch) because "positive pitch = camera tilts nose-down" means
        # the look direction (robot +Y after R_opt2body) rotates toward −Z,
        # which is a negative rotation around robot X.
        self._R: np.ndarray = _rz(cam_yaw) @ _rx(-cam_pitch) @ _R_OPT2BODY

        # Camera origin in robot base_link frame.
        self._t = np.array([cam_x, cam_y_fwd, cam_z], dtype=np.float64)

        # Z offset to convert robot-frame height to LiDAR sensor convention.
        # After transform: z_robot = true height above ground.
        # Safety check expects z such that z + LIDAR_MOUNT_HEIGHT = true height.
        self._z_offset = -LIDAR_MOUNT_HEIGHT  # z_sensor = z_robot − LIDAR_MOUNT_HEIGHT

        # Pixel grid — recomputed lazily when frame size changes.
        self._grid_hw: Optional[tuple] = None
        self._dirs: Optional[np.ndarray] = None  # (H, W, 3) unit direction vectors

    # ------------------------------------------------------------------
    def _build_grid(self, h: int, w: int) -> None:
        if self._grid_hw == (h, w):
            return
        us = np.arange(w, dtype=np.float64)
        vs = np.arange(h, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)   # (H, W)
        # Unit direction in camera optical frame (z=1 forward plane):
        #   x_cam = (u - cx) / fx,  y_cam = (v - cy) / fy,  z_cam = 1.0
        x_dir = (uu - _CX) / _FX
        y_dir = (vv - _CY) / _FY
        z_dir = np.ones_like(x_dir)
        self._dirs = np.stack([x_dir, y_dir, z_dir], axis=-1)  # (H, W, 3)
        self._grid_hw = (h, w)

    # ------------------------------------------------------------------
    def project(self, depth_mm: Optional[np.ndarray]) -> np.ndarray:
        """
        Project a depth image to a 3D obstacle cloud in LiDAR sensor convention.

        Parameters
        ----------
        depth_mm : np.ndarray or None
            HxW uint16 depth image in millimetres (from OakDriver).

        Returns
        -------
        pts : np.ndarray
            Nx3 float32 (x, y, z) in LiDAR sensor frame convention, where
            z + LIDAR_MOUNT_HEIGHT = height above ground.
            Empty array if depth_mm is None/empty.
        """
        if depth_mm is None or depth_mm.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        h, w = int(depth_mm.shape[0]), int(depth_mm.shape[1])
        self._build_grid(h, w)

        step = self.subsample
        depth_sub = depth_mm[::step, ::step].astype(np.float64)  # (Hs, Ws)
        dirs_sub = self._dirs[::step, ::step]                      # (Hs, Ws, 3)

        depth_m = depth_sub * 1e-3  # mm → m

        # Valid mask: depth within sensor range.
        valid = (depth_m >= _DEPTH_MIN_M) & (depth_m <= _DEPTH_MAX_M)

        if not valid.any():
            return np.empty((0, 3), dtype=np.float32)

        # 3D points in camera optical frame: p_cam = dir * depth
        pts_cam = dirs_sub * depth_m[..., np.newaxis]   # (Hs, Ws, 3)
        pts_cam_flat = pts_cam.reshape(-1, 3)[valid.ravel()]

        # Transform to robot base_link frame: p_robot = R @ p_cam + t
        pts_robot = pts_cam_flat @ self._R.T + self._t   # (N, 3)

        # Apply forward-distance filter.
        y_fwd = pts_robot[:, 1]
        fwd_valid = (y_fwd >= 0.05) & (y_fwd <= self.y_max)

        # Apply height filter: discard ground and sky.
        z_robot = pts_robot[:, 2]
        h_above_ground = z_robot
        height_valid = (h_above_ground >= 0.05) & (h_above_ground <= 2.5)

        keep = fwd_valid & height_valid
        if not keep.any():
            return np.empty((0, 3), dtype=np.float32)

        pts_robot = pts_robot[keep]

        # Convert to LiDAR sensor convention: z_sensor = z_robot − LIDAR_MOUNT_HEIGHT
        # so that z_sensor + LIDAR_MOUNT_HEIGHT = ground-relative height (as SafetyMonitor expects).
        out = pts_robot.astype(np.float32)
        out[:, 2] += self._z_offset   # subtract LIDAR_MOUNT_HEIGHT

        return out
