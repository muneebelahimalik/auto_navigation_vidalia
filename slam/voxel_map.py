#!/usr/bin/env python3
"""
voxel_map.py — Incremental 3-D voxel map for field mapping.

The 2-D SLAM engine estimates the robot pose (x, y, heading); this module uses
that pose to accumulate every full 3-D LiDAR scan into a single world-frame 3-D
point map.  A ground robot moves on the 2-D ground manifold, so a 3-DOF pose
plus the static mount-tilt correction is enough to register the 3-D geometry —
no full 6-DOF SLAM required (see CLAUDE.md, "LiDAR Field Mapping").

Storage is a voxel hash: at most one point per ``voxel``-sized cell, so the map
size grows with the field's surface area, not with the number of scans.  Keys
are packed into a single Python int per cell and held in a set for O(1) dedup;
the representative point of each occupied cell is appended to parallel lists and
materialised to an Nx3 array on demand.

z convention: ``z`` is height ABOVE THE LOCAL GROUND PLANE (metres) — the robot's
own elevation is not tracked by the 2-D SLAM, so absolute terrain elevation
change is not captured; ground reads ~0, crop ~0.05–0.30 m, obstacles taller.
"""

from __future__ import annotations

import numpy as np


class VoxelMap:
    # Per-axis voxel index is offset by _OFF and packed into 21 bits, so each
    # axis spans ±2^20 voxels (±100 km at 0.10 m) — far beyond any field.
    _BITS = 21
    _OFF = 1 << 20
    _M = 1 << 21

    def __init__(
        self,
        voxel: float = 0.15,
        z_min: float = -1.0,
        z_max: float = 5.0,
    ) -> None:
        self.voxel = float(voxel)
        self.z_min = float(z_min)
        self.z_max = float(z_max)
        self._keys: set[int] = set()
        self._xs: list[float] = []
        self._ys: list[float] = []
        self._zs: list[float] = []

    # ------------------------------------------------------------------
    def add_points(self, pts_world: np.ndarray) -> int:
        """Insert Nx3 world-frame points; return the number of NEW voxels added.

        Points outside [z_min, z_max] (sensor noise below ground / sky returns)
        are dropped.  Only the first point seen in each voxel is kept.
        """
        if pts_world is None or len(pts_world) == 0:
            return 0
        z = pts_world[:, 2]
        keep = (z >= self.z_min) & (z <= self.z_max)
        pts = pts_world[keep]
        if len(pts) == 0:
            return 0

        idx = np.floor(pts / self.voxel).astype(np.int64)
        ix = idx[:, 0] + self._OFF
        iy = idx[:, 1] + self._OFF
        iz = idx[:, 2] + self._OFF
        keys = ix + iy * self._M + iz * (self._M * self._M)

        # One representative point per distinct voxel hit this scan.
        uniq_keys, first = np.unique(keys, return_index=True)
        added = 0
        kset = self._keys
        for k, i in zip(uniq_keys.tolist(), first.tolist()):
            if k not in kset:
                kset.add(k)
                self._xs.append(float(pts[i, 0]))
                self._ys.append(float(pts[i, 1]))
                self._zs.append(float(pts[i, 2]))
                added += 1
        return added

    # ------------------------------------------------------------------
    def to_points(self) -> np.ndarray:
        """Return the Nx3 float32 array of occupied-voxel representative points."""
        if not self._xs:
            return np.zeros((0, 3), dtype=np.float32)
        return np.column_stack([self._xs, self._ys, self._zs]).astype(np.float32)

    @property
    def count(self) -> int:
        return len(self._xs)
