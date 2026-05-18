#!/usr/bin/env python3
"""
occupancy_grid.py — Log-odds 2D occupancy grid with ray-cast free-space clearing.

Each LiDAR scan updates the grid via Bresenham-style ray casting:
  - All grid cells along the ray from robot to obstacle endpoint are
    decremented by _L_FREE (marked more free).
  - The endpoint cell itself is incremented by _L_OCC (marked occupied).

This prevents ghost obstacles from accumulating indefinitely, which was
the primary cause of noisy/useless maps in open agricultural fields.

Grid origin is at the centre of the array so the robot can travel in
any direction without hitting an edge (for the default 150×150 m size).
"""

from __future__ import annotations

import numpy as np


class OccupancyGrid:
    def __init__(self, size_m: float = 150.0, resolution: float = 0.10) -> None:
        self.res = float(resolution)
        self.n = int(size_m / resolution)
        self.origin = self.n // 2          # grid index for world (0, 0)
        self._log_odds = np.zeros((self.n, self.n), dtype=np.float32)
        self._L_OCC = 0.85                 # log-odds increment per hit
        self._L_FREE = 0.40               # log-odds decrement per free cell
        self._L_MIN = -8.0                # clamp lower bound (definitely free)
        self._L_MAX = 12.0                # clamp upper bound (definitely occupied)

    # ------------------------------------------------------------------
    def update_scan(
        self, robot_x: float, robot_y: float, pts_world: np.ndarray
    ) -> None:
        """
        Update the occupancy grid from a full 360° scan using ray casting.

        For each of 360 angular sectors (1°/sector), keeps only the closest
        return, then traces a ray from robot position to that endpoint.
        Free cells along the ray are decremented; the endpoint cell is
        incremented.  All grid updates are done with a single bulk
        np.add.at call for efficiency.

        Parameters
        ----------
        robot_x, robot_y : float
            Robot position in world frame (metres).
        pts_world : np.ndarray
            Nx2 float array of obstacle points in world frame (x, y).
        """
        if len(pts_world) == 0:
            return

        # Robot position in grid coordinates (float for linspace)
        rx_g = robot_x / self.res + self.origin
        ry_g = robot_y / self.res + self.origin

        # Vector from robot to each point
        dx = pts_world[:, 0] - robot_x
        dy = pts_world[:, 1] - robot_y

        # Bin into 360 angle sectors (0..359 degrees)
        angles_deg = (np.degrees(np.arctan2(dy, dx)) % 360.0).astype(np.int32)
        # For each sector, keep the index of the closest return
        ranges_sq = dx * dx + dy * dy
        sector_closest = {}  # sector -> index into pts_world
        for i in range(len(pts_world)):
            s = int(angles_deg[i])
            if s not in sector_closest or ranges_sq[i] < ranges_sq[sector_closest[s]]:
                sector_closest[s] = i

        # Accumulate free-cell and occupied-cell indices
        free_rows = []
        free_cols = []
        occ_rows = []
        occ_cols = []

        for idx in sector_closest.values():
            px_g = pts_world[idx, 0] / self.res + self.origin
            py_g = pts_world[idx, 1] / self.res + self.origin

            # Number of steps along the ray (based on grid-cell distance)
            n_steps = max(int(np.hypot(px_g - rx_g, py_g - ry_g)), 1)

            # Trace ray from robot to endpoint (inclusive of endpoint)
            xs = np.linspace(rx_g, px_g, n_steps + 1)
            ys = np.linspace(ry_g, py_g, n_steps + 1)

            # Convert to integer grid indices
            xi = np.round(xs).astype(np.int32)
            yi = np.round(ys).astype(np.int32)

            # In-bounds mask
            in_b = (xi >= 0) & (xi < self.n) & (yi >= 0) & (yi < self.n)

            if n_steps > 0:
                # All cells except the last are free-space
                free_mask = in_b[:-1]
                free_rows.append(yi[:-1][free_mask])
                free_cols.append(xi[:-1][free_mask])

            # Last cell is the occupied endpoint
            if in_b[-1]:
                occ_rows.append(yi[-1:])
                occ_cols.append(xi[-1:])

        # Bulk update: free cells decremented, occupied cells incremented
        if free_rows:
            fr = np.concatenate(free_rows)
            fc = np.concatenate(free_cols)
            np.add.at(self._log_odds, (fr, fc), -self._L_FREE)

        if occ_rows:
            or_ = np.concatenate(occ_rows)
            oc = np.concatenate(occ_cols)
            np.add.at(self._log_odds, (or_, oc), self._L_OCC)

        # Clamp to prevent saturation
        np.clip(self._log_odds, self._L_MIN, self._L_MAX, out=self._log_odds)

    # ------------------------------------------------------------------
    def mark_occupied(self, world_pts: np.ndarray) -> None:
        """
        Legacy interface: update grid from Nx2 world-frame obstacle points
        (endpoint-only, no free-space clearing).

        Kept for backward compatibility with map_io.py.
        Vectorised with numpy — handles a full 30k-point scan in <1 ms.
        """
        if len(world_pts) == 0:
            return
        gx = (np.round(world_pts[:, 0] / self.res).astype(int) + self.origin)
        gy = (np.round(world_pts[:, 1] / self.res).astype(int) + self.origin)
        in_bounds = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        gx, gy = gx[in_bounds], gy[in_bounds]
        np.add.at(self._log_odds, (gy, gx), self._L_OCC)
        np.clip(self._log_odds, self._L_MIN, self._L_MAX, out=self._log_odds)

    # ------------------------------------------------------------------
    def get_occupied_world(self, threshold: float = 0.5) -> np.ndarray:
        """Return Nx2 float32 array of occupied cell world coordinates."""
        gy, gx = np.where(self._log_odds > threshold)
        if len(gx) == 0:
            return np.zeros((0, 2), dtype=np.float32)
        wx = (gx.astype(np.float32) - self.origin) * self.res
        wy = (gy.astype(np.float32) - self.origin) * self.res
        return np.column_stack([wx, wy])

    @property
    def cell_count(self) -> int:
        return int(np.sum(self._log_odds > 0.5))
