#!/usr/bin/env python3
"""
map_io.py — Save, load, and render the occupancy grid map.

No external dependencies beyond numpy and Python stdlib.
PNG is written with the built-in zlib module — no PIL/Pillow required.
"""

from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from slam.occupancy_grid import OccupancyGrid


# ---------------------------------------------------------------------------
# PNG writer (pure stdlib + numpy)
# ---------------------------------------------------------------------------

def _png_chunk(tag: bytes, data: bytes) -> bytes:
    body = tag + data
    return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)


def _write_png(path: Path, rgb: np.ndarray) -> None:
    """Write an HxWx3 uint8 RGB array to a PNG file (no PIL required)."""
    h, w = rgb.shape[:2]
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    png = (
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", ihdr)
        + _png_chunk(b"IDAT", zlib.compress(raw, 6))
        + _png_chunk(b"IEND", b"")
    )
    path.write_bytes(png)


# ---------------------------------------------------------------------------
# Line and circle drawing on numpy RGB arrays
# ---------------------------------------------------------------------------

def _draw_line(
    img: np.ndarray,
    x0: int, y0: int,
    x1: int, y1: int,
    color: Tuple[int, int, int],
    thickness: int = 1,
) -> None:
    """Bresenham line, drawn thick by offsetting ±thickness//2 pixels."""
    h, w = img.shape[:2]
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    r = max(0, thickness // 2)
    while True:
        for dy_ in range(-r, r + 1):
            for dx_ in range(-r, r + 1):
                ny, nx = y0 + dy_, x0 + dx_
                if 0 <= ny < h and 0 <= nx < w:
                    img[ny, nx] = color
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def _draw_circle(
    img: np.ndarray,
    cx: int, cy: int,
    radius: int,
    color: Tuple[int, int, int],
) -> None:
    h, w = img.shape[:2]
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius ** 2:
                if 0 <= y < h and 0 <= x < w:
                    img[y, x] = color


def _draw_scale_bar(
    img: np.ndarray,
    pixels_per_metre: float,
    margin: int = 20,
) -> None:
    """Draw a 5 m scale bar in the bottom-left corner."""
    bar_m = 5.0
    bar_px = int(bar_m * pixels_per_metre)
    h, w = img.shape[:2]
    bx0 = margin
    bx1 = bx0 + bar_px
    by  = h - margin
    # Draw horizontal bar (3 px thick)
    for dy in range(-1, 2):
        for x in range(bx0, bx1 + 1):
            if 0 <= by + dy < h and 0 <= x < w:
                img[by + dy, x] = (50, 50, 50)
    # End ticks
    for dy in range(-5, 6):
        for bx in (bx0, bx1):
            if 0 <= by + dy < h and 0 <= bx < w:
                img[by + dy, bx] = (50, 50, 50)


# ---------------------------------------------------------------------------
# Render occupancy grid → RGB image
# ---------------------------------------------------------------------------

def render_png(
    grid: OccupancyGrid,
    trajectory: List[List[float]],
    output_path: Path,
    pad_metres: float = 3.0,
    upsample: int = 2,
) -> bool:
    """
    Render the occupancy grid as a PNG.

    Layout:
      - Light grey background (unknown space)
      - Dark cells (occupied — more hits = darker)
      - Blue line (robot trajectory)
      - Green circle (start position)
      - Red circle (end / current position)
      - Scale bar (5 m)

    Returns True if the PNG was written, False if there is nothing to render.
    """
    log_odds = grid._log_odds
    occupied_mask = log_odds > 0.5

    traj = np.array(trajectory, dtype=np.float64) if len(trajectory) >= 2 \
        else np.zeros((0, 2), dtype=np.float64)

    if not occupied_mask.any() and len(traj) == 0:
        return False

    res = grid.res
    origin = grid.origin
    pad = int(pad_metres / res)

    # Grid-index bounding box of all occupied cells + trajectory
    occ_gy, occ_gx = np.where(occupied_mask)

    all_gx_list = occ_gx.tolist()
    all_gy_list = occ_gy.tolist()

    if len(traj) > 0:
        traj_gx = (traj[:, 0] / res).astype(int) + origin
        traj_gy = (traj[:, 1] / res).astype(int) + origin
        all_gx_list += traj_gx.tolist()
        all_gy_list += traj_gy.tolist()

    if not all_gx_list:
        return False

    gx_min = max(0, min(all_gx_list) - pad)
    gx_max = min(grid.n - 1, max(all_gx_list) + pad)
    gy_min = max(0, min(all_gy_list) - pad)
    gy_max = min(grid.n - 1, max(all_gy_list) + pad)

    crop = log_odds[gy_min:gy_max + 1, gx_min:gx_max + 1]
    crop_h, crop_w = crop.shape

    # Upsample so individual 10 cm cells are clearly visible
    img_h = crop_h * upsample
    img_w = crop_w * upsample

    # Background: light grey (unknown / unmapped)
    img = np.full((img_h, img_w, 3), 230, dtype=np.uint8)

    # Occupied cells: dark grey, intensity proportional to confidence
    for gy in range(crop_h):
        for gx in range(crop_w):
            lo = crop[gy, gx]
            if lo > 0.5:
                # Map log-odds 0.5→10 to grey 180→20
                v = int(180 - (lo / grid._L_CLAMP) * 160)
                v = max(20, min(180, v))
                py0, py1 = gy * upsample, (gy + 1) * upsample
                px0, px1 = gx * upsample, (gx + 1) * upsample
                img[py0:py1, px0:px1] = (v, v, v)

    ppm = upsample / res   # pixels per metre

    # Trajectory (blue)
    if len(traj) >= 2:
        tx = ((traj[:, 0] / res).astype(int) + origin - gx_min) * upsample
        ty = ((traj[:, 1] / res).astype(int) + origin - gy_min) * upsample
        for i in range(len(tx) - 1):
            _draw_line(img, int(tx[i]), int(ty[i]), int(tx[i + 1]), int(ty[i + 1]),
                       (50, 130, 255), thickness=max(2, upsample))

        # Start (green) and end (red) markers
        r = max(5, int(0.3 * ppm))
        _draw_circle(img, int(tx[0]),  int(ty[0]),  r, (30, 200, 60))
        _draw_circle(img, int(tx[-1]), int(ty[-1]), r, (220, 50,  50))

    # Scale bar
    _draw_scale_bar(img, ppm)

    _write_png(output_path, img)
    return True


# ---------------------------------------------------------------------------
# Save / load
# ---------------------------------------------------------------------------

def save_map(
    grid: OccupancyGrid,
    trajectory: List[List[float]],
    save_dir: Path,
) -> Path:
    """
    Persist the map to <save_dir>/map.npz and render <save_dir>/map.png.
    Returns save_dir.
    """
    save_dir = Path(save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    npz_path = save_dir / "map.npz"
    np.savez_compressed(
        npz_path,
        log_odds   = grid._log_odds,
        resolution = np.float32(grid.res),
        origin     = np.int32(grid.origin),
        trajectory = np.array(trajectory, dtype=np.float32),
    )

    png_path = save_dir / "map.png"
    rendered = render_png(grid, trajectory, png_path)

    return save_dir


def load_map(save_dir: Path) -> Tuple[OccupancyGrid, np.ndarray]:
    """
    Load a previously saved map.
    Returns (OccupancyGrid, trajectory_Nx2_float32).
    """
    data = np.load(Path(save_dir) / "map.npz")
    res     = float(data["resolution"])
    origin  = int(data["origin"])
    lo      = data["log_odds"]
    traj    = data["trajectory"]

    size_m = lo.shape[0] * res
    grid = OccupancyGrid(size_m=size_m, resolution=res)
    grid._log_odds[:] = lo
    grid.origin = origin

    return grid, traj
