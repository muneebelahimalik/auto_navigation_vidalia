#!/usr/bin/env python3
"""
obstacle_filter.py — Pure-geometry LiDAR filtering and validation helpers.

No farm-ng / canbus dependency so this module can be imported by standalone
scripts (lidar_validate.py, test_lidar_*.py) without the full robot SDK.

Coordinate frame (sensor-centric, matches lidar_driver.py):
    Y  — forward (direction of travel)
    X  — right
    Z  — up from sensor spin axis

Ground-relative height
----------------------
The VLP-16 spin axis is LIDAR_MOUNT_HEIGHT metres above the ground plane.
A point at sensor-frame z has ground-relative height:
    h = z + LIDAR_MOUNT_HEIGHT

Using ground-relative height instead of a fixed sensor-frame z threshold
correctly handles objects below the LiDAR centreline (e.g. a 20-inch/0.508 m
box whose top is at sensor-frame z ≈ −0.191 m with the current mount).

Sector convention (azimuth 0°–360°, matches lidar_driver.py VLP-16 output):
    0°   → Y+  forward
    90°  → X+  right
    180° → Y−  rear
    270° → X−  left
"""

from __future__ import annotations

import math
from typing import Dict, List

from lidar.lidar_driver import LidarDriver, VelodynePoint

# ---------------------------------------------------------------------------
# Mount geometry — must match URDF and tf_static_base_to_velodyne.launch.py
# ---------------------------------------------------------------------------
LIDAR_MOUNT_HEIGHT = 0.699   # metres (z component base_link → velodyne; 27.5 in)

# ---------------------------------------------------------------------------
# Obstacle filter thresholds
# ---------------------------------------------------------------------------
# Minimum height above ground to classify a return as an obstacle.
# In sensor frame this equals z > MIN_OBSTACLE_GROUND_HEIGHT − LIDAR_MOUNT_HEIGHT
# = 0.15 − 0.699 = −0.549 m, which captures a 20-in (0.508 m) box whose top
# is at sensor-frame z ≈ −0.191 m.
MIN_OBSTACLE_GROUND_HEIGHT = 0.15   # metres above ground

# Safety corridor geometry
ROBOT_HALF_WIDTH = 0.60   # metres (Amiga body half-width + 0.135 m clearance)
LOOK_AHEAD_DIST = 2.0     # metres

# ---------------------------------------------------------------------------
# Sector definitions
# ---------------------------------------------------------------------------
_SECTOR_HALF_WIDTH = 30.0   # degrees; each sector spans ±30° around its centre


def _in_sector(azimuth: float, centre: float) -> bool:
    """True if azimuth (0–360) is within ±_SECTOR_HALF_WIDTH of centre."""
    diff = (azimuth - centre) % 360.0
    if diff > 180.0:
        diff -= 360.0
    return abs(diff) <= _SECTOR_HALF_WIDTH


def front_zone_points(
    points: List[VelodynePoint],
    look_ahead: float = LOOK_AHEAD_DIST,
    half_width: float = ROBOT_HALF_WIDTH,
    min_ground_height: float = MIN_OBSTACLE_GROUND_HEIGHT,
) -> List[VelodynePoint]:
    """
    Return points inside the forward safety corridor using ground-relative height.

    Corridor:
        Y ∈ (0, look_ahead]                — ahead of the robot
        |X| < half_width                   — within robot width + clearance
        z + LIDAR_MOUNT_HEIGHT > min_ground_height  — above ground
    """
    return [
        p for p in points
        if 0.0 < p.y <= look_ahead
        and abs(p.x) < half_width
        and (p.z + LIDAR_MOUNT_HEIGHT) > min_ground_height
    ]


def nearest_range(zone_points: List[VelodynePoint]) -> float:
    """Planar range to closest point in zone, or inf."""
    if not zone_points:
        return math.inf
    return min(math.hypot(p.x, p.y) for p in zone_points)


def sector_counts(
    points: List[VelodynePoint],
    max_range: float = 5.0,
) -> Dict[str, int]:
    """
    Count returns in the four cardinal 60°-wide sectors within max_range.

    Returns dict with keys 'fwd', 'right', 'rear', 'left'.
    """
    counts: Dict[str, int] = {"fwd": 0, "right": 0, "rear": 0, "left": 0}
    for p in points:
        if math.hypot(p.x, p.y) > max_range:
            continue
        az = p.azimuth % 360.0
        if _in_sector(az, 0.0):
            counts["fwd"] += 1
        elif _in_sector(az, 90.0):
            counts["right"] += 1
        elif _in_sector(az, 180.0):
            counts["rear"] += 1
        elif _in_sector(az, 270.0):
            counts["left"] += 1
    return counts


def nearest_in_sector(
    points: List[VelodynePoint],
    centre_az: float,
    max_range: float = 10.0,
) -> float:
    """Nearest planar range to any point within the given sector, or inf."""
    candidates = [
        math.hypot(p.x, p.y)
        for p in points
        if _in_sector(p.azimuth % 360.0, centre_az) and math.hypot(p.x, p.y) <= max_range
    ]
    return min(candidates, default=math.inf)


async def validate_lidar_startup(
    lidar: LidarDriver,
    num_scans: int = 5,
    min_points_per_scan: int = 5000,
    min_rings: int = 14,
) -> bool:
    """
    Collect num_scans, print per-scan diagnostics, and return True if healthy.

    Checks performed:
      - Total point count per scan >= min_points_per_scan
      - Ring coverage >= min_rings of 16 channels
      - Per-sector density and nearest obstacle per sector

    Raises no exceptions — returns False on failure so the caller can abort motion.
    """
    print(f"\n[LiDAR validate] collecting {num_scans} scans …")
    results: List[Dict] = []

    count = 0
    async for scan in lidar.scan_stream():
        rings_seen = len({p.ring for p in scan})
        sct = sector_counts(scan)
        n_fwd = nearest_in_sector(scan, 0.0)
        n_left = nearest_in_sector(scan, 270.0)
        n_right = nearest_in_sector(scan, 90.0)

        results.append({"total": len(scan), "rings": rings_seen, "sectors": sct})

        def _fmt(v: float) -> str:
            return f"{v:.2f} m" if v < math.inf else "  open"

        print(
            f"  [{count + 1}/{num_scans}] "
            f"pts={len(scan):5d} rings={rings_seen:2d}/16 | "
            f"fwd={sct['fwd']:4d} left={sct['left']:4d} right={sct['right']:4d} rear={sct['rear']:4d} | "
            f"nearest → fwd={_fmt(n_fwd)} left={_fmt(n_left)} right={_fmt(n_right)}"
        )

        count += 1
        if count >= num_scans:
            break

    if not results:
        print("[LiDAR validate] FAIL: no scans received — check UDP (port 2368)")
        return False

    avg_pts = sum(r["total"] for r in results) / len(results)
    avg_rings = sum(r["rings"] for r in results) / len(results)

    ok = True
    if avg_pts < min_points_per_scan:
        print(
            f"[LiDAR validate] FAIL: avg {avg_pts:.0f} pts/scan < {min_points_per_scan} — "
            "check 192.168.1.100 alias and UDP port 2368"
        )
        ok = False

    if avg_rings < min_rings:
        print(
            f"[LiDAR validate] WARN: {avg_rings:.1f}/16 rings active — "
            "possible beam blockage or partial obstruction"
        )

    if ok:
        avg_fwd = sum(r["sectors"]["fwd"] for r in results) / len(results)
        avg_lat = sum(r["sectors"]["left"] + r["sectors"]["right"] for r in results) / (2 * len(results))
        if avg_lat > 500 and avg_fwd < avg_lat * 0.1:
            print(
                f"[LiDAR validate] WARN: forward sector very sparse "
                f"(fwd={avg_fwd:.0f} vs lateral avg={avg_lat:.0f}) — "
                "verify mount yaw is 0°"
            )
        print(f"[LiDAR validate] OK — avg {avg_pts:.0f} pts/scan, {avg_rings:.1f}/16 rings")

    return ok
