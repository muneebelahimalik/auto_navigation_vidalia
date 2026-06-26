"""Unit tests for the native 2-D LiDAR SLAM stack (slam/).

Focus: the mount-correction fix — SLAM must yaw/tilt-correct the raw VLP-16
cloud into the robot frame before slicing, exactly like the row-follow stack.
Without it the 21.5° nose-down tilt ramps far-field ground into the slice band
and floods the map with moving ground.  Also covers the ICP scan matcher and
the occupancy grid.
"""
import math

import numpy as np
import pytest

from lidar.lidar_driver import VelodynePoint
from lidar.obstacle_filter import (
    LIDAR_MOUNT_HEIGHT,
    tilt_correct_pts,
    yaw_correct_pts,
)
from slam.scan_matcher import (
    Pose2D,
    correct_scan,
    extract_2d_slice,
    icp_2d,
    sensor_to_world,
)
from slam.occupancy_grid import OccupancyGrid
from slam.slam_engine import SlamEngine

YAW = math.radians(66.0)
TILT = math.radians(21.5)


def _robot_to_raw(pts_robot: np.ndarray) -> np.ndarray:
    """Inverse of correct_scan: build the RAW sensor cloud the VLP-16 would
    report for a set of known robot-frame points (apply +tilt then +yaw)."""
    # correct_scan = tilt_correct(yaw_correct(raw)); invert in reverse order.
    pts = tilt_correct_pts(pts_robot, -TILT)
    pts = yaw_correct_pts(pts, -YAW)
    return pts


def _as_scan(pts_xyz: np.ndarray):
    return [VelodynePoint(float(x), float(y), float(z), 10, 0, 0.0)
            for x, y, z in pts_xyz]


# ---------------------------------------------------------------------------
# Mount correction round-trip
# ---------------------------------------------------------------------------

def test_correct_scan_round_trips_robot_frame():
    """A known robot-frame cloud, pushed out to raw sensor coords and back
    through correct_scan, returns to the robot frame."""
    rng = np.random.default_rng(0)
    pts_robot = rng.uniform(-5, 5, size=(50, 3))
    raw = _robot_to_raw(pts_robot)
    recovered = correct_scan(raw, YAW, TILT)
    assert np.allclose(recovered, pts_robot, atol=1e-9)


# ---------------------------------------------------------------------------
# The core fix: tilt-corrected slice rejects ramped ground
# ---------------------------------------------------------------------------

def _flat_ground_scan():
    """Raw VLP-16 returns for a flat ground plane (true height h = 0) spread
    across the forward field of view."""
    ys = np.linspace(2.0, 8.0, 40)
    xs = np.linspace(-3.0, 3.0, 13)
    gx, gy = np.meshgrid(xs, ys)
    pts_robot = np.column_stack([
        gx.ravel(), gy.ravel(),
        np.full(gx.size, -LIDAR_MOUNT_HEIGHT),   # z so that h = 0 (ground)
    ])
    return _as_scan(_robot_to_raw(pts_robot))


def test_uncorrected_slice_is_flooded_by_ramped_ground():
    """Without correction the nose-down tilt ramps far ground into the
    [0.20, 1.50] band — the bug."""
    scan = _flat_ground_scan()
    leaked = extract_2d_slice(scan, min_gnd=0.20, max_gnd=1.50,
                              yaw_rad=0.0, tilt_rad=0.0)
    assert len(leaked) > 50          # lots of ground masquerading as structure


def test_corrected_slice_excludes_flat_ground():
    """With the yaw/tilt correction flat ground collapses to h≈0 and drops out
    of a structure band that starts at 0.20 m."""
    scan = _flat_ground_scan()
    kept = extract_2d_slice(scan, min_gnd=0.20, max_gnd=1.50,
                            yaw_rad=YAW, tilt_rad=TILT)
    assert len(kept) == 0


def test_corrected_slice_keeps_vertical_structure():
    """A vertical pole survives the corrected slice at its true robot-frame
    (x, y) location."""
    x0, y0 = 1.2, 4.0
    zs = np.linspace(-LIDAR_MOUNT_HEIGHT + 0.30, -LIDAR_MOUNT_HEIGHT + 1.40, 20)
    pole_robot = np.column_stack([np.full_like(zs, x0), np.full_like(zs, y0), zs])
    scan = _as_scan(_robot_to_raw(pole_robot))
    kept = extract_2d_slice(scan, min_gnd=0.20, max_gnd=1.50,
                            yaw_rad=YAW, tilt_rad=TILT)
    assert len(kept) >= 15
    assert np.allclose(kept[:, 0], x0, atol=1e-6)
    assert np.allclose(kept[:, 1], y0, atol=1e-6)


# ---------------------------------------------------------------------------
# ICP scan matcher
# ---------------------------------------------------------------------------

def test_icp_recovers_pure_translation():
    rng = np.random.default_rng(1)
    target = rng.uniform(-5, 5, size=(200, 2))
    shift = np.array([0.30, -0.20])
    source = target - shift                       # source @ I + shift = target
    pose, err = icp_2d(source, target, Pose2D(0.0, 0.0, 0.0))
    assert err < 1e-3
    assert pose.x == pytest.approx(shift[0], abs=1e-2)
    assert pose.y == pytest.approx(shift[1], abs=1e-2)
    assert pose.theta == pytest.approx(0.0, abs=1e-2)


def test_icp_recovers_small_rotation():
    rng = np.random.default_rng(2)
    target = rng.uniform(-5, 5, size=(200, 2))
    ang = math.radians(8.0)
    c, s = math.cos(ang), math.sin(ang)
    R = np.array([[c, -s], [s, c]])
    source = target @ R                            # rotate target by -ang
    pose, err = icp_2d(source, target, Pose2D(0.0, 0.0, 0.0))
    assert err < 1e-2
    assert pose.theta == pytest.approx(ang, abs=math.radians(1.0))


# ---------------------------------------------------------------------------
# Occupancy grid
# ---------------------------------------------------------------------------

def test_occupancy_grid_marks_endpoint_and_clears_free_space():
    grid = OccupancyGrid(size_m=20.0, resolution=0.10)
    # One obstacle 3 m ahead of a robot at the origin.
    pts = np.array([[0.0, 3.0]])
    grid.update_scan(0.0, 0.0, pts)
    occ = grid.get_occupied_world()
    assert len(occ) >= 1
    # The endpoint cell is occupied …
    assert np.any((np.abs(occ[:, 0]) < 0.06) & (np.abs(occ[:, 1] - 3.0) < 0.06))
    # … and a cell halfway along the ray was cleared (not occupied).
    assert not np.any((np.abs(occ[:, 0]) < 0.06) & (np.abs(occ[:, 1] - 1.5) < 0.06))


# ---------------------------------------------------------------------------
# Engine smoke test — corrected ground does not build a ghost map
# ---------------------------------------------------------------------------

def test_engine_ignores_flat_ground_with_correction():
    """A stream of flat-ground-only scans must not accumulate a structure map
    once the tilt correction is applied (slice starts above the ground)."""
    engine = SlamEngine(grid_size_m=40.0, slice_min=0.20, slice_max=1.50)
    scan = _flat_ground_scan()
    for _ in range(10):
        engine.process_scan(scan)
    # Ground is flattened out of the slice → too few slice points → no map.
    assert engine.cell_count == 0


def test_engine_maps_structure_and_tracks_pose():
    """With real vertical structure the engine populates the grid and keeps a
    sane (near-stationary) pose for a stationary sensor."""
    # A ring of poles around the robot gives ICP something to lock onto.
    poles = []
    for ang in np.linspace(0, 2 * math.pi, 24, endpoint=False):
        px, py = 5.0 * math.cos(ang), 5.0 * math.sin(ang)
        for z in np.linspace(-LIDAR_MOUNT_HEIGHT + 0.30, -LIDAR_MOUNT_HEIGHT + 1.2, 8):
            poles.append((px, py, z))
    scan = _as_scan(_robot_to_raw(np.array(poles)))

    engine = SlamEngine(grid_size_m=40.0, slice_min=0.20, slice_max=1.50)
    for _ in range(6):
        engine.process_scan(scan)
    state = engine.get_state()
    assert engine.cell_count > 0
    # Sensor never moved → pose should stay near the origin.
    assert abs(state.pose.x) < 0.5
    assert abs(state.pose.y) < 0.5
