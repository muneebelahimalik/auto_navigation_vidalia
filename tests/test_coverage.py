#!/usr/bin/env python3
"""
test_coverage.py — Regression tests for the field-coverage accounting.

Coverage is the operational payoff of SLAM on a row-follower: the serviced
swath stamped along the drift-corrected pose path.  These lock the geometry
(a straight pass covers ~length×swath), the gap behaviour, and the engine /
save-file integration.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from slam.coverage_map import CoverageGrid
from slam.map_io import save_map, save_trajectory_csv
from slam.occupancy_grid import OccupancyGrid


def test_single_pose_stamps_a_disc():
    cov = CoverageGrid(size_m=40, resolution=0.10, swath_m=2.0)
    cov.add_pose(0.0, 0.0)
    # A swath of 2 m → disc radius 1 m → area ≈ π m².
    assert 2.5 < cov.covered_area_m2 < 3.6
    assert cov.path_length_m == 0.0


def test_straight_pass_area_matches_length_times_swath():
    cov = CoverageGrid(size_m=80, resolution=0.10, swath_m=2.0)
    # Drive a 20 m straight line in small steps.
    for i in range(201):
        cov.add_pose(i * 0.1, 0.0)
    # Rectangle 20 m × 2 m = 40 m², plus two end half-discs (~π m²) ≈ 43 m².
    assert 40.0 < cov.covered_area_m2 < 46.0
    assert abs(cov.path_length_m - 20.0) < 0.2
    # A single non-overlapping pass → redundancy ≈ 1.
    assert 0.85 < cov.redundancy < 1.15


def test_gap_between_passes_is_not_covered():
    cov = CoverageGrid(size_m=80, resolution=0.10, swath_m=1.0)
    # Two parallel passes 5 m apart (gap >> swath) → middle stays uncovered.
    for i in range(101):
        cov.add_pose(i * 0.1, 0.0)
    for i in range(101):
        cov.add_pose(i * 0.1, 5.0)
    g = cov.origin
    # A cell at (x=8, y=3) is between the two passes AND off the return
    # transition diagonal (which runs (10,0)->(0,5)) → must be unserviced.
    mid_row = int(round(3.0 / cov.res)) + g
    col = int(round(8.0 / cov.res)) + g
    assert not cov.covered_mask[mid_row, col]


def test_interpolation_fills_a_fast_jump():
    # A single large jump (e.g. a dropped scan) must not leave a hole.
    cov = CoverageGrid(size_m=80, resolution=0.10, swath_m=1.0)
    cov.add_pose(0.0, 0.0)
    cov.add_pose(3.0, 0.0)          # 3 m jump in one step
    g = cov.origin
    # The midpoint of the jump should still be covered.
    row = g
    col = int(round(1.5 / cov.res)) + g
    assert cov.covered_mask[row, col]
    assert abs(cov.path_length_m - 3.0) < 1e-6


def test_redundancy_above_one_when_passes_overlap():
    cov = CoverageGrid(size_m=80, resolution=0.10, swath_m=2.0)
    # Two overlapping passes only 0.5 m apart (< swath) → double-covered.
    for i in range(101):
        cov.add_pose(i * 0.1, 0.0)
    for i in range(101):
        cov.add_pose(i * 0.1, 0.5)
    assert cov.redundancy > 1.3        # significant overlap


def test_stats_keys():
    cov = CoverageGrid(swath_m=1.5)
    cov.add_pose(0.0, 0.0); cov.add_pose(1.0, 0.0)
    s = cov.stats()
    assert set(s) == {"covered_area_m2", "path_length_m", "swath_m", "redundancy"}
    assert s["swath_m"] == 1.5


def test_save_map_writes_coverage_and_trajectory(tmp_path):
    grid = OccupancyGrid(size_m=40, resolution=0.10)
    cov = CoverageGrid(size_m=40, resolution=0.10, swath_m=1.5)
    traj = []
    full = []
    for i in range(51):
        x = i * 0.1
        cov.add_pose(x, 0.0)
        traj.append([x, 0.0]); full.append([x, 0.0, 0.0])
    out = save_map(grid, traj, tmp_path, coverage=cov,
                   full_trajectory=np.asarray(full))
    assert (out / "coverage.png").exists()
    assert (out / "trajectory.csv").exists()
    # CSV has a header + one row per pose.
    lines = (out / "trajectory.csv").read_text().strip().splitlines()
    assert lines[0] == "scan,x_m,y_m,heading_deg"
    assert len(lines) == 1 + len(full)
    # Covered mask is persisted in the npz.
    data = np.load(out / "map.npz")
    assert "covered_mask" in data
    assert data["covered_mask"].sum() > 0


def test_save_map_without_coverage_is_unchanged(tmp_path):
    # Back-compat: omitting coverage must not write coverage artefacts.
    grid = OccupancyGrid(size_m=40, resolution=0.10)
    out = save_map(grid, [[0.0, 0.0], [1.0, 0.0]], tmp_path)
    assert (out / "map.png").exists()
    assert not (out / "coverage.png").exists()
    assert not (out / "trajectory.csv").exists()
