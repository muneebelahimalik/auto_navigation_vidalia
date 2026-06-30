#!/usr/bin/env python3
"""
test_viz_perception.py — Sanity tests for the perception-figure tool.

The ray-cast scene generator is pure numpy and tested everywhere; the matplotlib
renderers are exercised only where matplotlib is installed (skipped on the brain
via importorskip, so `pytest tests/` stays green there).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from scripts.viz_perception import MOUNT_H, SELF_R, generate_lidar_raycast


def test_raycast_scene_geometry():
    P = generate_lidar_raycast(lateral=-0.05, heading_deg=3.0, spacing=0.62, seed=1)
    assert P.ndim == 2 and P.shape[1] == 3
    assert len(P) > 1000                          # a dense scan
    assert np.isfinite(P).all()
    # self-filter blind zone: no returns inside SELF_R planar range
    assert np.hypot(P[:, 0], P[:, 1]).min() >= SELF_R - 1e-6
    h = P[:, 2] + MOUNT_H                          # height above ground
    assert np.median(h) < 0.1                      # mostly ground returns
    assert h.max() > 0.12                          # crop rows rise above ground
    assert h.min() > -0.2                          # nothing absurdly below ground


def test_raycast_is_deterministic_with_seed():
    a = generate_lidar_raycast(seed=7)
    b = generate_lidar_raycast(seed=7)
    assert a.shape == b.shape and np.allclose(a, b)


def test_render_all_writes_three_figures(tmp_path):
    pytest.importorskip("matplotlib")
    from scripts.viz_perception import render_all
    P = generate_lidar_raycast(lateral=-0.06, heading_deg=3.5, spacing=0.62, seed=2)
    files = render_all(P, -0.06, 3.5, 0.62, tmp_path)
    names = {Path(f).name for f in files}
    assert "figure_perception_3d.png" in names
    assert "figure_perception_bev.png" in names
    assert "figure_perception_annotated.png" in names
    for f in files:
        assert Path(f).exists() and Path(f).stat().st_size > 0
