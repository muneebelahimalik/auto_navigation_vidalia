#!/usr/bin/env python3
"""Tests for navigation.scan_accumulator.ScanAccumulator — motion-compensated
multi-scan densification for the dense-canopy row fit."""
import math

import numpy as np
import pytest

from navigation.scan_accumulator import ScanAccumulator


def test_n1_is_passthrough():
    """n_scans=1 returns the scan unchanged (byte-identical to no accumulation)."""
    acc = ScanAccumulator(n_scans=1)
    pts = np.random.default_rng(0).normal(size=(50, 3))
    out = acc.update(pts, d_fwd=0.15, d_theta=0.0)
    assert out is pts


def test_forward_motion_registers_static_feature():
    """A static point, seen from two scans 0.5 m apart, must land at the SAME
    place in the merged current-frame cloud (motion-compensated) and the cloud
    must be densified (2× the points)."""
    acc = ScanAccumulator(n_scans=3)
    # scan 1 at the origin: a feature at robot (0.4, 3.0)
    acc.update(np.array([[0.4, 3.0, 0.2]]), d_fwd=0.0, d_theta=0.0)
    # robot advances 0.5 m; the SAME feature is now 0.5 m closer → (0.4, 2.5)
    merged = acc.update(np.array([[0.4, 2.5, 0.2]]), d_fwd=0.5, d_theta=0.0)
    assert len(merged) == 2                                   # densified
    # both copies align at the current-frame position (0.4, 2.5)
    assert np.allclose(merged[:, 0], 0.4, atol=1e-6)
    assert np.allclose(merged[:, 1], 2.5, atol=1e-6)
    assert np.allclose(merged[:, 2], 0.2, atol=1e-6)          # height untouched


def test_rotation_registers_static_feature():
    """With a heading change between scans, the accumulator still aligns a static
    feature (rotation-compensated)."""
    acc = ScanAccumulator(n_scans=2)
    p0 = np.array([[0.0, 3.0, 0.1]])                          # straight ahead, 3 m
    acc.update(p0, d_fwd=0.0, d_theta=0.0)
    # robot turns left (CCW) by 10°, no translation.  The same world feature now
    # appears rotated by −10° in the robot frame.
    dth = math.radians(10.0)
    r = 3.0
    # feature at world (0, 3); in the new robot frame rotated by dth:
    # robot = Rot(dth)^T · world  (translation 0)
    rx = math.cos(dth) * 0.0 + math.sin(dth) * r
    ry = -math.sin(dth) * 0.0 + math.cos(dth) * r
    merged = acc.update(np.array([[rx, ry, 0.1]]), d_fwd=0.0, d_theta=dth)
    assert len(merged) == 2
    # both copies land at the same current-frame point (the freshly-observed one)
    assert np.allclose(merged[0], merged[1], atol=1e-6)
    assert merged[1] == pytest.approx([rx, ry, 0.1], abs=1e-6)


def test_window_caps_at_n_scans():
    """Only the last n scans are retained."""
    acc = ScanAccumulator(n_scans=3)
    for _ in range(6):
        merged = acc.update(np.array([[0.0, 3.0, 0.0]]), d_fwd=0.15, d_theta=0.0)
    assert len(merged) == 3                                   # not 6


def test_max_points_subsamples():
    """max_points caps the merged cloud size."""
    acc = ScanAccumulator(n_scans=5, max_points=100)
    rng = np.random.default_rng(1)
    for _ in range(5):
        merged = acc.update(rng.normal(size=(60, 3)), d_fwd=0.02, d_theta=0.0)
    assert len(merged) == 100                                 # 300 → capped to 100


def test_reset_clears_buffer_and_pose():
    acc = ScanAccumulator(n_scans=3)
    acc.update(np.array([[0.4, 3.0, 0.2]]), d_fwd=0.3, d_theta=0.1)
    acc.reset()
    out = acc.update(np.array([[0.4, 3.0, 0.2]]), d_fwd=0.0, d_theta=0.0)
    assert len(out) == 1                                      # buffer cleared
    assert acc._x == 0.0 and acc._y == 0.0 and acc._phi == 0.0


def test_accumulation_densifies_a_sparse_dense_canopy_fit():
    """End-to-end intent: accumulating several noisy consecutive scans of the
    same two-row scene gives the detector many more points to fit (the density
    restoration the dense-canopy weave needs)."""
    from navigation.row_perception import RowDetector

    def scene(seed, n=120):
        rng = np.random.default_rng(seed)
        pts = []
        for c in (-0.38, 0.38):
            y = rng.uniform(1.6, 6.0, n // 2)
            x = c + rng.normal(0, 0.05, n // 2)
            z = rng.uniform(0.35, 0.55, n // 2) - 0.80
            pts.append(np.column_stack((x, y, z)))
        return np.vstack(pts)

    acc = ScanAccumulator(n_scans=6)
    merged = None
    for k in range(6):
        merged = acc.update(scene(k), d_fwd=0.015, d_theta=0.0)   # 1.5 cm/scan
    assert len(merged) > 120 * 4                                  # ~6× denser
    # the merged cloud still fits a centred row (registration didn't smear it)
    det = RowDetector(dual_row=True, crop_h_min=0.10, crop_h_max=0.70)
    e = det.update(merged)
    assert abs(e.lateral_offset) < 0.08
