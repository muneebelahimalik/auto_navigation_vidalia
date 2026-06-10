"""Unit tests for lidar/obstacle_filter.py tilt correction and the
camera/soybean_row_tracker.py dual-camera fusion geometry (the _fuse path is
deliberately OpenCV-free so it can be tested without cv2)."""
import math

import numpy as np
import pytest

from camera.soybean_row_tracker import DualCameraRowTracker, _SideResult
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT, tilt_correct_pts

TILT = math.radians(15.0)


# ---------------------------------------------------------------------------
# LiDAR tilt correction
# ---------------------------------------------------------------------------

def _world_to_tilted(y_w, z_w, tilt):
    """Inverse of tilt_correct_pts for a single (y, z)."""
    c, s = math.cos(tilt), math.sin(tilt)
    return y_w * c - z_w * s, y_w * s + z_w * c


def test_ground_return_corrects_to_zero_height():
    """A true ground point seen through a 15-deg nose-down mount must come out
    at ground-relative height ~0 after correction."""
    y_s, z_s = _world_to_tilted(3.0, -LIDAR_MOUNT_HEIGHT, TILT)
    pts = np.array([[0.4, y_s, z_s]])
    out = tilt_correct_pts(pts, TILT)
    h = out[0, 2] + LIDAR_MOUNT_HEIGHT
    assert h == pytest.approx(0.0, abs=1e-9)
    assert out[0, 1] == pytest.approx(3.0, abs=1e-9)
    assert out[0, 0] == 0.4                      # X untouched


def test_crop_height_preserved():
    """Soybean top at h=0.25 m, 2.5 m ahead -> corrected height = 0.25 m."""
    z_w = 0.25 - LIDAR_MOUNT_HEIGHT
    y_s, z_s = _world_to_tilted(2.5, z_w, TILT)
    out = tilt_correct_pts(np.array([[0.0, y_s, z_s]]), TILT)
    assert out[0, 2] + LIDAR_MOUNT_HEIGHT == pytest.approx(0.25, abs=1e-9)


def test_zero_tilt_is_identity():
    pts = np.array([[1.0, 2.0, 3.0]])
    assert tilt_correct_pts(pts, 0.0) is pts


def test_uncorrected_height_error_magnitude():
    """Document the failure mode: without correction a ground return 3 m out
    appears ~0.78 m above ground — above every obstacle threshold."""
    y_s, z_s = _world_to_tilted(3.0, -LIDAR_MOUNT_HEIGHT, TILT)
    h_uncorrected = z_s + LIDAR_MOUNT_HEIGHT
    assert h_uncorrected > 0.5


# ---------------------------------------------------------------------------
# Dual-camera flanking-row fusion (pure geometry, no cv2 required)
# ---------------------------------------------------------------------------

def side(world_x, heading=0.0, strength=100.0, green=0.2):
    return _SideResult(valid=True, world_x=world_x, heading=heading,
                       green_fraction=green, strength=strength)


def fuse_converged(tracker, left, right, iters=30):
    est = None
    for _ in range(iters):
        est = tracker._fuse(left, right)
    return est


def test_both_rows_midpoint():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    est = fuse_converged(tracker, side(-0.38), side(+0.38))
    assert est.lateral_offset == pytest.approx(0.0, abs=0.02)
    assert 0.30 <= est.confidence <= 0.50


def test_left_only_fallback_offsets_inward():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    est = fuse_converged(tracker, side(-0.30), _SideResult())
    assert est.lateral_offset == pytest.approx(-0.30 + 0.38, abs=0.02)
    assert est.confidence < 0.30                 # degraded, not dead


def test_right_only_fallback_offsets_inward():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    est = fuse_converged(tracker, _SideResult(), side(+0.30))
    assert est.lateral_offset == pytest.approx(0.30 - 0.38, abs=0.02)


def test_no_rows_decays_confidence():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    fuse_converged(tracker, side(-0.38), side(+0.38))
    conf = tracker._est.confidence
    for _ in range(4):
        est = tracker._fuse(_SideResult(), _SideResult())
    assert est.confidence < conf * 0.5
    assert not est.left_valid and not est.right_valid


def test_lopsided_strength_lowers_confidence():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    balanced = fuse_converged(tracker, side(-0.38, strength=100),
                              side(+0.38, strength=100)).confidence
    tracker.reset()
    lopsided = fuse_converged(tracker, side(-0.38, strength=100),
                              side(+0.38, strength=10)).confidence
    assert lopsided < balanced
