"""Unit tests for navigation/row_perception.py — LiDAR row detection geometry.

Synthetic VLP-16-like crop clouds are generated directly in the sensor frame
(X=right, Y=forward, Z=up, h = z + LIDAR_MOUNT_HEIGHT) so every geometric
claim of the detector can be checked without hardware.
"""
import math

import numpy as np
import pytest

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
from navigation.row_perception import RowDetector

RNG = np.random.default_rng(42)

ROW_SPACING = 0.76
HALF = ROW_SPACING / 2.0


def make_row(x_centre, n=130, y_lo=1.6, y_hi=6.8, x_sigma=0.03,
             h_lo=0.08, h_hi=0.25, heading_rad=0.0):
    """Points along one crop row at lateral position x_centre.

    heading_rad tilts the row direction toward +X (robot's right) — the
    same sign convention as RowEstimate.heading_error.
    """
    y = RNG.uniform(y_lo, y_hi, n)
    x = x_centre + RNG.normal(0.0, x_sigma, n) + y * math.tan(heading_rad)
    h = RNG.uniform(h_lo, h_hi, n)
    z = h - LIDAR_MOUNT_HEIGHT
    return np.column_stack((x, y, z))


def converge(det, pts, iters=25):
    """Feed the same cloud repeatedly so the EMA converges."""
    est = None
    for _ in range(iters):
        est = det.update(pts)
    return est


# ---------------------------------------------------------------------------
# Dual-row (soybean / centre-residue) mode
# ---------------------------------------------------------------------------

def test_dual_row_centred():
    """Robot centred between flanking rows -> lateral offset ~ 0."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([make_row(-HALF), make_row(+HALF)])
    est = converge(det, pts)
    assert est.valid
    assert abs(est.lateral_offset) < 0.05
    assert est.confidence > 0.5


def test_dual_row_offset_right():
    """Robot 0.2 m right of centre -> rows appear shifted left -> lateral ~ -0.2."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([make_row(-HALF - 0.20), make_row(+HALF - 0.20)])
    est = converge(det, pts)
    assert est.lateral_offset == pytest.approx(-0.20, abs=0.07)


def test_dual_row_single_side_fallback_steers_to_centre():
    """Only the LEFT row visible: the residue centre is half a row spacing to
    its RIGHT.  The old code returned the peak itself, steering the robot
    directly onto the soybean row (wrong sign of correction)."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = make_row(-0.20)            # left row close on the left; right row out of ROI
    est = converge(det, pts)
    expected = -0.20 + HALF          # +0.18: steer right, toward the residue centre
    assert est.lateral_offset == pytest.approx(expected, abs=0.07)
    assert est.lateral_offset > 0.0  # must NOT steer left onto the row


def test_dual_row_single_side_fallback_right_row():
    """Mirror case: only the RIGHT row visible -> centre is half spacing left."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = make_row(+0.20)
    est = converge(det, pts)
    assert est.lateral_offset == pytest.approx(0.20 - HALF, abs=0.07)
    assert est.lateral_offset < 0.0


def test_dual_row_spacing_prior_rejects_centre_clutter():
    """A weed clump near the centreline must not hijack the peak pairing.

    Old behaviour: innermost left/right peaks -> pairs (-0.38, +0.10) ->
    midpoint -0.14 (wrong).  With the row-spacing prior the (±0.38) pair wins
    because its separation matches row_spacing."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    clutter = make_row(+0.10, n=60, x_sigma=0.02)
    pts = np.vstack([make_row(-HALF), make_row(+HALF), clutter])
    est = converge(det, pts)
    assert abs(est.lateral_offset) < 0.06


def test_dual_row_heading_detection():
    """Rows angled 10 degrees to the robot's right -> positive heading error."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    theta = math.radians(10.0)
    pts = np.vstack([make_row(-HALF, heading_rad=theta),
                     make_row(+HALF, heading_rad=theta)])
    est = converge(det, pts)
    assert est.heading_error == pytest.approx(theta, abs=math.radians(4.0))


# ---------------------------------------------------------------------------
# Single-row (onion) mode
# ---------------------------------------------------------------------------

def test_single_row_centred():
    det = RowDetector(dual_row=False, crop_h_min=0.05, crop_h_max=0.60)
    pts = make_row(0.0, h_lo=0.15, h_hi=0.45)
    est = converge(det, pts)
    assert abs(est.lateral_offset) < 0.05
    assert est.confidence > 0.5


def test_single_row_offset():
    det = RowDetector(dual_row=False, crop_h_min=0.05, crop_h_max=0.60)
    pts = make_row(+0.25, h_lo=0.15, h_hi=0.45)
    est = converge(det, pts)
    assert est.lateral_offset == pytest.approx(0.25, abs=0.06)


# ---------------------------------------------------------------------------
# Temporal behaviour: gates, decay, reset
# ---------------------------------------------------------------------------

def test_lateral_jump_gate_limits_per_scan_snap():
    """A sudden 0.6 m jump of the detected row must be slewed, not followed."""
    det = RowDetector(dual_row=False, max_lateral_jump=0.30)
    converge(det, make_row(0.0))
    est = det.update(make_row(0.60))    # one outlier scan
    # ungated EMA step would be 0.35*0.60 = 0.21; gated: 0.35*0.30 = 0.105
    assert abs(est.lateral_offset) <= 0.12


def test_confidence_decays_on_empty_scans():
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est = converge(det, np.vstack([make_row(-HALF), make_row(+HALF)]))
    conf_full = est.confidence
    for _ in range(5):
        est = det.update(np.empty((0, 3)))
    assert est.confidence < conf_full * 0.5
    assert not est.valid
    assert est.row_end_confidence == 1.0


def test_reset_clears_smoothed_state():
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    converge(det, np.vstack([make_row(-HALF - 0.2), make_row(+HALF - 0.2)]))
    det.reset()
    est = det.update(np.empty((0, 3)))
    assert est.lateral_offset == 0.0
    assert est.confidence == 0.0
