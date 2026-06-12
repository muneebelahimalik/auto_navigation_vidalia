"""Unit tests for navigation/row_perception.py — LiDAR row detection geometry.

Synthetic VLP-16-like crop clouds are generated directly in the sensor frame
(X=right, Y=forward, Z=up, h = z + LIDAR_MOUNT_HEIGHT) so every geometric
claim of the detector can be checked without hardware.
"""
import math

import numpy as np
import pytest

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
from navigation.row_perception import RowDetector, find_row_midpoint

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


def test_dual_row_truncated_stripe_does_not_tilt_heading():
    """One row truncated in y (VLP-16 sector dropout): heading must stay ~0.

    Whole-cloud PCA over a full left stripe (y 1.6-6.8) plus a truncated
    right stripe (y 1.6-3.0) tilts the principal axis toward the line
    joining the stripe centroids — a spurious heading of several degrees
    that steered the robot steadily off the strip in the field.  The
    cluster-centred two-pass PCA removes the between-stripe covariance.
    """
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([make_row(-HALF, n=130),
                     make_row(+HALF, n=40, y_hi=3.0)])
    est = converge(det, pts)
    assert abs(est.heading_error) < math.radians(3.0)
    assert abs(est.lateral_offset) < 0.07


def test_dual_row_truncated_stripe_keeps_true_heading():
    """Same dropout scenario but with genuinely angled rows: the detector
    must still recover the true 8 degree heading, not flatten it to zero."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    theta = math.radians(8.0)
    pts = np.vstack([make_row(-HALF, n=130, heading_rad=theta),
                     make_row(+HALF, n=40, y_hi=3.0, heading_rad=theta)])
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


# ---------------------------------------------------------------------------
# Point-level fusion: camera ground points pooled into the LiDAR fit
# ---------------------------------------------------------------------------

def make_aux_rows(offset=0.0, n_per_row=200, y_lo=0.8, y_hi=4.5):
    """Camera-tracker-style metric ground points for both flanking rows."""
    rng = np.random.default_rng(11)
    pts = []
    for xc in (-HALF + offset, +HALF + offset):
        y = rng.uniform(y_lo, y_hi, n_per_row)
        x = xc + rng.normal(0.0, 0.03, n_per_row)
        pts.append(np.column_stack((x, y)))
    return np.vstack(pts)


def test_weighted_midpoint_downweights_heavy_clutter():
    """Many clutter points with low per-point weight must not outvote
    fewer full-weight row points in the histogram pairing."""
    rng = np.random.default_rng(3)
    rows = np.concatenate([rng.normal(-HALF, 0.02, 80), rng.normal(+HALF, 0.02, 80)])
    clutter = rng.normal(+0.12, 0.02, 400)            # 5x the count of one row
    cross = np.concatenate([rows, clutter])
    weights = np.concatenate([np.ones(160), np.full(400, 0.05)])  # mass 20
    lateral, _ = find_row_midpoint(cross, 0.80, 0.10, ROW_SPACING, weights=weights)
    assert abs(lateral) < 0.06


def test_point_fusion_camera_carries_empty_lidar_scan():
    """Empty VLP-16 crop ROI + camera ground points: the fit must survive at
    camera-capped confidence instead of decaying toward ACQUIRE."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    aux = make_aux_rows()
    est = None
    for _ in range(25):
        est = det.update(np.empty((0, 3)), aux_xy=aux)
    assert est.valid
    assert abs(est.lateral_offset) < 0.06
    assert 0.20 <= est.confidence <= 0.55          # capped by aux_mass_ratio
    assert est.row_end_confidence == 1.0           # row-end stays LiDAR-only


def test_point_fusion_boosts_sparse_lidar():
    """A sparse LiDAR scan plus camera points must beat the sparse scan alone."""
    sparse = np.vstack([make_row(-HALF, n=30), make_row(+HALF, n=30)])
    det_solo = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est_solo = converge(det_solo, sparse)
    det_fused = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est_fused = None
    for _ in range(25):
        est_fused = det_fused.update(sparse, aux_xy=make_aux_rows())
    assert est_fused.confidence > est_solo.confidence
    assert abs(est_fused.lateral_offset) < 0.06


def test_point_fusion_lidar_dominates_disagreeing_camera():
    """Healthy LiDAR centred, camera rows shifted +0.35 m (bad calibration /
    wrong rows): the capped camera mass must not drag the fit off the LiDAR."""
    lidar = np.vstack([make_row(-HALF), make_row(+HALF)])
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est = None
    for _ in range(25):
        est = det.update(lidar, aux_xy=make_aux_rows(offset=0.35))
    assert abs(est.lateral_offset) < 0.12


def test_point_fusion_none_aux_matches_lidar_only():
    """aux_xy=None must reproduce the pure-LiDAR fit exactly."""
    pts = np.vstack([make_row(-HALF - 0.1), make_row(+HALF - 0.1)])
    det_a = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    det_b = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    for _ in range(10):
        ea = det_a.update(pts)
        eb = det_b.update(pts, aux_xy=None)
    assert ea.lateral_offset == eb.lateral_offset
    assert ea.heading_error == eb.heading_error
    assert ea.confidence == eb.confidence


# ---------------------------------------------------------------------------
# Precision: finer bin_width should reduce centred-row lateral error
# ---------------------------------------------------------------------------

def test_fine_bin_width_improves_centre_precision():
    """bin_width=0.05 m should place the row centre within 3 cm of truth;
    the old 0.10 m default was limited to ±5 cm worst-case quantisation."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING, bin_width=0.05)
    pts = np.vstack([make_row(-HALF), make_row(+HALF)])
    est = converge(det, pts)
    assert abs(est.lateral_offset) < 0.03, (
        f"Expected lateral offset < 3 cm with 5 cm bins, got {est.lateral_offset*100:.1f} cm"
    )


def test_fine_bin_width_offset_accuracy():
    """With 5 cm bins, a 15 cm lateral offset should be reported within one
    bin width (5 cm) — coarser 10 cm bins give ±5 cm worst-case; the finer
    bins halve the quantisation error."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING, bin_width=0.05)
    pts = np.vstack([make_row(-HALF - 0.15), make_row(+HALF - 0.15)])
    est = converge(det, pts)
    assert est.lateral_offset == pytest.approx(-0.15, abs=0.05), (
        f"Expected ~-0.15 m within 5 cm, got {est.lateral_offset:.3f} m"
    )


# ---------------------------------------------------------------------------
# Default parameter assertions — catches accidental drift
# ---------------------------------------------------------------------------

def test_detector_default_bin_width():
    """Default bin_width should be 0.05 m for improved centre precision."""
    det = RowDetector()
    assert det.bin_width == 0.05


def test_detector_default_max_lateral_jump():
    """Default max_lateral_jump should be 0.20 m for tighter stability."""
    det = RowDetector()
    assert det.max_lateral_jump == 0.20


def test_lateral_jump_gate_tighter_default():
    """With default 0.20 m gate, a 0.50 m jump must be slewed to ≤ 0.08 m."""
    det = RowDetector(dual_row=False)
    converge(det, make_row(0.0))
    est = det.update(make_row(0.50))
    # gated step: 0.35 * 0.20 = 0.07 m from prior 0.0
    assert abs(est.lateral_offset) <= 0.08


def test_sparse_scan_heading_gate():
    """A dropout scan with < 2×min_points crop returns and a large heading
    jump must not update the heading EMA.

    Mirrors the field failure: VLP-16 sector dropout left n≈67 crop points
    whose foreshortened two-stripe view produced a raw PCA heading of ~27°.
    This passed the 30° gate and biased the smoothed heading to +10°,
    steering the robot rightward for the rest of the approach.
    With the sparse-scan gate (< 2×min_points AND |Δhdg| > 12°) the heading
    update is skipped and the smoothed heading stays near the pre-dropout value.
    """
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    # Converge at heading ~ 0 with plenty of points.
    good_pts = np.vstack([make_row(-HALF, n=130), make_row(+HALF, n=130)])
    est = converge(det, good_pts)
    hdg_before = est.heading_error
    assert abs(hdg_before) < math.radians(3.0), "baseline heading not near zero"

    # Craft a sparse scan (n_crop < 2×min_points=80) with a large raw heading.
    # heading_rad=math.radians(25) gives a clearly off-axis stripe.
    sparse_pts = np.vstack([
        make_row(-HALF, n=30, heading_rad=math.radians(25.0)),
        make_row(+HALF, n=30, heading_rad=math.radians(25.0)),
    ])
    est_after = det.update(sparse_pts)

    # The sparse-scan gate must prevent the heading from jumping > 12°.
    assert abs(est_after.heading_error - hdg_before) < math.radians(13.0), (
        f"Sparse scan allowed heading to jump from {math.degrees(hdg_before):.1f}° "
        f"to {math.degrees(est_after.heading_error):.1f}°"
    )
    # Specifically: heading must stay much closer to zero than 25°.
    assert abs(est_after.heading_error) < math.radians(14.0), (
        f"Heading drifted to {math.degrees(est_after.heading_error):.1f}° from sparse dropout"
    )
