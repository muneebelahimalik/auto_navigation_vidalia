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


def test_strip_lock_resists_row_hop():
    """Continuity prior: once tracking a strip, an ambiguous scene where the
    ADJACENT strip is also visible must NOT make the midpoint jump a whole row
    (the field-observed row-hop under an aggressive correction)."""
    det = RowDetector(dual_row=True, row_spacing=0.76)
    # lock onto the centre strip (rows at ±0.38) → tracked lateral ≈ 0
    converge(det, np.vstack([make_row(-0.38), make_row(+0.38)]), iters=25)
    assert abs(det._est.lateral_offset) < 0.06
    # robot 0.20 m right of the centre strip; adjacent strip also in view.
    # peaks at -0.58 (left row), +0.18 (shared inner row), +0.94 (adjacent outer).
    scene = np.vstack([make_row(-0.58), make_row(+0.18), make_row(+0.94)])
    est = converge(det, scene, iters=25)
    # keeps tracking the CENTRE strip (offset ≈ -0.20), does NOT hop to the
    # adjacent strip (which would read ≈ +0.56).
    assert -0.35 < est.lateral_offset < 0.02


def test_strip_lock_off_by_default_in_find_row_midpoint():
    """The shared function keeps spacing-only behaviour at prior_weight=0 (so the
    camera tracker and other callers are unchanged)."""
    from navigation.row_perception import find_row_midpoint
    # two equally-spaced pairs; with weight 0 the first (spacing-min) wins as before
    cross = np.concatenate([
        RNG.normal(-0.58, 0.02, 200), RNG.normal(0.18, 0.02, 200),
        RNG.normal(0.94, 0.02, 200)])
    lat0, _ = find_row_midpoint(cross, 1.2, 0.05, 0.76, prior_weight=0.0)
    lat_prior, _ = find_row_midpoint(cross, 1.2, 0.05, 0.76,
                                     prior_lateral=0.0, prior_weight=1.5)
    # with the prior, the centred strip (-0.20) is chosen over the adjacent (+0.56)
    assert lat_prior < 0.0


def test_strip_lock_weight_configurable_and_stronger_default():
    """The strip-lock prior is now constructor-configurable, and the default is
    stronger than the old 1.5 so an over-correction is less likely to hop."""
    assert RowDetector(dual_row=True).midpoint_prior_weight >= 2.5
    det = RowDetector(dual_row=True, row_spacing=0.76, midpoint_prior_weight=4.0)
    assert det.midpoint_prior_weight == 4.0


def test_row_end_waits_for_the_longer_line():
    """Soybean lines rarely end together.  When the LEFT line ends but the RIGHT
    line still runs (thinning but clearly present), row-end confidence must stay
    LOW — it is keyed off the STRONGER flanking side, not total density, so the
    robot keeps following the longer line to its true end."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    converge(det, np.vstack([make_row(-HALF), make_row(+HALF)]))
    # left line ended; right line present but thinning (~50 pts near its peak)
    est = converge(det, make_row(+HALF, n=50), iters=8)
    assert est.valid
    assert est.row_end_confidence < 0.15
    # the OLD total-density criterion would already be rising here — proving the
    # per-side rule is what keeps the robot on the remaining line.
    assert 1.0 - min(1.0, 50 / det.row_end_density) > 0.2


def test_row_end_high_once_both_lines_gone():
    """When BOTH flanking lines disappear, row-end confidence rises to ~1 so the
    navigator can trigger the headland turn."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    converge(det, np.vstack([make_row(-HALF), make_row(+HALF)]))
    est = det.update(np.empty((0, 3)))
    assert est.row_end_confidence > 0.9


def test_row_spacing_refines_toward_field_spacing():
    """The detector should REFINE the seed toward the field's actual spacing
    (within the anchored ±25% band) so the operator need not measure it exactly."""
    true_spacing = 0.88                     # actual field spacing (within +25% of seed)
    h = true_spacing / 2.0
    det = RowDetector(dual_row=True, row_spacing=0.76)   # standard-soybean seed
    assert det.spacing_estimate == 0.76
    pts = np.vstack([make_row(-h), make_row(+h)])
    converge(det, pts, iters=80)
    # The live estimate has moved off the seed toward the measured separation.
    assert det.spacing_estimate > 0.82
    assert det.spacing_estimate == pytest.approx(true_spacing, abs=0.06)


def test_spacing_estimate_stays_anchored_to_seed():
    """The estimate must NOT run away to an implausible value — a clutter pair
    far from the seed is rejected, keeping sp inside the ±25% band."""
    det = RowDetector(dual_row=True, row_spacing=0.76)   # band [0.57, 0.95]
    # Real rows at ±0.38 (sep 0.76) plus a clutter stripe far out that would form
    # a 1.10 m pair if it were ever accepted.
    pts = np.vstack([make_row(-0.38), make_row(+0.38), make_row(+0.72, n=40)])
    converge(det, pts, iters=60)
    assert 0.57 <= det.spacing_estimate <= 0.95


def test_spacing_estimate_survives_reset():
    """Calibration must persist across reset() (U-turn) — the field spacing
    does not change between rows."""
    det = RowDetector(dual_row=True, row_spacing=0.76)
    pts = np.vstack([make_row(-0.44), make_row(+0.44)])   # sep 0.88
    converge(det, pts, iters=80)
    learned = det.spacing_estimate
    assert learned > 0.82                  # has moved toward 0.88
    det.reset()
    assert det.spacing_estimate == learned  # unchanged by reset


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
# Terrain-adaptive crop band (sloped / undulating fields)
# ---------------------------------------------------------------------------

def make_row_graded(x_centre, grade_deg, n=130, y_lo=1.6, y_hi=6.8,
                    x_sigma=0.03, h_lo=0.08, h_hi=0.25):
    """Crop-row canopy on ground that ramps at grade_deg (rises with forward
    distance).  Ground passes through h=0 at the sensor (y=0), so canopy
    absolute height = h_local + tan(grade)*y."""
    y = RNG.uniform(y_lo, y_hi, n)
    x = x_centre + RNG.normal(0.0, x_sigma, n)
    h_local = RNG.uniform(h_lo, h_hi, n)
    z = (h_local - LIDAR_MOUNT_HEIGHT) + math.tan(math.radians(grade_deg)) * y
    return np.column_stack((x, y, z))


def make_ground_graded(grade_deg, n=400, y_lo=1.6, y_hi=6.8, roi_x=0.8):
    """Bare-soil returns on the same ramp (h_local ~ 0) — what lets the
    detector estimate and remove the grade."""
    y = RNG.uniform(y_lo, y_hi, n)
    x = RNG.uniform(-roi_x, roi_x, n)
    h_local = RNG.uniform(-0.02, 0.02, n)
    z = (h_local - LIDAR_MOUNT_HEIGHT) + math.tan(math.radians(grade_deg)) * y
    return np.column_stack((x, y, z))


def test_graded_field_crop_survives_with_detrend():
    """8° upslope: the soybean rows must still be detected (centred, confident)
    because the ground-slope detrend keeps the canopy inside the crop band."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([
        make_row_graded(-HALF, 8.0), make_row_graded(+HALF, 8.0),
        make_ground_graded(8.0),
    ])
    est = converge(det, pts)
    assert est.valid
    assert abs(est.lateral_offset) < 0.06
    assert est.confidence > 0.5
    assert abs(det.last_ground_slope - math.tan(math.radians(8.0))) < 0.05


def test_graded_field_crop_lost_without_detrend():
    """Same 8° slope with detrend disabled: the absolute band ejects the ramped
    canopy and the row is effectively lost — documents the field failure."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING, ground_detrend=False)
    pts = np.vstack([
        make_row_graded(-HALF, 8.0), make_row_graded(+HALF, 8.0),
        make_ground_graded(8.0),
    ])
    est = converge(det, pts)
    assert est.confidence < 0.35          # below the FOLLOW threshold


def test_flat_ground_detrend_is_noop():
    """On flat ground the detrend must do nothing (slope within dead-band)."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([make_row(-HALF), make_row(+HALF), make_ground_graded(0.0)])
    converge(det, pts)
    assert det.last_ground_slope == 0.0
    assert det.last_ground_roll == 0.0
    assert det.last_ground_shift == 0.0


def make_row_rolled(x_centre, roll_deg, n=130, y_lo=1.6, y_hi=6.8,
                    x_sigma=0.03, h_lo=0.08, h_hi=0.25):
    """Crop-row canopy on ground tilted about the FORWARD axis (cross-slope):
    absolute height rises with the cross-row coordinate x by tan(roll)*x."""
    y = RNG.uniform(y_lo, y_hi, n)
    x = x_centre + RNG.normal(0.0, x_sigma, n)
    h_local = RNG.uniform(h_lo, h_hi, n)
    z = (h_local - LIDAR_MOUNT_HEIGHT) + math.tan(math.radians(roll_deg)) * x
    return np.column_stack((x, y, z))


def make_ground_rolled(roll_deg, n=900, y_lo=1.6, y_hi=6.8, roi_x=2.0):
    """Bare-soil returns across a WIDE cross-row swath on a cross-slope, so the
    detector can estimate the lateral (roll) slope."""
    y = RNG.uniform(y_lo, y_hi, n)
    x = RNG.uniform(-roi_x, roi_x, n)
    h_local = RNG.uniform(-0.02, 0.02, n)
    z = (h_local - LIDAR_MOUNT_HEIGHT) + math.tan(math.radians(roll_deg)) * x
    return np.column_stack((x, y, z))


def test_cross_slope_roll_estimated_and_crop_survives():
    """10° cross-slope: the roll detrend must estimate dz/dx and keep BOTH
    flanking rows inside the crop band (they sit at different absolute heights
    on the tilt), so the midpoint stays centred and confident."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    pts = np.vstack([
        make_row_rolled(-HALF, 10.0), make_row_rolled(+HALF, 10.0),
        make_ground_rolled(10.0),
    ])
    est = converge(det, pts)
    assert est.valid
    assert abs(det.last_ground_roll - math.tan(math.radians(10.0))) < 0.05
    assert abs(est.lateral_offset) < 0.07
    assert est.confidence > 0.5


def test_cross_slope_ground_leak_removed_by_roll_detrend():
    """On a cross-slope the ground on the UP-slope side rises INTO the absolute
    crop band and is misread as crop (contaminating the row fit).  The roll
    detrend flattens the ground so that leak is removed: with the detrend the
    in-band count reflects real crop; without it the count is inflated by ground.
    Same cloud fed to both detectors so the only difference is the detrend."""
    pts = np.vstack([make_row_rolled(-HALF, 12.0), make_row_rolled(+HALF, 12.0),
                     make_ground_rolled(12.0)])
    det_on = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est_on = converge(det_on, pts)
    det_off = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                          ground_deadband_deg=45.0)   # suppresses both slope terms
    est_off = converge(det_off, pts)
    assert det_on.last_ground_roll != 0.0                   # detrend engaged
    assert det_off.last_ground_roll == 0.0                  # detrend suppressed
    # ground leaked into the band when the roll went uncorrected
    assert est_off.n_points > est_on.n_points + 40
    # and both still lock the (symmetric) strip centre
    assert abs(est_on.lateral_offset) < 0.07


def make_dipped_scene(drop=0.33):
    """Replica of the field stall: a terrain dip puts the whole ROI ~`drop` m
    below the flat-calibrated h=0.  Ground cluster at -drop, a furrow tail
    below it, and the two soybean rows as canopy ~0.08-0.30 m ABOVE the local
    ground (so absolute h ~ -0.25..-0.03, entirely below the 0.03 m crop
    floor)."""
    rng = np.random.default_rng(5)
    parts = []
    yg = rng.uniform(1.6, 6.8, 1500); xg = rng.uniform(-0.8, 0.8, 1500)
    parts.append(np.column_stack((xg, yg, (-drop + rng.normal(0, 0.04, 1500)) - LIDAR_MOUNT_HEIGHT)))
    yf = rng.uniform(1.6, 6.8, 400); xf = rng.uniform(-0.8, 0.8, 400)
    parts.append(np.column_stack((xf, yf, (-drop - 0.12 + rng.normal(0, 0.05, 400)) - LIDAR_MOUNT_HEIGHT)))
    for xc in (-HALF, +HALF):
        yc = rng.uniform(1.6, 6.8, 150); xx = xc + rng.normal(0, 0.03, 150)
        parts.append(np.column_stack((xx, yc, (-drop + rng.uniform(0.08, 0.30, 150)) - LIDAR_MOUNT_HEIGHT)))
    return np.vstack(parts)


def test_terrain_dip_crop_recovered_with_level_shift():
    """Ground ~0.35 m below h=0: the band must drop onto the local ground and
    recover the canopy (rows centred, confident)."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    est = converge(det, make_dipped_scene())
    assert est.valid
    assert abs(est.lateral_offset) < 0.07
    assert est.confidence > 0.5
    assert det.last_ground_shift < -0.20          # band was lowered onto ground


def test_terrain_dip_crop_lost_without_level_shift():
    """Same dip with ground_detrend disabled: the absolute band misses the
    sunk canopy — documents the field stall (n collapses, low confidence)."""
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING, ground_detrend=False)
    est = converge(det, make_dipped_scene())
    assert est.confidence < 0.35


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


def test_asymmetric_density_does_not_bias_heading():
    """4:1 LiDAR hit asymmetry between the two rows must not bias heading.

    The VLP-16 consistently illuminates the left soybean row with ~4× more
    returns than the right (azimuth coverage varies with lateral offset).
    Without equal-weight cluster normalisation, the left cluster dominates
    the pooled PCA, pulling the heading toward any local deviation of that
    row — producing a persistent +9° bias in the field.  With per-cluster
    weight normalisation both rows contribute equally regardless of density.
    """
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING)
    # Left row: 4× denser; right row: sparse.  Both are straight (heading 0°).
    pts = np.vstack([make_row(-HALF, n=520), make_row(+HALF, n=130)])
    est = converge(det, pts)
    assert abs(est.heading_error) < math.radians(3.0), (
        f"Asymmetric density biased heading to {math.degrees(est.heading_error):.1f}°"
    )
    assert abs(est.lateral_offset) < 0.05


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


# ---------------------------------------------------------------------------
# Heading–lateral consistency clamp: a large heading with a small, stable
# lateral offset is a terrain/PCA artifact, not a real orientation; capping it
# stops a heading-dominant pursuit from running away across the rows.
# Regression for the field log where heading ramped 0 -> -57 deg on a +8 deg
# grade while lateral stayed +/-0.05 m, and the robot turned ~45 deg and stuck.
# ---------------------------------------------------------------------------

def _smoothed(prev_hdg, prev_lat, fresh_hdg, fresh_lat, *, n=400, conf=0.8,
              clamp_lat=0.22):
    """Drive RowDetector._smooth directly with a known prev + fresh estimate.

    The heading–lateral consistency clamp is DISABLED by default in the detector
    (see RowDetector.__init__); these tests validate its math, so they enable it
    explicitly via clamp_lat."""
    from navigation.row_perception import RowEstimate
    det = RowDetector(dual_row=True)
    det.heading_consistency_lat = clamp_lat            # enable the clamp for the test
    det._est = RowEstimate(heading_error=prev_hdg, lateral_offset=prev_lat,
                           confidence=conf, n_points=n, valid=True)
    fresh = RowEstimate(heading_error=fresh_hdg, lateral_offset=fresh_lat,
                        confidence=conf, n_points=n, valid=True)
    return det._smooth(fresh), det


def test_heading_clamp_enabled_by_default_but_inert_in_normal_following():
    """Clamp is ON by default (grade-runaway safety net) but INERT in normal
    following — a small heading with a small lateral passes through unchanged, so
    it does not shift the RL policy's input off-slope."""
    from navigation.row_perception import RowEstimate
    det = RowDetector(dual_row=True)
    assert det.heading_consistency_lat > 0.0
    # normal following: small heading, small lateral -> untouched
    det._est = RowEstimate(heading_error=math.radians(3), lateral_offset=0.02,
                           confidence=0.8, n_points=400, valid=True)
    est = det._smooth(RowEstimate(heading_error=math.radians(3), lateral_offset=0.02,
                                  confidence=0.8, n_points=400, valid=True))
    assert abs(est.heading_error) == pytest.approx(math.radians(3), abs=1e-6)
    # pathological grade spike: big heading while centred -> clamped hard
    det._est = RowEstimate(heading_error=math.radians(-38), lateral_offset=0.0,
                           confidence=0.8, n_points=400, valid=True)
    est2 = det._smooth(RowEstimate(heading_error=math.radians(-38), lateral_offset=0.0,
                                   confidence=0.8, n_points=400, valid=True))
    assert abs(est2.heading_error) <= det.heading_cap_centred + 1e-9


def test_large_heading_when_centred_is_clamped_hard():
    # perfectly centred (lateral ~0) but heading huge -> clamped near the
    # centred cap (~7 deg), NOT left at 40+ deg to drive a turn on a straight row
    est, det = _smoothed(math.radians(-40), 0.0, math.radians(-51), 0.0)
    assert abs(est.lateral_offset) < 0.02
    assert abs(est.heading_error) <= det.heading_cap_centred + 1e-9
    assert est.heading_error < 0                      # sign preserved


def test_cap_scales_with_lateral_offset():
    # more off-strip -> a larger heading is allowed (needed to steer back)
    centred, det = _smoothed(math.radians(-40), 0.0, math.radians(-40), 0.0)
    partway, _ = _smoothed(math.radians(-40), 0.15, math.radians(-40), 0.15)
    assert abs(centred.heading_error) < abs(partway.heading_error)
    assert abs(centred.heading_error) <= det.heading_cap_centred + 1e-9
    assert abs(partway.heading_error) <= det.heading_cap_gate + 1e-9


def test_large_heading_large_lateral_is_not_clamped():
    # genuinely off the strip AND angled -> real, must NOT be clamped
    est, det = _smoothed(math.radians(-40), -0.5, math.radians(-45), -0.5)
    assert abs(est.heading_error) > det.heading_cap_gate


def test_normal_small_heading_unchanged_by_clamp():
    est, det = _smoothed(math.radians(3), 0.02, math.radians(4), 0.03)
    # a few degrees is below even the centred cap -> plain EMA blend, untouched
    assert math.radians(2) < abs(est.heading_error) < math.radians(5)


# ---------------------------------------------------------------------------
# Dense / tall-canopy adaptive fit — when the canopy closes over and the
# ground/furrow is no longer visible, weight the fit by canopy-height
# prominence so the taller rows dominate and the residue strip (a height
# VALLEY) is recovered.  Early growth stays on the plain density fit.
# ---------------------------------------------------------------------------

def _canopy_row(x_centre, n, h_lo, h_hi, sigma, y_lo=1.6, y_hi=6.6):
    rng = np.random.default_rng(int(abs(x_centre * 1000)) + n)
    y = rng.uniform(y_lo, y_hi, n)
    x = x_centre + rng.normal(0.0, sigma, n)
    h = rng.uniform(h_lo, h_hi, n)
    return np.column_stack((x, y, h - LIDAR_MOUNT_HEIGHT))


def make_dense_canopy_scene(offset):
    """Tall closed-canopy centre-residue scene.  ``offset`` = robot metres RIGHT
    of the strip centre → the strip appears at x=-offset, the two crop rows at
    -offset ± HALF.  The rows are tall ridges (canopy 0.45–0.78 m); the residue
    strip is a LOW valley; and dense low leaf/weed clutter fills the inter-row
    space (biased left) so the plain DENSITY midpoint is dragged off — the
    field failure this fixes."""
    c = -offset
    rng = np.random.default_rng(99)
    parts = [
        _canopy_row(c - HALF, 340, 0.45, 0.78, 0.08),   # tall left crop row
        _canopy_row(c + HALF, 340, 0.45, 0.78, 0.08),   # tall right crop row
        _canopy_row(c,        200, 0.05, 0.22, 0.12),   # low residue strip (valley)
    ]
    xg = rng.uniform(-0.78, 0.40, 420)                  # low clutter, biased left
    yg = rng.uniform(1.6, 6.6, 420)
    hg = rng.uniform(0.08, 0.30, 420)
    parts.append(np.column_stack((xg, yg, hg - LIDAR_MOUNT_HEIGHT)))
    return np.vstack(parts)


def test_dense_canopy_regime_engages_on_tall_crop():
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                      crop_h_min=0.03, crop_h_max=1.0)
    converge(det, make_dense_canopy_scene(0.0))
    assert det.last_dense is True                 # closed canopy → weighted fit engaged
    assert det.last_tall_frac >= det.dense_canopy_frac


def test_dense_canopy_recovers_offset_where_plain_fails():
    """Robot 0.20 m right of the strip: the height-weighted fit recovers
    lateral≈-0.20, while the plain density fit is dragged toward the low clutter."""
    pts = make_dense_canopy_scene(0.20)
    dense = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                        crop_h_min=0.03, crop_h_max=1.0, dense_canopy=True)
    plain = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                        crop_h_min=0.03, crop_h_max=1.0, dense_canopy=False)
    ed = converge(dense, pts)
    ep = converge(plain, pts)
    assert ed.lateral_offset == pytest.approx(-0.20, abs=0.06)     # accurate
    # the plain fit is off by > 0.15 m (the weave / row-hop error the fix removes)
    assert abs(ep.lateral_offset - (-0.20)) > 0.15
    assert abs(ed.lateral_offset - (-0.20)) < abs(ep.lateral_offset - (-0.20))


def test_early_growth_untouched_by_dense_canopy():
    """Short seedlings (all returns well below canopy_tall_h): the dense regime
    never engages and the fit is byte-identical to dense_canopy=False."""
    pts = np.vstack([make_row(-HALF - 0.20), make_row(+HALF - 0.20)])   # h 0.08–0.25
    on = RowDetector(dual_row=True, row_spacing=ROW_SPACING, dense_canopy=True)
    off = RowDetector(dual_row=True, row_spacing=ROW_SPACING, dense_canopy=False)
    e_on = converge(on, pts)
    e_off = converge(off, pts)
    assert on.last_dense is False                          # never triggered
    assert e_on.lateral_offset == pytest.approx(e_off.lateral_offset, abs=1e-9)
    assert e_on.heading_error == pytest.approx(e_off.heading_error, abs=1e-9)


def test_dense_canopy_can_be_disabled():
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                      crop_h_min=0.03, crop_h_max=1.0, dense_canopy=False)
    converge(det, make_dense_canopy_scene(0.0))
    assert det.last_dense is False


def test_dense_canopy_unbiased_by_density_asymmetry():
    """One canopy side thicker (more returns) than the other, robot centred:
    the height-weighted fit keys off canopy HEIGHT not point density, so it
    stays centred where a density fit drifts toward the thicker side
    (PDF failure mode: 'lateral offset becomes biased if one side is thicker')."""
    rng = np.random.default_rng(1)
    def row(xc, n, seed):
        r = np.random.default_rng(seed)
        y = r.uniform(1.6, 6.6, n); x = xc + r.normal(0, 0.09, n); h = r.uniform(0.45, 0.78, n)
        return np.column_stack((x, y, h - LIDAR_MOUNT_HEIGHT))
    pts = np.vstack([
        row(-HALF, 200, 1),                                   # left row: sparse
        row(+HALF, 500, 2),                                   # right row: 2.5x denser
        row(0.0, 180, 3) - np.array([0, 0, 0.55]),            # low residue strip
    ])
    det = RowDetector(dual_row=True, row_spacing=ROW_SPACING,
                      crop_h_min=0.03, crop_h_max=1.0, dense_canopy=True)
    est = converge(det, pts)
    assert det.last_dense is True
    assert abs(est.lateral_offset) < 0.04            # not dragged toward the thicker row


def test_dense_canopy_recovers_heading_where_plain_collapses():
    """A truly tilted (+6°) dense canopy with low leaf clutter: the plain PCA
    heading goes blob-like and collapses toward 0 (PDF failure mode 'heading
    estimate becomes noisier'); the canopy-height-weighted fit recovers it."""
    th = math.radians(6.0)
    def row(xc, n, seed):
        r = np.random.default_rng(seed)
        y = r.uniform(1.6, 6.6, n); x = xc + r.normal(0, 0.08, n) + y * math.tan(th)
        h = r.uniform(0.45, 0.78, n)
        return np.column_stack((x, y, h - LIDAR_MOUNT_HEIGHT))
    rng = np.random.default_rng(9)
    clutter = np.column_stack((rng.uniform(-0.78, 0.78, 400), rng.uniform(1.6, 6.6, 400),
                               rng.uniform(0.08, 0.30, 400) - LIDAR_MOUNT_HEIGHT))
    pts = np.vstack([row(-HALF, 340, 4), row(+HALF, 340, 5),
                     row(0.0, 200, 6) - np.array([0, 0, 0.55]), clutter])
    dense = converge(RowDetector(dual_row=True, row_spacing=ROW_SPACING, crop_h_min=0.03,
                                 crop_h_max=1.0, dense_canopy=True), pts)
    plain = converge(RowDetector(dual_row=True, row_spacing=ROW_SPACING, crop_h_min=0.03,
                                 crop_h_max=1.0, dense_canopy=False), pts)
    assert math.degrees(dense.heading_error) > 3.0                       # right sign, substantial
    assert abs(math.degrees(dense.heading_error) - 6.0) < abs(math.degrees(plain.heading_error) - 6.0)
