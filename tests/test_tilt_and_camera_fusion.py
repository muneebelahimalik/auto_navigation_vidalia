"""Unit tests for lidar/obstacle_filter.py tilt correction and the
camera/soybean_row_tracker.py dual-camera fusion geometry (the _fuse path is
deliberately OpenCV-free so it can be tested without cv2)."""
import math

import numpy as np
import pytest

from camera.soybean_row_tracker import DualCameraRowTracker, _SideResult
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT, tilt_correct_pts, yaw_correct_pts

TILT = math.radians(15.0)


# ---------------------------------------------------------------------------
# LiDAR yaw correction
# ---------------------------------------------------------------------------

def test_yaw_correct_front_bucket():
    """Front bucket at (x=-1.862, y=0.683) in sensor frame with 71° yaw → (x≈0, y≈2)."""
    yaw = math.radians(71.0)
    pts = np.array([[-1.862, 0.683, 0.0]])
    out = yaw_correct_pts(pts, yaw)
    assert out[0, 0] == pytest.approx(0.0, abs=0.05)
    assert out[0, 1] == pytest.approx(2.0, abs=0.05)
    assert out[0, 2] == 0.0   # Z untouched


def test_yaw_correct_right_bucket():
    """Right bucket at (x=0.630, y=1.917) in sensor frame with 71° yaw → (x≈2, y≈0)."""
    yaw = math.radians(71.0)
    pts = np.array([[0.630, 1.917, 0.0]])
    out = yaw_correct_pts(pts, yaw)
    assert out[0, 0] == pytest.approx(2.0, abs=0.05)
    assert out[0, 1] == pytest.approx(0.0, abs=0.05)


def test_yaw_zero_is_noop():
    """yaw_rad=0 must return the original array unchanged."""
    pts = np.array([[1.0, 2.0, 3.0]])
    out = yaw_correct_pts(pts, 0.0)
    assert out is pts   # identity (no copy made)


def test_yaw_correct_round_trip():
    """Applying +yaw then -yaw must recover the original point."""
    yaw = math.radians(71.0)
    pts = np.array([[-1.5, 1.2, 0.5]])
    out = yaw_correct_pts(yaw_correct_pts(pts, yaw), -yaw)
    np.testing.assert_allclose(out, pts, atol=1e-9)


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
# Dual-camera fusion (pure logic, no cv2 required)
#
# With both cameras FORWARD-FACING each camera independently estimates the
# SAME residue-strip centre; _fuse averages the two estimates and scales
# confidence by their agreement.
# ---------------------------------------------------------------------------

def side(world_x, heading=0.0, strength=500.0, green=0.2, quality=1.0):
    return _SideResult(valid=True, world_x=world_x, heading=heading,
                       green_fraction=green, strength=strength, quality=quality)


def fuse_converged(tracker, left, right, iters=30):
    est = None
    for _ in range(iters):
        est = tracker._fuse(left, right)
    return est


def test_agreeing_cameras_average_and_boost():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    est = fuse_converged(tracker, side(+0.04), side(-0.04))
    assert est.lateral_offset == pytest.approx(0.0, abs=0.02)
    assert 0.40 <= est.confidence <= 0.50


def test_disagreeing_cameras_lower_confidence():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    agree = fuse_converged(tracker, side(+0.02), side(-0.02)).confidence
    tracker.reset()
    disagree = fuse_converged(tracker, side(+0.15), side(-0.15)).confidence
    assert disagree < agree


def test_single_camera_still_full_estimate():
    """A forward camera sees BOTH rows, so one camera alone still produces a
    complete centre estimate — just at reduced confidence."""
    tracker = DualCameraRowTracker(row_spacing=0.76)
    est = fuse_converged(tracker, side(+0.10), _SideResult())
    assert est.lateral_offset == pytest.approx(0.10, abs=0.02)
    assert est.confidence < 0.30                 # degraded, not dead


def test_no_rows_decays_confidence():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    fuse_converged(tracker, side(+0.02), side(-0.02))
    conf = tracker._est.confidence
    for _ in range(4):
        est = tracker._fuse(_SideResult(), _SideResult())
    assert est.confidence < conf * 0.5
    assert not est.left_valid and not est.right_valid


def test_low_quality_sides_lower_confidence():
    tracker = DualCameraRowTracker(row_spacing=0.76)
    good = fuse_converged(tracker, side(+0.02, quality=1.0),
                          side(-0.02, quality=1.0)).confidence
    tracker.reset()
    poor = fuse_converged(tracker, side(+0.02, quality=0.2),
                          side(-0.02, quality=0.2)).confidence
    assert poor < good


# ---------------------------------------------------------------------------
# Ground-plane projection geometry (numpy only — exercises _side_from_mask)
#
# Synthetic scenes are FORWARD-projected through the same pinhole +
# extrinsics model the tracker inverts, so these tests close the loop on the
# full image -> robot-frame geometry chain.
# ---------------------------------------------------------------------------

from camera.soybean_row_tracker import _R_OPT2BODY, _rx  # noqa: E402

W, H = 640, 400
FX = FY = 452.0
CX, CY = 320.0, 200.0
CAM_Z = 0.92
CAM_Y_FWD = -0.465
PITCH = math.radians(15.0)


def make_tracker(**kw):
    args = dict(cam_x_left=-0.88, cam_x_right=0.88, row_spacing=0.76,
                cam_y_fwd=CAM_Y_FWD, cam_z=CAM_Z, cam_pitch_deg=15.0,
                canopy_z=0.10, min_green_fraction=0.0, pixel_step=1)
    args.update(kw)
    return DualCameraRowTracker(**args)


def render_mask(world_pts, cam_x):
    """Forward-project robot-frame points into a camera and rasterise a mask."""
    R = _rx(-PITCH) @ _R_OPT2BODY
    t = np.array([cam_x, CAM_Y_FWD, CAM_Z])
    p_cam = (world_pts - t) @ R          # = R.T @ (p - t) row-wise
    vis = p_cam[:, 2] > 0.1
    p_cam = p_cam[vis]
    u = (FX * p_cam[:, 0] / p_cam[:, 2] + CX).round().astype(int)
    v = (FY * p_cam[:, 1] / p_cam[:, 2] + CY).round().astype(int)
    mask = np.zeros((H, W), dtype=bool)
    ok = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    for du in (-1, 0, 1):                # dilate so thin lines survive
        for dv in (-1, 0, 1):
            uu = np.clip(u[ok] + du, 0, W - 1)
            vv = np.clip(v[ok] + dv, 0, H - 1)
            mask[vv, uu] = True
    return mask


def row_points(x_centre, z=0.10, y_lo=1.0, y_hi=4.5, n=900):
    rng = np.random.default_rng(7)
    y = np.linspace(y_lo, y_hi, n)
    x = x_centre + rng.normal(0.0, 0.02, n)
    return np.column_stack((x, y, np.full(n, z)))


def test_ground_projection_recovers_centred_rows():
    """Two rows at ±0.38 m, robot centred: each camera should independently
    report the residue centre at ~0 with near-zero heading."""
    tracker = make_tracker()
    world = np.vstack([row_points(-0.38), row_points(+0.38)])
    for cam_x in (-0.88, +0.88):
        res = tracker._side_from_mask(render_mask(world, cam_x), cam_x)
        assert res.valid
        assert res.world_x == pytest.approx(0.0, abs=0.08)
        assert abs(res.heading) < math.radians(4.0)


def test_ground_projection_offset_robot():
    """Robot 0.15 m right of centre -> rows appear at -0.53/+0.23 ->
    centre estimate ~ -0.15."""
    tracker = make_tracker()
    world = np.vstack([row_points(-0.38 - 0.15), row_points(+0.38 - 0.15)])
    res = tracker._side_from_mask(render_mask(world, -0.88), -0.88)
    assert res.valid
    assert res.world_x == pytest.approx(-0.15, abs=0.08)


def test_one_row_occluded_half_spacing_fallback():
    """Only the left row visible to a camera: find_row_midpoint's fallback
    puts the centre half a spacing to its right (= true centre here)."""
    tracker = make_tracker()
    world = row_points(-0.38)
    res = tracker._side_from_mask(render_mask(world, -0.88), -0.88)
    assert res.valid
    assert res.world_x == pytest.approx(0.0, abs=0.09)


def test_canopy_height_bias_cancels_in_two_camera_mean():
    """Green at z=0.30 m projected with canopy_z=0.10 m biases each camera's
    midpoint AWAY from that camera (~0.88*(k-1) m); the biases are opposite
    and cancel in the two-camera mean — the core reason fusion is an
    equal-weight average."""
    tracker = make_tracker()
    world = np.vstack([row_points(-0.38, z=0.30, y_hi=3.0),
                       row_points(+0.38, z=0.30, y_hi=3.0)])
    left = tracker._side_from_mask(render_mask(world, -0.88), -0.88)
    right = tracker._side_from_mask(render_mask(world, +0.88), +0.88)
    assert left.valid and right.valid
    assert left.world_x > 0.15           # pushed right, away from left camera
    assert right.world_x < -0.15         # pushed left, away from right camera
    fused = 0.5 * (left.world_x + right.world_x)
    assert fused == pytest.approx(0.0, abs=0.08)


def test_ground_points_exposed_for_point_fusion():
    """A valid side fit must publish its metric points via _last_side_pts —
    the feed for RowDetector point-level fusion (--cam-fusion point)."""
    tracker = make_tracker()
    world = np.vstack([row_points(-0.38), row_points(+0.38)])
    tracker._last_side_pts = None
    res = tracker._side_from_mask(render_mask(world, -0.88), -0.88)
    assert res.valid
    P = tracker._last_side_pts
    assert P is not None and P.ndim == 2 and P.shape[1] == 2
    assert len(P) >= 100
    # Points must be inside the tracker ROI, in the robot frame.
    assert np.all(np.abs(P[:, 0]) <= tracker.roi_x_half + 1e-9)
    assert np.all((P[:, 1] >= tracker.roi_y_min) & (P[:, 1] <= tracker.roi_y_max))
    # And cluster around the true row positions.
    assert np.abs(np.abs(P[:, 0]) - 0.38).mean() < 0.10


def test_invalid_side_publishes_no_points():
    tracker = make_tracker()
    tracker._last_side_pts = None
    res = tracker._side_from_mask(np.zeros((H, W), dtype=bool), -0.88)
    assert not res.valid
    assert tracker._last_side_pts is None


def test_depth_gate_rejects_tall_object():
    """Green pixels whose stereo depth is much shorter than the ground-ray
    distance belong to a tall object, not crop — they must be discarded."""
    tracker = make_tracker()
    # An elevated green blob (e.g. a person's clothing at 0.9 m height).
    world = row_points(0.0, z=0.85, y_lo=1.5, y_hi=2.0)
    mask = render_mask(world, -0.88)
    # Stereo reports the TRUE (short) range; ground ray expects much farther.
    depth = np.full((H, W), 1200, dtype=np.uint16)     # 1.2 m everywhere
    res = tracker._side_from_mask(mask, -0.88, depth)
    assert not res.valid                                # gated out entirely
