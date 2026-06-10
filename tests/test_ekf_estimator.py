"""Unit tests for navigation/ekf_estimator.py — motion-model signs and gating.

The motion-model sign tests encode the perception frame convention
(X=right, Y=forward; heading_error +ve = row angled to robot's right;
lateral_offset +ve = row centre to the right; angular +ve = CCW/left):

  * Driving forward with the row angled to the right (theta > 0) increases
    the lateral offset:  x_row(t) = x0 + v*t*tan(theta).
  * Turning left (omega > 0) rotates the row direction further to the
    robot's right, increasing heading_error.

Before the fix both derivatives had flipped signs, so the predict step
actively pushed the state away from where the robot was actually going —
matching the field observation that the EKF "destabilises the loop".
"""
import math
from types import SimpleNamespace

import pytest

from navigation.ekf_estimator import RowEKF
from navigation.row_perception import RowEstimate


def lidar_meas(lat=0.0, hdg=0.0, conf=0.9):
    return RowEstimate(heading_error=hdg, lateral_offset=lat,
                       confidence=conf, n_points=150, valid=True)


def cam_meas(lat=0.0, hdg=0.0, conf=0.4):
    return SimpleNamespace(lateral_offset=lat, heading_error=hdg, confidence=conf)


def seeded_ekf(lat=0.0, hdg=0.0):
    ekf = RowEKF()
    for _ in range(3):
        ekf.update_lidar(lidar_meas(lat=lat, hdg=hdg))
    return ekf


# ---------------------------------------------------------------------------
# Motion-model signs
# ---------------------------------------------------------------------------

def test_predict_forward_drift_sign():
    """theta > 0 (row to the right) + forward motion => lateral offset grows."""
    ekf = seeded_ekf(lat=0.0, hdg=0.20)
    lat_before = ekf.lateral_offset
    ekf.predict(linear_vel=0.30, angular_vel=0.0, dt=0.1)
    assert ekf.lateral_offset > lat_before
    assert ekf.lateral_offset == pytest.approx(
        lat_before + 0.30 * math.sin(0.20) * 0.1, abs=1e-9)


def test_predict_turn_sign_closes_the_loop():
    """The controller commands a RIGHT turn (omega < 0) for theta > 0; the
    predicted heading must move TOWARD zero, not away from it."""
    ekf = seeded_ekf(lat=0.0, hdg=0.20)
    ekf.predict(linear_vel=0.0, angular_vel=-0.35, dt=0.1)
    assert ekf.heading_error == pytest.approx(0.20 - 0.035, abs=1e-9)


def test_predict_left_turn_increases_heading():
    ekf = seeded_ekf(hdg=0.0)
    ekf.predict(linear_vel=0.0, angular_vel=+0.30, dt=0.1)
    assert ekf.heading_error == pytest.approx(+0.03, abs=1e-9)


def test_closed_loop_prediction_consistency():
    """Simulated pursuit: state (lat=0.2, hdg=0) with a right-turn command
    should predict the lateral offset shrinking over time once the robot
    has yawed right (theta < 0)."""
    ekf = seeded_ekf(lat=0.20, hdg=0.0)
    for _ in range(10):                      # 1 s of turning right while rolling
        ekf.predict(linear_vel=0.25, angular_vel=-0.20, dt=0.1)
    assert ekf.heading_error < 0.0           # yawed right
    assert ekf.lateral_offset < 0.20         # row drifting back toward centre


# ---------------------------------------------------------------------------
# Convergence and measurement gating
# ---------------------------------------------------------------------------

def test_convergence_after_two_measurements():
    ekf = RowEKF()
    assert not ekf.converged
    ekf.update_lidar(lidar_meas(lat=0.1, conf=0.8))
    ekf.update_lidar(lidar_meas(lat=0.1, conf=0.8))
    assert ekf.converged
    assert ekf.std_lateral < 0.08


def test_invalid_or_decayed_estimates_are_ignored():
    ekf = RowEKF()
    phantom = RowEstimate(lateral_offset=0.5, confidence=0.4, valid=False)
    ekf.update_lidar(phantom)
    assert not ekf.converged
    assert ekf.lateral_offset == 0.0


def test_camera_lateral_outlier_rejected():
    ekf = seeded_ekf(lat=0.0, hdg=0.0)
    ekf.update_camera(cam_meas(lat=1.0, conf=0.5))    # a full metre off
    assert abs(ekf.lateral_offset) < 0.02


def test_camera_heading_outlier_rejected():
    """A PCA flip on a round green blob can report ~90 deg heading; the
    heading gate must reject it instead of yanking the fused heading."""
    ekf = seeded_ekf(lat=0.0, hdg=0.0)
    ekf.update_camera(cam_meas(lat=0.0, hdg=math.radians(80), conf=0.5))
    assert abs(ekf.heading_error) < math.radians(3)


def test_camera_inlier_accepted():
    ekf = seeded_ekf(lat=0.0, hdg=0.0)
    before = ekf.lateral_offset
    ekf.update_camera(cam_meas(lat=0.05, conf=0.5))
    assert ekf.lateral_offset > before          # nudged toward the camera
    assert ekf.lateral_offset < 0.05            # but not all the way


def test_reset():
    ekf = seeded_ekf(lat=0.3, hdg=0.1)
    ekf.reset()
    assert not ekf.converged
    assert ekf.lateral_offset == 0.0
    assert ekf.heading_error == 0.0
