"""Unit tests for navigation/row_safety.py — three-zone obstacle monitor."""
import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
from navigation.row_safety import SafetyMonitor


def cluster(x, y, h, n=6):
    """n points at (x, y) with ground-relative height h."""
    pts = np.tile([x, y, h - LIDAR_MOUNT_HEIGHT], (n, 1))
    pts[:, 0] += np.linspace(-0.05, 0.05, n)
    return pts


def soybean_defaults():
    return SafetyMonitor(obstacle_height=0.50, tire_obstacle_height=0.35,
                         forward_half_width=0.60, tire_track=0.965)


def test_empty_scan_is_clear():
    assert not soybean_defaults().check(np.empty((0, 3))).blocked


def test_person_in_forward_zone_blocks():
    s = soybean_defaults().check(cluster(0.0, 1.8, h=1.2))
    assert s.forward_blocked
    assert s.blocked
    assert s.nearest_forward < 2.0


def test_soybean_canopy_passes_under():
    """Seedlings (h <= 0.30 m) in the path must NOT trip the forward zone."""
    s = soybean_defaults().check(cluster(0.0, 1.8, h=0.25, n=40))
    assert not s.blocked


def test_left_tire_zone():
    s = soybean_defaults().check(cluster(-0.965, 1.5, h=0.8))
    assert s.left_tire_blocked and not s.right_tire_blocked


def test_right_tire_zone():
    s = soybean_defaults().check(cluster(+0.965, 1.5, h=0.8))
    assert s.right_tire_blocked and not s.left_tire_blocked


def test_min_points_debounce():
    """Fewer than forward_min_points returns must not trigger a stop."""
    s = soybean_defaults().check(cluster(0.0, 1.8, h=1.2, n=3))
    assert not s.forward_blocked


def test_beyond_horizon_ignored():
    s = soybean_defaults().check(cluster(0.0, 4.0, h=1.2))
    assert not s.blocked


def test_outside_corridor_ignored():
    """A tall object outside both the body corridor and tire tracks passes."""
    s = soybean_defaults().check(cluster(1.6, 1.8, h=1.2))
    assert not s.blocked


def test_onion_thresholds_pass_adjacent_canopy():
    """Onion config: adjacent-row canopy at h~0.80 m under the 0.85 m tire
    threshold must not block (the historical L-TIRE false positive)."""
    onion = SafetyMonitor(obstacle_height=0.75, tire_obstacle_height=0.85,
                          forward_half_width=0.60, tire_track=0.965)
    s = onion.check(cluster(-0.965, 1.5, h=0.80, n=47))
    assert not s.blocked


def test_default_forward_half_width_is_narrow():
    """Default forward_half_width must be 0.60 m (field-safe narrow corridor)
    so that adjacent crop-row canopy at ±0.65 m does not false-alarm."""
    mon = SafetyMonitor()
    assert mon.forward_half_width == 0.60, (
        f"Expected 0.60 m, got {mon.forward_half_width} m — "
        "wide default causes false obstacles from adjacent crop rows"
    )


def test_default_forward_half_width_does_not_false_alarm_in_forward_zone():
    """Returns at x=0.65 (outside the 0.60 m forward corridor) at obstacle
    height must not trigger the FORWARD zone — confirming the narrowed default
    confines the forward check to the robot's true body width."""
    mon = SafetyMonitor()
    # h=0.76 just above obstacle_height=0.75; x=0.65 > forward_half_width=0.60
    # so the forward zone must not fire.  (Tire zone is separate and expected
    # to fire if the return is in the tire corridor — that is correct behaviour.)
    tall_outside_body = np.array([[0.65, 1.5, 0.76 - LIDAR_MOUNT_HEIGHT]])
    status = mon.check(np.tile(tall_outside_body, (10, 1)))
    assert not status.forward_blocked, (
        "Returns outside the 0.60 m body corridor must not trip the forward zone"
    )
