"""Unit tests for navigation/headland.py and navigation/odometry.py.

The headland U-turn is closed-loop on odometry, so the two are tested
together: a kinematic integrator stands in for the real wheel telemetry.
"""
import math

import pytest

from navigation.headland import HeadlandTurn
from navigation.odometry import WheelOdometry


class FakeOdo:
    """Minimal odometry stand-in: integrates the commanded twist exactly."""
    def __init__(self):
        self.distance = 0.0
        self.theta = 0.0
        self.x = 0.0
        self.y = 0.0

    def apply(self, v, w, dt):
        self.distance += abs(v) * dt
        theta_mid = self.theta + 0.5 * w * dt
        self.theta += w * dt
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt


def run_turn(turn_sign, row_spacing=0.76, exit_dist=1.0):
    odo = FakeOdo()
    turn = HeadlandTurn(odo, row_spacing=row_spacing, exit_dist=exit_dist,
                        speed=0.15, turn_rate=0.35)
    turn.begin(turn_sign)
    phases = [turn.phase]
    dt = 0.05
    for _ in range(20000):
        v, w = turn.step(dt)
        if turn.done:
            break
        odo.apply(v, w, dt)
        if turn.phase != phases[-1]:
            phases.append(turn.phase)
    return odo, turn, phases


def test_right_uturn_phase_sequence_and_geometry():
    odo, turn, phases = run_turn(+1.0, row_spacing=1.52)
    assert turn.done
    assert phases[:2] == ["EXIT", "ARC"]
    # A 180-deg clockwise arc => net heading ~ -pi (right U-turn).
    assert odo.theta == pytest.approx(-math.pi, abs=0.1)
    # Distance = exit + arc length (pi * radius), radius = shift/2 = 0.76.
    expected = 1.0 + math.pi * 0.76
    assert odo.distance == pytest.approx(expected, abs=0.05)


def test_left_uturn_mirrors_heading():
    odo, turn, _ = run_turn(-1.0)
    assert turn.done
    assert odo.theta == pytest.approx(+math.pi, abs=0.1)


def test_arc_command_is_a_moving_turn_not_a_pivot():
    """The turn must ARC (linear > 0), not pivot in place — an in-place pivot
    scrubs the wheels and the wheel heading over-reads, which is the field bug
    where the U-turn only reached ~90°."""
    odo = FakeOdo()
    turn = HeadlandTurn(odo, row_spacing=1.52, exit_dist=0.0)  # skip EXIT
    turn.begin(+1.0)
    v, w = turn.step(0.05)
    assert turn.phase == "ARC"
    assert v > 0.0                            # moving arc, not a pivot
    assert w < 0.0                            # right turn = clockwise = negative
    # Radius defaults to shift/2; forward speed = radius * turn_rate.
    assert v == pytest.approx(0.76 * 0.35, abs=1e-9)


def test_arc_radius_is_half_the_shift():
    """The default arc is the maximum-radius semicircle that lands on the next
    strip: lateral displacement over the 180° arc equals the shift distance."""
    odo, turn, _ = run_turn(+1.0, row_spacing=1.52, exit_dist=0.0)
    assert turn.turn_radius == pytest.approx(0.76)
    # Initial heading is +x, so the lateral displacement of a 180° arc is along
    # y and equals 2 * radius = shift; net x returns to ~0 (semicircle).
    assert abs(odo.y) == pytest.approx(1.52, abs=0.05)
    assert abs(odo.x) == pytest.approx(0.0, abs=0.05)


def test_explicit_turn_radius_overrides_default():
    odo = FakeOdo()
    turn = HeadlandTurn(odo, row_spacing=1.52, turn_radius=0.5, exit_dist=0.0)
    assert turn.turn_radius == 0.5


# ---------------------------------------------------------------------------
# Filter (IMU/GPS) heading source for the pivots
# ---------------------------------------------------------------------------

class FakeFilter:
    """Absolute-heading stand-in with the FilterHeading interface."""
    def __init__(self, usable=True):
        self.heading = 0.0
        self._usable = usable

    @property
    def usable(self):
        return self._usable


def run_turn_with_filter(usable=True, wheel_slip=0.7):
    """Drive a turn where the filter heading is ground truth and the wheel
    heading slips (scaled by wheel_slip), so the two diverge during pivots."""
    odo = FakeOdo()
    filt = FakeFilter(usable=usable)
    turn = HeadlandTurn(odo, row_spacing=1.52, exit_dist=1.0,
                        speed=0.15, turn_rate=0.35, heading_source=filt)
    turn.begin(+1.0)
    dt = 0.05
    for _ in range(20000):
        v, w = turn.step(dt)
        if turn.done:
            break
        odo.distance += abs(v) * dt
        odo.theta += w * dt * wheel_slip      # wheel heading under-reads (slip)
        filt.heading += w * dt                # filter = true heading
    return odo, turn, filt


def test_filter_heading_used_when_usable():
    """With a usable filter the pivots close on the (true) filter heading, so
    the U-turn reaches a correct ~180° even when the wheel heading slipped."""
    odo, turn, filt = run_turn_with_filter(usable=True, wheel_slip=0.7)
    assert turn.done
    assert turn.heading_source_name == "filter"
    assert filt.heading == pytest.approx(-math.pi, abs=0.15)   # true U-turn
    # The slipping wheel heading is well short — the bug the filter fixes.
    assert abs(odo.theta) < 0.85 * math.pi


def test_filter_falls_back_to_wheel_when_not_converged():
    """A non-usable filter must not be used; pivots fall back to wheel heading."""
    odo, turn, _ = run_turn_with_filter(usable=False, wheel_slip=1.0)
    assert turn.done
    assert turn.heading_source_name == "wheel"
    assert odo.theta == pytest.approx(-math.pi, abs=0.15)


def test_heading_source_latched_at_begin():
    """The source is latched at begin(): a filter that becomes usable only
    after the turn starts is not adopted mid-turn."""
    odo = FakeOdo()
    filt = FakeFilter(usable=False)
    turn = HeadlandTurn(odo, heading_source=filt)
    turn.begin(+1.0)
    assert turn.heading_source_name == "wheel"
    filt._usable = True                       # changes after begin()
    assert turn.heading_source_name == "wheel"


# ---------------------------------------------------------------------------
# Wheel odometry (commanded-velocity fallback path)
# ---------------------------------------------------------------------------

def test_straight_line_integration():
    odo = WheelOdometry(None)
    for _ in range(100):
        odo.tick(0.5, 0.0, 0.01)
    assert odo.distance == pytest.approx(0.5, abs=1e-6)
    assert odo.x == pytest.approx(0.5, abs=1e-6)
    assert odo.y == pytest.approx(0.0, abs=1e-6)


def test_quarter_circle_arc():
    """v=0.3, w=0.3 -> radius 1.0 m; integrate a quarter turn and compare
    with the analytic arc end point (midpoint integration => <1% error)."""
    odo = WheelOdometry(None)
    v, w = 0.3, 0.3
    dt = 0.01
    steps = int((math.pi / 2) / (w * dt))
    for _ in range(steps):
        odo.tick(v, w, dt)
    r = v / w
    assert odo.theta == pytest.approx(math.pi / 2, abs=0.02)
    assert odo.x == pytest.approx(r * math.sin(odo.theta), abs=0.01)
    assert odo.y == pytest.approx(r * (1 - math.cos(odo.theta)), abs=0.01)


def test_reverse_distance_accumulates_magnitude():
    odo = WheelOdometry(None)
    odo.tick(-0.2, 0.0, 1.0)
    assert odo.distance == pytest.approx(0.2)
    assert odo.x == pytest.approx(-0.2)
