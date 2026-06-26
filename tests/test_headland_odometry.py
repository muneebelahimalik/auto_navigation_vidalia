"""Unit tests for navigation/headland.py and navigation/odometry.py.

The headland U-turn is **perception-closed**: it drives a large-radius arc and
keeps arcing until the navigator detects the next row aligned ahead (then calls
finish()).  It uses odometry only for coarse arc-LENGTH guards — never the
heading, which a skid-steer over-reports.  A kinematic integrator stands in for
the wheel telemetry; the perception side is exercised in tests/test_state_logic
(turn_reacquired) and the navigator.
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


def _make(odo, **kw):
    kw.setdefault("row_spacing", 1.52)
    kw.setdefault("exit_dist", 1.0)
    kw.setdefault("speed", 0.15)
    kw.setdefault("turn_rate", 0.30)
    return HeadlandTurn(odo, **kw)


def test_exit_then_arc_sequence():
    odo = FakeOdo()
    turn = _make(odo, exit_dist=1.0, turn_radius=1.0)
    turn.begin(+1.0)
    assert turn.phase == "EXIT"
    dt = 0.05
    saw_arc = False
    for _ in range(2000):
        v, w = turn.step(dt)
        if turn.phase == "EXIT":
            assert v > 0.0 and w == 0.0       # straight exit
        if turn.phase == "ARC":
            saw_arc = True
            break
        odo.apply(v, w, dt)
    assert saw_arc
    # Switched to ARC right after driving exit_dist straight.
    assert odo.distance == pytest.approx(1.0, abs=0.05)


def test_arc_command_is_a_moving_turn_not_a_pivot():
    """The turn must ARC (linear > 0), not pivot in place — an in-place pivot
    scrubs the wheels and the wheel heading over-reads (the field bug where the
    U-turn only reached ~90°)."""
    odo = FakeOdo()
    turn = _make(odo, exit_dist=0.0, turn_radius=1.0)  # skip EXIT
    turn.begin(+1.0)
    v, w = turn.step(0.05)
    assert turn.phase == "ARC"
    assert v > 0.0                            # moving arc, not a pivot
    assert w < 0.0                            # right turn = clockwise = negative
    # Arc forward speed = radius * turn_rate.
    assert v == pytest.approx(1.0 * 0.30, abs=1e-9)


def test_left_turn_is_counterclockwise():
    odo = FakeOdo()
    turn = _make(odo, exit_dist=0.0)
    turn.begin(-1.0)
    _, w = turn.step(0.05)
    assert w > 0.0                            # left = CCW = positive


def test_explicit_turn_radius_overrides_default():
    odo = FakeOdo()
    turn = _make(odo, turn_radius=1.4, exit_dist=0.0)
    assert turn.turn_radius == 1.4


def test_default_radius_is_one_metre():
    odo = FakeOdo()
    turn = _make(odo)                         # no turn_radius given
    assert turn.turn_radius == pytest.approx(1.0)


def test_not_ready_to_reacquire_until_min_arc_driven():
    """ready_to_reacquire only after min_turn_frac of a nominal semicircle —
    so the turn cannot end on the row just left or an early cross-row glimpse."""
    odo = FakeOdo()
    turn = _make(odo, exit_dist=0.0, turn_radius=1.0,
                 min_turn_frac=0.55, max_turn_frac=3.0)
    turn.begin(+1.0)
    nominal = math.pi * 1.0
    dt = 0.05
    became_ready_at = None
    for _ in range(5000):
        v, w = turn.step(dt)
        if turn.done:
            break
        if turn.ready_to_reacquire and became_ready_at is None:
            became_ready_at = turn.arc_len
        odo.apply(v, w, dt)
    assert became_ready_at is not None
    assert became_ready_at == pytest.approx(0.55 * nominal, abs=0.1)


def test_finish_ends_the_turn_without_cap():
    """The navigator calls finish() when perception re-locks the next row."""
    odo = FakeOdo()
    turn = _make(odo, exit_dist=0.0, turn_radius=1.0)
    turn.begin(+1.0)
    dt = 0.05
    for _ in range(200):              # drive part of the arc
        v, w = turn.step(dt)
        odo.apply(v, w, dt)
    assert turn.phase == "ARC" and not turn.done
    turn.finish()
    assert turn.done
    assert turn.capped is False


def test_arc_length_cap_stops_the_turn():
    """With no finish() call the turn gives up after max_turn_frac semicircles
    (field edge / no next row) and reports capped."""
    odo = FakeOdo()
    turn = _make(odo, exit_dist=0.0, turn_radius=1.0, max_turn_frac=2.0)
    turn.begin(+1.0)
    dt = 0.05
    for _ in range(100000):
        v, w = turn.step(dt)
        if turn.done:
            break
        odo.apply(v, w, dt)
    assert turn.done
    assert turn.capped is True
    # Stopped near the cap distance (2 nominal semicircles).
    assert turn.arc_len == pytest.approx(2.0 * math.pi * 1.0, abs=0.1)


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
