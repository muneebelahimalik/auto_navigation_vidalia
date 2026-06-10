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

    def apply(self, v, w, dt):
        self.distance += abs(v) * dt
        self.theta += w * dt


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
    odo, turn, phases = run_turn(+1.0)
    assert turn.done
    assert phases[:4] == ["EXIT", "TURN_A", "SHIFT", "TURN_B"]
    # Two 90-deg pivots clockwise => net heading ~ -pi (right U-turn).
    assert odo.theta == pytest.approx(-math.pi, abs=0.15)
    # Straight segments: exit + shift.
    assert odo.distance == pytest.approx(1.0 + 0.76, abs=0.05)


def test_left_uturn_mirrors_heading():
    odo, turn, _ = run_turn(-1.0)
    assert turn.done
    assert odo.theta == pytest.approx(+math.pi, abs=0.15)


def test_pivot_commands_are_in_place():
    odo = FakeOdo()
    turn = HeadlandTurn(odo, exit_dist=0.0)   # skip EXIT instantly
    turn.begin(+1.0)
    v, w = turn.step(0.05)
    assert turn.phase == "TURN_A"
    assert v == 0.0
    assert w < 0.0                            # right turn = clockwise = negative


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
