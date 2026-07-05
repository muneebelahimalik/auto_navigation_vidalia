"""Unit tests for navigation/row_controller.py — pure-pursuit geometry.

Includes a closed-loop unicycle simulation that drives a kinematic robot
with the controller's own commands and asserts convergence onto the row —
the strongest offline check of the full sign chain
(perception convention -> controller -> canbus convention).
"""
import math

from navigation.row_controller import PurePursuitController
from navigation.row_perception import RowEstimate


def est(lat=0.0, hdg=0.0, conf=0.8):
    return RowEstimate(heading_error=hdg, lateral_offset=lat,
                       confidence=conf, valid=True)


# ---------------------------------------------------------------------------
# Sign conventions (canbus: angular > 0 = CCW / left turn)
# ---------------------------------------------------------------------------

def test_row_to_the_right_turns_right():
    ctrl = PurePursuitController()
    linear, angular = ctrl.compute(est(lat=+0.30))
    assert linear > 0.0
    assert angular < 0.0


def test_row_to_the_left_turns_left():
    ctrl = PurePursuitController()
    _, angular = ctrl.compute(est(lat=-0.30))
    assert angular > 0.0


def test_row_angled_right_turns_right():
    ctrl = PurePursuitController()
    _, angular = ctrl.compute(est(hdg=+0.20))
    assert angular < 0.0


def test_low_confidence_stops():
    ctrl = PurePursuitController(min_confidence=0.35)
    assert ctrl.compute(est(lat=0.3, conf=0.2)) == (0.0, 0.0)


def test_limits_respected():
    ctrl = PurePursuitController(max_linear=0.30, max_angular=0.40)
    linear, angular = ctrl.compute(est(lat=0.8, hdg=0.5, conf=1.0))
    assert 0.0 < linear <= 0.30
    assert abs(angular) <= 0.40


def test_speed_slows_with_heading_error():
    ctrl = PurePursuitController()
    v_straight, _ = ctrl.compute(est(hdg=0.0, conf=0.8))
    v_angled, _ = ctrl.compute(est(hdg=0.5, conf=0.8))
    assert v_angled < v_straight


# ---------------------------------------------------------------------------
# Closed-loop convergence
# ---------------------------------------------------------------------------

def simulate(x0, phi0, steps=900, dt=0.1, conf=0.8):
    """Kinematic unicycle following a row along world +Y at world x=0.

    World frame: x = right, y = forward; robot heading phi is CCW-positive
    with phi=0 pointing along +y (down the row).  In the robot frame the
    perception module would measure:
        heading_error  = phi          (row angled to the right when phi>0)
        lateral_offset = -x           (row centre to the right when x<0)
    Both identities follow from rotating the row line into the robot frame.
    """
    ctrl = PurePursuitController()
    x, phi = x0, phi0
    xs = [x]
    for _ in range(steps):
        v, w = ctrl.compute(est(lat=-x, hdg=phi, conf=conf))
        phi += w * dt
        x += v * -math.sin(phi) * dt
        xs.append(x)
    return x, phi, max(abs(v) for v in xs)


def test_converges_from_lateral_offset():
    x, phi, x_max = simulate(x0=0.30, phi0=0.0)
    assert abs(x) < 0.05
    assert abs(phi) < 0.06
    assert x_max < 0.40           # no divergent overshoot


def test_converges_from_heading_error():
    x, phi, x_max = simulate(x0=0.0, phi0=math.radians(20))
    assert abs(x) < 0.05
    assert abs(phi) < 0.06
    assert x_max < 0.45


def test_converges_from_combined_error():
    x, phi, x_max = simulate(x0=-0.25, phi0=math.radians(-15))
    assert abs(x) < 0.05
    assert abs(phi) < 0.06
    assert x_max < 0.45


# ---------------------------------------------------------------------------
# Integral (disturbance-cancelling) action
# ---------------------------------------------------------------------------

def simulate_drift(ki, drift=0.03, steps=900, dt=0.1, conf=0.8):
    """Closed loop with a CONSTANT cross-slope drift pushing the robot off the
    row.  Returns the steady-state |offset| (mean over the last 20 s)."""
    ctrl = PurePursuitController(ki=ki)
    x, phi, xs = 0.0, 0.0, []
    for _ in range(steps):
        v, w = ctrl.compute(est(lat=-x, hdg=phi, conf=conf))
        phi += w * dt
        x += (v * -math.sin(phi) + drift) * dt      # drift pushes +x
        xs.append(abs(x))
    return sum(xs[-200:]) / 200.0


def test_integral_rejects_steady_drift():
    """The leaky integral must cut the standing cross-track error a constant
    cross-slope drift leaves — the whole point of the enhancement.  Plain
    geometric pure pursuit (ki=0) can only chase the drift."""
    ss_geom = simulate_drift(ki=0.0)
    ss_integ = simulate_drift(ki=0.15)
    assert ss_integ < 0.6 * ss_geom          # materially smaller standing error
    assert ss_integ < 0.12                     # and small in absolute terms


def test_ki_zero_is_pure_geometric():
    """ki=0 restores the exact memoryless geometric law (backward compat): the
    integral never accumulates and does not shift the command."""
    geom = PurePursuitController(ki=0.0)
    for _ in range(50):
        geom.compute(est(lat=0.08, hdg=0.03))
    assert geom._eint == 0.0
    # one-shot command matches a hand-rolled controller with no integral history
    a = PurePursuitController(ki=0.0).compute(est(lat=0.08, hdg=0.03))
    b = geom.compute(est(lat=0.08, hdg=0.03))
    assert a == b


def test_reset_clears_integral():
    ctrl = PurePursuitController(ki=0.15)
    for _ in range(60):                        # wind the integral up on a steady offset
        ctrl.compute(est(lat=0.05, hdg=0.0))
    assert abs(ctrl._eint) > 0.0
    ctrl.reset()
    assert ctrl._eint == 0.0
