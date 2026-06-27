#!/usr/bin/env python3
"""
test_mpc.py — Regression tests for the MPPI/MPC steering controller.

Covers the same safety contract as RLController (fallback, clamp, interface)
plus the two MPC-specific behaviours: it drives the offset toward zero, and its
online disturbance observer lets it reject a persistent cross-slope drift that
pure pursuit cannot.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.row_controller import PurePursuitController
from navigation.row_mpc_controller import RowMPCController
from navigation.row_perception import RowEstimate
from sim.evaluate import evaluate, mpc_act_fn, pursuit_act_fn


def _est(e, theta, conf=0.9, valid=True):
    return RowEstimate(heading_error=theta, lateral_offset=e, confidence=conf, valid=valid)


# ---------------------------------------------------------------- safety contract
def test_low_confidence_falls_back_to_pursuit():
    mpc = RowMPCController(min_confidence=0.35)
    pp = PurePursuitController(min_confidence=0.35)
    est = _est(0.2, 0.1, conf=0.20)   # below threshold
    assert mpc.compute(est) == pp.compute(est)


def test_invalid_fix_falls_back_to_pursuit():
    mpc = RowMPCController()
    pp = PurePursuitController()
    est = _est(0.2, 0.1, conf=0.9, valid=False)
    assert mpc.compute(est) == pp.compute(est)


def test_angular_output_is_clamped():
    mpc = RowMPCController(max_angular=0.40)
    for e, th in [(0.6, 0.5), (-0.6, -0.5), (0.0, 0.4)]:
        _v, w = mpc.compute(_est(e, th))
        assert -0.40 - 1e-9 <= w <= 0.40 + 1e-9


def test_speed_matches_pursuit():
    # MPC only sets STEERING; the forward speed must equal pure-pursuit's.
    mpc = RowMPCController()
    pp = PurePursuitController()
    est = _est(0.1, 0.05)
    assert math.isclose(mpc.compute(est)[0], pp.compute(est)[0], rel_tol=1e-9)


def test_reset_clears_state():
    mpc = RowMPCController()
    for _ in range(5):
        mpc.compute(_est(0.3, 0.2))
    mpc.reset()
    assert mpc._d_hat == 0.0
    assert np.allclose(mpc._u_nom, 0.0)
    assert mpc._last is None


# ---------------------------------------------------------------- steering sign
def test_steers_to_reduce_positive_offset():
    # Row to the right (e>0) with zero heading: should command a right turn (w<0).
    mpc = RowMPCController(seed=1)
    ws = [mpc.compute(_est(0.3, 0.0))[1] for _ in range(5)]
    assert np.mean(ws) < 0.0


def test_steers_to_reduce_negative_offset():
    mpc = RowMPCController(seed=1)
    ws = [mpc.compute(_est(-0.3, 0.0))[1] for _ in range(5)]
    assert np.mean(ws) > 0.0


# ---------------------------------------------------------------- disturbance observer
def test_disturbance_observer_learns_drift_sign():
    """Feeding a steady rightward drift makes d_hat grow positive."""
    mpc = RowMPCController(seed=2)
    e = 0.0
    for _ in range(30):
        _v, w = mpc.compute(_est(e, 0.0))
        e += 0.03            # persistent rightward lateral drift each step
    assert mpc._d_hat > 0.02   # observer has picked up the positive drift


# ---------------------------------------------------------------- closed-loop sim
def test_mpc_beats_pursuit_on_crosstrack():
    seeds = list(range(60_000, 60_120))
    base = evaluate(pursuit_act_fn(), seeds)
    mpc = evaluate(mpc_act_fn(), seeds)
    # Lower cross-track RMSE and at least as reliable as pure pursuit.
    assert mpc["xtrack_rmse_m"][0] < base["xtrack_rmse_m"][0]
    assert mpc["success_rate"] >= base["success_rate"]


def test_mpc_act_fn_is_resettable():
    act = mpc_act_fn()
    assert hasattr(act, "reset")
    act(np.array([0.2, 0.1, 0.9, 0.0]))
    act.reset()   # must not raise
