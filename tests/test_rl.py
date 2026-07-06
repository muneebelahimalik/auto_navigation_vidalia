"""Unit tests for the optional RL steering feature (sim env, policy, controller,
ES update).  All numpy-only; no torch/gym required."""
import math

import numpy as np
import pytest

from navigation.row_perception import RowEstimate
from navigation.rl_policy import MLPPolicy, encode_obs
from navigation.rl_controller import RLController
from sim.row_follow_env import EnvConfig, RowFollowEnv, speed_from
from sim.evaluate import evaluate, policy_act_fn, pursuit_act_fn, rollout


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------

def test_env_reset_is_deterministic_per_seed():
    env = RowFollowEnv()
    o1 = env.reset(seed=3)
    o2 = env.reset(seed=3)
    assert np.allclose(o1, o2)
    o3 = env.reset(seed=4)
    assert not np.allclose(o1, o3)


def test_env_step_shapes_and_finite():
    env = RowFollowEnv()
    env.reset(seed=0)
    obs, r, done, info = env.step(0.1)
    assert obs.shape == (RowFollowEnv.OBS_DIM,)
    assert np.all(np.isfinite(obs)) and math.isfinite(r)
    assert set(["e", "theta", "off_road"]).issubset(info)


def test_env_terminates_off_row_with_penalty():
    """An uncompensated strong cross-slope drift pushes the robot off the row
    and ends the episode early with a penalty."""
    env = RowFollowEnv(EnvConfig(dropout_p=0.0))
    env.reset(seed=1)
    env._e = 0.55; env._theta = 0.0; env._grade = 0.20   # near edge, drifting out
    done = False; steps = 0; last = {}; rewards = []
    while not done and steps < 50:
        _o, r, done, last = env.step(0.0)    # no steering correction
        rewards.append(r); steps += 1
    assert last["off_road"] is True
    assert steps < env.cfg.max_steps          # ended early by going off-row
    assert rewards[-1] < 0                     # terminal off-road penalty


def test_speed_law_matches_pursuit_formula():
    # high conf, no heading error -> near max; low conf -> throttled toward min
    assert speed_from(0.92, 0.0) == pytest.approx(0.30 * 0.92, abs=1e-9)
    assert speed_from(0.20, 0.0) >= 0.08


# ---------------------------------------------------------------------------
# Policy
# ---------------------------------------------------------------------------

def test_policy_output_bounded_and_deterministic():
    pol = MLPPolicy(seed=1)
    for _ in range(20):
        o = np.random.default_rng().normal(size=4)
        a = pol.act(o)
        assert -1.0 <= a <= 1.0
    o = np.array([0.1, -0.05, 0.9, 0.2])
    assert pol.act(o) == pol.act(o)           # deterministic


def test_policy_param_roundtrip_and_save_load(tmp_path):
    pol = MLPPolicy(hidden=(8, 8), seed=2)
    p = pol.get_params()
    assert p.size == pol.n_params
    pol2 = MLPPolicy(hidden=(8, 8), seed=99)
    pol2.set_params(p)
    o = np.array([0.2, 0.1, 0.8, -0.3])
    assert pol2.act(o) == pytest.approx(pol.act(o), abs=1e-12)
    f = tmp_path / "pol.npz"
    pol.save(str(f))
    pol3 = MLPPolicy.load(str(f))
    assert pol3.act(o) == pytest.approx(pol.act(o), abs=1e-12)


def test_encode_obs_scaling():
    enc = encode_obs(0.5, 0.4, 1.0, 1.0, 0.5)
    assert np.allclose(enc, [1.0, 1.0, 1.0, 1.0, 1.0])


# ---------------------------------------------------------------------------
# Controller — drop-in + safety fallback
# ---------------------------------------------------------------------------

def test_rl_controller_matches_pursuit_interface():
    c = RLController(policy=None)
    est = RowEstimate(heading_error=0.1, lateral_offset=0.2, confidence=0.9, valid=True)
    v, w = c.compute(est)
    assert isinstance(v, float) and isinstance(w, float)


def test_no_policy_is_exactly_pursuit():
    from navigation.row_controller import PurePursuitController
    pp = PurePursuitController()
    c = RLController(policy=None)
    est = RowEstimate(heading_error=0.15, lateral_offset=-0.25, confidence=0.8, valid=True)
    assert c.compute(est) == pp.compute(est)


def test_low_confidence_falls_back_to_pursuit():
    from navigation.row_controller import PurePursuitController
    pp = PurePursuitController()
    pol = MLPPolicy(seed=5)
    c = RLController(policy=pol, min_confidence=0.35)
    est = RowEstimate(heading_error=0.1, lateral_offset=0.1, confidence=0.20, valid=True)
    assert c.compute(est) == pp.compute(est)   # below threshold → pursuit


def test_policy_steering_is_clamped():
    pol = MLPPolicy(seed=7)
    # blow up the weights so the raw output saturates
    pol.set_params(pol.get_params() * 50.0)
    c = RLController(policy=pol, max_angular=0.40)
    est = RowEstimate(heading_error=0.3, lateral_offset=0.4, confidence=0.9, valid=True)
    _v, w = c.compute(est)
    assert -0.40 <= w <= 0.40


# ---------------------------------------------------------------------------
# Residual composition: policy adds a bounded delta on top of the baseline
# ---------------------------------------------------------------------------

def test_residual_zero_policy_equals_baseline():
    """A residual policy that outputs ~0 must reproduce the pure-pursuit baseline
    (delta≈0 → total = baseline).  This is why residual RL starts at the
    baseline's performance instead of learning steering from scratch."""
    from navigation.row_controller import PurePursuitController
    pol = MLPPolicy(seed=3)
    pol.set_params(pol.get_params() * 0.0)        # exactly-zero network → delta≈0
    rl = RLController(policy=pol, residual=True, residual_scale=0.5)
    base = PurePursuitController()
    est = RowEstimate(heading_error=0.10, lateral_offset=0.12, confidence=0.9, valid=True)
    _v0, w_base = base.compute(est)
    _v1, w_res = rl.compute(est)
    assert abs(w_res - w_base) < 1e-6


def test_residual_output_is_bounded():
    """Even a saturating residual policy stays within ±max_angular."""
    pol = MLPPolicy(seed=7)
    pol.set_params(pol.get_params() * 50.0)
    rl = RLController(policy=pol, residual=True, residual_scale=0.5, max_angular=0.40)
    est = RowEstimate(heading_error=0.3, lateral_offset=0.4, confidence=0.9, valid=True)
    _v, w = rl.compute(est)
    assert -0.40 <= w <= 0.40


def test_residual_act_fn_starts_near_baseline_in_sim():
    """The residual env-policy with a near-zero net scores ≈ the pursuit baseline
    over held-out episodes (the residual head-start)."""
    from sim.evaluate import residual_act_fn
    pol = MLPPolicy(obs_dim=RowFollowEnv.OBS_DIM, seed=1)
    pol.set_params(pol.get_params() * 0.0)
    seeds = list(range(300, 340))
    base = evaluate(pursuit_act_fn(), seeds)["return"][0]
    res = evaluate(residual_act_fn(pol, scale=0.5), seeds)["return"][0]
    assert abs(res - base) < 1.0                    # essentially the baseline


# ---------------------------------------------------------------------------
# Pure-pursuit baseline keeps the simulated robot on the row (sanity that the
# env + baseline are coherent before any learning).
# ---------------------------------------------------------------------------

def test_pursuit_baseline_mostly_succeeds_in_sim():
    agg = evaluate(pursuit_act_fn(), seeds=range(0, 40), config=EnvConfig())
    assert agg["success_rate"] >= 0.8         # stays on the row most episodes
    assert agg["xtrack_rmse_m"][0] < 0.25     # within a reasonable corridor


# ---------------------------------------------------------------------------
# Evolution Strategies — validate the optimiser on a trivial, fast objective
# (maximise -||theta - target||^2) so the test is deterministic and quick.
# ---------------------------------------------------------------------------

def test_es_update_improves_quadratic_objective():
    rng = np.random.default_rng(0)
    n = 12
    target = rng.normal(size=n)
    theta = np.zeros(n)
    sigma, lr, half = 0.2, 0.3, 25

    def f(x):
        return -np.sum((x - target) ** 2)

    def rank_norm(s):
        r = np.empty_like(s); r[np.argsort(s)] = np.arange(len(s))
        return r / (len(s) - 1) - 0.5

    start = f(theta)
    for _ in range(60):
        eps = rng.normal(size=(half, n))
        scores = np.empty(2 * half)
        for i in range(half):
            scores[2 * i] = f(theta + sigma * eps[i])
            scores[2 * i + 1] = f(theta - sigma * eps[i])
        u = rank_norm(scores)
        g = sum((u[2 * i] - u[2 * i + 1]) * eps[i] for i in range(half))
        theta = theta + lr * g / (2 * half * sigma)
    assert f(theta) > start                    # ES moved uphill
    assert np.linalg.norm(theta - target) < np.linalg.norm(target)


# ---------------------------------------------------------------------------
# Drift integrator (RL analogue of the MPC disturbance observer)
# ---------------------------------------------------------------------------

def test_update_eint_accumulates_and_clips():
    from navigation.rl_policy import update_eint, EINT_CLIP
    # constant lateral -> integrator grows toward a bounded steady state
    e = 0.0
    for _ in range(200):
        e = update_eint(e, 0.2)
    assert 0.0 < e <= EINT_CLIP + 1e-9
    # opposite sign drives it negative
    e2 = 0.0
    for _ in range(200):
        e2 = update_eint(e2, -0.2)
    assert -EINT_CLIP - 1e-9 <= e2 < 0.0


def test_update_eint_leaks_back_to_zero():
    from navigation.rl_policy import update_eint
    e = 0.4
    for _ in range(300):
        e = update_eint(e, 0.0)          # no input -> decays
    assert abs(e) < 1e-3


def test_env_obs_is_five_dim_with_integrator():
    from sim.row_follow_env import RowFollowEnv
    assert RowFollowEnv.OBS_DIM == 5
    env = RowFollowEnv()
    obs = env.reset(seed=1)
    assert obs.shape == (5,)


def test_rl_controller_feeds_five_inputs_to_five_dim_policy():
    from navigation.rl_policy import MLPPolicy
    pol = MLPPolicy(obs_dim=5)
    c = RLController(policy=pol, min_confidence=0.35)
    assert c._use_eint is True
    est = RowEstimate(heading_error=0.05, lateral_offset=0.3, confidence=0.9, valid=True)
    # several confident steps -> integrator becomes non-zero, no crash
    for _ in range(20):
        v, w = c.compute(est)
    assert c._eint != 0.0
    assert abs(w) <= c.max_angular + 1e-9
    c.reset()
    assert c._eint == 0.0 and c._prev_a == 0.0


def test_rl_controller_backward_compatible_with_four_dim_policy():
    """A pre-integrator 4-input policy must still run (fed 4 inputs, no crash)."""
    from navigation.rl_policy import MLPPolicy
    pol = MLPPolicy(obs_dim=4)
    c = RLController(policy=pol, min_confidence=0.35)
    assert c._use_eint is False
    est = RowEstimate(heading_error=0.05, lateral_offset=0.2, confidence=0.9, valid=True)
    v, w = c.compute(est)
    assert abs(w) <= c.max_angular + 1e-9
