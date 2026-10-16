"""Onion-field RL: system-identified sim profile + the deployed onion policy.

The 2026-10-15 Vidalia run (bare raised bed, robot steering by the bed relief,
pursuit at 0.20 m/s) has a very different disturbance regime from the soybean
defaults: estimate noise and slow target wander dominate, the field is flat,
there are no dropouts.  ``onion_bed_config`` was fitted
(``scripts/calibrate_sim_from_run.py``) so that pure pursuit in the sim
reproduces that run's closed-loop statistics, and ``policies/follow_onion.npz``
was trained on it.
"""
import numpy as np

from navigation.rl_policy import MLPPolicy
from sim.evaluate import evaluate, policy_act_fn, pursuit_act_fn
from sim.row_follow_env import EnvConfig, RowFollowEnv, onion_bed_config

# Steady-FOLLOW statistics of the 2026-10-15 onion run (7714 scans).
FIELD = dict(lat_std=0.0564, dlat_std=0.0304, lat_ac10=0.4520,
             hdg_std=0.0383, dhdg_std=0.0092, w_abs=0.0077)


def test_default_sim_is_unchanged_by_the_field_profile_fields():
    """The new profile knobs default OFF: the soybean sim (and every policy and
    result built on it) is byte-identical."""
    c = EnvConfig()
    assert c.e_bias_std == 0.0 and c.th_bias_std == 0.0 and c.obs_ema == 0.0
    assert c.max_linear == 0.30
    a, b = RowFollowEnv(EnvConfig()), RowFollowEnv(EnvConfig())
    oa, ob = a.reset(seed=3), b.reset(seed=3)
    for _ in range(50):
        oa, *_ = a.step(0.1)
        ob, *_ = b.step(0.1)
    assert np.allclose(oa, ob)


def test_onion_profile_reproduces_the_field_run():
    """Pure pursuit in the onion profile matches the field closed-loop stats
    (what the controller saw) within ~25 %."""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))
    from calibrate_sim_from_run import sim_stats
    s = sim_stats(onion_bed_config(), n_ep=6, steps=1200, seed=1000)   # as fitted
    for k, v in FIELD.items():
        assert abs(s[k] - v) <= 0.25 * v + 1e-3, (k, s[k], v)


def test_onion_policy_beats_pursuit_in_onion_sim_without_thrashing():
    pol = MLPPolicy.load("policies/follow_onion.npz")
    cfg = onion_bed_config()
    seeds = range(123_000, 123_060)
    base = evaluate(pursuit_act_fn(), seeds, cfg)
    rl = evaluate(policy_act_fn(pol), seeds, cfg)
    assert rl["success_rate"] == 1.0
    assert rl["xtrack_rmse_m"][0] < base["xtrack_rmse_m"][0]
    # Smooth: steering effort within 2x pursuit's (the soybean policy is ~2.7x
    # here and 6.4x on the real onion estimates).
    assert rl["control_effort"][0] < 2.0 * base["control_effort"][0]
    assert rl["control_jerk"][0] < 0.03


def test_onion_policy_still_safe_on_soybean_sim():
    """Specialised for onion, but must not fall apart if run on the soybean
    disturbance model (cross-slope drift, dropouts)."""
    pol = MLPPolicy.load("policies/follow_onion.npz")
    agg = evaluate(policy_act_fn(pol), range(130_000, 130_060), EnvConfig())
    assert agg["success_rate"] >= 0.9
