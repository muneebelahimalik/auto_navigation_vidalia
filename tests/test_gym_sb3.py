"""Gymnasium wrapper + Stable-Baselines3 export (skipped where the libraries
are not installed — the robot never needs them)."""
from __future__ import annotations

import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")

from navigation.rl_policy import MLPPolicy, encode_obs          # noqa: E402
from sim.gym_env import RowFollowGymEnv                         # noqa: E402
from sim.row_follow_env import RowFollowEnv, onion_bed_config   # noqa: E402


@pytest.mark.parametrize("profile", ["soybean", "onion"])
def test_env_passes_gymnasium_checker(profile):
    from gymnasium.utils.env_checker import check_env
    check_env(RowFollowGymEnv(profile), skip_render_check=True)


def test_episode_seed_replays_the_core_simulator_exactly():
    """reset(options={'episode_seed': s}) == RowFollowEnv.reset(seed=s): the
    held-out evaluation episodes are the same for every trainer."""
    genv = RowFollowGymEnv("onion", c_u=0.8, c_du=8.0)
    core = RowFollowEnv(onion_bed_config(c_u=0.8, c_du=8.0))
    g_obs, _ = genv.reset(options={"episode_seed": 50_123})
    c_obs = core.reset(seed=50_123)
    np.testing.assert_allclose(g_obs, encode_obs(*c_obs), rtol=1e-6)
    rng = np.random.default_rng(0)
    for _ in range(200):
        a = rng.uniform(-1, 1, size=1)
        g_obs, g_r, term, trunc, _ = genv.step(a.astype(np.float32))
        c_obs, c_r, done, _ = core.step(a.astype(np.float32).astype(np.float64))
        np.testing.assert_allclose(g_obs, encode_obs(*c_obs), rtol=1e-5, atol=1e-6)
        assert g_r == pytest.approx(c_r, rel=1e-9)
        assert (term or trunc) == done
        if done:
            break


def test_reset_seed_is_reproducible():
    a, _ = RowFollowGymEnv().reset(seed=7)
    b, _ = RowFollowGymEnv().reset(seed=7)
    np.testing.assert_array_equal(a, b)


def test_clip_output_policy_roundtrip(tmp_path):
    pol = MLPPolicy(obs_dim=5, out_act="clip", seed=3)
    pol._b[-1][:] = 5.0                        # saturate → clip, not tanh
    p = tmp_path / "p.npz"
    pol.save(str(p))
    back = MLPPolicy.load(str(p))
    assert back.out_act == "clip"
    assert back.act([0, 0, 1, 0, 0]) == 1.0


@pytest.mark.parametrize("algo", ["ppo", "sac"])
def test_sb3_actor_exports_exactly_to_numpy(algo):
    pytest.importorskip("stable_baselines3")
    torch = pytest.importorskip("torch")
    from scripts.train_sb3 import build, export_actor
    from stable_baselines3.common.env_util import make_vec_env

    env = make_vec_env(RowFollowGymEnv, n_envs=1, seed=0)
    model = build(algo, env, seed=0)
    with torch.no_grad():                      # move off the near-zero init
        for p in model.policy.parameters():
            p.add_(0.5 * torch.randn_like(p))
    pol = export_actor(model)
    rng = np.random.default_rng(1)
    for _ in range(50):
        raw = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.4, 0.4),
                        rng.uniform(0, 1), rng.uniform(-1, 1), rng.uniform(-0.5, 0.5)])
        sb3_a, _ = model.predict(encode_obs(*raw).astype(np.float32), deterministic=True)
        assert pol.act(raw) == pytest.approx(float(sb3_a[0]), abs=1e-5)
