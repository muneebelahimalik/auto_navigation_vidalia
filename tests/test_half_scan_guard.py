"""Half-scan guard (field 2026-11-06): LiDAR packet loss took a whole left or
right half of the forward view in 4 of 5 scans.  A missing SIDE leaves no
internal azimuth hole, so the old sector-gap test let the scan through and the
bed was fitted from one side — lateral std ~8 cm with ~16 cm spikes."""
from pathlib import Path

import numpy as np
import pytest

from sim.onion_field import OnionField, stage_config
from tests.test_onion_growth_stages import _det

FIELD = Path(__file__).parent / "field_scans"


def test_real_field_scans_classified():
    det = _det()
    full = np.load(FIELD / "onion_20261106_complete.npy")
    half = np.load(FIELD / "onion_20261106_half_left_missing.npy")
    assert not det._is_half_scan(full)
    assert det._is_half_scan(half)
    assert det._is_half_scan(half[half[:, 0] > 0])
    assert det._is_half_scan(full[full[:, 0] < 0])        # right half lost


def _drive(stage, guard, every=5, n=60, d=0.08):
    """4 of 5 scans lose alternately the left / right half (every=5)."""
    f = OnionField(stage_config(stage), seed=0)
    det = _det()
    det.half_scan_guard = guard
    rng = np.random.default_rng(1)
    err, conf = [], []
    for i in range(n):
        P, tr = f.scan(xr=d, yr=0.05 * i, rng=rng, return_truth=True)
        if i > 10 and i % every != 0:
            P = P[P[:, 0] > 0] if (i // every) % 2 else P[P[:, 0] < 0]
        e = det.update(P)
        if i > 15:
            err.append(e.lateral_offset - tr["lateral"])
            conf.append(e.confidence)
    return np.array(err), np.array(conf), det


@pytest.mark.parametrize("stage", ["bare", "young", "mature"])
def test_guard_removes_the_half_scan_wobble(stage):
    err, conf, det = _drive(stage, guard=True)
    assert np.abs(err).max() < 0.02, (stage, np.abs(err).max())
    assert conf.min() > 0.9
    assert det.half_scans_skipped > 0
    err_off, _, _ = _drive(stage, guard=False)
    assert np.abs(err_off).max() > 0.10          # the field failure, reproduced


def test_persistent_half_scans_drop_confidence_below_follow():
    """Every scan halved (reader broken): never drive on it with confidence —
    after the short hold the estimate is capped below the 0.35 FOLLOW bar."""
    _, conf, _ = _drive("young", guard=True, every=10 ** 6)
    assert conf[-1] < 0.35
