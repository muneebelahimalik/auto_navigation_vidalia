"""Onion bed tracking at EVERY growth stage — ray-cast VLP-16 Hi-Res scans.

``sim/onion_field.py`` ray-casts the real beam geometry through raised beds and
a porous onion canopy, so occlusion is physical: transplants are invisible,
young rows are separate, from mid-season the canopy hides the bed-top soil and
shadows the wheel furrows, at maturity neighbouring rows interleave into one
canopy, and lodged tops droop over the shoulders.  The detector is built
exactly as ``row_follow.py --preset onion`` builds it.

Locks in the 2026-10 all-stage rework (see CLAUDE.md, onion section):
gap-filled multi-percentile bed profile, terrain-relative two-pass bed-axis
search + axis filter, furrow-referenced soil floor that holds under canopy,
crop-comb acceptance with leafy-row concentration test and lock hysteresis,
crop-only plausibility gate, view-quality confidence, sector-dropout guard,
absolute bed-presence floor, canopy-adaptive obstacle height.
"""
import math
from dataclasses import replace

import numpy as np
import pytest

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
from navigation.row_perception import RowDetector
from navigation.row_safety import SafetyMonitor
from sim.onion_field import STAGES, FieldConfig, OnionField, SensorConfig, stage_config


def _det():
    return RowDetector(roi_x_half=0.80, crop_h_min=0.05, crop_h_max=0.60,
                       dual_row=True, row_spacing=0.2794, bed_rows=4,
                       bed_shape=True, bed_pitch=1.83)


def _drive(field, d, yaw, n=26, settle=12, det=None, seed=0, sensor=None, hook=None):
    det = det or _det()
    rng = np.random.default_rng(seed)
    lat, hdg, conf, rowend, modes = [], [], [], [], []
    for i in range(n):
        P, tr = field.scan(xr=d, yr=0.02 * i, yaw_deg=yaw, rng=rng, sensor=sensor,
                           return_truth=True)
        if hook is not None:
            P = hook(P, rng)
        e = det.update(P)
        if i >= settle:
            lat.append(e.lateral_offset - tr["lateral"])
            hdg.append(math.degrees(e.heading_error - tr["heading"]))
            conf.append(e.confidence)
            rowend.append(e.row_end_confidence)
            modes.append(det.last_mode)
    return det, np.array(lat), np.array(hdg), np.array(conf), np.array(rowend), modes


@pytest.mark.parametrize("stage", ["bare", "seedling", "young", "mid", "mature", "lodged"])
def test_every_growth_stage_tracks_the_bed_centre(stage):
    """Centred and off-centre/angled: bias small (an under-read proportional
    to the offset is allowed — it does not move the controller's zero), noise
    small, heading right, confident, never a false row end."""
    f = OnionField(stage_config(stage), seed=0)
    for d, yaw in ((0.0, 0.0), (0.12, 4.0)):
        _, lat, hdg, conf, rowend, _ = _drive(f, d, yaw)
        tol = 0.03 if stage == "lodged" else 0.025      # a lodged canopy is ragged
        assert abs(lat.mean()) <= tol + 0.30 * abs(d), (stage, d, lat.mean())
        assert lat.std() <= 0.025, (stage, d, lat.std())
        assert np.abs(lat).max() <= 0.08, (stage, d, np.abs(lat).max())
        assert np.abs(hdg).max() <= 1.5, (stage, d, np.abs(hdg).max())
        assert conf.min() >= 0.8, (stage, d, conf.min())
        assert (rowend < 0.7).all(), (stage, d)


def test_crop_rows_steer_when_distinct_even_when_leafy():
    """Separate young rows give the precise comb fit (sub-cm); leafy rows whose
    leaves fan past the ±6 cm row window (52–59 % on the lines — the old 60 %
    rule refused them and the coarser profile steered) are still accepted by
    their on-row vs between-row concentration."""
    for st in (STAGES["young"], replace(STAGES["young"], elems=60, spread_top=0.07)):
        f = OnionField(FieldConfig(stage=st), seed=0)
        det, lat, _, _, _, modes = _drive(f, 0.05, 0.0, n=36, settle=20)
        assert modes.count("density") >= 0.8 * len(modes), (st, modes)
        assert np.abs(lat).max() < 0.02, (st, np.abs(lat).max())


def test_soil_floor_measured_when_visible_and_held_under_canopy():
    """The bed-top soil is the LOWEST substantial surface (not the dominant
    one — dense leaves out-number soil returns); centred on the bed profile so
    an off-centre robot never samples its furrow; once the canopy hides it the
    last measured bed top is held instead of jumping onto the canopy."""
    young = OnionField(stage_config("young"), seed=0)
    mature = OnionField(stage_config("mature", bed_h=0.09), seed=0)
    det = _det()
    rng = np.random.default_rng(1)
    for i in range(20):
        det.update(young.scan(0.30, 0.02 * i, rng=rng))
    assert abs(det.last_bed_floor - 0.09) < 0.02 and det.last_soil_visible
    for i in range(20):
        det.update(mature.scan(0.30, 0.02 * i, rng=rng))
    assert not det.last_soil_visible
    assert abs(det.last_bed_floor - 0.09) < 0.02          # held, not the 0.57 m canopy


def test_bare_stretch_in_a_full_canopy_is_not_a_bed_end():
    """The bed profile includes the crop, so the presence baseline learnt in a
    full canopy is large; a long stand gap of BARE bed must not read as the
    bed ending (absolute relief floor), while the headland still does."""
    mature = OnionField(stage_config("mature"), seed=0)
    bare = OnionField(stage_config("bare", bed_h=0.07), seed=0)
    det = _det()
    rng = np.random.default_rng(2)
    for i in range(40):
        det.update(mature.scan(0.05, 0.02 * i, rng=rng))
    for i in range(30):
        e = det.update(bare.scan(0.05, 0.02 * i, rng=rng))
        assert det.last_bed_present and e.confidence > 0.5, i


@pytest.mark.parametrize("stage", ["bare", "mature", "lodged"])
def test_bed_end_found_and_grass_never_followed(stage):
    f = OnionField(stage_config(stage, y_end=8.5), seed=0)
    det = _det()
    rng = np.random.default_rng(3)
    for i in range(32):                                  # establish the bed (latch the gate)
        det.update(f.scan(0.05, 0.0, rng=rng))
    lost_at, after, yr = None, [], 3.5
    while yr < 10.0:
        e = det.update(f.scan(0.05, yr, rng=rng))
        if lost_at is None and e.confidence < 0.35:
            lost_at = 8.5 - yr
        elif lost_at is not None:
            after.append(e.confidence)
        yr += 0.04                                        # 0.4 m/s at 10 Hz
    assert lost_at is not None and 0.5 <= lost_at <= 4.0, (stage, lost_at)
    assert max(after) < 0.35, (stage, max(after))


@pytest.mark.parametrize("stage", ["bare", "mature"])
def test_reentry_after_u_turn_at_large_offset_and_angle(stage):
    """After a U-turn the detector is reset and the next bed is entered up to
    ~0.35 m off and ~12° angled.  Was: an 8° heading cap kept the box 2–4° off
    (17–22 cm lateral error) and, with the axis rate-limited against the HELD
    output, deadlocked at zero confidence."""
    f = OnionField(stage_config(stage), seed=0)
    for d, yaw in ((0.30, 10.0), (-0.35, -8.0), (0.20, -12.0)):
        _, lat, hdg, conf, _, _ = _drive(f, d, yaw, n=30, settle=15)
        assert conf.min() >= 0.8, (stage, d, yaw, conf.min())
        assert abs(lat.mean()) <= 0.03 + 0.30 * abs(d), (stage, d, yaw, lat.mean())
        assert np.abs(hdg).max() <= 2.5, (stage, d, yaw)


def test_field_realities_weeds_low_bed_tilt_residual_on_a_bare_bed():
    """Bed-axis search is terrain-relative: an absolute height band lost a 4 cm
    bed, let a 1.5° tilt residual ramp far furrows into it, and let weeds
    dominate it (axis ±5–9° → 12–25 cm)."""
    cases = [(dict(weeds_per_m2=1.5), None), (dict(bed_h=0.04), None),
             ({}, SensorConfig(tilt_err_deg=1.5))]
    for kw, sens in cases:
        f = OnionField(stage_config("bare", **kw), seed=0)
        _, lat, _, conf, _, _ = _drive(f, 0.08, 0.0, sensor=sens)
        assert abs(lat.mean()) < 0.03 and np.abs(lat).max() < 0.06, (kw, lat.mean())
        assert conf.min() > 0.5, kw


def test_lost_azimuth_sector_is_not_read_as_a_furrow():
    """VLP-16 packet loss drops a whole azimuth sector; return-less columns
    there must not be taken for furrows — the scan gives no profile and the
    dropout hold coasts."""
    f = OnionField(stage_config("mid"), seed=0)

    def drop(P, rng):
        az = np.degrees(np.arctan2(P[:, 0], P[:, 1]))
        return P[(az < 5.0) | (az > 30.0)] if rng.random() < 0.5 else P

    det, lat, _, conf, _, _ = _drive(f, 0.05, 0.0, n=30, hook=drop)
    assert np.abs(lat).max() < 0.06 and conf.min() > 0.6
    rng = np.random.default_rng(0)
    P = f.scan(0.05, 0.0, rng=rng)
    az = np.degrees(np.arctan2(P[:, 0], P[:, 1]))
    det.update(P[(az < 5.0) | (az > 30.0)])
    assert det.last_sector_gap and det._shape is None


def test_canopy_at_lidar_height_lowers_confidence_instead_of_steering_blind():
    """Tops within ~7 cm of the LiDAR: nearly every downward beam hits leaves
    inside the self-filter and the view collapses.  Was: an unchecked crop fit
    steered at −17…−24° / 34 cm off with confidence ~0.85.  (0.66 m plants
    under the old 0.80 m mount; the limit moves up with the mount.)"""
    tall = LIDAR_MOUNT_HEIGHT - 0.14
    f = OnionField(FieldConfig(stage=replace(STAGES["mature"], height=tall)), seed=0)
    det, lat, hdg, conf, _, _ = _drive(f, 0.12, 0.0, n=30, settle=12)
    assert conf.max() < 0.45
    assert np.abs(hdg).max() < 6.0
    # by the view-quality gauge or by the bed fit giving up entirely
    assert det.last_view_quality < 0.6 or conf.max() < 0.1


def test_canopy_adaptive_obstacle_height():
    """Tall mature canopy raises the obstacle height above the crop (no false
    stops from leaf flutter); a person still stops the robot."""
    f = OnionField(FieldConfig(stage=replace(STAGES["mature"], height=0.60)), seed=0)
    det = _det()
    saf = SafetyMonitor(obstacle_height=0.75, tire_obstacle_height=0.85,
                        tire_track=0.959, canopy_clearance=0.15)
    fixed = SafetyMonitor(obstacle_height=0.75, tire_obstacle_height=0.85, tire_track=0.959)
    rng = np.random.default_rng(4)
    for i in range(30):
        det.update(f.scan(0.0, 0.05 * i, rng=rng))
    assert det.last_canopy_top is not None and 0.6 < det.last_canopy_top < 0.75
    P = f.scan(0.0, 1.5, rng=rng)
    tall_leaves = P.copy()
    near = (tall_leaves[:, 1] < 2.5) & (np.abs(tall_leaves[:, 0]) < 0.5)
    tall_leaves[near, 2] += 0.08                       # flutter above 0.75 m
    assert fixed.check(tall_leaves).forward_blocked
    assert not saf.check(tall_leaves, canopy_top=det.last_canopy_top).forward_blocked
    person = np.column_stack((rng.normal(0, 0.12, 300), rng.normal(2.0, 0.08, 300),
                              rng.uniform(0.2, 1.7, 300) - LIDAR_MOUNT_HEIGHT))
    assert saf.check(np.vstack([P, person]), canopy_top=det.last_canopy_top).forward_blocked
    assert saf.last_forward_height <= 0.95
    # off by default: soybean thresholds unchanged
    assert SafetyMonitor().canopy_clearance == 0.0


def test_simulator_reproduces_the_visibility_regimes():
    """The physics the rework relies on: transplants invisible, soil hidden
    from mid-season, wheel furrows shadowed (return-less) under a canopy."""
    rng = np.random.default_rng(5)

    def soil_frac(stage):
        f = OnionField(stage_config(stage), seed=0)
        P = f.scan(0.0, 0.0, rng=rng)
        h = P[:, 2] + LIDAR_MOUNT_HEIGHT
        m = (P[:, 1] > 1.5) & (P[:, 1] < 7.0) & (np.abs(P[:, 0]) < 0.5)
        return float(np.mean(np.abs(h[m] - f.cfg.stage.bed_h) < 0.03))

    assert soil_frac("seedling") > 0.9
    assert soil_frac("mid") < 0.15
    assert soil_frac("mature") < 0.02


def test_turn_hooks_release_the_bed_end_latch_and_free_the_axis():
    """Entering a headland U-turn must clear the 'bed ending' latch (it scales
    confidence ×0.3 and would block the perception-closed turn ending) and let
    the bed axis follow the raw search while the robot rotates."""
    f = OnionField(stage_config("bare", y_end=10.0), seed=0)
    det = _det()
    rng = np.random.default_rng(6)
    for i in range(40):
        det.update(f.scan(0.0, 0.0, rng=rng))
    yr = 0.0
    while not det._bed_ending and yr < 9.0:
        det.update(f.scan(0.0, yr, rng=rng))
        yr += 0.04
    assert det._bed_ending and yr > 5.0        # latched ~3–4 m before the end, not at once
    det.begin_turn()
    assert not det._bed_ending and det._turning
    det.end_turn()
    assert not det._turning and det._bed_axis is None


def test_navigator_calls_turn_hooks(monkeypatch):
    import sys
    import types
    for m in ("farm_ng", "farm_ng.canbus", "farm_ng.canbus.canbus_pb2", "farm_ng.canbus.packet",
              "farm_ng.core", "farm_ng.core.event_client", "farm_ng.core.event_service_pb2",
              "farm_ng.core.uri_pb2", "farm_ng.filter", "farm_ng.filter.filter_pb2"):
        if m not in sys.modules:
            mod = types.ModuleType(m)
            mod.__getattr__ = lambda name: type(name, (), {})
            monkeypatch.setitem(sys.modules, m, mod)
    from navigation.row_navigator import RowNavigator, _S

    class _Det:
        def __init__(self):
            self.calls = []

        def begin_turn(self):
            self.calls.append("begin")

        def end_turn(self):
            self.calls.append("end")

        def reset(self):
            self.calls.append("reset")

    det = _Det()
    nav = RowNavigator(None, det, SafetyMonitor(), None)
    nav._enter(_S.ROW_END)
    nav._enter(_S.HEADLAND)
    nav._enter(_S.APPROACH)
    assert det.calls[:2] == ["begin", "end"] and "reset" in det.calls


def test_bed_axis_trusts_count_cues_over_height_variance():
    """The height-variance cue is 5–17° off under a mature canopy but its
    peak/median ratio is larger, so 'clearest wins' let it drag the axis (U-turn
    re-entry from the 0.99 m mount); on a 4 cm bed the crop-band count is empty
    and 30° off.  Every single-scan search must stay within ~2°."""
    det = _det()
    for st, kw in (("mature", {}), ("bare", dict(bed_h=0.04)), ("lodged", {})):
        f = OnionField(stage_config(st, **kw), seed=0)
        rng = np.random.default_rng(3)
        for d, yaw in ((0.30, 10.0), (-0.20, -6.0)):
            for i in range(0, 24, 4):
                P, tr = f.scan(xr=d, yr=0.02 * i, yaw_deg=yaw, rng=rng, return_truth=True)
                a = det._bed_box_heading(P)
                assert a is not None and abs(math.degrees(a - tr["heading"])) < 2.6, (st, kw, d, yaw)


def test_poisoned_quarter_baseline_cannot_latch_a_false_bed_end():
    """Re-entering a mature bed angled (+0.30 m / +10°) right after a U-turn,
    one wild second-quarter profile seeded a ~51 cm relief baseline that, under
    the canopy, is rarely refreshed — every honest 10–18 cm reading then looked
    like the bed ending (confidence ×0.3).  An absolute 5 cm floor keeps it bed."""
    f = OnionField(stage_config("mature"), seed=0)
    det = _det()
    det._floor_lvl = 0.10                      # bed top tracked on the previous bed
    rng = np.random.default_rng(100)
    conf = []
    for i in range(40):
        P, tr = f.scan(xr=0.30, yr=0.02 * i, yaw_deg=10.0, rng=rng, return_truth=True)
        conf.append(det.update(P).confidence)
    assert not det._bed_ending and min(conf[15:]) > 0.8, min(conf[15:])
