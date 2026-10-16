"""Raised-bed (onion) mode — RowDetector(bed_rows=N).

Vidalia onion layout: the robot straddles a 72 in (1.83 m) raised bed carrying
4 onion rows at 11 in (0.279 m); the wheels run in the furrows.  The target is
the BED centre.  These tests lock in:

  * the 4-row comb fit lands on the bed centre for any start offset up to
    ~±0.35 m (the old single-row and 2-peak pairing modes lock onto the wrong
    rows ~14–28 cm off — a wheel on the bed);
  * a bare raised bed (no crop) is NOT a row — the crop band is measured from
    the bed-top soil, so a row end is detected;
  * a planter skip in one row does not hop the lock;
  * an angled bed is still fit correctly (heading-aligned ROI box);
  * the soybean path is untouched (bed_rows=0 default).
"""
import math

import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
from navigation.row_perception import RowDetector, find_bed_centre

SP = 0.2794                                     # 11 in
ROWS4 = [-1.5 * SP, -0.5 * SP, 0.5 * SP, 1.5 * SP]
PITCH = 1.83                                    # 72 in bed centres


def _bed_h(xr, bed_h=0.15, top=1.10, shoulder=0.20):
    a = np.abs(xr)
    h = np.where(a <= top / 2, bed_h, 0.0)
    ramp = (a > top / 2) & (a < top / 2 + shoulder)
    return np.where(ramp, bed_h * (1 - (a - top / 2) / shoulder), h)


def _scene(rng, d, rows=ROWS4, bed_h=0.15, n_row=170, n_gnd=6000, angle_deg=0.0):
    """Robot d m RIGHT of the bed centre (bed centre at x = −d), optionally yawed."""
    parts = []
    for bc in (-PITCH, 0.0, PITCH):
        xg = rng.uniform(bc - PITCH / 2, bc + PITCH / 2, n_gnd // 3)
        yg = rng.uniform(1.5, 7.0, len(xg))
        hg = _bed_h(xg - bc, bed_h) + rng.normal(0, 0.015, len(xg))
        parts.append(np.column_stack((xg - d, yg, hg)))
        for rx in rows:
            y = rng.uniform(1.5, 7.0, n_row)
            x = bc + rx + rng.normal(0, 0.04, n_row)
            h = bed_h + rng.uniform(0.10, 0.40, n_row)
            parts.append(np.column_stack((x - d, y, h)))
    P = np.vstack(parts)
    if angle_deg:
        t = math.radians(angle_deg)
        c, s = math.cos(t), math.sin(t)
        P[:, 0], P[:, 1] = c * P[:, 0] - s * P[:, 1], s * P[:, 0] + c * P[:, 1]
    P[:, 2] -= LIDAR_MOUNT_HEIGHT
    return P


def _bed_det(**kw):
    return RowDetector(dual_row=True, bed_rows=4, row_spacing=SP,
                       crop_h_min=0.05, crop_h_max=0.60, **kw)


def _settle(det, rng, d, n=25, **kw):
    e = None
    for _ in range(n):
        e = det.update(_scene(rng, d, **kw))
    return e


def test_bed_comb_finds_bed_centre_across_start_offsets():
    rng = np.random.default_rng(0)
    for d in (0.0, 0.10, -0.15, 0.20, -0.30):
        e = _settle(_bed_det(), rng, d)
        assert abs(e.lateral_offset - (-d)) < 0.03, (d, e.lateral_offset)
        assert e.confidence > 0.8


def test_single_row_mode_is_wrong_on_a_four_row_bed():
    """Documents WHY bed mode exists: nearest-peak locks onto an inner onion row."""
    rng = np.random.default_rng(1)
    e = _settle(RowDetector(dual_row=False, crop_h_min=0.05, crop_h_max=0.60), rng, 0.10)
    assert abs(e.lateral_offset - (-0.10)) > 0.08


def test_two_peak_pairing_aliases_where_comb_does_not():
    """An inner row inside the ±0.05 m pairing dead-band → wrong pair; comb fine."""
    rng = np.random.default_rng(2)
    pair = RowDetector(dual_row=True, row_spacing=SP, crop_h_min=0.05, crop_h_max=0.60)
    e_pair = _settle(pair, rng, 0.12)
    e_bed = _settle(_bed_det(), rng, 0.12)
    assert abs(e_pair.lateral_offset + 0.12) > 0.10
    assert abs(e_bed.lateral_offset + 0.12) < 0.03


def test_bare_raised_bed_soil_is_not_crop_and_flat_headland_is_row_end():
    """Bed-top soil sits inside the absolute crop band; measured from the bed
    surface it is excluded (no soil-as-crop).  A bare RAISED bed is then
    followed by its relief, and the row end is where the relief disappears
    (flat headland)."""
    rng = np.random.default_rng(3)
    det = _bed_det()
    e = _settle(det, rng, 0.05)
    assert e.row_end_confidence < 0.2 and e.confidence > 0.8
    assert 0.10 < det.last_bed_floor < 0.20                  # found the 0.15 m bed top
    for _ in range(12):
        e = det.update(_scene(rng, 0.05, rows=[], n_gnd=6000))
    assert e.n_points < 40                                   # soil not counted as crop
    assert det.last_mode == "shape" and abs(e.lateral_offset + 0.05) < 0.03
    for _ in range(12):                                      # flat headland: no bed, no crop
        e = det.update(_scene(rng, 0.05, rows=[], bed_h=0.0, n_gnd=6000))
    assert e.row_end_confidence > 0.7                        # navigator row-end threshold
    assert e.confidence < 0.35                               # below FOLLOW threshold


def test_planter_skip_does_not_hop_the_lock():
    rng = np.random.default_rng(4)
    for drop in (0, 1, 3):
        det = _bed_det()
        _settle(det, rng, 0.0)
        rows = [r for i, r in enumerate(ROWS4) if i != drop]
        for _ in range(30):
            e = det.update(_scene(rng, 0.0, rows=rows))
            assert abs(e.lateral_offset) < 0.04, (drop, e.lateral_offset)


def test_missing_outer_row_at_offset_start_uses_soil_edges():
    """Three visible rows fit two comb positions; the bed's soil edges decide."""
    rng = np.random.default_rng(5)
    e = _settle(_bed_det(), rng, -0.20, rows=ROWS4[1:])
    assert abs(e.lateral_offset - 0.20) < 0.04


def test_angled_bed_is_fit_with_aligned_roi():
    """At 5–8° to a 0.28 m-spaced bed the fixed ROI box cut rows diagonally
    (lateral bias −8…−15 cm); the heading-aligned box removes it."""
    rng = np.random.default_rng(6)
    for ang, d in ((5.0, 0.0), (8.0, 0.0), (-8.0, 0.15)):
        det = _bed_det()
        lats = []
        for i in range(30):
            e = det.update(_scene(rng, d, angle_deg=ang))
            if i >= 10:
                lats.append(e.lateral_offset)
        assert abs(np.mean(lats) - (-d)) < 0.03, (ang, d, np.mean(lats))
        assert abs(math.degrees(det.last_roi_rot) + ang) <= 1.5


def test_sparse_young_onions_still_tracked():
    rng = np.random.default_rng(7)
    e = _settle(_bed_det(), rng, 0.10, n_row=30)
    assert abs(e.lateral_offset + 0.10) < 0.04


def test_find_bed_centre_extra_beats_miss():
    """An observed row no comb explains costs more than a predicted-but-absent
    row (planter skip), so a 3-of-4 view stays on the tracked bed."""
    rng = np.random.default_rng(8)
    cross = np.concatenate([rng.normal(r, 0.03, 80) for r in ROWS4[1:]])   # outer-left missing
    lat, sf = find_bed_centre(cross, 0.80, 0.05, SP, 4,
                              prior_lateral=0.0, prior_weight=2.5)
    assert abs(lat) < 0.03
    assert 0.5 <= sf < 1.0


def test_soybean_default_unaffected():
    """bed_rows defaults to 0 → the soybean dual-row path never runs bed code."""
    det = RowDetector(dual_row=True)
    assert det.bed_rows == 0
    rng = np.random.default_rng(9)
    pts = []
    for c in (-0.38, 0.38):
        y = rng.uniform(1.6, 6.0, 150)
        pts.append(np.column_stack((c + rng.normal(0, 0.04, 150), y,
                                    rng.uniform(0.05, 0.25, 150) - LIDAR_MOUNT_HEIGHT)))
    e = None
    for _ in range(10):
        e = det.update(np.vstack(pts))
    assert abs(e.lateral_offset) < 0.03
    assert det.last_roi_rot == 0.0 and det.last_bed_floor == 0.0


def test_bare_bed_with_residual_slope_tracked_by_relief_not_soil():
    """Field failure (2026-10-15): on a BARE bed with a residual ~1.5–2° forward
    slope (near the 2° detrend dead-band), the far bed-top soil ramped into the
    bed-relative crop band and read as a row at off=+0.8 m, alternating with
    n=0 scan-to-scan.  Now the bed floor is a line in range (no soil-as-crop)
    and the bare bed is followed by its relief, centred and steady."""
    for deg in (1.5, 2.0, 2.5):
        rng = np.random.default_rng(10)
        det = _bed_det()
        lats = []
        for i in range(20):
            P = _scene(rng, 0.10, rows=[], n_gnd=6000)
            P[:, 2] += math.tan(math.radians(deg)) * P[:, 1]
            e = det.update(P)
            assert e.n_points < 80, (deg, e.n_points)      # (a few clod returns at most)
            if i >= 8:
                lats.append(e.lateral_offset)
        assert det.last_mode == "shape", deg
        assert abs(np.mean(lats) + 0.10) < 0.03 and np.std(lats) < 0.03, (deg, np.mean(lats))


def test_young_seedlings_invisible_bed_relief_steers():
    """The Vidalia field (2026-10-15): seedlings a few cm tall are invisible to
    the LiDAR; only the raised bed between the wheel furrows is.  Bed relief of
    5–12 cm must give a centred, confident estimate at any start offset."""
    for bed_h in (0.05, 0.08, 0.12):
        for d in (0.0, 0.15, -0.20):
            rng = np.random.default_rng(13)
            det = _bed_det()
            lats = []
            for i in range(25):
                e = det.update(_scene(rng, d, rows=[], bed_h=bed_h, n_gnd=9000))
                if i >= 10:
                    lats.append(e.lateral_offset)
            assert abs(np.mean(lats) + d) < 0.03, (bed_h, d, np.mean(lats))
            assert e.confidence > 0.8, (bed_h, d, e.confidence)


def test_planted_bed_with_residual_slope_steady():
    rng = np.random.default_rng(11)
    det = _bed_det()
    ns = []
    for i in range(20):
        P = _scene(rng, 0.10, n_gnd=6000)
        P[:, 2] += math.tan(math.radians(2.0)) * P[:, 1]
        e = det.update(P)
        if i >= 8:
            ns.append(e.n_points)
    assert abs(e.lateral_offset + 0.10) < 0.03
    assert max(ns) - min(ns) < 0.2 * np.mean(ns)          # no scan-to-scan flicker


def test_bed_offset_beyond_straddle_is_not_followed():
    """A fit that puts the bed centre beyond ±0.45 m (a wheel would be ON the
    bed) is a lock on something else — confidence must be 0."""
    rng = np.random.default_rng(12)
    det = _bed_det()
    e = None
    for _ in range(10):
        # single dense stripe 0.7 m to the right (e.g. neighbouring bed edge)
        y = rng.uniform(1.5, 7.0, 600)
        P = np.column_stack((0.70 + rng.normal(0, 0.03, 600), y,
                             0.15 + rng.uniform(0.1, 0.3, 600) - LIDAR_MOUNT_HEIGHT))
        e = det.update(P)
        assert e.confidence == 0.0 or abs(e.lateral_offset) <= det.bed_max_offset


# --- Bed end → grassy headland (field 2026-10-15) ---------------------------

def _end_scene(rng, d, y_end, bed_h=0.074):
    """Bare raised bed (onions invisible) that ENDS at y_end; beyond it flat
    headland with grass (0–25 cm) — the 2026-10-15 field row end."""
    P = _scene(rng, d, rows=[], bed_h=bed_h, n_gnd=9000)
    beyond = P[:, 1] > y_end
    P[beyond, 2] = -LIDAR_MOUNT_HEIGHT + rng.normal(0, 0.015, int(beyond.sum()))
    if y_end < 7.4:
        m = 4000
        g = np.column_stack((rng.uniform(-2.5, 2.5, m), rng.uniform(max(y_end, 1.5), 7.5, m),
                             rng.uniform(0.0, 0.25, m) - LIDAR_MOUNT_HEIGHT))
        P = np.vstack([P, g[g[:, 1] > y_end]])
    return P


def test_bed_end_into_grass_is_a_row_end_and_grass_is_never_followed():
    """Field failure: at the row end the relief faded, the heading crept to the
    20° box limit (robot started turning), and then the crop fit FOLLOWED THE
    GRASS for 6 m.  Now: heading stays bounded on the approach, confidence
    drops (row end) while the bed end is still ~3 m ahead, and the grass never
    regains a lock."""
    for d in (0.0, 0.10, -0.10):
        rng = np.random.default_rng(21)
        det = _bed_det()
        y_end, ended_at, after = 12.0, None, []
        while y_end > -2.0:
            e = det.update(_end_scene(rng, d, y_end))
            if 4.0 < y_end < 9.0:                       # approaching the end
                assert abs(math.degrees(e.heading_error)) < 4.0, (d, y_end)
                assert abs(e.lateral_offset + d) < 0.08, (d, y_end, e.lateral_offset)
            elif 3.2 < y_end <= 4.0 and e.confidence >= 0.35:   # final stretch
                assert abs(math.degrees(e.heading_error)) < 6.0, (d, y_end)
                assert abs(e.lateral_offset + d) < 0.15, (d, y_end, e.lateral_offset)
            if ended_at is None and y_end < 9.0 and e.confidence < 0.35:
                ended_at = y_end
            elif ended_at is not None:
                after.append(e.confidence)
            y_end -= 0.02
        assert ended_at is not None and 2.0 < ended_at < 4.5, (d, ended_at)
        assert max(after) < 0.35, (d, max(after))          # grass never followed
        assert det.last_mode == "no-bed"


def test_bed_presence_gate_on_real_field_contrast_trace():
    """Replays the bed-relief contrast logged on the 2026-10-15 Vidalia run
    (158 m; bed body to ~151.4 m, then grassy headland) through the real
    bed-presence gate: zero false 'no bed' scans over the 150 m body, and the
    row end confirmed (15 consecutive) by 152 m — the robot followed grass to
    158 m without it."""
    import os
    data = np.load(os.path.join(os.path.dirname(__file__), "data",
                                "field_20261015_onion_bed_contrast.npz"))
    c, dist = data["contrast"], data["row_dist"]
    det = RowDetector(dual_row=True, bed_rows=4, row_spacing=SP)
    present = []
    for v in c:
        det._shape = (0.0, 0.0, float(v)) if v > 0 else None
        present.append(det._bed_present())
    present = np.array(present)
    body = (dist > 1.0) & (dist < 151.0)
    assert not (~present[body]).any()
    run, end_dist = 0, None
    for i in range(len(c)):
        run = run + 1 if (not present[i] and dist[i] > 1.0) else 0
        if run >= 15:
            end_dist = float(dist[i])
            break
    assert end_dist is not None and 151.0 < end_dist < 152.5, end_dist


def test_gate_holds_through_relief_dropouts_on_all_2026_10_16_runs():
    """2026-10-16 field (MPC / pursuit / RL rows): the relief read ~0.07 m then
    exactly 0 (no shape fit) for 2–4 scans, then 0.07 again.  The old gate fed
    those zeros into its EMA and declared the bed ENDED 22–108 times per row
    (v=0 stops — the 'sudden brakes').  A missing fit is no evidence: the gate
    must hold through ≤ bed_dropout_hold scans.  Synthetic trace with the
    field's dropout pattern + a real bed end."""
    rng = np.random.default_rng(0)
    c = np.clip(rng.normal(0.072, 0.006, 3000), 0.04, None)
    # field-like dropouts: 2–7 consecutive zero scans every ~40 scans
    i = 200
    while i < 2700:
        c[i:i + rng.integers(2, 8)] = 0.0
        i += int(rng.integers(20, 60))
    c[2800:] = np.where(rng.random(200) < 0.5, 0.0, rng.uniform(0.0, 0.02, 200))  # bed end
    det = RowDetector(dual_row=True, bed_rows=4, row_spacing=SP)
    present = []
    for v in c:
        det._shape = (0.0, 0.0, float(v)) if v > 0 else None
        present.append(det._bed_present())
    present = np.array(present)
    assert present[:2800].all()
    first_absent = int(np.argmax(~present[2800:]))
    assert not present[2800:].all() and first_absent <= 15


def test_relief_dropout_coasts_instead_of_decaying():
    """A brief shape dropout with no crop must not decay confidence (the
    speed dip / FOLLOW-miss stop); a longer one must."""
    from navigation.row_perception import RowEstimate
    det = RowDetector(dual_row=True, bed_rows=4, row_spacing=SP)
    det._est = RowEstimate(heading_error=0.0, lateral_offset=0.02,
                           confidence=0.9, valid=True)
    det._last_shape = (0.02, 0.0, 0.07)          # bed tracked until now
    det._relief_sm, det._relief_base, det._relief_cnt = 0.07, 0.07, 100
    e1 = det.update(np.zeros((0, 3)))            # empty scan = shape dropout
    assert e1.confidence > 0.85 and det.last_mode == "shape"
    assert abs(e1.lateral_offset - 0.02) < 1e-6
    for _ in range(det.bed_dropout_hold + 4):
        e = det.update(np.zeros((0, 3)))
    assert e.confidence < 0.5
