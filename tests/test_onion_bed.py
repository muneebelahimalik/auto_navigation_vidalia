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


def _scene(rng, d, rows=ROWS4, bed_h=0.15, n_row=170, n_gnd=900, angle_deg=0.0):
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


def test_bare_raised_bed_is_a_row_end_not_a_row():
    """Bed-top soil sits inside the absolute crop band; measured from the bed
    surface it is excluded, so an empty bed reads as a row end."""
    rng = np.random.default_rng(3)
    det = _bed_det()
    e = _settle(det, rng, 0.05)
    assert e.row_end_confidence < 0.2 and e.confidence > 0.8
    assert 0.10 < det.last_bed_floor < 0.20                  # found the 0.15 m bed top
    for _ in range(12):
        e = det.update(_scene(rng, 0.05, rows=[]))
    assert e.row_end_confidence > 0.9
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
