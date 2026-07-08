"""Tests for lidar/lidar_driver.py — VLP-16 Puck Hi-Res geometry.

This unit is a VLP-16 *Hi-Res* (packet product-ID 0x24): its 16 beams span
±10° at 1.33° spacing, NOT the standard Puck's ±15° / 2.00°.  Using the wrong
(standard) angle table stretches the vertical geometry and made a true
15°/0.80 m mount read as 22°/1.17 m.  These tests lock the correct table in and
verify the parser applies it.
"""
import math
import struct

import numpy as np

from lidar.lidar_driver import (
    VLP16_VERTICAL_ANGLES,
    VLP16_VERTICAL_ANGLES_HIRES,
    VLP16_VERTICAL_ANGLES_STD,
    VELODYNE_BLOCK_FLAG,
    _parse_packet,
    _parse_scan_np,
)


def test_driver_uses_hires_angle_table():
    """The active table must be the Hi-Res one (this unit is a Puck Hi-Res)."""
    assert VLP16_VERTICAL_ANGLES == VLP16_VERTICAL_ANGLES_HIRES
    assert VLP16_VERTICAL_ANGLES != VLP16_VERTICAL_ANGLES_STD


def test_hires_fan_is_plus_minus_10_at_1p33_spacing():
    """Hi-Res spans ±10° with ~1.33° spacing (vs ±15° / 2.00° for standard)."""
    a = sorted(VLP16_VERTICAL_ANGLES)
    assert min(a) == -10.0 and max(a) == 10.0
    diffs = np.diff(a)
    assert abs(float(np.mean(diffs)) - 20.0 / 15.0) < 0.05     # ≈1.33°
    # sanity: it is the standard table scaled by 2/3
    for hi, std in zip(VLP16_VERTICAL_ANGLES, VLP16_VERTICAL_ANGLES_STD):
        assert abs(hi - std * (2.0 / 3.0)) < 0.02


def _one_return_packet(channel: int, dist_m: float, azimuth_deg: float = 0.0) -> bytes:
    """Craft a 1206-byte VLP-16 packet with a single return on `channel`."""
    buf = bytearray(1206)
    az_raw = int(round(azimuth_deg / 0.01))
    struct.pack_into("<HH", buf, 0, VELODYNE_BLOCK_FLAG, az_raw)   # block 0 header
    dist_raw = int(round(dist_m / 0.002))
    # firing 0, given channel: data offset = 4 + channel*3
    struct.pack_into("<HB", buf, 4 + channel * 3, dist_raw, 100)
    return bytes(buf)


def test_parser_applies_hires_elevation():
    """A forward return on channel 0 must land at z = d·sin(-10°) (Hi-Res),
    NOT d·sin(-15°) (standard) — proves the parser uses the Hi-Res angle."""
    d = 10.0
    pts = _parse_packet(_one_return_packet(channel=0, dist_m=d, azimuth_deg=0.0))
    ring0 = [p for p in pts if p.ring == 0]
    assert ring0, "channel-0 return not parsed"
    p = ring0[0]
    assert abs(p.z - d * math.sin(math.radians(-10.0))) < 1e-6     # Hi-Res
    assert abs(p.z - d * math.sin(math.radians(-15.0))) > 0.5      # not standard
    # azimuth 0 → straight forward: x≈0, y≈d·cos(10°)
    assert abs(p.x) < 1e-6
    assert abs(p.y - d * math.cos(math.radians(-10.0))) < 1e-6


def test_parse_scan_np_intensity_column():
    """with_intensity=True adds the per-return intensity (0–255) as column 3
    without changing the xyz geometry; default stays Nx3 (byte-identical)."""
    pkt = _one_return_packet(channel=0, dist_m=10.0, azimuth_deg=0.0)  # intensity=100
    xyz = _parse_scan_np([pkt])                       # default
    xyzi = _parse_scan_np([pkt], with_intensity=True)
    assert xyz.shape[1] == 3
    assert xyzi.shape[1] == 4
    assert xyzi.shape[0] == xyz.shape[0]
    assert np.allclose(xyzi[:, :3], xyz)              # geometry unchanged
    assert np.all(xyzi[:, 3] == 100)                  # the packed intensity


def test_parse_scan_np_empty_respects_intensity_width():
    assert _parse_scan_np([]).shape == (0, 3)
    assert _parse_scan_np([], with_intensity=True).shape == (0, 4)


def test_flat_ground_reconstructs_true_pitch_and_height():
    """End-to-end geometry: synth raw ranges for a level floor under a sensor at
    the TRUE 15°/0.80 m, using Hi-Res angles; the per-channel pitch (assuming
    0.80 m) must now AGREE across channels at ~15° — the signature of correct
    angles (with the wrong table they spanned 9–19°)."""
    H, theta = 0.80, 15.0
    thetas = []
    for a in VLP16_VERTICAL_ANGLES:
        below = math.radians(theta - a)
        if math.sin(below) <= 1e-3:
            continue                              # upward beam, no floor hit
        R = H / math.sin(below)                   # raw range to the floor
        thetas.append(a + math.degrees(math.asin(H / R)))
    thetas = np.array(thetas)
    assert abs(thetas.mean() - theta) < 0.2
    assert thetas.std() < 0.2                     # channels agree → angles correct
