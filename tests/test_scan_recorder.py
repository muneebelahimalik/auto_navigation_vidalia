"""Tests for navigation.scan_recorder.ScanRecorder — raw-scan streaming.

No matplotlib / hardware required; just numpy + a temp dir.
"""

import csv
import time

import numpy as np

from navigation.scan_recorder import ScanRecorder


def _drain(rec, timeout=5.0):
    """Wait until the background writer has flushed its queue."""
    t0 = time.monotonic()
    while (rec.saved + rec.dropped) < rec._sub and time.monotonic() - t0 < timeout:
        time.sleep(0.01)


def test_saves_scans_and_index(tmp_path):
    rec = ScanRecorder(tmp_path / "scans", every=1)
    pts = np.random.rand(50, 3).astype(np.float32)
    for i in range(3):
        rec.submit(pts, {"t": float(i), "state": "FOLLOW", "lateral": 0.1 * i,
                         "heading_deg": 2.0, "conf": 0.8, "n": 50})
    _drain(rec)
    rec.stop()

    assert rec.saved == 3
    saved = sorted((tmp_path / "scans").glob("scan_*.npy"))
    assert len(saved) == 3
    back = np.load(saved[0])
    assert back.shape == (50, 3)

    with open(tmp_path / "scans" / "index.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3
    assert rows[0]["state"] == "FOLLOW"
    assert rows[0]["file"] == "scan_00000.npy"
    assert int(rows[0]["n"]) == 50


def test_downsample_every(tmp_path):
    rec = ScanRecorder(tmp_path / "scans", every=5)
    pts = np.zeros((10, 3), dtype=np.float32)
    for _ in range(20):
        rec.submit(pts, {})
    _drain(rec)
    rec.stop()
    # 20 submissions, keep every 5th -> scans #5,10,15,20 = 4 saved.
    assert rec.saved == 4


def test_empty_and_none_scans_skipped(tmp_path):
    rec = ScanRecorder(tmp_path / "scans", every=1)
    rec.submit(None, {})
    rec.submit(np.empty((0, 3), dtype=np.float32), {})
    rec.submit(np.ones((5, 3), dtype=np.float32), {})
    _drain(rec)
    rec.stop()
    assert rec.saved == 1


def test_stop_is_idempotent(tmp_path):
    rec = ScanRecorder(tmp_path / "scans", every=1)
    rec.submit(np.ones((3, 3), dtype=np.float32), {})
    _drain(rec)
    rec.stop()
    rec.stop()  # must not raise
    assert rec.saved == 1
