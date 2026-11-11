"""pack_run.py: a recorded run survives pack → unpack (1 mm, files, index, text)."""
from __future__ import annotations

import csv
import json

import numpy as np

from scripts.pack_run import pack, unpack


def _fake_run(root, n=5, cols=3):
    run = root / "run_pursuit_20261112_101500"
    scans = run / "scans"
    scans.mkdir(parents=True)
    rng = np.random.default_rng(0)
    with open(scans / "index.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["file", "t", "state", "lateral", "heading_deg", "conf", "n"])
        for i in range(n):
            P = rng.uniform(-8, 8, size=(200 + i, cols)).astype(np.float32)
            if cols == 4:
                P[:, 3] = rng.integers(0, 256, size=len(P))
            P[0, :3] = [40.0, 0.0, 0.0]            # beyond 32 m → dropped
            np.save(scans / f"scan_{i:05d}.npy", P)
            w.writerow([f"scan_{i:05d}.npy", f"{i * 1.0:.2f}", "FOLLOW", "0.01", "0.5", "0.9", len(P)])
    (run / "telemetry.jsonl").write_text(json.dumps({"state": "FOLLOW"}) + "\n")
    (run / "manifest.json").write_text(json.dumps({"args": {"preset": "onion"}}))
    return run


def test_pack_unpack_roundtrip(tmp_path):
    run = _fake_run(tmp_path / "src")
    out = pack(run, tmp_path / "p.npz")
    back = unpack(out, tmp_path / "dst")
    assert back.name == run.name
    for i in range(5):
        a = np.load(run / "scans" / f"scan_{i:05d}.npy")[1:]
        b = np.load(back / "scans" / f"scan_{i:05d}.npy")
        assert b.shape == a.shape
        assert np.max(np.abs(a - b)) <= 0.0006
    assert (back / "scans" / "index.csv").read_text().splitlines()[0].startswith("file,t,")
    assert json.loads((back / "manifest.json").read_text())["args"]["preset"] == "onion"
    assert (back / "telemetry.jsonl").read_text() == (run / "telemetry.jsonl").read_text()


def test_pack_every_and_time_window_and_intensity(tmp_path):
    run = _fake_run(tmp_path / "src", n=10, cols=4)
    out = pack(run, tmp_path / "p.npz", every=2, t_lo=2.0, t_hi=7.0)
    back = unpack(out, tmp_path / "dst")
    names = sorted(p.name for p in (back / "scans").glob("scan_*.npy"))
    assert names == ["scan_00002.npy", "scan_00004.npy", "scan_00006.npy"]
    b = np.load(back / "scans" / "scan_00002.npy")
    a = np.load(run / "scans" / "scan_00002.npy")[1:]
    assert b.shape[1] == 4
    np.testing.assert_array_equal(b[:, 3], a[:, 3])
