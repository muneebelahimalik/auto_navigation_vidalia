#!/usr/bin/env python3
"""
test_run_metrics.py — Regression tests for the experiment-record metrics.

Locks the publication metrics computed from per-scan telemetry: FOLLOW
tracking accuracy, per-row breakdown, headland-turn extraction, state-time
budget, event counts, and the on-disk run-record bundle.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.run_metrics import compute_run_metrics, load_jsonl
from navigation.run_record import RunRecord


def _follow(t, lateral, heading_deg, rows_done=0, ang=0.0, n=400, lin=0.25):
    return {"t": t, "state": "FOLLOW", "lateral": lateral, "heading_deg": heading_deg,
            "rows_done": rows_done, "row_dist": t * 0.25, "ang_cmd": ang, "lin_cmd": lin,
            "n": n, "grade_deg": 3.0, "drop_m": -0.2, "rows_total": 6}


def _synthetic_run():
    recs = []
    tt = 0.0
    # Row 0: 10 FOLLOW scans, lateral alternating ±0.10 m, heading ±2°.
    for i in range(10):
        recs.append(_follow(tt, 0.10 if i % 2 else -0.10, 2.0 if i % 2 else -2.0, rows_done=0,
                            ang=0.1 if i % 2 else -0.1))
        tt += 0.1
    # ROW_END + a U-turn arc (IMU), 6 scans 0→175°, then back to FOLLOW.
    recs.append({"t": tt, "state": "ROW_END", "lateral": 0.0, "heading_deg": 0.0,
                 "rows_done": 0, "row_dist": 2.5, "n": 0, "rows_total": 6}); tt += 0.1
    for k, rot in enumerate(np.linspace(20, 175, 6)):
        recs.append({"t": tt, "state": "HEADLAND", "turn_phase": "ARC", "turn_dir": "R",
                     "turn_src": "imu", "turn_rot_deg": float(rot), "turn_arc_m": 0.5 + k * 0.6,
                     "lateral": 0.0, "heading_deg": 0.0, "rows_done": 1, "row_dist": 0.0,
                     "n": 0, "rows_total": 6}); tt += 0.1
    # Row 1: 8 FOLLOW scans, tighter tracking.
    for i in range(8):
        recs.append(_follow(tt, 0.05 if i % 2 else -0.05, 1.0 if i % 2 else -1.0, rows_done=1))
        tt += 0.1
    return recs


def test_follow_tracking_metrics():
    m = compute_run_metrics(_synthetic_run())
    f = m["follow"]
    # Row 0 lateral is ±0.10 → RMSE 10 cm; row 1 ±0.05 → pooled RMSE between.
    assert 7.0 < f["xtrack_rmse_cm"] < 10.1
    assert f["xtrack_max_cm"] == 10.0
    assert f["heading_rmse_deg"] > 0
    assert f["control_jerk"] > 0       # alternating ±0.1 → non-zero jerk
    assert m["n_scans"] == len(_synthetic_run())


def test_per_row_breakdown():
    m = compute_run_metrics(_synthetic_run())
    pr = {r["row"]: r for r in m["per_row"]}
    assert set(pr) == {0, 1}
    # Row 1 (±5 cm) must be tighter than row 0 (±10 cm).
    assert pr[1]["xtrack_rmse_cm"] < pr[0]["xtrack_rmse_cm"]
    assert pr[0]["n_scans"] == 10 and pr[1]["n_scans"] == 8


def test_turn_extraction():
    m = compute_run_metrics(_synthetic_run())
    assert m["n_turns"] == 1
    t = m["turns"][0]
    assert t["dir"] == "R" and t["src"] == "imu"
    assert t["final_rot_deg"] == 175.0
    assert t["completed"] is True        # handed back to FOLLOW
    assert m["turns_completed"] == 1


def test_state_budget_and_events():
    m = compute_run_metrics(_synthetic_run())
    sf = m["state_fraction"]
    assert abs(sum(sf.values()) - 1.0) < 1e-6
    assert "FOLLOW" in sf and "HEADLAND" in sf
    # One FOLLOW→ROW_END (not a loss); dropout scans = the n==0 rows.
    assert m["events"]["dropout_scans"] == 7   # 1 ROW_END + 6 ARC scans have n=0


def test_empty_run():
    assert compute_run_metrics([])["n_scans"] == 0


def test_load_jsonl_skips_torn_line(tmp_path):
    p = tmp_path / "telemetry.jsonl"
    p.write_text('{"state":"FOLLOW","t":0.0}\n{"state":"FOLLOW","t":0.1}\n{"bad":')
    recs = load_jsonl(p)
    assert len(recs) == 2          # torn final line ignored


def test_run_record_bundle(tmp_path):
    rd = tmp_path / "run_x"
    rec = RunRecord(rd, args={"controller": "mpc", "rows": 6},
                    calibration={"lidar_yaw_deg": 66.0})
    # manifest written immediately
    man = json.loads((rd / "manifest.json").read_text())
    assert man["args"]["controller"] == "mpc"
    assert man["calibration"]["lidar_yaw_deg"] == 66.0
    assert "git_commit" in man
    # write telemetry, then finalize
    with open(rec.telemetry_path, "w") as f:
        for r in _synthetic_run():
            f.write(json.dumps(r) + "\n")
    metrics = rec.finalize(coverage={"covered_area_m2": 120.0, "redundancy": 1.1,
                                     "path_length_m": 30.0, "controller": "mpc"})
    assert metrics is not None
    assert (rd / "summary.json").exists()
    assert (rd / "metrics_flat.csv").exists()
    assert (rd / "per_row.csv").exists()
    assert (rd / "turns.csv").exists()
    # coverage path length flows into distance
    assert metrics["distance_m"] == 30.0
    assert metrics["coverage"]["covered_area_m2"] == 120.0
