#!/usr/bin/env python3
"""
run_metrics.py — Compute publication-grade performance metrics from a run's
per-scan telemetry (the JSONL written by TelemetryLogger).

Pure numpy + stdlib so it runs on the Jetson at run-end and offline in
``scripts/analyze_run.py``.  The metrics are the ones an autonomy paper reports:
cross-track / heading accuracy (over FOLLOW), control effort & jerk, per-row
breakdown, headland-turn outcomes, state-time budget, terrain, and event counts.

Everything is derived from the telemetry fields produced by
``RowNavigator._telemetry_record`` so there is a single source of truth.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional

import numpy as np


def load_jsonl(path: str | Path) -> List[dict]:
    """Read a JSON-Lines telemetry file into a list of dicts (robust to a
    partially-written last line)."""
    recs: List[dict] = []
    p = Path(path)
    if not p.exists():
        return recs
    with open(p) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                recs.append(json.loads(line))
            except json.JSONDecodeError:
                continue   # ignore a torn final line
    return recs


def _arr(records: List[dict], key: str, default: float = 0.0) -> np.ndarray:
    return np.array([float(r.get(key, default) or 0.0) for r in records], dtype=np.float64)


def _rmse(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else 0.0


def _stats(x: np.ndarray) -> dict:
    if len(x) == 0:
        return {"rmse": 0.0, "mae": 0.0, "p95": 0.0, "max": 0.0, "mean": 0.0, "std": 0.0}
    ax = np.abs(x)
    return {
        "rmse": _rmse(x),
        "mae": float(np.mean(ax)),
        "p95": float(np.percentile(ax, 95)),
        "max": float(np.max(ax)),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
    }


def _turns(records: List[dict]) -> List[dict]:
    """Extract one summary per U-turn from contiguous HEADLAND/ARC scans."""
    turns: List[dict] = []
    cur: Optional[dict] = None
    for r in records:
        in_arc = r.get("state") == "HEADLAND" and r.get("turn_phase") == "ARC"
        if in_arc:
            t = float(r.get("t", 0.0))
            rot = float(r.get("turn_rot_deg", 0.0) or 0.0)
            arc = float(r.get("turn_arc_m", 0.0) or 0.0)
            if cur is None:
                cur = {"dir": r.get("turn_dir", "?"), "src": r.get("turn_src", "?"),
                       "t_start": t, "t_end": t, "max_rot_deg": rot,
                       "final_rot_deg": rot, "arc_m": arc, "n_scans": 1}
            else:
                cur["t_end"] = t
                cur["max_rot_deg"] = max(cur["max_rot_deg"], rot)
                cur["final_rot_deg"] = rot
                cur["arc_m"] = max(cur["arc_m"], arc)
                cur["src"] = r.get("turn_src", cur["src"])
                cur["n_scans"] += 1
        elif cur is not None:
            # The turn ended; "completed" if it handed back to a moving state.
            cur["completed"] = r.get("state") in ("FOLLOW", "APPROACH")
            cur["duration_s"] = round(cur["t_end"] - cur["t_start"], 2)
            turns.append(cur)
            cur = None
    if cur is not None:
        cur["completed"] = False
        cur["duration_s"] = round(cur["t_end"] - cur["t_start"], 2)
        turns.append(cur)
    for i, t in enumerate(turns):
        t["idx"] = i
        for k in ("max_rot_deg", "final_rot_deg", "arc_m"):
            t[k] = round(t[k], 2)
    return turns


def _per_row(records: List[dict]) -> List[dict]:
    """Tracking accuracy broken down by row (grouped by rows_done over FOLLOW)."""
    rows: dict[int, List[dict]] = {}
    for r in records:
        if r.get("state") != "FOLLOW":
            continue
        rows.setdefault(int(r.get("rows_done", 0)), []).append(r)
    out = []
    for row in sorted(rows):
        rs = rows[row]
        lat = _arr(rs, "lateral")
        hd = _arr(rs, "heading_deg")
        dist = _arr(rs, "row_dist")
        out.append({
            "row": row,
            "n_scans": len(rs),
            "dist_m": round(float(dist.max()) if len(dist) else 0.0, 2),
            "xtrack_rmse_cm": round(_rmse(lat) * 100, 2),
            "xtrack_max_cm": round(float(np.max(np.abs(lat))) * 100, 2) if len(lat) else 0.0,
            "heading_rmse_deg": round(_rmse(hd), 2),
        })
    return out


def compute_run_metrics(records: List[dict], *, coverage: Optional[dict] = None) -> dict:
    """Compute the full metrics dict for a run from its telemetry records."""
    n = len(records)
    if n == 0:
        return {"n_scans": 0}

    t = _arr(records, "t")
    duration = float(t.max() - t.min()) if n > 1 else 0.0
    states = [r.get("state", "?") for r in records]
    follow = [r for r in records if r.get("state") == "FOLLOW"]

    lat = _arr(follow, "lateral")
    hd = _arr(follow, "heading_deg")
    spd = _arr(follow, "lin_cmd")
    ang = _arr(follow, "ang_cmd")
    jerk = float(np.mean(np.abs(np.diff(ang)))) if len(ang) > 1 else 0.0

    # State-time budget.
    uniq, counts = np.unique(np.array(states), return_counts=True)
    state_frac = {str(s): round(float(c) / n, 4) for s, c in zip(uniq, counts)}

    # Event counts from state transitions.
    obstacle_stops = follow_losses = 0
    for a, b in zip(states[:-1], states[1:]):
        if b == "OBSTACLE_WAIT" and a != "OBSTACLE_WAIT":
            obstacle_stops += 1
        if a == "FOLLOW" and b == "ACQUIRE":
            follow_losses += 1
    dropout_scans = int(np.sum(_arr(records, "n") == 0))

    grade = _arr(records, "grade_deg")
    drop = _arr(records, "drop_m")

    # Distance: prefer the drift-corrected SLAM path; else sum per-row maxima.
    if coverage and coverage.get("path_length_m"):
        distance = float(coverage["path_length_m"])
    else:
        per_row_dist = {}
        for r in records:
            per_row_dist[int(r.get("rows_done", 0))] = max(
                per_row_dist.get(int(r.get("rows_done", 0)), 0.0),
                float(r.get("row_dist", 0.0) or 0.0))
        distance = float(sum(per_row_dist.values()))

    turns = _turns(records)
    metrics = {
        "n_scans": n,
        "duration_s": round(duration, 2),
        "scan_rate_hz": round(n / duration, 2) if duration > 0 else 0.0,
        "distance_m": round(distance, 2),
        "rows_completed": int(max((r.get("rows_done", 0) for r in records), default=0)),
        "rows_total": int(max((r.get("rows_total", 0) for r in records), default=0)),
        "state_fraction": state_frac,
        "follow": {
            "n_scans": len(follow),
            "xtrack_rmse_cm": round(_stats(lat)["rmse"] * 100, 2),
            "xtrack_mae_cm": round(_stats(lat)["mae"] * 100, 2),
            "xtrack_p95_cm": round(_stats(lat)["p95"] * 100, 2),
            "xtrack_max_cm": round(_stats(lat)["max"] * 100, 2),
            "heading_rmse_deg": round(_stats(hd)["rmse"], 2),
            "heading_mae_deg": round(_stats(hd)["mae"], 2),
            "heading_max_deg": round(_stats(hd)["max"], 2),
            "speed_mean_ms": round(float(np.mean(spd)) if len(spd) else 0.0, 3),
            "speed_std_ms": round(float(np.std(spd)) if len(spd) else 0.0, 3),
            "control_effort": round(float(np.mean(np.abs(ang))) if len(ang) else 0.0, 4),
            "control_jerk": round(jerk, 4),
        },
        "terrain": {
            "grade_mean_deg": round(float(np.mean(np.abs(grade))) if n else 0.0, 2),
            "grade_max_deg": round(float(np.max(np.abs(grade))) if n else 0.0, 2),
            "drop_mean_m": round(float(np.mean(np.abs(drop))) if n else 0.0, 2),
            "drop_max_m": round(float(np.max(np.abs(drop))) if n else 0.0, 2),
        },
        "events": {
            "obstacle_stops": obstacle_stops,
            "follow_losses": follow_losses,
            "dropout_scans": dropout_scans,
        },
        "turns": turns,
        "n_turns": len(turns),
        "turns_completed": int(sum(1 for t in turns if t.get("completed"))),
        "per_row": _per_row(records),
    }
    if coverage:
        metrics["coverage"] = coverage
    return metrics
