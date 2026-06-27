#!/usr/bin/env -S python3 -u
"""
analyze_run.py — Turn a recorded run folder into publication-ready tables.

Reads a ``runs/run_<ts>/`` folder (or any folder/telemetry.jsonl produced by
``row_follow.py --record`` / ``--telemetry``), (re)computes the performance
metrics, prints a summary table, and writes/refreshes:

    summary.json        full metrics (nested)
    metrics_flat.csv    key,value scalar metrics  (spreadsheet / LaTeX)
    per_row.csv         tracking accuracy per row
    turns.csv           one row per headland U-turn

Optionally aggregates SEVERAL runs into one comparison CSV (e.g. pursuit vs mpc):

    python3 scripts/analyze_run.py runs/run_A runs/run_B --compare results/field_compare.csv

Pure numpy + stdlib — runs on the brain; no pandas/matplotlib required.  Make
the figures yourself from the CSVs (the columns are figure-ready).
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.run_metrics import compute_run_metrics, load_jsonl


def _resolve_telemetry(path: Path) -> Path:
    """Accept a run dir, a telemetry.jsonl, or a bare .jsonl."""
    if path.is_dir():
        return path / "telemetry.jsonl"
    return path


def _load_coverage(run_dir: Path) -> dict | None:
    """Pull coverage stats from a sibling summary.json or map.npz if present."""
    summ = run_dir / "summary.json"
    if summ.exists():
        try:
            cov = json.loads(summ.read_text()).get("coverage")
            if cov:
                return cov
        except Exception:
            pass
    return None


def analyse_one(path: Path, *, write: bool = True) -> dict | None:
    tpath = _resolve_telemetry(path)
    records = load_jsonl(tpath)
    if not records:
        print(f"  [{path}] no telemetry found at {tpath}")
        return None
    run_dir = tpath.parent
    metrics = compute_run_metrics(records, coverage=_load_coverage(run_dir))
    metrics["run_id"] = run_dir.name

    if write:
        (run_dir / "summary.json").write_text(json.dumps(metrics, indent=2))
        _flat_csv(run_dir / "metrics_flat.csv", metrics)
        _table_csv(run_dir / "per_row.csv", metrics.get("per_row", []))
        _table_csv(run_dir / "turns.csv", metrics.get("turns", []))

    _print(metrics)
    return metrics


def _flat_csv(path: Path, metrics: dict) -> None:
    rows = []

    def walk(prefix, obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                walk(f"{prefix}.{k}" if prefix else k, v)
        elif isinstance(obj, (list, tuple)):
            return
        else:
            rows.append((prefix, obj))

    walk("", metrics)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "value"])
        w.writerows(rows)


def _table_csv(path: Path, rows: list) -> None:
    if not rows:
        return
    cols = list({k for r in rows for k in r.keys()})
    head = [c for c in ("idx", "row", "dir", "src") if c in cols]
    cols = head + [c for c in cols if c not in head]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c) for c in cols})


def _print(m: dict) -> None:
    f = m.get("follow", {})
    print(f"\n=== {m.get('run_id', '?')} ===")
    print(f"  {m.get('duration_s', 0):.0f}s  {m.get('n_scans', 0)} scans  "
          f"{m.get('distance_m', 0):.1f} m  rows {m.get('rows_completed', 0)}/"
          f"{m.get('rows_total', 0)}  turns {m.get('turns_completed', 0)}/{m.get('n_turns', 0)}")
    print(f"  FOLLOW  xtrack RMSE {f.get('xtrack_rmse_cm', 0):.1f} cm "
          f"(p95 {f.get('xtrack_p95_cm', 0):.1f}, max {f.get('xtrack_max_cm', 0):.1f})  "
          f"heading RMSE {f.get('heading_rmse_deg', 0):.1f}°  jerk {f.get('control_jerk', 0):.3f}")
    sf = m.get("state_fraction", {})
    print("  state %: " + "  ".join(f"{k} {v*100:.0f}" for k, v in sorted(sf.items())))
    for t in m.get("turns", []):
        print(f"    turn {t['idx']} [{t.get('dir','?')}/{t.get('src','?')}]: "
              f"{t.get('final_rot_deg',0):.0f}° rot, {t.get('arc_m',0):.1f} m arc, "
              f"{'completed' if t.get('completed') else 'INCOMPLETE'}")


_COMPARE_COLS = [
    "run_id", "controller", "duration_s", "distance_m", "rows_completed",
    "n_turns", "turns_completed",
    "follow.xtrack_rmse_cm", "follow.xtrack_p95_cm", "follow.xtrack_max_cm",
    "follow.heading_rmse_deg", "follow.control_jerk", "follow.speed_mean_ms",
    "coverage.covered_area_m2", "coverage.redundancy",
]


def _dig(d: dict, dotted: str):
    cur = d
    for part in dotted.split("."):
        if not isinstance(cur, dict):
            return ""
        cur = cur.get(part)
        if cur is None:
            return ""
    return cur


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyse recorded run(s)")
    ap.add_argument("runs", nargs="+", help="run dir(s) or telemetry.jsonl file(s)")
    ap.add_argument("--no-write", action="store_true",
                    help="Print only; do not (re)write summary/CSV files into the run dir")
    ap.add_argument("--compare", metavar="CSV",
                    help="Also write one comparison row per run to this CSV "
                         "(controller A/B tables for the paper)")
    args = ap.parse_args()

    results = []
    for r in args.runs:
        m = analyse_one(Path(r), write=not args.no_write)
        if m is not None:
            results.append(m)

    if args.compare and results:
        Path(args.compare).parent.mkdir(parents=True, exist_ok=True)
        with open(args.compare, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(_COMPARE_COLS)
            for m in results:
                ctrl = (m.get("coverage", {}) or {}).get("controller", "")
                row = []
                for c in _COMPARE_COLS:
                    row.append(m.get("run_id", "") if c == "run_id"
                               else (ctrl if c == "controller" else _dig(m, c)))
                w.writerow(row)
        print(f"\ncomparison table → {args.compare}  ({len(results)} runs)")


if __name__ == "__main__":
    main()
