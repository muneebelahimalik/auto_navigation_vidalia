#!/usr/bin/env python3
"""
run_record.py — One self-contained, reproducible experiment folder per run.

``--record`` bundles everything needed to analyse a field run and publish from
it into ``runs/run_<ts>/``:

    manifest.json      config + git commit + calibration + CLI  (reproducibility)
    telemetry.jsonl    one row per scan (perception, command, safety, turn, …)
    summary.json       computed performance metrics (see run_metrics.py)
    metrics_flat.csv   key,value table for spreadsheets / LaTeX
    per_row.csv        tracking accuracy per row
    turns.csv          one row per headland U-turn (rotation, arc, source, …)
    map.png/.npz, coverage.png, trajectory.csv   (when SLAM is on)

The manifest is written at start (so a crashed run is still identifiable); the
summary/CSVs are written at exit from the telemetry.  Nothing here ever raises
into the control loop — failures are swallowed and reported.
"""

from __future__ import annotations

import csv
import json
import platform
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

from navigation.run_metrics import compute_run_metrics, load_jsonl


def _git(*args: str) -> str:
    try:
        out = subprocess.run(["git", *args], capture_output=True, text=True, timeout=5,
                             cwd=str(Path(__file__).resolve().parent.parent))
        return out.stdout.strip()
    except Exception:
        return ""


class RunRecord:
    def __init__(self, run_dir: str | Path, *, args: Optional[dict] = None,
                 calibration: Optional[dict] = None, extra: Optional[dict] = None) -> None:
        self.dir = Path(run_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.start = datetime.now()
        self.manifest = {
            "run_id": self.dir.name,
            "start_time": self.start.isoformat(timespec="seconds"),
            "git_commit": _git("rev-parse", "HEAD"),
            "git_dirty": bool(_git("status", "--porcelain")),
            "git_branch": _git("rev-parse", "--abbrev-ref", "HEAD"),
            "host": platform.node(),
            "python": sys.version.split()[0],
            "command": " ".join(sys.argv),
            "args": args or {},
            "calibration": calibration or {},
        }
        if extra:
            self.manifest.update(extra)
        self._write_json("manifest.json", self.manifest)

    # ------------------------------------------------------------------
    @property
    def telemetry_path(self) -> str:
        return str(self.dir / "telemetry.jsonl")

    @property
    def map_dir(self) -> Path:
        return self.dir

    # ------------------------------------------------------------------
    def _write_json(self, name: str, obj: dict) -> None:
        try:
            (self.dir / name).write_text(json.dumps(obj, indent=2))
        except Exception as exc:                              # never crash a run
            print(f" [run_record] could not write {name}: {exc}")

    # ------------------------------------------------------------------
    def finalize(self, coverage: Optional[dict] = None) -> Optional[dict]:
        """Compute metrics from the telemetry and write summary + CSV tables.

        Returns the metrics dict (or None if there was no telemetry)."""
        try:
            records = load_jsonl(self.telemetry_path)
            if not records:
                print(" [run_record] no telemetry recorded — summary skipped.")
                return None
            metrics = compute_run_metrics(records, coverage=coverage)
            metrics["run_id"] = self.dir.name
            metrics["end_time"] = datetime.now().isoformat(timespec="seconds")
            # Self-describe the controller/config so the comparison table can
            # label each run without re-reading the manifest.
            _args = self.manifest.get("args", {})
            metrics["controller"] = _args.get("controller", "")
            metrics["policy"] = _args.get("policy", "") or ""
            self._write_json("summary.json", metrics)
            self._write_flat_csv(metrics)
            self._write_table_csv("per_row.csv", metrics.get("per_row", []))
            self._write_table_csv("turns.csv", metrics.get("turns", []))
            self._print_summary(metrics)
            return metrics
        except Exception as exc:
            print(f" [run_record] finalize failed: {exc}")
            return None

    # ------------------------------------------------------------------
    def _write_flat_csv(self, metrics: dict) -> None:
        """Flatten the scalar metrics to key,value rows (skip nested lists)."""
        rows = []

        def walk(prefix: str, obj) -> None:
            if isinstance(obj, dict):
                for k, v in obj.items():
                    walk(f"{prefix}.{k}" if prefix else k, v)
            elif isinstance(obj, (list, tuple)):
                return   # per_row / turns get their own CSVs
            else:
                rows.append((prefix, obj))

        walk("", metrics)
        try:
            with open(self.dir / "metrics_flat.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["metric", "value"])
                w.writerows(rows)
        except Exception as exc:
            print(f" [run_record] could not write metrics_flat.csv: {exc}")

    def _write_table_csv(self, name: str, rows: list) -> None:
        if not rows:
            return
        cols = list({k for r in rows for k in r.keys()})
        # stable, readable column order: common keys first
        head = [c for c in ("idx", "row", "dir", "src") if c in cols]
        cols = head + [c for c in cols if c not in head]
        try:
            with open(self.dir / name, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=cols)
                w.writeheader()
                for r in rows:
                    w.writerow({c: r.get(c) for c in cols})
        except Exception as exc:
            print(f" [run_record] could not write {name}: {exc}")

    # ------------------------------------------------------------------
    def _print_summary(self, m: dict) -> None:
        f = m.get("follow", {})
        print("\n" + "=" * 64)
        print(f" RUN SUMMARY — {m['run_id']}")
        print("=" * 64)
        print(f"  duration {m['duration_s']:.0f}s  |  {m['n_scans']} scans @ "
              f"{m['scan_rate_hz']:.1f} Hz  |  distance {m['distance_m']:.1f} m")
        print(f"  rows {m['rows_completed']}/{m['rows_total']}  |  "
              f"turns {m.get('turns_completed', 0)}/{m.get('n_turns', 0)} completed")
        print(f"  FOLLOW tracking:  xtrack RMSE {f.get('xtrack_rmse_cm', 0):.1f} cm "
              f"(max {f.get('xtrack_max_cm', 0):.1f})  |  heading RMSE "
              f"{f.get('heading_rmse_deg', 0):.1f}°  |  jerk {f.get('control_jerk', 0):.3f}")
        cov = m.get("coverage")
        if cov:
            print(f"  coverage {cov.get('covered_area_m2', 0):.0f} m²  "
                  f"redundancy {cov.get('redundancy', 0):.2f}×")
        print(f"  saved → {self.dir}/  (manifest, telemetry, summary, CSVs"
              f"{', coverage, trajectory' if cov else ''})")
        print("=" * 64 + "\n")
