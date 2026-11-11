#!/usr/bin/env python3
"""
pack_run.py — Pack a recorded run (``--record`` folder) into ONE compressed
file that can be committed with git, and unpack it again elsewhere.

A run folder holds thousands of small files (``scans/scan_NNNNN.npy``) and can
be hundreds of MB.  The pack keeps everything the offline tools need —
``scans/`` (+ ``index.csv``), ``telemetry.jsonl``, ``manifest.json``,
``summary.json`` and the small CSVs — and stores the points as int16
millimetres (1 mm resolution; points farther than 32 m on any axis are
dropped — the row detector uses ≤ 7 m, SLAM ≤ 30 m).  An intensity column
(``--lidar-intensity`` runs) is kept as uint8.

    # on the robot or the dev PC
    python3 scripts/pack_run.py runs/run_pursuit_20261112_101500
    #   -> field_data/run_pursuit_20261112_101500.npz   (prints its size)
    python3 scripts/pack_run.py runs/run_x --every 2    # keep every 2nd saved scan
    python3 scripts/pack_run.py runs/run_x --from 120 --to 260   # seconds of the run

    # anywhere else: restore a normal run folder for replay_scans.py etc.
    python3 scripts/pack_run.py --unpack field_data/run_x.npz --out runs/
GitHub refuses files over 100 MB: if the pack is bigger, use --every or
--from/--to (the script warns).  NumPy only — runs on the brain.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import sys
from pathlib import Path

import numpy as np

TEXT_FILES = ("manifest.json", "summary.json", "telemetry.jsonl", "metrics_flat.csv",
              "per_row.csv", "turns.csv", "trajectory.csv", "perception_state.json")
LIMIT_MM = 32_000
GITHUB_MAX = 95e6


def pack(run: Path, out: Path, every: int = 1, t_lo: float = -1e18, t_hi: float = 1e18) -> Path:
    scans = run / "scans"
    rows = []
    if (scans / "index.csv").exists():
        with open(scans / "index.csv") as f:
            rows = list(csv.DictReader(f))
    else:
        rows = [{"file": p.name} for p in sorted(scans.glob("scan_*.npy"))]
    keep = []
    for i, r in enumerate(rows):
        t = float(r["t"]) if r.get("t") not in (None, "") else 0.0
        if t_lo <= t <= t_hi and (scans / r["file"]).exists():
            keep.append(r)
    keep = keep[::max(1, every)]

    pts, extra, counts = [], [], []
    dropped = 0
    for r in keep:
        P = np.load(scans / r["file"])
        P = P[np.all(np.isfinite(P[:, :3]), axis=1)]
        mm = np.round(P[:, :3] * 1000.0)
        ok = np.all(np.abs(mm) < LIMIT_MM, axis=1)
        dropped += int((~ok).sum())
        pts.append(mm[ok].astype(np.int16))
        if P.shape[1] > 3:
            extra.append(np.clip(P[ok, 3], 0, 255).astype(np.uint8))
        counts.append(int(ok.sum()))

    data = {
        "format": np.array("pack_run/1"),
        "run_name": np.array(run.name),
        "pts_mm": np.concatenate(pts) if pts else np.zeros((0, 3), np.int16),
        "counts": np.array(counts, dtype=np.int64),
        "files": np.array([r["file"] for r in keep]),
    }
    if extra and len(extra) == len(pts):
        data["intensity"] = np.concatenate(extra)
    if keep and len(keep[0]) > 1:
        buf = io.StringIO()
        w = csv.DictWriter(buf, fieldnames=list(keep[0].keys()))
        w.writeheader()
        w.writerows(keep)
        data["index_csv"] = np.array(buf.getvalue())
    for name in TEXT_FILES:
        p = run / name
        if p.exists():
            data["txt__" + name] = np.array(p.read_text())
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **data)
    size = out.stat().st_size
    print(f"packed {len(keep)} scans ({sum(counts):,} points; {dropped} beyond 32 m dropped) "
          f"+ {sum(1 for k in data if k.startswith('txt__'))} text files -> {out}  "
          f"({size / 1e6:.1f} MB)")
    if size > GITHUB_MAX:
        print(f"WARNING: {size / 1e6:.0f} MB is over GitHub's 100 MB file limit — "
              f"re-run with --every 2 (or more) or --from/--to.", file=sys.stderr)
    return out


def unpack(pack_file: Path, out_root: Path) -> Path:
    d = np.load(pack_file, allow_pickle=False)
    run = out_root / str(d["run_name"])
    scans = run / "scans"
    scans.mkdir(parents=True, exist_ok=True)
    pts = d["pts_mm"].astype(np.float32) / 1000.0
    inten = d["intensity"].astype(np.float32) if "intensity" in d.files else None
    start = 0
    for name, n in zip(d["files"], d["counts"]):
        P = pts[start:start + n]
        if inten is not None:
            P = np.column_stack([P, inten[start:start + n]])
        np.save(scans / str(name), P)
        start += n
    if "index_csv" in d.files:
        (scans / "index.csv").write_text(str(d["index_csv"]))
    for k in d.files:
        if k.startswith("txt__"):
            (run / k[5:]).write_text(str(d[k]))
    print(f"unpacked {len(d['files'])} scans -> {run}")
    return run


def main() -> None:
    ap = argparse.ArgumentParser(description="Pack / unpack a recorded run")
    ap.add_argument("run", nargs="?", help="run folder to pack (runs/run_...)")
    ap.add_argument("--out", default=None,
                    help="pack: output .npz (default field_data/<run>.npz); unpack: parent dir")
    ap.add_argument("--every", type=int, default=1, help="keep every Nth saved scan")
    ap.add_argument("--from", dest="t_lo", type=float, default=-1e18, help="start time (s)")
    ap.add_argument("--to", dest="t_hi", type=float, default=1e18, help="end time (s)")
    ap.add_argument("--unpack", metavar="PACK", help="restore a run folder from a pack")
    args = ap.parse_args()
    if args.unpack:
        unpack(Path(args.unpack), Path(args.out or "runs"))
        return
    if not args.run:
        ap.error("give a run folder to pack, or --unpack PACK")
    run = Path(args.run).resolve()
    if not (run / "scans").is_dir() and not (run / "telemetry.jsonl").exists():
        ap.error(f"{run} has no scans/ or telemetry.jsonl — is it a --record run folder?")
    out = Path(args.out) if args.out else Path("field_data") / f"{run.name}.npz"
    pack(run, out, args.every, args.t_lo, args.t_hi)


if __name__ == "__main__":
    main()
