#!/usr/bin/env python3
"""
make_run_schedule.py — randomised, balanced run schedule for the field
controller comparison (docs/EXPERIMENTAL_PLAN.md).

One field VISIT = 3 sessions x 3 runs.  Each session drives the three
controllers, each on a different 3-bed SET, so tire tracks of different
controllers never share a furrow within a session.  The assignment is a 3x3
Graeco-Latin square (session x bed set; Latin = controller, Greek = run
position), so over the visit every controller
  * drives every bed set exactly once,
  * runs in every session exactly once,
  * takes every run position (1st / 2nd / 3rd of the session) exactly once.
Controller labels, bed-set labels and session order are randomised with a
seed that is printed, so the schedule is reproducible.

Serpentine start side: run positions 1 and 3 sweep LEFT (first U-turn left)
and position 2 sweeps RIGHT on odd visits, the reverse on even visits; with
the Latin square that gives every controller 2:1 left/right first turns per
visit and 3:3 over two visits.

    python3 scripts/make_run_schedule.py --field TP1 --visit 1 --seed 101
    python3 scripts/make_run_schedule.py --field DS1 --visit 2 --seed 202 --csv sched.csv
"""
from __future__ import annotations

import argparse
import csv
import sys

import numpy as np

CONTROLLERS = {
    "PP": "--controller pursuit",
    "MPC": "--controller mpc --mpc-noise 0.10 --mpc-temp 0.1",
    "RL": "--controller rl --policy policies/trainers/ppo_onion_s1.npz",
}
COMMON = ("python3 scripts/row_follow.py --auto --preset onion {ctrl} --speed 0.20 "
          "{sweep} --headland-arc-cap 4.5 --record --no-slam --save-scans 2")

# Orthogonal 3x3 Latin squares: LATIN[s][b] = controller index,
# GREEK[s][b] = run position (0-based), for session s and bed set b.
LATIN = [[0, 1, 2], [1, 2, 0], [2, 0, 1]]
GREEK = [[0, 1, 2], [2, 0, 1], [1, 2, 0]]


def schedule(field: str, visit: int, seed: int, sets=("A", "B", "C")):
    rng = np.random.default_rng(seed)
    ctrl = list(CONTROLLERS)
    ctrl = [ctrl[i] for i in rng.permutation(3)]
    bsets = [sets[i] for i in rng.permutation(3)]
    sess_order = rng.permutation(3)
    rows = []
    for s_new, s in enumerate(sess_order):
        runs = []
        for b in range(3):
            pos = GREEK[s][b]
            left = (pos != 1) if visit % 2 == 1 else (pos == 1)
            c = ctrl[LATIN[s][b]]
            sweep = "--beds-left 2 --beds-right 0" if left else "--beds-left 0 --beds-right 2"
            runs.append(dict(field=field, visit=visit, session=s_new + 1, run_pos=pos + 1,
                             controller=c, bed_set=bsets[b],
                             start_side="right edge of set, sweep LEFT" if left
                             else "left edge of set, sweep RIGHT",
                             first_turn="L" if left else "R",
                             command=COMMON.format(ctrl=CONTROLLERS[c], sweep=sweep)))
        rows += sorted(runs, key=lambda r: r["run_pos"])
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--field", required=True, help="field id, e.g. TP1 (transplanted) / DS1")
    ap.add_argument("--visit", type=int, required=True, help="visit number in this field (1, 2, …)")
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--sets", default="A,B,C", help="labels of the three 3-bed sets")
    ap.add_argument("--csv", default="", help="also write the schedule to this CSV")
    a = ap.parse_args()
    rows = schedule(a.field, a.visit, a.seed, tuple(a.sets.split(",")))
    print(f"field {a.field}  visit {a.visit}  seed {a.seed}")
    for r in rows:
        print(f"S{r['session']} run {r['run_pos']}: {r['controller']:4s} bed set {r['bed_set']}  "
              f"first turn {r['first_turn']} ({r['start_side']})\n    {r['command']}")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"-> {a.csv}", file=sys.stderr)


if __name__ == "__main__":
    main()
