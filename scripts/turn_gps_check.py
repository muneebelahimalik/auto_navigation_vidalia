#!/usr/bin/env python3
"""
turn_gps_check.py — what did the robot REALLY do during each U-turn? (offline)

Compares the turn's own rotation estimate (telemetry ``turn_rot_deg``, from the
filter/IMU heading or wheels) with the GPS track logged on the side by
scripts/gnss_logger.py: GPS course change (from vel_north/vel_east), path
length, effective turn radius, and how far sideways the robot ended up from
the row it left (two beds over = 3.66 m).  Read-only analysis — touches no
navigation code.  RTK FLOAT positions (~0.5 m absolute) still show the turn's
SHAPE: their error drifts slowly, so over one minute the relative track holds.

    python3 scripts/turn_gps_check.py                       # newest run + newest GPS log
    python3 scripts/turn_gps_check.py runs/run_X gnss/gnss_Y.jsonl
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys
from datetime import datetime

R_N, R_E = 110540.0, 111320.0          # m per degree (lat, lon at equator)


def load_run(run):
    man = json.load(open(os.path.join(run, "manifest.json")))
    rows = [json.loads(l) for l in open(os.path.join(run, "telemetry.jsonl")) if l.strip()]
    if rows and "mono" in rows[0]:
        for r in rows:
            r["_clk"] = r["mono"]
        return rows, "mono"
    t0 = datetime.fromisoformat(man["start_time"]).timestamp()
    for r in rows:
        r["_clk"] = t0 + r.get("t", 0.0)
    return rows, "wall"                 # ±a few s (start-up time before telemetry)


def load_gps(path, clock):
    out = []
    for l in open(path):
        d = json.loads(l)
        if d.get("path") != "/pvt":
            continue
        m = d["msg"]
        if "latitude" not in m:
            continue
        out.append({"clk": d["mono"] if clock == "mono" else d["wall"],
                    "lat": m["latitude"], "lon": m["longitude"],
                    "vn": m.get("vel_north", 0.0), "ve": m.get("vel_east", 0.0),
                    "spd": m.get("ground_speed", 0.0)})
    return out


def turns(rows):
    """[(start_idx, end_idx)] of each HEADLAND stretch."""
    out, s = [], None
    for i, r in enumerate(rows):
        h = r.get("state") == "HEADLAND"
        if h and s is None:
            s = i
        if not h and s is not None:
            out.append((s, i - 1)); s = None
    if s is not None:
        out.append((s, len(rows) - 1))
    return out


def analyse(rows, gps, a, b):
    arc = [r for r in rows[a:b + 1] if r.get("turn_phase") == "ARC"]
    if not arc:
        print("  no ARC phase logged"); return
    c0, c1 = arc[0]["_clk"], arc[-1]["_clk"]
    # Row direction = GPS course over the 15 s of FOLLOW before the turn.
    pre = [g for g in gps if rows[a]["_clk"] - 40 <= g["clk"] <= rows[a]["_clk"] - 25
           and g["spd"] > 0.08]
    seg = [g for g in gps if c0 <= g["clk"] <= c1]
    if len(seg) < 5:
        print(f"  only {len(seg)} GPS samples in the arc — GPS log does not cover this turn")
        return
    lat0, lon0 = seg[0]["lat"], seg[0]["lon"]
    k = math.cos(math.radians(lat0))
    def xy(g):                           # east, north (m) from arc start
        return (g["lon"] - lon0) * R_E * k, (g["lat"] - lat0) * R_N
    if pre:
        row_dir = math.atan2(sum(g["ve"] for g in pre), sum(g["vn"] for g in pre))
    else:
        row_dir = math.atan2(seg[0]["ve"], seg[0]["vn"])
    # GPS course change, unwrapped, only while moving.
    crs, prev, acc = [], None, 0.0
    for g in seg:
        if g["spd"] < 0.08:
            crs.append(acc); continue
        c = math.atan2(g["ve"], g["vn"])
        if prev is not None:
            acc += (c - prev + math.pi) % (2 * math.pi) - math.pi
        prev = c
        crs.append(acc)
    path = sum(math.dist(xy(p), xy(q)) for p, q in zip(seg, seg[1:]))
    gps_turn = abs(crs[-1])
    print(f"  ARC  {c1 - c0:5.1f} s   wheel/odom arc {arc[-1].get('turn_arc_m', 0):.2f} m   "
          f"GPS path {path:.2f} m")
    print(f"  rotation: turn estimate {arc[-1].get('turn_rot_deg', 0):6.1f}° "
          f"[{arc[-1].get('turn_src', '?')}]   GPS course change {math.degrees(gps_turn):6.1f}°")
    if gps_turn > 0.3:
        print(f"  effective radius (GPS path / GPS rotation): {path / gps_turn:.2f} m")
    # Where it ended up relative to the row it left.
    ex, ey = xy(seg[-1])
    ux, uy = math.sin(row_dir), math.cos(row_dir)          # along-row unit (E, N)
    along = ex * ux + ey * uy
    side = ex * uy - ey * ux                                # + = right of row direction
    print(f"  arc end vs arc start: {along:+.2f} m along the row, {side:+.2f} m sideways "
          f"({'right' if side > 0 else 'left'}; two beds over = 3.66 m)")
    print("   time   est_rot  gps_rot   east  north  speed")
    j, last = 0, -9.0
    for i, g in enumerate(seg):
        if g["clk"] - last < 3.0:
            continue
        last = g["clk"]
        while j + 1 < len(arc) and arc[j + 1]["_clk"] <= g["clk"]:
            j += 1
        x, y = xy(g)
        print(f"  {g['clk'] - c0:6.1f}  {arc[j].get('turn_rot_deg', 0):7.1f}  "
              f"{math.degrees(abs(crs[i])):7.1f}  {x:5.2f}  {y:5.2f}  {g['spd']:.2f}")


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob("runs/run_*"), key=os.path.getmtime)
    gp = sys.argv[2] if len(sys.argv) > 2 else max(glob.glob("gnss/gnss_*.jsonl"), key=os.path.getmtime)
    rows, clock = load_run(run)
    gps = load_gps(gp, clock)
    print(f"run {run}\ngps {gp}\nclock join: {clock}"
          + ("" if clock == "mono" else "  (approximate: run predates the 'mono' stamp)"))
    ts = turns(rows)
    if not ts:
        print("no HEADLAND phase in this run"); return
    for n, (a, b) in enumerate(ts, 1):
        print(f"\nturn {n}: t={rows[a].get('t', 0):.1f}–{rows[b].get('t', 0):.1f} s, "
              f"dir {rows[a].get('turn_dir', '?')}")
        analyse(rows, gps, a, b)


if __name__ == "__main__":
    main()
