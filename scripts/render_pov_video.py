#!/usr/bin/env -S python3 -u
"""
render_pov_video.py — Cinematic POV video of the LiDAR autonomous navigation.

Turns a recorded run's LiDAR scan time-series into a self-driving-style POV
movie: the height-coloured point cloud with the soybean rows and structure, seen
from a chase camera that rides with the robot, annotated LIVE with the autonomy
decisions — detected strip-centre (planned path), heading, look-ahead target,
detection ROI, forward safety zone (turns red when blocked), the ego robot, and
a HUD (state, cross-track, heading, speed, turn rate, rows done, grade/roll).

Data sources
------------
  --run runs/run_<ts>       a --record run: reads scans/scan_*.npy + scans/index.csv
                            (and telemetry.jsonl for command/safety fields)
  --scans DIR --telemetry F point a scans dir + telemetry.jsonl explicitly
  --demo                    synthesize a weaving fly-through (no run needed) so you
                            can preview the look and the overlays immediately

Output
------
Always renders PNG frames to ``<out>_frames/``.  Then assembles:
  * a GIF via Pillow (no ffmpeg needed) — an immediate, shareable preview;
  * an MP4 via ffmpeg if it is on PATH (best for slides/paper) — otherwise the
    exact ffmpeg command is printed so you can build it on a machine that has it
    (your dev PC does).

Examples
--------
    # Preview the style with a synthetic fly-through:
    python3 scripts/render_pov_video.py --demo --frames 120 --out results/pov_demo

    # Best POV from a real recorded run:
    python3 scripts/render_pov_video.py --run runs/run_mpc_20260705_1200 \
        --view chase --fps 15 --out results/pov_run

    # Top-down "sensor scope" movie instead of the 3-D chase cam:
    python3 scripts/render_pov_video.py --run runs/run_... --view scope --out results/scope
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# Reuse the exact system geometry + height colormap the still-figure tool uses.
from scripts.viz_perception import (
    _height_cmap, generate_lidar_raycast,
    MOUNT_H, ROI_Y, ROI_X, SELF_R, TIRE_TRACK, TIRE_HW, FWD_HW, SAFE_DIST,
    LOOKAHEAD, BODY_HW, BODY_BACK, BODY_FWD,
)

BG = "#0a0d14"


# ---------------------------------------------------------------------------
# Frame data model
# ---------------------------------------------------------------------------
class Frame:
    __slots__ = ("P", "lateral", "heading_deg", "state", "conf", "lin", "ang",
                 "fwd_blk", "lt_blk", "rt_blk", "rows_done", "grade", "roll",
                 "sp", "t")

    def __init__(self, P, lateral=0.0, heading_deg=0.0, state="FOLLOW", conf=0.0,
                 lin=0.0, ang=0.0, fwd_blk=False, lt_blk=False, rt_blk=False,
                 rows_done=0, grade=0.0, roll=0.0, sp=0.76, t=0.0):
        self.P = P
        self.lateral = lateral; self.heading_deg = heading_deg
        self.state = state; self.conf = conf
        self.lin = lin; self.ang = ang
        self.fwd_blk = fwd_blk; self.lt_blk = lt_blk; self.rt_blk = rt_blk
        self.rows_done = rows_done; self.grade = grade; self.roll = roll
        self.sp = sp; self.t = t


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------
def _load_telemetry(path: Path):
    """telemetry.jsonl → list of dicts (or [])."""
    if not path or not path.exists():
        return []
    rows = []
    for line in path.open():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except Exception:
                pass
    return rows


def _nearest_tel(tel_rows, t):
    """Nearest telemetry record to time t (for command/safety fields)."""
    if not tel_rows:
        return {}
    best, bd = None, 1e9
    for r in tel_rows:
        d = abs(float(r.get("t", 0.0)) - t)
        if d < bd:
            bd, best = d, r
    return best or {}


def load_run(scans_dir: Path, telemetry: Path, stride: int, max_frames: int):
    index = scans_dir / "index.csv"
    if not index.exists():
        raise SystemExit(f"no index.csv in {scans_dir} — is this a --record run's scans/ dir?")
    tel_rows = _load_telemetry(telemetry)
    frames = []
    with index.open() as f:
        rows = list(csv.DictReader(f))
    for i, row in enumerate(rows):
        if i % stride:
            continue
        fp = scans_dir / row["file"]
        if not fp.exists():
            continue
        P = np.load(fp)[:, :3].astype(float)
        t = float(row.get("t", 0.0))
        tr = _nearest_tel(tel_rows, t)
        frames.append(Frame(
            P,
            lateral=float(row.get("lateral", 0.0)),
            heading_deg=float(row.get("heading_deg", 0.0)),
            state=row.get("state", "FOLLOW"),
            conf=float(row.get("conf", 0.0)),
            lin=float(tr.get("lin_cmd", 0.0)), ang=float(tr.get("ang_cmd", 0.0)),
            fwd_blk=bool(tr.get("fwd_blocked", False)),
            lt_blk=bool(tr.get("ltire_blocked", False)),
            rt_blk=bool(tr.get("rtire_blocked", False)),
            rows_done=int(tr.get("rows_done", 0)),
            grade=float(tr.get("grade_deg", 0.0)), roll=float(tr.get("roll_deg", 0.0)),
            sp=float(tr.get("sp", 0.76) or 0.76), t=t))
        if max_frames and len(frames) >= max_frames:
            break
    if not frames:
        raise SystemExit("no frames loaded — check the scans dir / stride.")
    return frames


def make_demo(n_frames: int, fps: int):
    """Synthetic weaving fly-through so the look/overlays can be previewed with
    no recorded run.  The robot oscillates gently across the strip and the
    heading leads the correction, exercising every overlay."""
    frames = []
    for i in range(n_frames):
        ph = 2 * math.pi * i / 55.0
        lateral = 0.11 * math.sin(ph)
        heading = -7.0 * math.cos(ph)           # heading leads the lateral
        conf = 0.72 + 0.16 * math.sin(ph * 1.7)
        grade = 3.0 * math.sin(ph * 0.5)
        P = generate_lidar_raycast(lateral=lateral, heading_deg=heading,
                                   spacing=0.62, grade_deg=grade, seed=i)
        frames.append(Frame(P, lateral=lateral, heading_deg=heading, state="FOLLOW",
                            conf=conf, lin=0.28 * conf, ang=-0.018 * heading,
                            grade=grade, roll=0.0, sp=0.62, t=i / fps))
    return frames


# ---------------------------------------------------------------------------
# Overlay geometry
# ---------------------------------------------------------------------------
def _box_faces(x0, x1, y0, y1, z0, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    return [[v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]],
            [v[0], v[1], v[5], v[4]], [v[2], v[3], v[7], v[6]],
            [v[1], v[2], v[6], v[5]], [v[0], v[3], v[7], v[4]]]


VIEWS = {          # (elev, azim) — cinematic chase / POV looking down the rows
    "chase": (26, -68),   # 3/4 self-driving chase cam (default, most cinematic)
    "pov":   (12, -89),   # low first-person, looking straight down the rows
    "high":  (40, -72),   # elevated survey angle
    "scope": None,        # 2-D top-down handled separately
}


def render_frame_3d(frame: Frame, cfg, out_path: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    P = frame.P
    if cfg.max_points and len(P) > cfg.max_points:      # thin for speed
        P = P[np.random.default_rng(0).choice(len(P), cfg.max_points, replace=False)]
    h = P[:, 2] + MOUNT_H
    th = math.radians(frame.heading_deg)

    fig = plt.figure(figsize=(cfg.w / cfg.dpi, cfg.h / cfg.dpi), dpi=cfg.dpi,
                     facecolor=BG)
    ax = fig.add_subplot(111, projection="3d", facecolor=BG)
    ax.set_position([-0.06, -0.10, 1.12, 1.20])          # fill the frame
    try:
        ax.set_proj_type("persp", focal_length=0.62)     # true POV perspective
    except Exception:
        try:
            ax.set_proj_type("persp")
        except Exception:
            pass

    order = np.argsort(h)                                # draw low→high so rows pop
    ax.scatter(P[order, 0], P[order, 1], h[order], c=h[order], cmap=_height_cmap(),
               vmin=-0.05, vmax=0.45, s=cfg.psize, alpha=0.95,
               edgecolors="none", depthshade=False)

    # planned path (detected strip-centre) — glowing
    yp = np.linspace(SELF_R, ROI_Y[1], 40)
    xp = frame.lateral + yp * math.tan(th)
    ax.plot(xp, yp, np.full_like(yp, 0.02), color="#00e5ff", lw=3.4, zorder=10)
    ax.plot(xp, yp, np.full_like(yp, 0.02), color="white", lw=1.0, alpha=0.7, zorder=11)
    lx = frame.lateral + LOOKAHEAD * math.tan(th)
    ax.scatter([lx], [LOOKAHEAD], [0.05], s=150, marker="*", color="#ffd000",
               edgecolors="k", lw=0.5, zorder=12)

    # detection ROI floor
    roi = [[(-ROI_X, ROI_Y[0], 0.0), (ROI_X, ROI_Y[0], 0.0),
            (ROI_X, ROI_Y[1], 0.0), (-ROI_X, ROI_Y[1], 0.0)]]
    pc = Poly3DCollection(roi, facecolor="#00e5ff", alpha=0.06, edgecolor="#00e5ff", lw=1.0)
    pc.set_zsort("min"); ax.add_collection3d(pc)

    # forward safety zone — green normally, red when blocked
    safe_col = "#ff3b3b" if frame.fwd_blk else "#2ecc71"
    sz = Poly3DCollection(_box_faces(-FWD_HW, FWD_HW, 0.2, SAFE_DIST, 0.0, 0.55),
                          facecolor=safe_col, alpha=0.08 if not frame.fwd_blk else 0.16,
                          edgecolor=safe_col, linewidths=1.0)
    sz.set_zsort("average"); ax.add_collection3d(sz)

    # ego robot wireframe + forward arrow
    er = Poly3DCollection(_box_faces(-BODY_HW, BODY_HW, BODY_BACK, BODY_FWD, 0.0, 0.45),
                          facecolor="#ffffff", alpha=0.08, edgecolor="#ffffff", linewidths=1.6)
    er.set_zsort("average"); ax.add_collection3d(er)
    ax.quiver(0, BODY_FWD, 0.22, 0, 1.5, 0, color="#00e5ff", lw=2.6, arrow_length_ratio=0.22)

    ax.set_xlim(-2.6, 2.6); ax.set_ylim(-0.4, 7.6); ax.set_zlim(0, 0.9)
    try:
        ax.set_box_aspect((5.2, 8.0, 1.4))
    except Exception:
        pass
    elev, azim = cfg.elev_azim
    ax.view_init(elev=elev, azim=azim)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor(BG); pane.pane.set_edgecolor("#0e141d")
        pane.line.set_color("#0e141d")
    ax.grid(False)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])

    _hud(ax, frame)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.savefig(out_path, facecolor=BG)
    plt.close(fig)


def render_frame_scope(frame: Frame, cfg, out_path: Path):
    """Top-down 'sensor scope' frame (2-D) — cheaper, also paper-friendly."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    P = frame.P
    h = P[:, 2] + MOUNT_H
    th = math.radians(frame.heading_deg)
    fig = plt.figure(figsize=(cfg.w / cfg.dpi, cfg.h / cfg.dpi), dpi=cfg.dpi, facecolor=BG)
    ax = fig.add_subplot(111, facecolor=BG)
    for r in (2, 4, 6, 8):
        ax.add_patch(Circle((0, 0), r, fill=False, ec="#1b2531", lw=1.0, zorder=1))
    order = np.argsort(h)
    ax.scatter(P[order, 0], P[order, 1], c=h[order], cmap=_height_cmap(),
               vmin=-0.05, vmax=0.45, s=cfg.psize, alpha=0.95, edgecolors="none", zorder=3)
    ax.add_patch(Rectangle((-ROI_X, ROI_Y[0]), 2 * ROI_X, ROI_Y[1] - ROI_Y[0],
                           fill=False, ec="#00e5ff", ls="--", lw=1.3, alpha=0.8, zorder=5))
    safe_col = "#ff3b3b" if frame.fwd_blk else "#2ecc71"
    ax.add_patch(Rectangle((-FWD_HW, 0.2), 2 * FWD_HW, SAFE_DIST - 0.2, fc=safe_col,
                           ec=safe_col, alpha=0.12, lw=1, zorder=4))
    yp = np.linspace(SELF_R, ROI_Y[1], 40)
    ax.plot(frame.lateral + yp * math.tan(th), yp, color="#00e5ff", lw=3, zorder=8)
    ax.plot(frame.lateral + yp * math.tan(th), yp, color="white", lw=1, alpha=0.6, zorder=9)
    lx = frame.lateral + LOOKAHEAD * math.tan(th)
    ax.scatter([lx], [LOOKAHEAD], s=170, marker="*", color="#ffd000",
               edgecolors="k", lw=0.6, zorder=10)
    ax.add_patch(Circle((0, 0), SELF_R, fc="#11161f", ec="#222b38", ls=":", lw=1.1,
                        alpha=0.6, zorder=2))
    ax.add_patch(Rectangle((-BODY_HW, BODY_BACK), 2 * BODY_HW, BODY_FWD - BODY_BACK,
                           fc="#e8e8e8", ec="white", alpha=0.9, zorder=11))
    ax.annotate("", xy=(0, 1.7), xytext=(0, BODY_FWD),
                arrowprops=dict(arrowstyle="-|>", color="#00e5ff", lw=2.2), zorder=11)
    ax.set_xlim(-5, 5); ax.set_ylim(-2.4, 9.2); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#1c2430")
    _hud(ax, frame, twod=True)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    fig.savefig(out_path, facecolor=BG)
    plt.close(fig)


def _hud(ax, frame: Frame, twod=False):
    """Heads-up telemetry overlay (axes-fraction text)."""
    txt = ax.text2D if hasattr(ax, "text2D") and not twod else ax.text
    state_col = {"FOLLOW": "#2ecc71", "ACQUIRE": "#ffd000", "OBSTACLE_WAIT": "#ff3b3b",
                 "ROW_END": "#00e5ff", "HEADLAND": "#b06bff", "APPROACH": "#ffd000",
                 "DONE": "#9aa0aa"}.get(frame.state, "#e8e8e8")
    lines = [
        (f"{frame.state}", state_col, 15),
        (f"cross-track  {frame.lateral*100:+5.1f} cm", "#e8e8e8", 11),
        (f"heading      {frame.heading_deg:+5.1f}°", "#e8e8e8", 11),
        (f"conf         {frame.conf:4.2f}", "#e8e8e8", 11),
        (f"v {frame.lin:4.2f} m/s   ω {frame.ang:+5.2f} rad/s", "#9aa0aa", 10),
    ]
    if abs(frame.grade) > 0.5 or abs(frame.roll) > 0.5:
        lines.append((f"grade {frame.grade:+.0f}°  roll {frame.roll:+.0f}°",
                      "#9aa0aa", 10))
    if frame.rows_done:
        lines.append((f"rows done  {frame.rows_done}", "#9aa0aa", 10))
    y = 0.975
    for s, c, fs in lines:
        kw = dict(transform=ax.transAxes, ha="left", va="top", color=c, fontsize=fs,
                  family="DejaVu Sans", fontweight="bold" if fs >= 15 else "normal")
        if twod:
            ax.text(0.015, y, s, **kw)
        else:
            ax.text2D(0.015, y, s, **kw)
        y -= 0.045 if fs >= 15 else 0.033
    tag = "VLP-16 Hi-Res • autonomous row-following (POV)"
    (ax.text2D if not twod else ax.text)(0.985, 0.02, tag, transform=ax.transAxes,
                                         ha="right", va="bottom", color="#3a4658",
                                         fontsize=9)


# ---------------------------------------------------------------------------
# Assembly
# ---------------------------------------------------------------------------
def assemble(frame_paths, out_base: Path, fps: int, want_gif: bool):
    mp4 = out_base.with_suffix(".mp4")
    made = []
    if shutil.which("ffmpeg"):
        pattern = str(frame_paths[0].parent / "frame_%05d.png")
        cmd = ["ffmpeg", "-y", "-framerate", str(fps), "-i", pattern,
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", str(mp4)]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
            made.append(mp4)
        except Exception as exc:  # noqa: BLE001
            print(f"  ffmpeg failed ({exc}); GIF only.")
    else:
        print("  ffmpeg not on PATH — skipping MP4.  Build it where ffmpeg exists:")
        print(f"    ffmpeg -framerate {fps} -i {frame_paths[0].parent}/frame_%05d.png "
              f"-c:v libx264 -pix_fmt yuv420p -crf 18 {mp4}")
    if want_gif or not made:
        from PIL import Image
        gif = out_base.with_suffix(".gif")
        imgs = [Image.open(p).convert("P", palette=Image.ADAPTIVE) for p in frame_paths]
        imgs[0].save(gif, save_all=True, append_images=imgs[1:],
                     duration=int(1000 / fps), loop=0, optimize=True)
        made.append(gif)
    return made


# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--run", help="a --record run dir (uses scans/ + telemetry.jsonl)")
    src.add_argument("--scans", help="scans dir (with index.csv)")
    src.add_argument("--demo", action="store_true", help="synthetic fly-through preview")
    ap.add_argument("--telemetry", help="telemetry.jsonl (with --scans)")
    ap.add_argument("--view", choices=list(VIEWS), default="chase")
    ap.add_argument("--fps", type=int, default=12)
    ap.add_argument("--stride", type=int, default=1, help="use every Nth saved scan")
    ap.add_argument("--frames", type=int, default=120, help="demo: number of frames")
    ap.add_argument("--max-frames", type=int, default=0, help="cap frames (0=all)")
    ap.add_argument("--res", default="1600x900", help="WxH pixels")
    ap.add_argument("--dpi", type=int, default=100)
    ap.add_argument("--psize", type=float, default=4.0, help="point size")
    ap.add_argument("--max-points", type=int, default=26000, help="thin cloud for speed")
    ap.add_argument("--no-gif", action="store_true", help="skip the GIF (MP4 only if ffmpeg)")
    ap.add_argument("--out", default="results/pov", help="output base path")
    args = ap.parse_args()

    w, hh = (int(v) for v in args.res.lower().split("x"))
    cfg = argparse.Namespace(w=w, h=hh, dpi=args.dpi, psize=args.psize,
                             max_points=args.max_points,
                             elev_azim=VIEWS[args.view] or (16, -90))

    if args.demo:
        frames = make_demo(args.frames, args.fps)
        print(f"[pov] demo fly-through: {len(frames)} frames")
    else:
        scans_dir = Path(args.run) / "scans" if args.run else Path(args.scans)
        tel = (Path(args.run) / "telemetry.jsonl" if args.run
               else (Path(args.telemetry) if args.telemetry else None))
        frames = load_run(scans_dir, tel, args.stride, args.max_frames)
        print(f"[pov] loaded {len(frames)} frames from {scans_dir}")

    out_base = Path(args.out)
    out_base.parent.mkdir(parents=True, exist_ok=True)
    frames_dir = Path(str(out_base) + "_frames")
    frames_dir.mkdir(parents=True, exist_ok=True)

    render = render_frame_scope if args.view == "scope" else render_frame_3d
    paths = []
    for i, fr in enumerate(frames):
        p = frames_dir / f"frame_{i:05d}.png"
        render(fr, cfg, p)
        paths.append(p)
        if (i + 1) % 10 == 0 or i + 1 == len(frames):
            print(f"  rendered {i+1}/{len(frames)}")

    made = assemble(paths, out_base, args.fps, want_gif=not args.no_gif)
    print("\n[pov] wrote:")
    for m in made:
        print(f"    {m}")
    print(f"    {len(paths)} PNG frames in {frames_dir}/")


if __name__ == "__main__":
    main()
