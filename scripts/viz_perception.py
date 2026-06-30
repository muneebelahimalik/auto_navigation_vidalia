#!/usr/bin/env -S python3 -u
"""
viz_perception.py — Poster/paper figure of what the LiDAR perception sees.

Renders a professional two-panel figure of one VLP-16 scan in the ROBOT frame
(after the 66° yaw / 21.5° nose-down tilt correction the navigation stack uses):

  LEFT  — bird's-eye "what the LiDAR sees", height-coloured, with every element
          of the autonomy pipeline annotated: forward direction, the two soybean
          rows flanking the residue strip, the detected strip-centre (tracking
          target) + heading, the look-ahead point, the cross-track offset, the
          detection ROI, the self-filter blind zone, the robot footprint, and the
          three safety zones (forward + left/right tire tracks).
  RIGHT — 3-D perspective of the same scan (height-coloured) with the robot and
          forward-travel arrow, so a viewer sees the crop rows standing above the
          ground plane.

Data source (in order of authenticity):
  --scan FILE.npy / FILE.ply   render a REAL captured scan (Nx3, robot frame)
  --telemetry F --index N       seed a geometry-faithful scene from a real scan's
                                measured offset/heading/spacing/grade
  (default)                     a representative soybean-row scene from the
                                field calibration

Outputs <out>.png (300 dpi, poster-ready) and <out>.svg (vector, scales cleanly).

This is a desktop/figure tool (uses matplotlib); it does not run on the brain's
control path and is never imported by the navigation stack.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# ---- system constants (match lidar/obstacle_filter.py + navigation defaults) ----
MOUNT_H = 0.75          # LiDAR drum-centre height above ground (m)
ROI_Y = (1.5, 7.0)      # detection ROI forward extent (m)
ROI_X = 0.80            # detection ROI half-width (m)
SELF_R = 1.5            # self-filter radius (m)
TIRE_TRACK = 0.959      # wheel-centre half-track (m)
TIRE_HW = 0.25          # tire-zone half-width (m)
FWD_HW = 0.60           # forward safety-zone half-width (m)
SAFE_DIST = 2.5         # safety-zone forward horizon (m)
CROP_BAND = (0.03, 0.30)
LOOKAHEAD = 2.0         # pure-pursuit look-ahead (m)
BODY_HW = 0.50          # robot body half-width for drawing (m)
BODY_BACK, BODY_FWD = -0.6, 0.4   # body extent in Y for drawing


# ---------------------------------------------------------------------------
def generate_scene(lateral=-0.05, heading_deg=3.5, spacing=0.62, grade_deg=1.0,
                   n_crop=660, seed=0):
    """A geometry-faithful soybean dual-row scan in the robot frame (Nx3 xyz),
    with z = height_above_ground - MOUNT_H (the corrected sensor convention)."""
    rng = np.random.default_rng(seed)
    th = np.radians(heading_deg)
    gslope = np.tan(np.radians(grade_deg))
    pts = []

    def row(cx0, npts, h_lo, h_hi, xj):
        y = rng.uniform(ROI_Y[0] - 0.3, ROI_Y[1] + 1.0, npts)
        # thin the row with range (VLP-16 returns sparser far away)
        keep = rng.random(npts) < np.clip(1.2 - 0.10 * y, 0.15, 1.0)
        y = y[keep]
        x = cx0 + lateral + y * np.tan(th) + rng.normal(0, xj, len(y))
        h = rng.uniform(h_lo, h_hi, len(y)) + gslope * y
        pts.append(np.column_stack([x, y, h - MOUNT_H]))

    # two flanking soybean rows (the dominant repeatable structure)
    row(-spacing / 2, n_crop // 2, *CROP_BAND, 0.035)
    row(+spacing / 2, n_crop // 2, *CROP_BAND, 0.035)
    # sparse residue/stubble on the centre strip (the bare tracking target)
    ys = rng.uniform(ROI_Y[0], ROI_Y[1], 60)
    xs = lateral + ys * np.tan(th) + rng.normal(0, 0.05, 60)
    pts.append(np.column_stack([xs, ys, rng.uniform(0.0, 0.06, 60) + gslope * ys - MOUNT_H]))
    # ground returns across and beyond the ROI
    yg = rng.uniform(ROI_Y[0] - 0.2, 9.0, 500)
    xg = rng.uniform(-3.0, 3.0, 500)
    pts.append(np.column_stack([xg, yg, rng.normal(0, 0.02, 500) + gslope * yg - MOUNT_H]))
    # outer rows beyond the ROI (excluded by ROI_X — shows the ROI doing its job)
    for cx0 in (-1.52, +1.52):
        yo = rng.uniform(ROI_Y[0], ROI_Y[1] + 1.0, 90)
        xo = cx0 + lateral + yo * np.tan(th) + rng.normal(0, 0.04, 90)
        pts.append(np.column_stack([xo, yo, rng.uniform(*CROP_BAND, 90) + gslope * yo - MOUNT_H]))

    P = np.vstack(pts)
    P = P[np.hypot(P[:, 0], P[:, 1]) >= SELF_R]   # apply the self-filter blind zone
    return P


def _crop_height(x, y, lateral, theta, spacing):
    """Canopy height field (m): two flanking rows (+ outer rows) as ridges."""
    h = np.zeros_like(x)
    for c0, peak in ((-spacing / 2, 0.22), (spacing / 2, 0.22),
                     (-1.14, 0.24), (1.14, 0.24)):
        xr = lateral + c0 + y * np.tan(theta)
        hc = peak * np.exp(-((x - xr) ** 2) / (2 * 0.075 ** 2))
        hc = np.where((y >= 0.6) & (y <= 9.2), hc, 0.0)
        h = np.maximum(h, hc)
    return h


def generate_lidar_raycast(lateral=-0.06, heading_deg=4.0, spacing=0.62,
                           grade_deg=1.0, az_res_deg=0.35, seed=0):
    """Ray-cast a physically-faithful VLP-16 scan: 16 channels (−15°..+15°)
    spun over azimuth, the unit pitched 21.5° nose-down at MOUNT_H, intersecting
    a ground plane + crop-row ridges.  Produces the characteristic Velodyne
    ground rings that bend up over the crop rows.  Returns Nx3 robot-frame xyz
    (z = height above ground)."""
    rng = np.random.default_rng(seed)
    T = np.radians(21.5)                      # nose-down pitch
    th = np.radians(heading_deg)
    gslope = np.tan(np.radians(grade_deg))
    eps = np.radians(np.linspace(-15, 15, 16))             # 16 channels
    az = np.radians(np.arange(0, 360, az_res_deg))         # azimuth sweep
    E, A = np.meshgrid(eps, az)
    E = E.ravel(); A = A.ravel()
    # beam direction in sensor frame (x=right, y=forward, z=up), az from forward
    dx = np.cos(E) * np.sin(A)
    dy = np.cos(E) * np.cos(A)
    dz = np.sin(E)
    # apply nose-down pitch about x-axis (forward tilts toward ground)
    dy2 = dy * np.cos(T) + dz * np.sin(T)
    dz2 = -dy * np.sin(T) + dz * np.cos(T)
    down = dz2 < -1e-3                          # only beams heading groundward
    dx, dy2, dz2 = dx[down], dy2[down], dz2[down]
    # ground-plane intersection (ground z=0, sensor at height MOUNT_H)
    tg = -MOUNT_H / dz2
    xg, yg = tg * dx, tg * dy2
    # keep a sensible scene window
    keep = (tg > 0) & (yg > -2.5) & (yg < 11) & (np.abs(xg) < 6)
    dx, dy2, dz2, xg, yg = dx[keep], dy2[keep], dz2[keep], xg[keep], yg[keep]
    # lift points that actually hit a crop ridge before the ground
    hc = _crop_height(xg, yg, lateral, th, spacing)
    hit_crop = hc > 0.03
    th_t = np.where(hit_crop, (hc - MOUNT_H) / dz2, tg[keep] if False else -MOUNT_H / dz2)
    # recompute crop hits at the canopy plane (small inward shift)
    x = xg.copy(); y = yg.copy(); z = np.zeros_like(xg)
    if hit_crop.any():
        tc = (hc[hit_crop] - MOUNT_H) / dz2[hit_crop]
        x[hit_crop] = tc * dx[hit_crop]
        y[hit_crop] = tc * dy2[hit_crop]
        z[hit_crop] = hc[hit_crop] + rng.normal(0, 0.015, hit_crop.sum())  # canopy texture
    z = z + gslope * y                                       # terrain grade
    z = z + rng.normal(0, 0.008, len(z))                     # range noise
    # store in the stack's convention: P[:,2] = height_above_ground − MOUNT_H
    P = np.column_stack([x, y, z - MOUNT_H])
    P = P[np.hypot(P[:, 0], P[:, 1]) >= SELF_R]              # self-filter blind zone
    return P


def load_scan(path: str) -> np.ndarray:
    p = Path(path)
    if p.suffix == ".npy":
        a = np.load(p)
        return a[:, :3].astype(float)
    if p.suffix == ".ply":
        # minimal binary/ascii PLY xyz reader
        import struct
        data = p.read_bytes()
        hdr_end = data.index(b"end_header\n") + len(b"end_header\n")
        header = data[:hdr_end].decode("ascii", "ignore")
        n = int([l for l in header.splitlines() if l.startswith("element vertex")][0].split()[-1])
        if "binary_little_endian" in header:
            # assume float x,y,z first
            rec = np.frombuffer(data[hdr_end:hdr_end + n * 12], dtype="<f4", count=n * 3)
            return rec.reshape(n, 3).astype(float)
        rows = [list(map(float, l.split()[:3])) for l in data[hdr_end:].decode().splitlines()[:n]]
        return np.array(rows)
    raise ValueError(f"unsupported scan file: {path}")


# ---------------------------------------------------------------------------
def render(P, lateral, heading_deg, spacing, out_base, *, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, FancyArrow

    h = P[:, 2] + MOUNT_H                     # height above ground for colour
    th = np.radians(heading_deg)
    plt.rcParams.update({"font.size": 11, "font.family": "DejaVu Sans"})
    fig = plt.figure(figsize=(17, 8.2), dpi=300)
    cmap = "viridis"
    vmin, vmax = -0.05, max(0.6, float(np.percentile(h, 98)))

    # ===================== LEFT: annotated bird's-eye =====================
    ax = fig.add_subplot(1, 2, 1)
    sc = ax.scatter(P[:, 0], P[:, 1], c=h, cmap=cmap, vmin=vmin, vmax=vmax,
                    s=6, alpha=0.85, edgecolors="none", zorder=3)

    # detection ROI
    ax.add_patch(Rectangle((-ROI_X, ROI_Y[0]), 2 * ROI_X, ROI_Y[1] - ROI_Y[0],
                           fill=False, ec="#222", ls="--", lw=1.6, zorder=5))
    ax.text(ROI_X + 0.05, ROI_Y[1], "detection ROI\n(|x|≤0.8, y∈[1.5,7])",
            fontsize=9, va="top", color="#222")

    # safety zones (translucent — green = clear)
    ax.add_patch(Rectangle((-FWD_HW, 0.2), 2 * FWD_HW, SAFE_DIST - 0.2,
                           fc="#2ca02c", ec="#2ca02c", alpha=0.10, lw=1, zorder=2))
    for sgn in (-1, 1):
        ax.add_patch(Rectangle((sgn * TIRE_TRACK - TIRE_HW, 0.2), 2 * TIRE_HW,
                               SAFE_DIST - 0.2, fc="#1f77b4", ec="#1f77b4",
                               alpha=0.10, lw=1, zorder=2))
    ax.text(0.66, 2.45, "forward\nsafety zone", ha="left", fontsize=8,
            color="#1a7d1a")
    ax.text(TIRE_TRACK, -0.05, "tire zone", ha="center", va="top", fontsize=8, color="#1f77b4")
    ax.text(-TIRE_TRACK, -0.05, "tire zone", ha="center", va="top", fontsize=8, color="#1f77b4")

    # self-filter blind zone
    ax.add_patch(Circle((0, 0), SELF_R, fill=True, fc="#bbbbbb", ec="#888",
                        ls=":", lw=1.2, alpha=0.20, zorder=1))
    ax.text(-1.02, 0.62, "self-filter\nblind zone\n(<1.5 m)", ha="right", fontsize=8,
            color="#555")

    # robot footprint + wheels
    ax.add_patch(Rectangle((-BODY_HW, BODY_BACK), 2 * BODY_HW, BODY_FWD - BODY_BACK,
                           fc="#444", ec="k", alpha=0.85, zorder=6))
    for sgn in (-1, 1):
        ax.add_patch(Rectangle((sgn * TIRE_TRACK - 0.09, BODY_BACK), 0.18, 0.34,
                               fc="#111", ec="k", zorder=6))
    ax.text(0, (BODY_BACK + BODY_FWD) / 2, "Amiga", color="w", ha="center",
            va="center", fontsize=8, zorder=7)

    # detected strip-centre (tracking target) + heading, and the flanking rows
    yy = np.array([ROI_Y[0], ROI_Y[1]])
    cx = lateral + yy * np.tan(th)
    ax.plot(cx, yy, color="#ff7f0e", lw=2.6, ls="--", zorder=8,
            label="detected strip-centre (target)")
    for sgn in (-1, 1):
        ax.plot(lateral + sgn * spacing / 2 + yy * np.tan(th), yy, color="#d62728",
                lw=1.4, alpha=0.7, zorder=7)
    ax.text(lateral + spacing / 2 + 0.04, ROI_Y[0] + 0.5, "soybean rows\n(±%.0f cm)"
            % (spacing / 2 * 100), fontsize=8, color="#d62728")

    # look-ahead point
    lx = lateral + LOOKAHEAD * np.tan(th)
    ax.scatter([lx], [LOOKAHEAD], s=130, marker="*", color="#ff7f0e",
               edgecolors="k", lw=0.6, zorder=9, label="pure-pursuit look-ahead")

    # forward-travel arrow
    ax.add_patch(FancyArrow(0, 0.45, 0, 1.45, width=0.05, head_width=0.22,
                            head_length=0.32, fc="#1f77b4", ec="#1f77b4", zorder=8))
    ax.text(0.22, 1.05, "forward\ntravel", color="#1f77b4", fontsize=9,
            fontweight="bold", ha="left", va="center")

    # cross-track offset annotation (magnitude at the robot: x=0 → x=lateral)
    yoff = 2.5
    ax.annotate("", xy=(lateral, yoff), xytext=(0, yoff),
                arrowprops=dict(arrowstyle="<->", color="#9467bd", lw=1.8), zorder=10)
    ax.plot([0, 0], [0.45, yoff], color="#9467bd", ls=":", lw=1.0, alpha=0.7, zorder=4)
    ax.text(-0.95, yoff, "cross-track\noffset %.0f cm" % (abs(lateral) * 100),
            color="#9467bd", fontsize=8, ha="right", va="center")
    # heading annotation
    ax.text(lateral + 5.7 * np.tan(th) + 0.12, 5.7, "heading\nerror %.1f°" % heading_deg,
            color="#ff7f0e", fontsize=8, rotation=0)

    ax.set_xlim(-3.0, 3.0)
    ax.set_ylim(-0.9, 9.2)
    ax.set_aspect("equal")
    ax.set_xlabel("x — lateral (m)   →  right")
    ax.set_ylabel("y — forward (m)   →  travel")
    ax.set_title("(a) Bird's-eye LiDAR view — robot frame, yaw+tilt corrected",
                 fontsize=12, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    # 1 m scale bar
    ax.plot([-2.7, -1.7], [-0.6, -0.6], "k-", lw=3)
    ax.text(-2.2, -0.5, "1 m", ha="center", fontsize=8)
    cb = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.02)
    cb.set_label("height above ground (m)")

    # ===================== RIGHT: 3-D perspective =====================
    ax3 = fig.add_subplot(1, 2, 2, projection="3d")
    # focus on the ROI neighbourhood so the two flanking rows are the clear
    # feature; split ground vs crop so the rows visibly rise off the plane.
    m = (P[:, 1] >= 1.3) & (P[:, 1] <= 7.6) & (np.abs(P[:, 0]) < 1.7)
    Pm, hm = P[m], h[m]
    g = hm < 0.045
    ax3.scatter(Pm[g, 0], Pm[g, 1], hm[g], c="#c9c9c9", s=3, alpha=0.35,
                edgecolors="none")                       # ground plane (muted)
    ax3.scatter(Pm[~g, 0], Pm[~g, 1], hm[~g], c=hm[~g], cmap=cmap, vmin=vmin,
                vmax=vmax, s=10, alpha=0.95, edgecolors="none")   # crop rows
    # robot box on the ground + forward arrow
    bx = [-BODY_HW, BODY_HW, BODY_HW, -BODY_HW, -BODY_HW]
    by = [BODY_BACK, BODY_BACK, BODY_FWD, BODY_FWD, BODY_BACK]
    ax3.plot(bx, by, [0] * 5, color="k", lw=2)
    ax3.quiver(0, 0.4, 0.02, 0, 1.5, 0, color="#1f77b4", lw=2.5, arrow_length_ratio=0.2)
    ax3.text(0.2, 1.8, 0.04, "forward", color="#1f77b4", fontsize=9, fontweight="bold")
    ax3.text(-0.62, 6.6, 0.30, "soybean rows", color="#2a7", fontsize=9)
    ax3.set_xlabel("x — lateral (m)", labelpad=2)
    ax3.set_ylabel("y — forward (m)", labelpad=6)
    ax3.set_zlabel("height above ground (m)", labelpad=2)
    ax3.set_xlim(-1.7, 1.7)
    ax3.set_ylim(1.3, 7.6)
    ax3.set_zlim(0, 0.45)
    try:
        ax3.set_box_aspect((4, 8, 1.8))
    except Exception:
        pass
    ax3.view_init(elev=20, azim=-60)
    ax3.set_title("(b) 3-D view — crop rows standing above the ground plane",
                  fontsize=12, fontweight="bold")

    sup = title or ("Autonomous row-following — VLP-16 LiDAR perception "
                    "(soybean centre-residue strip)")
    fig.suptitle(sup, fontsize=14, fontweight="bold", y=0.99)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    png, svg = out_base + ".png", out_base + ".svg"
    fig.savefig(png, dpi=300, bbox_inches="tight")
    fig.savefig(svg, bbox_inches="tight")
    print(f"wrote {png}\nwrote {svg}  ({len(P)} points)")
    return png, svg


def render_av3d(P, lateral, heading_deg, spacing, out_base, *, title=None):
    """Self-driving-car-style 3-D point-cloud render on a dark scene, with the
    ego robot and the perception/decision overlays (detected strip-centre path,
    look-ahead, ROI corridor, forward safety zone)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

    h = P[:, 2] + MOUNT_H
    th = np.radians(heading_deg)
    BG = "#0a0d14"
    plt.rcParams.update({"font.size": 11, "font.family": "DejaVu Sans",
                         "text.color": "#e8e8e8", "axes.labelcolor": "#cfd3da",
                         "xtick.color": "#9aa0aa", "ytick.color": "#9aa0aa"})
    fig = plt.figure(figsize=(16, 10), dpi=300, facecolor=BG)
    ax = fig.add_subplot(111, projection="3d", facecolor=BG)

    # point cloud — height-coloured on dark (classic Velodyne look)
    order = np.argsort(h)                      # draw low→high so rows pop
    sc = ax.scatter(P[order, 0], P[order, 1], h[order], c=h[order], cmap="turbo",
                    vmin=-0.05, vmax=0.45,
                    s=4.0, alpha=0.95, edgecolors="none", depthshade=False)

    yy = np.array([ROI_Y[0], ROI_Y[1]])
    cx = lateral + yy * np.tan(th)

    # --- decision overlays ---
    # detected strip-centre = planned path (glowing line just above ground)
    yp = np.linspace(SELF_R, ROI_Y[1], 40)
    xp = lateral + yp * np.tan(th)
    ax.plot(xp, yp, np.full_like(yp, 0.02), color="#00e5ff", lw=3.5, zorder=10)
    ax.plot(xp, yp, np.full_like(yp, 0.02), color="white", lw=1.0, alpha=0.7, zorder=11)
    # look-ahead target
    lx = lateral + LOOKAHEAD * np.tan(th)
    ax.scatter([lx], [LOOKAHEAD], [0.05], s=160, marker="*", color="#ffd000",
               edgecolors="k", lw=0.5, zorder=12)
    # ROI corridor floor (translucent)
    roi = [[(-ROI_X, ROI_Y[0], 0.0), (ROI_X, ROI_Y[0], 0.0),
            (ROI_X, ROI_Y[1], 0.0), (-ROI_X, ROI_Y[1], 0.0)]]
    pc = Poly3DCollection(roi, facecolor="#00e5ff", alpha=0.07, edgecolor="#00e5ff", lw=1.2)
    pc.set_zsort("min"); ax.add_collection3d(pc)
    # forward safety zone as a translucent 3-D volume
    _box(ax, -FWD_HW, FWD_HW, 0.2, SAFE_DIST, 0.0, 0.55, "#2ecc71", 0.06)

    # ego robot (Amiga) — wireframe box + forward arrow
    _box(ax, -BODY_HW, BODY_HW, BODY_BACK, BODY_FWD, 0.0, 0.45, "#ffffff", 0.10,
         edge="#ffffff", elw=1.8)
    ax.quiver(0, BODY_FWD, 0.22, 0, 1.6, 0, color="#00e5ff", lw=3, arrow_length_ratio=0.22)

    # scene styling — minimal, dark, AV-dashboard feel
    ax.set_xlim(-3.2, 3.2); ax.set_ylim(-1.5, 9.5); ax.set_zlim(0, 0.55)
    try:
        ax.set_box_aspect((6.4, 11, 1.1))
    except Exception:
        pass
    ax.view_init(elev=20, azim=-60)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.pane.set_facecolor(BG); pane.pane.set_edgecolor("#10161e")
        pane.pane.set_alpha(1.0)
        pane.line.set_color("#10161e")               # dim the axis spines
    ax.grid(True, color="#101822", linewidth=0.4)
    ax.set_xlabel("lateral  x (m)"); ax.set_ylabel("forward  y (m)")
    ax.set_zlabel("height (m)")
    ax.tick_params(labelsize=8)

    cb = fig.colorbar(sc, ax=ax, fraction=0.022, pad=0.02)
    cb.set_label("height above ground (m)", color="#cfd3da")
    cb.ax.yaxis.set_tick_params(color="#9aa0aa")
    cb.outline.set_edgecolor("#1c2430")

    # legend (proxy handles)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    handles = [
        Line2D([0], [0], color="#00e5ff", lw=3, label="planned path (strip-centre)"),
        Line2D([0], [0], marker="*", color="#ffd000", lw=0, markersize=12,
               markeredgecolor="k", label="look-ahead target"),
        Patch(facecolor="#2ecc71", alpha=0.5, label="forward safety zone"),
        Patch(facecolor="#00e5ff", alpha=0.35, label="detection ROI"),
        Patch(facecolor="#ffffff", alpha=0.6, label="ego robot (Amiga)"),
    ]
    leg = ax.legend(handles=handles, loc="upper left", fontsize=9,
                    facecolor="#10151d", edgecolor="#2a3340", labelcolor="#e8e8e8")

    sup = title or ("VLP-16 LiDAR perception — 3-D point cloud + autonomy decisions "
                    "(soybean row-following)")
    fig.suptitle(sup, fontsize=15, fontweight="bold", color="#ffffff", y=0.93)

    png = out_base + ".png"
    fig.savefig(png, dpi=300, facecolor=BG, bbox_inches="tight")
    print(f"wrote {png}  ({len(P)} points)")
    return png


def render_av_bev(P, lateral, heading_deg, spacing, out_base, *, title=None):
    """Top-down self-driving-style "sensor scope": the LiDAR rings from above on
    a dark scene, with range rings and the autonomy decision overlays."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    h = P[:, 2] + MOUNT_H
    th = np.radians(heading_deg)
    BG = "#0a0d14"
    plt.rcParams.update({"font.size": 11, "font.family": "DejaVu Sans",
                         "text.color": "#e8e8e8", "axes.labelcolor": "#cfd3da",
                         "xtick.color": "#9aa0aa", "ytick.color": "#9aa0aa"})
    fig = plt.figure(figsize=(9.5, 11), dpi=300, facecolor=BG)
    ax = fig.add_subplot(111, facecolor=BG)

    # faint range rings (classic sensor-scope look)
    for r in (2, 4, 6, 8):
        ax.add_patch(Circle((0, 0), r, fill=False, ec="#1b2531", lw=1.0, ls="-", zorder=1))
        ax.text(0.0, r, f"{r} m", color="#3a4658", fontsize=8, ha="center", va="bottom", zorder=1)

    order = np.argsort(h)
    sc = ax.scatter(P[order, 0], P[order, 1], c=h[order], cmap="turbo",
                    vmin=-0.05, vmax=0.45, s=4.0, alpha=0.95, edgecolors="none", zorder=3)

    # detection ROI + safety zones
    ax.add_patch(Rectangle((-ROI_X, ROI_Y[0]), 2 * ROI_X, ROI_Y[1] - ROI_Y[0],
                           fill=False, ec="#00e5ff", ls="--", lw=1.4, alpha=0.8, zorder=5))
    ax.add_patch(Rectangle((-FWD_HW, 0.2), 2 * FWD_HW, SAFE_DIST - 0.2,
                           fc="#2ecc71", ec="#2ecc71", alpha=0.10, lw=1, zorder=4))
    for sgn in (-1, 1):
        ax.add_patch(Rectangle((sgn * TIRE_TRACK - TIRE_HW, 0.2), 2 * TIRE_HW,
                               SAFE_DIST - 0.2, fc="#2ecc71", ec="#2ecc71",
                               alpha=0.07, lw=1, zorder=4))

    # planned path (strip-centre) + look-ahead
    yp = np.linspace(SELF_R, ROI_Y[1], 40)
    ax.plot(lateral + yp * np.tan(th), yp, color="#00e5ff", lw=3, zorder=8)
    ax.plot(lateral + yp * np.tan(th), yp, color="white", lw=1, alpha=0.6, zorder=9)
    lx = lateral + LOOKAHEAD * np.tan(th)
    ax.scatter([lx], [LOOKAHEAD], s=180, marker="*", color="#ffd000",
               edgecolors="k", lw=0.6, zorder=10)

    # self-filter blind zone + ego robot + forward arrow
    ax.add_patch(Circle((0, 0), SELF_R, fc="#11161f", ec="#222b38", ls=":", lw=1.2,
                        alpha=0.6, zorder=2))
    ax.add_patch(Rectangle((-BODY_HW, BODY_BACK), 2 * BODY_HW, BODY_FWD - BODY_BACK,
                           fc="#e8e8e8", ec="white", alpha=0.9, zorder=11))
    ax.annotate("", xy=(0, 1.7), xytext=(0, BODY_FWD),
                arrowprops=dict(arrowstyle="-|>", color="#00e5ff", lw=2.4), zorder=11)

    ax.set_xlim(-5, 5)
    ax.set_ylim(-2.5, 9.2)
    ax.set_aspect("equal")
    ax.set_xlabel("lateral  x (m)   →  right")
    ax.set_ylabel("forward  y (m)   →  travel")
    ax.tick_params(labelsize=8)
    for s in ax.spines.values():
        s.set_color("#1c2430")
    ax.grid(False)

    cb = fig.colorbar(sc, ax=ax, fraction=0.04, pad=0.02)
    cb.set_label("height above ground (m)", color="#cfd3da")
    cb.ax.yaxis.set_tick_params(color="#9aa0aa")
    cb.outline.set_edgecolor("#1c2430")

    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    handles = [
        Line2D([0], [0], color="#00e5ff", lw=3, label="planned path"),
        Line2D([0], [0], marker="*", color="#ffd000", lw=0, markersize=12,
               markeredgecolor="k", label="look-ahead"),
        Patch(facecolor="#2ecc71", alpha=0.4, label="safety zones"),
        Line2D([0], [0], color="#00e5ff", lw=1.4, ls="--", label="detection ROI"),
        Patch(facecolor="#e8e8e8", label="ego robot"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8.5, facecolor="#10151d",
              edgecolor="#2a3340", labelcolor="#e8e8e8")
    sup = title or ("LiDAR top-down sensor view — rings + detected path "
                    "(soybean row-following)")
    fig.suptitle(sup, fontsize=13, fontweight="bold", color="#ffffff", y=0.95)
    fig.tight_layout(rect=(0, 0, 1, 0.95))

    png = out_base + ".png"
    fig.savefig(png, dpi=300, facecolor=BG, bbox_inches="tight")
    print(f"wrote {png}  ({len(P)} points)")
    return png


def _box(ax, x0, x1, y0, y1, z0, z1, color, alpha, edge=None, elw=1.0):
    """Draw a translucent 3-D box (axis-aligned) on a 3-D axis."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [[v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]],
             [v[0], v[1], v[5], v[4]], [v[2], v[3], v[7], v[6]],
             [v[1], v[2], v[6], v[5]], [v[0], v[3], v[7], v[4]]]
    pc = Poly3DCollection(faces, facecolor=color, alpha=alpha,
                          edgecolor=edge or color, linewidths=elw)
    pc.set_zsort("average")
    ax.add_collection3d(pc)


def main() -> None:
    ap = argparse.ArgumentParser(description="LiDAR perception figure for poster/paper")
    ap.add_argument("--scan", help="real scan file (.npy Nx3 or .ply, robot frame)")
    ap.add_argument("--telemetry", help="seed scene geometry from this run's scan")
    ap.add_argument("--index", type=int, default=-1, help="scan index into --telemetry (FOLLOW)")
    ap.add_argument("--lateral", type=float, default=-0.05)
    ap.add_argument("--heading", type=float, default=3.5)
    ap.add_argument("--spacing", type=float, default=0.62)
    ap.add_argument("--grade", type=float, default=1.0)
    ap.add_argument("--out", default="results/lidar_perception", help="output base path")
    ap.add_argument("--title", default=None)
    ap.add_argument("--mode", choices=["annotated", "av3d", "av_bev"], default="annotated",
                    help="'annotated' = 2-panel schematic bird's-eye+3D; "
                         "'av3d' = dense self-driving-style 3-D point cloud (ray-cast "
                         "VLP-16 rings) with decision overlays on a dark scene; "
                         "'av_bev' = top-down sensor scope (rings from above + range "
                         "rings + detected path) on a dark scene.")
    args = ap.parse_args()

    lateral, heading, spacing = args.lateral, args.heading, args.spacing
    if args.telemetry:
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from navigation.run_metrics import load_jsonl
        F = [r for r in load_jsonl(args.telemetry) if r.get("state") == "FOLLOW"]
        if F:
            r = F[args.index]
            lateral, heading = r.get("lateral", lateral), r.get("heading_deg", heading)
            spacing = r.get("sp", spacing)
            print(f"seeded from telemetry: lateral={lateral:.3f} heading={heading:.2f} sp={spacing:.3f}")

    if args.scan:
        P = load_scan(args.scan)
        print(f"loaded real scan: {len(P)} points from {args.scan}")
    elif args.mode in ("av3d", "av_bev"):
        P = generate_lidar_raycast(lateral=lateral, heading_deg=heading,
                                   spacing=spacing, grade_deg=args.grade)
    else:
        P = generate_scene(lateral=lateral, heading_deg=heading, spacing=spacing,
                           grade_deg=args.grade)

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    if args.mode == "av3d":
        render_av3d(P, lateral, heading, spacing, args.out, title=args.title)
    elif args.mode == "av_bev":
        render_av_bev(P, lateral, heading, spacing, args.out, title=args.title)
    else:
        render(P, lateral, heading, spacing, args.out, title=args.title)


if __name__ == "__main__":
    main()
