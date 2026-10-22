"""
onion_field.py — ray-cast VLP-16 Puck Hi-Res scans of a Vidalia onion field,
at any growth stage.

Why a ray-caster: the hard part of the onion field is OCCLUSION.  Seedlings
are invisible (pencil-thin), so the robot steers by the raised bed; later the
canopy hides the bed-top soil, leaves of neighbouring rows interleave into one
continuous bed canopy, and at maturity the tops lodge and droop over the bed
shoulders.  Placing points directly (as the older unit tests do) cannot show
any of that — which surface the LiDAR actually returns from depends on the
beam geometry, the canopy porosity and what stands in front of what.

Model (world frame: beds run along +y, x to the right, h = height above the
furrow bottom the wheels run in):

  * Ground: periodic raised beds (pitch 1.83 m = 72 in), flat top, linear
    shoulders, furrows between; smooth soil roughness; optional bed END at
    ``y_end`` beyond which the headland is flat and grassy.
  * Crop: 4 rows per bed at 0.279 m (11 in), plants every 0.1016 m (4 in).
    Each plant is a cloud of leaf ELEMENTS (spheres of radius ``leaf_r``)
    whose lateral/along-row spread grows with height (onion leaves fan out).
    The element count sets the canopy's optical density, so porosity, partial
    occlusion of the soil and shadowing all come out of the ray-cast.
  * Sensor: the 16 Hi-Res beams (±10°, 1.33°), 0.2° azimuth steps (600 rpm),
    mounted ``height`` m above the furrow and pitched ``tilt_deg`` nose-down;
    range noise; nearest surface per beam (z-buffer over leaf elements +
    marched heightfield intersection).

Output is what ``RowNavigator`` feeds the detector: the yaw/tilt-CORRECTED
robot-frame cloud (x right, y forward, z up relative to the sensor, so
h = z + LIDAR_MOUNT_HEIGHT), self-filtered at planar range ≥ 1.5 m.

Growth stages (``STAGES``) follow a Vidalia sweet-onion season: transplants
(invisible to the LiDAR) → young (4–6 leaves, separate rows) → mid (bulb
initiation, rows full) → mature (leaves of neighbouring rows interleave: one
continuous bed canopy) → lodged (tops fallen over, broad, low, drooping past
the bed shoulders).  Beds also slump over the season, so relief drops.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Optional

import numpy as np

from lidar.lidar_driver import VLP16_VERTICAL_ANGLES_HIRES
from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT

PITCH = 1.83                     # 72 in bed centres
ROW_SP = 0.2794                  # 11 in
PLANT_SP = 0.1016                # 4 in


@dataclass
class Stage:
    name: str
    height: float                # canopy height above the bed top (m)
    spread_base: float           # lateral std of leaf elements at the base (m)
    spread_top: float            # lateral std at the canopy top (m)
    along: float                 # along-row std of a plant's elements (m)
    elems: int                   # leaf elements per plant (sets optical density)
    leaf_r: float                # leaf element radius (m)
    bed_h: float                 # bed relief above the furrow at this stage (m)
    droop: float = 0.0           # extra outward lean of the outer rows' leaves (m)
    z_pow: float = 1.0           # element height distribution: z = H·u**z_pow
    lodged: bool = False         # tops fallen over: elements low, spread wide


# Vidalia season.  Optical density per stage: horizontal optical depth across
# one row ≈ elems/PLANT_SP·π·leaf_r²/height — ~0.1 for transplants (mostly
# see-through), ~1 young, ~2–3 mid, >4 once rows interleave.
STAGES = {
    "bare":     Stage("bare", 0.0, 0.0, 0.0, 0.0, 0, 0.005, bed_h=0.10),
    "seedling": Stage("seedling", 0.07, 0.004, 0.008, 0.006, 3, 0.004, bed_h=0.10),
    "young":    Stage("young", 0.20, 0.010, 0.040, 0.020, 30, 0.010, bed_h=0.09),
    "mid":      Stage("mid", 0.35, 0.015, 0.080, 0.040, 70, 0.015, bed_h=0.08),
    "mature":   Stage("mature", 0.50, 0.020, 0.150, 0.060, 110, 0.020, bed_h=0.07,
                      droop=0.06),
    "lodged":   Stage("lodged", 0.28, 0.080, 0.200, 0.080, 120, 0.020, bed_h=0.06,
                      droop=0.15, z_pow=1.8, lodged=True),
}


@dataclass
class FieldConfig:
    stage: Stage = field(default_factory=lambda: STAGES["mid"])
    bed_top: float = 1.05        # bed-top width (m)
    shoulder: float = 0.22       # shoulder run (m)
    bed_x0: float = 0.0          # world x of the reference bed centre
    rows: tuple = (0, 1, 2, 3)   # which of the 4 rows are planted (skips)
    y_end: Optional[float] = None    # bed END (world y); None = beds run on
    grass_h: float = 0.22        # headland grass height (m)
    weeds_per_m2: float = 0.0    # weed clumps on bed/furrow
    roughness: float = 0.008     # soil roughness amplitude (m)
    gap_prob: float = 0.0        # fraction of plants missing (random stand gaps)


@dataclass
class SensorConfig:
    height: float = LIDAR_MOUNT_HEIGHT
    tilt_deg: float = 12.3       # field calibration (2026-10)
    az_half_deg: float = 62.0    # simulate only the forward sector the detector uses
    az_step_deg: float = 0.2
    beam_half_div_deg: float = 0.08
    range_noise: float = 0.015
    max_range: float = 14.0
    self_radius: float = 1.5
    tilt_err_deg: float = 0.0    # residual calibration error left in the cloud


_EL = np.radians(np.asarray(VLP16_VERTICAL_ANGLES_HIRES, dtype=float))


class OnionField:
    """A static field instance (plant/weed layout fixed by ``seed``)."""

    def __init__(self, cfg: FieldConfig = None, seed: int = 0):
        self.cfg = cfg or FieldConfig()
        self.seed = seed
        rng = np.random.default_rng(seed)
        self._ph = rng.uniform(0, 2 * math.pi, 6)
        self._plant_jit = rng.integers(0, 2**31 - 1)

    # ------------------------------------------------------------------ ground
    def ground_h(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        c = self.cfg
        u = np.mod(x - c.bed_x0 + 0.5 * PITCH, PITCH) - 0.5 * PITCH
        a = np.abs(u)
        t = 0.5 * c.bed_top
        bh = c.stage.bed_h
        h = np.where(a <= t, bh, np.clip(bh * (1.0 - (a - t) / c.shoulder), 0.0, bh))
        p = self._ph
        h = h + c.roughness * (0.6 * np.sin(7.1 * x + p[0]) * np.sin(5.3 * y + p[1])
                               + 0.4 * np.sin(19.0 * x + p[2]) * np.sin(13.0 * y + p[3]))
        if c.y_end is not None:
            flat = c.roughness * 0.5 * np.sin(3.0 * x + p[4]) * np.sin(4.0 * y + p[5])
            h = np.where(y > c.y_end, flat, h)
        return h

    # ------------------------------------------------------------ vegetation
    def _elements(self, xr: float, yr: float, rng: np.random.Generator):
        """Leaf / weed / grass elements around the robot: (N,3) centres, (N,) radii."""
        c = self.cfg
        st = c.stage
        X, R = [], []
        x_lo, x_hi = xr - 2.9, xr + 2.9
        y_lo, y_hi = yr - 0.5, yr + 11.5
        if st.elems > 0 and st.height > 0:
            k0 = int(math.floor((x_lo - c.bed_x0) / PITCH)) - 1
            k1 = int(math.ceil((x_hi - c.bed_x0) / PITCH)) + 1
            j0 = int(math.floor(y_lo / PLANT_SP))
            j1 = int(math.ceil(y_hi / PLANT_SP))
            js = np.arange(j0, j1)
            for k in range(k0, k1 + 1):
                bc = c.bed_x0 + k * PITCH
                for r in c.rows:
                    off = (r - 1.5) * ROW_SP
                    rx = bc + off
                    if rx < x_lo - 0.4 or rx > x_hi + 0.4:
                        continue
                    # deterministic per-plant layout (stand gaps, jitter)
                    prng = np.random.default_rng(
                        (self._plant_jit + 7919 * (k + 1000) + 104729 * r) % (2**31 - 1))
                    jit_all = prng.normal(0, 0.01, (len(js) + 2000,))
                    gap_all = prng.random(len(js) + 2000)
                    idx = js - j0 + 1000 + 0 * js
                    py = js * PLANT_SP + jit_all[np.clip(js % 2000, 0, 1999)]
                    keep = gap_all[np.clip(js % 2000, 0, 1999)] >= c.gap_prob
                    if c.y_end is not None:
                        keep &= py < c.y_end
                    py = py[keep]
                    if len(py) == 0:
                        continue
                    n = st.elems
                    u = rng.random((len(py), n))
                    zf = u ** st.z_pow                       # fraction of height
                    sig = st.spread_base + (st.spread_top - st.spread_base) * zf
                    lean = st.droop * zf * np.sign(off) * (abs(off) > ROW_SP)   # outer rows droop out
                    ex = rx + lean + rng.normal(0, 1, (len(py), n)) * sig
                    ey = py[:, None] + rng.normal(0, 1, (len(py), n)) * (st.along * (0.4 + zf))
                    if st.lodged:
                        ez = st.height * zf * (0.35 + 0.65 * rng.random((len(py), n)))
                    else:
                        ez = st.height * zf
                    pts = np.column_stack((ex.ravel(), ey.ravel(), ez.ravel()))
                    m = ((pts[:, 0] > x_lo) & (pts[:, 0] < x_hi)
                         & (pts[:, 1] > y_lo) & (pts[:, 1] < y_hi))
                    pts = pts[m]
                    pts[:, 2] += self.ground_h(pts[:, 0], pts[:, 1])
                    X.append(pts)
                    R.append(np.full(len(pts), st.leaf_r))
        area = (x_hi - x_lo) * (y_hi - y_lo)
        if c.weeds_per_m2 > 0:
            nw = rng.poisson(c.weeds_per_m2 * area)
            cx = rng.uniform(x_lo, x_hi, nw)
            cy = rng.uniform(y_lo, y_hi, nw)
            per = 25
            wx = (cx[:, None] + rng.normal(0, 0.05, (nw, per))).ravel()
            wy = (cy[:, None] + rng.normal(0, 0.05, (nw, per))).ravel()
            wz = (rng.uniform(0.03, 0.25, (nw, 1)) * rng.random((nw, per))).ravel()
            wz += self.ground_h(wx, wy)
            X.append(np.column_stack((wx, wy, wz)))
            R.append(np.full(len(wx), 0.012))
        if c.y_end is not None and c.y_end < y_hi:
            g_lo = max(c.y_end, y_lo)
            ng = int(1800 * (x_hi - x_lo) * (y_hi - g_lo))
            gx = rng.uniform(x_lo, x_hi, ng)
            gy = rng.uniform(g_lo, y_hi, ng)
            gz = c.grass_h * rng.random(ng) ** 0.7 + self.ground_h(gx, gy)
            X.append(np.column_stack((gx, gy, gz)))
            R.append(np.full(ng, 0.012))
        if not X:
            return np.empty((0, 3)), np.empty(0)
        return np.vstack(X), np.concatenate(R)

    # ------------------------------------------------------------------ scan
    def scan(self, xr: float = 0.0, yr: float = 0.0, yaw_deg: float = 0.0,
             sensor: SensorConfig = None, rng: Optional[np.random.Generator] = None,
             return_truth: bool = False):
        """One corrected robot-frame scan with the sensor at world (xr, yr).

        ``yaw_deg``: robot heading relative to the bed axis (+ = turned toward
        +x / right).  Returns (N,3) points; with ``return_truth`` also a dict
        with the true bed-centre lateral offset and heading in the detector's
        convention (lateral +: bed centre to the RIGHT of the sensor; heading:
        angle of the bed axis in the robot frame, atan2(dir_x, dir_y))."""
        s = sensor or SensorConfig()
        rng = rng or np.random.default_rng()
        psi = math.radians(yaw_deg)
        cps, sps = math.cos(psi), math.sin(psi)
        tilt = math.radians(s.tilt_deg)
        ct, stt = math.cos(tilt), math.sin(tilt)

        az = np.radians(np.arange(-s.az_half_deg, s.az_half_deg + 1e-9, s.az_step_deg))
        nb = len(az)
        EL, AZ = np.meshgrid(_EL, az, indexing="ij")          # (16, nb)
        dsx = np.cos(EL) * np.sin(AZ)
        dsy = np.cos(EL) * np.cos(AZ)
        dsz = np.sin(EL)
        # sensor → robot (undo nose-down pitch) → world (yaw)
        dry = dsy * ct + dsz * stt
        drz = -dsy * stt + dsz * ct
        drx = dsx
        dwx = drx * cps + dry * sps
        dwy = -drx * sps + dry * cps
        dwz = drz
        H = s.height

        # --- ground: march the heightfield, then bisect -------------------
        rng_g = np.full(dwx.shape, np.inf)
        down = dwz < -1e-3
        if down.any():
            gx, gy, gz = dwx[down], dwy[down], dwz[down]
            top = self.cfg.stage.bed_h + 3 * self.cfg.roughness + 0.02
            s0 = np.maximum((H - top) / -gz, 0.0)
            s1 = np.minimum((H + 3 * self.cfg.roughness + 0.01) / -gz, s.max_range)
            step = 0.03
            nst = int(np.ceil(float(np.max(s1 - s0)) / step)) + 1
            res = np.full(len(gx), np.inf)
            prev = s0.copy()
            alive = s0 < s1
            for i in range(1, nst + 1):
                if not alive.any():
                    break
                si = np.minimum(s0 + i * step, s1)
                ai = np.nonzero(alive)[0]
                zr = H + gz[ai] * si[ai]
                zg = self.ground_h(xr + gx[ai] * si[ai], yr + gy[ai] * si[ai])
                hit = zr <= zg
                if hit.any():
                    hi = ai[hit]
                    lo_s, hi_s = prev[hi].copy(), si[hi].copy()
                    for _ in range(6):
                        mid = 0.5 * (lo_s + hi_s)
                        below = (H + gz[hi] * mid) <= self.ground_h(xr + gx[hi] * mid,
                                                                     yr + gy[hi] * mid)
                        hi_s = np.where(below, mid, hi_s)
                        lo_s = np.where(below, lo_s, mid)
                    res[hi] = hi_s
                    alive[hi] = False
                prev[ai] = si[ai]
                alive &= si < s1
            rng_g[down] = res

        # --- vegetation: z-buffer leaf elements onto the beam grid ----------
        rng_v = np.full(dwx.shape, np.inf)
        E, Rr = self._elements(xr, yr, rng)
        if len(E):
            # world → robot (inverse yaw) → sensor (re-apply pitch)
            vx, vy, vz = E[:, 0] - xr, E[:, 1] - yr, E[:, 2] - H
            rx = vx * cps - vy * sps
            ry = vx * sps + vy * cps
            sy = ry * ct - vz * stt
            sz = ry * stt + vz * ct
            sx = rx
            rr = np.sqrt(sx * sx + sy * sy + sz * sz)
            ok = (rr > 0.3) & (rr < s.max_range)
            sx, sy, sz, rr, Rr = sx[ok], sy[ok], sz[ok], rr[ok], Rr[ok]
            el = np.arcsin(np.clip(sz / rr, -1, 1))
            a = np.arctan2(sx, sy)
            alpha = np.arcsin(np.clip(Rr / rr, 0, 1)) + math.radians(s.beam_half_div_deg)
            a0 = az[0]
            astep = math.radians(s.az_step_deg)
            for ch in range(16):
                d = el - _EL[ch]
                m = np.abs(d) < alpha
                if not m.any():
                    continue
                half = np.sqrt(np.maximum(alpha[m] ** 2 - d[m] ** 2, 0.0))
                b0 = np.ceil((a[m] - half - a0) / astep).astype(int)
                b1 = np.floor((a[m] + half - a0) / astep).astype(int)
                rv = rr[m] - Rr[m]
                span = b1 - b0 + 1
                okb = span > 0
                b0, span, rv = b0[okb], span[okb], rv[okb]
                if not len(b0):
                    continue
                rep = np.repeat(np.arange(len(b0)), span)
                offs = np.arange(len(rep)) - np.repeat(np.cumsum(span) - span, span)
                bins = b0[rep] + offs
                vals = rv[rep]
                inb = (bins >= 0) & (bins < nb)
                row = rng_v[ch]
                np.minimum.at(row, bins[inb], vals[inb])

        R = np.minimum(rng_g, rng_v)
        hitm = np.isfinite(R)
        Rn = R[hitm] + rng.normal(0, s.range_noise, int(hitm.sum()))
        px = dwx[hitm] * Rn
        py = dwy[hitm] * Rn
        pz = H + dwz[hitm] * Rn
        # world offsets → robot frame
        rx = px * cps - py * sps
        ry = px * sps + py * cps
        P = np.column_stack((rx, ry, pz - LIDAR_MOUNT_HEIGHT))
        P = P[np.hypot(P[:, 0], P[:, 1]) >= s.self_radius]
        if s.tilt_err_deg:
            from lidar.obstacle_filter import tilt_correct_pts
            P = tilt_correct_pts(P, math.radians(s.tilt_err_deg))
        if not return_truth:
            return P
        # Truth in the detector's convention: nearest bed centre to the robot.
        k = round((xr - self.cfg.bed_x0) / PITCH)
        bcx = self.cfg.bed_x0 + k * PITCH
        lat = (bcx - xr) * cps            # perpendicular distance, + = right
        hdg = -psi                        # bed axis seen from the robot
        return P, dict(lateral=lat, heading=hdg, bed_centre_x=bcx)


def stage_config(name: str, **kw) -> FieldConfig:
    st = STAGES[name]
    if "bed_h" in kw:
        st = replace(st, bed_h=kw.pop("bed_h"))
    return FieldConfig(stage=st, **kw)
