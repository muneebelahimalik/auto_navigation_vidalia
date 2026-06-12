#!/usr/bin/env python3
"""
soybean_row_tracker.py — Dual-camera centre-residue tracker for soybean fields,
built for the REAL mount geometry: two FORWARD-FACING OAK-Ds with a 15°
nose-down pitch, one above each tire (±0.88 m from centreline, 0.92 m high,
46.5 cm behind the LiDAR).  Both cameras see almost the same forward scene —
BOTH flanking soybean rows and the residue strip — from laterally shifted
viewpoints (a 1.76 m wide-baseline pair).

How it works (per camera, per frame)
------------------------------------
1.  HSV green mask of the RGB frame (crop vegetation).
2.  Ground-plane projection (inverse perspective mapping): every green pixel
    is ray-cast through the known camera intrinsics + extrinsics onto the
    plane z = canopy_z.  This produces METRIC (x, y) crop points in the robot
    frame — the same frame and convention as the LiDAR detector — without
    using stereo depth (which is noisy on thin seedlings at 1–4 m).
3.  Optional depth sanity gate: pixels whose stereo depth is far SHORTER than
    the ground-ray distance are looking at something tall standing above the
    crop (person, equipment), not at row vegetation — they are discarded so
    obstacles cannot smear green into the row estimate.
4.  The projected point cloud is fitted exactly like a LiDAR scan: PCA → row
    heading; histogram of the cross-row coordinate → flanking-row peaks;
    ``find_row_midpoint`` (shared with the LiDAR detector) applies the
    row-spacing prior and the single-side half-spacing fallback.

Each camera therefore yields an INDEPENDENT metric estimate of the SAME
residue-strip centre.  Fusion is then trivial and well-founded:

  * centre  = mean of the two per-camera midpoints
  * heading = mean of the two per-camera headings
  * agreement between the cameras scales confidence

Why the plain mean (and why two cameras beat one)
-------------------------------------------------
Ground projection assumes the green pixels sit at height ``canopy_z``.  Real
canopy tops sit a few cm higher, so each ray overshoots: every projected
point moves radially AWAY from its camera by a factor k ≈ cam_z/(cam_z − h).
For the LEFT camera (at −0.88 m) that pushes both rows — and their midpoint —
to the RIGHT by ≈ 0.88·(k−1); for the RIGHT camera the same bias points LEFT.
The two per-camera midpoints are biased in OPPOSITE directions by the same
magnitude, so their unweighted mean cancels the canopy-height bias to first
order.  This is the real common-mode-rejection benefit of the wide-baseline
pair, and it is why fusion uses an equal-weight mean rather than a
strength-weighted one.  Graceful degradation is also stronger than the old
side-assigned design: ONE camera alone still sees both rows and still
produces a full midpoint estimate (at reduced confidence).

The output ``DualRowEstimate`` keeps the attribute contract the navigators
and EKF consume (``lateral_offset``, ``heading_error``, ``confidence``,
``green_fraction``), so this remains a drop-in for ``VisualRowDetector``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from navigation.row_perception import find_row_midpoint

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[soybean_row_tracker] opencv not installed — dual-camera tracking disabled")

# Default OAK-D stereo intrinsics at 640×400 (matches oak_driver / depth_to_points).
# Auto-scaled proportionally to the actual frame size.
_DEFAULT_FX = 452.0
_DEFAULT_FY = 452.0
_DEFAULT_CX = 320.0
_DEFAULT_CY = 200.0
_DEFAULT_W = 640
_DEFAULT_H = 400

# Rotation: camera optical frame (x=right, y=down, z=into-scene) → robot body
# frame (X=right, Y=forward, Z=up) at zero yaw/pitch.  Same as depth_to_points.
_R_OPT2BODY = np.array([
    [1.0,  0.0,  0.0],
    [0.0,  0.0,  1.0],
    [0.0, -1.0,  0.0],
], dtype=np.float64)


def _rx(pitch: float) -> np.ndarray:
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _pca_dir(Q: np.ndarray) -> "tuple[np.ndarray, float]":
    """Principal direction (forward-positive) and linearity of centred 2D points."""
    cov = (Q.T @ Q) / len(Q)
    evals, evecs = np.linalg.eigh(cov)
    direction = evecs[:, 1]
    if direction[1] < 0.0:
        direction = -direction
    lam_major, lam_minor = float(evals[1]), float(evals[0])
    linearity = (lam_major - lam_minor) / (lam_major + lam_minor + 1e-9)
    return direction, linearity


@dataclass
class DualRowEstimate:
    """Residue-strip centre estimate fused from the two forward cameras."""
    lateral_offset: float = 0.0     # m; +ve = residue-strip centre is to robot's right
    heading_error: float = 0.0      # rad; +ve = rows angled to robot's right
    confidence: float = 0.0         # 0..1 (camera-only range capped at ~0.5)
    green_fraction: float = 0.0     # max green fraction across both cameras (row-end cue)
    left_valid: bool = False        # left camera produced a centre estimate this frame
    right_valid: bool = False       # right camera produced a centre estimate this frame
    left_offset: float = math.nan   # left camera's centre estimate (m, robot frame)
    right_offset: float = math.nan  # right camera's centre estimate (m, robot frame)


@dataclass
class _SideResult:
    """One camera's independent estimate of the residue-strip centre."""
    valid: bool = False
    world_x: float = 0.0       # estimated residue-strip centre (m, robot frame)
    heading: float = 0.0       # row heading error (rad)
    green_fraction: float = 0.0
    strength: float = 0.0      # number of projected crop points (diagnostic)
    quality: float = 0.0       # 0..1 fit quality (density × linearity × spacing)


class DualCameraRowTracker:
    """Track the residue-strip centre with two forward-facing OAK-Ds.

    Drop-in for ``VisualRowDetector``: identical ``update()`` signature, and
    the returned estimate exposes the attributes the navigator/EKF consume.
    """

    def __init__(
        self,
        cam_x_left: float = -0.88,
        cam_x_right: float = 0.88,
        row_spacing: float = 0.76,
        cam_y_fwd: float = -0.465,
        cam_z: float = 0.920,
        cam_pitch_deg: float = 15.0,
        green_h_lo: int = 35,
        green_h_hi: int = 85,
        green_s_lo: int = 40,
        green_v_lo: int = 40,
        roi_y_min: float = 0.8,
        roi_y_max: float = 5.0,
        roi_x_half: float = 0.90,
        bin_width: float = 0.05,
        canopy_z: float = 0.10,
        depth_gate_ratio: float = 0.5,
        min_points: int = 60,
        full_points: int = 600,
        min_green_fraction: float = 0.02,
        pixel_step: int = 2,
        ema_alpha: float = 0.30,
        fx: Optional[float] = None,
        fy: Optional[float] = None,
        cx: Optional[float] = None,
        cy: Optional[float] = None,
    ) -> None:
        """
        Parameters
        ----------
        cam_x_left / cam_x_right : signed lateral camera offsets (m).
        row_spacing : flanking-row separation (m) — peak-pairing prior and
            single-side fallback distance.
        cam_y_fwd : camera forward offset from the LiDAR origin (m, negative =
            behind), so projected points land in the LiDAR/robot frame.
        cam_z / cam_pitch_deg : mount height (m) and nose-down pitch (deg).
        roi_y_min/max : forward band of projected points kept (m).  Beyond
            ~5 m a one-pixel error spans tens of cm — those rows are excluded.
        roi_x_half : lateral ROI half-width in the ROBOT frame (m); 0.90 keeps
            both flanking rows (±0.38) and excludes the next rows out (±1.14).
        canopy_z : assumed height of the green pixels above ground (m).  Set
            to roughly half the typical canopy height; residual error is
            common-mode between the two cameras and cancels in the fused mean.
        depth_gate_ratio : reject a green pixel when its stereo depth is
            shorter than this fraction of the ground-ray distance — it is
            then a TALL object (person/equipment), not crop.  At cam_z=0.92,
            ratio 0.5 rejects objects taller than ≈0.46 m and keeps seedling
            canopy.  Pixels with no valid depth are kept (gate is advisory).
        pixel_step : process every Nth pixel row/col of the mask (speed).
        """
        self.cam_x_left = cam_x_left
        self.cam_x_right = cam_x_right
        self.row_spacing = row_spacing
        self.cam_y_fwd = cam_y_fwd
        self.cam_z = cam_z
        self.cam_pitch = math.radians(cam_pitch_deg)
        self.green_h_lo = green_h_lo
        self.green_h_hi = green_h_hi
        self.green_s_lo = green_s_lo
        self.green_v_lo = green_v_lo
        self.roi_y_min = roi_y_min
        self.roi_y_max = roi_y_max
        self.roi_x_half = roi_x_half
        self.bin_width = bin_width
        self.canopy_z = canopy_z
        self.depth_gate_ratio = depth_gate_ratio
        self.min_points = min_points
        self.full_points = full_points
        self.min_green_fraction = min_green_fraction
        self.pixel_step = max(1, pixel_step)
        self.ema_alpha = ema_alpha

        self._user_intrinsics = not (fx is None and fy is None and cx is None and cy is None)
        self._fx = fx if fx is not None else _DEFAULT_FX
        self._fy = fy if fy is not None else _DEFAULT_FY
        self._cx = cx if cx is not None else _DEFAULT_CX
        self._cy = cy if cy is not None else _DEFAULT_CY

        # Both cameras share pitch and zero yaw, so the per-pixel ground maps
        # differ only by the constant cam_x offset → one shared cache.
        self._R = _rx(-self.cam_pitch) @ _R_OPT2BODY
        self._maps_hw: Optional[tuple] = None
        self._m_sdx: Optional[np.ndarray] = None    # s·dx  (add cam_x for ground x)
        self._m_gy: Optional[np.ndarray] = None     # ground y (robot frame)
        self._m_s: Optional[np.ndarray] = None      # expected stereo depth (m)
        self._m_valid: Optional[np.ndarray] = None  # ray hits ground inside fwd ROI

        self._est = DualRowEstimate()
        # Metric robot-frame ground points from the most recent update() —
        # consumed by RowDetector point-level fusion (--cam-fusion point).
        self.last_ground_points: Optional[np.ndarray] = None
        self._last_side_pts: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the smoothed estimate (call when starting a new row)."""
        self._est = DualRowEstimate()
        self.last_ground_points = None

    # ------------------------------------------------------------------
    def update(
        self,
        rgb_left: Optional[np.ndarray],
        depth_left: Optional[np.ndarray],
        rgb_right: Optional[np.ndarray],
        depth_right: Optional[np.ndarray],
    ) -> DualRowEstimate:
        """Estimate the residue-strip centre from both forward cameras."""
        self.last_ground_points = None
        if not _CV2_OK:
            return DualRowEstimate()

        self._last_side_pts = None
        left = self._process_side(rgb_left, depth_left, self.cam_x_left)
        pts_l = self._last_side_pts

        self._last_side_pts = None
        right = self._process_side(rgb_right, depth_right, self.cam_x_right)
        pts_r = self._last_side_pts

        parts = [p for p in (pts_l, pts_r) if p is not None and len(p)]
        if parts:
            self.last_ground_points = np.vstack(parts)
        return self._fuse(left, right)

    # ------------------------------------------------------------------
    def _fuse(self, left: _SideResult, right: _SideResult) -> DualRowEstimate:
        """Combine the two per-camera centre estimates.

        Both cameras estimate the SAME quantity, so fusion is an equal-weight
        mean: the canopy-height projection bias of the two cameras is equal
        and opposite (see module docstring) and cancels in the mean.  The
        cameras' agreement scales confidence — two independent viewpoints
        that land on the same centre corroborate each other; disagreement
        signals occlusion or a wrong peak on one side.

        Separated from ``update()`` (which does the OpenCV image processing)
        so the fusion logic is unit-testable without an OpenCV dependency.
        """
        green = max(left.green_fraction, right.green_fraction)

        if left.valid and right.valid:
            centre = 0.5 * (left.world_x + right.world_x)
            heading = 0.5 * (left.heading + right.heading)
            agreement = max(0.0, 1.0 - abs(left.world_x - right.world_x) / 0.30)
            quality = 0.5 * (left.quality + right.quality)
            confidence = min(0.50, (0.30 + 0.20 * agreement) * (0.7 + 0.3 * quality))
        elif left.valid or right.valid:
            side = left if left.valid else right
            centre = side.world_x
            heading = side.heading
            confidence = 0.18 + 0.07 * side.quality      # 0.18 .. 0.25
        else:
            return self._decay()

        return self._smooth(DualRowEstimate(
            lateral_offset=centre,
            heading_error=heading,
            confidence=confidence,
            green_fraction=green,
            left_valid=left.valid,
            right_valid=right.valid,
            left_offset=left.world_x if left.valid else math.nan,
            right_offset=right.world_x if right.valid else math.nan,
        ))

    # ------------------------------------------------------------------
    def _process_side(
        self,
        rgb: Optional[np.ndarray],
        depth: Optional[np.ndarray],
        cam_x: float,
    ) -> _SideResult:
        """One camera: green mask → ground projection → row fit."""
        if rgb is None:
            return _SideResult()
        mask = self._green_mask(rgb)
        return self._side_from_mask(mask, cam_x, depth)

    # ------------------------------------------------------------------
    def _green_mask(self, rgb: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower = np.array([self.green_h_lo, self.green_s_lo, self.green_v_lo], dtype=np.uint8)
        upper = np.array([self.green_h_hi, 255, 255], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    # ------------------------------------------------------------------
    def _side_from_mask(
        self,
        mask: np.ndarray,
        cam_x: float,
        depth: Optional[np.ndarray] = None,
    ) -> _SideResult:
        """Geometry core (numpy only — unit-testable without OpenCV).

        ``mask``: HxW boolean or 0/255 uint8 green mask.
        """
        h, w = int(mask.shape[0]), int(mask.shape[1])
        self._build_maps(h, w)

        step = self.pixel_step
        sub = (mask[::step, ::step] > 0)
        valid = self._m_valid[::step, ::step]

        # Green fraction over the lower 65 % of the frame (row-end cue),
        # before any geometric filtering.
        row_lo = int(mask.shape[0] * 0.35)
        lower_part = mask[row_lo:, :]
        frac = float((lower_part > 0).sum()) / max(lower_part.size, 1)

        vs, us = np.nonzero(sub & valid)
        if len(vs) == 0:
            return _SideResult(green_fraction=frac)
        vs_full = vs * step
        us_full = us * step

        gx = cam_x + self._m_sdx[vs_full, us_full]
        gy = self._m_gy[vs_full, us_full]

        # Depth sanity gate: a stereo depth much SHORTER than the ground-ray
        # distance means the pixel looks at a tall object standing above the
        # crop, not at row vegetation lying near the ground plane.
        if depth is not None and depth.size and self.depth_gate_ratio > 0.0:
            dh, dw = int(depth.shape[0]), int(depth.shape[1])
            dv = (vs_full * dh) // h
            du = (us_full * dw) // w
            d_m = depth[dv, du].astype(np.float64) * 1e-3
            expected = self._m_s[vs_full, us_full]
            keep = (d_m <= 0.05) | (d_m >= self.depth_gate_ratio * expected)
            gx, gy = gx[keep], gy[keep]

        in_roi = np.abs(gx) <= self.roi_x_half
        gx, gy = gx[in_roi], gy[in_roi]
        n = int(gx.shape[0])
        if n < self.min_points:
            return _SideResult(green_fraction=frac)

        P = np.column_stack((gx, gy))

        # --- Pass 1: whole-cloud PCA (approximate heading) ---
        # With an OFF-CENTRE camera the two rows have different visible
        # extents — the outer row's near end falls outside the FOV — so the
        # cross-covariance between the two stripes tilts the whole-cloud
        # principal axis by a few degrees, which rotates the cross axis and
        # biases the midpoint toward the camera.
        direction, _ = _pca_dir(P - P.mean(axis=0))
        perp = np.array([direction[1], -direction[0]])
        cross = P @ perp
        lateral, _ = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, self.row_spacing)

        # --- Pass 2: cluster-centred PCA (exact within-row heading) ---
        # Split the points into the two row clusters around the pass-1
        # midpoint, centre each cluster on its own centroid, and re-run the
        # PCA on the pooled centred points.  Per-cluster centring removes
        # the between-stripe covariance entirely, leaving only the
        # within-row direction — unbiased regardless of visible extents.
        centred = []
        for C in (P[cross < lateral], P[cross >= lateral]):
            if len(C) >= max(20, int(0.15 * n)):
                centred.append(C - C.mean(axis=0))
        if centred:
            direction, linearity = _pca_dir(np.vstack(centred))
            perp = np.array([direction[1], -direction[0]])
            cross = P @ perp
        else:
            _, linearity = _pca_dir(P - P.mean(axis=0))
        heading = math.atan2(direction[0], direction[1])
        lateral, spacing_factor = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, self.row_spacing)

        density = min(1.0, n / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.15) / 0.55))
        quality = density * linear_factor * spacing_factor

        if frac < self.min_green_fraction or quality <= 0.0:
            return _SideResult(green_fraction=frac)

        # Expose the gated metric points for point-level fusion (only from
        # sides that passed every quality gate above).
        self._last_side_pts = P

        return _SideResult(
            valid=True,
            world_x=float(lateral),
            heading=heading,
            green_fraction=frac,
            strength=float(n),
            quality=quality,
        )

    # ------------------------------------------------------------------
    def _build_maps(self, h: int, w: int) -> None:
        """Precompute per-pixel ground-plane intersections (shared by both
        cameras — they differ only by the constant cam_x offset)."""
        if self._maps_hw == (h, w):
            return
        if self._user_intrinsics:
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy
        else:
            fx = self._fx * (w / _DEFAULT_W)
            fy = self._fy * (h / _DEFAULT_H)
            cx = self._cx * (w / _DEFAULT_W)
            cy = self._cy * (h / _DEFAULT_H)
            if self._maps_hw is None and (w != _DEFAULT_W or h != _DEFAULT_H):
                print(f"[soybean_row_tracker] auto-scaled intrinsics for {w}×{h}: "
                      f"fx={fx:.0f} fy={fy:.0f}")

        us = np.arange(w, dtype=np.float64)
        vs = np.arange(h, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)
        d_cam = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(uu)], axis=-1)
        d_rob = d_cam @ self._R.T                       # (H, W, 3) robot-frame rays

        dz = d_rob[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (self.canopy_z - self.cam_z) / dz       # ray scale to the canopy plane
        gy = self.cam_y_fwd + s * d_rob[..., 1]

        # Valid: ray points downward and lands inside the forward ROI.
        valid = (dz < -1e-3) & (s > 0.2) & (gy >= self.roi_y_min) & (gy <= self.roi_y_max)

        self._m_sdx = s * d_rob[..., 0]
        self._m_gy = gy
        self._m_s = s                                   # = expected stereo (z) depth
        self._m_valid = valid
        self._maps_hw = (h, w)

    # ------------------------------------------------------------------
    def _smooth(self, fresh: DualRowEstimate) -> DualRowEstimate:
        """EMA blend with heading + lateral outlier gates (mirrors RowDetector)."""
        a = self.ema_alpha
        prev = self._est

        hdg = fresh.heading_error
        lat = fresh.lateral_offset
        if prev.confidence > 0.15:
            delta = hdg - prev.heading_error
            max_delta = math.radians(30.0)
            if abs(delta) > max_delta:
                hdg = prev.heading_error + math.copysign(max_delta, delta)
            d_lat = lat - prev.lateral_offset
            if abs(d_lat) > 0.30:
                lat = prev.lateral_offset + math.copysign(0.30, d_lat)

        self._est = DualRowEstimate(
            lateral_offset=a * lat + (1.0 - a) * prev.lateral_offset,
            heading_error=a * hdg + (1.0 - a) * prev.heading_error,
            confidence=a * fresh.confidence + (1.0 - a) * prev.confidence,
            green_fraction=fresh.green_fraction,
            left_valid=fresh.left_valid,
            right_valid=fresh.right_valid,
            left_offset=fresh.left_offset,
            right_offset=fresh.right_offset,
        )
        return self._est

    # ------------------------------------------------------------------
    def _decay(self) -> DualRowEstimate:
        """No row detected this frame — keep geometry, decay confidence."""
        prev = self._est
        self._est = DualRowEstimate(
            lateral_offset=prev.lateral_offset,
            heading_error=prev.heading_error,
            confidence=prev.confidence * 0.6,
            green_fraction=0.0,
            left_valid=False,
            right_valid=False,
        )
        return self._est
