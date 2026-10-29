#!/usr/bin/env python3
"""
row_perception.py — Online centre crop-row detection from VLP-16 scans.

The Amiga straddles a centre residue strip (soybean field) or a raised crop
bed (onion field): the robot body passes over the centre while the wheels run
in the furrows on either side.  This module detects the centre line ahead of
the robot in real time with NO prior map and reports how the robot sits
relative to it.

Pipeline (per scan, sensor frame X=right, Y=forward, Z=up):
  1. Crop a forward region of interest (ROI) ahead of the robot.
  2. Keep only points inside the crop-height band above ground, using
     ground-relative height  h = z + LIDAR_MOUNT_HEIGHT.  Bare-soil furrow
     returns (h ~ 0) drop out; raised crop rows survive.
  3. Project to a 2-D bird's-eye view (X, Y).
  4. PCA over the whole ROI -> dominant *row direction*.  All crop rows are
     parallel, so the elongation axis is the row heading.
  5. Locate the lateral centre:
       Single-row mode (default): histogram peak nearest the robot centreline.
       Dual-row mode (--dual-row): midpoint between the nearest left and right
         peaks — used in soybean fields where the robot straddles a dark centre
         residue strip flanked by soybean rows on each side; the midpoint is
         the centre of the residue strip, not either soybean row.
  6. Exponential smoothing over time so the controller follows the row
     trend, not every plant-to-plant wobble.

Output: a RowEstimate the controller and state machine consume directly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT


@dataclass
class RowEstimate:
    """Robot pose relative to the centre crop row / residue strip."""
    heading_error: float = 0.0       # rad; +ve = row angled to robot's right
    lateral_offset: float = 0.0      # m;   +ve = centre row is to the right
    confidence: float = 0.0          # 0..1 row-detection quality
    row_end_confidence: float = 0.0  # 0..1; high when crop ahead disappears
    n_points: int = 0                # crop-band points used in the fit
    valid: bool = False              # True when a row was detected this scan


def find_row_midpoint(
    cross: np.ndarray,
    roi_x_half: float,
    bin_width: float,
    row_spacing: float,
    weights: "Optional[np.ndarray]" = None,
    prior_lateral: float = 0.0,
    prior_weight: float = 0.0,
) -> "tuple[float, float]":
    """Locate the residue-strip centre on the cross-row axis.

    Shared by the LiDAR ``RowDetector`` (dual-row mode) and the camera
    ``DualCameraRowTracker`` — both sensors observe the same two flanking
    crop rows, so the centre-finding logic is identical once their
    observations are expressed as cross-row coordinates in the robot frame.

    ``weights`` (optional) assigns a per-point mass to the histogram —
    used by point-level fusion to mix LiDAR points (mass 1 each) with
    camera ground points (down-weighted so the camera cannot outvote the
    LiDAR when they disagree).

    Returns ``(lateral, spacing_factor)`` where ``spacing_factor`` in
    (0, 1] penalises confidence when the detected peak geometry deviates
    from the expected row spacing.

    Pair selection uses a row-spacing prior AND a continuity prior: among all
    left/right peak combinations, the winning pair minimises
    ``|separation − row_spacing| + prior_weight·|midpoint − prior_lateral|``.

    The spacing term alone is AMBIGUOUS in a periodic crop field: a soybean
    field has rows every ``row_spacing`` m, so "on my strip, offset +half" and
    "on the NEXT strip, offset −half" produce two peak pairs that BOTH match the
    spacing.  Minimising spacing error alone then picks one arbitrarily, and when
    the robot corrects past ~half a row the detector can jump the midpoint by a
    whole strip — the controller centres on the adjacent strip and the robot
    hops rows (worst under an aggressive controller).  The continuity term
    (``prior_lateral`` = the strip currently tracked, ``prior_weight`` > 0) keeps
    the detector locked to the strip it is following; a jump to the adjacent
    strip costs ≈ one row spacing and is rejected unless the robot has genuinely
    crossed the half-way point.  ``prior_weight = 0`` restores the old
    spacing-only behaviour (used by the camera tracker).

    When only one side is visible (robot far off-centre, or one row occluded),
    the residue centre is half a row spacing INWARD from the visible row —
    choosing, among candidates, the one nearest ``prior_lateral``.
    """
    peaks = histogram_peaks(cross, roi_x_half, bin_width, weights)

    # Separate into left (x < -0.05 m) and right (x > +0.05 m) peaks.
    # The small dead-band avoids treating noise exactly at centre as a peak.
    left_peaks = [p for p in peaks if p < -0.05]
    right_peaks = [p for p in peaks if p > 0.05]

    half_spacing = 0.5 * row_spacing

    if left_peaks and right_peaks:
        best_pair = None
        best_err = float("inf")           # spacing error of the winning pair
        best_cost = float("inf")
        for pl in left_peaks:
            for pr in right_peaks:
                err = abs((pr - pl) - row_spacing)
                mid = 0.5 * (pl + pr)
                cost = err + prior_weight * abs(mid - prior_lateral)
                if cost < best_cost:
                    best_cost = cost
                    best_err = err
                    best_pair = (pl, pr)
        pl, pr = best_pair
        # Penalise confidence when even the best pair's separation is far
        # from the expected spacing (clutter posing as a row).
        spacing_factor = max(0.5, 1.0 - best_err / row_spacing)
        return 0.5 * (pl + pr), spacing_factor

    if left_peaks:
        # Only left rows visible — strip centre is half a spacing to the right
        # of a left row; pick the candidate nearest the tracked strip.
        cand = min((p + half_spacing for p in left_peaks),
                   key=lambda m: abs(m - prior_lateral))
        return cand, 0.7
    if right_peaks:
        cand = min((p - half_spacing for p in right_peaks),
                   key=lambda m: abs(m - prior_lateral))
        return cand, 0.7

    # All peaks inside the dead-band (clutter on the residue strip itself):
    # treat as "approximately centred" rather than steering at the clutter.
    return min(peaks, key=lambda p: abs(p - prior_lateral)), 0.5


def find_bed_centre(
    cross: np.ndarray,
    roi_x_half: float,
    bin_width: float,
    row_spacing: float,
    bed_rows: int,
    weights: "Optional[np.ndarray]" = None,
    prior_lateral: float = 0.0,
    prior_weight: float = 0.0,
    edge_margin: float = 0.10,
    soil_centre: "Optional[float]" = None,
    soil_weight: float = 0.6,
    miss_cost: float = 0.5,
    extra_cost: float = 1.0,
) -> "tuple[float, float]":
    """Locate the centre of a multi-row raised bed (onion) on the cross-row axis.

    The robot straddles a bed carrying ``bed_rows`` crop rows at a uniform
    ``row_spacing`` (Vidalia onion: 4 rows at 11 in = 0.279 m), so the rows sit
    at ``c + (k − (N−1)/2)·row_spacing`` for k = 0..N−1 around the bed centre c.

    ``find_row_midpoint`` pairs ONE left and ONE right peak and ignores peaks
    inside a ±0.05 m dead-band (correct for a soybean residue strip, which has no
    row on the centreline).  On a 4-row bed that is ambiguous — three adjacent
    pairs all match the spacing — and once the robot is ~0.09 m off-centre an
    inner row enters the dead-band and the WRONG pair wins (sim: +17 cm, then
    +34 cm, i.e. a wheel driven onto the bed).  Instead fit the whole row COMB:
    every (peak, slot) assignment proposes a bed centre; each proposal is scored
    by how well the predicted comb explains the observed peaks:

        cost = 0.5·misses + 1.0·extras + mean_residual/spacing
               + prior_weight·|c − prior| + soil_weight·|c − soil_centre|/spacing

    where *misses* are predicted rows well inside the ROI with no peak near them
    and *extras* are observed peaks no predicted row explains.  A centre shifted
    by one spacing leaves a real row unexplained (an extra) once all the bed's
    rows are in view, so the comb is unambiguous until the NEXT bed's rows enter
    the ROI — capture widens from ~±0.09 m to roughly ±0.35 m.  Extras weigh
    more than misses because a row can be genuinely ABSENT (planter skip, gap)
    but an observed row cannot be imagined; with that asymmetry the tracking
    prior (one spacing ≈ 0.7 at weight 2.5) out-votes a single missing row, so
    a planter skip in an outer row does not hop the comb mid-row.

    ``soil_centre`` (optional) is an independent bed-centre estimate from the
    raised-bed soil edges; it breaks the remaining ambiguity when a whole row is
    missing at start-up (three visible rows fit two comb positions equally).

    The winning centre is refined to sub-bin accuracy from the points near each
    matched row.  Returns ``(lateral, spacing_factor)`` with the same meaning as
    ``find_row_midpoint`` (spacing_factor falls with the fraction of predicted
    rows actually seen).
    """
    n_rows = max(1, int(bed_rows))
    peaks = histogram_peaks(cross, roi_x_half, bin_width, weights)
    offsets = (np.arange(n_rows) - 0.5 * (n_rows - 1)) * row_spacing
    tol = 0.35 * row_spacing
    inner = roi_x_half - edge_margin
    pk = np.asarray(peaks, dtype=float)

    best = None
    for p in peaks:
        for off in offsets:
            c = p - off
            exp = c + offsets
            # residual of each predicted row to its nearest observed peak
            d = np.abs(exp[:, None] - pk[None, :])
            near = d.min(axis=1)
            matched = near <= tol
            visible = np.abs(exp) <= inner
            misses = int(np.sum(visible & ~matched))
            # observed peaks explained by no predicted row
            extras = int(np.sum(d.min(axis=0) > tol))
            resid = float(near[matched].mean()) if matched.any() else tol
            cost = (miss_cost * misses + extra_cost * extras + resid / row_spacing
                    + prior_weight * abs(c - prior_lateral))
            if soil_centre is not None:
                cost += soil_weight * abs(c - soil_centre) / row_spacing
            if best is None or cost < best[0]:
                best = (cost, c, matched, visible)

    _, c, matched, visible = best
    # Sub-bin refinement: average (point − slot offset) over points near each
    # matched predicted row.
    exp = c + offsets
    num = den = 0.0
    w_all = np.ones(len(cross)) if weights is None else weights
    for k in range(n_rows):
        if not matched[k]:
            continue
        sel = np.abs(cross - exp[k]) <= 0.5 * row_spacing * 0.8
        ws = float(w_all[sel].sum())
        if ws > 0:
            num += float((w_all[sel] * (cross[sel] - offsets[k])).sum())
            den += ws
    if den > 0:
        c = num / den
    n_vis = max(1, int(np.sum(visible | matched)))
    spacing_factor = max(0.5, min(1.0, float(np.sum(matched)) / n_vis))
    return float(c), spacing_factor


def bed_relief_centre(
    x: np.ndarray,
    h: np.ndarray,
    pitch: float,
    core_half: float = 0.40,
    furrow_half: float = 0.10,
    x_half: float = 1.5,
    bin_w: float = 0.04,
    c_max: float = 0.50,
    stat: "float | tuple" = 50.0,
    fill_gaps: bool = False,
):
    """Bed centre from the RAISED-BED RELIEF alone (no crop needed).

    Straddling a raised bed, the cross-row height profile is a plateau (bed
    top) with a furrow ``pitch/2`` to each side (where the wheels run).  For
    each candidate centre ``c`` the contrast is

        s(c) = min( core(c) − furrowL(c),  core(c) − furrowR(c) )

    with core = median height over |x−c| ≤ core_half and the furrows sampled
    at c ± pitch/2 (± furrow_half).  Requiring BOTH furrows to be lower makes
    it symmetric and immune to one-sided clutter; the arg-max is the bed
    centre.  ``h`` must already have any forward ramp/level removed.

    Returns ``(centre, contrast_m)`` or None when the profile is too sparse.
    Used when the crop is too small for the LiDAR to see (young onion
    seedlings are a few cm tall and pencil-thin — indistinguishable from soil
    clods), which is exactly when the bed is freshly formed and its relief is
    sharpest.

    Every growth stage (``stat=90, fill_gaps=True``): the profile is the
    ``stat`` percentile of each column (the visible TOP surface — soil, crop
    rows, or the merged canopy), and a column with almost no returns counts as
    LOW.  From mid-season on the canopy walls shadow the wheel furrows so they
    return nothing at all; a gap in the returns IS the furrow, not missing
    data.  The furrows the wheels run in are then the one feature present at
    every stage — bare bed, seedlings, separate rows, one interleaved canopy,
    lodged tops — and the bed centre is midway between them.
    """
    sel = np.abs(x) <= x_half
    if int(sel.sum()) < 150:
        return None
    xs, hs = x[sel], h[sel]
    nb = int(round(2 * x_half / bin_w))
    idx = np.clip(((xs + x_half) / bin_w).astype(int), 0, nb - 1)
    # one lexsort (bin, height) → each bin's median is its middle element
    order = np.lexsort((hs, idx))
    idx_s, h_s = idx[order], hs[order]
    bounds = np.searchsorted(idx_s, np.arange(nb + 1))
    cntb = np.diff(bounds)
    def _one(stat):
        prof = np.full(nb, np.nan)
        okb = cntb >= (3 if fill_gaps else 5)
        lo_i = bounds[:-1][okb]
        c_i = cntb[okb]
        if stat == 50.0:
            prof[okb] = 0.5 * (h_s[lo_i + (c_i - 1) // 2] + h_s[lo_i + c_i // 2])
        else:
            prof[okb] = h_s[lo_i + np.minimum(c_i - 1, (c_i * stat / 100.0).astype(int))]
        cent = -x_half + (np.arange(nb) + 0.5) * bin_w
        if fill_gaps:
            if float(okb.mean()) < 0.4:
                return None                # not a gap pattern — just too few returns
            prof[~okb] = float(np.min(prof[okb]))      # return-less column = furrow / gap
        if float(np.isfinite(prof).mean()) < 0.7:
            return None                    # too sparse a profile to trust its shape

        def band_mean(c0, half):
            m = np.abs(cent - c0) <= half
            v = prof[m]
            ok = np.isfinite(v)
            if int(ok.sum()) < max(2, int(0.5 * m.sum())):
                return None
            return float(v[ok].mean())

        # Vectorised band means over all candidate centres (prefix sums over the
        # profile bins) — this runs every scan on the Jetson.
        fin = np.isfinite(prof)
        csum = np.concatenate([[0.0], np.cumsum(np.where(fin, prof, 0.0))])
        ccnt = np.concatenate([[0], np.cumsum(fin.astype(int))])

        def band_means(c0, half):
            i0 = np.clip(np.ceil((c0 - half + x_half) / bin_w - 0.5).astype(int), 0, nb)
            i1 = np.clip(np.floor((c0 + half + x_half) / bin_w - 0.5).astype(int) + 1, 0, nb)
            span = np.maximum(i1 - i0, 0)
            k = ccnt[i1] - ccnt[i0]
            m = (csum[i1] - csum[i0]) / np.maximum(k, 1)
            good = (span > 0) & (k >= np.maximum(2, 0.5 * span))
            return np.where(good, m, np.nan)

        cands = np.arange(-c_max, c_max + 1e-9, 0.01)
        core_v = band_means(cands, core_half)
        fl_v = band_means(cands - 0.5 * pitch, furrow_half)
        fr_v = band_means(cands + 0.5 * pitch, furrow_half)
        score = np.minimum(core_v - fl_v, core_v - fr_v)
        score = np.where(np.isfinite(score), score, -np.inf)
        k = int(np.argmax(score))
        if not np.isfinite(score[k]):
            return None
        best = float(score[k])
        if best <= 0.0:
            return None                    # no bed above its furrows
        # The contrast is FLAT over the range of c where the core stays on the bed
        # top, so the arg-max only localises coarsely: take the middle of the
        # near-max plateau, then refine on the bed's two SHOULDER edges (where the
        # profile crosses half-way between bed top and furrow) — sharp features.
        near_max = cands[np.isfinite(score) & (score >= best - 0.15 * best)]
        c = float(np.mean(near_max))
        top = band_mean(c, core_half)
        fl = band_mean(c - 0.5 * pitch, furrow_half)
        fr = band_mean(c + 0.5 * pitch, furrow_half)
        if top is None or fl is None or fr is None:
            return c, best

        def edge(sign, floor):
            half = 0.5 * (top + floor)
            prev_x = prev_v = None
            for xc in np.arange(c, c + sign * 0.5 * pitch, sign * bin_w):
                b = int((xc + x_half) / bin_w)
                if not 0 <= b < nb or not np.isfinite(prof[b]):
                    continue
                v = prof[b]
                if v < half:
                    if prev_v is None:
                        return float(cent[b])
                    t = (prev_v - half) / max(prev_v - v, 1e-9)
                    return float(prev_x + t * (cent[b] - prev_x))
                prev_x, prev_v = float(cent[b]), v
            return None

        el, er = edge(-1, fl), edge(+1, fr)
        if el is not None and er is not None and 0.3 < er - el < pitch:
            c = 0.5 * (el + er)
        return c, best

    if isinstance(stat, (tuple, list)):
        return [_one(float(st_)) for st_ in stat]      # several percentiles, one sort
    return _one(float(stat))

def histogram_peaks(
    cross: np.ndarray,
    roi_x_half: float,
    bin_width: float,
    weights: "Optional[np.ndarray]" = None,
) -> "list[float]":
    """Smoothed-histogram peak positions (m) on the cross-row axis.

    Single source of the peak-detection logic used by both the midpoint
    pairing (``find_row_midpoint``) and the peak-cluster-centred PCA.
    """
    lo, hi = -roi_x_half, roi_x_half
    edges = np.arange(lo, hi + bin_width, bin_width)
    hist, edges = np.histogram(cross, bins=edges, weights=weights)
    centres = 0.5 * (edges[:-1] + edges[1:])
    smooth = np.convolve(hist.astype(float), [0.25, 0.5, 0.25], mode="same")
    thresh = max(3.0, 0.25 * float(smooth.max()))
    idx = [
        i for i in range(1, len(smooth) - 1)
        if smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]
        and smooth[i] >= thresh
    ]
    if not idx:
        idx = [int(np.argmax(smooth))]
    return [float(centres[i]) for i in idx]


def _weighted_pca_dir(P: np.ndarray, w: Optional[np.ndarray] = None) -> "tuple[np.ndarray, float]":
    """Principal direction (forward-positive) and linearity of 2D points.

    With ``w=None`` this is byte-identical to the original unweighted PCA;
    with weights it computes the weighted mean/covariance so down-weighted
    camera points influence the fit proportionally to their mass.
    """
    if w is None:
        mean = P.mean(axis=0)
        Q = P - mean
        cov = (Q.T @ Q) / len(Q)
    else:
        W = float(w.sum())
        mean = (w[:, None] * P).sum(axis=0) / W
        Q = P - mean
        cov = (Q.T @ (w[:, None] * Q)) / W
    evals, evecs = np.linalg.eigh(cov)          # ascending
    direction = evecs[:, 1]
    if direction[1] < 0.0:                       # forward-positive
        direction = -direction
    lam_major, lam_minor = float(evals[1]), float(evals[0])
    linearity = (lam_major - lam_minor) / (lam_major + lam_minor + 1e-9)
    return direction, linearity


def _cluster_centred_pca(
    P: np.ndarray,
    w: np.ndarray,
    cross: np.ndarray,
    peak_positions: "list[float]",
    fallback: "tuple[np.ndarray, float]",
    max_peak_dist: float = 0.25,
) -> "tuple[np.ndarray, float]":
    """Within-row PCA: assign every point to its nearest histogram peak,
    centre each peak cluster on its own (weighted) centroid, and fit the
    pooled centred points.

    Removes the between-stripe covariance that tilts a whole-cloud PCA when
    the stripes have unequal extents (different sensor coverage) — and,
    because clustering is per PEAK rather than a two-way split, stripes
    contributed by a mis-calibrated sensor form their own clusters and
    cannot tilt the heading either.

    Points farther than ``max_peak_dist`` from every detected peak are
    excluded from the heading fit: a sub-threshold stripe (e.g. a
    mis-calibrated camera's rows, too light to register as a peak in the
    weighted histogram) would otherwise be absorbed into the nearest REAL
    peak's cluster and tilt its within-row direction.  Such points still
    participate in the midpoint pairing via the histogram — only the
    heading fit ignores them.  Returns ``fallback`` when no usable cluster
    remains.
    """
    peaks = np.asarray(peak_positions, dtype=float)
    if len(peaks) == 0:
        return fallback
    dists = np.abs(cross[:, None] - peaks[None, :])
    assign = np.argmin(dists, axis=1)
    near_peak = dists[np.arange(len(cross)), assign] <= max_peak_dist
    total = float(w.sum())
    parts_p, parts_w = [], []
    for k in range(len(peaks)):
        sel = (assign == k) & near_peak
        if not sel.any():
            continue
        ws = w[sel]
        mass = float(ws.sum())
        if mass < 0.05 * total:
            continue                      # degenerate sliver — skip
        C = P[sel]
        centroid = (ws[:, None] * C).sum(axis=0) / mass
        parts_p.append(C - centroid)
        # Normalise each cluster to unit total weight before pooling.
        # Without this, a cluster with 4× more points (e.g. the left soybean
        # row when the VLP-16 illuminates it at a wider azimuth angle) would
        # contribute 4× as much to the covariance and pull the heading toward
        # its own local direction — in the field this produced a persistent
        # +9° heading bias even from full scans.
        parts_w.append(ws / mass)
    if not parts_p:
        return fallback
    return _weighted_pca_dir(np.vstack(parts_p), np.concatenate(parts_w))


class RowDetector:
    """
    Stateful centre-row / centre-residue detector.  Call update() once per
    LiDAR scan.

    All distances are metres in the sensor frame (X=right, Y=forward).

    dual_row=False (default): single crop row under the robot (onion field).
    dual_row=True: robot straddles a centre residue strip flanked by crop rows
      on each side (soybean field).  Lateral offset = midpoint between the
      nearest left and right crop-band histogram peaks.
    """

    def __init__(
        self,
        roi_y_min: float = 1.5,
        roi_y_max: float = 7.0,
        roi_x_half: float = 0.80,
        crop_h_min: float = 0.03,
        crop_h_max: float = 0.30,
        min_points: int = 40,
        full_points: int = 130,
        refine_window: float = 0.18,
        bin_width: float = 0.05,
        row_end_density: float = 70.0,
        row_end_side_density: float = 45.0,
        row_end_side_window: float = 0.20,
        ema_alpha: float = 0.35,
        temporal_trust: bool = True,
        temporal_ref_conf: float = 0.60,
        temporal_min_gain: float = 0.45,
        dual_row: bool = False,
        row_spacing: float = 0.76,
        midpoint_prior_weight: float = 2.5,
        max_lateral_jump: float = 0.20,
        aux_y_min: float = 0.5,
        aux_mass_ratio: float = 0.5,
        ground_detrend: bool = True,
        ground_min_pts: int = 60,
        ground_max_grade_deg: float = 25.0,
        ground_deadband_deg: float = 2.0,
        ground_level_spread: float = 0.35,
        ground_roll_x_half: float = 2.0,
        heading_cap_deg: float = 7.0,
        heading_gate_deg: float = 22.0,
        heading_abs_max_deg: float = 35.0,
        dense_canopy: bool = True,
        canopy_tall_h: float = 0.45,
        dense_canopy_frac: float = 0.35,
        strip_floor_pct: float = 25.0,
        row_end_veto_density: float = 200.0,
        reliability_floor: float = 0.35,
        bed_rows: int = 0,
        bed_shape: bool = True,
        bed_pitch: float = 1.83,
        bed_shape_min_contrast: float = 0.02,
    ) -> None:
        self.roi_y_min = roi_y_min
        self.roi_y_max = roi_y_max
        self.roi_x_half = roi_x_half
        self.crop_h_min = crop_h_min
        self.crop_h_max = crop_h_max
        self.min_points = min_points
        self.full_points = full_points
        self.refine_window = refine_window
        self.bin_width = bin_width
        self.row_end_density = row_end_density
        # Per-side row-end criterion (dual-row): a row has only truly ENDED when
        # BOTH flanking crop rows are gone.  In a soybean field the two rows
        # rarely end at the same plant — one line runs longer.  Basing row-end
        # on TOTAL density lets the confidence sag as the first row thins,
        # which the navigator can misread as a row end while the longer line is
        # still clearly there.  Instead we measure the mass near EACH flanking
        # peak and key row-end off the STRONGER side: as long as either row
        # keeps ≥ row_end_side_density points near its peak, the row has not
        # ended and the robot keeps following the remaining (longer) line to
        # its true end.  row_end_side_window is the cross-row half-width around
        # each flanking peak the mass is counted in.
        self.row_end_side_density = row_end_side_density
        self.row_end_side_window = row_end_side_window
        self.ema_alpha = ema_alpha
        # Confidence-weighted temporal filter: scale the per-scan EMA gain by how
        # much to trust THIS scan (its confidence relative to a healthy lock), so
        # a noisy low-confidence scan moves the tracked row less and the estimate
        # leans on recent history — one bad scan can't jump the steering
        # (row ends, VLP-16 dropouts, transient occlusion).  Floored at
        # temporal_min_gain so a genuine sustained change still comes through.
        # NOTE: this addresses LOW-confidence jitter; the dense-canopy weave was a
        # HIGH-confidence bias, fixed at the source by the prominence weighting.
        self.temporal_trust = temporal_trust
        self.temporal_ref_conf = temporal_ref_conf
        self.temporal_min_gain = temporal_min_gain
        self.last_gain = ema_alpha            # diagnostic: EMA gain used this scan
        self.dual_row = dual_row
        # ``row_spacing`` is only a SEED for the self-calibrating estimate below
        # — not a fixed geometry the operator must measure.  It is the prior the
        # peak-pairing uses to tell the two flanking crop rows apart from weed
        # clutter, and the inward offset for the single-side fallback.
        self.row_spacing = row_spacing
        # Multi-row raised-bed mode (onion): bed_rows ≥ 2 replaces the two-peak
        # midpoint with a whole-bed comb fit (``find_bed_centre``) — the lateral
        # target is the BED centre and row_spacing is the in-bed row spacing.
        # 0 = off (soybean residue-strip pairing, unchanged).
        self.bed_rows = int(bed_rows)
        # Bed-surface floor for the crop band (bed mode): percentile of ROI
        # height over |x| ≤ bed_floor_x_half, clamped to [0, bed_floor_max].
        self.bed_floor_x_half = 0.50
        self.bed_floor_pct = 10.0
        self.bed_floor_max = 0.35
        self.bed_soil_max = 0.20               # m; highest plausible bed top above the furrow (4–8 in beds)
        self.bed_soil_peak_frac = 0.15         # soil peak ≥ this × the largest height peak
        self.bed_floor_default = 0.10          # m; bed top assumed until soil is first seen
        self._floor_lvl = None                 # tracked bed-top level (persists across rows)
        self._floor_slope = 0.0
        self._floor_high = 0
        self._furrow = None                    # tracked furrow ground line (c0, c1)
        self.last_soil_visible = False         # diagnostic: bed-top soil seen this scan
        self.last_soil_bins = 0
        self.last_bed_floor = 0.0              # diagnostic: floor at ROI middle last scan
        self.last_bed_floor_slope = 0.0        # diagnostic: residual forward slope of the bed top
        # Straddling a bed, the bed centre can only be so far off the robot
        # centreline before a wheel is ON the bed — a fit beyond this is a lock
        # on something else (neighbouring bed, soil edge, clutter): zero its
        # confidence so the state machine never follows it.
        self.bed_max_offset = 0.45
        # Bare-bed relief tracking (bed mode): when the crop is too small for
        # the LiDAR (young onion seedlings), steer by the raised bed itself —
        # the plateau between the two wheel furrows (bed_pitch apart).
        self.bed_shape = bed_shape
        self.bed_pitch = bed_pitch
        self.bed_shape_x_half = 1.5            # m; cross-row swath for the relief profile
        self.bed_shape_min_contrast = bed_shape_min_contrast   # m; bed-above-furrow to trust
        self.bed_shape_ref = 0.05              # m; contrast giving full confidence
        self.bed_shape_jump = 0.10             # m; one-scan profile jump treated as an outlier
        # View quality: returns reaching the 3 m × 5.5 m profile swath (smoothed).
        # Bare → mature canopy: 2000–4000.  When the canopy top nears the
        # LiDAR's own height (0.80 m) almost every downward beam hits leaves
        # inside the 1.5 m self-filter and the view collapses (ray-cast sim,
        # tops ≈ 0.73 m: ~590 returns, 13–18 cm errors) — confidence is scaled
        # down so the robot slows and then stops rather than steer blind.
        self.view_n_min = 300.0
        self.view_n_full = 1200.0
        self._view_n = None
        self.last_view_quality = 1.0
        self.last_sector_gap = False           # diagnostic: azimuth sector missing this scan
        self._canopy_top = None                # crop canopy top above the furrow (m), slow EMA
        self._canopy_n = 0
        self.last_canopy_top = None
        # Growth-stage-proof profile (see bed_relief_centre): the 90th-percentile
        # height of ALL returns per column, return-less columns = furrow.  The
        # old soil-only profile needed the bed-top soil, which the canopy hides
        # from mid-season on (ray-cast sim: 4 % of returns are soil at mid, 0 %
        # mature/lodged), and it depended on the bed-floor estimate — whose
        # soil↔canopy jumps caused the 2026-10-16 relief dropouts.
        self.bed_profile_stat = 90.0
        self.bed_profile_stats = (50.0, 75.0, 90.0)
        self.bed_profile_fill = True
        # Bed-axis filter for the ROI box: the memoryless sharpness search is
        # unbiased but scatters ~1–2° per scan on a lodged canopy, and every 1°
        # of box error costs ~5 cm of lateral at the sensor (lever arm), so the
        # measurement is low-passed (the true axis changes slowly: robot yaw
        # rate ≲ 0.5°/scan).
        self.bed_axis_alpha = 0.3
        self.bed_axis_band_min = 0.30          # crop-band count cue needs this share of the swath
        self.bed_axis_rate_deg = 1.0           # max axis change per scan (robot yaw ≲ 0.5°/scan)
        self._bed_axis = None
        self._shape_far_ok = None              # far half still shows the bed (last scan)
        self._bed_y_cut = None                 # bed visibly ends before this range (m)
        self._far_base = None                  # running relief baselines of the far half /
        self._q2_base = None                   # second quarter (bed-end detection)
        self._q2_tick = 0
        self._qcut_run = 0
        self.bed_end_cut_scans = 6
        self.bed_q2_abs = 0.05                 # m; second-quarter relief that is always "bed"
        self._bed_end_armed = False            # far bed seen since reset (see _bed_shape)
        self._bed_ending = False               # diagnostic + latch: bed seen ending ≲ 3 m ahead
        self._turning = False                  # navigator: headland U-turn in progress
        self._far_back = 0
        self._shape = None
        self.last_bed_contrast = 0.0           # diagnostic: bed relief contrast last scan (m)
        self.bed_far_ratio = 0.7               # far-half relief must be ≥ this × near to steer heading
        # Heading cap while steering by the bed.  Was 8° (a crutch against the
        # old box-follows-heading ratchet at a bed end); the box now comes from
        # the memoryless, smoothed, rate-limited sharpness search, and a U-turn
        # can hand over up to ~11.5° off-axis — at 8° the box stayed 2–4° off and
        # re-entry read 17–22 cm wrong (ray-cast sim).
        self.bed_shape_max_heading = math.radians(15.0)
        # Bed-presence gate (field 2026-10-15: at the row end the bed relief
        # faded and the crop fit then FOLLOWED GRASS for 6 m).  Once the relief
        # has been tracked for ``bed_latch_scans`` scans the field is known to
        # have raised beds, so the bed is REQUIRED: a smoothed relief below
        # ``bed_present_frac`` × its running baseline = the bed has ended →
        # confidence 0 / row end, whatever the crop fit says.  On the field run
        # this gave 0 false triggers over 150 m and confirmed the end at the
        # bed end instead of 6 m into the grass.
        self.bed_present_frac = 0.6
        self.bed_return_frac = 0.8
        self.bed_present_abs = 0.045           # m; this much relief is a bed whatever the baseline
        self.bed_return_scans = 8
        self._present_run = 0
        self.bed_return_abs = 0.06
        self.bed_latch_scans = 30
        self._relief_base = None
        self._relief_cnt = 0
        self._relief_sm = None
        # Relief DROPOUTS (field 2026-10-16, all three controllers): mid-row the
        # shape fit sometimes returns nothing for 2–4 scans (the bed-top floor
        # jumps between soil and onion canopy) — relief ~0.07 m, then exactly 0,
        # then 0.07 again.  Fed into the EMA those zeros tripped the gate 22–108
        # times per row → v=0 stops = the "sudden brakes".  A missing fit is NO
        # EVIDENCE, not evidence of a flat headland: hold the gate (and coast on
        # the last bed geometry) for up to ``bed_dropout_hold`` scans; a real bed
        # end keeps reading nothing/low and still trips it (replayed on the
        # logged contrast: 0 false stops on all runs, bed end found ≤ 0.1 m later).
        self.bed_dropout_hold = 8
        self._shape_miss = 0
        self._last_shape = None
        self.last_bed_present = True           # diagnostic
        self._crop_agree = 0.0                 # EMA of crop-fit ↔ bed agreement
        self.crop_lock_level = 0.8
        self.crop_unlock_level = 0.5
        self._crop_locked = False
        self.crop_row_tol = 0.06               # m; crop point "on a row line"
        self.crop_row_frac = 0.6               # fraction of crop points on the row lines
        # …or, for leafy rows whose leaves fan past ±6 cm (sim: 52–59 % on the
        # lines), at least this fraction AND a clear on-row vs between-row
        # concentration.  Grass at a bed end: 31–38 %, ratio ≈ 1.
        self.crop_row_frac_min = 0.45
        self.crop_comb_ratio = 1.4
        # Once LOCKED a lower concentration keeps the comb: from the 0.99 m mount
        # the steeper view sees more of leafy rows' fanned leaves between the
        # rows (ratio 1.35–1.55 vs 1.55–1.95 at 0.80 m), which flickered the
        # lock off.  Grass never earns the lock at 1.5 (bed end: median ≈ 1.0).
        self.crop_comb_ratio_hold = 1.3
        self.bed_roi_rot_max = math.radians(20.0)   # cap on the heading-aligned ROI rotation
        self.last_roi_rot = 0.0                # diagnostic: ROI box rotation last scan (rad)
        self.bed_dir_x_half = 2.0              # m; swath for the row-direction search
        self.bed_soil_min_raise = 0.06         # m; bed must be this raised to use its soil edges
        self.bed_soil_tol = 0.04               # m; |h − floor| band counted as bed-top soil
        self.bed_soil_edge_pct = 3.0           # percentile of soil cross-coord = bed edge
        self._bed_soil_xy = None
        self.last_soil_centre = None           # diagnostic: soil-edge bed centre last scan
        self.last_crop_fit = None              # diagnostic: (lat, hdg, conf, row_frac) of the crop fit
        self.last_comb_ratio = 0.0             # diagnostic: crop on-row / between-row concentration
        # Self-calibrating spacing: whenever both rows are clearly seen, the
        # measured peak separation refines this estimate (slow EMA, outlier-
        # gated), so the detector converges to the field's ACTUAL row spacing
        # from the seed.  The operator can leave --row-spacing at the default.
        self._spacing_est = row_spacing
        self.auto_spacing = True
        self.max_lateral_jump = max_lateral_jump
        # Heading–lateral consistency clamp.  On a row the detector is tracking,
        # a LARGE heading with a SMALL, stable lateral offset is physically
        # impossible: if the robot were really angled that far it would pile up
        # lateral offset within a scan or two.  So that signature is a PCA
        # artifact — terrain grade tilting the fit, or a sparse VLP-16 scan that
        # dropped azimuth sectors.  A heading-dominant controller that chases it
        # weaves or runs away across the rows (field logs: heading ramped 0 → −57°
        # on a +8° grade with lateral ±0.05 m; and a capped-but-still-large 20°
        # heading drove a sustained left turn on a centred straight row).
        #
        # The cap is PROPORTIONAL to how far off-strip the robot is:
        #   cap = cap_centred + (cap_gate − cap_centred)·min(1, |lat|/lat_gate)
        # Beyond lat_gate the heading is left UNCLAMPED.
        #
        # ENABLED BY DEFAULT.  History: a FLAT cap held heading at a large value
        # near centre and drove a sustained turn (bad); it was briefly disabled
        # entirely, but then a field run with NO guard ran away again on a +6°
        # grade (heading spiked 0 → −38° → +34° with |lateral| ≈ 0, the robot
        # turned ~75° and stuck in ACQUIRE) — the exact failure this clamp
        # prevents.  The PROPORTIONAL cap resolves the earlier concern: in normal
        # following |lateral| is small AND heading is small (±3°), which is well
        # under the near-centre cap, so the clamp is INERT and does NOT shift the
        # RL policy's input; it only bites on the pathological grade spike (large
        # heading while centred), capping it toward ~7° so no controller can be
        # driven into the runaway.  It is a SAFETY NET; the real fix is at the PCA
        # source (why heading spikes on a grade) — needs raw scans from a sloped
        # run.  Set heading_consistency_lat = 0 to disable.
        self.heading_consistency_lat = 0.22          # m; 0 = clamp disabled
        # cap when perfectly centred (tighten on sloped/tall-crop fields where the
        # PCA heading wanders — a lower cap stops pursuit chasing the artifact and
        # walking the robot across rows); exposed as --heading-cap.
        self.heading_cap_centred = math.radians(heading_cap_deg)   # rad; cap when perfectly centred
        self.heading_cap_gate = math.radians(heading_gate_deg)     # rad; cap at the lateral gate
        # Absolute heading ceiling (applies even far off-strip, where the
        # proportional cap relaxes).  A row-following heading beyond this is
        # non-physical (bad fit in a sparse/gappy patch); clamping it stops a
        # reactive controller turning ~90° into the crop.  Exposed as --heading-max.
        self.heading_abs_max = math.radians(heading_abs_max_deg)
        # Continuity ("strip-lock") prior for dual-row pairing: bias the midpoint
        # toward the strip currently tracked so a correction that overshoots does
        # not alias onto the adjacent strip and hop rows.  0 disables it.
        # Raised 1.5 -> 2.5 (and exposed via --strip-lock): a field run had the
        # robot over-correct a lateral error far enough that the detector paired
        # onto the ADJACENT strip and the robot committed to the next row.  The
        # pair cost is |sep − spacing| + weight·|midpoint − tracked_lateral|, so
        # a higher weight makes hopping a whole spacing (cost ≈ weight·spacing)
        # decisively lose to staying on the tracked strip and steering back —
        # the robot must genuinely cross past the half-way line before the
        # detector will accept the new strip.
        self.midpoint_prior_weight = midpoint_prior_weight
        # --- Dense / tall-canopy robustness (growth-stage adaptive) ---------
        # The default row fit keys the two flanking soybean rows off a DENSITY
        # contrast: at early growth the rows carry crop returns while the furrow
        # / residue strip between them is bare GROUND (few in-band points), so
        # the cross-row histogram has two clean peaks with a gap.  As the canopy
        # grows tall and closes over (field: ~0.70–0.75 m), the crop band fills
        # solid — the VLP-16 sees leaves/stems everywhere and almost no ground —
        # so the histogram becomes a plateau, the peak-pairing jitters, and the
        # lateral/heading/spacing estimates weave (field log: sp bouncing
        # 0.60↔0.76, heading ±5–22°, robot hopping strips).
        #
        # But the residue strip the robot straddles is still a canopy-height
        # VALLEY between the two taller crop rows.  So in the dense regime we
        # weight every point by its canopy-height PROMINENCE (height above the
        # local strip floor): the taller row-ridge returns dominate the fit and
        # the strip re-appears as a gap between two weighted-density peaks.  This
        # reuses the ENTIRE existing pipeline (weighted histogram_peaks,
        # find_row_midpoint spacing-prior + strip-lock, cluster-centred PCA) —
        # it only supplies a per-point weight.  Early growth is untouched:
        # dense_frac stays below the gate so the weight is never applied and the
        # fit is byte-identical to before.
        self.dense_canopy = dense_canopy
        self.canopy_tall_h = canopy_tall_h            # m; a point above this (ground-relative) is "tall canopy"
        self.dense_canopy_frac = dense_canopy_frac    # fraction of ROI pts tall → dense regime
        self.strip_floor_pct = strip_floor_pct        # percentile of ROI height = residue-strip canopy floor
        self.last_dense = False                       # diagnostic: dense regime this scan?
        self.last_tall_frac = 0.0                     # diagnostic: fraction of ROI pts above canopy_tall_h
        # --- Self-consistency arbiter (growth-stage mixture-of-experts) -------
        # In the dense regime the detector no longer BLINDLY forces the
        # canopy-height fit.  It computes BOTH perception "experts" — the
        # ground/furrow density fit and the canopy-height-prominence fit — and
        # keeps whichever produces the more SELF-CONSISTENT row geometry this
        # scan (clean PCA linearity × good spacing-prior pairing × agreement with
        # the tracked strip).  This makes the growth-stage handling emergent
        # instead of a brittle single threshold, and is the extensible slot for a
        # LEARNED gate (train the weights on --record telemetry later).  The
        # winning score is exposed as ``last_reliability`` (a calibrated 0–1
        # trust signal the controller can slow on).
        self.arbiter_agree_scale = 0.5                # m; lateral disagreement that halves the agreement term
        self.last_reliability = 0.0                   # diagnostic: winning hypothesis self-consistency
        self.last_mode = "density"                    # diagnostic: which expert won ("density"/"canopy")
        # Reliability-gated HOLD (dense-canopy runaway / weave guard).  When the
        # canopy has closed over the ground, the density expert's cross-row
        # histogram is a plateau and its PCA heading is ill-conditioned; the
        # arbiter is meant to prefer the canopy expert, but in field logs BOTH
        # experts' self-consistency (`last_reliability`) collapsed to ~0.1 on the
        # degenerate scans while the fit's heading spiralled monotonically
        # 8°→60° over ~1.5 m and the lateral drifted 0.1→0.43 m — with crop
        # clearly still present (n≈800).  Chasing that garbage fit produced the
        # left/right dense-canopy weave AND (via the mislocated flanking-row
        # windows) a spurious row-end that fired a HEADLAND turn mid-row.  Good
        # tracking keeps reliability ≥ 0.7 (p10 0.73 on a clean 142 m run), so a
        # floor cleanly separates the two: when reliability < reliability_floor in
        # the dense regime, the fit is untrustworthy — HOLD the previous smoothed
        # geometry (don't chase it) while still reporting the low confidence so the
        # state machine knows tracking degraded.  Non-dense / high-reliability
        # scans are unaffected (byte-identical).  0 disables the hold.
        self.reliability_floor = reliability_floor
        # Row-end density veto.  A row END means the crop band ahead is EMPTY.
        # If the ROI still holds a near-full crop mass the "row ended" reading is
        # a mislocated-window artifact of a degenerate dense-canopy fit, never a
        # real end.  Field logs separate cleanly: FALSE mid-row row-ends all had
        # n ≥ 647 (with runaway heading); REAL row ends all had n ≤ 78.  So when
        # the total LiDAR crop count n ≥ row_end_veto_density the row-end
        # confidence is forced to 0 — a full ROI can never be a row end.  Set 0 to
        # disable.  (The reliability hold above usually prevents the runaway in the
        # first place; this is the independent safety net on the turn decision.)
        self.row_end_veto_density = row_end_veto_density
        # Point-level fusion of camera ground points (aux_xy in update()):
        # aux_y_min lets near-field camera points (LiDAR self-filter blind
        # zone) into the fit; aux_mass_ratio caps the camera's total
        # histogram mass at this fraction of full_points so the camera can
        # never outvote a healthy LiDAR scan.
        self.aux_y_min = aux_y_min
        self.aux_mass_ratio = aux_mass_ratio
        # Terrain-adaptive crop band: a fixed mount-tilt correction is
        # calibrated for FLAT ground, so on a grade the ground (and the crop
        # riding on it) ramps out of the absolute [crop_h_min, crop_h_max]
        # height band and the crop vanishes (field-observed: n collapsed
        # 450->46, FOLLOW dropped to ACQUIRE the moment the field pitched).
        # ground_detrend estimates the residual forward grade per scan and
        # removes its SLOPE so the band tracks the local ground.
        self.ground_detrend = ground_detrend
        self.ground_min_pts = ground_min_pts
        self.ground_level_spread = ground_level_spread
        # Lateral (roll) ground detrend: the forward detrend above removes dz/dy,
        # but a CROSS-SLOPE (or lateral undulation ahead) tilts the ground about
        # the forward axis — dz/dx — which the forward-only detrend cannot see.
        # Uncorrected, one flanking soybean row rides higher than the other, the
        # absolute crop band clips the high (or low) side asymmetrically, and the
        # dual-row midpoint/heading is biased toward the better-seen row — the
        # downhill drift seen in the sloped field runs.  The roll slope is fit
        # from a WIDER cross-row swath (|x| <= ground_roll_x_half, past the
        # detection ROI) so the estimate has a long lever arm and is stable even
        # with the crop rows sitting inside the ROI.  Pure-LiDAR (no IMU); b ≈ 0
        # on flat/level ground, so the flat behaviour is byte-identical.
        self.ground_roll_x_half = ground_roll_x_half
        self._gmax = math.tan(math.radians(ground_max_grade_deg))
        self._gdead = math.tan(math.radians(ground_deadband_deg))
        self.last_ground_slope = 0.0   # dz/dy actually applied last scan (diag)
        self.last_ground_roll = 0.0    # dz/dx actually applied last scan (diag)
        self.last_ground_shift = 0.0   # band level shift applied last scan (diag)
        self._spacing_factor = 1.0   # confidence penalty when peak pair spacing is off
        self._est = RowEstimate()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget the smoothed estimate (call when starting a new row).

        During a headland turn the ROI sweeps across the headland and the
        EMA accumulates garbage geometry; without a reset the lateral and
        heading outlier gates then fight the first genuine detections of
        the next row.
        """
        self._est = RowEstimate()
        self._spacing_factor = 1.0
        self._last_shape = None            # never coast across a turn
        self._shape_miss = 0
        self._bed_axis = None
        self._shape_far_ok = None
        self._bed_y_cut = None
        self._qcut_run = 0
        self._bed_ending = False
        self._far_back = 0
        self._crop_locked = False
        self._crop_agree = 0.0

    def begin_turn(self) -> None:
        """Navigator hook: a headland U-turn starts.  Release the bed-END latch
        (it would hold confidence ×0.3 through the arc and block the
        perception-closed turn ending) and let the bed axis follow the raw
        memoryless direction search — the robot rotates faster (~1.7°/scan)
        than the axis filter's straight-driving rate limit."""
        self._bed_ending = False
        self._qcut_run = 0
        self._bed_y_cut = None
        self._turning = True

    def end_turn(self) -> None:
        """Navigator hook: the U-turn is over (any state after HEADLAND)."""
        self._turning = False
        self._bed_axis = None

    # ------------------------------------------------------------------
    def update(self, pts: np.ndarray, aux_xy: Optional[np.ndarray] = None) -> RowEstimate:
        """
        Detect the centre row/residue from an Nx3 (x, y, z) sensor-frame
        point array.

        ``aux_xy`` (optional, point-level fusion): Mx2 (x, y) metric ground
        points in the same robot frame — e.g. the camera tracker's
        ground-projected green pixels.  When provided, both sensors'
        evidence is pooled into ONE weighted row fit instead of fusing two
        independently fitted estimates.  Camera points are down-weighted so
        their total histogram mass never exceeds ``aux_mass_ratio ×
        full_points`` — a healthy LiDAR scan always outvotes the camera,
        while an empty LiDAR scan can still be carried by camera evidence
        at proportionally reduced confidence.

        Returns a smoothed RowEstimate.  When no row is found the previous
        estimate is decayed (confidence reduced, row-end confidence raised)
        so the state machine can react instead of acting on a stale fix.
        """
        # Bed mode: align the ROI box with the TRACKED bed heading.  The fixed
        # robot-frame box cuts narrow-spaced rows diagonally when the robot is
        # angled to the bed (at 5° and 0.28 m spacing the rows drift over two
        # row-spacings across the 7 m ROI depth, so the far ROI pulls in the
        # neighbouring bed and biases lateral/heading — sim: −8 cm at 5°,
        # −15 cm at 8°).  Rotating the points by the tracked heading keeps the
        # box along the rows; the heading is added back below.  Lateral (the
        # perpendicular sensor-to-bed-centre distance) is rotation-invariant.
        # The box angle comes from a memoryless row-direction search
        # (``_bed_box_heading``) so a biased first fit at a large start angle
        # cannot lock in; the tracked heading is only the fallback.
        roi_rot = 0.0
        if self.bed_rows >= 2:
            lim = self.bed_roi_rot_max
            have = pts is not None and len(pts)
            # Bed direction, memoryless: the cross-row sharpness search works at
            # every growth stage (bare bed tops, rows, merged or lodged canopy —
            # ≤ 0.7° rms in the ray-cast sim); the soil-relief search is the
            # fallback (3–5° rms once crop is up).  The tracked heading is only
            # the last resort.
            # Bed ending ahead (far half lost its relief last scan): search the
            # direction on the near half only — the grass/headland beyond made
            # the full-depth search wander and the axis creep to ~10° (sim).
            y_cut = self._bed_y_cut
            # Bed ends within the nearest quarter (≲ 3 m): the bed direction
            # does not change in its last metres — hold the axis (no search at
            # all; the grass beyond dragged it to ~10°).
            yq_ = 0.25 * (3 * self.roi_y_min + self.roi_y_max)
            hold = (y_cut is not None and y_cut <= yq_ + 1e-6
                    and self._bed_axis is not None and self._bed_end_armed)
            ang = self._bed_box_heading(pts, y_cut) if (have and not hold) else None
            if ang is None and have and not hold:
                ang = self._bed_relief_heading(pts)
            if ang is not None:
                if self._bed_axis is None or self._turning:
                    self._bed_axis = ang
                else:
                    # Low-pass, and the axis can't swing more than ~2° between
                    # scans (robot turn rate; near a bed end most of the view is
                    # headland and single-scan searches scatter).  Limited
                    # against the FILTER's own state, never the output heading:
                    # tied to the output, a first misaligned box whose bed
                    # centre was refused (offset guard → heading held) held the
                    # box too — a deadlock at zero confidence (sim: re-entry
                    # at 0.30 m / 10°).
                    step = math.radians(self.bed_axis_rate_deg)
                    d_ax = self.bed_axis_alpha * (ang - self._bed_axis)
                    self._bed_axis += max(-step, min(step, d_ax))
                ang = self._bed_axis
            elif self._bed_axis is not None:
                ang = self._bed_axis
            elif self._est.valid:
                ang = float(self._est.heading_error)
            if ang is not None:
                if self.last_mode in ("shape", "no-bed", "none"):
                    ang = max(-self.bed_shape_max_heading, min(self.bed_shape_max_heading, ang))
                roi_rot = max(-lim, min(lim, ang))
        self.last_roi_rot = roi_rot
        cr, sr = math.cos(roi_rot), math.sin(roi_rot)

        # --- LiDAR crop-band points ---
        if pts is not None and len(pts) > 0:
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
            if roi_rot:
                x, y = x * cr - y * sr, x * sr + y * cr
            h = z + LIDAR_MOUNT_HEIGHT
            in_xy = (
                (y >= self.roi_y_min) & (y <= self.roi_y_max)
                & (np.abs(x) <= self.roi_x_half)
            )
            # Remove the residual forward ground grade so the crop-height band
            # follows undulating / sloped terrain (see __init__ note).  Only
            # the SLOPE is removed (pivot at the sensor, y=0), which keeps the
            # flat-ground behaviour byte-identical and is robust to the
            # furrow/soil height split (both ramp together → shared slope).
            a = self._ground_slope(y[in_xy], z[in_xy]) if self.ground_detrend else 0.0
            # Lateral (roll) slope dz/dx from a wider cross-row swath, fit on the
            # FORWARD-detrended height so the two axes are decoupled (see
            # __init__ note).  b == 0 on flat ground → no-op.
            b = 0.0
            if self.ground_detrend:
                wide = ((y >= self.roi_y_min) & (y <= self.roi_y_max)
                        & (np.abs(x) <= self.ground_roll_x_half))
                if int(wide.sum()) >= self.ground_min_pts:
                    b = self._ground_roll(x[wide], z[wide] - a * y[wide])
            self.last_ground_roll = b
            h_eff = (h - a * y - b * x) if (a or b) else h
            # Level correction: on undulating terrain the local ground can sit
            # well below the flat-calibrated h=0 (field-observed: ROI ground at
            # ~-0.35 m, the whole canopy pushed below the 0.03 m crop floor and
            # clipped — only 14 pts survived).  When a real ground+canopy column
            # is present (wide height spread) and its median sits below 0, lower
            # the band onto the local ground so the canopy — correctly elevated
            # ABOVE that ground — stays inside the band.  The wide-spread gate
            # keeps canopy-only clouds (narrow span) on the absolute band, and
            # the min(0,·) means it only ever lowers the band, never raises it.
            shift = 0.0
            if self.ground_detrend:
                hr = h_eff[in_xy]
                if len(hr) >= self.ground_min_pts:
                    spread = float(np.percentile(hr, 95) - np.percentile(hr, 5))
                    if spread > self.ground_level_spread:
                        shift = min(0.0, float(np.percentile(hr, 50)))
            # Raised-bed floor (bed mode only): the bed-top soil sits ~0.10–0.25 m
            # above the furrow the wheels (and h=0) ride in, i.e. INSIDE the crop
            # band — a bare bed then reads as a perfect, confident "row" and a
            # row end is never detected.  Measure the crop band from the local
            # bed surface instead: the low percentile of height over the central
            # swath (bed top dominates it).
            #
            # The floor is a LINE in the forward direction, floor(y) = c0 + c1·y,
            # fitted to the per-range-bin low percentile with NO slope dead-band.
            # A scalar floor (or the 2° dead-banded forward detrend) leaves a
            # residual 1.5–2° ramp in place, and against the tight bed-relative
            # band that ramp lifts the far bed-top soil into the "crop" band: on a
            # BARE bed the far soil read as a confident row (field: n≈400,
            # off=+0.8 m, alternating with n=0 as the slope crossed the 2°
            # dead-band every other scan).
            # Bed profile (bed mode): bed centre between the two wheel furrows
            # from the visible top surface — bare bed, rows or one merged
            # canopy alike (see _bed_shape / bed_relief_centre).  Independent of
            # the bed-floor estimate, so it is computed first and centres the
            # floor's soil swath.
            self._shape = None
            if self.bed_rows >= 2 and self.bed_shape and self.bed_profile_fill:
                self._shape = self._bed_shape(x, y, h_eff)
            bed_floor_y = 0.0                         # scalar or per-point array
            bed_floor = 0.0                           # floor at the ROI middle (diag)
            if self.bed_rows >= 2:
                # Swath on the TRACKED bed centre, so an off-centre robot never
                # samples its wheel furrow as "lowest surface".
                if self._shape is not None:
                    xc = float(self._shape[0])
                else:
                    xc = float(self._est.lateral_offset) if self._est.valid else 0.0
                cen = in_xy & (np.abs(x - xc) <= self.bed_floor_x_half)
                ymid_ = 0.5 * (self.roi_y_min + self.roi_y_max)
                # Ground model = FURROW line (tracked) + bed relief above it
                # (tracked).  Measuring the bed top relative to the furrow in
                # the same range bin makes the soil-vs-canopy test immune to a
                # residual forward ramp, and the furrow keeps tracking that ramp
                # after the canopy has hidden the bed top.
                wide = ((y >= self.roi_y_min) & (y <= self.roi_y_max)
                        & (np.abs(x - xc) <= self.bed_shape_x_half))
                # Fit on heights WITHOUT the dead-banded forward detrend (it
                # switches on/off as the residual ramp crosses 2°, which made
                # the far bed top flicker in/out of the crop band — field
                # 2026-10-15); the furrow line owns the ramp in bed mode.
                hb = h_eff + a * y if a else h_eff
                # Under a canopy (bed top hidden) the furrow readings are canopy
                # wall bottoms — freeze the line; update it only while soil shows.
                fr = None
                # (including the first fit: starting in a closed canopy the line
                # stays on the calibrated flat ground).
                # Once soil has been measured and is now hidden, never START a
                # line here either (it would be fitted to canopy-wall bottoms).
                if int(wide.sum()) >= 200 and (
                        self.last_soil_visible
                        or (self._furrow is None and self._floor_lvl is None)):
                    fr = self._furrow_fit(y[wide], hb[wide] - shift)
                if fr is not None:
                    f_lvl = fr[0] + fr[1] * ymid_
                    if self._furrow is not None:
                        p_lvl = self._furrow[0] + self._furrow[1] * ymid_
                        f_lvl = p_lvl + 0.4 * max(-0.03, min(0.03, f_lvl - p_lvl))
                        f_sl = self._furrow[1] + 0.4 * (fr[1] - self._furrow[1])
                    else:
                        f_sl = fr[1]
                    self._furrow = (f_lvl - f_sl * ymid_, f_sl)
                f0, f1 = self._furrow if self._furrow is not None else (0.0, 0.0)
                fl = None
                if int(cen.sum()) >= self.ground_min_pts:
                    fl = self._bed_floor_line(y[cen], hb[cen] - shift - (f0 + f1 * y[cen]))
                if fl is not None:
                    lvl0 = fl[0] + fl[1] * ymid_
                    ref = self._floor_lvl if self._floor_lvl is not None else self.bed_floor_default
                    # Soil can't rise 5 cm between scans: a much HIGHER "lowest
                    # surface" is low canopy (lodged tops), unless it persists.
                    if self._floor_lvl is not None and lvl0 > ref + 0.05:
                        self._floor_high += 1
                        if self._floor_high < 15:
                            fl = None
                    else:
                        self._floor_high = 0
                if fl is not None:
                    lvl = fl[0] + fl[1] * ymid_
                    if self._floor_lvl is not None:
                        # Beds don't change height scan to scan: low-pass the
                        # level and bound its step (a stray canopy read can't jump it).
                        d = max(-0.03, min(0.03, lvl - self._floor_lvl))
                        lvl = self._floor_lvl + 0.4 * d
                    self._floor_lvl = lvl
                    self.last_soil_visible = True
                else:
                    # Soil hidden (mid-season canopy and later): keep the last
                    # measured bed relief, or a nominal bed if never seen.
                    self.last_soil_visible = False
                    if self._floor_lvl is None:
                        self._floor_lvl = self.bed_floor_default
                # back into the h_eff frame the crop band is cut in
                bed_floor_y = f0 + (f1 - a) * y + self._floor_lvl
                bed_floor = f0 + f1 * ymid_ + self._floor_lvl
                self.last_bed_floor_slope = f1
            self.last_bed_floor = float(bed_floor)
            if self.bed_rows >= 2:
                # Crop canopy top over the bed BEYOND the safety zone (y ≥ 2.6 m,
                # so an obstacle inside the zone cannot raise its own threshold),
                # slow EMA — for the canopy-adaptive obstacle height.
                xc_ = float(self._shape[0]) if self._shape is not None else (
                    float(self._est.lateral_offset) if self._est.valid else 0.0)
                top_m = ((y >= 2.6) & (y <= self.roi_y_max)
                         & (np.abs(x - xc_) <= 0.6))
                if int(top_m.sum()) >= 20 and self.last_bed_present:
                    t = float(np.percentile(h_eff[top_m] - shift, 95))
                    self._canopy_top = t if self._canopy_top is None else (
                        self._canopy_top + 0.05 * (t - self._canopy_top))
                    self._canopy_n += 1
                self.last_canopy_top = self._canopy_top if self._canopy_n >= 20 else None
            # Bed-top soil returns (only when the bed is genuinely raised): the
            # soil plateau's two edges give an independent bed-centre estimate.
            self._bed_soil_xy = None
            if self.bed_rows >= 2 and bed_floor >= self.bed_soil_min_raise:
                soil = in_xy & (np.abs(h_eff - shift - bed_floor_y) <= self.bed_soil_tol)
                if int(soil.sum()) >= self.ground_min_pts:
                    self._bed_soil_xy = np.column_stack((x[soil], y[soil]))
            self.last_ground_slope = a
            self.last_ground_shift = shift
            h_rel = h_eff - bed_floor_y
            roi = (in_xy & (h_rel >= self.crop_h_min + shift)
                   & (h_rel <= self.crop_h_max + shift))
            cx, cy = x[roi], y[roi]
            # ground-relative height of each ROI point (bed-top-relative in bed
            # mode; bed_floor_y = 0 otherwise → unchanged)
            ch = h_rel[roi]
            n = int(cx.shape[0])
            if self.bed_rows >= 2 and self.bed_shape and not self.bed_profile_fill:
                soil_m = h_rel < self.crop_h_min + shift          # legacy soil-only profile
                self._shape = self._bed_shape(x[soil_m], y[soil_m], h[soil_m])
        else:
            cx = cy = ch = np.empty(0)
            n = 0
            self._shape = None

        # --- Dense / tall-canopy regime: height-prominence point weights ------
        # When the canopy has closed over (a large fraction of the ROID band is
        # tall), weight each point by how far it rises above the residue-strip
        # floor so the two taller crop rows dominate the fit and the strip shows
        # as a gap between them.  Off in early growth (weight vector = None →
        # original unweighted fit).  See __init__ note.
        dense = False
        h_weight = None
        if self.dense_canopy and n >= self.min_points:
            self.last_tall_frac = float(np.mean(ch > self.canopy_tall_h))
            if self.last_tall_frac >= self.dense_canopy_frac:
                floor = float(np.percentile(ch, self.strip_floor_pct))
                hw = np.clip(ch - floor, 0.0, None)
                if float(hw.max()) > 1e-6:
                    dense = True
                    h_weight = hw
        else:
            self.last_tall_frac = 0.0
        self.last_dense = dense

        # Row-end confidence stays LiDAR-only: the camera's contribution to
        # row-end detection is the separate green-fraction veto, and camera
        # mass must not be able to mask a genuinely sparse crop band ahead.
        row_end_conf = 1.0 - min(1.0, n / self.row_end_density)

        # --- auxiliary camera ground points (same robot frame) ---
        A = None
        if aux_xy is not None and len(aux_xy):
            if roi_rot:
                aux_xy = np.column_stack((aux_xy[:, 0] * cr - aux_xy[:, 1] * sr,
                                          aux_xy[:, 0] * sr + aux_xy[:, 1] * cr))
            ax, ay = aux_xy[:, 0], aux_xy[:, 1]
            keep = (
                (ay >= self.aux_y_min) & (ay <= self.roi_y_max)
                & (np.abs(ax) <= self.roi_x_half)
            )
            A = aux_xy[keep]
            if len(A) == 0:
                A = None

        cam_mass = 0.0
        if A is not None:
            cam_mass = min(float(len(A)), self.aux_mass_ratio * self.full_points)

        total_mass = n + cam_mass
        self.last_bed_contrast = 0.0 if self._shape is None else float(self._shape[2])
        if self.bed_rows >= 2 and self.bed_shape and not self._bed_present():
            # Raised beds established, but the bed has ended (flat headland /
            # grass): never follow a crop fit here — it is the grass.
            self.last_mode = "no-bed"
            return self._decay(row_end_conf=1.0)
        if total_mass < self.min_points:
            if self._shape is not None:
                # No visible crop, but the raised bed itself is clear: steer
                # by the bed relief (young seedlings / bare stretch of bed).
                self._crop_agree *= 0.9          # no crop seen → crop lock fades
                return self._shape_estimate(roi_rot, total_mass)
            if (self.bed_rows >= 2 and self.bed_shape and self._last_shape is not None
                    and 0 < self._shape_miss <= self.bed_dropout_hold):
                # Brief relief dropout on a bed we were tracking: coast on the
                # last bed geometry instead of decaying (decay = speed dip/stop).
                self._shape = self._last_shape
                est = self._shape_estimate(roi_rot, total_mass)
                self._shape = None
                return est
            if self.bed_rows >= 2:
                self.last_mode = "none"
            return self._decay(row_end_conf=row_end_conf)

        # --- pooled point set + per-expert weight vectors ---
        # DENSITY expert: uniform LiDAR weight (the original ground/furrow fit),
        # camera points down-weighted.  CANOPY expert (dense regime only):
        # canopy-height prominence, emphasising the taller row ridges so the
        # residue strip re-appears as a gap.
        P = np.column_stack((cx, cy))
        if A is not None:
            cam_w = np.full(len(A), cam_mass / len(A))
            w_density = np.concatenate([np.ones(n), cam_w])
            w_canopy = np.concatenate([h_weight, cam_w]) if dense else None
            P = np.vstack([P, A]) if n else np.asarray(A, dtype=float)
        else:
            w_density = None            # pure-LiDAR early-growth path stays unweighted
            w_canopy = h_weight if dense else None

        if self.dual_row:
            # --- Self-consistency arbiter over the perception experts ---------
            # Fit BOTH representations and keep whichever gives the cleaner,
            # more consistent row geometry this scan (linearity × spacing
            # pairing × agreement with the tracked strip).  In early growth only
            # the density expert exists → byte-identical to before.  In dense
            # canopy the arbiter normally picks the canopy expert, but falls back
            # to density if the height fit is momentarily worse — no brittle
            # single threshold decides it.  The two-pass cluster-centred PCA that
            # each fit runs removes the between-stripe covariance that would
            # otherwise bias the heading (VLP-16 azimuth dropouts / unequal
            # stripe extents / mixed-sensor y-coverage).
            experts = [("density", w_density)]
            if w_canopy is not None:
                experts.append(("canopy", w_canopy))
            best = None
            for name, wc in experts:
                lat_c, dir_c, lin_c, sf_c, cross_c, sp_c = self._dual_fit(P, wc, n)
                linf = max(0.0, min(1.0, (lin_c - 0.20) / 0.55))
                agree = 1.0 - min(1.0, abs(lat_c - float(self._est.lateral_offset))
                                  / self.arbiter_agree_scale)
                score = linf * sf_c * (0.5 + 0.5 * agree)   # self-consistency reliability
                if best is None or score > best[0]:
                    best = (score, name, lat_c, dir_c, lin_c, sf_c, cross_c, sp_c, wc)
            (self.last_reliability, self.last_mode, lateral, direction, linearity,
             self._spacing_factor, cross, self._spacing_est, w) = best
            # last_dense already reflects the REGIME (canopy expert spawned);
            # last_mode reports which expert the arbiter actually kept.
            # Row-end on the LONGER line: count crop mass near each flanking row
            # (at lateral ± half-spacing) and key row-end off the STRONGER side,
            # so a row is only "ended" once BOTH flanking lines are gone — the
            # robot keeps following whichever soybean line runs longer to its
            # true end (see __init__ note).  LiDAR-ONLY (cross[:n]): like the
            # total-density criterion above, the camera must not be able to mask
            # a genuinely sparse LiDAR crop band and hide a real row end.
            hs = 0.5 * self._spacing_est
            cross_lidar = cross[:n]
            if len(cross_lidar) and self.bed_rows >= 2:
                # Bed mode: strongest of the N predicted bed rows (a row end
                # only when EVERY onion row in the bed has ended).  Window is
                # capped below half the spacing so adjacent rows don't share it.
                win = min(self.row_end_side_window, 0.45 * self._spacing_est)
                offs = (np.arange(self.bed_rows) - 0.5 * (self.bed_rows - 1)) * self._spacing_est
                row_strength = float(max(
                    int(np.sum(np.abs(cross_lidar - (lateral + o)) <= win)) for o in offs))
            elif len(cross_lidar):
                lsel = np.abs(cross_lidar - (lateral - hs)) <= self.row_end_side_window
                rsel = np.abs(cross_lidar - (lateral + hs)) <= self.row_end_side_window
                row_strength = float(max(int(lsel.sum()), int(rsel.sum())))
            else:
                row_strength = 0.0
            row_end_conf = 1.0 - min(1.0, row_strength / self.row_end_side_density)
            # Veto: a full ROI can never be a row end.  When the runaway heading
            # mislocates the flanking windows, row_strength reads low and this
            # would spuriously fire — but n (total crop) is still high, so clamp
            # row-end confidence to 0.  Real row ends have n≪veto (crop gone).
            if self.row_end_veto_density and n >= self.row_end_veto_density:
                row_end_conf = 0.0
        else:
            # Single-row mode (onion): one crop row under the robot; no arbiter.
            # Seed the row axis by PCA (canopy-height-weighted in the dense
            # regime, else uniform) then take the histogram peak nearest the
            # centreline, refined with a tight window so adjacent rows do not
            # merge.
            w = w_canopy if (dense and w_density is None) else w_density
            if w_density is not None and n >= self.min_points:
                direction, linearity = _weighted_pca_dir(P[:n])
            else:
                direction, linearity = _weighted_pca_dir(P, w)
            cross = P @ np.array([direction[1], -direction[0]])
            peak = self._nearest_peak(cross, weights=w)
            near = np.abs(cross - peak) <= self.refine_window
            if int(near.sum()) >= 8:
                lateral = float(np.average(cross[near],
                                           weights=None if w is None else w[near]))
            else:
                lateral = peak

        heading = math.atan2(direction[0], direction[1]) + roi_rot
        density = min(1.0, total_mass / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
        confidence = density * linear_factor
        if self.dual_row:
            confidence *= self._spacing_factor
        # Bed mode: a bed centre further off than bed_max_offset is physically
        # impossible while straddling the bed (a wheel would be on it) — it is a
        # lock on something else.  Zero its confidence and don't let it drag the
        # tracked geometry.
        if self.bed_rows >= 2 and abs(lateral) > self.bed_max_offset:
            confidence = 0.0
            if self._est.valid:
                lateral = float(self._est.lateral_offset)
                heading = float(self._est.heading_error)
        # Bed mode: the crop fit must agree with a confident bed relief.  A few
        # hundred soil clods / weeds can pass the crop band and fit a "row";
        # when that contradicts the bed geometry (or is weaker than it), the
        # bed relief wins.
        comb_ok = True
        self.last_comb_ratio = 0.0
        if self.bed_rows >= 2:
            # Independent evidence that the crop points really ARE onion rows:
            # most of them must sit on the predicted row lines.  (The comb fit
            # itself is pulled toward the tracked bed, so position agreement
            # alone is not evidence; uniform grass puts only ~30 % there.)
            row_frac = 0.0
            cl = cross[:n] if (self.dual_row and n) else np.empty(0)
            if len(cl):
                offs = (np.arange(self.bed_rows) - 0.5 * (self.bed_rows - 1)) * self._spacing_est
                dmin = np.min(np.abs(cl[:, None] - (lateral + offs)[None, :]), axis=1)
                row_frac = float(np.mean(dmin <= self.crop_row_tol))
                # Same count on the lines MIDWAY between the rows (and half a
                # spacing outside the outer rows): rows — even leafy ones whose
                # leaves fan past the ±6 cm window — pile up on the row lines;
                # uniform grass / clods do not.
                mids = np.concatenate([offs[:-1] + 0.5 * self._spacing_est,
                                       [offs[0] - 0.5 * self._spacing_est,
                                        offs[-1] + 0.5 * self._spacing_est]])
                dmid = np.min(np.abs(cl[:, None] - (lateral + mids)[None, :]), axis=1)
                off_frac = float(np.mean(dmid <= self.crop_row_tol))
                self.last_comb_ratio = row_frac / max(off_frac, 0.02)
            self.last_crop_fit = (lateral, heading, confidence, row_frac)
            comb_ok = confidence >= 0.5 and (
                row_frac >= self.crop_row_frac
                or (row_frac >= self.crop_row_frac_min
                    and self.last_comb_ratio >= (self.crop_comb_ratio_hold
                                                 if self._crop_locked
                                                 else self.crop_comb_ratio)))
        if self.bed_rows >= 2 and self._shape is not None:
            agrees = (comb_ok and abs(lateral - self._shape[0]) <= 0.12
                      and abs(heading - (self._shape[1] + roi_rot)) <= math.radians(4.0))
            # Crop LOCK: the crop fit only steers once it has agreed with the
            # bed consistently (~2 s).  Visible onion rows earn it quickly; a
            # sudden mass of grass at the bed end (field: n 0→800) never does.
            self._crop_agree = 0.9 * self._crop_agree + 0.1 * (1.0 if agrees else 0.0)
            # Hysteresis: lock at crop_lock_level, release below crop_unlock_level.
            if self._crop_agree >= self.crop_lock_level:
                self._crop_locked = True
            elif self._crop_agree < self.crop_unlock_level:
                self._crop_locked = False
            # While locked, one marginal scan (leaves fanning past the row
            # window) keeps the crop fit if it still sits on the tracked bed —
            # dropping to the coarser profile for a scan or two only adds noise.
            steady = (self._crop_locked and self._est.valid and confidence >= 0.5
                      and row_frac >= self.crop_row_frac_min
                      and abs(lateral - float(self._est.lateral_offset)) <= 0.06)
            if not ((agrees and self._crop_locked) or steady):
                return self._shape_estimate(roi_rot, total_mass)
        elif self.bed_rows >= 2 and self.bed_shape and not comb_ok:
            # No bed profile this scan AND the crop is not a believable 4-row
            # comb.  Ray-cast sim: with tops within ~7 cm of the LiDAR the view
            # collapses and a free crop fit read −17…−24° / 34 cm off at
            # confidence ~0.85 — never steer on that: coast briefly on the last
            # bed geometry, then let confidence fall (the robot stops).
            self._crop_agree *= 0.9
            if self._last_shape is not None and 0 < self._shape_miss <= self.bed_dropout_hold:
                self._shape = self._last_shape
                est = self._shape_estimate(roi_rot, total_mass)
                self._shape = None
                return est
            self.last_mode = "none"
            return self._decay(row_end_conf=row_end_conf)
        # Bed mode, crop fit accepted: same straddle heading cap as the bed
        # relief (grass/clutter at a bed end must not swing the robot).
        if self.bed_rows >= 2 and self.bed_shape and self._relief_cnt >= self.bed_latch_scans:
            lim = self.bed_shape_max_heading
            heading = max(-lim, min(lim, heading))

        # Reliability-gated HOLD: in the dense regime, if the winning expert's
        # self-consistency collapsed (degenerate plateau fit — the dense-canopy
        # runaway/weave signature), do NOT chase this scan's geometry.  Hold the
        # previous smoothed lateral/heading so the estimate leans on history; keep
        # the (low) confidence so the state machine still sees the degraded lock.
        # Good tracking keeps reliability well above the floor, so this is inert
        # there.  Never engages until a prior estimate exists (self._est.valid).
        if (self.dual_row and dense and self._est.valid
                and self.reliability_floor
                and self.last_reliability < self.reliability_floor):
            lateral = float(self._est.lateral_offset)
            heading = float(self._est.heading_error)

        return self._smooth(RowEstimate(
            heading_error=heading,
            lateral_offset=lateral,
            confidence=confidence,
            row_end_confidence=row_end_conf,
            n_points=int(round(total_mass)),
            valid=True,
        ))

    # ------------------------------------------------------------------
    def _ground_slope(self, y: np.ndarray, z: np.ndarray) -> float:
        """Residual forward ground slope (dz/dy) over the ROI.

        Per 0.5 m forward range-bin the ground is the low percentile of z
        (the lowest returns = bare soil / furrow, whatever their absolute
        height — grade-agnostic).  A line is fit through these per-bin ground
        points; only its slope is used.  Using the slope (not the level) makes
        the estimate immune to the furrow/soil height bimodality and to the
        absolute calibration: a canopy-only cloud (no ground returns) yields a
        ~flat per-bin floor → slope ≈ 0 → no correction.  Returns 0 unless the
        slope clears a small dead-band (ignores flat-ground estimation noise),
        clamped to a plausible maximum grade.
        """
        if len(y) < self.ground_min_pts:
            return 0.0
        edges = np.arange(self.roi_y_min, self.roi_y_max + 0.5, 0.5)
        yc, gc = [], []
        idx = np.digitize(y, edges)
        for b in range(1, len(edges)):
            sel = idx == b
            if int(sel.sum()) < 8:
                continue
            yc.append(0.5 * (edges[b - 1] + edges[b]))
            gc.append(float(np.percentile(z[sel], 20)))
        if len(yc) < 3:
            return 0.0
        a = float(np.polyfit(np.asarray(yc), np.asarray(gc), 1)[0])
        a = max(-self._gmax, min(self._gmax, a))
        return a if abs(a) > self._gdead else 0.0

    # ------------------------------------------------------------------
    def _ground_roll(self, x: np.ndarray, z: np.ndarray) -> float:
        """Residual lateral ground slope (dz/dx) across the forward swath.

        Mirrors ``_ground_slope`` but bins by the cross-row coordinate x: per
        0.25 m x-bin the ground is the low percentile of z (bare soil / furrow,
        whatever its absolute height), and a line is fit through the per-bin
        ground points.  ``z`` is passed already forward-detrended so this
        isolates the lateral (roll) tilt of the ground plane.  Dead-banded and
        clamped exactly like the forward slope, so it is a no-op on level ground
        and cannot run away on a bad bin.
        """
        if len(x) < self.ground_min_pts:
            return 0.0
        edges = np.arange(-self.ground_roll_x_half,
                          self.ground_roll_x_half + 0.25, 0.25)
        xc, gc = [], []
        idx = np.digitize(x, edges)
        for b in range(1, len(edges)):
            sel = idx == b
            if int(sel.sum()) < 8:
                continue
            xc.append(0.5 * (edges[b - 1] + edges[b]))
            gc.append(float(np.percentile(z[sel], 20)))
        if len(xc) < 3:
            return 0.0
        b = float(np.polyfit(np.asarray(xc), np.asarray(gc), 1)[0])
        b = max(-self._gmax, min(self._gmax, b))
        return b if abs(b) > self._gdead else 0.0

    # ------------------------------------------------------------------
    def _nearest_peak(self, cross: np.ndarray, weights: Optional[np.ndarray] = None) -> float:
        """Histogram the cross-row axis; return the peak nearest centreline."""
        lo, hi = -self.roi_x_half, self.roi_x_half
        edges = np.arange(lo, hi + self.bin_width, self.bin_width)
        hist, edges = np.histogram(cross, bins=edges, weights=weights)
        centres = 0.5 * (edges[:-1] + edges[1:])
        smooth = np.convolve(hist.astype(float), [0.25, 0.5, 0.25], mode="same")
        thresh = max(3.0, 0.25 * float(smooth.max()))
        peaks = [
            i for i in range(1, len(smooth) - 1)
            if smooth[i] >= smooth[i - 1] and smooth[i] >= smooth[i + 1]
            and smooth[i] >= thresh
        ]
        if not peaks:
            peaks = [int(np.argmax(smooth))]
        best = min(peaks, key=lambda i: abs(centres[i]))
        return float(centres[best])

    # ------------------------------------------------------------------
    def _midpoint_peaks(self, cross: np.ndarray) -> float:
        """Dual-row / soybean centre-residue mode: locate the residue-strip
        centre via the shared ``find_row_midpoint`` (row-spacing prior +
        single-side half-spacing fallback)."""
        self._refine_spacing(cross)
        lateral, self._spacing_factor = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, self._spacing_est,
            prior_lateral=float(self._est.lateral_offset),
            prior_weight=self.midpoint_prior_weight)
        return lateral

    # ------------------------------------------------------------------
    def _dual_fit(self, P: np.ndarray, w: "Optional[np.ndarray]", n: int):
        """One dual-row geometric fit for a given per-point weight vector.

        Runs the two-pass cluster-centred PCA and the spacing-prior midpoint for
        the supplied weights (``w=None`` = uniform), WITHOUT mutating any state,
        so several perception hypotheses (experts) can be compared per scan.
        Returns ``(lateral, direction, linearity, spacing_factor, cross,
        spacing_refined)``.
        """
        if w is not None and n >= self.min_points:
            direction, linearity = _weighted_pca_dir(P[:n])
        else:
            direction, linearity = _weighted_pca_dir(P, w)
        perp = np.array([direction[1], -direction[0]])
        cross = P @ perp
        w_eff = w if w is not None else np.ones(len(P))
        for _ in range(2):
            peaks = histogram_peaks(cross, self.roi_x_half, self.bin_width, weights=w)
            direction, linearity = _cluster_centred_pca(
                P, w_eff, cross, peaks, fallback=(direction, linearity))
            perp = np.array([direction[1], -direction[0]])
            cross = P @ perp
        sp = self._refined_spacing(cross, weights=w)
        if self.bed_rows >= 2:
            # No tracked bed yet (start / after a U-turn): the strip-lock prior
            # would just pull toward x=0, so keep only a faint centre preference
            # and let the comb + soil edges decide.
            pw = self.midpoint_prior_weight if self._est.valid else 0.2
            lateral, spacing_factor = find_bed_centre(
                cross, self.roi_x_half, self.bin_width, sp, self.bed_rows,
                weights=w, prior_lateral=float(self._est.lateral_offset),
                prior_weight=pw, soil_centre=self._soil_centre(perp))
            return lateral, direction, linearity, spacing_factor, cross, sp
        lateral, spacing_factor = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, sp,
            weights=w, prior_lateral=float(self._est.lateral_offset),
            prior_weight=self.midpoint_prior_weight)
        return lateral, direction, linearity, spacing_factor, cross, sp

    def _bed_present(self) -> bool:
        """Bed-presence gate (see __init__): update the smoothed relief and its
        running baseline; False once raised beds are established (latched) and
        the smoothed relief has dropped below bed_present_frac × baseline."""
        if self._shape is None:
            self._shape_miss += 1
            if self._relief_sm is not None and self._shape_miss <= self.bed_dropout_hold:
                return self.last_bed_present        # dropout: no evidence either way
        else:
            self._shape_miss = 0
            self._last_shape = self._shape
        c = float(self._shape[2]) if self._shape is not None else 0.0
        self._relief_sm = c if self._relief_sm is None else 0.35 * c + 0.65 * self._relief_sm
        latched = self._relief_cnt >= self.bed_latch_scans
        # Hysteresis: once the bed has ended, it must come back clearly (0.8×)
        # before tracking resumes — no flicker at the threshold.
        frac = self.bed_return_frac if not self.last_bed_present else self.bed_present_frac
        floor_abs = self.bed_return_abs if not self.last_bed_present else self.bed_present_abs
        # Present if the relief is near its baseline OR clearly raised in
        # absolute terms: the profile includes the crop canopy, so the baseline
        # learnt in a full canopy (~0.3–0.5 m) would otherwise call a bare
        # stretch of bed (a long stand gap, 0.10 m) "bed ended".  A real bed end
        # is a flat headland (≈ 0) and still trips it.
        present = (not latched) or self._relief_sm >= min(frac * self._relief_base, floor_abs)
        # Once the bed has ENDED it must look present for bed_return_scans in a
        # row before tracking resumes: on the last metre of a bed the profile is
        # half grass and one good-looking scan re-locked it ~20 cm off (sim).
        if latched and not self.last_bed_present:
            self._present_run = self._present_run + 1 if present else 0
            present = self._present_run >= self.bed_return_scans
        else:
            self._present_run = 0
        # Baseline: slow EMA of healthy relief only (never learns the fade).
        if c >= self.bed_shape_ref * 0.8 and (
                not latched or self._relief_sm >= 0.8 * self._relief_base):
            self._relief_base = c if self._relief_base is None else 0.01 * c + 0.99 * self._relief_base
            self._relief_cnt += 1
        self.last_bed_present = present
        return present

    def _shape_estimate(self, roi_rot: float, n_mass: float) -> RowEstimate:
        """RowEstimate from the bed relief (self._shape), robot frame.

        Confidence scales with the bed-above-furrow contrast (full at
        bed_shape_ref).  Row end = the relief disappearing (headland is flat),
        so it is reported as the complement of the contrast confidence."""
        lat, hdg, contrast = self._shape
        conf = max(0.0, min(1.0, contrast / self.bed_shape_ref))
        # The bed visibly ends within ~3 m (nearest-quarter cut, sustained):
        # declare the row end now, as the old relief gate did — the preset's
        # 3.5 m headland exit is calibrated to an end seen ~3 m ahead.
        if self._bed_ending:
            conf *= 0.3
        self.last_mode = "shape"
        self.last_reliability = conf
        self._spacing_factor = 1.0
        # Straddling a bed with a few cm of furrow margin, the robot cannot be
        # more than a few degrees off the bed axis — cap the heading so a
        # fading/biased fit can't steer it out, and refuse an impossible offset.
        lim = self.bed_shape_max_heading
        heading = max(-lim, min(lim, hdg + roi_rot))
        # One-scan profile outlier: the robot cannot move 10 cm sideways in a
        # scan, so a reading that far from a confident track gets half the
        # trust (smaller EMA gain); a genuine sustained shift still comes through.
        if (self._est.valid and self._est.confidence >= 0.5
                and abs(lat - float(self._est.lateral_offset)) > self.bed_shape_jump):
            conf *= 0.5
        if abs(lat) > self.bed_max_offset:
            conf = 0.0
            if self._est.valid:
                lat = float(self._est.lateral_offset)
                heading = float(self._est.heading_error)
        return self._smooth(RowEstimate(
            heading_error=heading,
            lateral_offset=lat,
            confidence=conf,
            row_end_confidence=1.0 - conf,
            n_points=int(round(n_mass)),
            valid=True,
        ))

    def _bed_centre_robust(self, xs: np.ndarray, hd: np.ndarray):
        """Bed centre between the wheel furrows, robust to the growth stage:
        the median of the centres found on the 50th / 75th / 90th-percentile
        height profiles (``bed_relief_centre`` with gap fill).  Each percentile
        has a stage it is weak at (median: an interleaved canopy's furrow dip
        is shallow; 90th: a lodged canopy's ragged top) — the median of the
        three had the smallest worst case at every stage in the ray-cast sim
        (≤ 13 cm worst single scan vs 22–29 cm for any one percentile).
        Returns (centre, contrast) with the 90th-percentile contrast, or None."""
        cs, con = [], None
        rs = bed_relief_centre(xs, hd, pitch=self.bed_pitch, x_half=self.bed_shape_x_half,
                               c_max=self.bed_max_offset, stat=tuple(self.bed_profile_stats),
                               fill_gaps=self.bed_profile_fill)
        for r in (rs or ()):
            if r is None or r[1] < self.bed_shape_min_contrast:
                continue
            cs.append(r[0])
            con = r[1] if con is None else max(con, r[1])
        if not cs:
            return None
        return float(np.median(cs)), float(con)

    def _bed_shape(self, x: np.ndarray, y: np.ndarray, h: np.ndarray):
        """Bed geometry from the bed profile: (lateral, heading, contrast).

        Works in the heading-aligned ROI frame (the box comes from the smoothed
        memoryless sharpness search, ≤ 1° at every growth stage), so the bed
        axis is taken as the box axis and only the bed CENTRE is measured here:
        midway between the two wheel furrows, over the whole ROI depth.  Any
        forward ramp/level is removed per 0.5 m range bin (median), so
        slope/tilt residuals cannot bias the cross-row profile.

        Why no near-vs-far heading any more: the lateral is reported at the
        SENSOR (y = 0), 1.5–7 m behind the evidence, and once a canopy closes
        over the far furrows the far-half centre is noise (ray-cast sim: 14 cm
        rms at 2.6–3.7 m on a mature canopy) — a free near/far heading turned
        that into ±8° and ±0.3–0.6 m lateral errors.

        The bed END shows up first in the far half (field 2026-10-15), so when
        the far half's relief falls below ``bed_far_ratio`` × the near half's,
        only the near half is used.  None when the relief is too faint.
        """
        sel = ((y >= self.roi_y_min) & (y <= self.roi_y_max)
               & (np.abs(x) <= self.bed_shape_x_half))
        nsel = int(sel.sum())
        self._view_n = nsel if self._view_n is None else 0.85 * self._view_n + 0.15 * nsel
        if nsel < 400:
            return None
        xs, ys, hs = x[sel], y[sel], h[sel]
        # VLP-16 UDP packet loss drops whole azimuth SECTORS.  The empty columns
        # would read as furrows (return-less = gap), so a scan with a hole in
        # its azimuth coverage gives no profile; the dropout hold coasts on the
        # last bed geometry for it.
        az = np.degrees(np.arctan2(xs, ys))
        occ = np.bincount(np.clip(((az + 46.0) / 2.0).astype(int), 0, 45), minlength=46) > 0
        occ_i = np.nonzero(occ)[0]
        if len(occ_i) and int(np.max(np.diff(occ_i), initial=1)) > 3:
            self.last_sector_gap = True
            return None
        self.last_sector_gap = False
        if len(xs) > 9000:                    # Jetson budget: profile needs far fewer
            st = int(np.ceil(len(xs) / 9000))
            xs, ys, hs = xs[::st], ys[::st], hs[::st]
        yb = ((ys - self.roi_y_min) / 0.5).astype(int)
        hd = np.empty_like(hs)
        for b in np.unique(yb):
            m = yb == b
            hd[m] = hs[m] - np.median(hs[m])
        kw = dict(pitch=self.bed_pitch, x_half=self.bed_shape_x_half,
                  c_max=self.bed_max_offset, stat=self.bed_profile_stat,
                  fill_gaps=self.bed_profile_fill)
        ymid = 0.5 * (self.roi_y_min + self.roi_y_max)
        near = ys < ymid
        mc = self.bed_shape_min_contrast
        fn = bed_relief_centre(xs[near], hd[near], **kw)
        ff = bed_relief_centre(xs[~near], hd[~near], **kw)
        near_ok = fn is not None and fn[1] >= mc
        # "The bed is ENDING ahead" = the far half's relief dropped below ITS
        # OWN running baseline (not below the near half's: under a mature
        # canopy the far furrows are always partly hidden, so far < 0.7 × near
        # fired mid-bed).  Same for the second quarter vs the first.
        ffc = ff[1] if (ff is not None and ff[1] >= mc) else 0.0
        far_ok = ffc > 0.0 and (self._far_base is None or ffc >= self.bed_far_ratio * self._far_base)
        if far_ok and near_ok:
            self._far_base = ffc if self._far_base is None else self._far_base + 0.02 * (ffc - self._far_base)
        self._shape_far_ok = bool(far_ok)
        yq = 0.5 * (self.roi_y_min + ymid)
        if far_ok:
            self._bed_y_cut = None
            r = self._bed_centre_robust(xs, hd)
        elif near_ok:
            # Approaching the bed END: the far half is already headland.  If
            # the end is INSIDE the near half too (second quarter fading vs its
            # baseline), use the nearest quarter only — the grass beyond
            # biased it 15 cm.
            q1 = ys < yq
            q2 = near & ~q1
            f2 = bed_relief_centre(xs[q2], hd[q2], **kw)
            f2c = f2[1] if (f2 is not None and f2[1] >= mc) else 0.0
            # …or above an absolute floor, as for bed presence: mid-bed this
            # quarter's relief is ≥ 8 cm at every stage, at a bed end 0–4 cm
            # (sim).  A baseline seeded by one wild profile right after a
            # U-turn (51 cm, mature canopy, 0.99 m mount) is only refreshed on
            # the rare scans the far half is visible, and alone it latched a
            # false "bed ending" for the rest of the bed.
            q2_ok = f2c > 0.0 and (self._q2_base is None
                                   or f2c >= min(self.bed_far_ratio * self._q2_base,
                                                 self.bed_q2_abs))
            if self._q2_base is None and f2c >= self.bed_q2_abs:
                self._q2_base = f2c           # this quarter has shown the bed
            if self._far_base is None and self._q2_base is None:
                # The far bed has never shown relief since the reset: a short
                # view (tall canopy), not an end — keep the whole swath.  Cutting
                # the axis search to the nearest 1.4 m let it run away 0 → 10°
                # (sim, 0.70 m plants, 15.7° mount: 25 cm at confidence 0.9).
                self._bed_y_cut = None
                r = self._bed_centre_robust(xs, hd)
            elif q2_ok:
                self._bed_y_cut = ymid
                r = self._bed_centre_robust(xs[near], hd[near])
            else:
                self._bed_y_cut = yq
                r = self._bed_centre_robust(xs[q1], hd[q1])
        else:
            return None
        # A bed END is a CHANGE — the far bed was seen, then fades.  If neither
        # the far half nor the second quarter has shown any relief since the
        # reset, there is no evidence of an end, only of a short view (tall
        # canopy from the steeper 15.7° mount: the second quarter never shows
        # relief, and the latch capped confidence at 0.3 from the first metre).
        self._bed_end_armed = self._far_base is not None or self._q2_base is not None
        self._qcut_run = self._qcut_run + 1 if (
            self._bed_y_cut is not None and self._bed_y_cut <= yq + 1e-6
            and self._bed_end_armed) else 0
        # Latch "bed ending" once the quarter cut is sustained; release only
        # when the far bed is clearly back (10 scans) or on reset().
        if self._qcut_run >= self.bed_end_cut_scans:
            self._bed_ending = True
        self._far_back = self._far_back + 1 if far_ok else 0
        if self._far_back >= 10:
            self._bed_ending = False
        if far_ok and near_ok:
            # second-quarter baseline (cheap: only while the whole bed is in view
            # every few scans)
            self._q2_tick = (self._q2_tick + 1) % 4
            if self._q2_tick == 0:
                q2 = near & (ys >= yq)
                f2 = bed_relief_centre(xs[q2], hd[q2], **kw)
                if f2 is not None and f2[1] >= mc:
                    self._q2_base = f2[1] if self._q2_base is None else self._q2_base + 0.08 * (f2[1] - self._q2_base)
        if r is None:
            return None
        # Relief strength for confidence / the bed-presence gate: the NEAR
        # bed counts (a bed ending 5 m ahead is not yet a row end here).
        contrast = max(r[1], fn[1] if near_ok else 0.0)
        return r[0], 0.0, contrast

    @staticmethod
    def _surface_mode(hv: np.ndarray, bin_m: float = 0.01) -> float:
        """Height of the dominant flat surface in a set of returns.

        The bed top is a flat surface, so its returns pile into a narrow height
        spike; crop spreads over tens of cm and the shoulders/furrows are a
        minority, so neither moves the spike.  A low percentile instead slides
        down onto the shoulder when the robot is off-centre (or up into the
        crop under a dense canopy).  Smoothed 1 cm histogram, arg-max.
        """
        lo, hi = float(hv.min()), float(hv.max())
        if hi - lo < 2 * bin_m:
            return float(np.median(hv))
        nb = int((hi - lo) / bin_m) + 1
        cnt = np.bincount(((hv - lo) / bin_m).astype(int), minlength=nb).astype(float)
        cnt = np.convolve(cnt, [1.0, 2.0, 3.0, 2.0, 1.0], mode="same")
        return lo + (int(np.argmax(cnt)) + 0.5) * bin_m

    def _soil_mode(self, hv: np.ndarray, bin_m: float = 0.01, lo_lim: float = -0.05,
                   hi_lim: "Optional[float]" = None,
                   peak_frac: "Optional[float]" = None) -> "Optional[float]":
        """Height of the bed-top SOIL in a set of returns, or None if hidden.

        The soil is the LOWEST substantial surface: crop, weeds and canopy all
        stand on it.  The old rule (the DOMINANT surface) flipped to the canopy
        as soon as leaves out-numbered soil returns — dense young rows (sim:
        soil 70 vs canopy 87 returns per bin) emptied the crop band, and the
        bed floor jumped soil↔canopy scan to scan (2026-10-16 relief dropouts).
        A peak counts if it holds ≥ ``bed_soil_peak_frac`` of the largest peak
        and lies in the plausible bed-top range; none → soil not visible."""
        if hi_lim is None:
            hi_lim = self.bed_soil_max
        if peak_frac is None:
            peak_frac = self.bed_soil_peak_frac
        if len(hv) < 8:
            return None
        lo, hi = float(hv.min()), float(hv.max())
        if hi - lo < 2 * bin_m:
            v = float(np.median(hv))
            return v if lo_lim <= v <= hi_lim else None
        nb = int((hi - lo) / bin_m) + 1
        cnt = np.bincount(((hv - lo) / bin_m).astype(int), minlength=nb).astype(float)
        cnt = np.convolve(cnt, [1.0, 2.0, 3.0, 2.0, 1.0], mode="same")
        peak = float(cnt.max())
        need = max(peak_frac * peak, 8.0)
        for i in range(nb):
            if cnt[i] < need:
                continue
            if (i == 0 or cnt[i] >= cnt[i - 1]) and (i == nb - 1 or cnt[i] >= cnt[i + 1]):
                v = lo + (i + 0.5) * bin_m
                if v > hi_lim:
                    return None           # lowest real surface is above any bed top: canopy
                if v >= lo_lim:
                    return v
        return None

    def _furrow_fit(self, y: np.ndarray, h: np.ndarray) -> "Optional[tuple[float, float]]":
        """The furrow (wheel-track) ground as a line in range, or None.

        Per 0.5 m range bin, the lowest substantial surface across the wide
        swath (both wheel furrows) is the furrow bottom; a line through those
        tracks any residual forward ramp (tilt calibration error, terrain) —
        which the ±2° dead-banded ground detrend leaves in place.  Rejected if
        its near level or slope is implausible (a canopy wall is not ground)."""
        edges = np.arange(self.roi_y_min, self.roi_y_max + 0.5, 0.5)
        idx = np.digitize(y, edges)
        yc, fc = [], []
        for b in range(1, len(edges)):
            sel = idx == b
            if int(sel.sum()) < 8:
                continue
            v = self._soil_mode(h[sel], lo_lim=-0.35, hi_lim=0.35, peak_frac=0.10)
            if v is None:
                continue
            yb_c = 0.5 * (edges[b - 1] + edges[b])
            # Must continue the furrow already tracked (or the calibrated
            # ground): under a canopy the far bins' "lowest surface" is the
            # bottom of a canopy wall, 0.3–0.4 m up — a line through it and the
            # near furrow passed as a 3° ramp (ray-cast sim).
            if self._furrow is not None:
                pred = self._furrow[0] + self._furrow[1] * yb_c
                if abs(v - pred) > 0.08:
                    continue
            yc.append(yb_c)
            fc.append(v)
        if len(yc) < 3:
            return None
        # Trim bins off the line: from the 0.99 m mount some 0.5 m bins catch
        # too few furrow returns (ring spacing / shadowing) and their "lowest
        # surface" is the bed top, 9–15 cm up — it failed the smoothness test
        # below on every scan, so the line was first fitted later, under a
        # canopy, on canopy-wall bottoms (bed floor read +5 cm).  At most a
        # third of the bins may be dropped.
        yc, fc = np.asarray(yc), np.asarray(fc)
        n0 = len(yc)
        while True:
            c1, c0 = (float(v) for v in np.polyfit(yc, fc, 1))
            res = np.abs(fc - (c0 + c1 * yc))
            k = int(np.argmax(res))
            if res[k] <= 0.03 or len(yc) - 1 < max(3, n0 - n0 // 3):
                break
            yc, fc = np.delete(yc, k), np.delete(fc, k)
        # The wheels stand IN the furrow, so at the robot (y = 0) the furrow is
        # the calibrated ground: |intercept| ≤ 4 cm whatever the ramp.
        if abs(c1) > math.tan(math.radians(3.5)) or abs(c0) > 0.04:
            return None
        # Furrow ground is a smooth line; canopy-wall bottoms (closed canopy,
        # furrow shadowed) are ragged — a fresh fit must lie on its bins.
        if self._furrow is None:
            res = fc - (c0 + c1 * yc)
            if float(np.sqrt(np.mean(res * res))) > 0.02:
                return None
        return c0, c1

    def _bed_floor_line(self, y: np.ndarray, hr: np.ndarray) -> "tuple[float, float]":
        """Bed-top surface as a line in range: (c0, c1) with floor = c0 + c1·y.

        Per 0.5 m range bin, the low percentile of height is the bed-top soil
        (crop sits above it); a least-squares line through those bin floors
        tracks any residual forward slope exactly (no dead-band).  Falls back
        to a flat floor when fewer than three bins are populated.  Level and
        slope are clamped to plausible ranges so a weird scan cannot run away.
        """
        edges = np.arange(self.roi_y_min, self.roi_y_max + 0.5, 0.5)
        idx = np.digitize(y, edges)
        yc, fc = [], []
        for b in range(1, len(edges)):
            sel = idx == b
            if int(sel.sum()) < 8:
                continue
            v = self._soil_mode(hr[sel])
            if v is None:
                continue                      # this range bin sees no soil (canopy)
            yc.append(0.5 * (edges[b - 1] + edges[b]))
            fc.append(v)
        self.last_soil_bins = len(yc)
        lim = self.bed_floor_max
        if not yc:
            return None
        if len(yc) >= 3:
            c1, c0 = (float(v) for v in np.polyfit(np.asarray(yc), np.asarray(fc), 1))
            c1 = max(-self._gmax, min(self._gmax, c1))
            # re-anchor the level at the bins' mean range after the slope clamp
            ym = float(np.mean(yc))
            c0 = float(np.mean(fc)) - c1 * ym
            mid = c0 + c1 * ym
            if abs(mid) > lim:
                c0 += math.copysign(lim, mid) - mid
            return c0, c1
        f = float(np.mean(fc))
        return max(-lim, min(lim, f)), 0.0

    def _bed_relief_heading(self, pts: np.ndarray) -> "Optional[float]":
        """Bed direction from the soil relief (no crop needed), memoryless.

        Rotating the cloud to the true bed axis makes the cross-row SOIL
        profile sharpest (plateau + furrows line up along the whole depth);
        any misalignment smears the shoulders across x and lowers the profile
        variance.  Soil ≈ the lower 60 % of heights per 0.5 m range bin (crop
        tops/grass above it), forward ramp removed per bin.  1° grid over
        ±bed_shape_max_heading + parabolic refinement.  None if too sparse or
        the relief is flat (headland)."""
        x, y = pts[:, 0], pts[:, 1]
        h = pts[:, 2]
        sel = ((y >= self.roi_y_min) & (y <= self.roi_y_max)
               & (np.abs(x) <= self.bed_shape_x_half + 0.3))
        if int(sel.sum()) < 600:
            return None
        xs, ys, hs = x[sel], y[sel], h[sel]
        if len(xs) > 8000:
            st = int(np.ceil(len(xs) / 8000))
            xs, ys, hs = xs[::st], ys[::st], hs[::st]
        yb = ((ys - self.roi_y_min) / 0.5).astype(int)
        keep = np.zeros(len(hs), dtype=bool)
        hd = np.empty_like(hs)
        for b in np.unique(yb):
            m = yb == b
            hb = hs[m]
            if len(hb) < 8:
                continue
            # Soil only: at/below the dominant surface (+3 cm) — bed top and
            # furrows stay, crop tops and GRASS (bed-end headland) go.
            surf = self._surface_mode(hb)
            hd[m] = hb - surf
            keep[m] = hb <= surf + 0.03
        xs, ys, hd = xs[keep], ys[keep], hd[keep]
        if len(xs) > 2500:
            st = int(np.ceil(len(xs) / 2500))
            xs, ys, hd = xs[::st], ys[::st], hd[::st]
        lim = math.degrees(self.bed_shape_max_heading)
        angs = np.radians(np.arange(-lim, lim + 1e-9, 1.0))
        bw = 0.04
        xh = self.bed_shape_x_half
        nb = int(round(2 * xh / bw))
        score = np.full(len(angs), -np.inf)
        for i, a in enumerate(angs):
            xr = xs * math.cos(a) - ys * math.sin(a)
            m = np.abs(xr) < xh
            b = ((xr[m] + xh) / bw).astype(int)
            cnt = np.bincount(b, minlength=nb)
            s = np.bincount(b, weights=hd[m], minlength=nb)
            ok = cnt >= 5
            if int(ok.sum()) < 0.6 * nb:
                continue
            prof = s[ok] / cnt[ok]
            score[i] = float(prof.var())
        k = int(np.argmax(score))
        if not np.isfinite(score[k]) or score[k] < (0.5 * self.bed_shape_min_contrast) ** 2:
            return None
        fin = score[np.isfinite(score)]
        if score[k] < 1.05 * float(np.median(fin)):
            return None                       # no distinct alignment (flat / smeared)
        a = float(angs[k])
        if 0 < k < len(angs) - 1 and np.isfinite(score[k - 1]) and np.isfinite(score[k + 1]):
            den = score[k - 1] - 2 * score[k] + score[k + 1]
            if den < 0:
                a += math.radians(1.0) * 0.5 * (score[k - 1] - score[k + 1]) / den
        return a

    def _bed_box_heading(self, pts: np.ndarray,
                         y_max: "Optional[float]" = None) -> "Optional[float]":
        """Row / bed direction by cross-row sharpness (bed-mode ROI alignment).

        Projects the swath's points onto the cross-row axis for each candidate
        heading and returns the heading at which the cross-row structure is
        sharpest.  Three cues — the clearer COUNT cue, the height-variance cue
        only when no count cue has a clear peak (growth-stage ray-cast sim):

          * COUNT of the upper surface (height above each range bin's median ≥
            −2 cm: bed tops / rows / canopy, not furrows) — terrain-relative, so
            a 4 cm bed, a 1.5° tilt residual or weeds don't break it;
          * COUNT in the absolute crop band — a mature canopy's walls (only
            when the band holds ≥ bed_axis_band_min of the swath);
          * HEIGHT-profile variance — a lodged canopy: a broad, evenly-hit
            mound with no count stripes, but a clear mound-vs-furrow profile.

        Two passes: ±bed_roi_rot_max at 2°, then ±2.5° at 0.25° on a swath
        re-selected in the first estimate's frame.  A robot-frame swath is
        lopsided about an angled bed and biased the angle 1–2° toward zero at
        8–10° (≈ 10 cm of lateral at the sensor).  Memoryless, so it cannot
        lock onto a wrong angle the way a self-seeded fit can.  None when there
        are too few points or no cue has a clear winner."""
        r = self._box_search(pts, 0.0, math.degrees(self.bed_roi_rot_max), 2.0, y_max=y_max)
        if r is None:
            return None
        a, cue = r
        r2 = self._box_search(pts, a, 2.5, 0.25, cue=cue, y_max=y_max)   # refine, same cue
        return r2[0] if r2 is not None else a

    def _box_search(self, pts: np.ndarray, centre: float, half_deg: float,
                    step_deg: float, cue: "Optional[int]" = None,
                    y_max: "Optional[float]" = None):
        """One sharpness search around ``centre``; returns (angle, cue index)."""
        x, y = pts[:, 0], pts[:, 1]
        h = pts[:, 2] + LIDAR_MOUNT_HEIGHT
        if centre:
            c, s_ = math.cos(centre), math.sin(centre)
            x, y = x * c - y * s_, x * s_ + y * c
        # ONE fixed point set for every candidate angle (a wide swath that also
        # holds the neighbouring beds' parallel rows) and a histogram wide
        # enough that no point ever falls outside it — so the sharpness values
        # are directly comparable across angles.
        sel = ((y >= self.roi_y_min) & (y <= (y_max or self.roi_y_max))
               & (np.abs(x) <= self.bed_dir_x_half)
               & (h <= self.crop_h_max + self.bed_floor_max))
        if int(sel.sum()) < self.ground_min_pts:
            return None
        xs, ys, hs = x[sel], y[sel], h[sel]
        if len(xs) > 3000:                    # Jetson budget: stride-subsample
            step = int(np.ceil(len(xs) / 3000))
            xs, ys, hs = xs[::step], ys[::step], hs[::step]
        yb = ((ys - self.roi_y_min) / 0.5).astype(int)
        hd = np.empty_like(hs)
        for b in np.unique(yb):
            m = yb == b
            hd[m] = hs[m] - np.median(hs[m])
        up = hd >= -0.02
        band = hs >= self.crop_h_min
        n_ang = int(round(2 * half_deg / step_deg)) + 1
        angs = np.radians(np.linspace(-half_deg, half_deg, n_ang))
        # cross-row coordinate of every point for every candidate heading
        cross = np.cos(angs)[:, None] * xs[None, :] - np.sin(angs)[:, None] * ys[None, :]
        lo = float(cross.min())
        nb = int((float(cross.max()) - lo) / self.bin_width) + 2
        sharp = np.empty(n_ang)
        sharp_b = np.empty(n_ang)
        hvar = np.full(n_ang, np.nan)
        for i in range(n_ang):
            b = ((cross[i] - lo) / self.bin_width).astype(int)
            cu = np.bincount(b[up], minlength=nb).astype(float)
            sharp[i] = float((cu * cu).sum())
            cb = np.bincount(b[band], minlength=nb).astype(float)
            sharp_b[i] = float((cb * cb).sum())
            cnt = np.bincount(b, minlength=nb).astype(float)
            hs_b = np.bincount(b, weights=hd, minlength=nb)
            ok = cnt >= 3
            if int(ok.sum()) >= 8:
                hvar[i] = float((hs_b[ok] / cnt[ok]).var())
        self._cue_ratios = [0.0, 0.0, 0.0]
        cands = []                            # (ratio, angle, cue) of every clear cue
        for ci, score in enumerate((sharp, sharp_b, hvar)):
            if cue is not None and ci != cue:
                continue
            if not np.isfinite(score).all() or float(score.max()) <= 0.0:
                continue
            k = int(np.argmax(score))
            ratio = score[k] / max(float(np.median(score)), 1e-12)
            self._cue_ratios[ci] = float(ratio)
            if ratio < 1.15 and cue is None:
                continue
            a = float(angs[k])
            if 0 < k < n_ang - 1:
                den = score[k - 1] - 2 * score[k] + score[k + 1]
                if den < 0:
                    a += math.radians(step_deg) * 0.5 * (score[k - 1] - score[k + 1]) / den
            cands.append((float(ratio), centre + a, ci))
        if not cands:
            return None                       # no clear row direction this scan
        # Which cue to trust (ray-cast sim, every stage, 0.80 and 0.99 m
        # mounts): the COUNT cues are ≤ ~1° wherever they have a clear peak —
        # except the crop-band count when the band is nearly empty (4 cm bed,
        # tilt residual: ≤ 15 % of the swath, up to 30° off).  The
        # height-variance cue is 5–17° off under a mature canopy, yet the only
        # accurate one on a lodged canopy, where no count cue has a clear peak.
        # Its peak/median ratio lives on a larger scale (1.6–2.0 even on a bare
        # bed), so "clearest wins" let it drag a mature-canopy axis 5° (U-turn
        # re-entry, 0.99 m mount).
        if cue is None:
            band_ok = float(np.mean(band)) >= self.bed_axis_band_min
            counts = [c for c in cands if c[2] == 0 or (c[2] == 1 and band_ok)]
            if counts:
                cands = counts
            elif not any(c[2] == 2 for c in cands):
                return None
            else:
                cands = [c for c in cands if c[2] == 2]
        best = max(cands, key=lambda c: c[0])
        return best[1], best[2]

    def _soil_centre(self, perp: np.ndarray) -> "Optional[float]":
        """Bed centre from the raised-bed soil plateau's two edges, or None.

        Valid only when BOTH edges are seen well inside the ROI and the plateau
        width is plausible — otherwise (flat ground, robot far off-centre,
        bed top wider than the ROI) the edges are just the ROI clip."""
        self.last_soil_centre = None
        if self._bed_soil_xy is None:
            return None
        sc = self._bed_soil_xy @ perp
        lo = float(np.percentile(sc, self.bed_soil_edge_pct))
        hi = float(np.percentile(sc, 100.0 - self.bed_soil_edge_pct))
        lim = self.roi_x_half - 0.08
        if lo <= -lim or hi >= lim or not (0.40 <= hi - lo <= 1.40):
            return None
        self.last_soil_centre = 0.5 * (lo + hi)
        return self.last_soil_centre

    def _refined_spacing(self, cross: np.ndarray,
                         weights: "Optional[np.ndarray]" = None) -> float:
        """Pure: the row-spacing estimate refined by this scan's peaks (does NOT
        mutate ``self._spacing_est``) so several perception hypotheses can be
        evaluated per scan before one is committed.

        When a clear left AND right peak are present, their separation is a
        direct measurement of the field's row spacing.  A slow EMA folds it in,
        gated to reject implausible separations (clutter) and large jumps, so
        the estimate tracks the true spacing without chasing noise.  The seed
        (``--row-spacing``) only matters until the first good two-row view.
        """
        if not self.auto_spacing:
            return self._spacing_est
        peaks = histogram_peaks(cross, self.roi_x_half, self.bin_width, weights)
        left = [p for p in peaks if p < -0.05]
        right = [p for p in peaks if p > 0.05]
        if not (left and right):
            return self._spacing_est
        # Pick the pair whose separation is closest to the current estimate.
        pl, pr = min(((l, r) for l in left for r in right),
                     key=lambda lr: abs((lr[1] - lr[0]) - self._spacing_est))
        sep = pr - pl
        # Anchor the self-cal to the seed.  A field's crop spacing is FIXED and
        # roughly known (standard row widths), so the estimate should only
        # REFINE within a band around the seed (±25%), never run away: field
        # logs showed an un-anchored estimate wandering 0.52–1.02 m around a
        # true ~0.76 m, dragging the row-centre target and adding weave.  Also
        # reject per-scan outliers (clutter) more tightly (40% of the estimate),
        # with a slow EMA.
        lo, hi = 0.75 * self.row_spacing, 1.25 * self.row_spacing
        if lo <= sep <= hi and abs(sep - self._spacing_est) <= 0.40 * self._spacing_est:
            return float(np.clip(0.08 * sep + 0.92 * self._spacing_est, lo, hi))
        return self._spacing_est

    def _refine_spacing(self, cross: np.ndarray,
                        weights: "Optional[np.ndarray]" = None) -> None:
        """Commit the refined spacing (mutating wrapper for the single-row path)."""
        self._spacing_est = self._refined_spacing(cross, weights)

    @property
    def spacing_estimate(self) -> float:
        """The live, self-calibrated row-spacing estimate (metres)."""
        return self._spacing_est

    # ------------------------------------------------------------------
    def _smooth(self, fresh: RowEstimate) -> RowEstimate:
        """Exponentially blend a fresh estimate into the running state."""
        if self.bed_rows >= 2 and self._view_n is not None:
            # Bed mode: a collapsed view (canopy at LiDAR height) lowers trust
            # in whatever was fitted — see view_n_min / view_n_full.
            q = (self._view_n - self.view_n_min) / (self.view_n_full - self.view_n_min)
            q = max(0.0, min(1.0, q))
            self.last_view_quality = q
            if q < 1.0:
                fresh = RowEstimate(heading_error=fresh.heading_error,
                                    lateral_offset=fresh.lateral_offset,
                                    confidence=fresh.confidence * q,
                                    row_end_confidence=fresh.row_end_confidence,
                                    n_points=fresh.n_points, valid=fresh.valid)
        # Confidence-weighted gain: trust a fresh scan in proportion to its
        # confidence (relative to a healthy lock), floored so a sustained real
        # change still gets through.  A low-confidence scan therefore nudges the
        # tracked lateral/heading less and the estimate holds its recent trend.
        a = self.ema_alpha
        if self.temporal_trust:
            g = fresh.confidence / self.temporal_ref_conf if self.temporal_ref_conf else 1.0
            a = self.ema_alpha * max(self.temporal_min_gain, min(1.0, g))
        self.last_gain = a
        prev = self._est

        # Heading outlier gate: if the fresh PCA heading jumps more than 30°
        # from the current smoothed heading, clamp it. Sparse or nearly-round
        # crop clusters (row ends, cardboard, thin canopy) give PCA directions
        # that can flip ±90°; without the gate the heading EMA oscillates ±50°
        # and pure-pursuit produces diverging steering corrections.
        # Threshold 0.15 (was 0.30): n=0 scans decay confidence ×0.75, so
        # starting from conf≈0.45 it falls to 0.34→0.25. With 0.30 the gate
        # disabled at 0.25 and allowed unclamped jumps, drifting heading +5°/scan.
        hdg_fresh = fresh.heading_error
        lat_fresh = fresh.lateral_offset
        if prev.confidence > 0.15:
            delta = hdg_fresh - prev.heading_error
            # Sparse-scan heading quality gate: a VLP-16 dropout scan that
            # loses whole azimuth sectors (left=0 right=0 in validate output)
            # leaves mostly forward-facing beams.  The foreshortened two-stripe
            # view makes the PCA heading unreliable — in the field a scan with
            # n≈67 crop points produced a raw heading of ~27° when the true
            # heading was ~1°.  This is larger than the standard 30° gate so
            # it passed through and was folded into the EMA, biasing the robot
            # rightward for the remainder of the approach.
            # When the scan is sparse (< 2×min_points) AND the heading jumps
            # more than 12°, discard the heading update entirely and keep the
            # previous smoothed heading.  Small genuine changes (≤12°) are
            # still accepted even from sparse scans.  The lateral offset and
            # confidence are updated normally — only the heading is frozen.
            sparse = fresh.n_points < 2 * self.min_points
            if sparse and abs(delta) > math.radians(12.0):
                hdg_fresh = prev.heading_error          # skip heading update
            elif abs(delta) > math.radians(30.0):
                hdg_fresh = prev.heading_error + math.copysign(math.radians(30.0), delta)
            # Lateral outlier gate: the dual-row midpoint can snap by half a
            # row spacing in one scan when peak pairing changes (one row
            # momentarily occluded).  The robot cannot physically translate
            # that fast at 10 Hz, so clamp the per-scan jump.
            d_lat = lat_fresh - prev.lateral_offset
            if abs(d_lat) > self.max_lateral_jump:
                lat_fresh = prev.lateral_offset + math.copysign(self.max_lateral_jump, d_lat)

        sm_hdg = a * hdg_fresh + (1 - a) * prev.heading_error
        sm_lat = a * lat_fresh + (1 - a) * prev.lateral_offset
        # Heading–lateral consistency clamp (see __init__): the reported heading
        # magnitude is capped by a bound that scales with the lateral offset, so
        # a centred robot on a straight row cannot report (and the controller
        # cannot chase) a large heading that is really a terrain/PCA artifact.
        if abs(sm_lat) < self.heading_consistency_lat:
            frac = abs(sm_lat) / self.heading_consistency_lat
            cap = self.heading_cap_centred + (
                self.heading_cap_gate - self.heading_cap_centred) * frac
            if abs(sm_hdg) > cap:
                sm_hdg = math.copysign(cap, sm_hdg)
        # ABSOLUTE heading ceiling — applies REGARDLESS of lateral offset.
        # The proportional cap above relaxes to unlimited once |lateral| passes
        # its gate (0.22 m), on the theory that a genuinely off-strip robot may
        # need a large steering angle to return.  A field RL run proved that
        # assumption exploitable: in a sparse/gappy patch the detector produced a
        # SELF-CONSISTENT but wrong fit reporting heading +68°→+86° AND lateral
        # −0.42 m together (rel ≈ 0.5, so the reliability hold did not fire), and
        # the reactive RL controller whipped the robot ~90° into the crop wall
        # (repeated OBSTACLE_WAIT; the operator had to take over).  A row-following
        # heading beyond ~35° is non-physical — you never approach a crop row at
        # 60–86° — so it is a bad fit, not a real angled approach.  Clamp it
        # unconditionally so no controller (especially RL, which reacts fastest)
        # can be driven to turn violently on a garbage estimate.  Normal following
        # (±5°) and genuine off-strip recovery (≲ atan(offset/lookahead), ~10–20°)
        # are well under the ceiling, so this is inert there.  --heading-max.
        if abs(sm_hdg) > self.heading_abs_max:
            sm_hdg = math.copysign(self.heading_abs_max, sm_hdg)
        self._est = RowEstimate(
            heading_error=sm_hdg,
            lateral_offset=sm_lat,
            # Confidence tracks at the BASE rate (not trust-weighted) so a real
            # quality drop still lowers it and the state machine / speed law react.
            confidence=self.ema_alpha * fresh.confidence + (1 - self.ema_alpha) * prev.confidence,
            row_end_confidence=fresh.row_end_confidence,
            n_points=fresh.n_points,
            valid=True,
        )
        return self._est

    # ------------------------------------------------------------------
    def _decay(self, row_end_conf: float = 1.0) -> RowEstimate:
        """No detection this scan — keep geometry, drop confidence.

        Factor 0.75 (was 0.5) — at 10 Hz a VLP-16 can produce 1–3 empty crop
        ROI scans due to beam angle variation between rotations; 0.5 caused
        confidence to drop below the 0.35 FOLLOW threshold after just 2 empty
        scans (~200 ms), triggering spurious FOLLOW→ACQUIRE→FOLLOW cycles.
        With 0.75 it takes ~5 consecutive empty scans (~500 ms) to cross the
        threshold, which is combined with the navigator's follow_miss_thresh
        counter for a second layer of debounce.
        """
        prev = self._est
        self._est = RowEstimate(
            heading_error=prev.heading_error,
            lateral_offset=prev.lateral_offset,
            confidence=prev.confidence * 0.75,
            row_end_confidence=max(prev.row_end_confidence, row_end_conf),
            n_points=0,
            valid=False,
        )
        return self._est
