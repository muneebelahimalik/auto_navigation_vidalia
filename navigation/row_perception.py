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
        dense_canopy: bool = True,
        canopy_tall_h: float = 0.45,
        dense_canopy_frac: float = 0.35,
        strip_floor_pct: float = 25.0,
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
        # --- LiDAR crop-band points ---
        if pts is not None and len(pts) > 0:
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
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
            self.last_ground_slope = a
            self.last_ground_shift = shift
            roi = (in_xy & (h_eff >= self.crop_h_min + shift)
                   & (h_eff <= self.crop_h_max + shift))
            cx, cy = x[roi], y[roi]
            ch = h_eff[roi]                    # ground-relative height of each ROI point
            n = int(cx.shape[0])
        else:
            cx = cy = ch = np.empty(0)
            n = 0

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
        if total_mass < self.min_points:
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
            if len(cross_lidar):
                lsel = np.abs(cross_lidar - (lateral - hs)) <= self.row_end_side_window
                rsel = np.abs(cross_lidar - (lateral + hs)) <= self.row_end_side_window
                row_strength = float(max(int(lsel.sum()), int(rsel.sum())))
            else:
                row_strength = 0.0
            row_end_conf = 1.0 - min(1.0, row_strength / self.row_end_side_density)
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

        heading = math.atan2(direction[0], direction[1])
        density = min(1.0, total_mass / self.full_points)
        linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
        confidence = density * linear_factor
        if self.dual_row:
            confidence *= self._spacing_factor

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
        lateral, spacing_factor = find_row_midpoint(
            cross, self.roi_x_half, self.bin_width, sp,
            weights=w, prior_lateral=float(self._est.lateral_offset),
            prior_weight=self.midpoint_prior_weight)
        return lateral, direction, linearity, spacing_factor, cross, sp

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
