# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/);
versions are git tags on `main`.

## [Unreleased] — headland U-turn reliability (not yet field-validated)

Work toward the row-to-row turn milestone (built on the v0.1.0 baseline).

### Added — 3-D field mapping (`--map-3d`, mapping only)
- **3-D point-cloud map of the field**, built on the corrected 2-D SLAM pose
  (2.5-D: a ground robot's 3-DOF pose + the static tilt correction registers the
  full 3-D scan — no fragile 6-DOF SLAM needed). `slam/voxel_map.py` accumulates
  one point per voxel (size grows with field area, not scan count);
  `slam/scan_matcher.robot_xyz_to_world` registers each full corrected cloud with
  the loop-closed pose. `slam_mapper.py --map-3d` writes `map3d.ply` (binary,
  height-coloured, opens in CloudCompare/MeshLab/Foxglove) + `map3d.npz`; status
  line shows the live voxel count. Map z is height above the local ground plane
  (robot elevation isn't tracked → not an absolute DEM). New `--voxel-3d`
  (0.15 m default). Unit-tested in `tests/test_slam.py` (voxel dedup, pose
  registration, PLY round-trip, engine 3-D build).

### Fixed — native LiDAR SLAM mapping (mapping only, not navigation)
- **SLAM now yaw/tilt-corrects the cloud, like the row-follow stack.** The 2-D
  SLAM mapper (`scripts/slam_mapper.py`, `slam/`) was slicing the **raw**
  sensor-frame cloud — it never applied the 66° yaw / 21.5° nose-down tilt
  correction. With the tilt uncorrected, flat ground 6 m ahead reads h≈1.4 m and
  falls *inside* the `[0.20, 1.50]` slice band, so the 2-D slice filled with
  ramped ground that moves with the robot — degrading ICP and smearing the map
  (and the deskew, which assumes +Y = forward, ran along the wrong axis at the
  66° yaw). `slam/scan_matcher.py::correct_scan` now applies yaw→tilt before the
  slice; `SlamEngine` and `slam_mapper.py` expose `--lidar-yaw` / `--lidar-tilt`
  (defaults matching the row-follow calibration). The slice band now defaults to
  h ∈ [0.05, 1.50] m so crop rows are captured as map structure (the dominant
  feature in open fields); `--slice-min 0.20` gives a structure-only map. New
  `tests/test_slam.py` (correction round-trip, ground-rejection regression, ICP,
  occupancy grid, engine smoke tests).

### Changed — IMU-measured U-turn (replaces perception-only arc)
- **The U-turn now measures real rotation with the IMU.** The perception-only
  arc still failed in the field: the open-loop arc under-rotated (slip makes the
  real radius ~1.7× the commanded 1.0 m, so "done" was only ~90° of rotation),
  and the dual-row detector locked onto **grass** at the headland (conf ~0.65),
  ending the turn at ~90° on the wrong feature — the robot then "followed" grass
  where the soybean rows weren't. Root cause across every attempt: nothing
  reliably measured how far the robot had actually rotated (wheel heading
  over-reports ~2×; the arc under-rotates; perception gets fooled by grass).
  Fix: `navigation/headland.py` accumulates the **`FilterState` (IMU) heading
  change** during the arc — using the *relative* change, so it works even when
  `has_converged` is False (no RTK lock, the normal field case; the old code
  wrongly required convergence and fell back to wheel heading). 
  `row_navigator._step_headland` keeps arcing until ~180° of REAL rotation; a
  perception lock may end the turn ONLY after ≥ `reacquire_min_turn` (150°) of
  measured rotation AND at conf ≥ `--reacquire-conf` (raised 0.55 → 0.72) — so a
  confident grass detection at ~90° can no longer end it; it completes on heading
  alone at 175° → APPROACH. Falls back to a 4 m arc-length window + strict
  perception if the IMU isn't live; the `max_turn_frac` arc cap still STOPS at a
  field edge. Simulated at 55 % wheel slip with grass present the whole arc, the
  turn ends at a true ~170° on the real row. Status:
  `R-UTURN:ARC rot=120°[imu] arc=2.1m reacq=2/4`. Unit-tested
  (`test_headland_odometry.py` imu rotation tracking/unwrap/stale; navigator
  gating via the same thresholds).

### Changed — perception-closed U-turn (superseded by the IMU-measured turn above)
- **The U-turn no longer trusts wheel heading to know when it's done.** The
  previous arc still closed on wheel-odometry heading and still under-rotated in
  the field (~90° instead of 180°): a 4-wheel skid-steer scrubs enough even on a
  moderate-radius arc that "180° of wheel heading" ≈ 90° of real rotation, so
  the robot finished pointing across the rows and drove off the field, then got
  stuck in APPROACH. `navigation/headland.py` is rewritten to be
  **perception-closed**: it drives a gentle large-radius arc (auto 1.0 m) and
  `row_navigator` ends the turn the instant the dual-row detector reports the
  next row **confidently aligned and roughly centred ahead**
  (`state_logic.turn_reacquired`, held `reacquire_frames` scans, after a
  `min_turn_frac` minimum arc), handing straight to FOLLOW. Odometry is used
  ONLY for arc-LENGTH guards (forward distance is reliable; heading is not), and
  a `max_turn_frac` cap STOPS the robot if no row ever appears (field edge).
  Simulated against a 2× wheel-heading over-report, the turn still ends at a true
  ~175°. New `--reacquire-conf`; `--headland-radius` default → 1.0 m;
  `--headland-turn-rate` default → 0.30. Unit-tested (`test_state_logic.py`
  turn_reacquired; `test_headland_odometry.py` arc/ready/finish/cap).

### Changed — arc U-turn (superseded by the perception-closed turn above)
- **The U-turn now drives a smooth maximum-radius arc instead of two in-place
  pivots.** Field failure: the robot ran all four pivot phases on wheel heading
  yet physically rotated only ~90°, ended up pointing across the rows, and drove
  off the field. Root cause: a 4-wheel skid-steer **scrubs** when pivoting in
  place, so the wheel-derived `measured_angular_rate` badly over-reports the
  body rotation — each "90°" pivot finished ~45°. A rolling **arc** scrubs far
  less, so the heading-closed turn reaches a true 180°. The largest radius that
  still lands on the next strip is a **semicircle of radius = shift/2**
  (`navigation/headland.py` rewritten: phases `EXIT → ARC → DONE`). New
  `--headland-radius` overrides the auto radius; `--headland-turn-rate` is now
  the arc's angular rate (arc forward speed = radius × rate). Re-tested in
  `tests/test_headland_odometry.py` (arc geometry, lateral displacement = shift,
  moving-not-pivot command, filter-heading path).

### Added — turn safety & robustness
- **Hard guard against driving off the field.** A cumulative `post_turn_max_dist`
  (default 5.0 m, `--post-turn-max-dist`) bounds the total distance travelled
  after a U-turn — APPROACH creep plus any short, un-settled FOLLOW segments —
  before a stable down-row FOLLOW is established. If the budget is spent without
  settling, the robot STOPS rather than hunting off the end of the field.
- **Row-end blind-spot confirmation.** The VLP-16 is blind inside ~1.5 m, so a
  brief sparse patch or the last plants in the near zone can read as a row end.
  The turn's straight EXIT leg now doubles as a confirmation: if solid crop
  reappears in the ROI during EXIT (before any rotation), the row had not
  actually ended — the turn **aborts back to FOLLOW** (`_step_headland`,
  `headland_abort_frames`). This is the "take a little extra distance unless
  obstacle" check; the safety monitor still pauses the leg on a real obstacle.

### Changed
- **LiDAR yaw re-calibrated 71° → 66°** after a mount disturbance (lens
  cleaning rotated the sensor ~5°). Measured with the new `diag_birdseye.py`
  forward-object locator (a straight-ahead bucket read +5.4° at yaw 71, 0.0° at
  yaw 66). Pitch independently re-confirmed at 21.5°. `--lidar-yaw` default
  updated in `row_follow.py` / `diag_birdseye.py`.
- **`diag_birdseye.py` forward-object locator** added: reports the azimuth of
  elevated forward objects so the mount yaw can be re-verified to ~1° any time
  the LiDAR is touched (`--range 3`, read the straight-ahead azimuth).

### Fixed (cont.)
- **Stuck in ACQUIRE right after the U-turn — "turned … then went to follow
  and acquire" and hung.** The U-turn → APPROACH → FOLLOW handoff lands on the
  next row with a marginal, partly-in-ROI view (field: n≈70 vs ≈700 on a
  well-centred row, confidence flickering around the 0.35 threshold). FOLLOW
  then dropped to a *stationary* ACQUIRE after ~0.3 m — and a stationary sensor
  on a sparse, half-visible row cannot improve its view, so it hung at the row
  start. New **post-turn settling window** (`state_logic.post_turn_loss_action`):
  from the end of the turn until the robot has driven `post_turn_settle_dist`
  (2.0 m) of continuous FOLLOW down the new row, a non-row-end FOLLOW loss
  re-enters **APPROACH** (keeps creeping forward) instead of stalling in
  ACQUIRE — the moving sensor fills the ROI and re-locks. Still bounded by
  `--approach-max-dist`, so a genuinely missing row stops rather than driving
  off the field. Status shows `APPROACH … SETTLE`. Unit-tested.
- **FOLLOW→ACQUIRE hang at the real row end.** At a row end with residual sparse
  clutter (~40 straggler/weed returns flickering in the ROI) `row_end_confidence`
  never crossed 0.70, and/or a short row left `row_dist` under `row_end_min_dist`,
  so the FOLLOW-exit check fell to the ACQUIRE branch — which had no row-end
  escape and hung forever hunting for a row that isn't there. New **ACQUIRE
  row-end escape** (`state_logic.acquire_rowend_escape`): when ACQUIRE was entered
  *from FOLLOW* and the crop band is empty for `row_end_frames` consecutive scans,
  go to ROW_END → headland. Unit-tested.

### Added
- **APPROACH leg closes the row-to-row loop.** The U-turn ends at the headland
  margin pointing down the next row but with no crop in the ROI yet; the old
  code entered a stationary ACQUIRE there and hung forever (field failure:
  "nothing in front, goes to ACQUIRE and stuck"). New `APPROACH` state creeps
  forward (`--approach-speed`) into the next row until perception re-locks, then
  hands to FOLLOW; stops after `--approach-max-dist` if no row is found (field
  edge / overshoot). Decision in `state_logic.approach_action`, unit-tested.
- **Filter (IMU/GPS) heading for the pivots.** `navigation/filter_heading.py`
  subscribes to the filter service (`FilterState`, port 20001) and exposes the
  fused absolute heading. `HeadlandTurn` now closes the two 90° pivots on this
  heading when it is fresh and converged — robust to the wheel slip that an
  in-place skid-steer pivot causes — and falls back to wheel-odometry heading
  otherwise. The source is latched per turn and shown in the status line
  (`R-UTURN:TURN_A[filter]`/`[wheel]`).
- **`--headland-shift` (default 1.52 m):** the centre-to-centre SHIFT distance
  to the next strip the robot straddles, made distinct from `--row-spacing`
  (0.76 m, the in-strip soybean-row separation used by the detector).

### Fixed
- **Row end never reached the headland turn**, then **spurious turns on slopes.**
  Crop running out trips both the row-end signal and the low-confidence miss
  counter at once. First fix routed the miss to `ROW_END` when a real row had
  been driven and the band ahead was empty — but a 4-frame trigger then misread
  a brief crop **dropout on a slope** (crop flickering 0↔700) as a row end and
  started a headland turn mid-row. Final logic (`navigation/state_logic.py
  follow_loss_action`): a real row end requires a *long continuous* crop absence
  (`row_end_frames`, raised 8→15) — the miss counter resets the instant crop
  reappears, so an intermittent slope dropout can never accumulate it; a
  non-row-end loss re-acquires after `follow_miss_thresh`; otherwise wait.
  Unit-tested incl. the slope-dropout regression (`action(4, True) == "WAIT"`).

### Notes
- Logic is unit-tested (filter-vs-wheel selection, latching, distinct shift
  distance, row-end-vs-loss decision); the full turn still needs field
  validation (pivot accuracy, re-acquisition of the next row). 98 tests pass.

## [v0.1.0] — 2026-06-23 — First field-validated row-follow baseline

First version that autonomously follows a soybean centre-residue row in the
field: drives the strip centred, with stable heading, and survives real
(non-flat) terrain. Tagged as a known-good checkpoint to build on.

### Fixed — perception/geometry root causes
- **LiDAR correction order.** Apply yaw **then** tilt (was tilt→yaw). The two
  do not commute at the 71° mount yaw; the wrong order left a >1 m ground ramp
  that biased the heading PCA and drove the robot off-row. Regression-locked.
- **Pitch calibration.** Field-swept the true robot-frame pitch to **21.5°**
  (the eyeballed 15° was body lean) via `diag_birdseye.py --tilt-sweep`. Set as
  the default with yaw **71°** so field runs need no flags. `LIDAR_MOUNT_HEIGHT`
  confirmed correct at 0.75 m (tape).
- **ROS 2 bridge TF.** Publish `base_link→velodyne` as identity rotation — the
  cloud is already yaw/tilt-corrected upstream, so the old −15° pitch in the TF
  re-tilted an already-flat cloud in the live 3-D view.

### Fixed — connectivity
- **Canbus host.** `service_config.json` pointed at the Tailscale FQDN while the
  process runs on the brain itself, so `/twist` looped out through Tailscale and
  intermittently timed out. Use `localhost` for the on-brain canbus/filter
  services. Added a retrying handshake with a longer first-call timeout.

### Added — field robustness (terrain-adaptive crop band)
- **Grade detrend.** Estimate the forward ground slope per scan (low percentile
  of z per range-bin, slope-only → robust to furrow/soil bimodality) and remove
  it, so crop stays inside the height band on sloped fields. No-op on flat /
  canopy-only data.
- **Level shift on dips.** When a real ground+canopy column sits below the
  flat-calibrated `h=0` (terrain dip), lower the crop band onto the local ground
  so the (correctly elevated) canopy is not clipped from below. Gated on height
  spread; only ever lowers the band.
- Live `grade=±N°` / `drop=−N.Nm` shown in the status line.

### Added — tooling
- `diag_birdseye.py`: `--tilt-sweep LO:HI:STEP` to calibrate pitch, and a
  detector-ROI height profile printout for diagnosing marginal spots.
- `--no-ground-detrend` escape hatch on `row_follow.py`.

### Notes / not yet validated
- Headland U-turn (`--headland`) and multi-row serpentine exist but are **not
  yet field-validated** — next milestone.
- 89 unit tests pass (`python3 -m pytest tests/ -q`).
