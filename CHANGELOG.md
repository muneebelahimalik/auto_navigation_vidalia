# Changelog

All notable changes to this project are documented here.
Format loosely follows [Keep a Changelog](https://keepachangelog.com/);
versions are git tags on `main`.

## [Unreleased] — headland U-turn reliability (not yet field-validated)

Work toward the row-to-row turn milestone (built on the v0.1.0 baseline).

### Fixed — ROOT CAUSE: LiDAR is a VLP-16 Puck Hi-Res, not a standard VLP-16
- The driver's `VLP16_VERTICAL_ANGLES` used the standard Puck's ±15° / 2.00°
  channel table, but the packet product-ID byte is `0x24` = **Puck Hi-Res**
  (±10° / 1.33°).  Wrong (wider) angles stretch the reconstructed vertical
  geometry, so the real **15° nose-down / 0.80 m** mount reconstructed as a
  phantom **~22° / 1.17 m** — which made the tilt sweep, box test, and ring-fit
  all agree on wrong values while the tape (0.80 m) and phone level (~15°)
  disagreed.  **The field measurements were right; the driver table was wrong.**
  Fixed: `VLP16_VERTICAL_ANGLES` → Hi-Res table.  On-robot verification: with the
  correct angles the per-channel pitch collapses from a 9–19° fan to 14.6°±0.1°
  and the raw-geometry fit returns 14.8° / 0.81 m.  Re-derived calibration:
  `--lidar-tilt` 21.7 → **15.0**, `--lidar-yaw` **0**, `LIDAR_MOUNT_HEIGHT`
  **0.80** (floor now reads h≈0, no −0.37 m offset).  Propagated through
  `slam_engine`/`slam_mapper` defaults, the ROS bridge + dev-PC TF launch files,
  `viz_perception`, README and CLAUDE.md.  Regression-locked in
  `tests/test_lidar_driver.py`.
- New diagnostics: `scripts/diag_sensor_model.py` (reads the packet product-ID),
  `scripts/diag_pitch_rings.py` (solves pitch AND height from raw ring geometry,
  no correction code in the loop — the tool that cracked this), and
  `scripts/diag_filter_imu.py` (probes the filter/OAK IMU for pitch/roll).

### Added — row-follow robustness (strip-lock, longer-line row-end, roll detrend)
- **Strip-lock** default raised 1.5 → 2.5 and exposed as `--strip-lock W`: a
  large lateral correction can no longer alias onto the adjacent strip and commit
  the robot to the next row.
- **Row-end on the longer line** (dual-row): row-end keys off the STRONGER
  flanking row's mass, so a row only ends once BOTH soybean lines are gone —
  the robot follows whichever line runs longer to its true end.
- **LiDAR-only lateral (roll) ground detrend**: extends the forward-slope
  detrend with `dz/dx` from a wide cross-row swath, so a cross-slope no longer
  makes one flanking row read higher than the other or leaks ground into the
  crop band.  No IMU/cameras; no-op on flat ground.  Shown as `roll=` in status /
  `roll_deg` in telemetry.  Tests in `tests/test_row_perception.py`.

### Fixed — grade heading-runaway guard back ON; figure colormap crash
- A field RL run tracked well for ~9 m, then on a +6° grade the heading estimate
  ran away exactly as before (0 → −38° → +34° with |lateral| ≈ 0), the robot
  turned ~75° and stuck in ACQUIRE.  The proportional heading-consistency clamp
  prevents this, but it had been disabled.  **Re-enabled by default.**  The
  proportional cap (unlike the earlier flat cap) is INERT in normal following —
  small heading + small lateral is well under the near-centre cap, so it does
  not shift the RL policy's input — and only bites on the pathological grade
  spike (large heading while centred), capping it toward ~7° so no controller
  can be driven into the runaway.  It is a SAFETY NET; the real fix is at the
  PCA source (why heading spikes on a grade) and needs raw `scans/` from a
  sloped run.  Test updated.
- Perception figures failed to render on the brain with "Colormap turbo is not
  recognized" (older Jetson matplotlib).  `viz_perception` now picks `turbo`
  only when present and falls back to `viridis`, so the figures render on the
  brain again.

### Changed — scan recorder saves from the very first scan; RL default policy
- `ScanRecorder` now always saves the **first non-empty scan** (data from the
  very start of the run), then every ``every``-th scan after it — so a figure can
  be rendered from t≈0 offline, not only from the first downsample multiple.
  Robust to leading empty scans while the sensor spins up.  Tests updated.
- `--controller rl` with no `--policy` now defaults to
  `policies/follow_jerk8.0.npz` (the smoothest sweep policy — as smooth as MPC,
  tighter tracking), so RL works out of the box.

### Added — RL drift integrator + field-calibrated sim (cross-slope rejection)
- A field RL run drifted downhill and off the row on cross-slopes.  Root cause is
  structural: the policy observed only `[lateral, heading, conf, prev_action]`,
  and a memoryless reactive policy CANNOT cancel a steady disturbance (constant
  cross-slope drift → constant steady-state error).  Added a 5th observation — a
  **leaky drift integrator** (`rl_policy.update_eint`, the RL analogue of the MPC
  disturbance observer), maintained identically in the sim, the eval harness, and
  the deployed `RLController`, so the policy can learn a steady counter-steer.
  Network is now `[5→16→16→1]` (385 params); 4-input policies still load and run
  (integrator dropped) for backward compatibility.
- The sim's heading noise is now **grade-correlated, calibrated from field
  telemetry** (`theta_noise + theta_noise_grade_k·|grade|`): flat ~1.9° → sloped
  ~5.2° std, matching the measured flat-vs-graded FOLLOW heading spread.  Training
  against the disturbance that actually caused the field weave teaches the policy
  to distrust noisy heading and lean on the lateral offset.
- Retrained `policies/follow.npz`.  Held-out (200 episodes): cross-track RMSE
  **21.4 → 8.5 cm**, success **92 → 99.5 %**; on the steepest cross-slopes
  pure-pursuit collapses to **7 %** where the integrator policy holds at **93 %**
  (edges MPC on tracking, 8.5 vs 14.2 cm; MPC still smoother).  `RLController`
  gained a `reset()` (drift integrator + prev-action), now called on a new row
  after a U-turn.  Note: results are IN SIM — the sim-to-real gap is the headline
  risk, so field trials keep pure-pursuit armed and start perception-only.
  Regression-locked in `tests/test_rl.py` (integrator, 5-dim obs, backward compat).
- Retrained the jerk-sweep variants (`follow_jerk{1.0,3.0,8.0}.npz`) on the new
  5-input architecture and regenerated `results/controller_pareto.csv` (new
  reproducible generator `scripts/gen_controller_pareto.py`, 200 held-out
  episodes, same seeds for every controller).  The smoothness dial is clean:
  raising the jerk penalty `--c-du` 0.3 → 8.0 **halves control jerk
  (0.099 → 0.051) with negligible tracking cost** (xtrack 8.5 → 8.9 cm, success
  99.5 %); `rl_jerk8.0` is as smooth as MPC (0.051 vs 0.054) but tracks tighter
  (8.9 vs 14.2 cm).  Pick the knee for the field.

### Reverted — heading consistency clamp is now OFF by default
- The heading–lateral consistency clamp (below) is **disabled by default**
  (`heading_consistency_lat = 0.0`).  It manipulates the exact signal the learned
  RL policy was trained on unclamped (a distribution shift that can make RL steer
  unpredictably) and near centre it suppresses a genuine heading correction,
  which can add its own weave — the operator reported steering got WORSE, not
  better, with it active, relative to the strip-lock-prior baseline they judged
  good.  The code path is retained behind the param (set `> 0` to re-enable for
  testing), but the default steering path is now exactly the strip-lock baseline.
  The correct fix for the terrain-grade heading artifact is at the PCA source
  (needs raw `scans/` from a sloped run), not a controller-input clamp.

### Added (default-off) — heading consistency clamp on terrain grade
- Field RL run: on terrain grade spikes (`grade +7…+9°`, `drop −0.2 m`) the PCA
  heading estimate ran away (0 → −57°) **while the lateral offset stayed ±0.05 m**
  — geometrically impossible for a real orientation (a robot that angled would
  pile up lateral offset), so it was a terrain/PCA artifact.  The heading-dominant
  pure-pursuit (lookahead 2.0 m) chased it into a hard turn; the robot physically
  rotated ~45°, lost the row, re-acquired **on the diagonal** and got stuck
  crawling across the rows.  Added a **heading–lateral consistency clamp** in
  `RowDetector._smooth`: when the robot is demonstrably still centred
  (`|lateral| < heading_consistency_lat`, 0.22 m) the reported heading magnitude
  is capped (`heading_consistency_cap`, 22°) — ample for real straight-row
  corrections, but it removes the artifact the controller was amplifying, so the
  runaway can't start.  **The cap is proportional to the lateral offset**
  (`heading_cap_centred` 7° at lateral 0 → `heading_cap_gate` 22° at
  `heading_consistency_lat` 0.22 m, unclamped beyond): a centred robot on a
  straight row has ~0 real heading error, so a big reading there is noise and is
  pulled hard toward 0; a genuinely off-strip robot keeps the authority it needs
  to steer back.  A second field run (RL) showed the earlier FLAT 22° cap still
  left a capped-but-large ~20° heading driving a sustained left turn on a centred
  straight row — the proportional cap halves the heading exactly in that weave
  region (24 of 497 scans, artifact-only) while leaving normal following
  untouched.  Regression-locked in `test_large_heading_when_centred_is_clamped_hard`
  / `_cap_scales_with_lateral_offset` / `_large_lateral_is_not_clamped` /
  `_normal_small_heading_unchanged`.
- The **early** perception render now decimates the cloud used for the figures
  (the full cloud is still saved to `perception_scan.npy`), so the background
  render can't hog Jetson CPU and stutter the control loop; the exit-time render
  still uses the full cloud.

### Changed — perception figures now render early, not at exit
- The three perception figures (and `perception_scan.npy` / `perception_state.json`)
  are now written **a few scans into the run** — once a confident FOLLOW lock has
  settled (`perception_capture_after`, default 5 qualifying scans) a one-shot
  **background daemon thread** saves the scan and renders the figures off the
  control loop (matplotlib never blocks driving).  Previously this only happened
  at exit, so a crash mid-run lost the figures and you had to wait for the whole
  run to finish.  An exit-time render remains as a fallback only if the run never
  reached a confident lock.  Regression-locked by a background-thread render test
  in `tests/test_viz_perception.py`.

### Fixed — row distance wiped on every obstacle pause (grown-crop field)
- Field telemetry (grown soybean) showed the forward safety zone tripping on
  the now-taller crop ~1.7 m ahead ~20 times in one row, each time cycling
  FOLLOW → OBSTACLE_WAIT → ACQUIRE → FOLLOW — and the ACQUIRE → FOLLOW
  transition reset `_row_dist` to 0 every time, so the robot never accumulated
  down-row distance (`rows_done` stuck at 0 for 10 min) and row-end / headland
  timing became unreliable.  The reset was wrong: the ONLY paths into ACQUIRE
  are mid-row recoveries (a lost lock, or an obstacle clearing) on the *same*
  row — the robot has not returned to the row start.  A genuinely new row after
  a U-turn is reached via HEADLAND/APPROACH → FOLLOW, which reset `_row_dist`
  explicitly; at boot it is already 0.  Removed the erroneous reset so row
  progress survives transient obstacle pauses (including the operator taking
  manual control to drive past an obstacle and handing back).

### Fixed — headland turn could hang on grown crop ahead
- Same root cause: during the U-turn arc the robot sweeps to face the next row
  block, and grown crop directly ahead tripped the forward obstacle zone, which
  pauses the turn in place — in the field log this wedged the turn for 24 s
  until the crop happened to fall out of the zone.  The fix is operational
  (raise `--obstacle-height` above the grown crop, e.g. 0.70 m, so crop is never
  read as an obstacle in any state) plus the row-distance fix above; documented
  in the grown-crop guidance.

### Added — `--no-slam` to skip SLAM under `--record`
- `--record` still bundles telemetry + metrics + perception figures + raw
  scans, but `--no-slam` now suppresses the SLAM map / coverage / trajectory
  outputs for runs where field coverage assurance isn't wanted.

### Added — raw-scan streaming for `--record` (`--save-scans`)
- A `--record` run now keeps the **raw corrected point-cloud time series**, not
  just one representative scan.  `navigation/scan_recorder.py` (`ScanRecorder`)
  streams each corrected robot-frame scan to `runs/<ts>/scans/scan_NNNNN.npy`
  plus an `index.csv` (file, t, state, lateral, heading_deg, conf, n), so ANY
  moment of the run can be re-rendered offline into perception figures or
  animations later — including on a machine that has matplotlib when the brain
  does not.  The save runs in a **background daemon thread** fed by an O(1),
  non-blocking `submit()` from the control loop (bounded queue, drops rather
  than blocks if the disk can't keep up), so recording can never slow or
  destabilise driving.  On by default with `--record` at every 10th scan
  (≈ 1 Hz); `--save-scans` tunes it (bare = every scan / full fidelity, `N` =
  every Nth, `0` = off).  Regression-locked in `tests/test_scan_recorder.py`.
  Combined with the continuously-written `telemetry.jsonl`, all raw data is now
  preserved regardless of whether the figures render at exit.

### Fixed — row-hopping during corrections (all controllers; worst under RL)
- In a periodic soybean field the dual-row midpoint is ambiguous: "on my strip,
  offset +half" and "on the next strip, offset −half" produce two peak pairs that
  both match the row spacing.  `find_row_midpoint` minimised spacing error only,
  so it could pick the adjacent strip arbitrarily; the controller then centred on
  it and the robot drifted onto the next strip / crossed rows (and RL, reacting
  fastest, hopped most).  Added a **continuity ("strip-lock") prior**: the pair
  cost is now `|sep − spacing| + prior_weight·|midpoint − tracked_lateral|`, so
  the detector stays locked to the strip it is following — a jump to the adjacent
  strip costs ≈ one row spacing and is rejected unless the robot has genuinely
  crossed the half-way point.  `RowDetector.midpoint_prior_weight = 1.5`;
  `prior_weight = 0` preserves the old behaviour for the camera tracker; on
  `reset()` the prior is 0 so the post-turn re-acquire prefers the nearest
  (centred) strip.  Also cleans the RL/MPC input (no more jumped target to
  chase).  Regression-locked in `test_strip_lock_resists_row_hop` /
  `test_strip_lock_off_by_default_in_find_row_midpoint`.


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

### Added — perception figure for poster/paper (`scripts/viz_perception.py`)
- Renders a professional two-panel figure of what the LiDAR perception sees in
  the robot frame (yaw+tilt corrected): (a) an annotated bird's-eye view —
  height-coloured points with the forward-travel direction, the two soybean rows
  flanking the residue strip, the detected strip-centre (tracking target) +
  heading, the pure-pursuit look-ahead, the cross-track offset, the detection
  ROI, the self-filter blind zone, the robot footprint, and the three safety
  zones; and (b) a 3-D view with the crop rows standing above the ground plane.
  Renders a REAL captured scan (`--scan file.npy/.ply`), or a geometry-faithful
  scene seeded from a run's telemetry (`--telemetry … --index N`) / the field
  calibration.  Outputs 300-dpi PNG + vector SVG.  Desktop/figure tool
  (matplotlib); not on the brain's control path, never imported by the nav stack.
- `--mode av3d` adds a self-driving-car-style dense 3-D point-cloud render: a
  ray-cast VLP-16 scan (16 channels × azimuth, 21.5° nose-down, ground + crop
  ridges) producing the characteristic Velodyne ground rings bending up over the
  crop rows, on a dark scene with the ego robot and the decision overlays
  (planned strip-centre path, look-ahead target, ROI corridor, forward safety
  zone), height-coloured (turbo).
- `--mode av_bev` adds the matching top-down "sensor scope": the LiDAR rings from
  above with range rings (2/4/6/8 m) and the decision overlays (planned path,
  look-ahead, ROI, safety zones, ego robot) on the same dark scene — the classic
  AV-dashboard inset to pair with the 3-D hero shot.
- **`--record` now auto-generates the perception figures from real field data.**
  During a recorded run the navigator snapshots the densest high-confidence
  FOLLOW scan (the corrected robot-frame cloud + detected strip-centre geometry);
  on exit `row_follow.py` saves `perception_scan.npy` + `perception_state.json`
  and renders `figure_perception_3d.png`, `figure_perception_bev.png`, and
  `figure_perception_annotated.png/.svg` into the run folder via
  `scripts/viz_perception.render_all`.  The raw scan is always saved; if
  matplotlib is not installed on the brain the figures are skipped with a
  one-line offline-render command (control path stays matplotlib-free).
  `tests/test_viz_perception.py` covers the ray-cast scene + render bundle.

### Added — `--record`: complete reproducible experiment folder per run
- One flag captures everything needed to analyse and publish from a field run
  into `runs/run_<ts>/`: `manifest.json` (git commit + dirty flag + full CLI
  args + calibration — the reproducibility anchor), `telemetry.jsonl` (per-scan,
  now incl. the live row-spacing `sp`), and on exit a computed `summary.json` +
  `metrics_flat.csv` + `per_row.csv` + `turns.csv`, plus the SLAM
  `coverage.png`/`trajectory.csv`/`map.png`.  `--record` turns on `--telemetry`
  + `--slam` and writes the manifest at start (a crashed run is still
  identifiable).
- `navigation/run_metrics.py` (numpy-only, runs on the brain) computes the
  publication metrics from the telemetry: FOLLOW cross-track RMSE/MAE/p95/max,
  heading RMSE/MAE/max, control effort + jerk, speed, state-time budget, event
  counts (obstacle stops, FOLLOW losses, dropout scans), terrain grade/drop,
  per-row breakdown, and per-turn outcomes (rotation °, arc, `imu`/`wheel`
  source, completed).  `navigation/run_record.py` writes the folder.
- `scripts/analyze_run.py` (re)computes the tables offline and aggregates
  several runs into one comparison CSV — e.g. the field `pursuit` vs `mpc` A/B,
  the counterpart to the sim `results/controller_pareto.csv`.  Unit-tested in
  `tests/test_run_metrics.py` (tracking metrics, per-row, turn extraction, state
  budget/events, torn-line robustness, on-disk bundle).

### Fixed — U-turn completes near 180° on the live IMU (was ending ~30° short)
- With the filter-heading subscriber fixed, the U-turn now runs on the real
  gyro (`rot=…[imu]`).  That exposed that the rotation gates were tuned for the
  wheel regime: `reacquire_min_turn=150°` and `turn_complete=175°` were chosen
  when 150° *displayed* ≈ 110° *physical*.  In TRUE IMU degrees those let a
  perception lock end the turn at ~152° real — 28° short — so the robot entered
  FOLLOW angled and drove across the rows (heading climbing past +29°).  Raised
  the gates for the IMU regime: `reacquire_min_turn_deg` 150 → **170**,
  `turn_complete_deg` 175 → **178**, so the turn comes most of the way around
  before perception (or heading alone) ends it.  179 tests pass.

### Changed — U-turn scrub default lowered (field calibration)
- **`--turn-scrub-comp` default 0.6 → 0.5.** When the IMU/filter heading is not
  publishing (`rot=…[wheel]`), the turn estimates real rotation as wheel-heading ×
  scrub_comp; too high a value over-reports rotation, so the "turned-enough" gate
  opens early and a perception lock ends the turn under-rotated (robot then drives
  diagonally across the rows).  A field run ended at a real ~120° while `rot`
  displayed 159° (wheels reported 265°), i.e. true scrub ≈ 0.45 — 0.6 was too
  high.  Lowered the default and clarified the calibration (when the robot truly
  completes 180°, `rot` should read ~180; heavy-slip fields ~0.45).  This is a
  per-surface calibration; the real fix is a live gyro (filter `FilterState` or an
  IMU source) so the turn doesn't depend on the scrub guess at all.

### Changed — fewer geometry parameters (sensor/perception-driven)
- **Removed `--headland-shift` (dead parameter).** The U-turn has been fully
  perception-closed for a while — it arcs until the LiDAR re-acquires the next
  row aligned ahead (gated by measured rotation), so no strip-spacing value
  sizes it.  `--headland-shift` was stored on `HeadlandTurn` but never read by
  any turn logic; it is gone from the CLI, `RowNavigator`, the camera navigator
  and `HeadlandTurn`.  Set rows with `--rows N --headland`; the sensors decide
  where the next row is.
- **`--row-spacing` is now a self-calibrating SEED, not a fixed geometry.**
  Clarified that it is a detector PRIOR (it pairs the two flanking crop rows
  apart from weed clutter and sets the single-side fallback offset), never a
  movement/turn parameter.  `RowDetector` now refines it online from the
  observed left/right peak separation (slow EMA, outlier-gated to plausible crop
  spacing), so it converges to the field's ACTUAL spacing from the default seed
  — the operator normally only sets `--rows`.  The live estimate shows as
  `sp=N.NNm` in the status line and survives U-turn `reset()`s.  Regression
  tests: converges from a wrong seed, still centres, persists across reset.

### Added — field COVERAGE map (the operational use case for SLAM)
- **Coverage assurance from the drift-corrected SLAM pose** — the answer to "is
  SLAM even useful when we already have point clouds?". A single scan is local
  and instantaneous; what SLAM adds is the drift-corrected trajectory (scan-
  matching + loop closure keep it metric where raw odometry smears after a few
  rows). `slam/coverage_map.py` (`CoverageGrid`) stamps the robot's working
  swath (`--swath`, default 1.92 m = wheel track) along that path into a
  serviced-ground mask. On save (`slam_mapper.py` and `row_follow.py --slam`):
  `coverage.png` (serviced swath in green over the structure map — **gaps are
  missed rows**), `trajectory.csv` (as-driven `scan,x,y,heading`), the covered
  mask + stats in `map.npz`, and a printed report of serviced area, path length
  and a **redundancy** ratio (swept ÷ unique area; 1.0 = no overlap, >1 =
  double-driven). On by default whenever SLAM runs; `SlamState` gains
  `covered_area_m2` / `path_length_m` / `coverage_redundancy` and the status line
  shows `cov=NNNm²`. This is the precision-ag deliverable (did the robot cover
  the field, and how well?) and the natural controller A/B (drive the same field
  with `pursuit` vs `mpc`, overlay trajectories, compare coverage/redundancy).
  Unit-tested in `tests/test_coverage.py` (swath geometry, gap detection, jump
  interpolation, redundancy, save-file round-trip, back-compat).

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

### Added — optional learned (RL) steering, as an opt-in feature
- **Reinforcement-learning steering policy for FOLLOW** — a bounded refinement on
  top of pure-pursuit, never a replacement for the safety architecture. Scope is
  deliberately narrow and honest: it learns ONLY the in-row angular command;
  the state machine, safety monitor and headland turn are untouched, forward
  speed stays the pure-pursuit formula, and `RLController` falls back to
  pure-pursuit below `min_confidence` or when no policy is loaded (a missing/bad
  policy can never disable the robot). Output is hard-clamped to `max_angular`.
  - `sim/row_follow_env.py`: a numpy Gym-style simulator of the steering task
    with the realism that makes a learned policy plausibly useful — skid-steer
    slip, per-episode cross-slope drift, sensor noise + dropout. Forward speed
    uses the real controller's formula so RL and pure-pursuit are compared on
    identical dynamics.
  - `navigation/rl_policy.py`: tiny numpy MLP (no torch/gym) — μs forward pass,
    deploys on the Jetson with zero new deps; params flatten for gradient-free
    training; saves to `.npz`.
  - `navigation/rl_controller.py`: drop-in for `PurePursuitController`.
  - `scripts/train_rl.py`: Evolution Strategies trainer (Salimans et al. 2017 —
    a legitimate, numpy-only, gradient-free RL method); reports the pure-pursuit
    baseline throughout for an honest comparison.
  - `scripts/eval_controller.py`: held-out comparison (same per-seed
    disturbances) with a cross-slope-severity breakdown — the "does RL help?" test.
  - `row_follow.py --controller rl --policy <file>` (default stays `pursuit`).
  - Unit-tested (env determinism/termination, policy bounds/round-trip,
    controller fallback/clamp, ES optimiser). Caveat documented: results are
    in-sim; sim-to-real transfer must be field-validated with the pure-pursuit
    fallback armed.

### Added — optional model-predictive (MPC/MPPI) steering, as an opt-in feature
- **Sampling-based MPC steering for FOLLOW** (`--controller mpc`) — a
  state-of-the-art upgrade over the memoryless pure-pursuit law, added on TOP of
  it (default stays `pursuit`; the current controller runs unchanged).
  `navigation/row_mpc_controller.py` implements **MPPI** (Model-Predictive Path
  Integral control; Williams et al. 2017): each step it samples K candidate
  steering sequences over an H-step horizon, rolls each through a kinematic
  row-tracking model, and returns the cost-weighted (soft-argmin) first action —
  gradient-free, numpy-only, deploys on the Jetson with zero new deps.
  - **Online disturbance observer** — the key field win. A scalar Luenberger
    estimate of the persistent cross-slope drift (slope/mis-calibration) updates
    from the predicted-vs-measured lateral residual; MPPI previews WITH it, so it
    actively *cancels* the slope drift pure-pursuit can only chase (the integral
    action a geometric law lacks). A gentle slip estimate does the same for
    skid-steer under-rotation.
  - Same safety contract as the RL controller: steering only (speed stays the
    pure-pursuit formula), pure-pursuit fallback below `min_confidence`/invalid
    fix, hard `max_angular` clamp, `reset()` per row.
  - Held-out sim (200 episodes): cross-track RMSE 22.5 → 15.7 cm, success 87 →
    96 %, and on the steepest cross-slopes pure-pursuit's success collapses
    (0 %) where MPC holds (100 %) — exactly the disturbance-rejection benefit.
    Better heading behaviour than the RL policy (6.6° vs ~10° RMSE) with no
    training required. `scripts/eval_controller.py --mpc` reports it with the
    same cross-slope breakdown; `results/controller_pareto.csv` has the full
    pursuit/RL-jerk-sweep/MPC comparison. Unit-tested in `tests/test_mpc.py`
    (fallback, clamp, steering sign, observer learns the drift sign, beats
    pursuit in closed-loop sim). Caveat: sim results — field-validate with the
    pure-pursuit fallback armed.

### Added — field telemetry + live mapping
- **Per-scan telemetry log (`--telemetry`).** `navigation/telemetry.py` writes one
  JSON line per scan — state, confidence, crop-point count, lateral/heading
  error, row-end confidence, terrain grade/drop, command output, the three
  safety zones, and the headland-turn state (phase, real rotation, arc length,
  source) — to `logs/run_<ts>.jsonl`. Load with `pandas.read_json(path,
  lines=True)`; this is the data behind the "unified dashboard". Logging swallows
  all errors and never affects control. Unit-tested.
- **SLAM mapping alongside row-follow (`--slam`, `--map-3d`).** Optional field
  mapping while driving, run in a separate thread fed by the navigator (one
  shared LiDAR reader; O(1) non-blocking handoff; pipeline errors swallowed) so
  it cannot affect control. Saves `map.png/npz` (+`map3d.ply`). `slam/slam_runner.py`.

### Changed — U-turn rotation from wheel heading when the IMU is offline
- **The filter/IMU heading isn't published in the test field** (the filter
  service needs a GPS fix it doesn't have), so the IMU-gated turn ran on its
  arc-length fallback and **under-rotated** (~135°), landing the robot angled
  across the rows. Fix: when the IMU heading isn't live, measure rotation from
  the **wheel-odometry heading** (`measured_angular_rate`, always available)
  scaled by `--turn-scrub-comp` (default 0.6) — it over-reports on a skid-steer
  arc, but unlike a fixed arc length it *responds to how much the robot is
  actually turning*, and the on-screen `rot=NNN[wheel]` makes `scrub_comp`
  a one-run calibration. The arc angular rate now also **eases in** (smoothstep
  over `ramp_dist`) so the skid-steer doesn't lurch. `--reacquire-conf` default
  raised to 0.72 (rejects the grass that fooled it at the headland). Status:
  `R-UTURN:ARC rot=NNN[imu|wheel] …`. Unit-tested (wheel fallback, IMU-preferred,
  arc ease-in).

### Changed — IMU-measured U-turn (wheel-heading fallback added above)
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
