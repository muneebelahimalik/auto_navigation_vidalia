# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Git Workflow — ALWAYS push to main

Whenever a change is completed (code, tests, or docs), **commit it and push it
to `main`** — the user has standing permission for this; do not wait to be
asked and do not leave finished work only on a side branch.  In remote Claude
Code sessions that develop on a `claude/*` session branch: push the session
branch first, then fast-forward `main` to it (`git checkout main && git merge
--ff-only <branch> && git push origin main`) and switch back.  Run
`python3 -m pytest tests/ -q` before pushing.

## Project Overview

Autonomous navigation workspace for the farm-ng Amiga robot using a Velodyne VLP-16 LiDAR
and two OAK-D stereo cameras. Goal: fully autonomous precision row following in soybean fields
(centre-residue strip tracking) and onion fields.

Three parallel stacks are maintained:

| Stack | Entry point | When to use |
|---|---|---|
| **Native Python — LiDAR** (recommended on brain) | `scripts/row_follow.py` | Amiga brain — primary stack, ROS 2 not required |
| **Native Python — Camera-only** | `scripts/cam_follow.py` | When LiDAR unavailable or for lightweight deployment |
| **ROS 2 Foxy Docker** (visualization) | `bash ros2_bridge/start.sh` | Live topic visualization via Foxglove browser UI |

---

## System — Amiga Brain (camphor-clone)

Full hardware and software configuration. Every architectural decision follows from these facts.

| Property | Value |
|---|---|
| **SoC** | NVIDIA Jetson Xavier NX |
| **OS** | Ubuntu 20.04.6 LTS (Focal Fossa) |
| **Architecture** | aarch64 (ARM64) |
| **Kernel** | Linux 5.10.104-tegra (L4T R35.2.1 — Jetson Linux) |
| **JetPack** | 5.1 |
| **Python** | 3.8.10 (system) |
| **Home directory** | `/mnt/managed_home/farm-ng-user-laserweeding/` (symlinked as `~/`) |
| **NVMe** | `/dev/nvme0n1p1` mounted at `/mnt` — 234 GB, **persistent across reboots** |
| **Shared memory** | `/dev/shm` — tmpfs, 7.3 GB, shared via Docker volume |
| **Docker storage** | `/var/lib/docker` on NVMe — **persists across reboots** |
| **farm-ng venv** | `/farm_ng_image/venv/` — pre-installed, persistent |
| **depthai version** | **2.22.0.0** — do NOT upgrade to 2.23+ |
| **opencv-python** | **4.7.0.68** |
| **Tailscale IP** | `100.66.121.56` |

### Overlay Filesystem — Critical Architecture Constraint

The Amiga OS root `/` is an **overlay filesystem**. On every reboot, the system reverts to the
base image. **Only paths on the NVMe (`~/`, `/var/lib/docker`) survive reboots.**

| Path | Persistent? | Notes |
|---|---|---|
| `~/` = `/mnt/managed_home/farm-ng-user-laserweeding/` | **YES** | All code must live here |
| `/var/lib/docker` | **YES** | Docker images cached here — built once, reused forever |
| `~/.config/systemd/user/` | **YES** | Systemd user services persist here |
| `/opt/ros/` | **NO** | Wiped on every reboot — cannot install ROS natively |
| `/usr/local/`, `/etc/` | **NO** | Wiped on every reboot — no system-level installs |
| `/farm_ng_image/venv/` | **YES** | Pre-installed at build time; do not re-install |
| `/dev/shm` | NO (tmpfs) | RAM-backed, fast IPC; cleared on reboot but available at runtime |

**Consequence for ROS 2:** ROS 2 Foxy runs inside a Docker container. The image
`dustynv/ros:foxy-ros-base-l4t-r35.2.1` is pulled/built once and stored in `/var/lib/docker`.
It is available on every reboot without reinstallation.

**Consequence for Python deps:** Always install via `pip` inside the farm-ng venv
(`/farm_ng_image/venv/`) or the project's own virtualenv, both of which live under `~/`.

### System — Development PC

| Property | Value |
|---|---|
| **OS** | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| **SSH access** | `ssh farm-ng-user-laserweeding@100.66.121.56` (Tailscale) |
| **ROS 2** | Not installed — install Humble if RViz/bag recording needed on PC |

---

## Build & Run Commands

All commands assume the workspace root is `~/auto_navigation_vidalia`.

### One-time setup (on the brain)

```bash
# Activate the pre-installed farm-ng venv (required every session):
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia
```

### Unit tests (any machine — no hardware, no farm-ng SDK required)

```bash
pip install numpy pytest
python3 -m pytest tests/ -q
```

Onion growth stages (`test_onion_growth_stages.py`, ray-cast sim): every stage
from bare bed to lodged tops, bed end into grass, U-turn re-entry, weeds / low
beds / tilt error / sector dropouts, tall-canopy view collapse, canopy-adaptive
obstacle height (~2–3 min — the slowest file).
Covers: dual-row midpoint geometry, single-side fallback and row-spacing prior
(`test_row_perception.py`), EKF motion-model signs and measurement gates
(`test_ekf_estimator.py`), pure-pursuit sign conventions plus a closed-loop
convergence simulation (`test_row_controller.py`), the three safety zones
(`test_row_safety.py`), the headland U-turn and wheel odometry
(`test_headland_odometry.py`), LiDAR tilt correction and dual-camera fusion
(`test_tilt_and_camera_fusion.py`).  Run after touching perception, control,
or fusion code, before any field deployment.

### LiDAR Row Follow — Soybean Field (centre-residue strip, primary)

```bash
# Perception-only — verify detection first (robot stays still):
python3 scripts/row_follow.py --dual-row

# Autonomous (robot WILL move):
python3 scripts/row_follow.py --auto --dual-row

# Autonomous with OAK-D cameras:
python3 scripts/row_follow.py --auto --dual-row --camera

# Debug mode — stream height histogram, save bird's-eye PNG:
python3 scripts/row_follow.py --debug
```

### LiDAR Row Follow — Onion Field (Vidalia raised bed, 4 rows/bed)

Vidalia layout: the robot straddles a 72 in (1.83 m) raised bed carrying **4
onion rows at 11 in (0.279 m)**, plants 4 in apart; wheels in the furrows.
`--preset onion` = `--bed-rows 4 --row-spacing 0.2794 --headland-radius 0.915 --headland-exit 3.5` (exit: the bed-end gate fires ~3 m before the bed end)
+ crop band [0.05, 0.60] m **above the bed top** + obstacle 0.75 / tire 0.85 m.

**Bed mode (`--bed-rows N`, `RowDetector(bed_rows=N)`, `find_bed_centre`).** The
old onion mode (single-row nearest peak) is only right for ONE row per bed; on
a 4-row bed it locks onto an inner onion row ~14 cm off the bed centre, and the
soybean 2-peak pairing aliases once the robot is ~9 cm off-centre (an inner row
falls in the ±5 cm pairing dead-band).  Bed mode instead:
- fits the whole **N-row comb** — each (peak, slot) proposes a bed centre,
  scored `0.5·misses + 1.0·extras + residual + strip-lock prior + soil-edge
  term`; extras (observed rows nothing explains) outweigh misses (planter
  skips), so capture is ~±0.35 m and a skipped row never hops the lock;
- measures the crop band from the **bed-top soil** (`last_bed_floor`, P20 of
  height over |x| ≤ 0.5 m) — otherwise the 0.15 m bed soil sits inside the band
  and a bare bed reads as a confident row (row end never detected);
- uses the bed's **soil edges** as an independent centre (`last_soil_centre`)
  to break the one real ambiguity (a whole row missing at a start offset);
- **aligns the ROI box with the row direction** (`last_roi_rot`, memoryless
  histogram-sharpness search ±20°) — at 0.28 m spacing a 5° yaw drifts the rows
  two spacings across the 7 m ROI, which biased the fixed box −8…−15 cm.
- **bare-bed RELIEF tracker** (`bed_relief_centre`, `_bed_shape`, on by
  default; `--no-bed-shape`, `--bed-pitch 1.83`): young seedlings (a few cm,
  pencil-thin — the 2026-10-15 field) are invisible to the VLP-16, so the robot
  steers by the raised bed itself — cross-row SOIL-height profile (forward ramp
  removed per range bin), bed centre = midpoint of the two shoulder edges
  between the bed top and the wheel furrows at ±pitch/2; heading from the
  near/far halves.  Used when no crop is seen, or when the crop fit is weak or
  disagrees (> 0.12 m) with the bed shape.  Confidence = contrast / 5 cm;
  row end = relief gone (flat headland).  Status `rel=6cm[BED-SHAPE]`,
  telemetry `bed_contrast_m`, `mode=shape`.  The bed-top floor is the per-range
  dominant-surface (1 cm histogram mode) LINE — no slope dead-band, so a
  residual ~2° ramp can't lift far soil into the crop band (field: bare soil
  read as a row at off=+0.8 m); a bed centre beyond ±0.45 m (wheel on the
  bed) gets confidence 0.
  **Field result 2026-10-15 (bare bed, seedlings invisible):** 150 m in
  `[BED-SHAPE]` mode, lateral mean +0.2 cm / std 5.7 cm, heading std 2.0°,
  relief 7.4 ± 0.7 cm, conf 1.0 throughout.
- **Bed END / grass headland** (same run: relief faded, heading crept to the
  20° box cap, then the crop fit FOLLOWED GRASS for 6 m).  Fixes:
  **bed-presence gate** (`_bed_present`) — once the relief has been tracked
  (`bed_latch_scans`=30), a smoothed relief below 0.6× its running baseline =
  bed ended → conf 0 / row end whatever the crop fit says; hysteresis 0.8× to
  resume (on the real contrast trace: 0 false triggers over 150 m, end
  confirmed at 151.5 m — `tests/data/field_20261015_onion_bed_contrast.npz`);
  **relief-based heading** (`_bed_relief_heading`: rotation maximising the
  soil-profile sharpness, soil = ≤ surface+3 cm so grass/crop never count;
  ±2°/scan rate limit) replaces "box follows tracked heading" (it ratcheted);
  far half / near quarters only steer heading when their relief is ≥0.7× the
  nearer part (the bed end shows up far-first); bed-mode heading cap ±8°;
  **crop lock** — a crop fit steers only after ~2 s of agreeing with the bed
  AND ≥60 % of its points on the 4 predicted row lines (grass ≈30 %).  Sim
  (bed ending into grass): heading ≤ 2–4°, row end ~3 m before the bed end,
  grass never re-locked.  Status `[BED-END]`.
- **Smooth driving (field 2026-10-16, three rows: MPC 194 m, pursuit 128 m,
  RL 71 m — tracking 4–5 cm lateral std, 1.4–2.1° heading std, all good).**
  The operator saw sudden slow-downs / brake-like jerks.  Two causes:
  (1) **false bed-end stops** — mid-row the relief fit returns nothing for 2–4
  scans (bed-top floor jumping between soil and onion canopy: relief 0.07 →
  exactly 0 → 0.07); the gate's EMA read those zeros as "bed ended" 22–108
  times per row → v=0 for ~1 s each.  Fix: a missing fit is NO evidence —
  `_bed_present` holds and the estimate coasts on the last bed geometry for
  ≤ `bed_dropout_hold` (8) scans; replayed on all three logged contrast traces
  → 0 false triggers, real bed end still found (≤0.1 m later on 2026-10-15).
  (2) **speed ∝ confidence** — relief mode (~0.9) vs crop-fit mode (~0.7)
  switching ~1×/s surged the speed ±25 %.  Fix: `speed_conf_full`
  (`--speed-conf-full`, onion preset **0.65**; 1.0 = original law) saturates
  the confidence term, plus a FOLLOW/APPROACH speed **rate limiter**
  (`--accel-limit 0.15`, `--decel-limit 0.5` m/s²; stops are never softened).
  Offline replay of the three runs: stops 382/163/57 → 18/15/1, speed CV
  0.21–0.26 → 0.07–0.14.  NOTE: the robot now actually cruises near `--speed`
  (mean 0.15 → ~0.19 at `--speed 0.20`); lower `--speed` for the old pace.
- **Serpentine side (`--beds-left N --beds-right M`).** The first U-turn fixes
  which side of the field the sweep covers (left-first keeps moving left).
  Started on the 2nd bed from the right (rightmost unplanted), the default
  RIGHT turn went toward the unplanted side and found no bed (APPROACH 3 m →
  DONE).  The LiDAR cannot tell planted from unplanted beds while the onions
  are invisible, so the operator states the layout:
  `navigation/coverage_plan.py::plan_serpentine` turns beds-to-cover on each
  side (excluding the start bed) into the first-turn side (more beds; explicit
  `--turn-dir` wins) and `--rows` = 1 + beds on that side (capped — never turns
  off the field edge), auto-enabling `--headland`.  Tests
  `tests/test_coverage_plan.py`.
- **Every growth stage (2026-10-22 rework, `sim/onion_field.py`,
  `scripts/eval_onion_growth.py`, `tests/test_onion_growth_stages.py`).**
  A ray-cast VLP-16 Hi-Res simulator (real beam geometry, 12.3° tilt, 0.2°
  azimuth, porous leaf-element canopy, raised-bed heightfield, grass headland,
  weeds, stand gaps) showed what the LiDAR sees per stage: **transplants**
  invisible; **young** rows separate; from **mid** season the canopy hides
  the bed-top soil (4 % of returns, 0 % mature) and SHADOWS the wheel furrows
  (columns with 0–6 returns); **mature** rows interleave into one canopy whose
  furrow dip is only ~10 cm; **lodged** tops are a broad low mound.  The old
  stack failed from mid-season on (bed floor clamped onto the canopy, soil-only
  relief blind, crop fit vetoed by a broken relief fit: 15–20 cm biases, ±8°
  heading, a −17…−24° / 34 cm fit at confidence 0.85 under a tall canopy).
  What changed (all bed mode; soybean untouched):
  - **Bed profile = top surface of ALL returns** (`bed_relief_centre(stat=…,
    fill_gaps=True)`): median of the bed centres on the 50/75/90th-percentile
    column profiles; a return-less column counts as LOW (a shadowed furrow is
    a gap, not missing data).  The two wheel furrows are the one feature
    present at every stage.  Independent of the bed floor.
  - **Bed axis from a memoryless two-pass sharpness search**
    (`_bed_box_heading`/`_box_search`): three cues (terrain-relative upper-
    surface count, absolute crop-band count, height-profile variance) — the
    clearer COUNT cue wins (the crop-band count only when the band holds
    ≥ 30 % of the swath — a 4 cm bed / tilt residual leaves it empty and 30°
    off); the height-variance cue only when no count cue has a clear peak
    (lodged).  It is 5–17° off under a mature canopy, and "clearest peak wins"
    let it drag the axis 5° at a U-turn re-entry from the 0.99 m mount (its
    ratio scale is larger).  ±20° at 2° then ±2.5° at 0.25° on a swath re-selected
    in the first estimate's frame (a robot-frame swath biased 8–10° angles by
    1–2°).  ≤ 0.6° rms bare→mature, ~1° lodged (both mounts).  Low-passed (`bed_axis_alpha`
    0.3) and rate-limited against its OWN state (1°/scan; tied to the output
    heading it deadlocked at re-entry).  The shape no longer fits a near/far
    heading (the far furrows are hidden under canopy → ±8° → ±0.3 m).
    Heading cap in shape mode 8° → 15° (a U-turn hands over up to 11.5°).
  - **Ground model = furrow line + bed relief** (`_furrow_fit`, `_soil_mode`):
    the bed-top soil is the LOWEST substantial surface (≥ 15 % of the largest
    height peak), measured relative to the furrow line in the same range bin
    — ramp-proof (1.5–2.5° residual tilt; the dead-banded detrend is bypassed
    in bed mode) — on a swath centred on the bed profile.  The furrow line is
    tied to the calibrated ground at the wheels (|intercept| ≤ 4 cm), must be
    smooth when first fitted (outlier range bins trimmed — from 0.99 m a few
    bins see too few furrow returns and read the bed top), is never STARTED
    once soil has been seen and is hidden, and is frozen while the canopy
    hides the soil;
    the bed relief is low-passed, a > 5 cm rise needs 15 scans, and the last
    measured value is HELD under the canopy (`last_soil_visible`).
  - **Crop comb**: accepted on ≥ 60 % of points on the row lines OR ≥ 45 % with
    an on-row/between-row concentration ≥ 1.4 to lock, ≥ 1.3 once locked
    (leafy rows: 52–59 %, ratio 1.55–1.95 from 0.80 m, 1.35–1.8 from 0.99 m —
    the steeper view sees more fanned leaves between rows; grass at a bed end:
    31–38 %, median ≈ 1.0, p95 1.24); lock hysteresis 0.8 / 0.5 (one
    marginal scan no longer drops to the profile); with NO bed profile a crop
    fit must still pass the comb test or confidence decays.
  - **Bed presence**: present if relief ≥ min(0.6 × baseline, 4.5 cm) (a full
    canopy's baseline made a bare stand gap read as "bed ended"); once ended,
    8 consecutive present scans to resume.  **Bed end**: the far half / second
    quarter dropping below ITS OWN relief baseline (not the near half's; the
    second quarter also counts as bed above an absolute 5 cm — mid-bed it is
    ≥ 8 cm at every stage, at an end 0–4 cm — so a baseline poisoned by one
    wild profile after a U-turn can't latch a false end);
    the profile and axis search are then cut to the bed that remains, the axis
    is held once the end is inside the nearest quarter, and a sustained
    nearest-quarter cut latches "bed ending" (confidence ×0.3 → row end
    ~2–3 m before the end, as the preset's 3.5 m exit expects).  Navigator
    hooks `begin_turn()` / `end_turn()` release the latch for the U-turn.
  - **View quality** (`last_view_quality`, `VIEW=` in status): smoothed count
    of returns in the 3 m × 5.5 m swath; when tops near the LiDAR height
    the view collapses and confidence falls → the robot slows, then stops
    (sim, mature canopy: 0.80 m mount → plants ≳ 0.63 m tall; **0.99 m mount
    → tracks plants up to ~0.78 m, stops ≳ 0.82 m**).  **Sector dropouts**: a hole in the azimuth
    coverage → no profile this scan (coast), never "furrows".  Profile jumps
    > 10 cm from a confident track get half the trust.
  - **Canopy-adaptive obstacle height** (`SafetyMonitor(canopy_clearance=…)`,
    `--canopy-clearance`, onion preset 0.15): obstacle ≥ canopy top + 0.15 m
    (top measured beyond the 2.5 m safety zone so an obstacle can't raise its
    own threshold), capped 0.95 m forward / 1.0 m tires — tall tops no longer
    false-stop; a person still stops.  Off by default (soybean unchanged).
  **Sim results, 0.99 m mount** (`eval_onion_growth.py`, 5 start cases per
  stage, bias / std): bare & seedling ≤ 1.1 / ≤ 1.1 cm; young ≤ 0.4 / ≤ 0.2 cm
  (crop comb); mid ≤ 0.5 / ≤ 1.2 cm; mature ≤ 2.6 / ≤ 1.4 cm; lodged ≤ 2.2 /
  ≤ 2.2 cm; heading ≤ 1.1°; confidence 1.0, no false row ends; bed end found
  2.5–3.5 m early at every stage (what the 3.5 m headland exit expects) and
  grass never followed (max confidence after 0.32).  (0.80 m mount, before
  the raise: mature ≤ 6 cm, lodged ≤ 5 cm, bed end 2.1–3.8 m early — the
  onion suites still pass there.)  Mature/lodged bias is an UNDER-READ ∝ offset (~⅓) —
  zero when centred, so the controller still converges to the centre.  Robust
  suite (`--robust`): weeds, 30 % stand gaps, 4 cm beds, 1.5° tilt error,
  azimuth-sector dropouts, ±3° weaving, re-entry after a U-turn at up to
  0.35 m / 12° — all ≤ ~5 cm bias.  **Known limits:** a whole outer row
  missing under a merged (mid+) canopy reads ~13 cm toward the remaining rows
  (inherent); plants within ~0.17 m of the LiDAR height collapse the view →
  the robot stops (≳ 0.82 m tall plants on the 0.99 m mount; it was ≳ 0.63 m
  on the old 0.80 m mount); all of this is simulation — field-confirm with `--record`
  (telemetry now logs `canopy_top_m`, `soil_visible`, `view_q`, `view_n`,
  `sector_gap`, `comb_ratio`, `crop_locked`, `fwd_obs_h`).  Detector cost
  11–17 ms/scan (x86), similar to the previous release.
Camera row tracking is disabled in bed mode (the camera tracker is 2-row only);
telemetry gains `bed_floor_m`, `soil_centre`, `roi_rot_deg`; status shows
`bed=0.15m`.  Sim (`tests/test_onion_bed.py`): bias ≤ 1 cm from 0 to ±35 cm start
offset, sparse/young rows, skips, flat or 0.25 m beds, yaw up to ~15°.

```bash
# Growth-stage simulator check (any machine, no robot): every stage + field realities
python3 scripts/eval_onion_growth.py --bed-end          # bare → seedling → young → mid → mature → lodged
python3 scripts/eval_onion_growth.py --robust           # weeds, gaps, low beds, tilt error, dropouts, re-entry

# Perception-only — robot stays still; check off≈0, bed≈0.10–0.20m, sp≈0.28:
python3 scripts/row_follow.py --preset onion --record

# Autonomous single bed (robot WILL move):
python3 scripts/row_follow.py --auto --preset onion --controller pursuit --record

# Multi-bed serpentine (headland U-turn not yet field-validated in onion).
# Say how many beds remain on each side of the start bed — it turns toward them:
python3 scripts/row_follow.py --auto --preset onion --beds-left 4 --beds-right 0 --record
# (equivalent manual form: --rows 5 --headland --turn-dir left)

# Other layouts: 2 rows/bed at 0.34 m → --preset onion --bed-rows 2 --row-spacing 0.34
#                1 centre row         → --preset onion --bed-rows 1

# Autonomous with ROS 2 visualization bridge output:
python3 scripts/row_follow.py --auto --dual-row --ros2-bridge

# Debug mode — stream height histogram, save bird's-eye PNG:
python3 scripts/row_follow.py --debug
```

### Camera-Only Row Follow (no LiDAR)

```bash
# Perception-only — verify green detection:
python3 scripts/cam_follow.py

# Autonomous:
python3 scripts/cam_follow.py --auto

# Depth-edge (colour-independent) detector:
python3 scripts/cam_follow.py --auto --detector depth-edge

# Custom HSV range (e.g. brown cardboard):
python3 scripts/cam_follow.py --auto --hsv-h-lo 10 --hsv-h-hi 25 --hsv-s-lo 25

# Multi-row with headland turns:
python3 scripts/cam_follow.py --auto --rows 4 --headland
```

### LiDAR Field Mapping — native SLAM (mapping only, NOT navigation)

`scripts/slam_mapper.py` builds a map of the field from the VLP-16 while you
drive (manually or via row-follow). It is **mapping only** — it never commands
the wheels. Pipeline (`slam/`): yaw/tilt-correct the raw cloud → horizontal slice
→ deskew → trimmed 2-D ICP scan-matching (wheel-odometry warm-start when the
canbus is up, constant-velocity otherwise) → periodic scan-to-map ICP loop
closure → log-odds occupancy grid with ray-cast free-space clearing. Ctrl+C
saves `maps/<ts>/map.npz` + `map.png` (and `map3d.ply` with `--map-3d`).

**Primary use case — field COVERAGE assurance (`coverage.png`).** A single
VLP-16 scan is instantaneous and local; it can't tell you whether the robot
covered the field. The value SLAM adds is the **drift-corrected pose
trajectory** (scan-matching + loop closure keep it metric where raw wheel
odometry would smear after a few rows), and the operational payoff is a
**coverage map**: the robot's working swath (`--swath`, default 1.92 m = wheel
track) stamped along that path (`slam/coverage_map.py`). On save it writes
`coverage.png` (serviced swath in green over the structure map — **gaps are
missed rows**), `trajectory.csv` (as-driven `scan,x,y,heading`), and reports
serviced area, path length, and a **redundancy** ratio (swept ÷ unique area;
1.0 = no overlap, >1 = double-driven). This is the figure for "did the robot
autonomously cover the field, and how well?" — and the natural A/B for the
controllers (drive the same field with `pursuit` vs `mpc`, overlay trajectories,
compare coverage + redundancy). Coverage is on by default whenever SLAM runs
(standalone `slam_mapper.py` or `row_follow.py --slam`); regression-locked in
`tests/test_coverage.py`. For per-pose tracking error, join `trajectory.csv`
with the `--telemetry` log by scan order.

### Experiment records for publishing — `--record`

For any run you want to **analyse / publish from**, add `--record`.  It bundles
a single self-contained, reproducible folder `runs/run_<controller>_<ts>/`
(self-labelling — e.g. `run_rl_...`, `run_mpc_...`, `run_pursuit_...`):

| File | Contents |
|---|---|
| `manifest.json` | git commit + dirty flag + branch, full CLI args, calibration (yaw/tilt/mount/ROI/controller), host, start time — **reproducibility anchor** |
| `telemetry.jsonl` | one row per scan: state, conf, n, lateral/heading error, row-end conf, grade/drop, `sp`, command, safety zones, headland-turn fields |
| `summary.json` | computed metrics (see below) |
| `metrics_flat.csv` | `metric,value` scalars — drops into a spreadsheet / LaTeX table |
| `per_row.csv` | tracking accuracy per row (xtrack RMSE/max, heading RMSE, distance) |
| `turns.csv` | one row per headland U-turn (dir, source `imu`/`wheel`, max/final rotation °, arc m, completed) |
| `coverage.png`, `trajectory.csv`, `map.png/.npz` | SLAM coverage + drift-corrected path |
| `perception_scan.npy` + `perception_state.json` | a REAL captured corrected scan (densest high-confidence FOLLOW frame) + its detected geometry |
| `figure_perception_3d.png`, `_bev.png`, `_annotated.png/.svg` | the three perception figures rendered from that real scan (self-driving-style 3-D, top-down sensor scope, annotated) |
| `scans/scan_NNNNN.npy` + `scans/index.csv` | the **raw corrected point-cloud time series** — the **first scan is always saved (data from the very start of the run)**, then every 10th (≈ 1 Hz) — so ANY moment, including t≈0, can be re-rendered into figures/animations offline; index row = file, t, state, lateral, heading_deg, conf, n. Tune with `--save-scans` (bare = every scan, `N` = every Nth, `0` = off) |

`--record` turns on `--telemetry` + `--slam` automatically and writes the
manifest at start (so a crashed run is still identified) and the summary at
exit.  It snapshots a **real corrected LiDAR scan** (the densest high-confidence
FOLLOW frame) and renders the **three perception figures** from it **a few scans
into the run** — once a confident FOLLOW lock has settled (`perception_capture_after`,
default 5 qualifying scans), a one-shot **background thread** (`scripts/viz_perception.py`,
never blocks driving) writes `perception_scan.npy` + `perception_state.json` +
the figures seconds after start, so a crash mid-run still leaves them; an
exit-time render runs only as a fallback if the run never reached a confident
lock.  The raw `perception_scan.npy` is always saved, and if matplotlib is not
installed on the brain the figures are skipped with a one-line command to render
them offline.  **All raw data is
kept regardless of matplotlib**: `telemetry.jsonl` is written continuously
(one line per scan, survives a crash), and the `scans/` time series
(`navigation/scan_recorder.py`, a background daemon thread so it never slows
driving) lets you render every figure later, offline, from any moment of the
run.  **POV navigation VIDEO** — `scripts/render_pov_video.py` turns the `scans/`
time series + `telemetry.jsonl` into a cinematic self-driving-style POV movie
(chase-cam perspective, height-coloured Hi-Res cloud with the rows/structure,
live overlays: detected strip-centre path, heading, look-ahead, ROI, forward
safety zone that reddens when blocked, ego robot, and a HUD of
state/cross-track/heading/speed/ω/grade/roll).  Renders PNG frames → GIF via
Pillow (no ffmpeg needed) and an MP4 via ffmpeg when present (else prints the
command).  `--view chase|pov|high|scope`, `--fps`, `--stride`, `--res`, and a
`--demo` synthetic fly-through for previewing with no run:
`python3 scripts/render_pov_video.py --run runs/run_<ts> --view chase --out results/pov`.
Metrics are computed by `navigation/run_metrics.py` (numpy-only, runs on
the brain): FOLLOW cross-track RMSE/MAE/p95/max (cm), heading RMSE/MAE/max (°),
control effort + jerk, forward-speed mean/std, state-time budget, event counts
(obstacle stops, FOLLOW losses, dropout scans), terrain grade/drop, per-row
breakdown, and per-turn outcomes.  Regression-locked in `tests/test_run_metrics.py`.

```bash
# Record everything for a run:
python3 scripts/row_follow.py --auto --dual-row --rows 6 --headland --controller mpc --record

# (Re)compute tables from a recorded run, or compare controllers A/B:
python3 scripts/analyze_run.py runs/run_<ts>
python3 scripts/analyze_run.py runs/run_pursuit runs/run_mpc --compare results/field_compare.csv
```

The headline field A/B for the paper: run the same field once with
`--controller pursuit --record` and once with `--controller mpc --record`, then
`analyze_run.py … --compare` emits one CSV row per run (xtrack RMSE/p95/max,
heading RMSE, jerk, coverage, redundancy) — the field counterpart to the sim
`results/controller_pareto.csv`.

```bash
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia

# Drive the robot (joystick / row-follow) while this maps; Ctrl+C to save:
python3 scripts/slam_mapper.py

# Structure-only map (exclude crop rows, keep posts/equipment/edges):
python3 scripts/slam_mapper.py --slice-min 0.20

# Override mount calibration (defaults match the row-follow stack):
python3 scripts/slam_mapper.py --lidar-yaw 0 --lidar-tilt 15

# 3-D field map (point cloud) alongside the 2-D grid:
python3 scripts/slam_mapper.py --map-3d                  # → maps/<ts>/map3d.ply
python3 scripts/slam_mapper.py --map-3d --voxel-3d 0.10  # denser
```

**3-D mapping (`--map-3d`) — 2.5-D, mapping only.** A ground robot moves on the
2-D ground manifold, so full 6-DOF 3-D SLAM is unnecessary (and fragile in
feature-sparse fields, heavy on the Jetson). Instead the **2-D SLAM pose**
(x, y, heading — already loop-closure-corrected) registers every **full
yaw/tilt-corrected 3-D scan** into a world-frame voxel map (`slam/voxel_map.py`,
one point per voxel so map size grows with field area, not scan count). On
Ctrl+C it writes `map3d.ply` (binary, height-coloured — opens in CloudCompare /
MeshLab / Foxglove) + `map3d.npz`. Map z is **height above the local ground
plane** (ground ≈ 0, crop ≈ 0.05–0.30 m, obstacles taller); the robot's own
elevation is not tracked, so absolute terrain elevation is not captured (a true
DEM would need 3-D pose / GPS-IMU z). Loop closures snap the pose, which can
leave a faint seam in the cloud at revisits — acceptable for lawnmower coverage.
`--map-3d` (off by default), `--voxel-3d M` (0.15 m; smaller = denser, more
memory). Regression-locked in `tests/test_slam.py`.

**Critical — same yaw/tilt correction as row-follow.** SLAM yaw/tilt-corrects
the raw VLP-16 cloud into the robot frame (`slam/scan_matcher.py::correct_scan`,
**yaw first then tilt**) before slicing. Without it the ~15° nose-down tilt
ramps far-field ground returns up into the slice band — flat ground 6 m ahead
reads h≈1.5 m — flooding the 2-D slice with ground that moves with the robot,
which wrecks ICP and smears the map. The slice band defaults to h ∈ [0.05, 1.50]
m so the crop rows are captured as map structure (the dominant repeatable
feature in an open field, and what ICP locks onto); `--slice-min 0.20` excludes
the crop for a structure-only map. Regression-locked in `tests/test_slam.py`.

| `slam_mapper.py` flag | Default | Description |
|---|---|---|
| `--lidar-yaw DEG` | **0.0** | Mount yaw correction (applied FIRST) — matches row-follow (forward-facing) |
| `--lidar-tilt DEG` | **15.0** | Nose-down pitch correction (applied AFTER yaw); VLP-16 Hi-Res angles |
| `--slice-min M` | **0.05** | Slice lower bound, ground-relative m (includes crop; raise to 0.20 for structure-only) |
| `--slice-max M` | **1.50** | Slice upper bound, ground-relative m |
| `--map-3d` | off | Also build a 3-D point-cloud map → `map3d.ply` + `.npz` |
| `--voxel-3d M` | **0.15** | 3-D voxel size (m); smaller = denser, more memory |
| `--swath M` | **1.92** | Working width serviced per pass for `coverage.png` (= wheel track); gaps = missed rows |
| `--icp-points N` | 400 | ICP subsample count |
| `--map-every N` | 1 | Update the grid every N scans |
| `--autosave N` / `--no-autosave` | 60 s | Periodic auto-save interval |
| `--no-odom` | off | Ignore wheel encoders (constant-velocity ICP warm-start only) |

### Optional: Learned (RL) steering — experimental, opt-in

A reinforcement-learning policy can replace the **FOLLOW steering command only**
(the angular velocity); everything else — state machine, safety monitor, headland
turn, forward-speed law — is unchanged. The learned policy is a **bounded
refinement on a known-good geometric controller**, never a replacement for the
safety architecture: `RLController` falls back to pure-pursuit below
`min_confidence` or when no policy is loaded, and the output is hard-clamped to
`--max-angular`. Default is `--controller pursuit`; nothing changes unless you
opt in. Numpy-only (no torch/gym), so the policy deploys on the Jetson with zero
new dependencies.

**Field over-correction & the slew-rate bound (`--rl-slew`).** A field RL run
thrashed left/right at the row start until it stalled in ACQUIRE (heading std
20.8° vs MPC 2.7°; 7.6× pursuit's steering effort). Replaying the run showed the
cause was NOT the policy — in sim the deployed `follow_jerk8.0.npz` is already
smooth (jerk 0.051 ≈ MPC, xtrack 8.6 cm) — but the **dense-canopy heading
runaway feeding it ±60–70° garbage**, which RL (reacting fastest) amplified. The
primary fix is at the perception source (the reliability hold / row-end veto,
validated on the real `run_pursuit_20260707_000410` scans). As an *independent*
hard field-safety bound, `--rl-slew R` caps the change in the steering command
per control step (rad/s per ~0.1 s scan), so no policy and no noisy input can
swing the wheels faster than R — try 0.10–0.15 in the field. Off by default
(preserves existing behaviour).

**Control-effort penalty → the deployed field policy (`follow_field.npz`, DEFAULT
for `--controller rl`).** After the safety fixes stopped the ~90° whip, a field
RL run still steered "weirdly" — aggressive, over-turning — even though it
tracked (13 cm) and was low-jerk.  Replaying the real `run_rl_20260709_010533`
scans through ALL THREE controllers on the *identical* perception estimates
isolated the cause: RL's steering effort was **|ω| 0.147 — 6.7× pursuit and 1.5×
MPC** for the same input, and smoothing the input did NOT reduce it, so it was
the POLICY, not perception noise.  Root cause: `follow_jerk8.0` was trained with
a strong *jerk* penalty (smooth CHANGES) but a near-zero *effort* penalty
(`c_u=0.05`), so it holds large steady steering cheaply (low jerk, high
magnitude).  Retraining with a real effort penalty (`--c-u 0.8 --c-du 8.0`)
gives `follow_field.npz`: it **keeps RL's tracking edge** (sim held-out xtrack
**10.3 cm — still best**, beats MPC 14.2 and pursuit 17.9) while cutting the
real-field steering effort to **|ω| 0.090 — MPC's level** (0.095), with a lower
peak too.  So RL is now best-tracking AND as smooth as MPC — the intended
result.  This IS a new deployed default (the earlier residual retrain was a
negative result; this effort-penalty retrain is the win).  Measure any candidate
the same honest way with `scripts/replay_scans.py` on real field scans.  The
`--c-u` sweep (0.05 → 0.4 → 0.8) is the accuracy/smoothness knee for the paper's
controller Pareto.  Pair RL with **`--accumulate 3`** in dense canopy: on the
real scans it cut the heading *peaks* 18°→13° (the spikes that trigger lunges)
with negligible lateral cost.

**Drift integrator (field-motivated).** The policy observes
`[lateral, heading, conf, prev_action, drift_integral]`. A field RL run drifted
downhill and off the row on cross-slopes; a memoryless reactive policy
*structurally cannot* cancel a steady disturbance (constant drift → constant
steady-state error). The 5th input is a **leaky integral of the lateral offset**
(`rl_policy.update_eint`, the RL analogue of the MPC disturbance observer),
maintained identically in the sim, the eval, and the deployed controller, so the
policy can learn a steady counter-steer. Held-out sim (200 episodes, disturbance
model **calibrated from field telemetry** — grade-correlated heading noise +
cross-slope drift): cross-track RMSE **21.4 → 8.5 cm**, success **92 → 99.5 %**;
on the steepest cross-slopes pure-pursuit success **collapses to 7 %** where the
integrator policy holds at **93 %**. It even edges MPC on tracking (8.5 vs
14.2 cm) though MPC stays smoother (heading 6.1° vs 9.3°, jerk 0.054 vs 0.099).
The old 4-input policies still load and run (integrator dropped).

```bash
# 1. Train the steering policy in the simulator (Evolution Strategies):
python3 scripts/train_rl.py --iters 300 --out policies/follow.npz

# 2. Honest comparison vs pure-pursuit on held-out episodes (same disturbances):
python3 scripts/eval_controller.py --policy policies/follow.npz --episodes 200 \
    --csv results/eval.csv          # per-episode rows (incl. each episode's drift) for figures

# 3. If (and only if) it wins, run it — pure-pursuit stays the fallback:
python3 scripts/row_follow.py --auto --dual-row --controller rl --policy policies/follow.npz
```

**Accuracy vs smoothness trade-off (sweepable).** The reward's control-jerk
penalty `--c-du` (with `--c-e` / `--c-theta` / `--c-u`) trades tracking accuracy
against control smoothness; train at a few values and pick the knee of the
Pareto front. The `--csv` export from `eval_controller.py` (columns: controller,
seed, grade_drift, slip, xtrack_rmse_m, heading_rmse_deg, control_jerk, success,
return) drops straight into a scatter (xtrack vs drift) or box plots.

| File | Role |
|---|---|
| `sim/row_follow_env.py` | Gym-style steering simulator: skid-steer slip, cross-slope drift, sensor noise+dropout; **heading noise is grade-correlated, calibrated from field telemetry** (flat ~1.9° → sloped ~5.2° std) so training sees the disturbance that caused the field weave; forward speed = the real pure-pursuit formula so RL and pursuit face identical dynamics |
| `sim/evaluate.py` | Shared rollout + metrics (cross-track RMSE, heading RMSE, control jerk, success rate); pure-pursuit-as-env-policy baseline |
| `navigation/rl_policy.py` | Tiny numpy MLP (`[5→16→16→1]`, tanh, 385 params); flatten/`.npz` save-load; single-source obs encoding. Obs = `[lateral, heading, conf, prev_action, drift_integral]` — the 5th input is a **leaky drift integrator** (`update_eint`, the RL analogue of the MPC disturbance observer) so a memoryless policy can cancel steady cross-slope drift. 4-input policies still load/run (integrator dropped) for backward compat |
| `navigation/rl_controller.py` | Drop-in for `PurePursuitController`; pursuit fallback + output clamp |
| `scripts/train_rl.py` | Evolution-Strategies trainer (numpy-only, gradient-free); reports baseline throughout |
| `scripts/eval_controller.py` | Held-out pursuit-vs-RL-vs-MPC table + cross-slope-severity breakdown (`--mpc` to include MPC) |

**Onion field — sim system-ID + retrained policy (`follow_onion.npz`).** The
onion bed (2026-10-15) is a different disturbance regime from soybean: the
perception estimate is noisy (lateral std 5.6 cm, scan-to-scan 3 cm, slow
target wander) on flat ground with no dropouts at 0.20 m/s.  The soybean policy
(`follow_field`) steered **6.4×** pursuit's effort on the real onion estimates.
`scripts/calibrate_sim_from_run.py <telemetry.jsonl>` fits the sim's noise
model (white + AR(1) wandering lateral/heading bias, detector EMA, speed) so
pure pursuit IN THE SIM reproduces the run's closed-loop stats (within ~10 %);
the fit is `sim.row_follow_env.onion_bed_config()` (`--profile onion` in
`train_rl.py` / `eval_controller.py`; default sim unchanged).
`scripts/replay_controllers.py <telemetry.jsonl> --mpc-onion --policy …` replays
the REAL logged estimates through every controller (open-loop, identical
inputs) — the honest sim-to-real check (pursuit replay reproduces its logged
command: |ω| 0.0076 vs 0.0077).  Trained `--profile onion --c-u 0.8 --c-du 8.0`
(from scratch; fine-tunes and heavier penalties were tried and steered MORE on
the real estimates): onion sim xtrack **8.0 cm vs pursuit 9.7**, 100 % success;
real-estimate replay |ω| **3.1×** pursuit (vs 6.4× follow_field), peak 0.12 vs
0.21 rad/s, jerk 0.0014 ≈ pursuit; still 97 % on the soybean sim.
**MPC in the onion field:** default MPPI steered 5.3× pursuit on the real
estimates; raising its effort/jerk costs made it WORSE (12–19×: the soft-argmin
flattens toward averaging the sampling noise).  The calm setting is sampling
noise 0.20, temperature 0.3, observer gain 0.05 (flat field — a fast slope
observer only chases noise): **3.5×**, same tracking.  `--preset onion` applies
both: `--controller rl` → `follow_onion.npz` + `--rl-slew 0.12`;
`--controller mpc` → `--mpc-noise 0.20 --mpc-temp 0.3 --mpc-drift-gain 0.05`.
Regression-locked in `tests/test_onion_rl.py`.

### Optional: Model-predictive (MPC/MPPI) steering — experimental, opt-in

A state-of-the-art alternative to pure pursuit for the **FOLLOW steering command
only**, added on top (default stays `--controller pursuit`; the current
controller runs unchanged with `--controller mpc` off). `RowMPCController`
(`navigation/row_mpc_controller.py`) is **MPPI** — Model-Predictive Path Integral
control (Williams et al. 2017): it samples K steering sequences over an H-step
horizon, rolls each through a kinematic row-tracking model, and applies the
cost-weighted soft-argmin action. Gradient-free, numpy-only, no new deps.

The field win is the **online disturbance observer**: a scalar Luenberger
estimate of the persistent cross-slope drift, updated from the predicted-vs-
measured lateral residual. MPPI previews *with* it, so it actively cancels the
slope drift that pure pursuit (a memoryless geometric law with no integral
action) can only chase — the source of the slope-weaving in the field logs. Same
safety contract as RL: steering only, pure-pursuit fallback below
`min_confidence`/invalid fix, hard `--max-angular` clamp.

```bash
# Compare pursuit vs MPC (and RL) on held-out episodes (same per-seed disturbances):
python3 scripts/eval_controller.py --mpc --policy policies/follow.npz --episodes 200

# Run it (pure-pursuit stays the low-confidence fallback; no training needed):
python3 scripts/row_follow.py --auto --dual-row --controller mpc
```

Held-out sim (200 episodes): cross-track RMSE 22.5 → 15.7 cm, success 87 → 96 %;
on the steepest cross-slopes pure-pursuit's success collapses to 0 % where MPC
holds at 100 %. Lower heading RMSE than the RL policy (6.6° vs ~10°) with no
training. Full pursuit/RL-jerk-sweep/MPC table: `results/controller_pareto.csv`.

| File | Role |
|---|---|
| `navigation/row_mpc_controller.py` | MPPI sampling MPC + online drift/slip observers; drop-in for `PurePursuitController`; pursuit fallback + `reset()` |
| `sim/evaluate.py::mpc_act_fn` | MPC as a resettable env policy (rollout clears its state per episode) |

**Authenticity / caveats (read before trusting any result):** (1) The reward is
a per-step **alive bonus** + forward progress − quadratic costs on
offset/heading/**control jerk**; the alive bonus dominates so the only way to
maximise return is to stay on the row the whole episode and be smooth — without
it the agent reward-hacks by driving off-row early to stop paying per-step costs
(observed and fixed during development). (2) All results are **in simulation**;
the sim-to-real gap is the headline risk, so any field trial must keep the
pure-pursuit fallback armed and start in perception-only mode. (3) RL is not
expected to beat a well-tuned geometric controller on the nominal task — its
plausible edge is robustness to cross-slope drift and smoother control; the eval
harness reports the truth either way. Regression-locked in `tests/test_rl.py`.

### ROS 2 Visualization Bridge (Docker — on the Amiga Brain)

```bash
# Terminal 1: run navigation with bridge output:
python3 scripts/row_follow.py --auto --tire-height 0.85 --ros2-bridge

# Terminal 2: build and start the ROS 2 Foxy container:
bash ros2_bridge/start.sh
# First run: ~10–20 min (pulls ~4–6 GB L4T image)
# Subsequent runs: ~5 seconds (uses cached image)

# Install autostart service (runs on every reboot):
bash ros2_bridge/install_autostart.sh

# Visualize in any browser — no install on laptop/tablet:
# https://app.foxglove.dev → Open connection → Rosbridge WebSocket → ws://100.66.121.56:8765
```

### ROS 2 Foxy (dev PC only — NOT on the brain)

```bash
source /opt/ros/foxy/setup.bash
cd ~/auto_navigation_vidalia
colcon build --symlink-install
source install/setup.bash

# Full production run:
ros2 launch vidalia_bringup localize_nav.launch.py database_path:=~/maps/field.db rviz:=true
```

---

## Architecture

### Native Python Stack — LiDAR Row Follow

```
Velodyne VLP-16 (192.168.1.201 UDP :2368, 10 Hz)
   │  lidar/lidar_driver.py        raw UDP → Nx3 numpy array
   │  lidar/obstacle_filter.py     tilt_correct_pts(tilt_rad=15°) + self-filter (≥ 1.5 m)
   ▼
   ┌────────────────────────────────────────────┐
   │  navigation/row_perception.py              │  PCA → RowEstimate
   │  navigation/row_safety.py                  │  3-zone obstacle monitor
   └───────────────────┬────────────────────────┘
                       │
   OAK-D cameras (optional, async)
   │  camera/oak_driver.py          farm-ng EventClient → localhost:50010
   │  camera/depth_obstacle.py      inner-edge depth strip → obstacle blocked/clear
   │  camera/row_detector_visual.py HSV green centroid + PCA heading (linearity-gated)
   ▼
   navigation/row_navigator.py     state machine + LiDAR/camera fusion
   navigation/row_controller.py    pure-pursuit → (linear_vel, angular_vel)
   canbus/canbus_interface.py      Twist2d via request_reply("/twist")
   Amiga wheels

   (optional) /dev/shm/vidalia_pts.bin + vidalia_status.json  ──► Docker ROS 2 bridge
```

### Camera-Only Stack

```
OAK-D left + right (farm-ng EventClient → localhost:50010, service_name=oak0/oak1)
   │  camera/oak_driver.py            RGB + depth frames (JPEG over gRPC)
   │  camera/row_detector_visual.py   HSV green centroid  ─┐
   │    OR                                                  ├── VisualRowEstimate
   │  camera/row_detector_depth_edge.py  Canny/Hough      ─┘
   │  camera/depth_obstacle.py        inner-edge depth strip → obstacle check
   ▼
   navigation/row_navigator_cam.py    CamRowNavigator (same state machine)
   navigation/row_controller.py       pure-pursuit (unchanged)
   canbus/canbus_interface.py
   Amiga wheels
```

### ROS 2 Visualization Bridge (Docker)

```
row_follow.py --ros2-bridge
   │  writes /dev/shm/vidalia_pts.bin      (int32 n + float32 xyz)
   │  writes /dev/shm/vidalia_status.json  (state, velocities, obstacle flags)
   ▼
Docker container (dustynv/ros:foxy-ros-base-l4t-r35.2.1, --runtime nvidia)
   │  vidalia_node.py   reads /dev/shm/ at 12 Hz
   │    publishes: /velodyne_points, /tf_static, /row_viz, /safety_viz, /cmd_vel
   │  foxglove_bridge   WebSocket on port 8765
   ▼
https://app.foxglove.dev → ws://100.66.121.56:8765  (any browser, zero install)
```

**farm-ng OS 2.0 API pattern (DO NOT use deprecated OS 1.0 classes):**
```python
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig

# Send velocity command:
await client.request_reply("/twist", Twist2d(linear_velocity_x=0.5, angular_velocity=0.0))

# Subscribe to a service:
async for event, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
    ...
```

---

## Row-Follow Stack — Detailed Reference

### Key Files

| File | Purpose |
|---|---|
| `scripts/row_follow.py` | CLI entry point; parses flags, wires tasks, asyncio loop |
| `scripts/cam_follow.py` | Camera-only entry point |
| `navigation/row_navigator.py` | State machine: ACQUIRE → FOLLOW → ROW_END → HEADLAND / OBSTACLE_WAIT; `/dev/shm` bridge writer |
| `navigation/row_navigator_cam.py` | Camera-only state machine (same states + closed-loop headland) |
| `navigation/row_perception.py` | PCA-based LiDAR row detector; single-row + dual-row (midpoint) modes |
| `navigation/headland.py` | **Perception-closed arc U-turn** driver; EXIT→ARC, keeps arcing (large radius) until the navigator detects the next row aligned ahead; odometry used only for arc-length guards (never heading) |
| `navigation/odometry.py` | Wheel odometry (measured AmigaTpdo1 speed; commanded-velocity fallback) |
| `navigation/row_safety.py` | Three-zone obstacle monitor (forward + left/right tire tracks) |
| `navigation/row_controller.py` | Pure-pursuit speed/steering controller |
| `lidar/lidar_driver.py` | Async UDP VLP-16 driver; vectorised numpy scan_stream_np() |
| `lidar/obstacle_filter.py` | `tilt_correct_pts()` + `LIDAR_MOUNT_HEIGHT`; self-filter logic |
| `camera/oak_driver.py` | Async OAK-D driver via farm-ng EventClient (localhost:50010) |
| `camera/depth_obstacle.py` | Depth-frame inner-edge obstacle detector (`col_centre_frac`) |
| `camera/soybean_row_tracker.py` | **Dual-camera ground-projection tracker** — both forward-facing OAK-Ds project green pixels onto the canopy plane (IPM); each independently estimates the residue centre; fusion = equal-weight mean (cancels canopy-height bias) |
| `camera/row_detector_visual.py` | HSV green-centroid + linearity-gated PCA heading (single-row/onion) |
| `camera/row_detector_depth_edge.py` | Colour-independent Canny/Hough + depth-gap lateral detector |
| `ros2_bridge/Dockerfile` | L4T-optimised ROS2 Foxy image: `dustynv/ros:foxy-ros-base-l4t-r35.2.1` |
| `ros2_bridge/vidalia_node.py` | ROS2 Python node: reads `/dev/shm/`, publishes 5 topics at 12 Hz |
| `ros2_bridge/start.sh` | Build + run container with `--runtime nvidia` + `/dev/shm` bind mount |
| `ros2_bridge/install_autostart.sh` | Installs `~/.config/systemd/user/vidalia-ros2-bridge.service` |

---

### Soybean Field Autonomy — Full Pipeline

The soybean field is a **centre-residue strip** layout: the robot straddles a
dark strip of crop residue, with a soybean row flanking it on each side and the
wheels running in the furrows.  The control target is the **centre of the
residue strip**, tracked from three independent sources and fused.

```
                LEFT soybean row     centre residue strip     RIGHT soybean row
                       │              (tracking target)              │
   left OAK-D ─────────┴─────────────────────┬───────────────────────┴───────── right OAK-D
   (forward-facing, above left tire,         │           (forward-facing, above right tire,
    sees BOTH rows)                          │            sees BOTH rows)
                                  VLP-16 LiDAR dual-row
                              (midpoint of L+R crop peaks)
```

**Three estimators of the same residue-strip centre:**

1. **LiDAR dual-row** (`RowDetector(dual_row=True)`) — histograms the crop-band
   cross-row coordinate; `find_row_midpoint` pairs the left/right peaks with a
   row-spacing prior and returns their **midpoint**.  Primary source (full 0–1
   confidence).
2. **Dual-camera ground projection** (`DualCameraRowTracker`) — both OAK-Ds are
   **forward-facing** (15° nose-down, ±0.88 m above the tires) and see **both**
   flanking rows.  Each camera HSV-masks the green, ray-casts every green pixel
   through the known intrinsics + extrinsics onto the canopy plane (inverse
   perspective mapping — metric robot-frame points, no stereo depth needed),
   then fits the points with the **same pipeline as the LiDAR** (cluster-centred
   PCA heading → cross-row histogram → shared `find_row_midpoint`).  Each camera
   yields an independent metric estimate of the SAME centre; fusion is their
   equal-weight mean (camera-capped 0–0.5 confidence, scaled by cross-camera
   agreement).
3. **Fusion** — two levels, selected by `--cam-fusion` (both keep the camera
   subordinate to the LiDAR):
   - `estimate` (default, original behaviour): EKF (`--ekf`) or
     confidence-weighted average folds the camera centre into the LiDAR
     centre; camera weight is capped at half the LiDAR weight, and the
     confidence boost scales with LiDAR↔camera agreement.
   - `point` (opt-in, measurement-level): the tracker's metric ground points
     are pooled with the LiDAR crop points into **one weighted row fit** —
     a single cross-row histogram, one spacing-prior peak pairing, one PCA
     over all evidence (`RowDetector.update(pts, aux_xy=...)`).  Camera
     total mass is capped at `aux_mass_ratio × full_points` (50 % of a full
     LiDAR scan), so a healthy LiDAR always dominates on disagreement —
     with a centred LiDAR and a camera mis-calibrated by 0.35 m the fused
     lateral moves < 0.05 m, because the spacing prior simply pairs the
     LiDAR peaks.  Camera points carry the fit through empty VLP-16
     crop-ROI scans (no more confidence decay toward ACQUIRE during
     dropouts) and extend coverage into the < 1.5 m LiDAR self-filter
     blind zone (`aux_y_min = 0.5 m`).  The camera estimate is then NOT
     fused again at the estimate level (no double counting).

**Cluster-centred PCA (ALL dual-row fits, LiDAR-only included):** every point
is assigned to its nearest cross-row histogram peak, each peak cluster is
centred on its own centroid, and the pooled centred points are re-fit —
iterated twice so a badly tilted seed axis cannot mis-assign the far end of
the longer stripe.  Points farther than 0.25 m from every peak are excluded
from the heading fit (they still vote in the midpoint histogram), so
sub-threshold clutter stripes cannot tilt the heading.  This matters even
without cameras: the VLP-16 routinely drops whole azimuth sectors (UDP packet
loss — `left=0` / `right=0` scans in validate output), so one row's stripe is
often truncated in y relative to the other; whole-cloud PCA over two stripes
with unequal extents tilts toward the line joining the stripe centroids,
which in the field accumulated into a steady +12° heading bias and a
rightward drift off the strip.  Regression-locked in
`test_dual_row_truncated_stripe_does_not_tilt_heading`.

**Why the wide-baseline pair beats one camera:** ground projection assumes the
green sits at `canopy_z`; real canopy tops sit higher, so each ray overshoots
radially AWAY from its camera by k ≈ cam_z/(cam_z − h).  The left camera's
midpoint is biased ≈ +0.88·(k−1) (to the right), the right camera's by the same
amount to the LEFT — **equal and opposite**, so the equal-weight mean cancels
canopy-height bias to first order (verified in
`test_canopy_height_bias_cancels_in_two_camera_mean`).  Shared lighting/exposure
errors also cancel, heading noise drops by √2, and degradation is graceful: one
camera alone still sees BOTH rows and still produces a complete centre estimate
at reduced confidence; one row occluded → `find_row_midpoint` falls back to the
visible row ± half the row spacing.

**Per-camera fitting details** (`soybean_row_tracker.py`):
- Ground maps (per-pixel ray→canopy-plane intersection) are precomputed once per
  frame size and shared by both cameras (same pitch, zero yaw — they differ only
  by the constant `cam_x` offset).
- **Depth sanity gate**: a green pixel whose stereo depth is < 0.5× the
  ground-ray distance is a TALL object (person, equipment) standing above the
  crop — discarded so obstacles cannot smear green into the row estimate.
  Pixels with no valid depth are kept (the gate is advisory).
- **Two-pass cluster-centred PCA**: an off-centre camera sees the outer row's
  near end cut off by the FOV; whole-cloud PCA over two stripes with unequal
  extents tilts the axis ~2–3° and biases the midpoint ~0.10 m toward the
  camera.  Pass 2 splits the points at the pass-1 midpoint, centres each row
  cluster on its own centroid, and re-runs PCA on the pooled centred points —
  within-row direction only, unbiased regardless of visible extents.
- ROI: y ∈ [0.8, 5.0] m (beyond ~5 m one pixel spans tens of cm), |x| ≤ 0.90 m
  in the ROBOT frame (keeps rows at ±0.38, excludes next rows at ±1.14).

**End-of-row U-turn** (`HeadlandTurn`, perception-closed) — ⚠ not yet field-validated:

```
ROW_END ─► HEADLAND ──────────────────────────────────► FOLLOW (next row)
            EXIT (drive exit_dist straight, clear last plants;
                  abort back to FOLLOW if crop reappears — blind-spot guard)
            ARC  (gentle large-radius arc; KEEP arcing until the LiDAR sees the
                  next row aligned ahead → hand straight to FOLLOW.  Give up &
                  STOP after the arc-length cap = no next row / field edge.)
```

**Why IMU-measured, not wheel/arc/perception alone (field-critical):** every
earlier design lacked a reliable measure of how far the robot had ACTUALLY
rotated.  Wheel heading over-reports ~2× (skid-steer scrub); the open-loop arc
*under*-rotates (slip widens the real radius far past the commanded one, so a
given arc length is much less rotation — a "1.0 m radius" command traced ~1.7 m
in the field, ~90° at the point the turn thought it was done); and a
perception-only exit got fooled because the dual-row detector **locks onto grass**
at the headland (it can't tell grass from soybean rows) and ended the turn at
~90° on the wrong feature.  The fix uses the one rotation sensor independent of
wheel contact: the **IMU in the filter service** (`FilterState` heading).  The
turn only needs the *relative* heading CHANGE over ~10 s, and IMU yaw is locally
accurate even before the GPS filter globally converges — so `HeadlandTurn`
accumulates the absolute IMU heading change during the arc **regardless of
`has_converged`** (the previous code wrongly required convergence and fell back
to the unreliable wheel heading).  `row_navigator._step_headland` then:
- keeps arcing until ~180° of REAL rotation;
- only lets a perception lock end the turn AFTER ≥ `reacquire_min_turn` (**170°**)
  of measured rotation AND at high confidence (`--reacquire-conf`, default 0.72)
  — so a confident *grass* detection at ~90°, or a noisy "aligned" reading while
  still 30° short, can no longer end it early.  **These are TRUE degrees when the
  IMU is live (`rot=…[imu]`)**, so the floor must be near 180° — the old 150°/175°
  were tuned for the wheel regime where 150° displayed ≈ 110° physical and ended
  the turn ~30° short (robot then drove into the rows at an angle);
- completes on heading alone at `turn_complete_deg` (**178°**) → then APPROACH creeps
  in to acquire the actual row if perception hasn't already locked it.
**When the filter/IMU is NOT publishing** (common in the test field — the filter
service needs a GPS fix it doesn't have, so `FilterState` never arrives), the
turn falls back to the **wheel-odometry heading** (`measured_angular_rate`,
always live) scaled by `--turn-scrub-comp` (default **0.5**; heavy-slip fields
~0.45): the skid-steer arc over-reports body rotation, so real rotation ≈ wheel ×
scrub_comp.  This still *responds to how much the robot is actually turning*
(unlike a fixed arc length), and the status `rot=NNN[wheel]` lets you calibrate
`scrub_comp` in one run — watch `rot=` and when the robot physically completes
180°, `rot` should read ~180.  **Too HIGH a scrub_comp ends the turn EARLY**: a
field run that ended at a real ~120° while `rot` displayed 159° (so the wheels
reported 265°) needs scrub_comp ≈ 120/265 ≈ 0.45.  If `rot` reads LOW when the
robot has truly turned, raise it.  The
arc angular rate also **eases in** (smoothstep over the first `ramp_dist`) so the
skid-steer doesn't lurch — smoother and less initial scrub.  A `max_turn_frac`
arc cap STOPS the robot if nothing ever completes.  Simulated with the IMU off,
body turning 55 % of commanded and grass the whole arc, the turn ends at a true
~170° on the real row.  `--headland-radius` (auto 1.0 m), `--headland-turn-rate`
(arc rate), `--reacquire-conf` (0.72), `--turn-scrub-comp` (0.5).
Status: `R-UTURN:ARC rot=120°[imu|wheel] arc=2.1m reacq=2/4`.

**No separate APPROACH after the turn (normally):** because the turn only ends
once the next row is already locked aligned ahead, it hands **straight to
FOLLOW**.  The APPROACH state remains as a *fallback* (and post-turn settling
net): if the fresh FOLLOW lock is marginal for the first metre, an early loss
re-enters APPROACH to creep forward (`--approach-speed`) rather than stalling,
bounded by `--approach-max-dist`.  Status shows `SETTLE …`.

**Off-the-field guard:** `--post-turn-max-dist` (default 5.0 m) bounds the TOTAL
distance travelled after a U-turn (APPROACH creep + any short, un-settled FOLLOW
segments) before a stable down-row FOLLOW is established; if spent without
settling, the robot STOPS instead of hunting off the end of the field.

**Blind-spot row-end confirmation:** the VLP-16 is blind inside ~1.5 m, so the
last plants in the near zone (or a brief sparse patch) can read as a row end.
The straight EXIT leg doubles as a confirmation — if solid crop reappears in the
ROI during EXIT (before any rotation) the row had not actually ended and the
turn **aborts back to FOLLOW** (`headland_abort_frames`, default 3).  This is
the "take a little extra distance unless obstacle" check; a real obstacle still
pauses the leg via the safety monitor.

Turn direction alternates each row for **serpentine coverage** (right, left,
right, …), starting from `--turn-dir` (default right).  The loop repeats until
`--rows N` rows are complete, then DONE.

**Heading source = IMU *relative* change, not the converged filter pose.** The
turn uses `FilterState.heading` only for its CHANGE during the arc, so it works
even when `has_converged` is False (no RTK lock) — which is the normal case in
the test field. It never needs the absolute/global heading. If the filter
service isn't publishing at all, the turn falls back to the arc-length window +
strict perception (status shows `[arc]` instead of `[imu]`). The turn is fully
perception-closed — the IMU rotation + the next-row lock decide where it lands —
so there is no strip-spacing parameter sizing it (the old `--headland-shift`,
which only stored a value the turn never read, has been removed).

**Field geometry (Vidalia soybean field):**
Two soybean rows flank the centre residue/stubble strip inside the wheel tracks
(≈ ±0.38 m from the strip centre, separation ≈ 0.76 m = standard row spacing).
Outer rows run just outside each tire track (≈ ±1.14 m) and are already
excluded by the default `--roi-x 0.80` ROI.  Use `--row-spacing 0.76` (the
default) and `--roi-x 0.80` (the default) — no special flags are needed.

**Initial alignment:** if the robot is parked at an angle to the rows (heading
error > ~5°), the pure-pursuit look-ahead will initially steer toward the row's
direction rather than the lateral offset, appearing to drift sideways for the
first 5–10 m before converging.  Position the robot roughly aligned to the rows
before starting, or add `--align-heading --align-speed 0.25` to have the robot
pivot in place before FOLLOW.

**Run it:**
```bash
# Perception-only (verify L/R row detection + midpoint before moving):
python3 scripts/row_follow.py --dual-row --camera

# Single row, autonomous, LiDAR + dual cameras:
python3 scripts/row_follow.py --auto --dual-row --camera --no-cam-obstacles

# With automatic heading alignment before FOLLOW (pivots in place first):
python3 scripts/row_follow.py --auto --dual-row --camera --no-cam-obstacles --align-heading

# Multi-row serpentine coverage with closed-loop U-turns:
python3 scripts/row_follow.py --auto --dual-row --camera --no-cam-obstacles --rows 6 --headland \
    --row-spacing 0.76 --turn-dir right
```

---

### Sensor Frame Convention

> **⚠️ SENSOR IS A VLP-16 PUCK HI-RES (±10°, 1.33° spacing) — NOT a standard VLP-16.**
> Confirmed 2026-07 from the packet product-ID byte `0x24`
> (`scripts/diag_sensor_model.py`).  The driver originally used the standard
> Puck's ±15° / 2.00° angle table; with the wrong (wider) angles the vertical
> geometry is stretched, so a true **15° nose-down / 0.80 m** mount reconstructs
> as a phantom **~22° / 1.17 m** — which sent the tilt/height calibration below
> chasing wrong values for months.  `lidar/lidar_driver.py::VLP16_VERTICAL_ANGLES`
> is now the Hi-Res table (regression-locked in `tests/test_lidar_driver.py`).
> **Diagnose any future "pitch/height doesn't match the tape" with
> `scripts/diag_pitch_rings.py`** — it solves pitch & height from raw ring
> geometry with no correction code in the loop (correct angles → all 16 channels
> agree; wrong angles → they fan out 9–19°).
>
> **CURRENT MOUNT (2026-10 RAISED mount, Hi-Res angles):** the LiDAR was
> raised to **~0.98–1.00 m** (tape); everything else about the mount is the
> same (forward-facing, nose-down).  `LIDAR_MOUNT_HEIGHT = 0.99 m` is the
> provisional value until the box calibration below is run; override per run
> with `row_follow.py --lidar-height M`.  Last field calibration on the
> previous 0.80 m mount (2026-10): `--lidar-yaw 1.7 --lidar-tilt 12.3` —
> re-derive both after the raise.  **Everything below about 66° yaw / 21.5°
> pitch / 0.75 m predates the 2026-07 re-mount AND the Hi-Res fix — treat it
> as history, not current calibration.**
>
> **Mount calibration procedure (after ANY mount change; robot on its wheels,
> flat ground):**
> 1. **Pitch + height, pipeline-free** — clear ~8 m straight ahead (box OUT of
>    the way): `python3 scripts/diag_pitch_rings.py --height 0.99` → joint fit
>    `pitch θ` and `height H`; a tight fit (< 3 cm spread) prints
>    `Use: --lidar-tilt θ and --lidar-height H`.
> 2. **Tilt + yaw with the box** — box straight ahead, ~4–5 m (a far target pins
>    yaw ~2× tighter): `python3 scripts/diag_birdseye.py --mount-height H
>    --lidar-yaw 1.7 --tilt-sweep 10:16:0.25 --locate-range 6 --box-height B`
>    → flattest tilt (should agree with step 1 within ~0.5°), implied mount
>    height (`H − ground@3.75m`), and the box azimuth → corrected yaw =
>    applied − azimuth.  Re-run with the corrected `--lidar-yaw` until the box
>    reads |azimuth| < ~0.5°.  `--box-height` (tape) checks the box top reads
>    its true height above the floor beside it (vertical scale).
> 3. Set `LIDAR_MOUNT_HEIGHT` in `lidar/obstacle_filter.py` (and the ROS
>    bridge / URDF / launch z) to H, and run with the new `--lidar-yaw` /
>    `--lidar-tilt`.  Perception-only check: `row_follow.py --preset onion` on
>    a bed → `bed≈0.10–0.20m`, `off≈0`.

- **X** = right, **Y** = forward, **Z** = up (sensor frame, after tilt correction)
- Ground-relative height: `h = z_corrected + LIDAR_MOUNT_HEIGHT`
- `LIDAR_MOUNT_HEIGHT = 0.99 m` (defined in `lidar/obstacle_filter.py`; ground to VLP-16 Hi-Res drum centre, robot on wheels — 2026-10 raised mount, tape ~0.98–1.00 m; was 0.80 m).  `--lidar-height M` overrides it for one run (`set_mount_height()` rebinds every importing module); the manifest records `lidar_height_m`
- **LiDAR yaw: 0° (forward-facing, 2026-07 re-mount).** The sensor Y+ axis points along
  robot-forward, so `yaw_correct_pts` is a no-op.  **Verify after ANY mount change** with the
  `diag_birdseye.py` object locator: place a target straight ahead and read its azimuth
  (should be ~0; corrected yaw = applied − straight-ahead azimuth).  Earlier side-yawed mounts
  used ~66–71°.  Implemented in `lidar/obstacle_filter.py::yaw_correct_pts()`.
- **Correction order: YAW FIRST, then TILT** (`row_navigator.py`, `scripts/diag_birdseye.py`).
  The two corrections do NOT commute when yaw ≠ 0: yaw aligns the cloud to the robot frame so
  the subsequent tilt rotates the nose-down PITCH about the robot's left-right (X) axis.  With
  the current yaw=0 mount the order is moot, but the pipeline keeps it for any future side-yaw
  (regression-locked in `test_yaw_then_tilt_recovers_flat_ground` /
  `test_tilt_then_yaw_leaves_residual_ramp`).
- **LiDAR tilt: 15° (verified).** Phone level on the sensor read ~15°, and the raw ring geometry
  (`diag_pitch_rings.py`, pipeline-independent) fits **14.8°** — all 16 channels agree to ±0.1°.
  The ground-slope sweep (`diag_birdseye.py --tilt-sweep 12:18:0.25`) crosses zero at **~15°**.
  **The old "21.5°" was NOT a real pitch** — it was an artifact of the driver using the standard
  VLP-16 ±15° angle table on a unit that is actually a **Puck Hi-Res** (±10°); the stretched
  vertical geometry inflated a true 15° into a phantom 22°.  Fixed in
  `lidar/lidar_driver.py` (Hi-Res table), regression-locked in `tests/test_lidar_driver.py`.
- **(2026-07 mount) `LIDAR_MOUNT_HEIGHT = 0.80 m` — tape-confirmed (robot on wheels), and the raw ring fit agreed
  (0.81 m); raised to ~0.99 m in 2026-10.** Before the Hi-Res driver fix the ground read ~−0.37 m at the flat tilt (the same
  angle-table artifact that inflated the pitch also inflated the apparent height to ~1.17 m);
  with the correct angles the floor now reads h ≈ 0.  Re-derive from `diag_pitch_rings.py` (solves
  pitch AND height from raw geometry) if the mount is ever disturbed.
- Crop geometry (soybean): seedling canopy h ≈ 0.03–0.30 m; tires run in furrows between soybean beds
- Crop geometry (onion): canopy h ≈ 0.10–0.60 m; adjacent row canopy h ≈ 0.70–0.85 m

---

### LiDAR Tilt Correction

**Critical:** The VLP-16 is mounted with a 15° forward (nose-down) tilt. Without correction,
`h = z_sensor + LIDAR_MOUNT_HEIGHT` is wrong by `y × sin(15°)` ≈ 0.52 m at 2 m forward.

**Impact without correction:**
- Soybean at true h=0.15 m appears as h=0.68 m → above crop band max (0.30 m) → ACQUIRE forever
- Ground return at 3 m appears as h=0.80 m → above obstacle threshold (0.50 m) → OBSTACLE_WAIT immediately

**Fix** — `lidar/obstacle_filter.py::tilt_correct_pts(pts, tilt_rad)`:
```
y_world =  y_sensor · cos(θ) + z_sensor · sin(θ)
z_world = −y_sensor · sin(θ) + z_sensor · cos(θ)
```
Applied in `row_navigator.py` after self-filtering, before `detector.update()` and `safety.check()`.

**CLI flag:** `--lidar-tilt DEG` (default: **15.0**) — applied AFTER `--lidar-yaw`.  Verified
via `scripts/diag_birdseye.py --tilt-sweep 12:18:0.25` (ground slope crosses zero at ~15°) and
`scripts/diag_pitch_rings.py` (raw ring geometry, pipeline-independent: 14.8°).  Re-run the
sweep if the mount is disturbed.  (The old 21.5° was an artifact of the driver's wrong ±15°
VLP-16 angle table — this unit is a Puck **Hi-Res** (±10°); fixed in `lidar_driver.py`.)

---

### Tuned Parameters

#### Self-filter (`lidar/obstacle_filter.py` / `--self-radius`)
| Parameter | Value | Rationale |
|---|---|---|
| `self_radius` | **1.5 m** | Robot frame seen at ~0.72 m planar range; 1.5 m clears all body returns without cutting crop ROI (which starts at y=1.5 m) |

#### Row Perception (`navigation/row_perception.py`)
| Parameter | Soybean default | Onion field | Rationale |
|---|---|---|---|
| Crop height band | h ∈ [**0.03, 0.30**] m | h ∈ [0.05, 0.60] m | Soybean seedlings 3–30 cm; onion canopy 5–60 cm |
| Detection mode | **dual-row** (`--dual-row`) | single-row (default) | Soybean: find midpoint between left+right soybean rows; onion: nearest crop row to centreline |
| ROI depth | y ∈ [1.5, 7.0] m | same | Past self-filter; within reliable row geometry |
| ROI half-width | \|x\| ≤ 0.80 m | same | Captures flanking soybean rows (±0.38 m) and excludes outer rows (±1.14 m, just outside the tire tracks); `--roi-x` now bounds the dual-camera tracker too |
| PCA linearity threshold | > 0.20 | same | Below this the cluster is not line-like; confidence forced to 0 |
| Density normaliser | n / 130 pts | same | 130 points = full confidence at typical VLP-16 density |
| Confidence formula | `min(1, n/130) × max(0, min(1, (linearity−0.20)/0.55))` | same | |
| EMA alpha | 0.35 | same | Smooths per-frame jitter without adding excessive lag |
| Heading outlier gate | **30°** | same | Fresh PCA headings that jump >30° from smoothed heading are clamped; prevents ±50° oscillations from sparse/round clusters (row ends, cardboard) |
| Empty-scan decay factor | **0.75** | same | Was 0.5; slower confidence decay on n=0 scans — takes ~5 consecutive empty scans to cross the 0.35 FOLLOW threshold instead of 2 |

#### State Machine (`navigation/row_navigator.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `acquire_conf` | **0.35** | Lowered from 0.45 — soybean seedlings give sparser returns; lower threshold suits the typical 0.40–0.55 confidence range in sparse crops |
| Acquire consecutive frames | 5 | Must see ≥ 5 frames ≥ acquire_conf before entering FOLLOW |
| `obstacle_clear_secs` | **1.5 s** | Consecutive clear time required to leave OBSTACLE_WAIT |
| Min confidence for control | 0.35 | Below this the controller outputs zero velocity |
| `follow_miss_thresh` | **4** | Consecutive sub-threshold scans required to drop FOLLOW→ACQUIRE; stops motion (v=0,ω=0) during gap but stays in FOLLOW; prevents rapid cycling on intermittent empty VLP-16 scans |
| `acq_miss_thresh` | **2** | Consecutive sub-threshold scans required to reset ACQUIRE counter; one isolated empty scan no longer wipes acq=4/5 back to 0 |
| `cam_block_frames` | **3** | Consecutive camera-blocked frames required before OBSTACLE_WAIT; prevents false positives from intermittent depth noise (at 40% hit rate, getting 3 in a row is rare) |

#### Safety Monitor (`navigation/row_safety.py`)
| Parameter | Soybean default | Onion field | Rationale |
|---|---|---|---|
| `forward_dist` | 2.5 m | same | Stopping horizon ahead |
| `forward_half_width` | **0.60 m** | same | Narrower than physical body — prevents triggering on adjacent crop when slightly off-centre |
| `obstacle_height` (forward) | **0.50 m** | 0.75 m | Soybean seedlings h≤0.30 m pass; humans/posts (>0.50 m) stop. Onion: 0.75 m |
| `tire_obstacle_height` | **0.65 m** | 0.85 m | Soybean fields with dried residue stalks: 0.35–0.50 m catches crop material and causes L-TIRE false stops; 0.65 m passes residue/seedlings (h≤0.60 m) while still stopping real hazards. Onion: 0.85 m |
| `tire_track` | **0.959 m** | same | Half of 75.5 in (1.9177 m) wheel-centre-to-wheel-centre; sets L/R-TIRE zone positions |
| `tire_half_width` | 0.25 m | ± corridor around each wheel centreline |
| `tire_dist` | 2.5 m | Same stopping horizon as forward zone |
| `near` | 0.20 m | Ignore returns closer than this (sensor artefacts at beam origin) |
| `forward_min_points` | 4 | Minimum LiDAR returns to trigger forward stop |
| `tire_min_points` | 4 | Minimum LiDAR returns to trigger tire-track stop |

#### Pure-Pursuit Controller (`navigation/row_controller.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `lookahead` | **2.0 m** | Look-ahead distance — 1.0 m causes diverging oscillation when starting >15° off heading |
| `max_linear` | 0.30 m/s | Conservative field speed |
| `min_linear` | 0.08 m/s | Minimum creep speed when turning hard |
| `min_confidence` | 0.35 | Zero output below this |
| Speed formula | `conf × max(0.25, 1.0 − \|θ\|/1.0) × max_linear` | Slows for both low confidence and large heading error |
| `ki` | **0.15** | Leaky-integral gain on the cross-track offset — the disturbance-cancelling term (see below). `ki=0` restores the pure geometric law |
| `i_leak` | **0.94** | Per-step integral leak (~1.6 s memory at 10 Hz) — fast enough to stay damped (no limit cycle in the noiseless closed loop) |
| `i_clamp` | **0.15 rad/s** | Anti-windup cap on the integral's steering contribution |
| `i_rate` | **0.20 m/s** | Integrate only when the offset is PERSISTENT (\|Δoffset\| < i_rate·dt) — engages on a steady slope error, refuses to wind up on a fast transient approach |

**Integral (disturbance-cancelling) action — the "advanced" baseline.** Plain
pure pursuit is memoryless: on a cross-slope it settles with a STANDING
cross-track error because the geometric feedback needs a non-zero offset to
produce the counter-steer that balances a constant drift.  That standing error
is the field slope-weave, and it is why the MPC (disturbance observer) and the
RL policy (drift integrator) were added on top.  The baseline itself now carries
the same idea at its simplest and safest: a **leaky, anti-windup,
confidence-and-rate-gated integral** of the cross-track offset adds a
counter-steer so a steady drift is cancelled and the steady-state offset → ~0.
It is the fallback under BOTH `--controller rl` and `--controller mpc`, so the
whole stack rides on a stronger foundation.  Held-out sim (200 episodes, same
cross-slope disturbances): cross-track RMSE **23.5 → 19.2 cm**, success
**85.5 → 95.3 %** (the integral keeps the robot on the row on steep slopes where
plain pure pursuit ran off), with cleaner flat-ground tracking too and no change
to the noiseless convergence.  Regression-locked in `tests/test_row_controller.py`
(`test_integral_rejects_steady_drift`, `test_ki_zero_is_pure_geometric`).

---

### Full CLI Flag Reference — `scripts/row_follow.py`

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable canbus and send velocity commands to wheels |
| `--rows N` | 1 | Stop after N rows completed (serpentine when combined with `--headland`) |
| `--headland` | off | **Closed-loop** headland U-turns between rows (odometry feedback) |
| `--row-spacing M` | **0.76** | SEED for the in-strip soybean-row separation (detector PRIOR, not a movement param): peak-pairing prior + single-side fallback offset. **Self-calibrating** — the detector refines it from the observed peak separation each scan (live value shown as `sp=N.NNm`), so the default suffices for most fields. The U-turn does NOT use it (perception-closed) |
| `--turn-dir D` | **right** | Direction of the first U-turn (`right`/`left`); subsequent turns alternate, so the sweep moves toward this side. Default = toward the side with more beds when `--beds-left/--beds-right` are given |
| `--beds-left N` / `--beds-right N` | — | Field layout: beds still to cover on each side of the start bed (excluding it; don't count unplanted beds). Picks the first-turn side (more beds), sets `--rows` = 1 + beds on that side (caps an explicit `--rows`), enables `--headland` |
| `--headland-exit M` | **1.0** | Straight distance driven past the row end before the arc; doubles as the blind-spot row-end confirmation (turn aborts back to FOLLOW if crop reappears during it) |
| `--headland-speed M` | **0.15** | Forward speed during the straight headland EXIT phase (m/s) |
| `--headland-turn-rate R` | **0.30** | Angular rate (rad/s) of the U-turn arc; arc forward speed = radius × this rate |
| `--headland-radius M` | **0** (auto 1.0) | U-turn arc radius (m); 0 = auto = 1.0 m. The turn arcs gently and keeps arcing until the LiDAR sees the next row aligned ahead (it does NOT count heading, which a skid-steer over-reports). Bigger = less scrub + slower sweep = easier re-lock |
| `--reacquire-conf C` | **0.72** | Detection confidence (with small heading error + offset, held a few scans) required to end the U-turn and FOLLOW the next row. Raise if it ends early on a misaligned glimpse; lower if it arcs past a good lock |
| `--turn-scrub-comp K` | **0.5** | Skid-steer scrub compensation for the U-turn rotation estimate when the IMU/filter heading is NOT live (`rot=…[wheel]`). Real rotation ≈ wheel heading × K. Too HIGH → turn ends early (under-rotated). Calibrate from the on-screen `rot=` readout: when the robot truly completes 180°, `rot` should read ~180 (heavy-slip fields ~0.45). Ignored when `[imu]` |
| `--approach-speed M` | **0.12** | Forward speed of the post-turn APPROACH leg that drives into the next row until perception re-locks (m/s) |
| `--approach-max-dist M` | **3.0** | Max distance the APPROACH leg searches for the next row before stopping (field-edge / overshoot guard, m) |
| `--post-turn-max-dist M` | **5.0** | Cumulative distance after a U-turn (APPROACH creep + short un-settled FOLLOW segments) before a stable down-row FOLLOW is required; hard guard against driving off the field |
| `--slam` | off | Enable SLAM odometry integration (currently no-op) |
| `--speed M` | 0.30 | Max forward speed m/s |
| `--canopy-clearance M` | **0** (onion **0.15**) | Canopy-adaptive obstacle height (bed mode): an obstacle must rise M above the measured crop canopy top (measured beyond the safety zone), capped 0.95 m forward / 1.0 m tires — tall mature onions don't false-stop; a person still stops. 0 = fixed thresholds |
| `--speed-conf-full C` | **1.0** (onion **0.65**) | Confidence at/above which speed is not reduced: `v = max × min(1, conf/C) × heading factor`. 1.0 = original speed ∝ confidence |
| `--accel-limit A` / `--decel-limit A` | **0.15 / 0.5** | Forward-speed rate limits (m/s²) in FOLLOW/APPROACH; 0 = off. Zero-speed stops and obstacle stops pass through immediately |
| `--lidar-tilt DEG` | **15.0** | Nose-down PITCH correction (degrees), applied AFTER `--lidar-yaw` (2026-07 forward-facing mount). Verify via `diag_birdseye.py --tilt-sweep 12:18:0.25` (ground slope → 0 at ~15°) and `diag_pitch_rings.py` (raw geometry: 14.8°). Old 21.5° was an artifact of the driver's wrong angle table — this is a VLP-16 Puck **Hi-Res** (±10°), fixed in `lidar_driver.py` |
| `--lidar-height M` | **0.99** (code constant) | LiDAR drum-centre height above ground for this run; overrides `LIDAR_MOUNT_HEIGHT` everywhere (crop band, bed floor, safety heights, SLAM). Use the height from `diag_pitch_rings.py` until the constant is updated |
| `--lidar-yaw DEG` | **0.0** | Sensor mount yaw (CCW positive) relative to robot-forward (0 = forward-facing, 2026-07 re-mount; was 66/71 on the side-yawed mount). Applied FIRST (before tilt). Verify with `diag_birdseye.py` object locator (straight-ahead target → azimuth ~0). Re-verify after any mount change |
| `--roi-x W` | 0.80 | Row detection ROI half-width m — applies to BOTH the LiDAR detector and the dual-camera tracker (wired in both scripts) |
| `--crop-min H` | 0.05 | Minimum crop height above ground m |
| `--crop-max H` | 0.60 | Maximum crop height above ground m |
| `--no-ground-detrend` | off | Disable the terrain-adaptive crop band. By default the detector estimates the forward ground grade per scan (low percentile of z per range-bin) and removes its slope so crop stays inside the height band on sloped/undulating fields; the live grade is shown as `grade=±N°` in the status line. Pass this to revert to a fixed flat-ground band |
| `--self-radius R` | **1.5** | Self-filter radius — discard returns within R m (robot body) |
| `--heading-cap DEG` | **7.0** | Near-centre heading-consistency cap (deg). On-strip (small lateral offset) the detected row heading is clamped to this magnitude — a large heading with a small offset is a PCA/terrain artifact, and pure-pursuit chasing it walks the robot across rows on **sloped / tall-bushy-crop** fields. **Lower to 4–5 if the robot drifts sideways and crosses rows on a slope.** Relaxes proportionally toward `--heading-gate` as the offset grows, so genuine off-strip corrections steer fully |
| `--heading-gate DEG` | **22.0** | Heading cap at the lateral gate (deg): the cap ramps from `--heading-cap` (centred) to this value as \|lateral\| reaches 0.22 m; beyond that the heading is unclamped |
| `--no-dense-canopy` | off | Disable the tall/dense-canopy adaptive row fit. By default, when the canopy has closed over and the ground/furrow is no longer visible (≥ `--dense-frac` of the crop band is taller than `--canopy-tall-h`), the detector weights each point by its canopy-height **prominence** so the two taller crop rows dominate and the residue strip re-appears as a gap between them — fixing the lateral/heading/spacing weave that a plateau-shaped histogram causes in dense crop. Early growth is unaffected (weighting only engages in the dense regime; live `DENSE(NN%)` in the status line and `dense`/`tall_frac` in telemetry). Pass to force the plain density fit at all stages |
| `--canopy-tall-h M` | **0.45** | Ground-relative height (m) above which a return counts as "tall canopy" for the dense-regime trigger |
| `--dense-frac F` | **0.35** | Fraction of crop-band points that must be tall (> `--canopy-tall-h`) for the dense-canopy weighted fit to engage. Lower (≈0.25) to engage earlier as the crop fills in |
| `--reliability-floor R` | **0.35** | Dense-canopy runaway/weave guard: when the arbiter self-consistency (`reliability`) drops below this in the dense regime, HOLD the previous heading/lateral instead of chasing the degenerate closed-canopy fit. Good tracking stays ≥0.7 so the hold is inert there. 0 disables |
| `--row-end-veto N` | **200** | A full ROI can never be a row end: when total LiDAR crop `n ≥ N`, row-end confidence is forced to 0 — stops a degenerate dense fit (runaway heading mislocating the flanking-row windows) firing a HEADLAND mid-row. Field logs: false ends n≥647, real ends n≤78. 0 disables |
| `--accumulate N` | **1** | Motion-compensated multi-scan accumulation for the ROW FIT (dense-canopy signal restoration; 1 = off). Registers the last N corrected scans into the current frame via odometry so the closed-canopy residue-strip valley is fit on ~N× the density and the per-scan plateau noise (weave source) averages out. Try 5–8 in dense crop. Safety monitor still uses the raw single scan (obstacles never smeared). Offline A/B with `scripts/replay_scans.py` |
| `--lidar-intensity` | off | Capture per-return LiDAR intensity (0–255) into the recorded scan stream (Nx4) alongside `--save-scans`. In the 903 nm NIR band green canopy and dry residue/soil reflect differently, so intensity is a candidate residue-strip discriminator when the closed canopy flattens the height signal. **Live detection path is unchanged (Nx3)** — intensity rides as a parallel column into the save path only; analyse the saved Nx4 scans with `scripts/replay_scans.py` (reports row−strip intensity contrast + sign consistency) before wiring it into the live fit. Pair with `--record --save-scans` on a dense run |
| `--acquire-conf C` | **0.35** | Min row-detection confidence (0–1) to leave ACQUIRE |
| `--dual-row` | off | Soybean / centre-residue mode: lateral offset = midpoint of left+right flanking crop peaks |
| `--bed-rows N` | off | Raised-bed (onion) mode: steer to the centre of a bed carrying N rows at `--row-spacing` (in-bed spacing). N≥2 → comb fit + bed-top crop band + soil-edge tie-break + row-aligned ROI (implies `--dual-row`); 1 → single centre row. Vidalia: `--preset onion` (= 4 @ 0.279 m) |
| `--preset onion` | — | Vidalia raised bed: `--bed-rows 4 --row-spacing 0.2794 --headland-radius 0.915`, crop band [0.05, 0.60] m above bed top, obstacle 0.75 m, tire 0.85 m |
| `--obstacle-height H` | **0.50** | Min ground-relative height m to count as obstacle in FORWARD zone (soybean default; onion: 0.75) |
| `--tire-height H` | **0.65** | Min height for TIRE-ZONE obstacles (soybean default; 0.35–0.50 causes false L-TIRE stops on dried residue stalks; onion: 0.85) |
| `--camera` | off | Enable OAK-D stereo cameras |
| `--cam-fusion L` | **estimate** | Camera↔LiDAR fusion level: `estimate` = blend independently fitted estimates (original); `point` = pool camera ground points with LiDAR crop points into one weighted row fit (requires `--camera --dual-row`; falls back to `estimate` otherwise) |
| `--cam-left-id S` | "" | Left camera farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right camera farm-ng service name (default: oak1) |
| `--cam-x M` | **0.88** | Camera lateral offset from centreline m (half of 1.76 m inter-camera span; measured) |
| `--cam-stop-dist M` | **2.5** | Camera depth obstacle stop distance m |
| `--cam-block-frames N` | **3** | Consecutive camera-blocked frames required to trigger OBSTACLE_WAIT |
| `--cam-depth-3d` | **off** | 3-D depth fusion through height-filtered SafetyMonitor (default: off — camera at 0.92 m projects near-horizontal views to z ≈ cam_z which exceeds 0.75 m obstacle threshold in empty space; reliable only when cam_z ≤ obstacle_height + 0.1 m) |
| `--no-cam-depth-3d` | — | Disable 3-D depth fusion (default) |
| `--cam-height M` | **0.920** | Camera height above ground (m); measured |
| `--cam-y-fwd M` | **-0.465** | Camera forward offset from LiDAR along robot Y (m); negative = behind LiDAR (measured: 46.5 cm behind) |
| `--cam-yaw-deg DEG` | **0.0** | Yaw from robot forward axis (degrees). 0 = forward-facing (default). Left cam receives negative value, right cam positive. |
| `--cam-pitch-deg DEG` | **15.0** | Downward pitch of camera mount (degrees). Matches VLP-16 nose-down tilt. |
| `--ros2-bridge` | off | Write scan + nav status to `/dev/shm/` at each scan for the Docker ROS 2 bridge |
| `--slam` | off | Build a field map in a background thread while driving (mapping only; no effect on control). Saved on exit |
| `--no-slam` | off | Disable SLAM even under `--record` (skips map / coverage / trajectory). The rest of the record bundle — telemetry, metrics, perception figures, raw scans — is unaffected |
| `--save-scans [EVERY]` | on w/ `--record` (10) | With `--record`, stream the raw corrected LiDAR scans to `runs/<ts>/scans/` (bare = every scan, `N` = every Nth, `0` = off) for offline figure/animation rendering |
| `--map-3d` / `--voxel-3d M` | off / 0.15 | With `--slam`, also build the 3-D point cloud (`map3d.ply`) |
| `--telemetry [PATH]` | off | Log one JSON line per scan (state, conf, crop points, lateral/heading error, row-end conf, grade/drop, command, safety zones, headland turn, `sp`) → `logs/run_<ts>.jsonl`. Load with `pandas.read_json(path, lines=True)`. No effect on control |
| `--record [DIR]` | off | **Record a COMPLETE reproducible experiment folder** (auto-named `runs/run_<controller>_<ts>/` — e.g. `run_rl_...`, `run_mpc_...`, `run_pursuit_...`): bundles `--telemetry` + `--slam` and writes `manifest.json` (git commit + args + calibration), computes `summary.json` + `metrics_flat.csv` + `per_row.csv` + `turns.csv` on exit, plus `coverage.png`/`trajectory.csv`/`map.png`. **Use on every run you want to publish from.** Analyse with `scripts/analyze_run.py` |
| `--debug` | off | Stream LiDAR height histogram + save bird's-eye PNG |
| `--save-dir DIR` | — | Save raw point-cloud numpy arrays to DIR |
| `--no-validate` | off | Skip LiDAR startup health check |

### Full CLI Flag Reference — `scripts/cam_follow.py`

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable motion (default: perception-only) |
| `--rows N` | 1 | Number of rows to cover |
| `--headland` | off | **Closed-loop** headland U-turns between rows (odometry feedback) |
| `--dual-row` | off | Soybean mode: dual-camera ground-projection tracker (overrides `--detector`) |
| `--row-spacing M` | **0.76** | Soybean row spacing / next-strip distance |
| `--roi-x M` | **0.90** | Dual-camera tracker lateral ROI half-width (robot frame); 0.55 on the four-row Vidalia field |
| `--turn-dir D` | **right** | First U-turn direction (`right`/`left`); subsequent alternate |
| `--headland-exit M` | **1.0** | Straight distance past row end before first pivot |
| `--headland-speed M` | **0.12** | Forward speed during straight headland phases |
| `--headland-turn-rate R` | **0.35** | Pivot rate during 90° turns (rad/s) |
| `--speed M` | **0.20** | Max forward speed m/s (lower than LiDAR due to camera latency) |
| `--detector` | `hsv` | `hsv` = HSV green centroid (default); `depth-edge` = colour-independent |
| `--cam-left-id S` | "" | Left OAK-D farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right OAK-D farm-ng service name (default: oak1) |
| `--cam-x M` | **0.88** | Camera lateral offset from centreline m (half of 1.76 m inter-camera span; measured) |
| `--cam-stop-dist M` | **2.5** | Depth obstacle stop distance m |
| `--cam-block-frames N` | **3** | Consecutive camera-blocked frames required to trigger OBSTACLE_WAIT |
| `--acquire-conf F` | **0.20** | Min visual confidence to leave ACQUIRE |
| `--acquire-green F` | **0.08** | Min green fraction (≥8% of strip) to leave ACQUIRE |
| `--fps N` | 10 | OAK-D capture frame rate |
| `--hsv-h-lo H` | 35 | HSV hue lower bound 0–180 (default: green; use ~10 for brown/cardboard) |
| `--hsv-h-hi H` | 85 | HSV hue upper bound 0–180 |
| `--hsv-s-lo S` | 40 | HSV saturation lower bound 0–255 (lower to ~25 for pale colours) |
| `--hsv-v-lo V` | 40 | HSV value lower bound 0–255 |

---

### State Machine

```
         ┌──────────────────┐
    boot  │     ACQUIRE      │  confidence ≥ acquire_conf (0.35)
  ───────>│  5 consecutive   │────────────────────────────────────►┐
       ┌─>│  frames needed   │                                      │
       │  └──────────────────┘                                      │
       │           ▲ obstacle clears (1.5 s consecutive)           ▼
       │           │                                       ┌──────────────┐
       │  ┌──────────────────┐   obstacle detected         │    FOLLOW    │
       │  │  OBSTACLE_WAIT   │◄────────────────────────────│  pure-pursuit│
       │  └──────────────────┘                             │  cmd sent    │
       │                                                   └──────┬───────┘
       │                                                          │ row end
       │  U-turn complete                                         ▼
       │  ┌──────────────────┐   more rows + --headland    ┌──────────────┐
       └──│     HEADLAND     │◄────────────────────────────│   ROW_END    │
          │ EXIT→ARC; arc    │                             └──────┬───────┘
          │ until next row   │   last row (no --headland)         │
          │ aligned ahead    │                                    ▼
          └──────────────────┘                            ┌──────────────┐
                                                          │     DONE     │
                                                          └──────────────┘
```

ROW_END requires: row_dist ≥ row_end_min_dist **and** LiDAR crop-band sparse
(row_end_confidence high) **and** (cameras off **or** green fraction low) — the
camera cross-check stops a single sparse VLP-16 scan from faking a row end while
soybean foliage is still clearly in view.

---

### OAK-D Camera Integration

#### Hardware Connectivity — PoE switch (CRITICAL)

OAK-D cameras are **not USB devices** on the brain. They connect via a **PoE switch** and are
managed exclusively by the `amiga_service` C++ binary. Direct `depthai` access always fails with
`X_LINK_DEVICE_NOT_FOUND` — `amiga_service` holds exclusive camera handles.

`camera/oak_driver.py` uses farm-ng `EventClient` (gRPC) instead of `depthai`.
The `device_id` / `--cam-left-id` / `--cam-right-id` flags specify the farm-ng sub-service
name (e.g. `oak0`, `oak1`), not a depthai MXID serial.

| Item | Value |
|---|---|
| Camera service host | `localhost` |
| Camera service port | **50010** |
| Sub-service for camera 1 | `oak0` (one camera currently connected) |
| Sub-service for camera 2 | `oak1` (times out — hardware not connected; logs one-time warning) |
| Subscription pattern | `SubscribeRequest(uri=Uri(path='/rgb', query='service_name=oak0'))` |
| Image stream paths | `/rgb`, `/disparity`, `/left`, `/right`, `/imu` |
| Frame proto type | `OakFrame` — fields: `meta`, `image_data` (bytes) |
| Image encoding | JPEG bytes — decode with `cv2.imdecode(np.frombuffer(msg.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)` |
| RGB frame size | ~277 KB JPEG → HxWx3 BGR uint8 |
| Disparity encoding | Grayscale JPEG uint8 (~7 KB) → `depth_mm = BASELINE_MM × FOCAL_PX / disparity_px` |
| OAK-D calibration | baseline ≈ 75 mm, focal ≈ 452 px @ 640 px width |

#### oak1 Warning Suppression

`oak1` is not physically connected. The farm-ng EventClient retries the gRPC subscription
every ~0.5 s and logs a WARNING each time. Fix applied in `camera/oak_driver.py`:

- `logging.getLogger("oak/client").setLevel(logging.ERROR)` in `OakDriver.run()`
- `_is_not_found()` detects `NOT_FOUND` / `"no matching topics"` in exception message
- On first NOT_FOUND: print one-time `"[oak_driver:right] service 'oak1' not available"` then `return` (stop retrying)

#### Depth Obstacle Strip Geometry

Both cameras are **forward-facing** at 0° yaw, mounted at 0.920 m height on the left
and right sides of the robot with a 15° nose-down pitch, 46.5 cm behind the LiDAR. Because
they face straight ahead, the robot's forward path is roughly at the **image centre** of each camera.

| Camera | Position | `col_centre_frac` | Why |
|---|---|---|---|
| Left | −0.88 m (left of robot) | **0.5** | Robot path is ahead — image centre |
| Right | +0.88 m (right of robot) | **0.5** | Robot path is ahead — image centre |

The **3-D depth fusion** (`--cam-depth-3d`, default **off**) is disabled by default because the
camera at 0.92 m height projects near-horizontal views to z ≈ cam_z ≈ 0.92 m, which exceeds
the 0.75 m forward obstacle threshold even in empty space, causing persistent false OBSTACLE_WAIT.
It is reliable only if cam_z ≤ obstacle_height + 0.10 m (i.e., camera mounted ≤ 0.85 m high).
The 1-D strip (`col_centre_frac`) is not used by default; safety is LiDAR-only when cameras are on.

#### Camera Block Debounce

A single-frame camera-blocked event is NOT treated as an obstacle. The `cam_block_frames`
counter (default **3**) requires N consecutive blocked frames before `cam_blocked = True`.
At a ~40% intermittent hit rate, 3 consecutive blocked frames is rare, which allows the
`obstacle_clear_secs = 1.5 s` timer to complete and the robot to keep moving.

#### Diagnosing Camera Connectivity

```bash
# Confirm amiga_service is running and port 50010 is open:
ps aux | grep amiga_service
ss -tlnp | grep 50010

# Quick Python subscription test (run in farm-ng venv):
python3 -c "
import asyncio, sys
sys.path.insert(0, '/farm_ng_image/venv/lib/python3.8/site-packages')
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
from farm_ng.core.uri_pb2 import Uri
async def test():
    cfg = EventServiceConfig(name='oak', host='localhost', port=50010)
    req = SubscribeRequest(uri=Uri(path='/rgb', query='service_name=oak0'), every_n=1)
    async for event, msg in EventClient(cfg).subscribe(req, decode=True):
        print('OK — got frame, image_data bytes:', len(msg.image_data))
        break
asyncio.run(test())
"
```

---

### `/dev/shm` IPC Bridge

`row_follow.py --ros2-bridge` calls `_write_bridge()` in `row_navigator.py` on every LiDAR scan:

| File | Content | Format |
|---|---|---|
| `/dev/shm/vidalia_pts.bin` | Current scan point cloud | `int32` n + `float32[n×3]` xyz |
| `/dev/shm/vidalia_status.json` | Nav state + velocities + safety flags | JSON (see below) |

Both files are written atomically (temp file + `os.rename`) to prevent partial reads.

```python
status = {
    "state": self.state,               # "ACQUIRE" / "FOLLOW" / "OBSTACLE_WAIT" / "ROW_END"
    "heading_error": float,            # radians
    "lateral_offset": float,           # metres (positive = right)
    "confidence": float,               # 0–1
    "row_end_confidence": float,
    "n_points": int,
    "linear_vel": float,               # m/s commanded
    "angular_vel": float,              # rad/s commanded
    "forward_blocked": bool,
    "left_tire_blocked": bool,
    "right_tire_blocked": bool,
    "cam_blocked": bool,
    "nearest_forward": float,          # metres (99.0 = no obstacle)
    "rows_done": int,
    "row_dist": float,                 # metres travelled in current row
    "ts": float,                       # monotonic timestamp
}
```

**Why not UDP port-sharing?** Linux `SO_REUSEPORT` with unicast UDP **load-balances** packets
between sockets (does NOT duplicate). Each process would receive only ~half the scans.
`/dev/shm` file IPC was chosen instead — atomic, zero-copy, and accessible from Docker via
`-v /dev/shm:/dev/shm`.

---

### ROS 2 Docker Bridge Details

#### Base Image Choice

| Image | Why |
|---|---|
| `dustynv/ros:foxy-ros-base-l4t-r35.2.1` | Built for Jetson Xavier NX, L4T R35.2.1, Jetpack 5.1; matches exact kernel on Amiga Brain; same approach as farm-ng's own amiga-ros-bridge |
| ~~`osrf/ros:foxy-ros-base`~~ | Generic ARM64; incompatible with Tegra kernel, CUDA libraries, NVIDIA container runtime |

#### Docker Run Flags

| Flag | Purpose |
|---|---|
| `--runtime nvidia` | Activates Jetson NVIDIA container runtime — mounts L4T GPU libraries |
| `--net=host` | Container shares host network namespace — ROS DDS discovery works transparently |
| `-v /dev/shm:/dev/shm` | Shares tmpfs RAM disk between navigation process and container |
| `-p 8765:8765` | Exposes Foxglove WebSocket port to Tailscale network |
| `-e ROS_DOMAIN_ID=42` | Isolates ROS 2 DDS domain from any other ROS instances on the network |

#### Persistent Autostart

`install_autostart.sh` creates `~/.config/systemd/user/vidalia-ros2-bridge.service`.
This path lives on the NVMe and survives reboots. The service starts automatically when
the user logs in (after `loginctl enable-linger` — run once if needed).

```bash
systemctl --user status  vidalia-ros2-bridge
systemctl --user stop    vidalia-ros2-bridge
systemctl --user restart vidalia-ros2-bridge
journalctl --user -u vidalia-ros2-bridge -f
```

#### ROS 2 Topics Published by `vidalia_node.py`

| Topic | Type | Notes |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | xyz + intensity (height-relative); frame `velodyne` |
| `/tf_static` | TF | `base_link → velodyne`: z=0.99 m (`LIDAR_MOUNT_HEIGHT` in `vidalia_node.py` — keep equal to `obstacle_filter.py`), identity rotation (cloud is already yaw+tilt corrected upstream) |
| `/row_viz` | `visualization_msgs/MarkerArray` | Arrow = row direction; green line = lateral offset |
| `/safety_viz` | `visualization_msgs/MarkerArray` | Wireframe boxes: forward zone + tire zones; green = clear, red = blocked |
| `/cmd_vel` | `geometry_msgs/Twist` | Current commanded velocity (linear.x, angular.z) |

---

### Visual Row Detection Detail

`camera/row_detector_visual.py` finds the green-crop centroid in each OAK-D RGB frame:

1. Crop image: middle third horizontally, lower two-thirds vertically
2. Convert BGR → HSV; apply colour mask (configurable via `--hsv-*` flags)
3. If green fraction < `min_green_fraction` (8%) → not valid this frame
4. Centroid pixel → lateral offset via `m_per_px = 2 × depth × tan(hfov/2) / width`
5. **PCA heading** — only computed when `linearity ≥ 0.30`:
   - `vals, vecs = np.linalg.eigh(cov)` — eigenvalue decomposition of green pixel covariance
   - `linearity = (vals[-1] - vals[0]) / (vals[-1] + 1e-6)`
   - When `linearity < 0.30` (round blob, row-end, isolated leaf) → `heading = 0.0`
   - This prevents ±90° heading oscillations when the green region is not elongated
6. EMA-smooth result (alpha=0.30); decay confidence × 0.5 on no-detection frames

Confidence contribution: 0.25 (one camera), up to 0.50 (both cameras, proportional to similarity).
Camera lateral estimate is capped at 50% of LiDAR weight in fusion.

**HSV tuning for different crops/test materials:**
| Material | `--hsv-h-lo` | `--hsv-h-hi` | `--hsv-s-lo` |
|---|---|---|---|
| Onion / green plants | 35 | 85 | 40 |
| Brown cardboard (lab test) | 10 | 25 | 25 |
| Blue markers | 100 | 130 | 40 |

---

### Depth-Edge Row Detector Detail

`camera/row_detector_depth_edge.py` — colour-independent, works in any lighting:

**Heading from edges (`_heading_from_edges`):**
1. Grayscale + Gaussian blur → Canny edge detection
2. HoughLinesP in the lower 2/3 of the image
3. Keep only lines that run primarily vertically (crop row direction)
4. Median slope → remove outliers → mean slope of inliers
5. Convert pixel slope to world heading via FOV scale: `heading = atan(slope_px × hFOV/vFOV)`
6. Returns `(line_conf, heading_rad)`; `line_conf = min(1, n_inliers / 8)`

**Lateral from depth (`_lateral_from_depth`):**
1. Extract horizontal band (middle third to middle third + 40% of height)
2. Per-column mean depth; fill all-NaN columns with global mean
3. Gaussian smooth the column profile
4. `argmax(col_smooth)` = deepest column = inter-row gap candidate
5. **Local-maximum validation**: gap_col must have shallower medians on BOTH flanks (±w/8 margin)
   — rejects open environments where argmax lands at image edge (no crop walls on both sides)
6. `depth_contrast = (gap_depth − mean_smooth) / mean_smooth` must exceed `min_depth_contrast`
7. Returns `(depth_contrast, lateral_offset_m)`

**Validity gate (`_process_side`):**
- A camera side is only valid when `depth_conf > 0` (depth contrast threshold met)
- Hough lines alone (without depth confirmation) are rejected — ambient structural edges
  (wall corners, machinery) bias heading in open environments

EMA smoothing (alpha=0.35); `_decay()` multiplies confidence by **0.75** (not 0.5) each frame with no detection.

---

### Confidence Scoring Detail

Row confidence is computed in `navigation/row_perception.py`:

```
density       = min(1.0, n_crop_points / 130)
linearity     = (lam_major - lam_minor) / (lam_major + lam_minor + 1e-9)   # PCA eigenvalue ratio
linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
raw_conf      = density × linear_factor
smoothed_conf = ema_alpha × raw_conf + (1 − ema_alpha) × prev_conf          # alpha=0.35
```

- `linearity < 0.20` → confidence = 0 (cluster is not line-shaped)
- `linearity = 0.75` → linear_factor = 1.0 (fully linear)
- `n = 130` at `linearity = 0.75` → confidence = 1.0
- In practice, well-centred onion rows give conf ≈ 0.70–0.85 in FOLLOW (up to 1.00 with tilt correction and centred start)

---

### Camera-Only Stack Parameters

**ACQUIRE logic (camera-only):**
Both conditions must hold for `acquire_frames=8` consecutive polls:
- `confidence ≥ acquire_conf (0.20)`
- `green_fraction ≥ acquire_green (0.08)` — at least 8% of the centre strip is the target colour

**Row-end logic (camera-only):**
`green_fraction < row_end_green (0.04)` for `row_end_frames=10` consecutive polls
(robot has driven past the last plant → colour disappears)

**Camera-only confidence range:** 0–0.50 (single-cam max 0.25; dual-cam max 0.50), compared
to 0–1.0 for LiDAR. This is by design — camera geometry is less reliable than 3D point cloud PCA.

**Heading estimation from image PCA:**
```
# Green pixel coords: (col, row) in image
vals, vecs = np.linalg.eigh(cov)          # eigenvalue decomposition
pc = vecs[:, -1]                           # principal component
if pc[1] > 0: pc = -pc                    # orient toward top of image (forward)
linearity = (vals[-1] - vals[0]) / (vals[-1] + 1e-6)
if linearity >= 0.30:                      # only use PCA heading if blob is elongated
    slope_px = pc[0] / max(-pc[1], 0.1)   # px-right per px-forward
    slope_world = slope_px × tan(hfov/2)/w_half / (tan(vfov/2)/h_half)
    heading_rad = atan(slope_world)
```
OAK-D defaults: hfov=73°, vfov=54°, 640×400 → scale factor ≈ 0.91.
**Linearity threshold 0.30** added to prevent ±90° flips when green region is a round blob
(row-end, isolated leaf, cardboard obstacle).

---

### Known Issues and Resolutions

| Symptom | Root Cause | Fix Applied |
|---|---|---|
| Permanent `OBSTACLE_WAIT — FWD@0.7m` | Robot frame returns passing self-filter at 0.72 m | Raised `self_radius` 1.0 → **1.5 m** |
| Permanent `OBSTACLE_WAIT — FWD@1.2m` | `rng >= self_radius` (inclusive) let boundary through | Raising self_radius further to 1.5 m resolves it |
| Permanent `OBSTACLE_WAIT — FWD@2.2m, n=482` | `obstacle_height=0.45 m` below LIDAR_MOUNT_HEIGHT; entire field was "obstacle" | Raised `obstacle_height` 0.45 → **0.75 m** |
| `L-TIRE(n=47)` false positive | Adjacent crop row canopy at h≈0.80–0.84 m triggering tire zone | Added `--tire-height`; set to **0.85 m** for onion fields |
| `L-TIRE(n=13–15)` OBSTACLE_WAIT cycling in soybean field | Dried corn/soybean residue stalks at h≈0.50–0.65 m in the tire corridor; `--tire-height 0.35–0.50` catches crop material | Raised soybean default `--tire-height` **0.35 → 0.65 m**; residue stalks pass, real hazards still stop the robot |
| `safe=clear` shown during OBSTACLE_WAIT | `cam_blocked=True` with empty `cam_reason` set `blocked=True` but `reason()` returned "clear", hiding the trigger in logs | Fixed `reason()` in `row_safety.py`: `if self.cam_blocked:` shows `cam_reason or "CAM"` unconditionally |
| Stuck in ACQUIRE (conf ≤ 0.52) | Robot 0.75 m off-centre → reduced PCA linearity → conf below threshold | Lowered `acquire_conf` 0.55 → **0.45** |
| `CAM-LEFT@1.8m` persistent OBSTACLE_WAIT | Single camera-blocked frame immediately set `cam_blocked = True` | Added `cam_block_frames=3`; requires 3 consecutive blocked frames |
| `WARNING:oak/client: no matching topics` flood | EventClient retries gRPC subscription every ~0.5 s; logs each attempt | Log level → ERROR + one-time "offline" message + return on NOT_FOUND (stop retrying) |
| ACQUIRE forever with 15° tilted mount | `h = z_sensor + height` wrong; crop appears at h=0.83 m (above crop band) | Added `tilt_correct_pts()` in `obstacle_filter.py`; wired via `--lidar-tilt` |
| OBSTACLE_WAIT immediately with 15° tilt | Ground returns appear at h=0.80 m → above obstacle threshold with uncorrected height | Same tilt correction fix |
| `[oak_driver] X_LINK_DEVICE_NOT_FOUND` | Cameras on PoE switch — `amiga_service` holds exclusive depthai handles | Rewrote `oak_driver.py` to use `EventClient` → localhost:50010 |
| Obstacle not detected by cameras (`safe=clear`) | *(historical)* Depth strip at image centre when cameras were side-facing | N/A — cameras are now forward-facing; `col_centre_frac=0.5` for both |
| Heading chaos ±70° at row end | PCA on round green blob → arbitrary principal direction → flips ±90° between frames | Added eigenvalue `linearity ≥ 0.30` check in `row_detector_visual.py` |
| `depth-edge` robot turns right (open room) | `argmax(col_smooth)` found far wall; no right flank for gap validation | Added local-maximum flank validation in `_lateral_from_depth` |
| `depth-edge` robot turns left (structural lines) | Hough detected warehouse edges without depth confirmation | Changed validity gate: `depth_conf == 0 → side invalid` regardless of Hough |
| Ctrl+C traceback during gRPC cleanup | `except Exception` doesn't catch `KeyboardInterrupt` (`BaseException`) | Changed to `except BaseException: pass` in cleanup gather |
| `RuntimeWarning: Mean of empty slice` | `np.nanmean` called on all-NaN column slice | Pre-check valid columns with `~np.all(np.isnan(band), axis=0)` |
| ACQUIRE/FOLLOW/OBSTACLE_WAIT oscillation | Real environmental obstacle at ~2.2–2.5 m intermittently entering forward zone | Not a code bug; resolves in clear onion rows |
| `\r` terminal shows only "clear" | Carriage-return overwrites intermediate blocked frames | Use `--debug` to see every frame; transitions always print with `\n` |
| Rapid FOLLOW→ACQUIRE→FOLLOW cycling | VLP-16 produces 1–3 empty crop-ROI scans/s; 0.5 decay factor crossed 0.35 threshold in 2 scans | Decay factor 0.5→**0.75**; `follow_miss_thresh=4` consecutive misses required to drop to ACQUIRE |
| Heading oscillating ±50° (cardboard/sparse row) | PCA direction flips on nearly-round clusters; EMA alone insufficient to damp | Added 30° outlier gate in `_smooth()` — clamps fresh heading to ≤30° from smoothed when conf>0.30 |
| EKF (`--ekf`) destabilised the loop / added lag | `predict()` motion-model signs were FLIPPED for both lateral (`−v·sinθ`) and heading (`−ω`); every predict step pushed the state away from the robot's actual motion | Corrected to `+v·sinθ·dt` and `+ω·dt`; regression-locked in `tests/test_ekf_estimator.py` |
| Dual-row: robot steers onto the visible soybean row when one side is occluded | `_midpoint_peaks` single-side fallback returned the row peak itself as the target | Fallback now offsets ±`row_spacing/2` inward (mirrors the dual-camera tracker); `--row-spacing` is wired into `RowDetector` |
| Dual-row midpoint dragged off-centre by weeds near the centreline | Innermost-peak pairing locked onto any clutter peak inside the strip | Row-spacing prior: the L/R peak pair whose separation best matches `row_spacing` wins; off-spacing pairs lower confidence |
| Robot steadily drifts forward-right; heading estimate climbs +5°→+14° while the robot turns right | VLP-16 drops whole azimuth sectors (UDP packet loss — `left=0`/`right=0` in validate, scans of 2 578 vs 17 000 pts), so one row stripe is truncated in y; whole-cloud PCA over stripes with unequal extents tilts toward the line joining the stripe centroids → spurious positive heading → pure-pursuit chases it rightward | Cluster-centred two-pass PCA (previously camera-fusion-only) now applied to ALL dual-row fits, iterated 2×, with a 0.25 m peak-distance gate; regression tests `test_dual_row_truncated_stripe_*` |
| Lateral estimate snaps ±half spacing in one scan | Peak pairing changes when a row drops out for one scan; EMA follows the jump | Lateral outlier gate in `_smooth()` — per-scan jump clamped to 0.30 m (mirrors the heading gate) |
| Robot follows the row, then during a correction drifts onto the NEXT strip / crosses rows (worst under RL) | The dual-row target is the midpoint of two rows, but a soybean field is PERIODIC — "on my strip, offset +half" and "on the next strip, offset −half" give two peak pairs that both match the spacing, so `find_row_midpoint` picked one arbitrarily and could alias onto the adjacent strip; an aggressive controller then centred on it. Worse under RL because it reacts fastest to the jumped target | **Continuity ("strip-lock") prior** in `find_row_midpoint`: pair cost = `|sep−spacing| + prior_weight·|midpoint − tracked_lateral|`, so the detector stays locked to the strip it is following — a jump to the adjacent strip costs ≈ one spacing and is rejected unless the robot has truly crossed the half-way point. `RowDetector.midpoint_prior_weight=1.5`; reset→prior 0 so the post-turn re-acquire prefers the nearest (centred) strip. Fixes hopping for pursuit/MPC/RL. Tests `test_strip_lock_*` |
| Robot keeps driving on last twist if LiDAR dies mid-row | `run()` blocked forever inside `async for`; no further commands sent | Scan-stall watchdog: no scan for 0.5 s → actively command zero velocity until the stream recovers |
| Dead camera stream freezes fusion + blocks ROW_END | `get_latest()` returns the final frame forever; frozen green fraction vetoes ROW_END indefinitely | Staleness gate (1.5 s) in both navigators treats old frames as "no camera" |
| Stale EMA from previous row fights re-acquisition after U-turn | Detector EMA accumulates garbage while the ROI sweeps the headland; outlier gates then clamp genuine detections of the next row | `RowDetector.reset()` / tracker `reset()` called on HEADLAND→ACQUIRE |
| Camera tracker ambiguous/biased with forward-facing mounts | Old side-assigned design ("each camera tracks its own side's row" via inner-half column search) assumed inward-looking cameras; forward-facing cameras see BOTH rows, so the column-histogram argmax was ambiguous and depth-scaled pixel offsets were imprecise | Rewrote `soybean_row_tracker.py` as a ground-projection (IPM) tracker: metric robot-frame points per camera, LiDAR-identical fitting via shared `find_row_midpoint`, equal-weight two-camera mean |
| Camera midpoint biased ~0.10 m toward each camera, heading off 2–3° | Whole-cloud PCA over two row stripes with unequal visible extents (outer row's near end outside FOV) tilts the principal axis | Two-pass cluster-centred PCA in `_side_from_mask` — pass 2 centres each row cluster on its own centroid before the PCA |
| Crop vanishes (n collapses ~450→46, FOLLOW→ACQUIRE, robot stops) when the field slopes/undulates | Absolute crop-height band `h ∈ [0.03, 0.30]` assumes flat ground; a fixed mount-tilt correction under-/over-rotates on a grade so the ground (and the crop on it) ramps out of the band (~5° grade shifts a 5 m return by 0.44 m) | Terrain-adaptive band in `RowDetector._ground_slope`: estimate the forward ground slope per scan (20th-pct of z per 0.5 m range-bin, slope-only → robust to furrow/soil bimodality, no-op on flat/canopy-only) and detrend `h_eff = h − slope·y`; `--no-ground-detrend` reverts; live `grade=±N°` in status. Tests `test_graded_field_crop_*` |
| Crop vanishes on a terrain DIP (ROI ground ~0.35 m below h=0; canopy sunk below the 0.03 m floor → only ~14 in-band pts, stall) — distinct from a slope | A uniform level offset (not a ramp): the local ground sits below the flat-calibrated `h=0`, so the absolute band floats above the (correctly elevated) canopy and clips it from below | Level shift in `RowDetector.update`: when a real ground+canopy column is present (ROI height spread > `ground_level_spread`=0.35 m) and its median sits below 0, lower the band by `min(0, P50)` so it rides the local ground; gate keeps canopy-only/flat data on the absolute band; only ever lowers. Live `drop=−N.Nm` in status. Tests `test_terrain_dip_crop_*` |
| Spurious headland turn mid-row on a slope | The row-end escalation fired after only 4 consecutive low-conf scans; a brief crop dropout on a slope (n flickering 0↔700) hit 4 quickly and was misread as a row end | `follow_loss_action`: a real row end needs a LONG CONTINUOUS absence (`row_end_frames`=15); the miss counter resets the instant crop returns, so an intermittent dropout can never reach it. Tests `test_slope_dropout_does_not_turn` |
| Robot stuck after the U-turn ("nothing in front", goes to ACQUIRE and hangs) | The turn ends at the headland margin with no crop yet in the ROI; ACQUIRE is stationary so it never sees the next row | New **APPROACH** state (`row_navigator._step_approach` / `state_logic.approach_action`): after the turn, creep forward at `--approach-speed` until the next row is solidly detected → FOLLOW; stop after `--approach-max-dist` if none found. Tests `test_approach_*` |
| Robot goes FOLLOW→ACQUIRE at the real row end and hangs (never reaches the turn) | The FOLLOW-exit row-end check was fooled: residual sparse clutter (~40 straggler/weed pts) kept `row_end_confidence` below 0.70, and/or a short row left `row_dist` < `row_end_min_dist`, so it fell to the ACQUIRE ("lost lock") branch — and ACQUIRE had no row-end escape, hunting forever for a row that isn't there | **ACQUIRE row-end escape** (`_step_acquire` / `state_logic.acquire_rowend_escape`): when ACQUIRE was entered FROM FOLLOW (we were on a row) and the crop band ahead is empty for `row_end_frames` consecutive scans → ROW_END (→ headland). Tests `test_acquire_escapes_*` |
| Robot THRASHES ACQUIRE↔ROW_END↔HEADLAND↔FOLLOW at sparse/thin spots mid-row (crop drops to n≈50 briefly, robot can't tell a thinning patch from a real row end while stationary) — bounced several times before committing/​re-locking (field: run still finished 3/3 rows + 160 m but with ugly transitions) | A STATIONARY came-from-FOLLOW ACQUIRE cannot distinguish "thinning patch" from "row end" — both read as sparse clutter, and any scan-/confidence-based escape either hangs or turns prematurely | **Creep-through-gap** (`state_logic.search_creep_reached_end`, `_step_acquire`): on losing the row the robot CREEPS slowly forward (`--row-end-search`, 2.0 m) instead of sitting still. A row that merely thinned re-appears within that distance → the acquire counter re-locks it → FOLLOW; a true row end stays empty for the whole creep → ROW_END → headland. Distance-based (robust to the row_end_confidence flicker a scan-counter can't survive); safety-gated (pauses on an obstacle); startup/post-obstacle ACQUIRE stays stationary. Live `SEARCH d/2.0m` in status. Tests `test_creep_*` |
| Headland turn STARTS then thrashes (ROW_END→HEADLAND→EXIT→FOLLOW→ACQUIRE… in a loop, never completes) at a real row end where the crop is sparse | The EXIT blind-spot abort required only `conf ≥ acquire_conf` (0.35) for the reappeared row. At a real row end the sparse residual crop reappears ALIGNED and centred during the 1 m EXIT creep but WEAK (field: conf 0.37, n≈68), which cleared the 0.35 bar → the turn aborted back to FOLLOW every time, then FOLLOW couldn't sustain on the sparse crop → loop | **Strong-lock abort threshold**: the EXIT abort now requires `conf ≥ reacquire_conf` (0.72 — the same bar used to END the U-turn), not `acquire_conf`. Only a genuinely dense row reappearing straight ahead (the true blind-spot gap) aborts; sparse crop/clutter no longer does, so the turn commits and rotates. Tests `test_headland_no_abort_on_sparse_aligned_clutter`, `test_headland_aborts_on_strong_aligned_row` |
| ACQUIRE row-end escape STILL hangs — robot stuck in ACQUIRE at the row end for 16 s, then never turns (goes to ACQUIRE instead of HEADLAND) | The escape counter was reset to 0 on **every** `row_end_confidence` dip. At a real row end the residual sparse clutter (n≈50–70 weed/residue pts after the last plants) intermittently lands near a flanking peak and momentarily reads as "row still here", so `row_end_confidence` FLICKERS above/below 0.70 every couple of scans (field log: conf pinned 0.28–0.34, `end=` bouncing 0.16↔0.91). The counter never reached `row_end_frames` consecutively → escape never fired → hang at `row_dist≈8 m` | **Flicker-robust counter** (`state_logic.rowend_count_update`): reset the confirmation counter ONLY on a genuinely confident re-acquire (`confidence ≥ acquire_conf` — a real row in front of a stationary sensor), increment when the band is empty ahead (`row_end_confidence ≥ row_end_conf`), and HOLD (not reset) on an ambiguous low-conf + low-row-end-conf scan. A `row_end_confidence` dip alone is not evidence of a row, so it no longer restarts the confirmation. Tests `test_rowend_counter_survives_flicker_field_case`, `test_rowend_counter_resets_only_on_confident_reacquire`, `test_rowend_counter_no_false_escape_on_recovering_row` |
| Robot HANGS in ACQUIRE at the row end and never turns (field: stuck 130+ scans at conf ≈0.28, `end` flickering 0.13–0.76, `row_dist` frozen) — a 2nd, subtler form of the flicker-hang | The ACQUIRE→row-end escape counter only incremented when the crop band read "empty" (`row_end_confidence ≥ 0.70`) and HELD otherwise. At this row end the residual sparse clutter (~50 straggler/weed pts) held `row_end_confidence` in a MID range (0.3–0.6, only occasionally ≥0.70), so the counter barely moved and never reached `row_end_frames` | **Count on can't-re-lock** (`state_logic.rowend_count_update`): in a came-from-FOLLOW ACQUIRE, EVERY scan that fails to re-lock a confident row (`confidence < acquire_conf`) now counts toward the escape; only a genuine confident re-acquire resets. The sustained inability to re-lock a row while stationary IS the row-end signal — `row_end_confidence` no longer gates it. A transient dropout still recovers conf and resets. Tests `test_rowend_counter_counts_whenever_it_cannot_relock`, `test_rowend_counter_escapes_on_midrange_clutter_field_case` |
| After the U-turn the robot follows the next row briefly (~0.3 m) then drops to ACQUIRE and hangs | The U-turn→APPROACH→FOLLOW handoff lands on the next row with a marginal, partly-in-ROI view (n≈70 vs ≈700 centred, conf flickering around 0.35); FOLLOW dropped to a *stationary* ACQUIRE, and a stationary sensor on a sparse half-visible row can't improve its view | **Post-turn settling window** (`state_logic.post_turn_loss_action`): from the end of the turn until `post_turn_settle_dist` (2.0 m) of continuous FOLLOW down the new row, a non-row-end FOLLOW loss re-enters **APPROACH** (keep creeping forward) instead of stalling in ACQUIRE — the moving sensor fills the ROI and re-locks; still bounded by `--approach-max-dist`. Status shows `SETTLE`. Tests `test_post_turn_*` |
| U-turn runs all phases on wheel heading but the robot physically rotates only ~90°, ends up pointing across the rows and drives off the field | An in-place pivot on a 4-wheel skid-steer scrubs all wheels, so the wheel-derived `measured_angular_rate` over-reports body rotation — each "90°" pivot finished ~45°, two summed to ~90° not 180° | First tried an **arc U-turn** (less scrub than a pivot) — still closed on wheel heading and still under-rotated (~90°): a moderate-radius arc scrubs enough that "180° of wheel heading" ≈ 90° physical. |
| Arc U-turn STILL only ~90° then bails to APPROACH and gets stuck/drives off | Closing the turn on wheel heading is unfixable on a scrubbing skid-steer (≈2× over-report) | Tried a **perception-closed arc** (keep arcing until the row is seen ahead) — but the open-loop arc under-rotated (slip widens the real radius) AND the detector locked onto **grass** at ~90° (can't tell grass from soybean), ending the turn early on the wrong feature. |
| Perception-closed arc turns ~90° then "follows" grass (soybean rows not there) | (a) open-loop arc under-rotates — slip makes the real radius ~1.7× the commanded 1.0 m, so the arc length at "done" was only ~90° of rotation; (b) the dual-row detector locks onto grass at the headland (conf ~0.65) and the perception gate accepted it | **IMU-measured turn** (`headland.py` accumulates `FilterState` heading change during the arc, regardless of `has_converged` — IMU yaw is locally accurate without GPS lock). `row_navigator._step_headland`: keep arcing until ~180° of REAL rotation; a perception lock may end the turn ONLY after ≥150° measured rotation AND conf ≥ 0.72 (so grass at ~90° / conf 0.65 can't); complete on heading alone at 175° → APPROACH. Fallback to arc-length window (4 m) + strict perception if the IMU isn't live. Simulated at 55 % slip + grass-the-whole-arc it ends at true ~170° on the real row. Tests `test_imu_heading_*`, `test_finish_*`, `test_arc_length_cap_*` |
| Robot drives off the end of the field after a turn, "following" spurious crop returns | The post-turn settling guard only bounded the APPROACH creep; a confident-but-wrong long FOLLOW lock (or repeated ping-pong) escaped the bound | **Cumulative post-turn guard** (`post_turn_max_dist`, 5.0 m): bounds total travel after a U-turn (APPROACH + short FOLLOW segments) before a stable down-row FOLLOW is required, else STOP |
| Headland U-turn ABORTS immediately and never turns; robot drives diagonally across the headland (field: at a real row end it toggled ROW_END→HEADLAND→FOLLOW repeatedly, then followed clutter at hdg −13°/lat +0.36) | The blind-spot EXIT abort fired whenever ANY crop reappeared at `conf ≥ acquire_conf`, with NO alignment check. At a real row end a dense field still returns crop — headland margin, adjacent rows, the just-ended row seen at an angle — but MISALIGNED (log: conf 0.5 at hdg +34–55°, lat −0.40). The guard read that as "row continues" and bailed out of every turn | **Alignment-gated abort** (`state_logic.headland_exit_row_continues`): the EXIT leg aborts back to FOLLOW only if the reappeared row is genuinely ALIGNED and CENTRED (`|heading| ≤ reacquire_align`, `|lateral| ≤ reacquire_offset`, conf ≥ acquire_conf) — the real blind-spot case where the same row returns where the robot was already pointing. Misaligned headland clutter no longer aborts, so the turn commits. Tests `test_headland_no_abort_on_misaligned_clutter`, `test_headland_aborts_on_genuine_aligned_row` |
| Row end declared early on the LiDAR near blind spot (< 1.5 m) | The last plants in the near zone leave the ROI sparse, reading as a row end before the row truly ends | **Blind-spot EXIT confirmation** (`_step_headland`): the turn's straight EXIT leg aborts back to FOLLOW if solid crop reappears in the ROI before any rotation (`headland_abort_frames`); a real obstacle still pauses the leg via the safety monitor |
| Robot weaves / hops rows in TALL DENSE canopy (lateral ±0.1–0.29 m, heading ±5–22°, `sp` bouncing 0.60↔0.76) though conf is healthy (~0.75, n~1260) | The dual-row fit keys the two flanking rows off a DENSITY contrast — rows carry crop returns, the furrow/strip between them is bare GROUND. Once the canopy (~0.70–0.75 m) closes over, the crop band (h∈[0.03,1.00]) fills solid with leaves/stems and almost no ground shows, so the cross-row histogram is a PLATEAU: peak-pairing jitters and lateral/heading/spacing weave. Affects pursuit/RL/MPC equally (all read the same RowEstimate) | **Growth-stage-adaptive fit** (`RowDetector`, `dense_canopy=True`): when ≥ `dense_canopy_frac` (0.35) of the crop band is taller than `canopy_tall_h` (0.45 m), weight every point by its canopy-height **prominence** (height above the strip-floor percentile) so the taller row RIDGES dominate and the residue strip re-appears as a canopy-height VALLEY between them — reusing the whole existing pipeline (weighted `histogram_peaks` / `find_row_midpoint` spacing-prior + strip-lock / cluster-centred PCA), just a per-point weight. Early growth is byte-identical (weighting never engages). Live `DENSE(NN%)` in status, `dense`/`tall_frac` in telemetry. `--no-dense-canopy` / `--canopy-tall-h` / `--dense-frac`. Tests `test_dense_canopy_*` |
| (Robustness upgrade to the above) A single hard `tall_frac ≥ 0.35` threshold decides density-vs-canopy fit — brittle at the transition, and blindly trusts the canopy fit even if it is momentarily worse | **Self-consistency arbiter** (mixture-of-experts, `RowDetector._dual_fit`): in the dense regime the detector fits BOTH experts (ground-density + canopy-height) each scan and keeps whichever has the higher **self-consistency** = PCA linearity × spacing-prior pairing × agreement with the tracked strip. The canopy expert bootstraps the correct strip during acquisition/disturbance (first-scan `mode=canopy`); once locked, whichever is cleaner holds. Emergent growth-stage handling, no brittle threshold; the winning `mode`/`reliability` are in telemetry and `DENSE(NN%)→mode` in status. Extensible slot for a LEARNED gate trained on `--record` data. Tests `test_arbiter_*` |
| One noisy scan (row end / VLP-16 dropout / transient occlusion) jerks the steering | Fixed-gain EMA blends every scan equally regardless of quality | **Confidence-weighted temporal filter** (`_smooth`, `temporal_trust=True`): scale the per-scan EMA gain by the fresh scan's confidence relative to a healthy lock (floored at `temporal_min_gain`), so a low-confidence scan nudges the tracked lateral/heading less and the estimate leans on recent history. Confidence itself still updates at the base rate so the state machine reacts. (Does not mask the dense-canopy HIGH-confidence weave — that is fixed at source.) Tests `test_temporal_filter_*` |
| Tall crop a few metres ahead on a SLOPE read as an obstacle → false forward stop in clear rows | The safety height test uses the fixed-tilt-corrected `h = z + mount`; on a grade flat ground (and the crop on it) ramps up with range, so crop just below the threshold on flat ground reads above it a few metres ahead on an up-slope | **Forward ground-grade detrend** in `SafetyMonitor.check(pts, ground_slope=…)`: subtract the detector's per-scan ground ramp (`h − slope·y`) from the height test so slope-induced apparent height is cancelled, while a REAL obstacle (rises above local ground) still exceeds the threshold. Recovers only the slope margin — crop genuinely taller than the threshold still needs `--obstacle-height` raised. 0 on flat → byte-identical. Tests `test_*_on_slope_*` |
| RL controller **whips the robot ~90° into the crop wall** at a sparse/gappy patch (repeated OBSTACLE_WAIT; operator had to take over) — even with `--rl-slew 0.12` active | At a thinning patch (~72 m, `tall_frac`→0.03, n 517→706) the dual-row detector produced a SELF-CONSISTENT but WRONG fit reporting heading **+68°→+86° AND lateral −0.42 m together**. That combination defeats both guards: the reliability hold didn't fire (the bad fit's `reliability` recovered to ~0.49, above the 0.35 floor) and the heading-consistency cap relaxes once \|lateral\| passes its gate (0.22 m). The reactive RL then held a hard turn for ~2.5 s (the slew bound correctly slowed the *onset* to ~0.11 rad/step but cannot stop a *sustained* wrong turn). Field logs: 40 FOLLOW scans hit \|hdg\|>35°, max 87° | **Absolute heading ceiling** (`heading_abs_max`=35°, `--heading-max`) applied in `_smooth` UNCONDITIONALLY (regardless of lateral): a row-following heading beyond ~35° is non-physical — you never approach a crop row at 60–86°, so it is a bad fit in a sparse patch, not a real angled approach. Clamps it so no controller (especially RL) can be driven to turn violently on a garbage estimate. Normal following (±5°) and genuine off-strip recovery (≲ atan(offset/lookahead) ≈ 10–20°) are far below the ceiling, so it is inert there. Helps MPC/pursuit too (MPC's −55° sparse-tail excursion). Tests `test_absolute_heading_ceiling_clamps_nonphysical_fit` |
| Robot took a HEADLAND U-turn **mid-row** in dense closed canopy (pursuit at 101 m; MPC & RL within a few m of a re-lock) — "weird mid-row turn" | In the fully-closed canopy the ground/furrow is barely visible, so the DENSITY expert's cross-row histogram is a plateau and its PCA heading is ill-conditioned. Field logs show the fit's heading spiralling MONOTONICALLY +8°→+61° over ~1.5 m while the lateral drifted 0.1→0.43 m and the arbiter self-consistency (`reliability`) collapsed to ~0.1 — **with crop clearly still present (n≈800)**. The runaway lateral/heading mislocated the flanking-row windows, so `row_end_confidence` spuriously read 1.0 and fired a headland. The per-scan 30° heading gate does not stop a slow 4°/scan ramp, and the heading-consistency cap relaxes once \|lateral\| passes its gate (0.22 m) — the exact positive-feedback hole the runaway drives through | **Two independent guards** in `RowDetector`: (1) **Row-end density veto** (`row_end_veto_density`=200, `--row-end-veto`) — a full ROI can never be a row end, so when total LiDAR crop `n ≥ 200` the row-end confidence is forced to 0 (field logs separate cleanly: false mid-row ends had n≥647, real row ends n≤78). (2) **Reliability-gated hold** (`reliability_floor`=0.35, `--reliability-floor`) — in the dense regime, when the arbiter `reliability` drops below the floor the closed-canopy fit is degenerate, so HOLD the previous smoothed heading/lateral instead of chasing it (good tracking stays ≥0.7, p10 0.73 on a clean 142 m run, so the hold is inert there). Confidence still reports the degraded lock so the state machine reacts. Fixes the mid-row headland **and** the dense left/right weave at source for pursuit/MPC/RL alike. Tests `test_row_end_vetoed_when_roi_full_of_crop`, `test_reliability_hold_*` |

---

### LiDAR Self-Filter Logic

The scan parser in `lidar/obstacle_filter.py` discards any return whose **planar range**
(distance in the XY plane, ignoring height) is less than `self_radius`. This is critical:

- The Amiga robot frame generates LiDAR returns at ~0.72 m planar range
- The crop ROI starts at y = 1.5 m, so `self_radius = 1.5 m` does not cut into crop detection
- The safety monitor's `near = 0.20 m` provides a second guard for beam-origin artefacts

The tilt correction is applied **after** self-filtering (planar range is nearly tilt-invariant
at 15° — only 3.4% difference — so the self-filter threshold does not change with tilt).

`validate_lidar_startup()` uses the **raw scan** (without self-filter) for startup diagnostics only.

---

## ROS 2 Stack (dev PC reference)

### TF Tree
```
map
 └── odom          (rtabmap via ICP odometry or EKF)
      └── base_link (icp_odometry or ekf_filter_node)
           └── velodyne  (static: x=1.130 m, z=0.990 m, from robot_state_publisher)
```

### Full Autonomous Stack (localize_nav.launch.py)

```
Amiga brain (Tailscale: camphor-clone.tail0be07.ts.net)
    │  canbus :6001  →  AmigaTpdo1 (wheel velocity, state)
    │  filter :20001 →  FilterState (GPS+IMU pose, heading)
    │
    ▼  amiga_ros2_bridge  (native ROS 2 gRPC, farm-ng OS 2.0 SDK)
         /amiga/vel  (TwistStamped)   ← measured speed
         /amiga/pose (Odometry/world) ← GPS pose (when converged)
         /cmd_vel    (Twist)          → Twist2d via request_reply("/twist")
    │
    +── amiga_odometry    /amiga/vel → /wheel_odom (dead-reckoning)
    │
    ▼  Velodyne VLP-16 (192.168.1.201:2368)
         /velodyne_packets → /velodyne_points (PointCloud2, 10 Hz)
    │
    +── icp_odometry      /velodyne_points → /odom + odom→base_link TF
    +── rtabmap           localization-only; map→odom TF; /map OccupancyGrid
    │
    └── Nav2
         controller_server   Regulated Pure Pursuit; max 1.5 m/s; 30 cm goal tolerance
         planner_server      NavFn/A*; 10 cm resolution
         behavior_server     spin / back-up / wait recoveries
         bt_navigator        NavigateToPose + FollowWaypoints BT actions
         waypoint_follower   /follow_waypoints action (field row traversal)
         lifecycle_manager   activates all Nav2 nodes
```

### ROS 2 Run Commands (dev PC only)

```bash
# Mapping (new field):
ros2 launch vidalia_bringup slam_nav.launch.py database_path:=~/maps/field.db rviz:=true
bash scripts/save_map.sh ~/maps/field

# Autonomous operation (known field):
ros2 launch vidalia_bringup localize_nav.launch.py database_path:=~/maps/field.db rviz:=true

# Individual sub-stacks:
ros2 launch vidalia_bringup sensors_live.launch.py
ros2 launch vidalia_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
ros2 launch vidalia_bringup nav2.launch.py
ros2 launch vidalia_bringup amiga_bringup_nodes.launch.py use_ekf:=true
ros2 launch vidalia_bringup slam_localization.launch.py database_path:=~/maps/field.db

# Tests:
colcon test --packages-select vidalia_bringup && colcon test-result --verbose
```
