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

### LiDAR Row Follow — Onion Field (single raised crop row)

```bash
# Perception-only — robot stays still, verify detection first:
python3 scripts/row_follow.py --crop-max 0.60 --obstacle-height 0.75 --tire-height 0.85

# Autonomous (robot WILL move):
python3 scripts/row_follow.py --auto --crop-max 0.60 --obstacle-height 0.75 --tire-height 0.85

# Autonomous with OAK-D cameras:
python3 scripts/row_follow.py --auto --crop-max 0.60 --obstacle-height 0.75 --tire-height 0.85 --camera

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
| `navigation/headland.py` | **Closed-loop U-turn** driver (odometry feedback); EXIT→TURN_A→SHIFT→TURN_B |
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

**End-of-row U-turn** (`HeadlandTurn`, closed-loop) — ⚠ not yet field-validated:

```
ROW_END ─► HEADLAND ──────────────────────────► APPROACH ──► FOLLOW (next row)
            EXIT  (drive exit_dist straight, clear last plants)
            TURN_A(pivot 90° toward next row — heading feedback)
            SHIFT (drive headland_shift straight to the next strip)
            TURN_B(pivot 90° same direction — now aligned down next row)
                                                  └─ APPROACH: creep forward into
                                                     the next row until perception
                                                     re-locks, then hand to FOLLOW
```

**APPROACH leg (critical for the loop to close):** the U-turn ends at the
headland margin *pointing down the next row but with no crop yet in the ROI*.
Entering a stationary ACQUIRE there hangs forever (field failure: robot stuck
after the turn with "nothing in front").  Instead the navigator enters
**APPROACH** — it creeps forward at `--approach-speed` (0.12 m/s) until the next
row is solidly detected (`acquire_conf` for a few scans), then hands off to
FOLLOW, whose pure-pursuit corrects any residual lateral error.  Bounded by
`--approach-max-dist` (3.0 m): if no row is found in that distance (true field
edge, or the turn overshot) the robot STOPS rather than driving on blindly.
Status shows `ENTER 1.2/3.0m acq=2/3`.

Turn direction alternates each row for **serpentine coverage** (right, left,
right, …), starting from `--turn-dir` (default right).  The loop repeats until
`--rows N` rows are complete, then DONE.

**Pivot heading source (`navigation/filter_heading.py`):** the two 90° pivots
turn IN PLACE, where a 4-wheel skid-steer scrubs and slips, so wheel-integrated
heading (AmigaTpdo1 `measured_angular_rate`) drifts — a 90° pivot can finish
10–20° off.  The turn therefore prefers the **filter service absolute heading**
(GPS+IMU `FilterState`, port 20001) when it is fresh and converged, falling back
to wheel-odometry heading otherwise.  The choice is **latched at the start of
each turn** so a pivot never mixes two heading references.  The active source is
shown in the status line as `R-UTURN:TURN_A[filter]` / `[wheel]`.  Straight
EXIT/SHIFT distances always use wheel odometry (accurate for straight driving).

**SHIFT distance vs row spacing — two different numbers:** `--headland-shift`
(default **1.52 m**) is the centre-to-centre distance to the NEXT strip the
robot straddles; `--row-spacing` (default **0.76 m**) is the in-strip soybean-row
separation the detector uses for peak pairing.  Do not conflate them — the turn
shifts by `--headland-shift`, the detector pairs rows by `--row-spacing`.

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

- **X** = right, **Y** = forward, **Z** = up (sensor frame, after tilt correction)
- Ground-relative height: `h = z_corrected + LIDAR_MOUNT_HEIGHT`
- `LIDAR_MOUNT_HEIGHT = 0.75 m` (defined in `lidar/obstacle_filter.py`; measured: ground to VLP-16 drum centre)
- **LiDAR yaw: 66° CCW (re-calibrated 2026-06; was 71°).** A mount disturbance (lens cleaning)
  rotated the sensor ~5°; re-measured with the `diag_birdseye.py` forward-object locator — a
  bucket placed straight ahead read azimuth +5.4° at yaw 71, and 0.0° at yaw 66 (corrected
  yaw = applied_yaw − straight-ahead-azimuth).  **Re-verify after ANY mount change**:
  `python3 scripts/diag_birdseye.py --range 3` and read the straight-ahead object's azimuth
  (should be ~0).  The pitch is independent and stayed 21.5°.  Implemented in
  `lidar/obstacle_filter.py::yaw_correct_pts()`.
- **Correction order: YAW FIRST, then TILT** (`row_navigator.py`, `scripts/diag_birdseye.py`).
  The two corrections do NOT commute when yaw ≠ 0: yaw aligns the cloud to the robot frame so
  the subsequent tilt rotates the nose-down PITCH about the robot's left-right (X) axis.
  Applying tilt first rotates about the un-yawed sensor X axis (~66–71° off) and leaves a >1 m
  residual ground ramp (regression-locked in `test_yaw_then_tilt_recovers_flat_ground` /
  `test_tilt_then_yaw_leaves_residual_ramp`).
- **LiDAR tilt: 21.5° (field-calibrated; default).** The 15° body lean the phone level measured
  is a roll about the sensor's *own* forward axis; because the sensor is yawed ~66°, that lean
  resolves IN THE ROBOT FRAME into a **~21.5° nose-down PITCH** (the eyeballed 15° was the body
  lean, not the robot-frame pitch).  Without it, flat ground reads h≈0 near the robot but
  h≈1.4 m at 6 m — that ramp is the uncorrected pitch.  Calibrated with
  `python3 scripts/diag_birdseye.py --tilt-sweep 20.5:22.5:0.25`: the ground SLOPE crosses zero
  at **21.5°** (slope +0.002).  The slope is mount-height-independent, so it pins the pitch
  unambiguously.  **Earlier "tilt 0 is correct / tilt 15 drives everything below ground" note was
  an artifact of the OLD tilt→yaw order and a missing yaw correction** — `tilt 15` then yawing
  over-rotated returns to h<0, misread as "no pitch needed".
- **`LIDAR_MOUNT_HEIGHT = 0.75 m` is correct — tape-confirmed (~74 cm).** The sweep's secondary
  "ground level" metric reads ~−0.37 m at the flat tilt, but that is NOT a mount-height error:
  it is the 15th-percentile tracking the **furrow bottoms**, which sit ~0.3 m below the crop bed
  (raised-bed / onion-bed geometry).  Crop on the bed correctly reads h ∈ [0.05, 0.30 m] with
  mount = 0.75 m.  Re-derive pitch with the slope (mount-independent), not the ground level.
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

**CLI flag:** `--lidar-tilt DEG` (default: **21.5**) — applied AFTER `--lidar-yaw`.  Field-calibrated
via `scripts/diag_birdseye.py --tilt-sweep 20.5:22.5:0.25` (ground slope crosses zero at 21.5°).
Re-run the sweep if the mount is disturbed.

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

---

### Full CLI Flag Reference — `scripts/row_follow.py`

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable canbus and send velocity commands to wheels |
| `--rows N` | 1 | Stop after N rows completed (serpentine when combined with `--headland`) |
| `--headland` | off | **Closed-loop** headland U-turns between rows (odometry feedback) |
| `--row-spacing M` | **0.76** | In-strip soybean-row separation: LiDAR dual-row peak-pairing prior + single-side fallback spacing, and the dual-camera single-row fallback spacing. NOT the headland shift distance |
| `--headland-shift M` | **1.52** | Centre-to-centre distance the U-turn SHIFTs to the next strip the robot straddles (distinct from `--row-spacing`) |
| `--turn-dir D` | **right** | Direction of the first U-turn (`right`/`left`); subsequent turns alternate |
| `--headland-exit M` | **1.0** | Straight distance driven past the row end before the first pivot |
| `--headland-speed M` | **0.15** | Forward speed during straight headland phases (m/s) |
| `--headland-turn-rate R` | **0.35** | Pivot rate during the two 90° turns (rad/s) |
| `--approach-speed M` | **0.12** | Forward speed of the post-turn APPROACH leg that drives into the next row until perception re-locks (m/s) |
| `--approach-max-dist M` | **3.0** | Max distance the APPROACH leg searches for the next row before stopping (field-edge / overshoot guard, m) |
| `--slam` | off | Enable SLAM odometry integration (currently no-op) |
| `--speed M` | 0.30 | Max forward speed m/s |
| `--lidar-tilt DEG` | **21.5** | Nose-down PITCH correction (degrees), applied AFTER `--lidar-yaw`. The body lean resolves into a ~21.5° robot-frame pitch once the yaw is corrected; field-calibrated via `diag_birdseye.py --tilt-sweep 20.5:22.5:0.25` (ground slope → 0 at 21.5°, unchanged across the yaw re-calibration) |
| `--lidar-yaw DEG` | **66.0** | Sensor mount yaw (CCW positive) relative to robot-forward (re-calibrated 2026-06 via the `diag_birdseye.py` object locator; was 71). Applied FIRST (before tilt). Re-verify after any mount change |
| `--roi-x W` | 0.80 | Row detection ROI half-width m — applies to BOTH the LiDAR detector and the dual-camera tracker (wired in both scripts) |
| `--crop-min H` | 0.05 | Minimum crop height above ground m |
| `--crop-max H` | 0.60 | Maximum crop height above ground m |
| `--no-ground-detrend` | off | Disable the terrain-adaptive crop band. By default the detector estimates the forward ground grade per scan (low percentile of z per range-bin) and removes its slope so crop stays inside the height band on sloped/undulating fields; the live grade is shown as `grade=±N°` in the status line. Pass this to revert to a fixed flat-ground band |
| `--self-radius R` | **1.5** | Self-filter radius — discard returns within R m (robot body) |
| `--acquire-conf C` | **0.35** | Min row-detection confidence (0–1) to leave ACQUIRE |
| `--dual-row` | off | Soybean / centre-residue mode: lateral offset = midpoint of left+right flanking crop peaks |
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
          │ EXIT→TURN_A→     │                             └──────┬───────┘
          │ SHIFT→TURN_B     │   last row (no --headland)         │
          │ (odometry loop)  │                                    ▼
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
| `/tf_static` | TF | `base_link → velodyne`: z=0.705 m, pitch=−15° (tilt corrected) |
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
| Robot keeps driving on last twist if LiDAR dies mid-row | `run()` blocked forever inside `async for`; no further commands sent | Scan-stall watchdog: no scan for 0.5 s → actively command zero velocity until the stream recovers |
| Dead camera stream freezes fusion + blocks ROW_END | `get_latest()` returns the final frame forever; frozen green fraction vetoes ROW_END indefinitely | Staleness gate (1.5 s) in both navigators treats old frames as "no camera" |
| Stale EMA from previous row fights re-acquisition after U-turn | Detector EMA accumulates garbage while the ROI sweeps the headland; outlier gates then clamp genuine detections of the next row | `RowDetector.reset()` / tracker `reset()` called on HEADLAND→ACQUIRE |
| Camera tracker ambiguous/biased with forward-facing mounts | Old side-assigned design ("each camera tracks its own side's row" via inner-half column search) assumed inward-looking cameras; forward-facing cameras see BOTH rows, so the column-histogram argmax was ambiguous and depth-scaled pixel offsets were imprecise | Rewrote `soybean_row_tracker.py` as a ground-projection (IPM) tracker: metric robot-frame points per camera, LiDAR-identical fitting via shared `find_row_midpoint`, equal-weight two-camera mean |
| Camera midpoint biased ~0.10 m toward each camera, heading off 2–3° | Whole-cloud PCA over two row stripes with unequal visible extents (outer row's near end outside FOV) tilts the principal axis | Two-pass cluster-centred PCA in `_side_from_mask` — pass 2 centres each row cluster on its own centroid before the PCA |
| Crop vanishes (n collapses ~450→46, FOLLOW→ACQUIRE, robot stops) when the field slopes/undulates | Absolute crop-height band `h ∈ [0.03, 0.30]` assumes flat ground; a fixed mount-tilt correction under-/over-rotates on a grade so the ground (and the crop on it) ramps out of the band (~5° grade shifts a 5 m return by 0.44 m) | Terrain-adaptive band in `RowDetector._ground_slope`: estimate the forward ground slope per scan (20th-pct of z per 0.5 m range-bin, slope-only → robust to furrow/soil bimodality, no-op on flat/canopy-only) and detrend `h_eff = h − slope·y`; `--no-ground-detrend` reverts; live `grade=±N°` in status. Tests `test_graded_field_crop_*` |
| Crop vanishes on a terrain DIP (ROI ground ~0.35 m below h=0; canopy sunk below the 0.03 m floor → only ~14 in-band pts, stall) — distinct from a slope | A uniform level offset (not a ramp): the local ground sits below the flat-calibrated `h=0`, so the absolute band floats above the (correctly elevated) canopy and clips it from below | Level shift in `RowDetector.update`: when a real ground+canopy column is present (ROI height spread > `ground_level_spread`=0.35 m) and its median sits below 0, lower the band by `min(0, P50)` so it rides the local ground; gate keeps canopy-only/flat data on the absolute band; only ever lowers. Live `drop=−N.Nm` in status. Tests `test_terrain_dip_crop_*` |
| Spurious headland turn mid-row on a slope | The row-end escalation fired after only 4 consecutive low-conf scans; a brief crop dropout on a slope (n flickering 0↔700) hit 4 quickly and was misread as a row end | `follow_loss_action`: a real row end needs a LONG CONTINUOUS absence (`row_end_frames`=15); the miss counter resets the instant crop returns, so an intermittent dropout can never reach it. Tests `test_slope_dropout_does_not_turn` |
| Robot stuck after the U-turn ("nothing in front", goes to ACQUIRE and hangs) | The turn ends at the headland margin with no crop yet in the ROI; ACQUIRE is stationary so it never sees the next row | New **APPROACH** state (`row_navigator._step_approach` / `state_logic.approach_action`): after the turn, creep forward at `--approach-speed` until the next row is solidly detected → FOLLOW; stop after `--approach-max-dist` if none found. Tests `test_approach_*` |

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
           └── velodyne  (static: x=1.130 m, z=0.800 m, from robot_state_publisher)
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
