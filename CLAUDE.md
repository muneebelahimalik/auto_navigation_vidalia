# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous navigation workspace for the farm-ng Amiga robot using a Velodyne VLP-16 LiDAR
and two OAK-D stereo cameras. Goal: fully autonomous laser-based weed control in onion fields.

Two parallel stacks are maintained:

| Stack | Entry point | When to use |
|---|---|---|
| **Native Python** (recommended on brain) | `scripts/row_follow.py` | Amiga brain (camphor-clone) — ROS 2 not installed |
| **Camera-only** (LiDAR-free backup) | `scripts/cam_follow.py` | When LiDAR unavailable or for lightweight deployment |
| **ROS 2 Foxy** (dev PC / future) | `ros2 launch vidalia_bringup …` | Development PC with ROS 2 Foxy installed |

**System — Amiga brain (camphor-clone)**:
- OS: Ubuntu 20.04.6 LTS (Focal Fossa), ARM64
- Python: 3.8.10
- **ROS 2 Foxy is NOT installed** on the brain. The base image uses an overlay
  filesystem; anything installed outside `~/` (including `/opt/ros/`) is wiped on reboot.
- The farm-ng Python SDK (OS 2.0 / Barley) is pre-installed at `/farm_ng_image/venv/`.
- All code, configs, and installs must live under `~/` (NVMe, persistent).
- depthai installed: **2.22.0.0** (not 2.23+)
- opencv-python installed: **4.7.0.68**

**System — Development PC**:
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- ROS 2 not yet installed (install Humble for RViz/bag recording if needed).
- SSH access: `ssh farm-ng-user-laserweeding@100.66.121.56` (Tailscale)

**Ultimate goal**: fully autonomous laser-based weed control in onion fields.

---

## Build & Run Commands

All commands assume the workspace root is `~/auto_navigation_vidalia`.

### One-time setup (on the brain)

```bash
# Activate the pre-installed farm-ng venv (required every session):
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia
```

### LiDAR Row Follow (primary)

```bash
# Perception-only — robot stays still, verify detection first:
python3 scripts/row_follow.py

# Autonomous (robot WILL move):
python3 scripts/row_follow.py --auto --tire-height 0.85

# Autonomous with OAK-D cameras:
python3 scripts/row_follow.py --auto --tire-height 0.85 --camera

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
| `navigation/row_navigator.py` | State machine: ACQUIRE → FOLLOW → ROW_END / OBSTACLE_WAIT |
| `navigation/row_navigator_cam.py` | Camera-only state machine |
| `navigation/row_perception.py` | PCA-based LiDAR row detector; EMA-smoothed RowEstimate |
| `navigation/row_safety.py` | Three-zone obstacle monitor (forward + left/right tire tracks) |
| `navigation/row_controller.py` | Pure-pursuit speed/steering controller |
| `lidar/lidar_driver.py` | Async UDP VLP-16 driver; vectorised numpy scan_stream_np() |
| `lidar/obstacle_filter.py` | `tilt_correct_pts()` + `LIDAR_MOUNT_HEIGHT`; self-filter logic |
| `camera/oak_driver.py` | Async OAK-D driver via farm-ng EventClient (localhost:50010) |
| `camera/depth_obstacle.py` | Depth-frame inner-edge obstacle detector (`col_centre_frac`) |
| `camera/row_detector_visual.py` | HSV green-centroid + linearity-gated PCA heading |
| `camera/row_detector_depth_edge.py` | Colour-independent Canny/Hough + depth-gap lateral detector |

---

### Sensor Frame Convention

- **X** = right, **Y** = forward, **Z** = up (sensor frame, after tilt correction)
- Ground-relative height: `h = z_corrected + LIDAR_MOUNT_HEIGHT`
- `LIDAR_MOUNT_HEIGHT = 0.699 m` (defined in `lidar/obstacle_filter.py`)
- **LiDAR tilt: 15° forward (nose-down)** — requires tilt correction before any height math
- Crop geometry: onion canopy h ≈ 0.10–0.60 m; adjacent row canopy h ≈ 0.70–0.85 m

---

### LiDAR Tilt Correction

**Critical:** The VLP-16 is mounted with a 15° forward (nose-down) tilt. Without correction,
`h = z_sensor + LIDAR_MOUNT_HEIGHT` is wrong by `y × sin(15°)` ≈ 0.52 m at 2 m forward.

**Impact without correction:**
- Onion crop at true h=0.30 m appears as h=0.83 m → above crop band max (0.60 m) → ACQUIRE forever
- Ground return at 3 m appears as h=0.80 m → above obstacle threshold (0.75 m) → OBSTACLE_WAIT immediately

**Fix** — `lidar/obstacle_filter.py::tilt_correct_pts(pts, tilt_rad)`:
```
y_world =  y_sensor · cos(θ) + z_sensor · sin(θ)
z_world = −y_sensor · sin(θ) + z_sensor · cos(θ)
```
Applied in `row_navigator.py` after self-filtering, before `detector.update()` and `safety.check()`.

**CLI flag:** `--lidar-tilt DEG` (default: **15.0**). Set `--lidar-tilt 0` for a flat mount.

---

### Tuned Parameters (current working values for onion field)

#### Self-filter (`lidar/obstacle_filter.py` / `--self-radius`)
| Parameter | Value | Rationale |
|---|---|---|
| `self_radius` | **1.5 m** | Robot frame seen at ~0.72 m planar range; 1.5 m clears all body returns without cutting crop ROI (which starts at y=1.5 m) |

#### Row Perception (`navigation/row_perception.py`)
| Parameter | Value | Rationale |
|---|---|---|
| Crop height band | h ∈ [0.05, 0.60] m | Onion plants; excludes ground and above-canopy clutter |
| ROI depth | y ∈ [1.5, 7.0] m | Past self-filter; within reliable row geometry |
| ROI half-width | \|x\| ≤ 0.80 m | Captures crop row ± shoulder without pulling in adjacent rows |
| PCA linearity threshold | > 0.20 | Below this the cluster is not line-like; confidence forced to 0 |
| Density normaliser | n / 130 pts | 130 points = full confidence at typical VLP-16 density |
| Confidence formula | `min(1, n/130) × max(0, min(1, (linearity−0.20)/0.55))` | |
| EMA alpha | 0.35 | Smooths per-frame jitter without adding excessive lag |

#### State Machine (`navigation/row_navigator.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `acquire_conf` | **0.45** | Lowered from 0.55 — robot 0.75 m off-centre reduces linearity enough that 0.55 was never reached |
| Acquire consecutive frames | 5 | Must see ≥ 5 frames ≥ acquire_conf before entering FOLLOW |
| `obstacle_clear_secs` | **1.5 s** | Consecutive clear time required to leave OBSTACLE_WAIT |
| Min confidence for control | 0.35 | Below this the controller outputs zero velocity |

#### Safety Monitor (`navigation/row_safety.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `forward_dist` | 2.5 m | Stopping horizon ahead |
| `forward_half_width` | 0.95 m | Robot body width + margin |
| `obstacle_height` (forward) | **0.75 m** | Above LIDAR_MOUNT_HEIGHT (0.699 m); onion plants (h≤0.60) pass; humans/posts stop |
| `tire_obstacle_height` | **0.85 m** (field default) | Raised above adjacent crop canopy (h≈0.80–0.84 m) to eliminate L-TIRE false positives |
| `tire_track` | 0.915 m | Half-track width of Amiga |
| `tire_half_width` | 0.25 m | ± corridor around each wheel centreline |
| `tire_dist` | 2.5 m | Same stopping horizon as forward zone |
| `near` | 0.20 m | Ignore returns closer than this (sensor artefacts at beam origin) |
| `forward_min_points` | 4 | Minimum LiDAR returns to trigger forward stop |
| `tire_min_points` | 4 | Minimum LiDAR returns to trigger tire-track stop |

#### Pure-Pursuit Controller (`navigation/row_controller.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `lookahead` | 2.0 m | Look-ahead distance |
| `max_linear` | 0.30 m/s | Conservative field speed |
| `min_linear` | 0.08 m/s | Minimum creep speed when turning hard |
| `min_confidence` | 0.35 | Zero output below this |
| Speed formula | `conf × max(0.25, 1.0 − \|θ\|/0.60) × max_linear` | Slows for both low confidence and large heading error |

---

### Full CLI Flag Reference — `scripts/row_follow.py`

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable canbus and send velocity commands to wheels |
| `--rows N` | 1 | Stop after N rows completed |
| `--headland` | off | Perform open-loop headland turns between rows |
| `--slam` | off | Enable SLAM odometry integration (currently no-op) |
| `--speed M` | 0.30 | Max forward speed m/s |
| `--lidar-tilt DEG` | **15.0** | Forward (nose-down) LiDAR tilt in degrees — **must match physical mount** |
| `--roi-x W` | 0.80 | Row detection ROI half-width m |
| `--crop-min H` | 0.05 | Minimum crop height above ground m |
| `--crop-max H` | 0.60 | Maximum crop height above ground m |
| `--self-radius R` | **1.5** | Self-filter radius — discard returns within R m (robot body) |
| `--acquire-conf C` | **0.45** | Min row-detection confidence (0–1) to leave ACQUIRE |
| `--obstacle-height H` | **0.75** | Min ground-relative height m to count as obstacle in FORWARD zone |
| `--tire-height H` | (=obstacle-height) | Min height for TIRE-ZONE obstacles; set **0.85** for onion fields |
| `--camera` | off | Enable OAK-D stereo cameras |
| `--cam-left-id S` | "" | Left camera farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right camera farm-ng service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset from centreline m |
| `--cam-stop-dist M` | **2.5** | Camera depth obstacle stop distance m |
| `--debug` | off | Stream LiDAR height histogram + save bird's-eye PNG |
| `--save-dir DIR` | — | Save raw point-cloud numpy arrays to DIR |
| `--no-validate` | off | Skip LiDAR startup health check |

### Full CLI Flag Reference — `scripts/cam_follow.py`

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable motion (default: perception-only) |
| `--rows N` | 1 | Number of rows to cover |
| `--headland` | off | Perform open-loop headland turns between rows |
| `--speed M` | **0.20** | Max forward speed m/s (lower than LiDAR due to camera latency) |
| `--detector` | `hsv` | `hsv` = HSV green centroid (default); `depth-edge` = colour-independent |
| `--cam-left-id S` | "" | Left OAK-D farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right OAK-D farm-ng service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset from centreline m |
| `--cam-stop-dist M` | **2.5** | Depth obstacle stop distance m |
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
    boot  │     ACQUIRE      │  confidence ≥ acquire_conf (0.45)
  ───────>│  5 consecutive   │────────────────────────────────────►┐
          │  frames needed   │                                      │
          └──────────────────┘                                      │
                   ▲ obstacle clears (1.5 s)                       ▼
                   │                                       ┌──────────────┐
          ┌──────────────────┐   obstacle detected         │    FOLLOW    │
          │  OBSTACLE_WAIT   │◄────────────────────────────│  pure-pursuit│
          └──────────────────┘                             │  cmd sent    │
                                                           └──────┬───────┘
                                                                  │ row end
                                                                  ▼
                                                           ┌──────────────┐
                                                           │   ROW_END    │
                                                           └──────────────┘
```

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
| Sub-service for camera 2 | `oak1` (times out — hardware not connected) |
| Subscription pattern | `SubscribeRequest(uri=Uri(path='/rgb', query='service_name=oak0'))` |
| Image stream paths | `/rgb`, `/disparity`, `/left`, `/right`, `/imu` |
| Frame proto type | `OakFrame` — fields: `meta`, `image_data` (bytes) |
| Image encoding | JPEG bytes — decode with `cv2.imdecode(np.frombuffer(msg.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)` |
| RGB frame size | ~277 KB JPEG → HxWx3 BGR uint8 |
| Disparity encoding | Grayscale JPEG uint8 (~7 KB) → `depth_mm = BASELINE_MM × FOCAL_PX / disparity_px` |
| OAK-D calibration | baseline ≈ 75 mm, focal ≈ 452 px @ 640 px width |

#### Depth Obstacle Strip Geometry

Side-mounted cameras see the robot's forward centreline path at the **inner image edge**,
not the image centre. The obstacle strip must be shifted accordingly via `col_centre_frac`:

| Camera | Position | `col_centre_frac` | Why |
|---|---|---|---|
| Left | −0.915 m (left of robot) | **0.80** | Robot path at col ~480 (inner right edge) |
| Right | +0.915 m (right of robot) | **0.20** | Robot path at col ~160 (inner left edge) |

At `col_centre_frac=0.5` (image centre), the robot centreline would only become visible at
≥5.7 m — useless for obstacle detection. The inner-edge strips cover **1.66–5 m forward**.

This fills the LiDAR blind zone (< 1.5 m, removed by self-filter) with meaningful obstacle data.

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

EMA smoothing (alpha=0.35); `_decay()` halves confidence each frame with no detection.

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
| Stuck in ACQUIRE (conf ≤ 0.52) | Robot 0.75 m off-centre → reduced PCA linearity → conf below threshold | Lowered `acquire_conf` 0.55 → **0.45** |
| ACQUIRE forever with 15° tilted mount | `h = z_sensor + height` wrong; crop appears at h=0.83 m (above crop band) | Added `tilt_correct_pts()` in `obstacle_filter.py`; wired via `--lidar-tilt` |
| OBSTACLE_WAIT immediately with 15° tilt | Ground returns appear at h=0.80 m → above obstacle threshold with uncorrected height | Same tilt correction fix |
| `[oak_driver] X_LINK_DEVICE_NOT_FOUND` | Cameras on PoE switch — `amiga_service` holds exclusive depthai handles | Rewrote `oak_driver.py` to use `EventClient` → localhost:50010 |
| `[oak_driver] THE_400_P AttributeError` | depthai 2.22.0 doesn't have `THE_400_P` (legacy note — no longer uses depthai) | N/A — depthai pipeline removed |
| Obstacle not detected by cameras (`safe=clear`) | Depth strip at image centre; robot centreline appears at col ~480/~160 from side cameras | Added `col_centre_frac=0.80/0.20` to shift strips to inner camera edges |
| Heading chaos ±70° at row end | PCA on round green blob → arbitrary principal direction → flips ±90° between frames | Added eigenvalue `linearity ≥ 0.30` check in `row_detector_visual.py` |
| `depth-edge` robot turns right (open room) | `argmax(col_smooth)` found far wall; no right flank for gap validation | Added local-maximum flank validation in `_lateral_from_depth` |
| `depth-edge` robot turns left (structural lines) | Hough detected warehouse edges without depth confirmation | Changed validity gate: `depth_conf == 0 → side invalid` regardless of Hough |
| Ctrl+C traceback during gRPC cleanup | `except Exception` doesn't catch `KeyboardInterrupt` (`BaseException`) | Changed to `except BaseException: pass` in cleanup gather |
| `RuntimeWarning: Mean of empty slice` | `np.nanmean` called on all-NaN column slice | Pre-check valid columns with `~np.all(np.isnan(band), axis=0)` |
| ACQUIRE/FOLLOW/OBSTACLE_WAIT oscillation | Real environmental obstacle at ~2.2–2.5 m intermittently entering forward zone | Not a code bug; resolves in clear onion rows |
| `\r` terminal shows only "clear" | Carriage-return overwrites intermediate blocked frames | Use `--debug` to see every frame; transitions always print with `\n` |

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
