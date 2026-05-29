# Vidalia Auto Navigation

> **Fully autonomous onion-row following and laser-based weed control for the farm-ng Amiga robot.**
> No pre-built map required. No ROS 2 required on the robot.

---

## What This Does

The Amiga robot straddles a crop bed and follows the centre onion row in real time using a
**Velodyne VLP-16 LiDAR** (primary) and two side-mounted **OAK-D stereo cameras** (obstacle
fill + visual supplement). A pure-pursuit controller steers the robot along the row at up to
0.30 m/s while a three-zone safety monitor watches for humans, posts, and equipment in the path.

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                      Amiga brain (ARM64)                         │
 │                                                                  │
 │  VLP-16 LiDAR ──► PCA row detect ──► pure-pursuit ──► canbus   │
 │  (10 Hz UDP)        heading + offset    controller    Twist2d   │
 │                                                                  │
 │  OAK-D left+right ──► depth obstacle ──► safety override        │
 │  (PoE, gRPC)          visual row fuse   OBSTACLE_WAIT           │
 └──────────────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Details |
|---|---|
| **Robot** | farm-ng Amiga (differential drive, 0.915 m half-track) |
| **Brain** | camphor-clone — Ubuntu 20.04 ARM64, farm-ng OS 2.0 / Barley venv |
| **LiDAR** | Velodyne VLP-16 — 192.168.1.201, UDP port 2368, 10 Hz, 16 rings |
| **LiDAR mount** | 0.699 m above ground, **15° forward (nose-down) tilt** |
| **Cameras** | 2× OAK-D (PoE switch) — managed by `amiga_service` on localhost:50010 |
| **Camera mount** | ±0.915 m from robot centreline, forward-facing, service names `oak0` / `oak1` |
| **Dev PC** | Ubuntu 22.04, SSH via Tailscale `100.66.121.56` |

---

## Quick Start

```bash
# On the Amiga brain
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia

# Perception-only — robot stays still, verify detection:
python3 scripts/row_follow.py

# Autonomous — robot WILL move:
python3 scripts/row_follow.py --auto --tire-height 0.85

# Autonomous with OAK-D cameras (fills LiDAR blind zone, adds visual row fuse):
python3 scripts/row_follow.py --auto --tire-height 0.85 --camera

# Camera-only mode (no LiDAR required):
python3 scripts/cam_follow.py --auto
```

Press **Ctrl+C** at any time to stop the robot immediately.

---

## Two Autonomy Stacks

### Stack 1 — LiDAR Row Follow (Primary)

**Entry point:** `scripts/row_follow.py`

Uses the VLP-16 as the primary sensor. Cameras are optional supplements.

```
VLP-16 (UDP :2368, 10 Hz)
   │  lidar/lidar_driver.py       raw UDP → Nx3 numpy array
   │  lidar/obstacle_filter.py    tilt correction + self-filter (≥ 1.5 m)
   ▼
   ┌─────────────────────────────────────────┐
   │  navigation/row_perception.py           │  PCA → heading + lateral offset
   │  navigation/row_safety.py               │  3-zone obstacle monitor
   └──────────────┬──────────────────────────┘
                  │
   OAK-D (optional, async)
   │  camera/oak_driver.py         farm-ng EventClient → RGB + depth frames
   │  camera/depth_obstacle.py     depth strip → blind-zone obstacle check
   │  camera/row_detector_visual.py HSV green centroid → lateral supplement
   ▼
   navigation/row_navigator.py    state machine + LiDAR/camera fusion
   navigation/row_controller.py   pure-pursuit → (linear_vel, angular_vel)
   canbus/canbus_interface.py     Twist2d via gRPC request_reply("/twist")
   Amiga wheels
```

**State machine:**
```
         ACQUIRE ──(conf ≥ 0.45, 5 frames)──► FOLLOW
           ▲                                      │
           │ obstacle clears (1.5 s)              │ obstacle detected
           │                                      ▼
         OBSTACLE_WAIT ◄─────────────────── (safety blocked)
                                                  │
                                           row end detected
                                                  ▼
                                             ROW_END
```

**CLI flags:**

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable motion (default: perception-only) |
| `--rows N` | 1 | Stop after N rows |
| `--headland` | off | Open-loop headland turns between rows |
| `--speed M` | 0.30 | Max linear speed m/s |
| `--lidar-tilt DEG` | **15.0** | Forward (nose-down) LiDAR tilt in degrees |
| `--roi-x W` | 0.80 | Row detection ROI half-width m |
| `--crop-min H` | 0.05 | Minimum crop height above ground m |
| `--crop-max H` | 0.60 | Maximum crop height above ground m |
| `--self-radius R` | 1.5 | Self-filter radius m (robot body returns) |
| `--acquire-conf C` | 0.45 | Min confidence to leave ACQUIRE |
| `--obstacle-height H` | 0.75 | Forward zone obstacle height threshold m |
| `--tire-height H` | (=obstacle-height) | Tire zone height — set **0.85** for onion fields |
| `--camera` | off | Enable OAK-D stereo cameras |
| `--cam-left-id S` | "" | Left camera farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right camera farm-ng service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset from centreline m |
| `--cam-stop-dist M` | **2.5** | Depth obstacle stop distance m |
| `--debug` | off | Stream height histogram instead of navigating |
| `--no-validate` | off | Skip LiDAR startup health check |

---

### Stack 2 — Camera-Only Row Follow (LiDAR-free backup)

**Entry point:** `scripts/cam_follow.py`

Uses only the OAK-D cameras. No Velodyne required.

```
OAK-D left + right (farm-ng EventClient → localhost:50010)
   │  camera/oak_driver.py           RGB + depth frames (JPEG over gRPC)
   │  camera/row_detector_visual.py  HSV green centroid → lateral + heading
   │    OR
   │  camera/row_detector_depth_edge.py  Canny/Hough heading + depth gap lateral
   │  camera/depth_obstacle.py       depth inner-edge strip → obstacle check
   ▼
   navigation/row_navigator_cam.py   same state machine as LiDAR mode
   navigation/row_controller.py      pure-pursuit (unchanged)
   canbus/canbus_interface.py
   Amiga wheels
```

**Key differences from LiDAR mode:**

| Aspect | LiDAR mode | Camera-only mode |
|---|---|---|
| Row detection | PCA on 3-D point cloud | HSV green centroid or Canny/Hough geometry |
| Heading estimate | LiDAR PCA (accurate) | Image-space PCA with FOV correction |
| Obstacle safety | 3-zone (fwd + 2 tires) | Depth inner-edge strip, both cameras |
| Tire-track monitoring | Yes | No |
| Row-end detection | Point density drop | Green fraction drops below threshold |
| Confidence range | 0–1 (typical 0.70+ in row) | 0–0.50 (camera-based) |
| Default max speed | 0.30 m/s | **0.20 m/s** |
| Lighting sensitivity | None | Degrades in harsh sun/shadow |

**CLI flags:**

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable motion |
| `--rows N` | 1 | Number of rows |
| `--headland` | off | Open-loop headland turns |
| `--speed M` | 0.20 | Max speed m/s |
| `--detector` | `hsv` | `hsv` = green centroid; `depth-edge` = colour-independent geometry |
| `--cam-left-id S` | "" | Left camera service name (default: oak0) |
| `--cam-right-id S` | "" | Right camera service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset m |
| `--cam-stop-dist M` | **2.5** | Depth obstacle stop distance m |
| `--acquire-conf F` | 0.20 | Min confidence to leave ACQUIRE |
| `--acquire-green F` | 0.08 | Min green fraction to leave ACQUIRE |
| `--fps N` | 10 | OAK-D capture rate |
| `--hsv-h-lo H` | 35 | HSV hue lower bound (0–180); use ~10 for brown/cardboard |
| `--hsv-h-hi H` | 85 | HSV hue upper bound |
| `--hsv-s-lo S` | 40 | HSV saturation lower bound; lower to ~25 for pale colours |
| `--hsv-v-lo V` | 40 | HSV value lower bound |

---

## Tuned Parameters (Onion Field)

### LiDAR Self-Filter
| Parameter | Value | Rationale |
|---|---|---|
| `self_radius` | **1.5 m** | Robot frame seen at ~0.72 m; 1.5 m clears body without cutting crop ROI at 1.5 m |

### LiDAR Tilt Correction
| Parameter | Value | Rationale |
|---|---|---|
| `lidar_tilt` | **15°** | Current nose-down mount; corrects `h = z + height` which is otherwise off by `y × sin(15°)` ≈ 0.78 m at 3 m |

### Row Perception
| Parameter | Value | Rationale |
|---|---|---|
| Crop height band | h ∈ [0.05, 0.60] m | Onion plants; excludes ground and above-canopy |
| ROI depth | y ∈ [1.5, 7.0] m | Past self-filter; within reliable row geometry |
| ROI half-width | \|x\| ≤ 0.80 m | Centre row ± shoulder; no adjacent rows |
| PCA linearity threshold | > 0.20 | Below = not line-like; confidence = 0 |
| Density normaliser | n / 130 pts | 130 pts = full confidence at VLP-16 density |
| EMA alpha | 0.35 | Smooths jitter without excessive lag |

### Safety Monitor
| Parameter | Value | Rationale |
|---|---|---|
| `obstacle_height` (forward) | **0.75 m** | Above LIDAR_MOUNT_HEIGHT (0.699 m); onion plants (≤0.60 m) pass; humans/posts stop |
| `tire_obstacle_height` | **0.85 m** | Above adjacent crop canopy (≈0.80–0.84 m) — eliminates false tire-zone stops |
| `forward_dist` | 2.5 m | Stopping horizon |
| `forward_half_width` | 0.95 m | Robot body + margin |
| `tire_track` | 0.915 m | Half-track of Amiga |

### Pure-Pursuit Controller
| Parameter | Value | Rationale |
|---|---|---|
| `lookahead` | 2.0 m | Look-ahead distance |
| `max_linear` | 0.30 m/s | Conservative field speed |
| `min_linear` | 0.08 m/s | Minimum creep speed in tight turns |
| `min_confidence` | 0.35 | Zero output below this |

---

## OAK-D Camera Integration

### Hardware Connectivity

OAK-D cameras connect via **PoE switch**, managed exclusively by `amiga_service`.
Direct `depthai` access always fails (`X_LINK_DEVICE_NOT_FOUND`).

| Item | Value |
|---|---|
| Service host | `localhost` |
| Service port | **50010** |
| Left camera | `oak0` |
| Right camera | `oak1` |
| Stream paths | `/rgb`, `/disparity`, `/left`, `/right`, `/imu` |
| Image encoding | JPEG bytes — decode with `cv2.imdecode` |
| RGB frame size | ~277 KB JPEG → HxWx3 BGR uint8 |
| Depth from disparity | `depth_mm = BASELINE_MM × FOCAL_PX / disparity_px` |
| OAK-D calibration | baseline ≈ 75 mm, focal ≈ 452 px @ 640 px width |

### Obstacle Strip Geometry

Side-mounted cameras see the robot's forward centreline path at the **inner image edge**,
not the image centre. The obstacle strip is shifted accordingly:

| Camera | `col_centre_frac` | Effective detection range |
|---|---|---|
| Left (`-0.915 m`) | **0.80** (inner right edge, col ~512) | 1.66–5 m forward |
| Right (`+0.915 m`) | **0.20** (inner left edge, col ~128) | 1.66–5 m forward |

This fills the LiDAR blind zone (< 1.5 m) where the self-filter removes all points.

### Connectivity Diagnostics

```bash
# Check amiga_service is running:
ps aux | grep amiga_service
ss -tlnp | grep 50010

# Test camera subscription:
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

## LiDAR Tilt Correction

The VLP-16 is mounted with a **15° forward (nose-down) tilt**. Without correction the raw
sensor Z axis is not vertical, causing large height errors:

| Point | Without correction | With correction |
|---|---|---|
| Onion crop at 2 m, h=0.30 m | h = **0.83 m** (above crop band, invisible) | h = **0.30 m** ✓ |
| Ground return at 3 m | h = **0.80 m** (trips safety stop!) | h = **0.00 m** ✓ |
| Person at 1.5 m, h=0.90 m | h = **1.28 m** (above threshold, detected) | h = **0.90 m** ✓ |

**Rotation applied** in `lidar/obstacle_filter.py::tilt_correct_pts()`, called in
`navigation/row_navigator.py` after self-filtering, before all height-based logic:

```
y_world =  y_sensor · cos(θ) + z_sensor · sin(θ)
z_world = −y_sensor · sin(θ) + z_sensor · cos(θ)
```

Use `--lidar-tilt 0` for a flat mount.

---

## Row Detection Algorithms

### HSV Green Centroid (default, `--detector hsv`)

1. Crop image to middle-third width, lower two-thirds height
2. Convert BGR → HSV; apply colour mask (configurable per crop)
3. If green fraction < 8% → no detection this frame
4. Compute centroid → lateral offset via `m_per_px = 2·depth·tan(hFOV/2) / width`
5. Fit PCA to green pixel cloud → heading (only when **linearity ≥ 0.30** to avoid unstable directions from round blobs at row-end)
6. EMA-smooth result (α = 0.30); decay confidence × 0.5 on no-detection frames

**HSV tuning for different crops:**

| Crop / test material | `--hsv-h-lo` | `--hsv-h-hi` | `--hsv-s-lo` |
|---|---|---|---|
| Onion / green plants | 35 | 85 | 40 |
| Brown cardboard (lab test) | 10 | 25 | 25 |
| Blue markers | 100 | 130 | 40 |

### Depth-Edge Geometry (`--detector depth-edge`)

Colour-independent; works in any lighting.

1. **Heading** — Canny edge + HoughLinesP on RGB; keep vertically-oriented lines; median slope → heading angle
2. **Lateral offset** — smooth per-column depth; find `argmax` (deepest column = inter-row gap); validate that both flanks are shallower (rejects open-environment false matches)
3. **Validity gate** — only report a side valid when depth contrast ≥ threshold; Hough lines alone (without depth confirmation) are rejected to prevent ambient structural edges from biasing heading

---

## Known Issues and Fixes

| Symptom | Root Cause | Fix |
|---|---|---|
| Permanent `OBSTACLE_WAIT — FWD@0.7m` | Robot frame LiDAR returns at 0.72 m passing self-filter | Raised `self_radius` 1.0 → **1.5 m** |
| Permanent `OBSTACLE_WAIT — FWD@2.2m` | `obstacle_height` below `LIDAR_MOUNT_HEIGHT`; entire field treated as obstacle | Raised `obstacle_height` 0.45 → **0.75 m** |
| `L-TIRE` false positives | Adjacent onion row canopy (h≈0.80–0.84 m) in tire zone | Added `--tire-height`; set **0.85 m** for onion fields |
| Stuck in ACQUIRE (conf never reaches 0.55) | Robot off-centre reduces PCA linearity; confidence capped | Lowered `acquire_conf` 0.55 → **0.45** |
| `X_LINK_DEVICE_NOT_FOUND` on camera init | `amiga_service` holds exclusive depthai handles; direct access impossible | Rewrote `oak_driver.py` to use farm-ng `EventClient` → localhost:50010 |
| Row not detected with 15° tilt (ACQUIRE forever) | `h = z + LIDAR_MOUNT_HEIGHT` wrong with tilted mount; crop appears above crop band | Added tilt rotation in `obstacle_filter.py::tilt_correct_pts()` |
| Ground returns trigger safety stop with 15° tilt | Without tilt correction ground returns appear at h≈0.80 m → above obstacle threshold | Same tilt correction fix |
| Obstacle not detected by cameras (`safe=clear`) | Depth strip centred on image; robot centreline off-centre at useful range | Added `col_centre_frac=0.80/0.20` to shift strip to inner camera edge |
| Heading chaos ±70° at row end | PCA on round green blob has arbitrary principal direction; flips between frames | Added eigenvalue linearity check (threshold 0.30) in `row_detector_visual.py` |
| `depth-edge` robot turns left/right in open room | No crop row geometry → argmax at image edge / structural Hough lines | Added depth local-maximum flank validation + depth-validity gate |
| Ctrl+C traceback during gRPC cleanup | `except Exception` doesn't catch `KeyboardInterrupt` (`BaseException`) | Changed to `except BaseException: pass` |
| `RuntimeWarning: Mean of empty slice` | `np.nanmean` on all-NaN column slice | Pre-check valid columns before calling `nanmean` |

---

## File Map

```
auto_navigation_vidalia/
├── scripts/
│   ├── row_follow.py          # LiDAR row follow entry point
│   └── cam_follow.py          # Camera-only entry point
├── navigation/
│   ├── row_navigator.py       # State machine (ACQUIRE/FOLLOW/OBSTACLE_WAIT/ROW_END)
│   ├── row_navigator_cam.py   # Camera-only state machine variant
│   ├── row_perception.py      # PCA-based LiDAR row detector
│   ├── row_safety.py          # 3-zone obstacle safety monitor
│   ├── row_controller.py      # Pure-pursuit speed/steering controller
│   └── row_perception.py      # RowEstimate dataclass + EMA
├── lidar/
│   ├── lidar_driver.py        # Async UDP VLP-16 driver (numpy scan_stream_np)
│   └── obstacle_filter.py     # tilt_correct_pts() + LIDAR_MOUNT_HEIGHT constant
├── camera/
│   ├── oak_driver.py          # farm-ng EventClient OAK-D driver (RGB + depth)
│   ├── depth_obstacle.py      # Depth inner-edge obstacle detector (col_centre_frac)
│   ├── row_detector_visual.py # HSV green centroid + PCA heading (linearity-gated)
│   └── row_detector_depth_edge.py  # Canny/Hough + depth gap geometry detector
├── canbus/
│   └── canbus_interface.py    # Twist2d send via farm-ng gRPC request_reply
├── service_config.json        # canbus (:6001) + filter (:20001) config
└── requirements.txt           # farm-ng-core, farm-ng-amiga, protobuf
```

---

## Setup

```bash
# On the Amiga brain — activate the pre-installed farm-ng venv:
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia

# One-time: install Python deps if not already present
pip install -r requirements.txt
```

The farm-ng venv at `/farm_ng_image/venv/` is persistent (NVMe). Everything under `~/`
survives reboots; anything outside (e.g. `/opt/ros/`) is wiped by the overlay filesystem.

---

## Development PC

```bash
# SSH via Tailscale:
ssh farm-ng-user-laserweeding@100.66.121.56

# ROS 2 (not installed yet — install Humble for RViz/bag recording):
# sudo apt install ros-humble-desktop
```

---

## Planned Next Steps

- **Real-time visualizations** — stream LiDAR point clouds, camera feeds, confidence/offset
  plots, and state-machine status to a laptop viewer (Rerun or Foxglove) over Tailscale
- **Multi-row coverage** — headland turns + row counter for unattended field runs
- **Laser weed control** — integrate laser firing gate with row position + weed detection
- **Confidence-based speed** — increase speed above 0.30 m/s once field parameters are locked
