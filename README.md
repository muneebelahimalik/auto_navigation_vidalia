# Vidalia Auto Navigation

> **Fully autonomous onion-row following and laser-based weed control for the farm-ng Amiga robot.**
> No pre-built map required. No ROS 2 required on the robot brain.

---

## Table of Contents

1. [What This Does](#what-this-does)
2. [Hardware](#hardware)
3. [Amiga Brain — System Details](#amiga-brain--system-details)
4. [Quick Start](#quick-start)
5. [Two Autonomy Stacks](#two-autonomy-stacks)
6. [ROS 2 Visualization Bridge](#ros-2-visualization-bridge)
7. [Tuned Parameters (Onion Field)](#tuned-parameters-onion-field)
8. [OAK-D Camera Integration](#oak-d-camera-integration)
9. [LiDAR Tilt Correction](#lidar-tilt-correction)
10. [Row Detection Algorithms](#row-detection-algorithms)
11. [Known Issues and Fixes](#known-issues-and-fixes)
12. [File Map](#file-map)
13. [Setup](#setup)
14. [Planned Next Steps](#planned-next-steps)

---

## What This Does

The Amiga robot straddles a crop bed and follows the centre onion row in real time using a
**Velodyne VLP-16 LiDAR** (primary) and two side-mounted **OAK-D stereo cameras** (obstacle
fill + visual supplement). A pure-pursuit controller steers the robot along the row at up to
0.30 m/s while a three-zone safety monitor watches for humans, posts, and equipment in the path.

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                      Amiga Brain (ARM64, Jetson Xavier NX)       │
 │                                                                  │
 │  VLP-16 LiDAR ──► PCA row detect ──► pure-pursuit ──► canbus   │
 │  (10 Hz UDP)        heading + offset    controller    Twist2d   │
 │                                                                  │
 │  OAK-D left+right ──► depth obstacle ──► safety override        │
 │  (PoE, gRPC)          visual row fuse   OBSTACLE_WAIT           │
 │                                                                  │
 │  /dev/shm IPC ──► Docker ROS2 bridge ──► Foxglove WS :8765      │
 │  (optional)          vidalia_node.py     browser visualization  │
 └──────────────────────────────────────────────────────────────────┘
```

---

## Hardware

| Component | Details |
|---|---|
| **Robot** | farm-ng Amiga (differential drive, 0.915 m half-track) |
| **Brain** | camphor-clone — Jetson Xavier NX, Ubuntu 20.04 ARM64, farm-ng OS 2.0 / Barley |
| **LiDAR** | Velodyne VLP-16 — 192.168.1.201, UDP port 2368, 10 Hz, 16 rings |
| **LiDAR mount** | 0.699 m above ground, **15° forward (nose-down) tilt** |
| **Cameras** | 2× OAK-D (PoE switch) — managed by `amiga_service` on localhost:50010 |
| **Camera mount** | ±0.915 m from robot centreline, forward-facing, service names `oak0` / `oak1` |
| **Dev PC** | Ubuntu 22.04, SSH via Tailscale `100.66.121.56` |

---

## Amiga Brain — System Details

These details are critical for understanding the software architecture — especially why Docker
is used for ROS 2 and why certain paths are chosen for persistent storage.

| Property | Value |
|---|---|
| **SoC** | NVIDIA Jetson Xavier NX |
| **OS** | Ubuntu 20.04.6 LTS (Focal Fossa) |
| **Architecture** | aarch64 (ARM64) |
| **Kernel** | Linux 5.10.104-tegra (L4T R35.2.1) |
| **JetPack** | 5.1 |
| **Python** | 3.8.10 (system) |
| **Home directory** | `/mnt/managed_home/farm-ng-user-laserweeding/` (symlinked as `~/`) |
| **NVMe device** | `/dev/nvme0n1p1` mounted at `/mnt` (234 GB, persistent) |
| **Shared memory** | `/dev/shm` — tmpfs, 7.3 GB |
| **Docker storage** | `/var/lib/docker` on NVMe — **persists across reboots** |
| **farm-ng venv** | `/farm_ng_image/venv/` — pre-installed, persistent |
| **depthai version** | 2.22.0.0 (do **not** upgrade to 2.23+) |
| **opencv-python** | 4.7.0.68 |
| **Tailscale IP** | 100.66.121.56 |

### Overlay Filesystem (Critical)

The Amiga OS root `/` uses an **overlay filesystem**. Every reboot wipes everything outside
`~/` (which is the NVMe). This means:

- `/opt/ros/` — **wiped on every reboot** — cannot install ROS natively
- `/farm_ng_image/venv/` — pre-installed at image build time, survives reboots
- `~/` (= `/mnt/managed_home/farm-ng-user-laserweeding/`) — **NVMe, fully persistent**
- `/var/lib/docker` — **NVMe, fully persistent** — Docker images survive every reboot
- `~/.config/systemd/user/` — **NVMe, persistent** — systemd user services persist

**Consequence:** ROS 2 is run inside a Docker container. The image is built once, stored in
`/var/lib/docker`, and available on every reboot without reinstallation.

---

## Quick Start

```bash
# On the Amiga brain — SSH in first:
ssh farm-ng-user-laserweeding@100.66.121.56

# Activate the pre-installed farm-ng venv (required every session):
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia

# Perception-only — robot stays still, verify detection:
python3 scripts/row_follow.py

# Autonomous — robot WILL move:
python3 scripts/row_follow.py --auto --tire-height 0.85

# Autonomous with OAK-D cameras (fills LiDAR blind zone < 1.5 m):
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
         ACQUIRE ──(conf ≥ 0.45, 5 consecutive frames)──► FOLLOW
           ▲                                                    │
           │ obstacle clears (1.5 s consecutive clear)         │ obstacle detected
           │                                                    ▼
         OBSTACLE_WAIT ◄──────────────────────────────── (safety blocked)
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
| `--self-radius R` | **1.5** | Self-filter radius m (discards robot body returns) |
| `--acquire-conf C` | **0.45** | Min confidence to leave ACQUIRE |
| `--obstacle-height H` | **0.75** | Forward zone obstacle height threshold m |
| `--tire-height H` | (=obstacle-height) | Tire zone height — set **0.85** for onion fields |
| `--camera` | off | Enable OAK-D stereo cameras |
| `--cam-left-id S` | "" | Left camera farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right camera farm-ng service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset from centreline m |
| `--cam-stop-dist M` | **2.5** | Depth obstacle stop distance m |
| `--cam-block-frames N` | **3** | Consecutive camera-blocked frames to trigger OBSTACLE_WAIT |
| `--ros2-bridge` | off | Write scan data + nav status to `/dev/shm/` for the ROS 2 Docker bridge |
| `--debug` | off | Stream LiDAR height histogram + save bird's-eye PNG |
| `--save-dir DIR` | — | Save raw point-cloud numpy arrays to DIR |
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
| `--cam-block-frames N` | **3** | Consecutive camera-blocked frames to trigger OBSTACLE_WAIT |
| `--acquire-conf F` | 0.20 | Min confidence to leave ACQUIRE |
| `--acquire-green F` | 0.08 | Min green fraction to leave ACQUIRE |
| `--fps N` | 10 | OAK-D capture rate |
| `--hsv-h-lo H` | 35 | HSV hue lower bound (0–180); use ~10 for brown/cardboard |
| `--hsv-h-hi H` | 85 | HSV hue upper bound |
| `--hsv-s-lo S` | 40 | HSV saturation lower bound; lower to ~25 for pale colours |
| `--hsv-v-lo V` | 40 | HSV value lower bound |

---

## ROS 2 Visualization Bridge

The Amiga Brain runs ROS 2 Foxy **inside a Docker container** — it is not installed natively
because the overlay filesystem would wipe `/opt/ros/` on every reboot. The Docker image is
stored in `/var/lib/docker` on the NVMe and persists across reboots.

### Architecture

```
Terminal 1 (navigation):
  python3 scripts/row_follow.py --auto --tire-height 0.85 --ros2-bridge
        │
        │  writes /dev/shm/vidalia_pts.bin      (int32 count + float32 xyz array)
        │  writes /dev/shm/vidalia_status.json   (nav state, velocities, obstacles)
        ▼
  /dev/shm  (tmpfs, 7.3 GB, shared with Docker via -v /dev/shm:/dev/shm)
        │
Terminal 2 (bridge):
  bash ros2_bridge/start.sh
        │  docker build   (first run ~10 min; cached thereafter)
        │  docker run --runtime nvidia …
        ▼
  Container (dustynv/ros:foxy-ros-base-l4t-r35.2.1)
        │  vidalia_node.py  reads /dev/shm/ at 12 Hz, publishes ROS 2 topics
        │  foxglove_bridge  WebSocket on port 8765
        ▼
Browser (any device on Tailscale):
  https://app.foxglove.dev
  → Open connection → Rosbridge WebSocket → ws://100.66.121.56:8765
```

### Why This Base Image

| Image | Reason |
|---|---|
| `dustynv/ros:foxy-ros-base-l4t-r35.2.1` | L4T R35.2.1 / Jetpack 5.1 optimised for Jetson Xavier NX; matches the exact kernel on the Amiga Brain |
| ~~`osrf/ros:foxy-ros-base`~~ | Generic ARM64 — does not know about Tegra kernel, CUDA libraries, or the NVIDIA container runtime |

This is the same approach used by farm-ng's official `amiga-ros-bridge` (which uses `dustynv/ros:noetic-pytorch-l4t-r35.2.1`).

### ROS 2 Topics Published

| Topic | Type | Description |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Live LiDAR scan with height-relative intensity field |
| `/tf_static` | TF | `base_link → velodyne` (x=0.959 m, z=0.699 m, pitch=−15°) |
| `/row_viz` | `visualization_msgs/MarkerArray` | Arrow = row direction; line = lateral offset |
| `/safety_viz` | `visualization_msgs/MarkerArray` | Wireframe boxes for forward + tire zones (green = clear, red = blocked) |
| `/cmd_vel` | `geometry_msgs/Twist` | Current velocity command |

### Running the Bridge

```bash
# Terminal 1 — navigation with bridge output enabled:
source /farm_ng_image/venv/bin/activate
cd ~/auto_navigation_vidalia
python3 scripts/row_follow.py --auto --tire-height 0.85 --ros2-bridge

# Terminal 2 — build and start the ROS 2 bridge container:
bash ros2_bridge/start.sh

# Browser — open on any device connected via Tailscale:
# 1. Go to  https://app.foxglove.dev
# 2. Click  "Open connection" → "Rosbridge WebSocket"
# 3. URL:   ws://100.66.121.56:8765
```

The first `docker build` takes 10–20 minutes (pulls ~4–6 GB L4T image). All subsequent runs
use the cached image and start in seconds.

### Autostart on Every Reboot

Install a persistent systemd user service (no sudo needed; lives on NVMe under `~/.config/`):

```bash
# Install + enable (runs on every reboot):
bash ros2_bridge/install_autostart.sh

# Check status:
systemctl --user status vidalia-ros2-bridge

# View logs:
journalctl --user -u vidalia-ros2-bridge -f

# Stop:
systemctl --user stop vidalia-ros2-bridge

# Remove:
bash ros2_bridge/install_autostart.sh remove
```

### Bridge Files

```
ros2_bridge/
├── Dockerfile           # FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1 + ROS2 packages
├── vidalia_node.py      # ROS2 Python node: reads /dev/shm/, publishes topics at 12 Hz
├── entrypoint.sh        # Starts vidalia_node + foxglove_bridge inside the container
├── start.sh             # docker build + docker run --runtime nvidia
├── install_autostart.sh # Installs ~/.config/systemd/user/vidalia-ros2-bridge.service
└── rviz/
    └── vidalia.rviz     # Pre-configured RViz2 layout (PointCloud2, markers, TF)
```

---

## Tuned Parameters (Onion Field)

### LiDAR Self-Filter

| Parameter | Value | Rationale |
|---|---|---|
| `self_radius` | **1.5 m** | Robot frame seen at ~0.72 m planar range; 1.5 m clears body without cutting crop ROI (which starts at y = 1.5 m) |

### LiDAR Tilt Correction

| Parameter | Value | Rationale |
|---|---|---|
| `lidar_tilt` | **15°** | Current nose-down mount angle; corrects height error of `y × sin(15°)` ≈ 0.78 m at 3 m forward |

### Row Perception

| Parameter | Value | Rationale |
|---|---|---|
| Crop height band | h ∈ [0.05, 0.60] m | Onion plants; excludes ground and above-canopy |
| ROI depth | y ∈ [1.5, 7.0] m | Past self-filter; within reliable row geometry |
| ROI half-width | \|x\| ≤ 0.80 m | Centre row ± shoulder; no adjacent rows |
| PCA linearity threshold | > 0.20 | Below = not line-like; confidence forced to 0 |
| Density normaliser | n / 130 pts | 130 pts = full confidence at VLP-16 density |
| EMA alpha | 0.35 | Smooths per-frame jitter without excessive lag |

### State Machine Thresholds

| Parameter | Value | Rationale |
|---|---|---|
| `acquire_conf` | **0.45** | Lowered from 0.55 — robot 0.75 m off-centre reduces PCA linearity; 0.55 was unreachable |
| Acquire consecutive frames | 5 | Must see ≥ 5 frames ≥ acquire_conf before entering FOLLOW |
| `obstacle_clear_secs` | **1.5 s** | Consecutive clear time required to leave OBSTACLE_WAIT |
| Min confidence for control | 0.35 | Below this the controller outputs zero velocity |
| `cam_block_frames` | **3** | Consecutive camera-blocked frames before OBSTACLE_WAIT; prevents false positives from intermittent depth noise |

### Safety Monitor

| Parameter | Value | Rationale |
|---|---|---|
| `obstacle_height` (forward) | **0.75 m** | Above LIDAR_MOUNT_HEIGHT (0.699 m); onion plants (h ≤ 0.60 m) pass; humans/posts stop |
| `tire_obstacle_height` | **0.85 m** | Above adjacent onion row canopy (h ≈ 0.80–0.84 m); eliminates false tire-zone stops |
| `forward_dist` | 2.5 m | Stopping horizon ahead |
| `forward_half_width` | 0.95 m | Robot body width + margin |
| `tire_track` | 0.915 m | Half-track of Amiga |
| `tire_half_width` | 0.25 m | ± corridor around each wheel centreline |
| `tire_dist` | 2.5 m | Same stopping horizon as forward zone |
| `forward_min_points` | 4 | Minimum LiDAR returns to trigger forward stop |
| `tire_min_points` | 4 | Minimum LiDAR returns to trigger tire-track stop |

### Pure-Pursuit Controller

| Parameter | Value | Rationale |
|---|---|---|
| `lookahead` | 2.0 m | Look-ahead distance |
| `max_linear` | 0.30 m/s | Conservative field speed |
| `min_linear` | 0.08 m/s | Minimum creep speed in tight turns |
| `min_confidence` | 0.35 | Zero output below this |
| Speed formula | `conf × max(0.25, 1.0 − \|θ\|/0.60) × max_linear` | Slows for both low confidence and large heading error |

---

## OAK-D Camera Integration

### Hardware Connectivity

OAK-D cameras connect via **PoE switch**, managed exclusively by the `amiga_service` C++ binary.
Direct `depthai` access always fails with `X_LINK_DEVICE_NOT_FOUND` — `amiga_service` holds
exclusive camera handles. All camera access goes through farm-ng `EventClient` (gRPC).

| Item | Value |
|---|---|
| Service host | `localhost` |
| Service port | **50010** |
| Left camera sub-service | `oak0` (connected) |
| Right camera sub-service | `oak1` (hardware not connected — will log one-time warning) |
| Stream paths | `/rgb`, `/disparity`, `/left`, `/right`, `/imu` |
| Image encoding | JPEG bytes — decode with `cv2.imdecode(np.frombuffer(data, "uint8"), cv2.IMREAD_UNCHANGED)` |
| RGB frame size | ~277 KB JPEG → HxWx3 BGR uint8 |
| Disparity | Grayscale JPEG uint8 → `depth_mm = BASELINE_MM × FOCAL_PX / disparity_px` |
| OAK-D calibration | baseline ≈ 75 mm, focal ≈ 452 px @ 640 px width |

### Obstacle Strip Geometry

Side-mounted cameras see the robot's forward centreline path at the **inner image edge**,
not the image centre. The obstacle strip is shifted accordingly via `col_centre_frac`:

| Camera | Position | `col_centre_frac` | Effective detection range | Why |
|---|---|---|---|---|
| Left | −0.915 m | **0.80** | 1.66–5.0 m forward | Robot path at col ~512 (inner right edge) |
| Right | +0.915 m | **0.20** | 1.66–5.0 m forward | Robot path at col ~160 (inner left edge) |

At `col_centre_frac = 0.5` (image centre), the robot centreline would only appear at ≥ 5.7 m —
useless for close obstacle detection. The inner-edge strips fill the LiDAR blind zone (< 1.5 m).

### Connectivity Diagnostics

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

## LiDAR Tilt Correction

The VLP-16 is mounted with a **15° forward (nose-down) tilt**. Without correction, raw sensor Z
is not vertical, causing large height calculation errors:

| Point | Without correction | With correction |
|---|---|---|
| Onion crop at 2 m, true h = 0.30 m | h = **0.83 m** (above crop band → invisible) | h = **0.30 m** ✓ |
| Ground return at 3 m, true h = 0.00 m | h = **0.80 m** (triggers safety stop!) | h = **0.00 m** ✓ |
| Person at 1.5 m, true h = 0.90 m | h = **1.28 m** (above threshold, detected) | h = **0.90 m** ✓ |

**Rotation applied** in `lidar/obstacle_filter.py::tilt_correct_pts()`, called in
`navigation/row_navigator.py` **after** self-filtering, **before** all height-based logic:

```
y_world =  y_sensor · cos(θ) + z_sensor · sin(θ)
z_world = −y_sensor · sin(θ) + z_sensor · cos(θ)
```

Use `--lidar-tilt 0` for a flat mount. Default is `--lidar-tilt 15.0`.

The self-filter threshold does not change with tilt — planar range (XY) varies by only 3.4%
at 15°, so the 1.5 m boundary is effectively tilt-invariant.

---

## Row Detection Algorithms

### HSV Green Centroid (default — `--detector hsv`)

1. Crop image to middle-third width, lower two-thirds height
2. Convert BGR → HSV; apply colour mask (configurable per crop via `--hsv-*` flags)
3. If green fraction < 8% → no detection this frame
4. Compute centroid → lateral offset: `m_per_px = 2 · depth · tan(hFOV/2) / width`
5. Fit PCA to green pixel cloud → heading (only when **linearity ≥ 0.30** to avoid random
   directions from round blobs at row-end)
6. EMA-smooth result (α = 0.30); decay confidence × 0.5 on no-detection frames

**HSV tuning for different crops:**

| Crop / test material | `--hsv-h-lo` | `--hsv-h-hi` | `--hsv-s-lo` |
|---|---|---|---|
| Onion / green plants | 35 | 85 | 40 |
| Brown cardboard (lab test) | 10 | 25 | 25 |
| Blue markers | 100 | 130 | 40 |

### Depth-Edge Geometry (`--detector depth-edge`)

Colour-independent; works in any lighting condition.

1. **Heading** — Canny edge detection + HoughLinesP on RGB; keep vertically-oriented lines;
   median slope → heading angle
2. **Lateral offset** — smooth per-column mean depth; find `argmax` (deepest column = inter-row
   gap); validate that both flanks are shallower (rejects open-environment false matches)
3. **Validity gate** — only report a camera side valid when depth contrast ≥ threshold; Hough
   lines alone (without depth confirmation) are rejected to prevent ambient structural edges
   from biasing heading in open environments

---

## Known Issues and Fixes

| Symptom | Root Cause | Fix Applied |
|---|---|---|
| Permanent `OBSTACLE_WAIT — FWD@0.7m` | Robot frame LiDAR returns at 0.72 m passed self-filter | Raised `self_radius` 1.0 → **1.5 m** |
| Permanent `OBSTACLE_WAIT — FWD@2.2m` | `obstacle_height` (0.45 m) below `LIDAR_MOUNT_HEIGHT`; entire field was obstacle | Raised `obstacle_height` 0.45 → **0.75 m** |
| `L-TIRE` false positives | Adjacent onion row canopy (h ≈ 0.80–0.84 m) in tire zone | Added `--tire-height`; set **0.85 m** for onion fields |
| Stuck in ACQUIRE (conf never reaches 0.55) | Robot off-centre reduces PCA linearity; threshold unreachable | Lowered `acquire_conf` 0.55 → **0.45** |
| `CAM-LEFT@1.8m` persistent OBSTACLE_WAIT | Single camera-blocked frame immediately set `cam_blocked = True` | Added `--cam-block-frames 3`: requires 3 consecutive blocked frames |
| `WARNING:oak/client: no matching topics` flood | farm-ng EventClient retries gRPC subscription every ~0.5 s, logging each attempt | Log level set to ERROR + one-time "offline" message + stop retrying on NOT_FOUND |
| `X_LINK_DEVICE_NOT_FOUND` on camera init | `amiga_service` holds exclusive depthai handles; direct access impossible | Rewrote `oak_driver.py` to use farm-ng `EventClient` → localhost:50010 |
| Row not detected with 15° tilt (ACQUIRE forever) | `h = z + LIDAR_MOUNT_HEIGHT` wrong with tilted mount; crop appears above crop band | Added tilt rotation via `tilt_correct_pts()` in `obstacle_filter.py` |
| Ground returns trigger safety stop with 15° tilt | Without tilt correction ground returns appear at h ≈ 0.80 m → above obstacle threshold | Same tilt correction fix |
| Obstacle not detected by cameras (`safe=clear`) | Depth strip at image centre; robot centreline at col ~480/160 from side cameras | Added `col_centre_frac = 0.80/0.20` to shift strip to inner camera edge |
| Heading chaos ±70° at row end | PCA on round green blob has arbitrary principal direction; flips between frames | Added eigenvalue linearity check (threshold 0.30) in `row_detector_visual.py` |
| `depth-edge` robot turns right (open room) | `argmax(col_smooth)` found far wall; no right flank for gap validation | Added local-maximum flank validation in `_lateral_from_depth` |
| `depth-edge` robot turns left (structural lines) | Hough detected warehouse edges without depth confirmation | Changed validity gate: `depth_conf == 0` → side invalid regardless of Hough |
| Ctrl+C traceback during gRPC cleanup | `except Exception` doesn't catch `KeyboardInterrupt` (`BaseException`) | Changed to `except BaseException: pass` in cleanup gather |
| `RuntimeWarning: Mean of empty slice` | `np.nanmean` called on all-NaN column slice | Pre-check valid columns with `~np.all(np.isnan(band), axis=0)` |

---

## File Map

```
auto_navigation_vidalia/
├── scripts/
│   ├── row_follow.py              # LiDAR row follow entry point
│   ├── cam_follow.py              # Camera-only entry point
│   ├── test_canbus.py             # canbus connectivity test
│   ├── test_lidar.py              # LiDAR connectivity test
│   ├── lidar_validate.py          # LiDAR health check
│   ├── visualize_lidar.py         # Live LiDAR point cloud viewer
│   └── setup_lidar_network.sh     # Configure 192.168.1.x static IP for VLP-16
├── navigation/
│   ├── row_navigator.py           # State machine: ACQUIRE/FOLLOW/OBSTACLE_WAIT/ROW_END
│   ├── row_navigator_cam.py       # Camera-only state machine variant
│   ├── row_perception.py          # PCA-based LiDAR row detector + RowEstimate EMA
│   ├── row_safety.py              # 3-zone obstacle safety monitor
│   └── row_controller.py          # Pure-pursuit speed/steering controller
├── lidar/
│   ├── lidar_driver.py            # Async UDP VLP-16 driver (vectorised scan_stream_np)
│   └── obstacle_filter.py         # tilt_correct_pts() + LIDAR_MOUNT_HEIGHT constant
├── camera/
│   ├── oak_driver.py              # farm-ng EventClient OAK-D driver (RGB + depth)
│   ├── depth_obstacle.py          # Depth inner-edge obstacle detector (col_centre_frac)
│   ├── row_detector_visual.py     # HSV green centroid + linearity-gated PCA heading
│   └── row_detector_depth_edge.py # Canny/Hough heading + depth-gap lateral detector
├── canbus/
│   └── canbus_interface.py        # Twist2d send via farm-ng gRPC request_reply("/twist")
├── ros2_bridge/                   # ROS 2 Foxy visualization (Docker — runs on Amiga Brain)
│   ├── Dockerfile                 # FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1
│   ├── vidalia_node.py            # ROS2 node: reads /dev/shm/, publishes 5 topics at 12 Hz
│   ├── entrypoint.sh              # Starts vidalia_node + foxglove_bridge inside container
│   ├── start.sh                   # docker build + docker run --runtime nvidia
│   ├── install_autostart.sh       # Installs persistent systemd user service
│   └── rviz/vidalia.rviz          # Pre-configured RViz2 layout
├── config/
│   ├── service_configs/
│   │   ├── canbus_config.json     # canbus service :6001
│   │   └── filter_config.json     # filter service :20001
│   └── velodyne_transform.yaml    # VLP-16 static transform (for ROS 2 stack)
├── slam/                          # SLAM modules (future use)
├── src/vidalia_bringup/           # ROS 2 colcon package (dev PC only)
├── service_config.json            # farm-ng service config
└── requirements.txt               # farm-ng-core, farm-ng-amiga, protobuf, numpy
```

---

## Setup

### On the Amiga Brain

```bash
# SSH into the brain:
ssh farm-ng-user-laserweeding@100.66.121.56

# Activate the pre-installed farm-ng venv (required every session):
source /farm_ng_image/venv/bin/activate

cd ~/auto_navigation_vidalia

# One-time: install Python deps if not already present:
pip install -r requirements.txt
```

The farm-ng venv at `/farm_ng_image/venv/` is persistent (NVMe). Everything under `~/`
survives reboots. Nothing outside `~/` (e.g. `/opt/ros/`) persists.

### One-Time: Build and Cache the ROS 2 Docker Image

```bash
# Build the L4T-optimised ROS 2 Foxy image (one time, ~10–20 min):
bash ros2_bridge/start.sh
# Image is stored in /var/lib/docker (NVMe) — survives every reboot.
# Subsequent starts use the cached image and take ~5 seconds.
```

### Verify LiDAR Network

```bash
# Set static IP for VLP-16 network interface (one-time):
bash scripts/setup_lidar_network.sh

# Verify LiDAR is reachable:
ping 192.168.1.201

# Run LiDAR health check:
python3 scripts/lidar_validate.py
```

---

## Planned Next Steps

- **Multi-row coverage** — headland turns + row counter for unattended field runs
- **Laser weed control** — integrate laser firing gate with row position + weed detection model
- **Confidence-based speed** — increase speed above 0.30 m/s once field parameters are locked
- **GPS/IMU fusion** — integrate filter service (:20001) for global position logging and
  return-to-start after field runs
- **RViz2 panel** — add dedicated row detection overlay panel to `vidalia.rviz` for field demos
