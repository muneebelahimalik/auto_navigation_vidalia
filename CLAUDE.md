# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Autonomous navigation workspace for the farm-ng Amiga robot using a Velodyne VLP-16 LiDAR.
Goal: fully autonomous laser-based weed control in onion fields.

Two parallel stacks are maintained:

| Stack | Entry point | When to use |
|---|---|---|
| **Native Python** (recommended on brain) | `python3 main.py` | Amiga brain (camphor-clone) — ROS 2 not installed |
| **ROS 2 Foxy** (dev PC / future) | `ros2 launch vidalia_bringup …` | Development PC with ROS 2 Foxy installed |

**System — Amiga brain (camphor-clone)**:
- OS: Ubuntu 20.04.6 LTS (Focal Fossa), ARM64
- Python: 3.8.10
- **ROS 2 Foxy is NOT installed** on the brain.  The base image uses an overlay
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

---

### Native Python Stack (Amiga brain — no ROS 2 required)

**One-time setup (on the brain):**
```bash
# Option A — activate the pre-installed farm-ng venv (preferred):
source /farm_ng_image/venv/bin/activate

# Option B — install into user Python:
bash scripts/install_farmng.sh
```

**Run:**
```bash
python3 main.py
python3 main.py --config /path/to/service_config.json
```

**Source environment:**
```bash
source scripts/env.sh    # activates farm-ng venv and/or ROS 2 if available
```

---

### ROS 2 Foxy Workspace (development PC only)

> **Note:** ROS 2 Foxy is NOT installed on the Amiga brain.
> Run `scripts/build.sh` only on a PC with ROS 2 Foxy installed.

**Build:**
```bash
source /opt/ros/foxy/setup.bash
cd ~/auto_navigation_vidalia
colcon build --symlink-install
source install/setup.bash
```
Or use the helper script: `bash scripts/build.sh`

---

### Mapping (first run in a new field)

**Build a map while driving autonomously -- RECOMMENDED for new fields:**
```bash
ros2 launch vidalia_bringup slam_nav.launch.py rviz:=true
ros2 launch vidalia_bringup slam_nav.launch.py database_path:=~/maps/field.db rviz:=true
```
Drive the robot around the field (teleop or nav goals) until the map is complete, then:
```bash
bash scripts/save_map.sh ~/maps/field
```

**Build a map with manual teleoperation only (no Nav2):**
```bash
ros2 launch vidalia_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
```

---

### Autonomous operation (known field)

**Full production run -- localization + Nav2 -- RECOMMENDED:**
```bash
ros2 launch vidalia_bringup localize_nav.launch.py database_path:=~/maps/field.db
ros2 launch vidalia_bringup localize_nav.launch.py database_path:=~/maps/field.db rviz:=true
```

**Send autonomous goals:**
```bash
# Single pose goal:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"

# Ordered waypoints (field row traversal):
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
    "{poses: [
       {header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}},
       {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}
    ]}"
```

---

### Individual sub-stacks

**Run live sensors (VLP-16 driver + static TF):**
```bash
ros2 launch vidalia_bringup sensors_live.launch.py
```

**Run full SLAM stack (VLP-16 + Amiga gRPC bridge + SLAM, no Nav2):**
```bash
ros2 launch vidalia_bringup slam_full.launch.py rviz:=true database_path:=~/maps/field.db
```

**Run only Nav2 (sensors + SLAM already running separately):**
```bash
ros2 launch vidalia_bringup nav2.launch.py
```

**Run only the Amiga bridge nodes (odometry + cmd_vel relay):**
```bash
ros2 launch vidalia_bringup amiga_bringup_nodes.launch.py
ros2 launch vidalia_bringup amiga_bringup_nodes.launch.py use_ekf:=true
```

**Run SLAM only (sensors already running separately):**
```bash
ros2 launch vidalia_bringup slam_rtabmap_lidar3d.launch.py rviz:=true database_path:=~/maps/field.db
```

**Localise against a saved map (no Nav2):**
```bash
ros2 launch vidalia_bringup slam_localization.launch.py database_path:=~/maps/field.db
```

**Run bag replay + SLAM:**
```bash
ros2 launch vidalia_bringup slam_bag_replay.launch.py
ros2 launch vidalia_bringup slam_bag_replay.launch.py bag_path:=/path/to/bag playback_rate:=0.5 database_path:=~/maps/field.db
```

**Save the built map (while SLAM is running):**
```bash
bash scripts/save_map.sh                     # saves to ~/maps/<timestamp>/
bash scripts/save_map.sh /data/field_map     # saves to specified directory
```

**Run tests (dev PC only — requires ROS 2 + colcon):**
```bash
colcon test --packages-select vidalia_bringup
colcon test-result --verbose
```

---

## Architecture

### Native Python Stack (no ROS 2)

```
Velodyne VLP-16 (192.168.1.201 UDP :2368)
        ↓  lidar/lidar_driver.py    — raw UDP packets → VelodynePoint list (10 Hz)
navigation/nav_logic.py             — obstacle avoidance, speed control
        ↓  canbus/canbus_interface.py
Amiga canbus service (:6001 Tailscale)
        ↓  Twist2d via request_reply("/twist")
Amiga wheels
```

**Key files:**
| File | Purpose |
|---|---|
| `main.py` | Entry point; loads `service_config.json`, starts LiDAR + nav loop |
| `service_config.json` | Combined `EventServiceConfigList` for canbus (:6001) + filter (:20001) |
| `requirements.txt` | `farm-ng-core>=2.0.0`, `farm-ng-amiga>=2.0.0`, `protobuf>=3.20` |
| `canbus/canbus_interface.py` | `CanbusInterface` — `send_twist(linear, angular)` via `request_reply` |
| `lidar/lidar_driver.py` | `LidarDriver` — async UDP receiver, yields 360° scans |
| `navigation/nav_logic.py` | `NavLogic` — reactive forward navigation with obstacle stop |

**farm-ng OS 2.0 API pattern (DO NOT use deprecated OS 1.0 classes):**
```python
from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig

# Send a velocity command:
await client.request_reply("/twist", Twist2d(linear_velocity_x=0.5, angular_velocity=0.0))

# Subscribe to a service:
async for event, message in EventClient(config).subscribe(config.subscriptions[0], decode=True):
    ...
```

---

### ROS 2 Stack (Foxy — dev PC)

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
    |  canbus service :6001  ->  AmigaTpdo1 (wheel velocity, state)
    |  filter service :20001 ->  FilterState (GPS+IMU pose, heading)
    |
    v  amiga_ros2_bridge  (native ROS 2 gRPC, farm-ng OS 2.0 SDK)
         /amiga/vel  (TwistStamped)   <- measured speed
         /amiga/pose (Odometry/world) <- GPS pose (when converged)
         /cmd_vel    (Twist)          -> forwarded as Twist2d via request_reply("/twist")
    |
    +-- amiga_odometry      /amiga/vel -> /wheel_odom (dead-reckoning)
    |
    v  Velodyne VLP-16  (192.168.1.201:2368)
         /velodyne_packets -> /velodyne_points (PointCloud2, 10 Hz)
    |
    +-- icp_odometry        /velodyne_points -> /odom  +  odom->base_link TF
    +-- rtabmap             localization-only (database loaded, map->odom TF)
    |                       publishes /map (OccupancyGrid) for Nav2 costmaps
    |
    └── Nav2
         controller_server   -- local costmap (10 m rolling) + Regulated Pure Pursuit
         planner_server      -- global costmap (full map) + NavFn/A* global planner
         behavior_server     -- spin / back-up / wait recoveries
         bt_navigator        -- Navigate-to-Pose BT action
         waypoint_follower   -- /follow_waypoints action (field row traversal)
         lifecycle_manager   -- activates all Nav2 nodes
```

### Node descriptions
1. **`amiga_ros2_bridge`** -- native gRPC bridge; OS 2.0 SDK; subscribe canbus via `config.subscriptions[0]` + 20s first-message timeout; filter via `decode=True`
2. **`amiga_odometry`** -- `/amiga/vel` -> `/wheel_odom`; midpoint RK2 integration; soil covariances
3. **`amiga_velocity_bridge`** -- fallback `/cmd_vel` relay; only needed without gRPC bridge
4. **`velodyne_driver_node`** -- UDP packets from VLP-16 at `192.168.1.201:2368`
5. **`velodyne_transform_node`** -- packets -> `PointCloud2` on `/velodyne_points`
6. **`icp_odometry`** -- ICP on pointcloud; publishes `odom -> base_link` TF at ~10 Hz
7. **`rtabmap`** -- 3D map, `map -> odom` TF; mapping or localization-only mode
8. **`controller_server`** -- Regulated Pure Pursuit local planner; max 1.5 m/s; 30 cm goal tolerance
9. **`planner_server`** -- NavFn/A* global planner; 10 cm costmap resolution
10. **`behavior_server`** -- spin / back-up / wait recoveries
11. **`bt_navigator`** -- NavigateToPose and FollowWaypoints BT actions
12. **`waypoint_follower`** -- executes ordered waypoint lists (field row traversal)

---

## Row-Follow Stack (Native Python — Primary Field Operation)

This is the actively developed stack for autonomous onion-row following.
Entry point: `scripts/row_follow.py`

### Data Flow

```
Velodyne VLP-16 (UDP :2368, 10 Hz)
        ↓  lidar/lidar_driver.py       — raw UDP → numpy Nx3 point array
        ↓  lidar/obstacle_filter.py    — self-filter (planar range < self_radius)
        ↓
   ┌────┴──────────────────────────┐
   │  navigation/row_perception.py  │  — PCA row detection → RowEstimate (lateral_offset, heading, conf)
   │  navigation/row_safety.py      │  — obstacle zone check → SafetyStatus
   └────┬──────────────────────────┘
        │
   OAK-D cameras (optional, async)
        ↓  camera/oak_driver.py        — EventClient→localhost:50010, RGB + depth frames
        ↓  camera/depth_obstacle.py    — depth centre-strip → DepthObstacleStatus (blind-zone fill)
        ↓  camera/row_detector_visual.py — HSV green centroid → VisualRowEstimate (lateral supplement)
        │
        ↓  navigation/row_navigator.py  — state machine + LiDAR/camera fusion
        ↓  navigation/row_controller.py — pure-pursuit → (linear_vel, angular_vel)
        ↓  canbus/canbus_interface.py   — Twist2d via request_reply("/twist")
Amiga wheels
```

### Key Files

| File | Purpose |
|---|---|
| `scripts/row_follow.py` | CLI entry point; parses all flags, wires tasks, runs asyncio loop |
| `navigation/row_navigator.py` | State machine: ACQUIRE → FOLLOW → ROW_END / OBSTACLE_WAIT |
| `navigation/row_perception.py` | PCA-based row detector; EMA-smoothed RowEstimate |
| `navigation/row_safety.py` | Three-zone obstacle monitor (forward + left/right tire tracks) |
| `navigation/row_controller.py` | Pure-pursuit speed/steering controller |
| `lidar/lidar_driver.py` | Async UDP LiDAR driver; vectorised numpy scan parser |
| `lidar/obstacle_filter.py` | Self-filter + ground filter; defines `LIDAR_MOUNT_HEIGHT` |
| `camera/oak_driver.py` | Async OAK-D driver via farm-ng EventClient (localhost:50010, service_name=oak0) |
| `camera/depth_obstacle.py` | Depth-frame centre-strip obstacle detector |
| `camera/row_detector_visual.py` | HSV green-centroid lateral offset estimator |

---

### Sensor Frame Convention

- **X** = right, **Y** = forward, **Z** = up (LiDAR sensor frame)
- Ground-relative height: `h = z + LIDAR_MOUNT_HEIGHT`
- `LIDAR_MOUNT_HEIGHT = 0.699 m` (defined in `lidar/obstacle_filter.py`)
- Crop geometry: onion canopy h ≈ 0.10–0.60 m; adjacent row canopy h ≈ 0.70–0.85 m

---

### Tuned Parameters (current working values for onion field)

#### Self-filter (`lidar/obstacle_filter.py` / `--self-radius`)
| Parameter | Value | Rationale |
|---|---|---|
| `self_radius` | **1.5 m** | Robot frame inner cage seen at ~0.72 m planar range; 1.5 m clears all body returns without cutting into crop ROI (which starts at y=1.5 m) |

#### Row Perception (`navigation/row_perception.py`)
| Parameter | Value | Rationale |
|---|---|---|
| Crop height band | h ∈ [0.05, 0.60] m | Onion plants; excludes ground and above-canopy clutter |
| ROI depth | y ∈ [1.5, 7.0] m | Starts past self-filter; ends where row geometry becomes unreliable |
| ROI half-width | \|x\| ≤ 0.80 m | Captures crop row ± shoulder without pulling in adjacent rows |
| PCA linearity threshold | > 0.20 | Below this the cluster is not line-like; confidence forced to 0 |
| Density normaliser | n / 130 | 130 points = full confidence at typical VLP-16 density |
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
| `obstacle_height` (forward) | **0.75 m** | Above LIDAR_MOUNT_HEIGHT (0.699 m); onion plants (h≤0.60) pass through; humans/posts stop the robot |
| `tire_obstacle_height` | **0.85 m** (field default) | Raised above adjacent crop canopy (h≈0.80–0.84 m) to eliminate L-TIRE false positives from neighbouring rows |
| `tire_track` | 0.915 m | Half-track width of Amiga |
| `tire_half_width` | 0.25 m | ± corridor around each wheel centreline |
| `tire_dist` | 2.5 m | Same stopping horizon as forward zone |
| `near` | 0.20 m | Ignore returns closer than this (sensor artefacts at beam origin) |
| `forward_min_points` | 4 | Minimum LiDAR returns to trigger forward stop |
| `tire_min_points` | 4 | Minimum LiDAR returns to trigger tire-track stop |

#### Pure-Pursuit Controller (`navigation/row_controller.py`)
| Parameter | Value | Rationale |
|---|---|---|
| `lookahead` | 2.0 m | Look-ahead distance for pure pursuit |
| `max_linear` | 0.30 m/s | Conservative field speed |
| `min_linear` | 0.08 m/s | Minimum creep speed when turning hard |
| `min_confidence` | 0.35 | Zero output below this |
| Speed formula | `conf × max(0.25, 1.0 − \|θ\|/0.60) × max_linear` | Slows for both low confidence and large heading error |

---

### Recommended Run Commands

**Perception-only debug (no canbus, no movement):**
```bash
python3 scripts/row_follow.py --debug
```

**Autonomous with tuned onion-field settings (no cameras):**
```bash
python3 scripts/row_follow.py --auto --tire-height 0.85
```

**Autonomous with OAK-D cameras enabled:**
```bash
python3 scripts/row_follow.py --auto --tire-height 0.85 --camera
```

**With specific camera device IDs:**
```bash
python3 scripts/row_follow.py --auto --tire-height 0.85 --camera \
    --cam-left-id <MXID_LEFT> --cam-right-id <MXID_RIGHT>
```

**Save debug point-cloud frames:**
```bash
python3 scripts/row_follow.py --auto --tire-height 0.85 --save-dir /tmp/scans
```

### Full CLI Flag Reference (`scripts/row_follow.py`)

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable canbus and send velocity commands to wheels |
| `--rows N` | — | Stop after N rows completed |
| `--headland D` | — | Headland turn distance in metres |
| `--slam` | off | Enable SLAM odometry integration |
| `--speed S` | 0.30 | Override max linear speed (m/s) |
| `--roi-x W` | 0.80 | Row detection ROI half-width (m) |
| `--crop-min H` | 0.05 | Minimum crop height in LiDAR frame (m) |
| `--crop-max H` | 0.60 | Maximum crop height in LiDAR frame (m) |
| `--self-radius R` | **1.5** | Self-filter radius — discard returns closer than R (m) |
| `--acquire-conf C` | **0.45** | Confidence threshold to enter FOLLOW from ACQUIRE |
| `--obstacle-height H` | **0.75** | Forward zone obstacle height threshold (m) |
| `--tire-height H` | (= obstacle-height) | Tire zone height threshold; set **0.85** for onion fields |
| `--camera` | off | Enable OAK-D stereo cameras |
| `--cam-left-id ID` | "" | farm-ng service name for left camera (default: oak0) |
| `--cam-right-id ID` | "" | farm-ng service name for right camera (default: oak1) |
| `--cam-x X` | 0.915 | Camera lateral offset from centreline (m) |
| `--cam-stop-dist D` | 1.2 | Camera depth stop distance (m) |
| `--debug` | off | Print verbose per-frame perception output |
| `--save-dir DIR` | — | Save raw point-cloud numpy arrays to DIR |
| `--no-validate` | off | Skip LiDAR startup validation |

---

### State Machine

```
         ┌──────────────────┐
    boot  │     ACQUIRE      │  confidence ≥ acquire_conf (0.45)
  ───────>│  5 consecutive   │─────────────────────────────────────>┐
          │  frames needed   │                                       │
          └──────────────────┘                                       │
                   ↑ obstacle clears (1.5 s consecutive)            ↓
                   │                                        ┌──────────────┐
          ┌──────────────────┐   obstacle detected          │    FOLLOW    │
          │  OBSTACLE_WAIT   │<─────────────────────────────│  pure-pursuit│
          └──────────────────┘                              │  cmd sent    │
                                                            └──────┬───────┘
                                                                   │ row end
                                                                   ↓
                                                            ┌──────────────┐
                                                            │   ROW_END    │
                                                            └──────────────┘
```

---

### Camera-Only Row Following (no LiDAR)

Entry point: `scripts/cam_follow.py`

Uses only the two OAK-D cameras — no Velodyne LiDAR required.

```
OAK-D left + right (farm-ng EventClient → localhost:50010, service_name=oak0/oak1)
        ↓  camera/oak_driver.py         — RGB + depth frames
        ↓  camera/row_detector_visual.py — HSV green → lateral offset + PCA heading
        ↓  camera/depth_obstacle.py      — depth centre-strip → obstacle blocked/clear
        ↓
   navigation/row_navigator_cam.py  — CamRowNavigator (same state machine)
        ↓  navigation/row_controller.py  — pure-pursuit (unchanged)
        ↓  canbus/canbus_interface.py
Amiga wheels
```

**Key differences from LiDAR mode:**

| Aspect | LiDAR mode | Camera-only mode |
|---|---|---|
| Row detection | PCA on 3-D point cloud | HSV green centroid + PCA on 2-D pixel mask |
| Heading estimate | LiDAR PCA (accurate) | Image-space PCA with FOV correction (approx.) |
| Obstacle safety | 3-zone monitor (fwd + 2 tires) | Depth centre-strip both cameras only |
| Tire-track monitoring | Yes | No |
| Row-end detection | `row_end_confidence` from point density | `green_fraction` drops < threshold |
| Confidence range | 0–1 (typical 0.70+ in good row) | 0–0.50 (lower, camera-based) |
| Default max speed | 0.30 m/s | **0.20 m/s** (lower for safety) |
| Lighting sensitivity | None | Degrades in harsh sun/shadow |

**Run commands:**
```bash
# Perception-only (no motion) — verify green detection:
python3 scripts/cam_follow.py

# Autonomous:
python3 scripts/cam_follow.py --auto

# Multi-row with headland turns:
python3 scripts/cam_follow.py --auto --rows 4 --headland
```

**cam_follow.py CLI flags:**

| Flag | Default | Description |
|---|---|---|
| `--auto` | off | Enable motion |
| `--rows N` | 1 | Number of rows |
| `--headland` | off | Open-loop headland turns |
| `--speed M` | **0.20** | Max speed m/s |
| `--cam-left-id S` | "" | Left OAK-D farm-ng service name (default: oak0) |
| `--cam-right-id S` | "" | Right OAK-D farm-ng service name (default: oak1) |
| `--cam-x M` | 0.915 | Camera lateral offset (m) |
| `--cam-stop-dist M` | 1.2 | Depth stop distance (m) |
| `--acquire-conf F` | **0.20** | Min confidence for ACQUIRE exit |
| `--acquire-green F` | **0.08** | Min green fraction for ACQUIRE exit |
| `--fps N` | 10 | Camera capture rate |

**ACQUIRE logic (camera-only):**
Both conditions must hold for `acquire_frames=8` consecutive polls:
- `confidence ≥ acquire_conf (0.20)`
- `green_fraction ≥ acquire_green (0.08)` — at least 8% of the centre strip is green

**Row-end logic (camera-only):**
`green_fraction < row_end_green (0.04)` for `row_end_frames=10` consecutive polls
(robot has driven past the last plant → green disappears)

**Heading estimation from image PCA:**
```
# Green pixel coords: (col, row) in image
PCA → principal component (dx_px, dy_px)
# Orient toward top of image (forward direction)
if dy_px > 0: flip sign
slope_px = dx_px / (-dy_px)           # px-right per px-forward
# Scale by FOV geometry
slope_world = slope_px × tan(hfov/2)/w × w / (tan(vfov/2)/h × h)
heading_rad  = atan(slope_world)
```
OAK-D defaults: hfov=73°, vfov=54°, 640×400 → scale factor ≈ 0.91

---

### OAK-D Camera Integration

Two OAK-D cameras are mounted on left and right sides of the Amiga (at ±0.915 m from centreline).

**Purpose:**
- **Depth obstacle detection** (`camera/depth_obstacle.py`): fills the LiDAR blind zone (< 1.5 m in front), stops robot if obstacle within `cam_stop_dist` (default 1.2 m)
- **Visual row confirmation** (`camera/row_detector_visual.py`): HSV green-centroid detection supplements LiDAR lateral offset estimate

**Fusion in `navigation/row_navigator.py`:**
- Camera lateral estimate blended at up to 50% of LiDAR weight
- Fused confidence: `min(1.0, lidar_conf + vis_conf × 0.3)`
- Camera depth block sets `safety.cam_blocked = True` → same OBSTACLE_WAIT as LiDAR block

**Hardware connectivity — PoE switch (CRITICAL):**

OAK-D cameras are **not USB devices** on the brain. They are connected via a **PoE switch** and
managed exclusively by the `amiga_service` C++ binary. `depthai` direct device access fails with
`X_LINK_DEVICE_NOT_FOUND` because `amiga_service` holds exclusive camera handles.

| Item | Value |
|---|---|
| Camera service host | `localhost` |
| Camera service port | **50010** |
| Sub-service for camera 1 | `oak0` (only one currently connected) |
| Sub-service for camera 2 | `oak1` (times out — hardware not connected) |
| Subscription pattern | `SubscribeRequest(uri=Uri(path='/rgb', query='service_name=oak0'))` |
| Image stream paths | `/rgb`, `/disparity`, `/left`, `/right`, `/imu` |
| Frame proto type | `OakFrame` — fields: `meta`, `image_data` (bytes) |
| Image encoding | JPEG bytes — decode with `cv2.imdecode(np.frombuffer(msg.image_data, dtype="uint8"), cv2.IMREAD_UNCHANGED)` |
| RGB frame size | ~277 KB JPEG → HxWx3 BGR uint8 |
| Disparity encoding | Grayscale JPEG uint8 (~7 KB) → convert to depth_mm: `depth_mm = BASELINE_MM * FOCAL_PX / disparity_px` |
| OAK-D calibration | baseline ≈ 75 mm, focal ≈ 452 px at 640 px width |

`camera/oak_driver.py` has been rewritten to use `EventClient` (farm-ng gRPC) instead of `depthai`.
The `device_id` / `--cam-left-id` / `--cam-right-id` CLI arguments now specify the farm-ng
sub-service name (e.g. `oak0`, `oak1`), not a depthai MXID serial.

**Diagnosing camera connectivity:**
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

### Known Issues and Resolutions

| Symptom | Root Cause | Fix Applied |
|---|---|---|
| Permanent `OBSTACLE_WAIT` — `FWD@0.7m` | Robot frame returns passing self-filter at 0.72 m | Raised `self_radius` 1.0 → **1.5 m** |
| Permanent `OBSTACLE_WAIT` — `FWD@1.2m` | `rng >= self_radius` (inclusive) let boundary returns through | Raising self_radius further to 1.5 m resolves it |
| Permanent `OBSTACLE_WAIT` — `FWD@2.2m, n=482` | `obstacle_height=0.45 m` was below LIDAR_MOUNT_HEIGHT; entire field was an "obstacle" | Raised `obstacle_height` 0.45 → **0.75 m** |
| `L-TIRE(n=47)` false positive | Adjacent crop row canopy at h≈0.80–0.84 m triggering tire zone | Added `--tire-height` flag; set to **0.85 m** for onion fields |
| Stuck in ACQUIRE (conf ≤ 0.52) | Robot 0.75 m off-centre reduces PCA linearity → confidence below old threshold 0.55 | Lowered `acquire_conf` 0.55 → **0.45** |
| `[oak_driver] X_LINK_DEVICE_NOT_FOUND` | Cameras connected via PoE switch — `amiga_service` owns exclusive depthai handles; direct depthai access is impossible | Rewrote `oak_driver.py` to use `EventClient` → localhost:50010 with `service_name=oak0` |
| `[oak_driver] THE_400_P AttributeError` | depthai 2.22.0 doesn't have `THE_400_P` (legacy note — driver no longer uses depthai) | N/A — depthai pipeline removed |
| ACQUIRE/FOLLOW/OBSTACLE_WAIT oscillation | Real environmental obstacle at ~2.2–2.5 m intermittently entering forward zone | Not a code bug; resolves in actual clear onion rows |
| `\r` terminal shows only "clear" during oscillation | Terminal carriage-return overwrites intermediate blocked frames | Use `--debug` to see every frame; transitions always print on `\n` |

---

### LiDAR Self-Filter Logic

The scan parser in `lidar/obstacle_filter.py` discards any return whose **planar range** (distance in the XY plane, ignoring height) is less than `self_radius`. This is critical because:

- The Amiga robot frame generates LiDAR returns at ~0.72 m planar range
- The crop ROI starts at y = 1.5 m, so `self_radius = 1.5 m` does not cut into crop detection
- The safety monitor's `near = 0.20 m` provides a second guard for sensor artefacts at the beam origin

The `validate_lidar_startup()` function in `obstacle_filter.py` intentionally uses the **raw scan** (without self-filter) for startup diagnostics only.

---

### Confidence Scoring Detail

Row confidence is computed in `navigation/row_perception.py`:

```
density      = min(1.0, n_crop_points / 130)
linearity    = 1 - (second_eigenvalue / first_eigenvalue)   # PCA ratio
linear_factor = max(0.0, min(1.0, (linearity - 0.20) / 0.55))
raw_conf     = density * linear_factor
smoothed_conf = ema_alpha * raw_conf + (1 - ema_alpha) * prev_conf   # alpha=0.35
```

- `linearity < 0.20` → confidence = 0 (cluster is not line-shaped)
- `linearity = 0.75` → linear_factor = 1.0 (fully linear)
- `n = 130` at `linearity = 0.75` → confidence = 1.0
- In practice, well-centred onion rows give conf ≈ 0.70–0.85 in FOLLOW

---

### Camera Depth Obstacle Detection Detail

`camera/depth_obstacle.py` examines a centre strip of the depth frame:

```
row_half = img_height * 0.40 / 2   # 40% of frame height
col_half = img_width  * 0.25 / 2   # 25% of frame width
strip    = depth[mid_row ± row_half, mid_col ± col_half]
```

A pixel is "close" if `min_dist_m (0.30) ≤ depth < stop_dist_m (1.2)`.
Blocked if `n_close ≥ min_pixels (50)`.

---

### Visual Row Detection Detail

`camera/row_detector_visual.py` finds the green-crop centroid in each OAK-D RGB frame:

1. Crop image: middle third horizontally, lower two-thirds vertically
2. Convert BGR → HSV; mask h∈[35,85], s≥40, v≥40
3. If green fraction < 8% of strip area → not valid
4. Centroid pixel → lateral offset via `m_per_px = 2 × depth × tan(hfov/2) / width`
5. EMA-smooth result (alpha=0.30); decay confidence × 0.5 when no green found

Confidence contribution: 0.20 (one side) or up to 0.50 (both sides similar fraction).
Camera lateral estimate is capped at 50% of LiDAR weight in fusion.
