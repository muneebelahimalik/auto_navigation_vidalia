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

**System — Development PC**:
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- ROS 2 not yet installed (install Humble for RViz/bag recording if needed).
- SSH access: `ssh farm-ng-user-laserweeding@100.66.121.56` (Tailscale)

**Ultimate goal**: fully autonomous laser-based weed control in onion fields.

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
