#!/usr/bin/env -S python3 -u
"""
slam_foxglove.py — 2D LiDAR SLAM streamed live to Foxglove Studio.

Runs entirely on the Amiga brain (no ROS 2 required).  The VLP-16
produces 360° point clouds at ~10 Hz; each scan is projected to a 2D
horizontal slice, matched against the previous scan with ICP to
estimate robot motion, and used to grow an occupancy grid map.

Everything is published over a Foxglove WebSocket server so you can
watch the map build in real time from your laptop browser.

────────────────────────────────────────────────────────────────────
QUICK START
────────────────────────────────────────────────────────────────────
On the Amiga brain (SSH from VS Code):

    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia
    python3 scripts/slam_foxglove.py

On your laptop — open any browser:

    https://app.foxglove.dev   (or install the desktop app)

    1. Click  "Open Connection"
    2. Choose "Foxglove WebSocket"
    3. URL:   ws://100.66.121.56:8766    ← Tailscale
          or  ws://192.168.1.100:8766    ← Ethernet

    4. Add a  "3D"  panel  (the + button → 3D)
    5. In panel settings, enable these topics:
         /velodyne/raw   — 3D raw scan  (sensor frame, colour by ring)
         /slam/scan      — 2D horizontal slice  (map frame, z = 0.05 m)
         /slam/map       — accumulated occupancy map  (map frame, z = 0)
         /slam/scene     — robot marker + trajectory line

    Tip: set /slam/map point size to 0.08 m so cells look solid.
    Tip: camera → top-down → follow /slam/scene robot position.
────────────────────────────────────────────────────────────────────

Options:
    --port        WebSocket port (default 8766)
    --map-size    Occupancy grid extent in metres (default 150)
    --resolution  Grid cell size in metres (default 0.10)
    --icp-points  Points used per ICP call (default 400)
    --every-n     Downsample raw 3D scan before publishing (default 2)
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from foxglove_websocket.server import FoxgloveServer
from lidar.lidar_driver import LidarDriver
from slam.slam_engine import SlamEngine
from slam.scan_matcher import extract_2d_slice, sensor_to_world

# ---------------------------------------------------------------------------
# Foxglove PointCloud schema + binary packing  (mirrors visualize_lidar.py)
# ---------------------------------------------------------------------------
FLOAT32 = 9
UINT8 = 1
POINT_STRIDE = 16   # x(4) y(4) z(4) intensity(1) ring(1) pad(2)

FIELDS_RAW = [
    {"name": "x",         "offset": 0,  "type": FLOAT32},
    {"name": "y",         "offset": 4,  "type": FLOAT32},
    {"name": "z",         "offset": 8,  "type": FLOAT32},
    {"name": "intensity", "offset": 12, "type": UINT8},
    {"name": "ring",      "offset": 13, "type": UINT8},
]

FIELDS_MAP = [
    {"name": "x",         "offset": 0,  "type": FLOAT32},
    {"name": "y",         "offset": 4,  "type": FLOAT32},
    {"name": "z",         "offset": 8,  "type": FLOAT32},
    {"name": "intensity", "offset": 12, "type": UINT8},
]

_DTYPE_RAW = np.dtype({
    "names":   ["x", "y", "z", "intensity", "ring"],
    "formats": [np.float32, np.float32, np.float32, np.uint8, np.uint8],
    "offsets": [0, 4, 8, 12, 13],
    "itemsize": POINT_STRIDE,
})

_DTYPE_MAP = np.dtype({
    "names":   ["x", "y", "z", "intensity"],
    "formats": [np.float32, np.float32, np.float32, np.uint8],
    "offsets": [0, 4, 8, 12],
    "itemsize": POINT_STRIDE,
})

_POINT_CLOUD_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "timestamp":    {"type": "object",
                         "properties": {"sec": {"type": "integer"},
                                        "nsec": {"type": "integer"}}},
        "frame_id":     {"type": "string"},
        "pose":         {"type": "object"},
        "point_stride": {"type": "integer"},
        "fields":       {"type": "array"},
        "data":         {"type": "string", "contentEncoding": "base64"},
    },
})

_SCENE_UPDATE_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "deletions": {"type": "array"},
        "entities":  {"type": "array"},
    },
})


def _pc_msg(arr: np.ndarray, fields: list, frame_id: str, sec: int, nsec: int) -> bytes:
    return json.dumps({
        "timestamp":    {"sec": sec, "nsec": nsec},
        "frame_id":     frame_id,
        "pose": {
            "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "point_stride": POINT_STRIDE,
        "fields":       fields,
        "data":         base64.b64encode(arr.tobytes()).decode(),
    }).encode()


def _pack_raw(scan, every_n: int) -> np.ndarray:
    pts = scan[::every_n] if every_n > 1 else scan
    arr = np.zeros(len(pts), dtype=_DTYPE_RAW)
    arr["x"]         = [p.x         for p in pts]
    arr["y"]         = [p.y         for p in pts]
    arr["z"]         = [p.z         for p in pts]
    arr["intensity"] = [p.intensity  for p in pts]
    arr["ring"]      = [p.ring       for p in pts]
    return arr


def _pack_world_pts(
    world_pts: np.ndarray,
    z: float,
    intensity: int,
) -> np.ndarray:
    """Pack Nx2 world-frame (x, y) points into a PointCloud at fixed z."""
    n = len(world_pts)
    arr = np.zeros(n, dtype=_DTYPE_MAP)
    if n > 0:
        arr["x"]         = world_pts[:, 0].astype(np.float32)
        arr["y"]         = world_pts[:, 1].astype(np.float32)
        arr["z"]         = np.full(n, z, dtype=np.float32)
        arr["intensity"] = intensity
    return arr


# ---------------------------------------------------------------------------
# SceneUpdate: robot sphere + heading line + trajectory strip
# ---------------------------------------------------------------------------

def _scene_msg(state, sec: int, nsec: int) -> bytes:
    px, py, theta = state.pose.x, state.pose.y, state.pose.theta

    # Robot's forward direction in world frame: R(theta) @ [0, 1] = [-sin, cos]
    # (theta=0 → robot faces world +Y; +Y is the initial forward direction)
    fwd_x = -math.sin(theta)
    fwd_y =  math.cos(theta)

    # Last 500 trajectory points as 3D points at z = 0.02 m
    traj = state.trajectory[-500:]
    traj_pts = [{"x": float(p[0]), "y": float(p[1]), "z": 0.02} for p in traj]

    ts = {"sec": sec, "nsec": nsec}
    identity_pose = {
        "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    }

    robot_entity = {
        "timestamp":   ts,
        "frame_id":    "map",
        "id":          "robot",
        "lifetime":    {"sec": 1, "nsec": 0},
        "frame_locked": False,
        "metadata":    [],
        "arrows":      [],
        "cubes":       [],
        "cylinders":   [],
        "triangles":   [],
        "texts":       [],
        "models":      [],
        # Red sphere at robot position
        "spheres": [
            {
                "pose": {
                    "position":    {"x": px, "y": py, "z": 0.35},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
                "size":  {"x": 0.30, "y": 0.30, "z": 0.30},
                "color": {"r": 1.0, "g": 0.15, "b": 0.15, "a": 1.0},
            }
        ],
        # Orange heading indicator line
        "lines": [
            {
                "type":           0,   # LINE_STRIP
                "pose":           identity_pose,
                "thickness":      0.12,
                "scale_invariant": False,
                "points": [
                    {"x": px,                     "y": py,                     "z": 0.35},
                    {"x": px + 0.80 * fwd_x,      "y": py + 0.80 * fwd_y,     "z": 0.35},
                ],
                "color":  {"r": 1.0, "g": 0.55, "b": 0.0, "a": 1.0},
                "colors": [],
                "indices": [],
            }
        ],
    }

    traj_entity = {
        "timestamp":   ts,
        "frame_id":    "map",
        "id":          "trajectory",
        "lifetime":    {"sec": 1, "nsec": 0},
        "frame_locked": False,
        "metadata":    [],
        "arrows":      [],
        "cubes":       [],
        "spheres":     [],
        "cylinders":   [],
        "triangles":   [],
        "texts":       [],
        "models":      [],
        "lines": [
            {
                "type":           0,   # LINE_STRIP
                "pose":           identity_pose,
                "thickness":      0.06,
                "scale_invariant": False,
                "points":  traj_pts,
                "color":   {"r": 0.25, "g": 0.65, "b": 1.0, "a": 0.90},
                "colors":  [],
                "indices": [],
            }
        ] if len(traj_pts) >= 2 else [],
    }

    return json.dumps({"deletions": [], "entities": [robot_entity, traj_entity]}).encode()


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def run(args: argparse.Namespace) -> None:
    print("=" * 62)
    print(" VLP-16 SLAM → Foxglove WebSocket")
    print("=" * 62)
    print(f" Server (Tailscale) : ws://100.66.121.56:{args.port}")
    print(f" Server (Ethernet)  : ws://192.168.1.100:{args.port}")
    print()
    print(" Open in browser : https://app.foxglove.dev")
    print("   Open Connection → Foxglove WebSocket → paste URL")
    print()
    print(" 3D panel topics:")
    print("   /velodyne/raw  — 3D scan, sensor frame (colour by ring)")
    print("   /slam/scan     — 2D horizontal slice, map frame")
    print("   /slam/map      — occupancy map, map frame (set pt size 0.08 m)")
    print("   /slam/scene    — robot sphere + heading + trajectory")
    print()
    print(f" Map: {args.map_size:.0f} × {args.map_size:.0f} m  |  "
          f"resolution: {args.resolution:.2f} m/cell  |  "
          f"ICP points: {args.icp_points}")
    print(" Press Ctrl+C to stop.")
    print("=" * 62)

    slam = SlamEngine(
        grid_size_m=args.map_size,
        grid_resolution=args.resolution,
        icp_points=args.icp_points,
    )

    async with FoxgloveServer("0.0.0.0", args.port, "VLP-16 SLAM") as server:
        await asyncio.sleep(0)
        try:
            ws_srv = await asyncio.wait_for(server.wait_opened(), timeout=5.0)
            bound = [s.getsockname() for s in (ws_srv.sockets or [])]
            print(f" Listening on: {bound}\n")
        except asyncio.TimeoutError:
            print(f" ERROR: could not bind port {args.port}. "
                  f"Check: sudo ss -tlnp | grep {args.port}")
            return
        except Exception as exc:
            print(f" ERROR starting server: {exc!r}")
            return

        # Register Foxglove channels
        ch_raw = await server.add_channel({
            "topic": "/velodyne/raw", "encoding": "json",
            "schemaName": "foxglove.PointCloud", "schema": _POINT_CLOUD_SCHEMA,
            "schemaEncoding": None,
        })
        ch_scan = await server.add_channel({
            "topic": "/slam/scan", "encoding": "json",
            "schemaName": "foxglove.PointCloud", "schema": _POINT_CLOUD_SCHEMA,
            "schemaEncoding": None,
        })
        ch_map = await server.add_channel({
            "topic": "/slam/map", "encoding": "json",
            "schemaName": "foxglove.PointCloud", "schema": _POINT_CLOUD_SCHEMA,
            "schemaEncoding": None,
        })
        ch_scene = await server.add_channel({
            "topic": "/slam/scene", "encoding": "json",
            "schemaName": "foxglove.SceneUpdate", "schema": _SCENE_UPDATE_SCHEMA,
            "schemaEncoding": None,
        })

        t_map_pub  = time.monotonic() - 1.0   # publish map immediately on first scan
        t_report   = time.monotonic()
        hz_counter = 0
        loop = asyncio.get_running_loop()

        async with LidarDriver() as lidar:
            async for raw_scan in lidar.scan_stream():
                ts_ns = time.time_ns()
                sec, nsec = divmod(ts_ns, 1_000_000_000)

                # Run ICP + grid update in a thread pool to avoid blocking asyncio
                await loop.run_in_executor(None, slam.process_scan, raw_scan)
                state = slam.get_state()

                # ── raw 3D scan (sensor frame) ──────────────────────────
                await server.send_message(
                    ch_raw, ts_ns,
                    _pc_msg(_pack_raw(raw_scan, args.every_n),
                            FIELDS_RAW, "velodyne", sec, nsec),
                )

                # ── 2D horizontal slice (world frame) ───────────────────
                pts_2d = extract_2d_slice(raw_scan)
                pts_2d_w = sensor_to_world(pts_2d, state.pose)
                await server.send_message(
                    ch_scan, ts_ns,
                    _pc_msg(_pack_world_pts(pts_2d_w, z=0.05, intensity=180),
                            FIELDS_MAP, "map", sec, nsec),
                )

                # ── robot + trajectory scene ────────────────────────────
                await server.send_message(
                    ch_scene, ts_ns,
                    _scene_msg(state, sec, nsec),
                )

                # ── occupancy map (1 Hz) ────────────────────────────────
                now = time.monotonic()
                if now - t_map_pub >= 1.0:
                    map_pts = slam.get_map()
                    await server.send_message(
                        ch_map, ts_ns,
                        _pc_msg(_pack_world_pts(map_pts, z=0.0, intensity=255),
                                FIELDS_MAP, "map", sec, nsec),
                    )
                    t_map_pub = now

                hz_counter += 1
                if now - t_report >= 5.0:
                    hz = hz_counter / (now - t_report)
                    p = state.pose
                    print(
                        f" scan={state.scan_count:04d} | "
                        f"x={p.x:+.2f} y={p.y:+.2f} θ={math.degrees(p.theta):+.1f}° | "
                        f"map={slam.cell_count:,} cells | "
                        f"icp_err={state.last_icp_error:.3f} m | "
                        f"{hz:.1f} Hz"
                    )
                    hz_counter = 0
                    t_report = now


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port",       type=int,   default=8766,
                   help="WebSocket port (default 8766)")
    p.add_argument("--map-size",   type=float, default=150.0,
                   help="Occupancy grid size in metres (default 150)")
    p.add_argument("--resolution", type=float, default=0.10,
                   help="Grid cell size in metres (default 0.10)")
    p.add_argument("--icp-points", type=int,   default=400,
                   help="Points used per ICP call (default 400)")
    p.add_argument("--every-n",    type=int,   default=2,  dest="every_n",
                   help="Publish every Nth raw 3D scan point (default 2)")
    args = p.parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\n[slam] Stopped.")


if __name__ == "__main__":
    main()
