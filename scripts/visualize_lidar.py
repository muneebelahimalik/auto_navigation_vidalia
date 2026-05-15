#!/usr/bin/env -S python3 -u
"""
visualize_lidar.py — Stream VLP-16 point cloud to Foxglove Studio (browser).

Publishes one foxglove.PointCloud message per 360° scan (~10 Hz) over a
Foxglove WebSocket server on port 8766.

Usage:
    source /farm_ng_image/venv/bin/activate
    cd /mnt/data/auto_navigation_vidalia
    python3 scripts/visualize_lidar.py

Then open Foxglove Studio in a browser:
    https://app.foxglove.dev  (or the desktop app)

Connect to:
    ws://100.66.121.56:8766          (over Tailscale from any device)
    ws://192.168.1.100:8766          (over Ethernet from a PC on that subnet)
    ws://localhost:8766              (locally on the Amiga)

In Foxglove Studio:
    1. "Open Connection" → "Foxglove WebSocket" → paste the URL above
    2. Add a "3D" panel  (+ → 3D)
    3. In the panel settings, add topic "/velodyne_points"
    4. The point cloud should appear immediately

Optional flags:
    --port 8767          use a different port
    --every-n 2          publish every 2nd point (halves bandwidth)
    --no-intensity       don't colour by intensity (faster)
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import struct
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from foxglove_websocket.server import FoxgloveServer
from lidar.lidar_driver import LidarDriver

# ---------------------------------------------------------------------------
# Foxglove PointCloud schema (JSON encoding)
# ---------------------------------------------------------------------------
# PackedElementField numeric types:
FLOAT32 = 9
UINT8   = 1

POINT_STRIDE = 16   # x(4) + y(4) + z(4) + intensity(1) + ring(1) + pad(2)

FIELDS = [
    {"name": "x",         "offset": 0,  "type": FLOAT32},
    {"name": "y",         "offset": 4,  "type": FLOAT32},
    {"name": "z",         "offset": 8,  "type": FLOAT32},
    {"name": "intensity", "offset": 12, "type": UINT8},
    {"name": "ring",      "offset": 13, "type": UINT8},
]

# Numpy structured dtype matching the 16-byte stride above
_POINT_DTYPE = np.dtype({
    "names":   ["x",       "y",       "z",       "intensity", "ring"],
    "formats": [np.float32, np.float32, np.float32, np.uint8,   np.uint8],
    "offsets": [0,          4,          8,          12,          13],
    "itemsize": POINT_STRIDE,
})

POINT_CLOUD_SCHEMA = json.dumps({
    "type": "object",
    "properties": {
        "timestamp":    {"type": "object",
                         "properties": {"sec":  {"type": "integer"},
                                        "nsec": {"type": "integer"}}},
        "frame_id":     {"type": "string"},
        "pose":         {"type": "object",
                         "properties": {
                             "position":    {"type": "object",
                                            "properties": {"x": {"type": "number"},
                                                           "y": {"type": "number"},
                                                           "z": {"type": "number"}}},
                             "orientation": {"type": "object",
                                            "properties": {"x": {"type": "number"},
                                                           "y": {"type": "number"},
                                                           "z": {"type": "number"},
                                                           "w": {"type": "number"}}}}},
        "point_stride": {"type": "integer"},
        "fields":       {"type": "array",
                         "items": {"type": "object",
                                   "properties": {"name":   {"type": "string"},
                                                  "offset": {"type": "integer"},
                                                  "type":   {"type": "integer"}}}},
        "data":         {"type": "string", "contentEncoding": "base64"},
    },
})


# ---------------------------------------------------------------------------
# Pack a scan into binary for Foxglove
# ---------------------------------------------------------------------------

def _pack_scan(scan, every_n: int = 1) -> bytes:
    pts = scan[::every_n] if every_n > 1 else scan
    arr = np.zeros(len(pts), dtype=_POINT_DTYPE)
    arr["x"]         = [p.x         for p in pts]
    arr["y"]         = [p.y         for p in pts]
    arr["z"]         = [p.z         for p in pts]
    arr["intensity"] = [p.intensity for p in pts]
    arr["ring"]      = [p.ring      for p in pts]
    return arr.tobytes()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def run(port: int, every_n: int) -> None:
    print("=" * 56)
    print(" VLP-16 → Foxglove WebSocket visualizer")
    print("=" * 56)
    print(f" Server:  ws://100.66.121.56:{port}  (Tailscale)")
    print(f"          ws://192.168.1.100:{port}   (Ethernet)")
    print()
    print(" Foxglove Studio:  https://app.foxglove.dev")
    print("   Open Connection → Foxglove WebSocket → paste URL")
    print("   Add panel: 3D → topic: /velodyne_points")
    print()
    if every_n > 1:
        print(f" Downsampling: every {every_n}th point")
    print(" Press Ctrl+C to stop.")
    print("=" * 56)

    async with FoxgloveServer("0.0.0.0", port, "VLP-16 LiDAR") as server:
        # Yield to the event loop so the server task can run and bind the port.
        # Without this, serve() is called lazily and the port may never open.
        await asyncio.sleep(0)
        try:
            ws_server = await asyncio.wait_for(server.wait_opened(), timeout=5.0)
            bound = [s.getsockname() for s in (ws_server.sockets or [])]
            print(f" Listening on: {bound}")
        except asyncio.TimeoutError:
            print(f" ERROR: WebSocket server failed to bind port {port}.")
            print(f"        Check: sudo ss -tlnp | grep {port}")
            return
        except Exception as e:
            print(f" ERROR starting WebSocket server: {e!r}")
            return

        chan_id = await server.add_channel({
            "topic": "/velodyne_points",
            "encoding": "json",
            "schemaName": "foxglove.PointCloud",
            "schema": POINT_CLOUD_SCHEMA,
            "schemaEncoding": None,
        })

        scan_count = 0
        t_report = time.monotonic()

        async with LidarDriver() as lidar:
            async for scan in lidar.scan_stream():
                ts_ns = time.time_ns()
                sec   = ts_ns // 1_000_000_000
                nsec  = ts_ns %  1_000_000_000

                data_bytes = _pack_scan(scan, every_n)
                payload = json.dumps({
                    "timestamp":    {"sec": sec, "nsec": nsec},
                    "frame_id":     "velodyne",
                    "pose": {
                        "position":    {"x": 0.0, "y": 0.0, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    },
                    "point_stride": POINT_STRIDE,
                    "fields":       FIELDS,
                    "data":         base64.b64encode(data_bytes).decode(),
                }).encode()

                await server.send_message(chan_id, ts_ns, payload)

                scan_count += 1
                now = time.monotonic()
                if now - t_report >= 3.0:
                    hz = scan_count / (now - t_report)
                    pts_published = len(scan) // every_n
                    kb = len(payload) / 1024
                    print(f" {pts_published:,} pts/scan  {hz:.1f} Hz  {kb:.0f} KB/msg")
                    scan_count = 0
                    t_report = now


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stream VLP-16 point cloud to Foxglove Studio"
    )
    parser.add_argument("--port",    type=int, default=8766,
                        help="WebSocket port (default: 8766)")
    parser.add_argument("--every-n", type=int, default=1, dest="every_n",
                        help="Publish every Nth point to reduce bandwidth (default: 1 = all)")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.port, args.every_n))
    except KeyboardInterrupt:
        print("\n[viz] Stopped.")


if __name__ == "__main__":
    main()
