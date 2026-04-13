#!/usr/bin/env python3
"""
lidar_driver.py — Velodyne VLP-16 UDP packet receiver.

The VLP-16 is NOT a farm-ng gRPC service — it communicates over its own
proprietary UDP protocol on port 2368.  This driver binds to the data port,
receives raw 1206-byte firing packets, and converts them into 3D point lists.

Network setup:
    VLP-16 IP:   192.168.1.201  (factory default)
    Receive port: 2368           (data port, UDP)
    Bind address: "" (0.0.0.0)  (receive from all interfaces)

    On the Amiga brain, ensure the network interface connected to the VLP-16
    has an IP in the 192.168.1.x /24 subnet (e.g. 192.168.1.100).

Coordinate frame (robot-centric, right-hand rule):
    X  — right
    Y  — forward (direction of travel)
    Z  — up

The VLP-16 mount is at base_link origin offset:
    x=+1.130 m (forward), z=+0.800 m (up)
    (matches tf_static_base_to_velodyne.launch.py and amiga_min.urdf)

For Foxglove visualisation, the brain's built-in Foxglove service streams
sensor data to a browser — no ROS 2 needed on the dev PC.
"""

from __future__ import annotations

import asyncio
import math
import socket
import struct
from dataclasses import dataclass
from typing import AsyncIterator, List

# ---------------------------------------------------------------------------
# VLP-16 constants
# ---------------------------------------------------------------------------
VELODYNE_DATA_PORT = 2368
VELODYNE_HOST = "192.168.1.201"

# 1 raw distance unit = 2 mm
VELODYNE_DISTANCE_UNIT = 0.002   # metres

# 1 raw azimuth unit = 0.01°
VELODYNE_ROTATION_UNIT = 0.01    # degrees

# Packet structure: 12 data blocks × 100 bytes + 6-byte timestamp + 4-byte status = 1206 bytes
VELODYNE_PACKET_SIZE = 1206
VELODYNE_BLOCK_COUNT = 12
VELODYNE_BLOCK_SIZE = 100
VELODYNE_BLOCK_FLAG = 0xEEFF

# VLP-16 vertical elevation angles per channel (degrees), channels 0-15
VLP16_VERTICAL_ANGLES = (
    -15.0,  1.0,
    -13.0,  3.0,
    -11.0,  5.0,
     -9.0,  7.0,
     -7.0,  9.0,
     -5.0, 11.0,
     -3.0, 13.0,
     -1.0, 15.0,
)


@dataclass(slots=True)
class VelodynePoint:
    """Single calibrated 3D return from the VLP-16."""
    x: float          # metres (right of sensor)
    y: float          # metres (forward of sensor)
    z: float          # metres (up from sensor)
    intensity: int    # 0-255 return intensity
    ring: int         # channel index 0-15 (elevation)
    azimuth: float    # degrees 0-359.99


def _parse_packet(raw: bytes) -> List[VelodynePoint]:
    """Parse a single 1206-byte VLP-16 data packet into calibrated 3D points."""
    points: List[VelodynePoint] = []

    for blk in range(VELODYNE_BLOCK_COUNT):
        offset = blk * VELODYNE_BLOCK_SIZE
        flag, az_raw = struct.unpack_from("<HH", raw, offset)
        if flag != VELODYNE_BLOCK_FLAG:
            continue

        azimuth_deg = az_raw * VELODYNE_ROTATION_UNIT  # 0.0 – 359.99°

        # Each block contains 2 firing sequences of 16 channels each.
        for firing in range(2):
            ch_offset = offset + 4 + firing * 48
            for ch in range(16):
                data_off = ch_offset + ch * 3
                dist_raw, intensity = struct.unpack_from("<HB", raw, data_off)
                if dist_raw == 0:
                    continue  # no return

                dist = dist_raw * VELODYNE_DISTANCE_UNIT
                vert_rad = math.radians(VLP16_VERTICAL_ANGLES[ch])
                horiz_rad = math.radians(azimuth_deg)

                x = dist * math.cos(vert_rad) * math.sin(horiz_rad)
                y = dist * math.cos(vert_rad) * math.cos(horiz_rad)
                z = dist * math.sin(vert_rad)

                points.append(
                    VelodynePoint(
                        x=x,
                        y=y,
                        z=z,
                        intensity=intensity,
                        ring=ch,
                        azimuth=azimuth_deg,
                    )
                )

    return points


class LidarDriver:
    """
    Async UDP receiver for the Velodyne VLP-16.

    Yields one full 360° scan (~32,000 points) per iteration at ~10 Hz.

    Usage::

        async with LidarDriver() as driver:
            async for points in driver.scan_stream():
                # points: list[VelodynePoint]
                nearest = min((p.y for p in points if p.y > 0), default=float("inf"))
                print(f"Nearest forward return: {nearest:.2f} m")
    """

    def __init__(
        self,
        host: str = VELODYNE_HOST,
        port: int = VELODYNE_DATA_PORT,
    ) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    async def __aenter__(self) -> "LidarDriver":
        self._loop = asyncio.get_running_loop()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self._sock.bind(("", self._port))
        return self

    async def __aexit__(self, *_) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    async def _recv_packet(self) -> bytes:
        assert self._loop is not None and self._sock is not None
        return await self._loop.sock_recv(self._sock, VELODYNE_PACKET_SIZE)

    async def scan_stream(self) -> AsyncIterator[List[VelodynePoint]]:
        """
        Yield one complete 360° scan per iteration (~10 Hz).

        Scans are accumulated until the azimuth wraps back past 0°, which
        signals the start of a new revolution.
        """
        accumulated: List[VelodynePoint] = []
        last_azimuth = 0.0

        while True:
            raw = await self._recv_packet()
            if len(raw) < VELODYNE_PACKET_SIZE:
                continue

            # Peek at first block azimuth to detect scan wrap
            first_az_raw = struct.unpack_from("<H", raw, 2)[0]
            first_az = first_az_raw * VELODYNE_ROTATION_UNIT

            if first_az < last_azimuth and accumulated:
                yield accumulated
                accumulated = []

            last_azimuth = first_az
            accumulated.extend(_parse_packet(raw))
