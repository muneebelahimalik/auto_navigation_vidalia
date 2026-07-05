#!/usr/bin/env python3
"""
lidar_driver.py — Velodyne VLP-16 UDP packet receiver.

The VLP-16 is NOT a farm-ng gRPC service — it communicates over its own
proprietary UDP protocol on port 2368.  This driver binds to the data port,
receives raw 1206-byte firing packets, and converts them into 3D point lists.

Network setup (Amiga brain — camphor-clone):
    VLP-16 IP:      192.168.1.201  (fixed — set in VLP-16 web UI)
    Amiga host IP:  192.168.1.100  (secondary alias on eth0, added by
                    vidalia-lidar-network.service / setup_lidar_network.sh)
    Receive port:   2368           (data port, UDP)
    Bind address:   "" (0.0.0.0)  (receive from all interfaces)
    VLP-16 dest IP: 192.168.1.100  (configured in VLP-16 web UI → Host addr)

    The Amiga eth0 primary IP is 10.95.76.1/24 (set by farmng_network_config.sh).
    The 192.168.1.100/24 alias is added automatically at boot by the
    vidalia-lidar-network systemd service.  To add it manually:
        sudo bash scripts/setup_lidar_network.sh

Coordinate frame (robot-centric, right-hand rule):
    X  — right
    Y  — forward (direction of travel)
    Z  — up

The LiDAR (a VLP-16 Puck Hi-Res — see VLP16_VERTICAL_ANGLES) is mounted
forward-facing (yaw 0), ~15° nose-down, at z ≈ 0.80 m above ground (2026-07
re-mount; forward x offset re-measured).  Earlier side-yawed mount used
z=0.699 m / x=0.959 m.

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

import numpy as np

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

# Vertical elevation angles per channel (degrees), channels 0-15, firing order.
#
# THIS UNIT IS A VLP-16 PUCK HI-RES, confirmed 2026-07 from the packet product-ID
# byte (0x24) via scripts/diag_sensor_model.py.  The Hi-Res packs its 16 beams
# into a ±10° fan (1.33° spacing) instead of the standard VLP-16's ±15° (2.00°).
# The standard table was in use here originally; with the wrong (wider) angles
# the driver stretched the vertical geometry, so a true 15° nose-down / 0.80 m
# mount reconstructed as ~22° / 1.17 m — which sent the whole tilt/height
# calibration chasing phantom values.  Field tape (0.80 m) + phone level (~15°)
# were correct; the driver table was not.  Hi-Res angles = standard × (2/3).
#
# Standard VLP-16 (kept for reference / other units):
VLP16_VERTICAL_ANGLES_STD = (
    -15.0,  1.0, -13.0,  3.0, -11.0,  5.0,  -9.0,  7.0,
     -7.0,  9.0,  -5.0, 11.0,  -3.0, 13.0,  -1.0, 15.0,
)
# VLP-16 Puck Hi-Res (this unit):
VLP16_VERTICAL_ANGLES_HIRES = (
    -10.00,  0.67,
     -8.67,  2.00,
     -7.33,  3.33,
     -6.00,  4.67,
     -4.67,  6.00,
     -3.33,  7.33,
     -2.00,  8.67,
     -0.67, 10.00,
)
VLP16_VERTICAL_ANGLES = VLP16_VERTICAL_ANGLES_HIRES

# Precomputed trig for the 32 channel slots in a data block (two 16-channel
# firing sequences).  Used by the vectorised parser _parse_scan_np().
_VLP16_VERT_RAD_32 = np.radians(
    np.array(VLP16_VERTICAL_ANGLES + VLP16_VERTICAL_ANGLES, dtype=np.float64)
)
_VLP16_COS_VERT_32 = np.cos(_VLP16_VERT_RAD_32)
_VLP16_SIN_VERT_32 = np.sin(_VLP16_VERT_RAD_32)


@dataclass()
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


def _parse_scan_np(packets: List[bytes]) -> np.ndarray:
    """
    Vectorised parse of a list of raw VLP-16 packets into an Nx3 float64
    array of (x, y, z) sensor-frame coordinates.

    Produces the same points as running _parse_packet() on each packet, but
    ~50x faster: all byte unpacking and trig run in numpy instead of a
    per-point Python loop.  scan_stream_np() relies on this to sustain the
    full 10 Hz scan rate the real-time perception pipeline needs — the
    per-point parser cannot keep up and causes dropped UDP packets.
    """
    if not packets:
        return np.zeros((0, 3), dtype=np.float64)

    body = VELODYNE_BLOCK_COUNT * VELODYNE_BLOCK_SIZE
    raw = b"".join(p[:body] for p in packets)
    blocks = np.frombuffer(raw, dtype=np.uint8).reshape(-1, VELODYNE_BLOCK_SIZE)

    flag = blocks[:, 0].astype(np.uint16) | (blocks[:, 1].astype(np.uint16) << 8)
    az_raw = blocks[:, 2].astype(np.uint16) | (blocks[:, 3].astype(np.uint16) << 8)
    azimuth_rad = np.radians(az_raw.astype(np.float64) * VELODYNE_ROTATION_UNIT)

    data = blocks[:, 4:VELODYNE_BLOCK_SIZE].reshape(-1, 32, 3)
    dist_raw = (data[:, :, 0].astype(np.uint32)
                | (data[:, :, 1].astype(np.uint32) << 8))
    dist = dist_raw.astype(np.float64) * VELODYNE_DISTANCE_UNIT

    planar = dist * _VLP16_COS_VERT_32[None, :]
    x = planar * np.sin(azimuth_rad)[:, None]
    y = planar * np.cos(azimuth_rad)[:, None]
    z = dist * _VLP16_SIN_VERT_32[None, :]

    good = (dist_raw != 0) & (flag[:, None] == VELODYNE_BLOCK_FLAG)
    return np.stack((x[good], y[good], z[good]), axis=1)


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
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        # Large receive buffer: the VLP-16 streams ~0.9 MB/s and the
        # consumer processes scans synchronously, so packets must be
        # buffered by the kernel during each processing burst.  Without
        # this the default buffer overflows and scans arrive fragmented
        # (missing azimuth sectors, wildly varying point counts).
        try:
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        except OSError:
            pass
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

    async def scan_stream_np(self) -> "AsyncIterator[np.ndarray]":
        """
        Yield one full 360° scan per iteration as an Nx3 float64 array of
        (x, y, z) sensor-frame coordinates.

        Vectorised equivalent of scan_stream().  Preferred for real-time
        perception: it sustains the full 10 Hz scan rate, whereas the
        per-point scan_stream() falls behind and drops UDP packets, which
        fragments scans (missing azimuth sectors, varying point counts).
        """
        packets: List[bytes] = []
        last_azimuth = 0.0

        while True:
            raw = await self._recv_packet()
            if len(raw) < VELODYNE_PACKET_SIZE:
                continue

            first_az = struct.unpack_from("<H", raw, 2)[0] * VELODYNE_ROTATION_UNIT

            if first_az < last_azimuth and packets:
                yield _parse_scan_np(packets)
                packets = []

            last_azimuth = first_az
            packets.append(raw)
