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
import sys
import threading
import time
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


def _parse_scan_np(packets: List[bytes], with_intensity: bool = False) -> np.ndarray:
    """
    Vectorised parse of a list of raw VLP-16 packets into an Nx3 float64
    array of (x, y, z) sensor-frame coordinates.

    Produces the same points as running _parse_packet() on each packet, but
    ~50x faster: all byte unpacking and trig run in numpy instead of a
    per-point Python loop.  scan_stream_np() relies on this to sustain the
    full 10 Hz scan rate the real-time perception pipeline needs — the
    per-point parser cannot keep up and causes dropped UDP packets.

    ``with_intensity=True`` returns Nx4 with the per-return intensity (0–255,
    the third byte of each channel record — already parsed here as
    ``data[:, :, 2]``, previously discarded) as column 3.  In the 903 nm NIR
    band green canopy and dry residue/soil reflect differently, so intensity is
    a candidate discriminator for the residue strip once the canopy has closed
    and the height signal is a plateau.  Default (Nx3) is byte-identical to
    before, so the live geometry path is unaffected.
    """
    ncols = 4 if with_intensity else 3
    if not packets:
        return np.zeros((0, ncols), dtype=np.float64)

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
    if with_intensity:
        intensity = data[:, :, 2].astype(np.float64)
        return np.stack((x[good], y[good], z[good], intensity[good]), axis=1)
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
        reader: str = "inline",
    ) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        # "inline": packets are read by the consumer between scans (original).
        # "thread": a background thread reads every packet and keeps only the
        # newest COMPLETE scan (see _ScanReader) — a slow consumer then skips
        # whole scans instead of getting half-scans from kernel packet drops.
        if reader not in ("inline", "thread"):
            raise ValueError(f"reader must be 'inline' or 'thread', not {reader!r}")
        self.reader_mode = reader
        self._reader: "_ScanReader | None" = None
        self.rcvbuf_bytes = 0
        # Health of the scan just yielded by scan_stream_np (both modes):
        # packets in it, ms since it completed, scans skipped so far (thread),
        # and whether it looked partial (fewer packets than a full revolution).
        self.last_scan_info: dict = {}

    async def __aenter__(self) -> "LidarDriver":
        self._loop = asyncio.get_running_loop()
        # SO_REUSEPORT below lets a SECOND program bind the port, but the kernel
        # then steers the whole VLP-16 flow to one socket (usually the older):
        # warn loudly if an earlier run is still alive (see lidar/port_check.py).
        try:
            from lidar.port_check import report_port_holders
            report_port_holders(self._port)
        except Exception:
            pass
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
        try:
            # The kernel silently caps SO_RCVBUF at net.core.rmem_max (often
            # ~208 KB ≈ 0.2 s of VLP-16 data); record what we really got.
            self.rcvbuf_bytes = int(self._sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
        except OSError:
            self.rcvbuf_bytes = 0
        return self

    async def __aexit__(self, *_) -> None:
        if self._reader is not None:
            self._reader.stop()
            self._reader = None
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

    async def scan_stream_np(self, with_intensity: bool = False) -> "AsyncIterator[np.ndarray]":
        """
        Yield one full 360° scan per iteration as an Nx3 float64 array of
        (x, y, z) sensor-frame coordinates.

        Vectorised equivalent of scan_stream().  Preferred for real-time
        perception: it sustains the full 10 Hz scan rate, whereas the
        per-point scan_stream() falls behind and drops UDP packets, which
        fragments scans (missing azimuth sectors, varying point counts).

        ``with_intensity=True`` yields Nx4 (x, y, z, intensity) — used only to
        CAPTURE intensity into the recorded scan stream; the live geometry path
        keeps the default Nx3.
        """
        if self.reader_mode == "thread":
            async for pts in self._scan_stream_thread(with_intensity):
                yield pts
            return

        packets: List[bytes] = []
        last_azimuth = 0.0
        full = _FullScanTracker()

        while True:
            raw = await self._recv_packet()
            if len(raw) < VELODYNE_PACKET_SIZE:
                continue

            first_az = struct.unpack_from("<H", raw, 2)[0] * VELODYNE_ROTATION_UNIT

            if first_az < last_azimuth and packets:
                self.last_scan_info = {"packets": len(packets), "age_ms": 0.0,
                                       "skipped": None,
                                       "partial": full.partial(len(packets))}
                yield _parse_scan_np(packets, with_intensity=with_intensity)
                packets = []

            last_azimuth = first_az
            packets.append(raw)

    async def _scan_stream_thread(self, with_intensity: bool) -> "AsyncIterator[np.ndarray]":
        """scan_stream_np() in "thread" mode: the background reader assembles
        every revolution; here we wait for the newest complete one."""
        assert self._sock is not None and self._loop is not None
        if self._reader is None:
            self._reader = _ScanReader(self._sock, self._loop)
            self._reader.start()
        reader = self._reader
        while True:
            await reader.event.wait()
            reader.event.clear()
            item = reader.take()
            if item is None:
                continue
            packets, t_done, partial = item
            self.last_scan_info = {
                "packets": len(packets),
                "age_ms": round((time.monotonic() - t_done) * 1000.0, 1),
                "skipped": reader.skipped,
                "partial": partial,
            }
            yield _parse_scan_np(packets, with_intensity=with_intensity)


class _FullScanTracker:
    """Packets in a FULL revolution (75–76 at 600 rpm; scales with rpm),
    learnt as the largest count seen recently.  A scan with < 90 % of it is
    partial — packets were lost (kernel buffer overflow) or it is the first
    revolution after the reader started mid-turn."""

    def __init__(self) -> None:
        self._recent: List[int] = []

    def partial(self, n: int) -> bool:
        self._recent = (self._recent + [n])[-50:]
        full = max(self._recent)
        return len(self._recent) >= 3 and n < 0.9 * full


class _ScanReader(threading.Thread):
    """Reads EVERY VLP-16 packet in a background thread and keeps only the
    newest COMPLETE revolution (field 2026-11-06: with packets read inline, a
    ~250 ms perception loop let the kernel buffer (~0.2 s) overflow, so 4 of 5
    scans lost a whole left or right half of the forward view).  socket.recv
    releases the GIL, so this keeps up even while the main thread computes."""

    def __init__(self, sock: socket.socket, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__(daemon=True, name="lidar-reader")
        self._sock = sock
        self._loop = loop
        self.event = asyncio.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._stop_flag = False
        self.received = 0        # complete revolutions assembled
        self.skipped = 0         # revolutions replaced before the consumer took them
        self.partial = 0         # revolutions with missing packets
        self._full = _FullScanTracker()

    # Python runs one thread at a time and, by default, hands over only every
    # 5 ms — too rarely for ~750 packets/s while perception keeps Python busy
    # (test: 5 ms → 7 of 8 scans partial; 1 ms → none).  0.5 ms while the
    # reader runs; restored on stop.
    SWITCH_INTERVAL_S = 0.0005

    def start(self) -> None:
        self._prev_switch = sys.getswitchinterval()
        sys.setswitchinterval(min(self._prev_switch, self.SWITCH_INTERVAL_S))
        super().start()

    def run(self) -> None:
        self._sock.settimeout(0.5)          # blocking with timeout (thread-owned now)
        packets: List[bytes] = []
        last_az = 0.0
        first = True
        while not self._stop_flag:
            try:
                raw = self._sock.recv(VELODYNE_PACKET_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break                        # socket closed
            if len(raw) < VELODYNE_PACKET_SIZE:
                continue
            az = struct.unpack_from("<H", raw, 2)[0] * VELODYNE_ROTATION_UNIT
            if az < last_az and packets:
                if first:                    # started mid-revolution: discard
                    first = False
                else:
                    self._publish(packets)
                packets = []
            last_az = az
            packets.append(raw)

    def _publish(self, packets: List[bytes]) -> None:
        partial = self._full.partial(len(packets))
        with self._lock:
            if self._latest is not None:
                self.skipped += 1
            self._latest = (packets, time.monotonic(), partial)
            self.received += 1
            self.partial += int(partial)
        try:
            self._loop.call_soon_threadsafe(self.event.set)
        except RuntimeError:                 # event loop closed
            self._stop_flag = True

    def take(self):
        with self._lock:
            item, self._latest = self._latest, None
        return item

    def stop(self) -> None:
        self._stop_flag = True
        if self.is_alive():
            self.join(timeout=1.0)
        prev = getattr(self, "_prev_switch", None)
        if prev is not None:
            sys.setswitchinterval(prev)
            self._prev_switch = None
