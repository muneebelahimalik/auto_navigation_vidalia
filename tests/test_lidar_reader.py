"""LiDAR reader thread: a slow consumer must get whole scans, never half-scans.

Field 2026-11-06: the perception loop took ~250 ms per scan, the kernel UDP
buffer (~0.2 s of VLP-16 data) overflowed, and 4 of 5 saved scans had lost the
whole left or right half of the forward view.  A fake VLP-16 here streams
real-format packets at 600 rpm to a consumer that is just as slow.
"""
import asyncio
import socket
import struct
import threading
import time

import numpy as np

from lidar.lidar_driver import (LidarDriver, VELODYNE_BLOCK_COUNT,
                                VELODYNE_PACKET_SIZE)

PKTS_PER_REV = 75          # 600 rpm: 754 packets/s


def _packet(az0_cdeg: int) -> bytes:
    """One firing packet: 12 blocks, every return at 5 m, azimuth az0 + 0.4°·k."""
    out = bytearray()
    for k in range(VELODYNE_BLOCK_COUNT):
        az = (az0_cdeg + 40 * k) % 36000
        out += b"\xff\xee" + struct.pack("<H", az) + (struct.pack("<H", 2500) + b"\x10") * 32
    out += b"\x00" * (VELODYNE_PACKET_SIZE - len(out))
    return bytes(out)


def _sender(port, stop, revs_per_s=10.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    step = 36000 // PKTS_PER_REV + 1        # 0.48° per packet → 75 packets/rev
    pkts = [_packet(i * step) for i in range(PKTS_PER_REV)]
    period = 1.0 / (revs_per_s * PKTS_PER_REV)
    t = time.perf_counter()
    i = 0
    while not stop.is_set():
        s.sendto(pkts[i % PKTS_PER_REV], ("127.0.0.1", port))
        i += 1
        t += period
        dt = t - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
    s.close()


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _azimuth_coverage(pts):
    az = (np.degrees(np.arctan2(pts[:, 0], pts[:, 1])) + 360.0) % 360.0
    occ = np.bincount((az // 10).astype(int), minlength=36) > 0
    return occ.mean()


def _run(reader, busy_s=0.25, n_scans=8, small_buffer=True):
    port = _free_port()
    stop = threading.Event()
    infos, cover = [], []

    async def main():
        async with LidarDriver(port=port, reader=reader) as drv:
            if small_buffer:     # the Brain's effective buffer is only ~0.2 s
                drv._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 64 * 1024)
            th = threading.Thread(target=_sender, args=(port, stop), daemon=True)
            th.start()
            try:
                async for pts in drv.scan_stream_np():
                    infos.append(dict(drv.last_scan_info))
                    cover.append(_azimuth_coverage(pts))
                    t_end = time.perf_counter() + busy_s     # slow perception loop
                    while time.perf_counter() < t_end:        # (holds the GIL)
                        sum(range(2000))
                    if len(infos) >= n_scans:
                        break
            finally:
                stop.set()
                th.join()

    asyncio.run(main())
    return infos[2:], cover[2:]          # skip start-up


def test_thread_reader_gives_whole_scans_to_a_slow_consumer():
    import sys
    before = sys.getswitchinterval()
    infos, cover = _run("thread")
    assert sys.getswitchinterval() == before            # restored on exit
    assert all(c == 1.0 for c in cover), cover          # every 10° slice present
    assert all(not i["partial"] for i in infos), infos
    assert all(PKTS_PER_REV - 2 <= i["packets"] <= PKTS_PER_REV + 2 for i in infos), infos
    assert infos[-1]["skipped"] > 0                     # whole scans were skipped instead
    assert all(i["age_ms"] < 150 for i in infos), infos  # and the data is fresh


def test_inline_reader_shows_the_field_failure():
    """The original path with the same slow consumer and small buffer loses
    packets: scans come out partial — the 2026-11-06 half-scans."""
    infos, cover = _run("inline")
    assert min(cover) < 0.9 or any(i["partial"] for i in infos), (cover, infos)
