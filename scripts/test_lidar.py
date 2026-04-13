#!/usr/bin/env python3
"""
test_lidar.py — Step 4: Verify Velodyne VLP-16 LiDAR connectivity and data.

Run after test_canbus.py passes.

Usage:
    python3 scripts/test_lidar.py              # listen for 5 s
    python3 scripts/test_lidar.py --duration 10
    python3 scripts/test_lidar.py --host 192.168.1.201 --port 2368

What this tests:
    1. Network reachability of the VLP-16 at 192.168.1.201
    2. UDP packet reception on port 2368
    3. Packet size and structure (valid 1206-byte firing packets)
    4. Point cloud parsing — correct 3D point output from LidarDriver
    5. Scan rate (~10 Hz expected from VLP-16)
    6. Range statistics (min/max/mean) to confirm real scene data

Expected results:
    - Packets received:   VLP-16 is powered and connected to this network interface
    - Valid packets:      UDP data has the 0xEEFF block flag (genuine VLP-16)
    - Scans decoded:      LidarDriver.scan_stream() produces non-empty point lists
    - Range ~ 0.5–100 m: realistic scene data (not all zeros)
"""

from __future__ import annotations

import argparse
import asyncio
import math
import socket
import struct
import sys
import time
from pathlib import Path

# Add repo root to path so lidar/ module is importable
sys.path.insert(0, str(Path(__file__).parent.parent))

VELODYNE_DATA_PORT = 2368
VELODYNE_HOST = "192.168.1.201"
VELODYNE_PACKET_SIZE = 1206
VELODYNE_BLOCK_FLAG = 0xEEFF
VELODYNE_ROTATION_UNIT = 0.01

PASS = 0
FAIL = 0


def ok(msg):
    global PASS
    print(f"  [OK]   {msg}")
    PASS += 1


def fail(msg, hint=""):
    global FAIL
    print(f"  [FAIL] {msg}")
    if hint:
        print(f"         {hint}")
    FAIL += 1


def info(msg):
    print(f"  [INFO] {msg}")


def sep(title):
    print(f"\n--- {title} ---")


# ---------------------------------------------------------------------------
# Network checks (no farm-ng SDK needed for LiDAR)
# ---------------------------------------------------------------------------

def check_ping(host: str) -> bool:
    import subprocess
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", host],
            capture_output=True, timeout=5,
        )
        return result.returncode == 0
    except Exception:
        return False


def check_interface_for_subnet(subnet_prefix: str) -> list:
    """Return list of local IPs that are in the given subnet prefix (e.g. '192.168.1.')."""
    import subprocess
    try:
        out = subprocess.check_output(["ip", "-4", "addr", "show"], text=True)
        addrs = []
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                addr = line.split()[1].split("/")[0]
                if addr.startswith(subnet_prefix):
                    addrs.append(addr)
        return addrs
    except Exception:
        return []


# ---------------------------------------------------------------------------
# Raw UDP test (no LidarDriver)
# ---------------------------------------------------------------------------

def recv_udp_packets_sync(port: int, count: int, timeout: float):
    """Receive up to `count` UDP packets, return list of raw bytes."""
    packets = []
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.bind(("", port))
        deadline = time.monotonic() + timeout
        while len(packets) < count and time.monotonic() < deadline:
            try:
                data = sock.recv(2048)
                packets.append(data)
            except socket.timeout:
                break
    finally:
        sock.close()
    return packets


# ---------------------------------------------------------------------------
# LidarDriver integration test (async)
# ---------------------------------------------------------------------------

async def test_lidar_driver(duration: float, host: str, port: int):
    """Run LidarDriver.scan_stream() for `duration` seconds and collect stats."""
    from lidar.lidar_driver import LidarDriver

    scan_count = 0
    total_points = 0
    all_ranges = []

    deadline = asyncio.get_event_loop().time() + duration
    async with LidarDriver(host=host, port=port) as driver:
        async for scan in driver.scan_stream():
            if asyncio.get_event_loop().time() >= deadline:
                break
            scan_count += 1
            total_points += len(scan)
            for p in scan:
                r = math.hypot(p.x, p.y, p.z)
                if r > 0:
                    all_ranges.append(r)

    return scan_count, total_points, all_ranges


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def async_main(args):
    print("=" * 52)
    print(" Velodyne VLP-16 LiDAR test")
    print(f" Target: {args.host}:{args.port}  duration: {args.duration} s")
    print("=" * 52)

    # ---- 1. Local network interface ----
    sep("1. Local network interface")
    local_ips = check_interface_for_subnet("192.168.1.")
    if local_ips:
        ok(f"Local interface on 192.168.1.x subnet: {local_ips}")
    else:
        fail(
            "No local interface found on 192.168.1.x subnet",
            "The VLP-16 is at 192.168.1.201 — your host needs an IP in this subnet.\n"
            "         Check:  ip addr show\n"
            "         Fix:    sudo ip addr add 192.168.1.100/24 dev <interface>",
        )

    # ---- 2. Ping the VLP-16 ----
    sep("2. VLP-16 reachability")
    info(f"Pinging {args.host} …")
    if check_ping(args.host):
        ok(f"Ping to {args.host} successful")
    else:
        fail(
            f"Cannot ping {args.host}",
            "Check:\n"
            "  • VLP-16 powered on?\n"
            "  • Ethernet cable connected?\n"
            "  • Local IP in 192.168.1.x subnet?\n"
            "  • VLP-16 factory IP is 192.168.1.201 (verify with VLP-16 web UI)",
        )

    # ---- 3. Raw UDP packets ----
    sep("3. Raw UDP packet reception (port 2368)")
    info(f"Listening on UDP :{args.port} for up to 3 s …")
    packets = recv_udp_packets_sync(args.port, count=20, timeout=3.0)

    if not packets:
        fail(
            f"No UDP packets received on port {args.port}",
            "VLP-16 is not sending data.  Check:\n"
            "  • Is the correct network interface connected?\n"
            "  • Is another process already bound to port 2368? (lsof -i udp:2368)\n"
            "  • VLP-16 data IP set to broadcast or this host's IP?",
        )
        print("")
        print("=" * 52)
        print(f" Results: {PASS} passed, {FAIL} failed")
        print("=" * 52)
        print(" Cannot proceed — fix UDP reception first.")
        print("")
        return

    ok(f"Received {len(packets)} UDP packets")

    # ---- 4. Packet structure ----
    sep("4. Packet structure validation")
    valid = 0
    sizes = set()
    for p in packets:
        sizes.add(len(p))
        if len(p) >= 4:
            flag = struct.unpack_from("<H", p, 0)[0]
            if flag == VELODYNE_BLOCK_FLAG:
                valid += 1

    if VELODYNE_PACKET_SIZE in sizes:
        ok(f"Packet size {VELODYNE_PACKET_SIZE} bytes confirmed (VLP-16 data format)")
    else:
        fail(f"Unexpected packet sizes: {sizes}  (expected {VELODYNE_PACKET_SIZE})")

    if valid > 0:
        ok(f"{valid}/{len(packets)} packets have valid 0xEEFF block flag (genuine VLP-16)")
    else:
        fail(
            "No packets have the VLP-16 0xEEFF block flag",
            "Data may be from a different device or port",
        )

    # ---- 5. LidarDriver scan stream ----
    sep(f"5. LidarDriver scan stream ({args.duration:.0f} s test)")
    info("Importing LidarDriver …")
    try:
        from lidar.lidar_driver import LidarDriver
        ok("lidar.lidar_driver.LidarDriver imported")
    except ImportError as e:
        fail(f"Cannot import LidarDriver: {e}", "Run from the workspace root: python3 scripts/test_lidar.py")
        return

    info(f"Running scan_stream() for {args.duration:.0f} s …")
    try:
        t0 = time.monotonic()
        scan_count, total_points, all_ranges = await test_lidar_driver(
            args.duration, args.host, args.port
        )
        elapsed = time.monotonic() - t0

        if scan_count == 0:
            fail("No complete 360° scans received")
        else:
            hz = scan_count / elapsed
            ok(f"Received {scan_count} complete scans in {elapsed:.1f} s  ({hz:.1f} Hz)")
            if 8 <= hz <= 12:
                ok(f"Scan rate {hz:.1f} Hz is within expected VLP-16 range (10 Hz ±2)")
            else:
                info(f"Scan rate {hz:.1f} Hz — expected ~10 Hz (check VLP-16 RPM setting)")

        if total_points > 0:
            ok(f"Total points decoded: {total_points:,}  (~{total_points//max(scan_count,1):,} per scan)")
        else:
            fail("Zero points decoded across all scans")

    except Exception as e:
        fail(f"LidarDriver error: {e!r}")
        return

    # ---- 6. Range statistics ----
    sep("6. Range statistics")
    if all_ranges:
        min_r = min(all_ranges)
        max_r = max(all_ranges)
        mean_r = sum(all_ranges) / len(all_ranges)
        ok(f"Range stats — min: {min_r:.2f} m  mean: {mean_r:.2f} m  max: {max_r:.2f} m")
        if min_r < 0.1:
            info("Very short returns (<10 cm) detected — possible internal reflections")
        if max_r > 0.5 and mean_r > 0.3:
            ok("Real scene data confirmed (non-trivial ranges)")
        else:
            info("Short ranges only — sensor may be indoors or pointing at a wall")
    else:
        fail("No range data to analyse")

    # ---- Summary ----
    print("")
    print("=" * 52)
    print(f" Results: {PASS} passed, {FAIL} failed")
    print("=" * 52)
    if FAIL == 0:
        print(" All LiDAR checks passed.")
        print(" Next step:  python3 main.py")
    else:
        print(" Fix the FAIL items above, then re-run.")
    print("")


def main():
    parser = argparse.ArgumentParser(description="Test Velodyne VLP-16 LiDAR")
    parser.add_argument(
        "--host", default=VELODYNE_HOST,
        help=f"VLP-16 IP address (default: {VELODYNE_HOST})",
    )
    parser.add_argument(
        "--port", type=int, default=VELODYNE_DATA_PORT,
        help=f"VLP-16 UDP data port (default: {VELODYNE_DATA_PORT})",
    )
    parser.add_argument(
        "--duration", type=float, default=5.0,
        help="Seconds to run the scan stream test (default: 5)",
    )
    args = parser.parse_args()
    asyncio.run(async_main(args))


if __name__ == "__main__":
    main()
