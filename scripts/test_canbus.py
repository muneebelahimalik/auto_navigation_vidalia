#!/usr/bin/env python3
"""
test_canbus.py — Step 3: Verify connectivity to the Amiga canbus and filter services.

Run after test_sdk.py passes.

Usage:
    source /farm_ng_image/venv/bin/activate
    python3 scripts/test_canbus.py              # uses service_config.json
    python3 scripts/test_canbus.py --scan       # scan common ports for active services
    python3 scripts/test_canbus.py --timeout 10 # wait up to 10 s for first message

What this tests:
    1. Tailscale / network reachability of camphor-clone
    2. TCP port reachability on canbus (:6001) and filter (:20001)
    3. Actual gRPC message reception from the canbus service (AmigaTpdo1)
    4. Actual gRPC message reception from the filter service (FilterState)

Expected results:
    - Ports reachable:   the gRPC services are running on the brain
    - First message:     the EventClient subscribe loop is working
    - AmigaTpdo1 decode: farm_ng SDK version is correct for the brain OS
"""

from __future__ import annotations

import argparse
import asyncio
import json
import socket
import sys
from pathlib import Path

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
# Config loading
# ---------------------------------------------------------------------------

CONFIG_PATH = Path(__file__).parent.parent / "service_config.json"


def load_configs():
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList
    except ImportError:
        print("ERROR: farm-ng SDK not found.  Run:  source /farm_ng_image/venv/bin/activate")
        sys.exit(1)

    if not CONFIG_PATH.exists():
        print(f"ERROR: {CONFIG_PATH} not found — run from workspace root")
        sys.exit(1)

    with open(CONFIG_PATH) as f:
        raw = json.load(f)
    cfg_list = json_format.ParseDict(raw, EventServiceConfigList())
    return {c.name: c for c in cfg_list.configs}


# ---------------------------------------------------------------------------
# Network checks
# ---------------------------------------------------------------------------

def check_tcp_port(host: str, port: int, timeout: float = 3.0) -> bool:
    """Return True if a TCP connection to host:port succeeds within timeout."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except (OSError, ConnectionRefusedError, TimeoutError):
        return False


def check_ping(host: str) -> bool:
    """Return True if host responds to a single ICMP ping."""
    import subprocess
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", host],
            capture_output=True,
            timeout=5,
        )
        return result.returncode == 0
    except Exception:
        return False


# ---------------------------------------------------------------------------
# gRPC reception test (async)
# ---------------------------------------------------------------------------

async def receive_one_message(config, timeout: float):
    """
    Subscribe to the service and wait for the first message.
    Returns (event, payload/message) or raises on timeout/error.
    """
    from farm_ng.core.event_client import EventClient

    client = EventClient(config)
    sub = config.subscriptions[0]

    # Try decode=True first (works for filter), fall back to decode=False (canbus)
    try:
        gen = client.subscribe(sub, decode=True)
        event, message = await asyncio.wait_for(gen.__anext__(), timeout=timeout)
        return event, message, True
    except Exception:
        gen = client.subscribe(sub, decode=False)
        event, payload = await asyncio.wait_for(gen.__anext__(), timeout=timeout)
        return event, payload, False


async def test_service(name: str, config, timeout: float) -> bool:
    """Test one service: port check + message reception."""
    host = config.host
    port = config.port

    # TCP port
    if check_tcp_port(host, port):
        ok(f"{name}: TCP port {host}:{port} reachable")
    else:
        fail(
            f"{name}: TCP port {host}:{port} NOT reachable",
            f"Check: Tailscale connected?  Is the {name} service running on the brain?",
        )
        return False

    # Message reception
    info(f"{name}: waiting up to {timeout:.0f} s for first message …")
    try:
        event, msg, decoded = await receive_one_message(config, timeout)
        ok(f"{name}: received first message (decoded={decoded})")

        # Extra canbus-specific decode
        if name == "canbus":
            try:
                from farm_ng.canbus.packet import AmigaTpdo1
                # Try to find payload_to_protobuf
                _p2p = None
                try:
                    from farm_ng.core.events_file_reader import payload_to_protobuf as p
                    _p2p = p
                except ImportError:
                    pass
                if _p2p is None:
                    try:
                        from farm_ng.core.event_client import payload_to_protobuf as p
                        _p2p = p
                    except ImportError:
                        pass

                if _p2p is not None and not decoded:
                    message = _p2p(event, msg)
                    tpdo1 = AmigaTpdo1.from_proto(message.amiga_tpdo1)
                elif decoded:
                    tpdo1 = AmigaTpdo1.from_proto(msg.amiga_tpdo1)
                else:
                    tpdo1 = AmigaTpdo1.from_proto(msg)

                ok(
                    f"  AmigaTpdo1 decoded: "
                    f"speed={tpdo1.meas_speed:.3f} m/s, "
                    f"ang_rate={tpdo1.meas_ang_rate:.3f} rad/s"
                )
            except Exception as e:
                info(f"  AmigaTpdo1 decode attempted but failed: {e}")
                info("  Message was still received — decode path may need tuning")

        # Extra filter-specific info
        if name == "filter" and decoded:
            try:
                converged = getattr(msg, "has_converged", "N/A")
                heading = getattr(msg, "heading", "N/A")
                ok(f"  FilterState: has_converged={converged}, heading={heading}")
            except Exception:
                pass

        return True

    except asyncio.TimeoutError:
        fail(
            f"{name}: no message received within {timeout:.0f} s",
            f"Port was reachable but no data arrived.\n"
            f"         Check: Is the {name} service running on the brain?\n"
            f"             ps aux | grep {name}",
        )
        return False
    except Exception as e:
        fail(f"{name}: error receiving message: {e!r}")
        return False


# ---------------------------------------------------------------------------
# Port scanner (--scan mode)
# ---------------------------------------------------------------------------

COMMON_PORTS = [
    (6001, "canbus"),
    (6002, "canbus-alt"),
    (20001, "filter"),
    (20002, "filter-alt"),
    (8042, "foxglove"),
    (50051, "grpc-default"),
]


def port_scan(host: str):
    sep(f"Port scan: {host}")
    found = []
    for port, label in COMMON_PORTS:
        if check_tcp_port(host, port, timeout=1.5):
            ok(f"  Port {port} ({label}) OPEN")
            found.append(port)
        else:
            info(f"  Port {port} ({label}) closed/filtered")
    if not found:
        fail("No known gRPC ports found open on the brain")
    return found


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

async def async_main(args):
    configs = load_configs()
    host = configs.get("canbus", configs.get("filter")).host

    print("=" * 52)
    print(" Amiga service connectivity test")
    print("=" * 52)

    # ---- Ping ----
    sep("1. Network reachability")
    if check_ping(host):
        ok(f"Ping to {host} successful")
    else:
        fail(
            f"Cannot ping {host}",
            "Check: tailscale status   |   sudo tailscale up",
        )

    # ---- Port scan (optional) ----
    if args.scan:
        port_scan(host)

    # ---- Service tests ----
    for name in ("canbus", "filter"):
        sep(f"{'2' if name == 'canbus' else '3'}. {name} service")
        if name in configs:
            await test_service(name, configs[name], args.timeout)
        else:
            info(f"{name} not in service_config.json — skipping")

    # ---- Summary ----
    print("")
    print("=" * 52)
    print(f" Results: {PASS} passed, {FAIL} failed")
    print("=" * 52)
    if FAIL == 0:
        print(" All service checks passed.")
        print(" Next step:  python3 scripts/test_lidar.py")
    else:
        print(" Fix the FAIL items above, then re-run.")
        print("")
        print(" Common fixes:")
        print("   • Tailscale down:       sudo tailscale up")
        print("   • Wrong port in config: cat /opt/farmng/config.json   (on brain)")
        print("   • Service not running:  ps aux | grep -E 'canbus|filter'  (on brain)")
    print("")


def main():
    parser = argparse.ArgumentParser(description="Test Amiga gRPC service connectivity")
    parser.add_argument(
        "--timeout", type=float, default=8.0,
        help="Seconds to wait for first message from each service (default: 8)",
    )
    parser.add_argument(
        "--scan", action="store_true",
        help="Also scan common gRPC ports on the brain",
    )
    args = parser.parse_args()
    asyncio.run(async_main(args))


if __name__ == "__main__":
    main()
