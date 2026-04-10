#!/usr/bin/env python3
"""
test_canbus.py — standalone diagnostic for the Amiga canbus gRPC service.

Mirrors the filter_client.py pattern the user already confirmed works,
but targets the canbus service.  Run this OUTSIDE ROS 2 to verify the
canbus port and subscription path before using the bridge node.

Usage:
    python3 scripts/test_canbus.py
    python3 scripts/test_canbus.py --config config/service_configs/canbus_config.json
    python3 scripts/test_canbus.py --host camphor-clone.tail0be07.ts.net --port 6001

If the correct port/path is unknown, use --scan to probe common ports:
    python3 scripts/test_canbus.py --scan
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path


def _check_sdk():
    try:
        from farm_ng.core.event_client import EventClient       # noqa: F401
        from farm_ng.core.event_service_pb2 import EventServiceConfig  # noqa: F401
        from google.protobuf import json_format                  # noqa: F401
        return True
    except ImportError as e:
        print(f"[ERROR] farm-ng SDK not found: {e}")
        print("Install with:  pip3 install farm-ng-amiga farm-ng-core")
        return False


async def probe_canbus(host: str, port: int, path: str, query: str,
                       timeout: float = 10.0, n_msgs: int = 5) -> bool:
    """
    Attempt to subscribe to path/query on host:port.
    Returns True if at least one message arrives within timeout seconds.
    """
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    from google.protobuf import json_format

    config = json_format.ParseDict(
        {
            'name': 'canbus',
            'host': host,
            'port': port,
            'subscriptions': [
                {'uri': {'path': path, 'query': query}, 'every_n': 1}
            ],
        },
        EventServiceConfig(),
    )
    sub_req = SubscribeRequest(
        uri=Uri(path=path, query=query),
        every_n=1,
    )

    print(f"  Probing  {host}:{port}  path={path!r}  query={query!r} …", end=' ', flush=True)
    count = 0
    try:
        client = EventClient(config)
        async with asyncio.timeout(timeout):
            async for event, payload in client.subscribe(sub_req, decode=False):
                if count == 0:
                    print(f"\n  [OK] First message received!")
                    print(f"       event.uri.path  = {event.uri.path!r}")
                    print(f"       event.uri.query = {event.uri.query!r}")
                    print(f"       payload bytes   = {len(payload)}")
                count += 1
                if count >= n_msgs:
                    break
        if count > 0:
            print(f"  Received {count} messages. SUCCESS.")
            return True
        else:
            print("  No messages received (stream opened but empty).")
            return False
    except TimeoutError:
        if count > 0:
            print(f"  {count} messages then timeout — partial success.")
            return True
        print(f"  Timeout after {timeout:.0f}s — no messages.")
        return False
    except Exception as e:
        print(f"\n  Error: {e!r}")
        return False


async def decode_and_print(host: str, port: int, path: str, query: str, n_msgs: int = 20):
    """Subscribe and attempt to decode AmigaTpdo1 from each message."""
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    from google.protobuf import json_format

    # Try importing AmigaTpdo1 (OS 2.0)
    try:
        from farm_ng.canbus.packet import AmigaTpdo1
        print("[SDK] AmigaTpdo1 from farm_ng.canbus.packet  OK")
    except ImportError:
        AmigaTpdo1 = None
        print("[SDK] AmigaTpdo1 not found in farm_ng.canbus.packet")

    # Try payload_to_protobuf
    payload_to_protobuf = None
    try:
        from farm_ng.core.events_file_reader import payload_to_protobuf as _p
        payload_to_protobuf = _p
        print("[SDK] payload_to_protobuf from events_file_reader  OK")
    except ImportError:
        pass
    if payload_to_protobuf is None:
        try:
            from farm_ng.core.event_client import payload_to_protobuf as _p
            payload_to_protobuf = _p
            print("[SDK] payload_to_protobuf from event_client  OK")
        except ImportError:
            print("[SDK] payload_to_protobuf not found — will show raw proto fields")

    config = json_format.ParseDict(
        {
            'name': 'canbus',
            'host': host,
            'port': port,
            'subscriptions': [
                {'uri': {'path': path, 'query': query}, 'every_n': 1}
            ],
        },
        EventServiceConfig(),
    )
    sub_req = SubscribeRequest(uri=Uri(path=path, query=query), every_n=1)

    print(f"\nDecoding {n_msgs} messages from {host}:{port}{path}?{query}")
    print("-" * 60)
    count = 0
    try:
        async with asyncio.timeout(30.0):
            async for event, payload in EventClient(config).subscribe(sub_req, decode=False):
                count += 1
                print(f"\n[msg {count}]  seq={event.sequence}  stamp={event.stamp.stamp}")
                if payload_to_protobuf is not None and AmigaTpdo1 is not None:
                    try:
                        message = payload_to_protobuf(event, payload)
                        print(f"  proto type: {type(message).__name__}")
                        # Try to extract AmigaTpdo1
                        tpdo1_proto = getattr(message, 'amiga_tpdo1', None)
                        if tpdo1_proto is not None:
                            tpdo1 = AmigaTpdo1.from_proto(tpdo1_proto)
                            print(f"  measured_speed        = {tpdo1.measured_speed:.4f} m/s")
                            print(f"  measured_angular_rate = {tpdo1.measured_angular_rate:.4f} rad/s")
                            print(f"  state                 = {tpdo1.state}")
                        else:
                            # Print all fields the proto has
                            fields = [(f.name, getattr(message, f.name, '?'))
                                      for f in message.DESCRIPTOR.fields]
                            print("  fields:", {k: v for k, v in fields[:10]})
                    except Exception as dec_err:
                        print(f"  decode error: {dec_err!r}")
                        print(f"  raw payload ({len(payload)} bytes): {payload[:40].hex()}")
                else:
                    print(f"  raw payload ({len(payload)} bytes): {payload[:40].hex()}")
                if count >= n_msgs:
                    break
    except TimeoutError:
        print(f"\nTimeout.  Received {count} messages total.")
    except Exception as e:
        print(f"\nError: {e!r}")


async def scan_ports(host: str):
    """Quick scan of common farm-ng service ports."""
    candidates = [
        (6001, '/state',  'service_name=canbus'),
        (6001, '/state',  ''),
        (6001, '/',       ''),
        (6002, '/state',  'service_name=canbus'),
        (6002, '/state',  ''),
        (50051, '/state', 'service_name=canbus'),
        (50051, '/state', ''),
        (20001, '/state', 'service_name=filter'),  # sanity check — known working
    ]
    print(f"Scanning {host} for canbus service…\n")
    found = []
    for port, path, query in candidates:
        ok = await probe_canbus(host, port, path, query, timeout=5.0, n_msgs=1)
        if ok:
            found.append((port, path, query))
    print("\n" + "=" * 60)
    if found:
        print("FOUND working endpoints:")
        for port, path, query in found:
            q = f"?{query}" if query else ""
            print(f"  {host}:{port}{path}{q}")
    else:
        print("No canbus endpoint responded.  Check Tailscale / robot power.")
    return found


async def main_async(args):
    if args.scan:
        await scan_ports(args.host)
        return

    # Load from JSON config if provided
    host, port, path, query = args.host, args.port, args.path, args.query
    if args.config:
        cfg = json.loads(Path(args.config).read_text())
        host = cfg.get('host', host)
        port = int(cfg.get('port', port))
        subs = cfg.get('subscriptions', [])
        if subs:
            uri = subs[0].get('uri', {})
            path = uri.get('path', path)
            query = uri.get('query', query)
        print(f"Loaded config: {host}:{port}  path={path!r}  query={query!r}\n")

    if args.decode:
        await decode_and_print(host, port, path, query, n_msgs=args.n)
    else:
        ok = await probe_canbus(host, port, path, query, timeout=args.timeout, n_msgs=args.n)
        if ok and not args.quiet:
            print("\nRunning decode pass (Ctrl-C to stop)…")
            await decode_and_print(host, port, path, query, n_msgs=args.n)


def main():
    if not _check_sdk():
        sys.exit(1)

    p = argparse.ArgumentParser(description="Diagnose Amiga canbus gRPC service")
    p.add_argument('--config', default='config/service_configs/canbus_config.json',
                   help='Path to canbus service_config JSON')
    p.add_argument('--host', default='camphor-clone.tail0be07.ts.net')
    p.add_argument('--port', type=int, default=6001)
    p.add_argument('--path', default='/state')
    p.add_argument('--query', default='service_name=canbus')
    p.add_argument('--timeout', type=float, default=10.0,
                   help='Seconds to wait for first message')
    p.add_argument('--n', type=int, default=5,
                   help='Number of messages to receive')
    p.add_argument('--decode', action='store_true',
                   help='Go straight to decode pass')
    p.add_argument('--scan', action='store_true',
                   help='Probe multiple ports/paths to find the service')
    p.add_argument('--quiet', action='store_true')
    args = p.parse_args()

    asyncio.run(main_async(args))


if __name__ == '__main__':
    main()
