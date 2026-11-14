#!/usr/bin/env python3
"""
diag_canbus_topics.py — READ-ONLY: list what the canbus service publishes and
what each message contains, so wheel odometry subscribes to the right topic.

Never sends a command.  Field 2026-11-13: the odometry logged odom_src=cmd on
every scan (no measured wheel speed), so this shows the real topics instead of
guessing them.

    source /farm_ng_image/venv/bin/activate
    python3 scripts/diag_canbus_topics.py              # canbus on port 6001
    python3 scripts/diag_canbus_topics.py --port 6001 --secs 3
"""
from __future__ import annotations

import argparse
import asyncio


async def main(port: int, secs: float) -> None:
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri

    client = EventClient(EventServiceConfig(name="canbus", host="localhost", port=port))
    try:
        uris = await asyncio.wait_for(client.list_uris(), timeout=secs)
    except BaseException as exc:
        print(f"could not list topics on port {port}: {exc!r}")
        uris = []
    print(f"port {port}: {len(uris)} topics")
    for u in uris:
        print(f"  path={u.path!r}  query={u.query!r}")

    # Try each listed topic, plus the paths odometry.py tries without a query.
    tries = [(u.path, u.query) for u in uris]
    for p in ("/state", "/twist", "/canbus", "/amiga"):
        if (p, "") not in tries:
            tries.append((p, ""))
    for path, query in tries:
        req = SubscribeRequest(uri=Uri(path=path, query=query), every_n=1)
        stream = client.subscribe(req, decode=True).__aiter__()
        try:
            _ev, msg = await asyncio.wait_for(stream.__anext__(), timeout=secs)
        except asyncio.TimeoutError:
            print(f"\n{path} ?{query}: no message in {secs:.0f} s")
            continue
        except BaseException as exc:
            print(f"\n{path} ?{query}: error {exc!r}")
            continue
        fields = [f.name for f, _ in msg.ListFields()] if hasattr(msg, "ListFields") else []
        print(f"\n{path} ?{query}: {type(msg).__name__}")
        print(f"  fields: {fields}")
        for name in ("measured_speed", "measured_angular_rate",
                     "linear_velocity_x", "angular_velocity"):
            if hasattr(msg, name):
                print(f"  {name} = {getattr(msg, name)}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--port", type=int, default=6001)
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    asyncio.run(main(a.port, a.secs))
