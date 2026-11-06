#!/usr/bin/env -S python3 -u
"""
gnss_logger.py — record the Amiga's GPS stream on the side, for ground truth.

READ-ONLY.  It only SUBSCRIBES to services that are already running on the
brain (the same way the Launcher apps read them).  It never opens the GNSS
serial port, never touches NTRIP, and nothing in the navigation stack imports
it — row_follow.py behaves exactly the same whether this runs or not.

Nothing is guessed: service ports and topic names are DISCOVERED on the brain.
Every listening localhost TCP port is asked for its topic list (farm-ng
``list_uris``); topics whose name or service mentions gps / gnss / pvt are
logged.  Every message is written verbatim (all fields, as published) — units
and field meanings are read from the log afterwards, not assumed here.

Each line of gnss/gnss_<ts>.jsonl:
    {"mono": <time.monotonic()>, "wall": <time.time()>, "port": 3001,
     "path": "/pvt", "query": "service_name=gps", "msg": {...all fields...}}
``mono`` is the same clock row_follow's telemetry uses.

Usage (brain, second terminal, farm-ng venv):
    python3 scripts/gnss_logger.py --discover    # list every service + topic, then exit
    python3 scripts/gnss_logger.py               # log GPS topics until Ctrl+C
    python3 scripts/gnss_logger.py --port P --path /x --query service_name=gps   # explicit
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

_FARMNG_SITE = Path("/farm_ng_image/venv/lib/python3.8/site-packages")
if _FARMNG_SITE.exists() and str(_FARMNG_SITE) not in sys.path:
    sys.path.insert(0, str(_FARMNG_SITE))

GPS_WORDS = ("gps", "gnss", "pvt", "rtk", "relpos", "ntrip")


def listening_tcp_ports(proc: str = "/proc") -> list:
    """Local TCP ports in LISTEN state (from /proc/net/tcp[6]; no root needed)."""
    ports = set()
    for table in ("net/tcp", "net/tcp6"):
        try:
            lines = (Path(proc) / table).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            f = line.split()
            if len(f) < 4 or f[3] != "0A":          # 0A = LISTEN
                continue
            try:
                ports.add(int(f[1].rsplit(":", 1)[1], 16))
            except (IndexError, ValueError):
                continue
    return sorted(ports)


def is_gps_topic(path: str, query: str) -> bool:
    s = f"{path} {query}".lower()
    return any(w in s for w in GPS_WORDS)


def _client(port: int):
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import EventServiceConfig
    return EventClient(EventServiceConfig(name=f"probe{port}", host="localhost", port=port))


async def _list_uris(port: int, timeout: float) -> list:
    """[(path, query), ...] the service on ``port`` publishes; [] if not farm-ng."""
    try:
        uris = await asyncio.wait_for(_client(port).list_uris(), timeout=timeout)
    except BaseException:                      # not a farm-ng service / no reply
        return []
    return [(u.path, u.query) for u in uris]


async def discover(timeout: float = 1.5) -> dict:
    ports = [p for p in listening_tcp_ports() if p >= 1024]
    found = {}
    for port, uris in zip(ports, await asyncio.gather(*(_list_uris(p, timeout) for p in ports))):
        if uris:
            found[port] = uris
    return found


def _to_dict(msg) -> dict:
    from google.protobuf.json_format import MessageToDict
    try:
        return MessageToDict(msg, preserving_proto_field_name=True)
    except Exception:                          # not a protobuf message
        return {"repr": repr(msg)}


async def _log_topic(port, path, query, out, counts, show_first):
    from farm_ng.core.event_service_pb2 import SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    key = f"{port}{path}?{query}"
    req = SubscribeRequest(uri=Uri(path=path, query=query), every_n=1)
    while True:
        try:
            async for _event, msg in _client(port).subscribe(req, decode=True):
                d = _to_dict(msg)
                out.write(json.dumps({"mono": time.monotonic(), "wall": time.time(),
                                      "port": port, "path": path, "query": query,
                                      "type": type(msg).__name__, "msg": d}) + "\n")
                counts[key] = counts.get(key, 0) + 1
                if show_first and counts[key] == 1:
                    print(f"\n[gnss] first message on {key} ({type(msg).__name__}):")
                    print(json.dumps(d, indent=1)[:3000])
        except asyncio.CancelledError:
            return
        except Exception as exc:               # service restarted etc. — retry
            print(f"\n[gnss] {key}: {exc!r} — retrying in 2 s")
            await asyncio.sleep(2.0)


async def run(args) -> None:
    if args.port and args.path:
        topics = [(args.port, args.path, args.query)]
    else:
        found = await discover()
        if not found:
            print("[gnss] no farm-ng services answered on localhost — is this the brain, "
                  "and is the farm-ng venv active?")
            return
        if args.discover:
            for port, uris in sorted(found.items()):
                print(f"port {port}:")
                for path, query in uris:
                    mark = "  <- GPS" if is_gps_topic(path, query) else ""
                    print(f"    {path}  {query}{mark}")
            return
        topics = [(p, path, q) for p, uris in found.items() for path, q in uris
                  if is_gps_topic(path, q)]
        if not topics:
            print("[gnss] no GPS-looking topic found. Run with --discover and pass "
                  "--port/--path/--query explicitly.")
            return
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / time.strftime("gnss_%Y%m%d_%H%M%S.jsonl")
    print(f"[gnss] logging {len(topics)} topic(s) → {path}")
    for t in topics:
        print(f"    port {t[0]}  {t[1]}  {t[2]}")
    counts: dict = {}
    with open(path, "a", buffering=1) as out:
        tasks = [asyncio.ensure_future(_log_topic(*t, out, counts, True)) for t in topics]
        t0 = time.monotonic()
        try:
            while True:
                await asyncio.sleep(1.0)
                el = time.monotonic() - t0
                s = "  ".join(f"{k.split('?')[0]}={n} ({n / el:.1f}/s)" for k, n in counts.items())
                print(f"\r[gnss] {el:6.0f}s  {s or 'waiting for messages…'}   ", end="")
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            print(f"\n[gnss] saved {path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--discover", action="store_true", help="list services/topics and exit")
    ap.add_argument("--out", default="gnss", help="output folder (default gnss/)")
    ap.add_argument("--port", type=int, default=0, help="explicit service port")
    ap.add_argument("--path", default="", help="explicit topic path, e.g. one shown by --discover")
    ap.add_argument("--query", default="", help="explicit topic query, as shown by --discover")
    try:
        asyncio.run(run(ap.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
