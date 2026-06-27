#!/usr/bin/env -S python3 -u
"""
diag_filter.py — Does the Amiga filter service publish a usable heading?

The headland U-turn prefers the filter service's FilterState heading (a GPS+IMU
fused yaw that does NOT depend on wheel contact, so it is immune to skid-steer
scrub).  When it is live the status shows ``rot=…[imu]``; when it is not, the
turn falls back to wheel odometry × --turn-scrub-comp (``rot=…[wheel]``), which
is only a calibrated guess.

This probe answers, in one run, WHY you are seeing ``[wheel]``:
  * is there a 'filter' entry in service_config.json?
  * does the service deliver a message on /state or /filter within a few s?
  * does that message carry a `heading` field, and what is `has_converged`?

The U-turn only needs the heading's *relative change*, so it works even when
has_converged is False — it just needs FilterState to ARRIVE.

Run on the brain:
    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia
    python3 scripts/diag_filter.py
"""
from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

_FARMNG_SITE = Path("/farm_ng_image/venv/lib/python3.8/site-packages")
if _FARMNG_SITE.exists() and str(_FARMNG_SITE) not in sys.path:
    sys.path.insert(0, str(_FARMNG_SITE))

_PATHS = ["/state", "/filter"]
_TIMEOUT = 8.0


def _load_filter_config():
    cfg_path = Path(__file__).parent.parent / "service_config.json"
    if not cfg_path.exists():
        print(f"  service_config.json not found at {cfg_path}")
        return None
    from google.protobuf import json_format
    from farm_ng.core.event_service_pb2 import EventServiceConfigList
    config_list = json_format.ParseDict(json.loads(cfg_path.read_text()),
                                        EventServiceConfigList())
    names = [c.name for c in config_list.configs]
    print(f"  services in service_config.json: {names}")
    for cfg in config_list.configs:
        if cfg.name == "filter":
            return cfg
    return None


async def _probe(cfg) -> bool:
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri

    for path in _PATHS:
        print(f"\n  trying {cfg.host}:{cfg.port}{path} …")
        try:
            stream = EventClient(cfg).subscribe(
                SubscribeRequest(uri=Uri(path=path), every_n=1), decode=True).__aiter__()
            _ev, msg = await asyncio.wait_for(stream.__anext__(), timeout=_TIMEOUT)
        except asyncio.TimeoutError:
            print(f"    TIMEOUT after {_TIMEOUT:.0f}s — no message on {path}")
            continue
        except Exception as exc:  # noqa: BLE001
            print(f"    error on {path}: {exc}")
            continue
        print(f"    GOT a message: {type(msg).__name__}")
        if hasattr(msg, "heading"):
            conv = getattr(msg, "has_converged", "n/a")
            print(f"    heading = {float(msg.heading):+.3f} rad   has_converged = {conv}")
            print("\n  RESULT: FilterState heading IS available — the U-turn will use "
                  "[imu] once the filter_heading subscriber fix is pulled.")
            return True
        print(f"    message has NO 'heading' field (fields: "
              f"{[f.name for f in msg.DESCRIPTOR.fields]})")
    return False


def main() -> None:
    print("Filter-service heading probe")
    print("=" * 60)
    cfg = _load_filter_config()
    if cfg is None:
        print("\n  RESULT: no 'filter' service in service_config.json.")
        print("  → the U-turn can only use wheel odometry ([wheel]).  Add the")
        print("    filter service to service_config.json (or start it) to get [imu].")
        return
    print(f"  filter service config: {cfg.host}:{cfg.port}")
    ok = asyncio.run(_probe(cfg))
    if not ok:
        print("\n  RESULT: the filter service did not deliver a heading.")
        print("  → it is likely not running / not publishing FilterState yet.")
        print("    Check it is up (farm-ng service manager) and re-run this probe.")
        print("    Until then, calibrate --turn-scrub-comp from the rot=…[wheel] readout.")


if __name__ == "__main__":
    main()
