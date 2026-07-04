#!/usr/bin/env -S python3 -u
"""
diag_filter_imu.py — Probe what orientation the Amiga publishes.

Answers ONE question for the slope-correction work: does the robot expose a
usable PITCH and ROLL (not just heading), and does it update before GPS
converges?  Two candidate sources are checked:

  1. filter service  (FilterState, /state)  — the GPS+IMU fused pose we already
     use for headland heading.  Its pose MAY be a 2-D nav pose (yaw only) — this
     script tells you whether it also carries pitch/roll.
  2. OAK-D IMU        (/imu on the camera service, port 50010) — a raw IMU with
     gravity-referenced accel; a fallback pitch/roll source if the filter has none.

For each source it prints the decoded message once, lists every field, and — if
it finds a quaternion (x,y,z,w) anywhere in the message — converts it to
roll/pitch/yaw so you can see whether pitch/roll are actually populated.

Run in the farm-ng venv on the brain:
    python3 scripts/diag_filter_imu.py                 # both sources
    python3 scripts/diag_filter_imu.py --source filter
    python3 scripts/diag_filter_imu.py --source oak --oak-id oak0
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _quat_to_rpy(x: float, y: float, z: float, w: float) -> "tuple[float, float, float]":
    """Quaternion -> (roll, pitch, yaw) in degrees (ZYX / aerospace)."""
    # roll (x-axis)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _find_quaternions(msg, prefix="") -> "list[tuple[str, tuple]]":
    """Recursively scan a protobuf message for a quaternion (fields x,y,z,w)."""
    found = []
    try:
        fields = {f.name: getattr(msg, f.name) for f in msg.DESCRIPTOR.fields}
    except Exception:
        return found
    names = set(fields)
    if {"x", "y", "z", "w"} <= names:
        q = (float(fields["x"]), float(fields["y"]),
             float(fields["z"]), float(fields["w"]))
        found.append((prefix or "<root>", q))
    for name, val in fields.items():
        if hasattr(val, "DESCRIPTOR"):
            found += _find_quaternions(val, f"{prefix}.{name}" if prefix else name)
    return found


def _dump(msg) -> None:
    print(f"  message type: {type(msg).__module__}.{type(msg).__name__}")
    try:
        names = [f.name for f in msg.DESCRIPTOR.fields]
        print(f"  fields: {names}")
    except Exception:
        pass
    for key in ("heading", "has_converged"):
        if hasattr(msg, key):
            print(f"    {key} = {getattr(msg, key)}")
    quats = _find_quaternions(msg)
    if not quats:
        print("    NO quaternion found → this message carries no 3-D orientation "
              "(pitch/roll NOT available here).")
    for path, q in quats:
        roll, pitch, yaw = _quat_to_rpy(*q)
        print(f"    quaternion @ {path}: xyzw={tuple(round(v,4) for v in q)}")
        print(f"      → roll={roll:+.2f}°  pitch={pitch:+.2f}°  yaw={yaw:+.2f}°  "
              f"(pitch/roll usable if these track the robot's tilt)")


def _load_filter_config():
    config_path = Path(__file__).resolve().parent.parent / "service_config.json"
    if not config_path.exists():
        print(f"  [filter] no service_config.json at {config_path}")
        return None
    from google.protobuf import json_format
    from farm_ng.core.event_service_pb2 import EventServiceConfigList
    with open(config_path) as f:
        raw = json.load(f)
    config_list = json_format.ParseDict(raw, EventServiceConfigList())
    for cfg in config_list.configs:
        if cfg.name == "filter":
            return cfg
    print("  [filter] no 'filter' service in service_config.json")
    return None


async def probe_filter(n: int) -> None:
    print("\n=== FILTER SERVICE (FilterState / /state) ===")
    cfg = _load_filter_config()
    if cfg is None:
        return
    from farm_ng.core.event_client import EventClient
    from farm_ng.core.event_service_pb2 import SubscribeRequest
    from farm_ng.core.uri_pb2 import Uri
    for path in ("/state", "/filter"):
        try:
            req = SubscribeRequest(uri=Uri(path=path), every_n=1)
            stream = EventClient(cfg).subscribe(req, decode=True).__aiter__()
            _e, msg = await asyncio.wait_for(stream.__anext__(), timeout=5.0)
            print(f"  subscribed at {path}")
            _dump(msg)
            return
        except asyncio.TimeoutError:
            print(f"  {path}: timeout (no message in 5 s)")
        except Exception as exc:  # noqa: BLE001
            print(f"  {path}: {type(exc).__name__}: {exc}")
    print("  filter service produced nothing usable.")


def _find_vec3(msg, prefix=""):
    """Recursively find (x,y,z) float triplets (a Vec3) in a protobuf message."""
    found = []
    try:
        fields = {f.name: getattr(msg, f.name) for f in msg.DESCRIPTOR.fields}
    except Exception:
        return found
    names = set(fields)
    if {"x", "y", "z"} <= names and "w" not in names:
        found.append((prefix or "<root>",
                      (float(fields["x"]), float(fields["y"]), float(fields["z"]))))
    for name, val in fields.items():
        if hasattr(val, "DESCRIPTOR"):
            found += _find_vec3(val, f"{prefix}.{name}" if prefix else name)
    return found


def _accel_to_pitch_roll(ax, ay, az):
    """Gravity vector (accel, m/s²) → pitch, roll (deg) in the sensor frame.

    Sensor convention here matches the LiDAR: x=right, y=forward, z=up.
    pitch = nose up/down about x; roll = tilt about y (forward)."""
    pitch = math.degrees(math.atan2(ay, math.hypot(ax, az)))
    roll = math.degrees(math.atan2(ax, math.hypot(ay, az)))
    return pitch, roll


async def probe_oak(oak_id: str, watch: int) -> None:
    print(f"\n=== OAK-D IMU (/imu on {oak_id}, port 50010) ===")
    try:
        from farm_ng.core.event_client import EventClient
        from farm_ng.core.event_service_pb2 import EventServiceConfig, SubscribeRequest
        from farm_ng.core.uri_pb2 import Uri
        cfg = EventServiceConfig(name="oak", host="localhost", port=50010)
        req = SubscribeRequest(uri=Uri(path="/imu", query=f"service_name={oak_id}"),
                               every_n=1)
        stream = EventClient(cfg).subscribe(req, decode=True).__aiter__()
        _e, msg = await asyncio.wait_for(stream.__anext__(), timeout=5.0)
        print("  subscribed at /imu")
        _dump(msg)

        # Dig into the first packet to reveal the accelerometer field path.
        pkts = list(getattr(msg, "packets", []))
        if not pkts:
            print("    (message had no packets this frame)")
            return
        pkt = pkts[0]
        print(f"  packet type: {type(pkt).__name__}  fields: "
              f"{[f.name for f in pkt.DESCRIPTOR.fields]}")
        vecs = _find_vec3(pkt)
        accel_path = None
        for path, (x, y, z) in vecs:
            mag = math.sqrt(x * x + y * y + z * z)
            tag = ""
            if 8.5 <= mag <= 11.0:          # ~9.8 → this is the accelerometer
                tag = "  <-- ACCELEROMETER (|a|≈g)"
                accel_path = path
            print(f"    vec3 @ {path}: ({x:+.3f}, {y:+.3f}, {z:+.3f})  |v|={mag:.2f}{tag}")
        if accel_path is None:
            print("    No ~9.8 m/s² vector found — check which vec3 is the accel.")
            return
        print(f"\n  >>> accelerometer at packet field '{accel_path}'.")
        print("      Streaming gravity-referenced pitch/roll — TILT the robot now")
        print("      (nose up a ramp = pitch; rock side-to-side = roll):\n")

        def _read_accel(m):
            for _p, v in _find_vec3(m):
                if 8.5 <= math.sqrt(sum(c * c for c in v)) <= 11.0:
                    return v
            return None

        import time
        t0 = time.monotonic()
        async for _e, m in stream:
            for p in getattr(m, "packets", []):
                a = _read_accel(p)
                if a is None:
                    continue
                # NOTE: axis mapping (which of x/y/z is fwd/right/up) is confirmed
                # by watching which angle moves when you tilt each way.
                pitch, roll = _accel_to_pitch_roll(*a)
                print(f"    accel=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})  "
                      f"pitch={pitch:+6.2f}°  roll={roll:+6.2f}°")
                break
            if time.monotonic() - t0 > watch:
                break
    except asyncio.TimeoutError:
        print("  /imu: timeout (IMU stream not publishing)")
    except Exception as exc:  # noqa: BLE001
        print(f"  /imu: {type(exc).__name__}: {exc}")


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["both", "filter", "oak"], default="both")
    ap.add_argument("--oak-id", default="oak0", help="OAK-D sub-service (default oak0)")
    ap.add_argument("--watch", type=int, default=15, metavar="SEC",
                    help="Seconds to stream live pitch/roll so you can tilt the "
                         "robot and confirm they track (default 15).")
    args = ap.parse_args()
    try:
        import farm_ng  # noqa: F401
    except ImportError:
        print("farm-ng SDK not found — run this in the farm-ng venv "
              "(source /farm_ng_image/venv/bin/activate).")
        return
    if args.source in ("both", "filter"):
        await probe_filter(5)
    if args.source in ("both", "oak"):
        await probe_oak(args.oak_id, args.watch)
    print("\nInterpretation:")
    print("  • If the filter message shows a quaternion whose pitch/roll track the")
    print("    robot's tilt (and updates with has_converged=False), the filter is")
    print("    our source → ground correction uses filter pitch/roll.")
    print("  • If the filter is yaw-only (no quaternion / pitch≈roll≈0 always), use")
    print("    the OAK-D /imu accel vector for gravity-referenced pitch/roll instead.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
