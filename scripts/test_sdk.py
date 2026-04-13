#!/usr/bin/env python3
"""
test_sdk.py — Step 2: Verify the farm-ng OS 2.0 Python SDK is correctly installed.

Run after test_system.sh passes.

Usage:
    # On the Amiga brain — use farm-ng venv:
    source /farm_ng_image/venv/bin/activate
    python3 scripts/test_sdk.py

    # Or after install_farmng.sh:
    python3 scripts/test_sdk.py

Expected result: all checks print OK.  Any FAIL needs fixing before proceeding.
"""

import sys

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


print("=" * 48)
print(" farm-ng SDK verification")
print("=" * 48)

# -----------------------------------------------------------------------
sep("1. Python version")
info(f"Python {sys.version}")
major, minor = sys.version_info[:2]
if (major, minor) >= (3, 8):
    ok(f"Python {major}.{minor} >= 3.8 (required)")
else:
    fail(f"Python {major}.{minor} is too old — need >= 3.8")

# -----------------------------------------------------------------------
sep("2. protobuf")
try:
    import google.protobuf
    ver = google.protobuf.__version__
    major_pb = int(ver.split(".")[0])
    minor_pb = int(ver.split(".")[1])
    if major_pb >= 4 or (major_pb == 3 and minor_pb >= 20):
        ok(f"protobuf {ver} >= 3.20 (required)")
    else:
        fail(
            f"protobuf {ver} is too old (need >= 3.20)",
            "Fix:  pip install 'protobuf>=3.20,<=5.27.5'",
        )
except ImportError:
    fail("protobuf not installed", "Fix:  pip install 'protobuf>=3.20,<=5.27.5'")

# -----------------------------------------------------------------------
sep("3. farm_ng core")
try:
    import farm_ng.core
    ok("farm_ng.core importable")
except ImportError as e:
    fail(f"farm_ng.core missing: {e}", "Fix:  pip install farm-ng-core>=2.0.0")

try:
    from farm_ng.core.event_client import EventClient
    ok("farm_ng.core.event_client.EventClient")
except ImportError as e:
    fail(f"EventClient missing: {e}")

try:
    from farm_ng.core.event_service_pb2 import EventServiceConfig, EventServiceConfigList, SubscribeRequest
    ok("farm_ng.core.event_service_pb2: EventServiceConfig, EventServiceConfigList, SubscribeRequest")
except ImportError as e:
    fail(f"event_service_pb2 missing: {e}")

try:
    from farm_ng.core.uri_pb2 import Uri
    ok("farm_ng.core.uri_pb2.Uri")
except ImportError as e:
    fail(f"uri_pb2 missing: {e}")

# -----------------------------------------------------------------------
sep("4. farm_ng canbus")
try:
    import farm_ng.canbus
    ok("farm_ng.canbus importable")
except ImportError as e:
    fail(f"farm_ng.canbus missing: {e}", "Fix:  pip install farm-ng-amiga>=2.0.0")

try:
    from farm_ng.canbus.canbus_pb2 import Twist2d
    ok("farm_ng.canbus.canbus_pb2.Twist2d  (OS 2.0 velocity command)")
    # Confirm expected fields exist
    t = Twist2d()
    t.linear_velocity_x = 0.0
    t.angular_velocity = 0.0
    ok("  Twist2d fields: linear_velocity_x, angular_velocity — OK")
except ImportError as e:
    fail(f"Twist2d missing: {e}")
except AttributeError as e:
    fail(f"Twist2d field missing: {e} — may be wrong SDK version")

try:
    from farm_ng.canbus.packet import AmigaTpdo1
    ok("farm_ng.canbus.packet.AmigaTpdo1  (OS 2.0 wheel state)")
except ImportError as e:
    fail(
        f"AmigaTpdo1 missing from farm_ng.canbus.packet: {e}",
        "This is the OS 2.0 location — do NOT use the old CanbusClient",
    )

# -----------------------------------------------------------------------
sep("5. payload_to_protobuf (location varies by SDK patch version)")
_found = False
try:
    from farm_ng.core.events_file_reader import payload_to_protobuf
    ok("payload_to_protobuf found in farm_ng.core.events_file_reader")
    _found = True
except ImportError:
    pass

if not _found:
    try:
        from farm_ng.core.event_client import payload_to_protobuf
        ok("payload_to_protobuf found in farm_ng.core.event_client")
        _found = True
    except ImportError:
        pass

if not _found:
    fail(
        "payload_to_protobuf not found in either location",
        "amiga_ros2_bridge canbus decode will fall back to raw payload",
    )

# -----------------------------------------------------------------------
sep("6. farm_ng_core_pybind (optional — used for filter state pose)")
try:
    from farm_ng_core_pybind import Pose3F64
    ok("farm_ng_core_pybind.Pose3F64 available (full filter pose decoding)")
except ImportError:
    info("farm_ng_core_pybind not available — filter pose will use proto-direct fallback")
    info("This is non-fatal; GPS heading will still be published")

# -----------------------------------------------------------------------
sep("7. asyncio (standard library)")
try:
    import asyncio
    ok(f"asyncio available (Python built-in)")
except ImportError:
    fail("asyncio missing — very unexpected on Python 3.8+")

# -----------------------------------------------------------------------
sep("8. service_config.json parse check")
import json
from pathlib import Path

config_path = Path(__file__).parent.parent / "service_config.json"
if not config_path.exists():
    fail(f"service_config.json not found at {config_path}")
else:
    try:
        with open(config_path) as f:
            raw = json.load(f)
        names = [c["name"] for c in raw.get("configs", [])]
        if "canbus" in names:
            ok(f"service_config.json parseable — services: {names}")
            canbus_cfg = next(c for c in raw["configs"] if c["name"] == "canbus")
            info(f"  canbus host: {canbus_cfg['host']}  port: {canbus_cfg['port']}")
        else:
            fail("service_config.json missing 'canbus' entry")
    except Exception as e:
        fail(f"service_config.json parse error: {e}")

    # Also test protobuf parse
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList
        with open(config_path) as f:
            raw = json.load(f)
        cfg_list = json_format.ParseDict(raw, EventServiceConfigList())
        ok(f"service_config.json → EventServiceConfigList OK ({len(cfg_list.configs)} services)")
    except Exception as e:
        fail(f"EventServiceConfigList parse failed: {e}")

# -----------------------------------------------------------------------
print("")
print("=" * 48)
print(f" Results: {PASS} passed, {FAIL} failed")
print("=" * 48)
if FAIL == 0:
    print(" All SDK checks passed.")
    print(" Next step:  python3 scripts/test_canbus.py")
else:
    print(" Fix the FAIL items above, then re-run this script.")
    print("")
    print(" Common fix (Amiga brain):")
    print("   source /farm_ng_image/venv/bin/activate")
    print("   python3 scripts/test_sdk.py")
    print("")
    print(" Or install SDK manually:")
    print("   bash scripts/install_farmng.sh")
print("")
