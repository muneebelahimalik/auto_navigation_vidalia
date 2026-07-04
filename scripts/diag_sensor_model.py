#!/usr/bin/env -S python3 -u
"""
diag_sensor_model.py — Identify the Velodyne model from its data packet.

Every VLP data packet ends with two factory bytes: byte 1204 = return mode,
byte 1205 = PRODUCT ID.  The product ID tells us the exact model, which fixes
the vertical channel-angle table the driver must use:

    0x22  VLP-16 (Puck)         channels ±15°, 2.00° spacing
    0x24  VLP-16 Puck Hi-Res    channels ±10°, 1.33° spacing   <-- suspected
    0x28  VLP-32C
    0x21  HDL-32E

If this reads 0x24 (Hi-Res) while lidar_driver.VLP16_VERTICAL_ANGLES is the
±15° table, that mismatch distorts every reconstructed point — which is exactly
what made a true 15°/0.80 m mount read as 22°/1.17 m.

Run on the brain:
    python3 scripts/diag_sensor_model.py
"""
from __future__ import annotations

import socket

PORT = 2368
PACKET = 1206

_PRODUCTS = {
    0x21: ("HDL-32E", "±10.67° 32-ch"),
    0x22: ("VLP-16 (Puck)", "±15°, 2.00° spacing — the driver's current table"),
    0x24: ("VLP-16 Puck Hi-Res", "±10°, 1.33° spacing — DRIVER TABLE IS WRONG for this"),
    0x28: ("VLP-32C", "-25°..+15° 32-ch"),
    0x31: ("VLP-16 Puck LITE", "±15°, 2.00° spacing"),
}
_RETURNS = {0x37: "Strongest", 0x38: "Last", 0x39: "Dual"}


def main() -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except OSError:
        pass
    s.bind(("", PORT))
    s.settimeout(5.0)
    print(f"[sensor-model] listening on UDP :{PORT} …")
    try:
        raw, addr = s.recvfrom(2048)
    except socket.timeout:
        print("  no packet in 5 s — is the VLP streaming? check the network alias.")
        return
    finally:
        s.close()
    if len(raw) < PACKET:
        print(f"  short packet ({len(raw)} bytes) — expected {PACKET}.")
        return
    ret_byte = raw[1204]
    pid_byte = raw[1205]
    ret = _RETURNS.get(ret_byte, f"unknown (0x{ret_byte:02x})")
    name, note = _PRODUCTS.get(pid_byte, (f"UNKNOWN (0x{pid_byte:02x})",
                                          "not in table — tell me this hex value"))
    print(f"  source            : {addr[0]}")
    print(f"  return-mode byte  : 0x{ret_byte:02x}  ({ret})")
    print(f"  PRODUCT-ID byte   : 0x{pid_byte:02x}")
    print(f"  >>> MODEL         : {name}")
    print(f"      {note}")
    if pid_byte == 0x24:
        print("\n  CONFIRMED: this is a Hi-Res. The driver's ±15° angle table is wrong;")
        print("  switching it to the ±10° Hi-Res table makes the mount read its true")
        print("  ~15° pitch and 0.80 m height (your tape + iPhone were correct).")
    elif pid_byte == 0x22:
        print("\n  This says standard VLP-16 (±15°). If the geometry still fits 1.17 m,")
        print("  the channel angles need to be measured empirically — send me this output.")


if __name__ == "__main__":
    main()
