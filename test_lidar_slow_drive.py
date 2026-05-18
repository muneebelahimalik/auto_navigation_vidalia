#!/usr/bin/env python3
"""
test_lidar_slow_drive.py — Low-speed LiDAR-gated drive test.

Runs LiDAR validation before moving, then commands the robot at a low speed
(default 0.12 m/s) while printing per-scan diagnostics.

Usage:
    python3 test_lidar_slow_drive.py              # validate then drive
    python3 test_lidar_slow_drive.py --validate-only   # check sensor, no motion
    python3 test_lidar_slow_drive.py --speed 0.08 --scans 20
"""

import argparse
import asyncio
import json
import math
from pathlib import Path

from google.protobuf import json_format
from farm_ng.core.event_service_pb2 import EventServiceConfigList

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import (
    LIDAR_MOUNT_HEIGHT,
    MIN_OBSTACLE_GROUND_HEIGHT,
    front_zone_points,
    nearest_range,
    sector_counts,
    validate_lidar_startup,
)

CONFIG_PATH = Path("service_config.json")


def load_canbus_config():
    with open(CONFIG_PATH) as f:
        raw = json.load(f)
    config_list = json_format.ParseDict(raw, EventServiceConfigList())
    configs = {cfg.name: cfg for cfg in config_list.configs}
    return configs["canbus"]


async def run_test(args):
    print("=" * 60)
    print("LOW-SPEED LIDAR-GATED MOVEMENT TEST")
    if args.validate_only:
        print("Mode: VALIDATE ONLY — no wheel commands will be sent")
    else:
        print("*** This WILL command the robot wheels ***")
    print(f"Speed limit       : {args.speed:.2f} m/s")
    print(f"Corridor          : 0 < y <= {args.look_ahead:.2f} m, |x| < {args.half_width:.2f} m")
    print(
        f"Min ground height : {args.min_ground_height:.2f} m above ground "
        f"(sensor-frame z > {args.min_ground_height - LIDAR_MOUNT_HEIGHT:.3f} m)"
    )
    print(f"Stop distance     : {args.stop_distance:.2f} m")
    print(f"Max drive scans   : {args.scans}")
    print("=" * 60)

    canbus = None
    if not args.validate_only:
        canbus = CanbusInterface(
            load_canbus_config(),
            max_linear=args.speed,
            max_angular=0.3,
        )

    try:
        async with LidarDriver() as lidar:
            # --- Pre-motion LiDAR validation ---
            healthy = await validate_lidar_startup(lidar, num_scans=args.validate_scans)
            if not healthy:
                print("\n[drive test] ABORT — LiDAR validation failed, not moving.")
                return
            if args.validate_only:
                print("\n[drive test] Validate-only mode complete.")
                return

            print(f"\n[drive test] Validation passed — starting {args.scans}-scan drive.\n")

            # --- Drive loop ---
            count = 0
            async for scan in lidar.scan_stream():
                count += 1

                front = front_zone_points(
                    scan,
                    look_ahead=args.look_ahead,
                    half_width=args.half_width,
                    min_ground_height=args.min_ground_height,
                )
                nearest = nearest_range(front)
                sct = sector_counts(scan)

                if nearest <= args.stop_distance:
                    cmd = 0.0
                    action = "STOP"
                elif nearest <= args.look_ahead:
                    ratio = (nearest - args.stop_distance) / (
                        args.look_ahead - args.stop_distance
                    )
                    cmd = args.speed * max(0.0, min(1.0, ratio))
                    action = "SLOW"
                else:
                    cmd = args.speed
                    action = "MOVE"

                print(
                    f"scan={count:03d} | front={len(front):3d} pts | "
                    f"nearest={nearest:.2f} m | cmd={cmd:.2f} m/s | {action} | "
                    f"sectors fwd={sct['fwd']} L={sct['left']} R={sct['right']}"
                )

                await canbus.send_twist(cmd, 0.0)

                if count >= args.scans:
                    break

    finally:
        if canbus is not None:
            print("Sending final STOP command.")
            await canbus.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, default=0.12, help="Max drive speed (m/s)")
    parser.add_argument("--scans", type=int, default=30, help="Number of drive-loop scans")
    parser.add_argument(
        "--validate-scans", type=int, default=5,
        help="Scans collected during pre-motion validation",
    )
    parser.add_argument("--half-width", type=float, default=0.35,
                        help="Safety corridor half-width (m)")
    parser.add_argument("--look-ahead", type=float, default=2.0,
                        help="Forward look-ahead distance (m)")
    parser.add_argument("--stop-distance", type=float, default=0.80,
                        help="Hard-stop range (m)")
    parser.add_argument(
        "--min-ground-height", type=float, default=MIN_OBSTACLE_GROUND_HEIGHT,
        help=f"Min height above ground to classify as obstacle "
             f"(m, default {MIN_OBSTACLE_GROUND_HEIGHT})",
    )
    parser.add_argument(
        "--validate-only", action="store_true",
        help="Run sensor validation then exit without moving",
    )
    args = parser.parse_args()
    asyncio.run(run_test(args))


if __name__ == "__main__":
    main()
