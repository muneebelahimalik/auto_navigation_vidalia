#!/usr/bin/env python3

import argparse
import asyncio
import json
import math
from pathlib import Path

from google.protobuf import json_format
from farm_ng.core.event_service_pb2 import EventServiceConfigList

from canbus.canbus_interface import CanbusInterface
from lidar.lidar_driver import LidarDriver


CONFIG_PATH = Path("service_config.json")


def load_canbus_config():
    with open(CONFIG_PATH) as f:
        raw = json.load(f)
    config_list = json_format.ParseDict(raw, EventServiceConfigList())
    configs = {cfg.name: cfg for cfg in config_list.configs}
    return configs["canbus"]


def front_zone(points, look_ahead, half_width, min_height):
    return [
        p for p in points
        if 0.0 < p.y <= look_ahead
        and abs(p.x) < half_width
        and p.z > min_height
    ]


def nearest_range(points):
    if not points:
        return math.inf
    return min(math.hypot(p.x, p.y) for p in points)


async def run_test(args):
    print("LOW-SPEED LIDAR-GATED MOVEMENT TEST.")
    print("This WILL command the robot wheels.")
    print(f"Speed limit: {args.speed:.2f} m/s")
    print(
        f"Corridor: 0 < y <= {args.look_ahead:.2f} m, "
        f"|x| < {args.half_width:.2f} m, z > {args.min_height:.2f} m"
    )
    print(f"Stop distance: {args.stop_distance:.2f} m")
    print(f"Max scans: {args.scans}")

    canbus = CanbusInterface(
        load_canbus_config(),
        max_linear=args.speed,
        max_angular=0.3,
    )

    try:
        async with LidarDriver() as lidar:
            count = 0
            async for scan in lidar.scan_stream():
                count += 1
                front = front_zone(
                    scan,
                    args.look_ahead,
                    args.half_width,
                    args.min_height,
                )
                nearest = nearest_range(front)

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
                    f"scan={count:03d} | front_points={len(front):3d} | "
                    f"nearest={nearest:.2f} m | cmd={cmd:.2f} m/s | {action}"
                )

                await canbus.send_twist(cmd, 0.0)

                if count >= args.scans:
                    break

    finally:
        print("Sending final STOP command.")
        await canbus.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--scans", type=int, default=30)
    parser.add_argument("--half-width", type=float, default=0.35)
    parser.add_argument("--look-ahead", type=float, default=2.0)
    parser.add_argument("--stop-distance", type=float, default=0.80)
    parser.add_argument("--min-height", type=float, default=0.10)
    args = parser.parse_args()

    asyncio.run(run_test(args))


if __name__ == "__main__":
    main()