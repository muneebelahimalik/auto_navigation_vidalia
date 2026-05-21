#!/usr/bin/env -S python3 -u
"""
cam_follow.py — Camera-only autonomous onion-row following (no LiDAR).

Uses two OAK-D stereo cameras (left + right, side-mounted at ±cam_x from
robot centreline) for:
  * Row detection  — HSV green-centroid lateral offset + PCA heading
  * Obstacle safety — depth-frame centre-strip obstacle detection

No Velodyne LiDAR required.  Useful as a lightweight backup or in
environments where LiDAR is not available.

Usage (on the Amiga brain):
    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia

    # 1. PERCEPTION-ONLY — robot does not move; verify green detection first:
    python3 scripts/cam_follow.py

    # 2. AUTONOMOUS — robot follows the row:
    python3 scripts/cam_follow.py --auto

    # Multi-row with headland turns:
    python3 scripts/cam_follow.py --auto --rows 4 --headland

Press Ctrl+C at any time to stop immediately.

Key flags:
    --auto            enable motion (default: perception-only)
    --rows N          number of rows (default: 1)
    --headland        open-loop headland turns between rows
    --speed M         max forward speed m/s (default: 0.20)
    --cam-left-id S   left OAK-D device serial (empty = auto-detect)
    --cam-right-id S  right OAK-D device serial (empty = auto-detect)
    --cam-x M         camera lateral offset from centreline (default: 0.915)
    --cam-stop-dist M depth obstacle stop distance (default: 1.2)
    --acquire-conf F  min confidence to leave ACQUIRE (default: 0.20)
    --fps N           camera capture rate (default: 10)
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

_FARMNG_SITE = Path("/farm_ng_image/venv/lib/python3.8/site-packages")
if _FARMNG_SITE.exists() and str(_FARMNG_SITE) not in sys.path:
    sys.path.insert(0, str(_FARMNG_SITE))

from camera.depth_obstacle import DepthObstacleDetector
from camera.oak_driver import OakDriver
from camera.row_detector_visual import VisualRowDetector
from navigation.row_controller import PurePursuitController
from navigation.row_navigator_cam import CamRowNavigator


# ---------------------------------------------------------------------------
def _load_canbus():
    config_path = Path(__file__).parent.parent / "service_config.json"
    if not config_path.exists():
        print(f"[cam_follow] service_config.json not found at {config_path}")
        return None
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList
        from canbus.canbus_interface import CanbusInterface

        with open(config_path) as f:
            raw = json.load(f)
        config_list = json_format.ParseDict(raw, EventServiceConfigList())
        for cfg in config_list.configs:
            if cfg.name == "canbus":
                return CanbusInterface(cfg)
        print("[cam_follow] no 'canbus' service in service_config.json")
    except Exception as exc:
        print(f"[cam_follow] could not load canbus config: {exc}")
    return None


# ---------------------------------------------------------------------------
async def _run(args: argparse.Namespace, nav_ref: list) -> None:
    canbus = _load_canbus()

    if args.auto and canbus is None:
        print("[cam_follow] --auto requested but no canbus available — "
              "falling back to perception-only mode.\n")
        args.auto = False

    left_cam = OakDriver(side="left", device_id=args.cam_left_id, fps=args.fps)
    right_cam = OakDriver(side="right", device_id=args.cam_right_id, fps=args.fps)

    vis_detector = VisualRowDetector(
        cam_x_left=-args.cam_x,
        cam_x_right=args.cam_x,
    )
    depth_left = DepthObstacleDetector(stop_dist_m=args.cam_stop_dist)
    depth_right = DepthObstacleDetector(stop_dist_m=args.cam_stop_dist)

    # Camera-only confidence range is lower than LiDAR, so min_confidence is tuned down.
    controller = PurePursuitController(
        max_linear=args.speed,
        min_confidence=0.12,
    )

    navigator = CamRowNavigator(
        canbus=canbus,
        left_cam=left_cam,
        right_cam=right_cam,
        vis_detector=vis_detector,
        depth_left=depth_left,
        depth_right=depth_right,
        controller=controller,
        auto=args.auto,
        rows=args.rows,
        headland=args.headland,
        acquire_conf=args.acquire_conf,
        acquire_green=args.acquire_green,
        poll_hz=float(args.fps) * 2,
    )
    nav_ref.append(navigator)

    print()
    print("=" * 68)
    print("  Camera-only onion-row follower  (no LiDAR)")
    print(f"  mode    : {'AUTONOMOUS — robot WILL move' if args.auto else 'perception-only (no motion)'}")
    print(f"  rows    : {args.rows}   headland turns: {'on' if args.headland else 'off'}")
    print(f"  speed   : {args.speed:.2f} m/s max")
    print(f"  cameras : left={args.cam_left_id or 'auto'}  right={args.cam_right_id or 'auto'}")
    print(f"  cam fps : {args.fps}   lateral offset: ±{args.cam_x:.3f} m")
    print(f"  stop    : depth obstacle at {args.cam_stop_dist:.1f} m")
    print(f"  acquire : conf ≥ {args.acquire_conf:.2f}  green ≥ {args.acquire_green:.3f}")
    print("  Press Ctrl+C to stop the robot immediately.")
    print("=" * 68)
    print()

    # Confirm canbus link before starting cameras.
    if canbus is not None:
        try:
            await canbus.stop()
            print("[cam_follow] canbus link OK (sent zero twist).")
        except Exception as exc:
            print(f"[cam_follow] canbus unreachable: {exc}")
            if args.auto:
                print("[cam_follow] cannot drive — switching to perception-only.")
                navigator.auto = False

    print("[cam_follow] starting OAK-D cameras …")
    cam_tasks = [
        asyncio.ensure_future(left_cam.run()),
        asyncio.ensure_future(right_cam.run()),
    ]

    # Wait for first frames, then verify at least one camera came up.
    await asyncio.sleep(2.5)
    left_ok = left_cam.get_latest() is not None
    right_ok = right_cam.get_latest() is not None
    if not left_ok and not right_ok:
        print(
            "\n[cam_follow] ERROR: no frames received from either camera after 2.5 s.\n"
            "  Check that the OAK-D devices are physically connected (USB 3.0 required)\n"
            "  and detected by the OS:\n"
            "    lsusb | grep -iE 'movidius|luxonis|03e7'\n"
            "    python3 -c \"import depthai as dai; "
            "print(dai.Device.getAllAvailableDevices())\"\n"
            "  Aborting."
        )
        for t in cam_tasks:
            t.cancel()
        await asyncio.gather(*cam_tasks, return_exceptions=True)
        return
    if not left_ok:
        print("[cam_follow] WARNING: left camera not producing frames — running right-only.")
    if not right_ok:
        print("[cam_follow] WARNING: right camera not producing frames — running left-only.")
    print("[cam_follow] cameras ready — following the centre crop row.\n")

    try:
        await navigator.run()
    finally:
        for t in cam_tasks:
            t.cancel()
        await asyncio.gather(*cam_tasks, return_exceptions=True)


# ---------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Camera-only autonomous onion-row following (no LiDAR)"
    )
    parser.add_argument("--auto", action="store_true",
                        help="Enable motion (default: perception-only)")
    parser.add_argument("--rows", type=int, default=1, metavar="N",
                        help="Number of rows to cover (default: 1)")
    parser.add_argument("--headland", action="store_true",
                        help="Perform open-loop headland turns between rows")
    parser.add_argument("--speed", type=float, default=0.20, metavar="M",
                        help="Max forward speed m/s (default: 0.20 — "
                             "lower than LiDAR mode, camera perception has more latency)")
    parser.add_argument("--cam-left-id", type=str, default="", metavar="SERIAL",
                        help="Left OAK-D device serial/MXID (empty = auto-detect)")
    parser.add_argument("--cam-right-id", type=str, default="", metavar="SERIAL",
                        help="Right OAK-D device serial/MXID (empty = auto-detect)")
    parser.add_argument("--cam-x", type=float, default=0.915, metavar="M",
                        help="Camera lateral offset from robot centreline (default: 0.915)")
    parser.add_argument("--cam-stop-dist", type=float, default=1.2, metavar="M",
                        help="Depth obstacle stop distance in metres (default: 1.2)")
    parser.add_argument("--acquire-conf", type=float, default=0.20, metavar="F",
                        help="Min visual confidence to leave ACQUIRE (default: 0.20)")
    parser.add_argument("--acquire-green", type=float, default=0.08, metavar="F",
                        help="Min green fraction to leave ACQUIRE (default: 0.08)")
    parser.add_argument("--fps", type=int, default=10, metavar="N",
                        help="OAK-D capture frame rate (default: 10)")
    args = parser.parse_args()

    nav_ref: list = []
    try:
        asyncio.run(_run(args, nav_ref))
    except KeyboardInterrupt:
        pass

    print("\n[cam_follow] stopping robot …")
    if nav_ref:
        try:
            asyncio.run(nav_ref[0].stop())
        except Exception:
            pass
    print("[cam_follow] done.")


if __name__ == "__main__":
    main()
