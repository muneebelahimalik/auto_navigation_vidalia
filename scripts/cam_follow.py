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
    --cam-left-id S   left OAK-D farm-ng service name (default: oak0)
    --cam-right-id S  right OAK-D farm-ng service name (default: oak1)
    --cam-x M         camera lateral offset from centreline (default: 0.915)
    --cam-stop-dist M depth obstacle stop distance (default: 2.5)
    --acquire-conf F  min confidence to leave ACQUIRE (default: 0.20)
    --fps N           camera capture rate (default: 10)
    --detector MODE   row detection: hsv (default) or depth-edge (colour-independent)
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
from camera.row_detector_depth_edge import DepthEdgeRowDetector
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

    if args.dual_row:
        # Soybean centre-residue mode: each camera tracks the flanking row on
        # its own side; the residue-strip centre is their midpoint.
        from camera.soybean_row_tracker import DualCameraRowTracker
        vis_detector = DualCameraRowTracker(
            cam_x_left=-args.cam_x,
            cam_x_right=args.cam_x,
            row_spacing=args.row_spacing,
            green_h_lo=args.hsv_h_lo,
            green_h_hi=args.hsv_h_hi,
            green_s_lo=args.hsv_s_lo,
            green_v_lo=args.hsv_v_lo,
        )
    elif args.detector == "depth-edge":
        vis_detector = DepthEdgeRowDetector(
            cam_x_left=-args.cam_x,
            cam_x_right=args.cam_x,
        )
    else:
        vis_detector = VisualRowDetector(
            cam_x_left=-args.cam_x,
            cam_x_right=args.cam_x,
            green_h_lo=args.hsv_h_lo,
            green_h_hi=args.hsv_h_hi,
            green_s_lo=args.hsv_s_lo,
            green_v_lo=args.hsv_v_lo,
        )
    # In dual-row soybean mode the crop rows themselves are always within 2.5 m of
    # the forward-facing cameras; depth obstacle checking would permanently block.
    # Disable it — safety comes from slow speed and user supervision.
    # In single-row mode, both cameras are forward-facing so col_centre_frac=0.5.
    if args.dual_row:
        depth_left = None
        depth_right = None
    else:
        depth_left = DepthObstacleDetector(
            stop_dist_m=args.cam_stop_dist,
            col_centre_frac=0.5,
        )
        depth_right = DepthObstacleDetector(
            stop_dist_m=args.cam_stop_dist,
            col_centre_frac=0.5,
        )

    # Camera-only confidence range is lower than LiDAR, so min_confidence is tuned down.
    controller = PurePursuitController(
        max_linear=args.speed,
        min_confidence=0.12,
    )

    # Wheel odometry for accurate row distance + closed-loop headland turns.
    # Reuses the canbus config for measured wheel speed when present; otherwise
    # dead-reckons from commanded velocity.
    from navigation.odometry import WheelOdometry
    canbus_config = canbus.config if canbus is not None else None
    odometry = WheelOdometry(canbus_config)

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
        cam_block_frames=args.cam_block_frames,
        row_spacing=args.row_spacing,
        headland_exit_dist=args.headland_exit,
        first_turn_sign=(1.0 if args.turn_dir == "right" else -1.0),
        headland_speed=args.headland_speed,
        headland_turn_rate=args.headland_turn_rate,
        odometry=odometry,
    )
    nav_ref.append(navigator)

    print()
    print("=" * 68)
    print("  Camera-only crop-row follower  (no LiDAR)")
    print(f"  mode     : {'AUTONOMOUS — robot WILL move' if args.auto else 'perception-only (no motion)'}")
    det_str = "dual-flanking-row (soybean)" if args.dual_row else args.detector
    print(f"  detector : {det_str}")
    print(f"  rows     : {args.rows}   headland turns: {'on' if args.headland else 'off'}")
    if args.headland:
        print(f"  headland : U-turn first={args.turn_dir}  row_spacing={args.row_spacing:.2f}m  "
              f"exit={args.headland_exit:.2f}m")
    print(f"  speed    : {args.speed:.2f} m/s max")
    print(f"  cameras  : left={args.cam_left_id or 'auto'}  right={args.cam_right_id or 'auto'}")
    print(f"  cam fps  : {args.fps}   lateral offset: ±{args.cam_x:.3f} m")
    print(f"  stop     : depth obstacle at {args.cam_stop_dist:.1f} m")
    print(f"  acquire  : conf ≥ {args.acquire_conf:.2f}  green ≥ {args.acquire_green:.3f}  frames=5")
    if args.detector == "hsv":
        print(f"  HSV      : h=[{args.hsv_h_lo},{args.hsv_h_hi}] s≥{args.hsv_s_lo} v≥{args.hsv_v_lo}")
    print("  Press Ctrl+C to stop the robot immediately.")
    print("=" * 68)
    print()

    # Confirm canbus link before starting cameras.
    if canbus is not None:
        try:
            await asyncio.wait_for(canbus.stop(), timeout=4.0)
            print("[cam_follow] canbus link OK (sent zero twist).")
        except asyncio.TimeoutError:
            print("[cam_follow] canbus check timed out (no response in 4 s).")
            if args.auto:
                print("[cam_follow] cannot drive — switching to perception-only.")
                navigator.auto = False
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
    if canbus_config is not None:
        cam_tasks.append(asyncio.ensure_future(odometry.run()))

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
                        help="Perform closed-loop headland U-turns between rows")
    parser.add_argument("--dual-row", action="store_true", default=False,
                        help="Soybean centre-residue mode: each camera tracks the "
                             "flanking row on its own side; the residue-strip centre "
                             "is their midpoint (overrides --detector).")
    parser.add_argument("--row-spacing", type=float, default=0.76, metavar="M",
                        help="Soybean row spacing / next-strip lateral distance (default: 0.76).")
    parser.add_argument("--turn-dir", choices=["right", "left"], default="right",
                        help="Direction of the first headland U-turn (default: right; "
                             "subsequent turns alternate).")
    parser.add_argument("--headland-exit", type=float, default=1.0, metavar="M",
                        help="Straight distance (m) past the row end before the first pivot (default: 1.0).")
    parser.add_argument("--headland-speed", type=float, default=0.12, metavar="M",
                        help="Forward speed (m/s) during straight headland phases (default: 0.12).")
    parser.add_argument("--headland-turn-rate", type=float, default=0.35, metavar="R",
                        help="Pivot rate (rad/s) during the 90° headland turns (default: 0.35).")
    parser.add_argument("--speed", type=float, default=0.20, metavar="M",
                        help="Max forward speed m/s (default: 0.20 — "
                             "lower than LiDAR mode, camera perception has more latency)")
    parser.add_argument("--cam-left-id", type=str, default="", metavar="SERVICE",
                        help="Left OAK-D farm-ng service name, e.g. oak0 (empty = oak0)")
    parser.add_argument("--cam-right-id", type=str, default="", metavar="SERVICE",
                        help="Right OAK-D farm-ng service name, e.g. oak1 (empty = oak1)")
    parser.add_argument("--cam-x", type=float, default=0.88, metavar="M",
                        help="Camera lateral offset from robot centreline (default: 0.88 — half of 1.76 m inter-camera span; measured)")
    parser.add_argument("--cam-stop-dist", type=float, default=2.5, metavar="M",
                        help="Depth obstacle stop distance in metres (default: 2.5)")
    parser.add_argument("--cam-block-frames", type=int, default=3, metavar="N",
                        help="Consecutive camera-blocked frames required to trigger "
                             "OBSTACLE_WAIT (default: 3)")
    parser.add_argument("--acquire-conf", type=float, default=0.20, metavar="F",
                        help="Min visual confidence to leave ACQUIRE (default: 0.20)")
    parser.add_argument("--acquire-green", type=float, default=0.05, metavar="F",
                        help="Min green fraction to leave ACQUIRE (default: 0.05)")
    parser.add_argument("--fps", type=int, default=10, metavar="N",
                        help="OAK-D capture frame rate (default: 10)")
    parser.add_argument("--detector", choices=["hsv", "depth-edge"], default="hsv",
                        help="Row detection algorithm: 'hsv' = HSV green-centroid (default), "
                             "'depth-edge' = Canny/Hough heading + stereo-depth gap lateral "
                             "(colour-independent, works in any lighting)")
    parser.add_argument("--hsv-h-lo", type=int, default=35, metavar="H",
                        help="HSV hue lower bound 0-180 (default: 35 = green; "
                             "use ~10 for brown/cardboard, ~100 for blue)")
    parser.add_argument("--hsv-h-hi", type=int, default=85, metavar="H",
                        help="HSV hue upper bound 0-180 (default: 85)")
    parser.add_argument("--hsv-s-lo", type=int, default=25, metavar="S",
                        help="HSV saturation lower bound 0-255 (default: 25; "
                             "lower for pale/young soybean seedlings)")
    parser.add_argument("--hsv-v-lo", type=int, default=30, metavar="V",
                        help="HSV value lower bound 0-255 (default: 30)")
    args = parser.parse_args()

    import logging
    # Silence the asyncio ERROR logs from gRPC's background polling thread
    # trying to post to the event loop after it closes.
    logging.getLogger("asyncio").setLevel(logging.CRITICAL)

    nav_ref: list = []
    import signal
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(_run(args, nav_ref))
    except KeyboardInterrupt:
        # Ignore further SIGINT during cleanup so a double Ctrl+C doesn't
        # raise KeyboardInterrupt inside cv2.imdecode (a blocking C call)
        # and bypass the CancelledError handlers.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        pending = asyncio.all_tasks(loop)
        for t in pending:
            t.cancel()
        if pending:
            try:
                loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
            except BaseException:
                pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        loop.close()

    print("\n[cam_follow] stopping robot …")
    if nav_ref:
        try:
            loop2 = asyncio.new_event_loop()
            loop2.run_until_complete(nav_ref[0].stop())
            loop2.close()
        except Exception:
            pass
    print("[cam_follow] done.")


if __name__ == "__main__":
    main()
