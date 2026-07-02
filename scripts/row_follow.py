#!/usr/bin/env -S python3 -u
"""
row_follow.py — Autonomous crop-row following on the Amiga brain (no ROS 2).

Runs entirely in the native Python stack: VLP-16 UDP -> LiDAR row
perception -> pure-pursuit control -> Twist2d over the Amiga canbus.

The robot detects and follows the centre line ahead of it in real time.
Two detection modes:

  Default (single-row): robot is centred over ONE raised crop row, e.g.
    onion beds.  The LiDAR detects the canopy directly under the robot.

  --dual-row: robot straddles a CENTRE RESIDUE STRIP flanked by crop rows on
    each side, e.g. soybean fields.  The LiDAR detects the soybean rows on
    both sides; the midpoint between them is the tracking target.
    Use with --crop-min 0.03 --crop-max 0.30 --obstacle-height 0.50.

No pre-built map and no survey are required — this is online, perception-
driven row following.

Usage (on the Amiga brain):
    source /farm_ng_image/venv/bin/activate
    cd ~/auto_navigation_vidalia

    # 1. PERCEPTION-ONLY — robot does not move; verify detection first:
    python3 scripts/row_follow.py

    # 2. AUTONOMOUS — soybean field, centre residue strip:
    python3 scripts/row_follow.py --auto --dual-row

    # 3. AUTONOMOUS — onion field, single crop row:
    python3 scripts/row_follow.py --auto --crop-max 0.60 --obstacle-height 0.75 --tire-height 0.85

Press Ctrl+C at any time to stop the robot immediately.

Key flags:
    --auto            enable motion (default: perception-only, no motion)
    --dual-row        soybean / centre-residue mode (midpoint of flanking rows)
    --rows N          number of rows to cover (default: 1)
    --headland        perform open-loop headland turns between rows
    --speed M         max forward speed m/s (default: 0.30)
    --no-validate     skip the LiDAR startup health check
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent.parent))

# Auto-add the farm-ng venv site-packages so the script works without
# needing "source /farm_ng_image/venv/bin/activate" first.
_FARMNG_SITE = Path("/farm_ng_image/venv/lib/python3.8/site-packages")
if _FARMNG_SITE.exists() and str(_FARMNG_SITE) not in sys.path:
    sys.path.insert(0, str(_FARMNG_SITE))

from lidar.lidar_driver import LidarDriver
from lidar.obstacle_filter import validate_lidar_startup, tilt_correct_pts
from navigation.row_controller import PurePursuitController
from navigation.row_navigator import RowNavigator
from navigation.row_perception import RowDetector
from navigation.row_safety import SafetyMonitor


# ---------------------------------------------------------------------------
# Canbus config
# ---------------------------------------------------------------------------

def _load_canbus_interface():
    """
    Load the canbus EventServiceConfig and build a CanbusInterface.

    Returns None if service_config.json or the farm-ng SDK is unavailable —
    the script then runs perception-only (it cannot command the robot).
    """
    config_path = Path(__file__).parent.parent / "service_config.json"
    if not config_path.exists():
        print(f" [row_follow] service_config.json not found at {config_path}")
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
        print(" [row_follow] no 'canbus' service in service_config.json")
    except Exception as exc:
        print(f" [row_follow] could not load canbus config: {exc}")
    return None


def _load_filter_config():
    """Return the 'filter' EventServiceConfig (GPS+IMU heading) or None.

    Used as the absolute-heading source for headland pivots; absent → the turn
    falls back to wheel-odometry heading.
    """
    config_path = Path(__file__).parent.parent / "service_config.json"
    if not config_path.exists():
        return None
    try:
        from google.protobuf import json_format
        from farm_ng.core.event_service_pb2 import EventServiceConfigList

        with open(config_path) as f:
            raw = json.load(f)
        config_list = json_format.ParseDict(raw, EventServiceConfigList())
        for cfg in config_list.configs:
            if cfg.name == "filter":
                return cfg
    except Exception as exc:
        print(f" [row_follow] could not load filter config: {exc}")
    return None


# ---------------------------------------------------------------------------
# Diagnostic loop
# ---------------------------------------------------------------------------

async def _debug_loop(lidar: LidarDriver, args: argparse.Namespace) -> None:
    """Stream LiDAR stats and save a bird's-eye-view PNG for diagnosis."""
    import math
    import time

    import numpy as np

    from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT

    print(f" DEBUG — collecting scans (mount height {LIDAR_MOUNT_HEIGHT:.3f} m, "
          f"tilt {args.lidar_tilt:.1f}° forward)")
    print(f" self-filter < {args.self_radius:.2f} m | "
          f"ROI |x| <= {args.roi_x:.2f} m, y 1.5-7.0 m")
    print(" A healthy run shows steady 'pts' (~28000) and 'hz' near 10.\n")

    h_edges = np.arange(-0.30, 1.05, 0.10)
    collected = []
    target = 25
    t0 = time.monotonic()
    count = 0
    hz = 0.0

    tilt_rad = math.radians(args.lidar_tilt)
    async for pts in lidar.scan_stream_np():
        count += 1
        now = time.monotonic()
        if now - t0 >= 1.0:
            hz = count / (now - t0)
            count = 0
            t0 = now
        if len(pts) == 0:
            continue

        rng = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[rng >= args.self_radius]
        if tilt_rad != 0.0:
            pts = tilt_correct_pts(pts, tilt_rad)
        strip = pts[(pts[:, 1] >= 1.5) & (pts[:, 1] <= 7.0)
                    & (np.abs(pts[:, 0]) <= args.roi_x)]
        h = strip[:, 2] + LIDAR_MOUNT_HEIGHT
        band = int(((h >= args.crop_min) & (h <= args.crop_max)).sum())
        hh, _ = np.histogram(h, bins=h_edges)
        bars = " ".join(f"{h_edges[i]:+.2f}:{int(hh[i]):<4d}"
                        for i in range(len(hh)) if hh[i] > 0)
        print(f"pts={len(pts):6d} hz={hz:4.1f} strip={len(strip):5d} "
              f"crop[{args.crop_min:.2f}-{args.crop_max:.2f}]={band}")
        if bars:
            print(f"   h-hist: {bars}")

        collected.append(pts[(pts[:, 1] >= 0.0) & (pts[:, 1] <= 9.0)
                              & (np.abs(pts[:, 0]) <= 2.5)])
        if len(collected) >= target:
            break

    if not collected:
        print("\n no scans collected — check the LiDAR connection")
        return
    _save_bev(np.vstack(collected), args)


def _height_colour(t):
    """Map normalised height t in [0,1] to an RGB uint8 array (blue -> red)."""
    import numpy as np

    t = np.clip(np.asarray(t, dtype=np.float64), 0.0, 1.0)
    stops = np.array([
        [0.00,  40,  60, 180],
        [0.25,  40, 180, 200],
        [0.50,  60, 200,  70],
        [0.75, 240, 220,  50],
        [1.00, 220,  50,  40],
    ])
    r = np.interp(t, stops[:, 0], stops[:, 1])
    g = np.interp(t, stops[:, 0], stops[:, 2])
    b = np.interp(t, stops[:, 0], stops[:, 3])
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


def _save_bev(pts, args: argparse.Namespace) -> None:
    """Render a top-down view of the forward LiDAR points, coloured by height."""
    import numpy as np

    from lidar.obstacle_filter import LIDAR_MOUNT_HEIGHT
    from slam.map_io import _write_png

    res = 0.015
    x_min, x_max = -2.5, 2.5
    y_min, y_max = 0.0, 9.0
    h_lo, h_hi = -0.20, 1.20
    bar_w = 16

    plot_w = int((x_max - x_min) / res)
    H = int((y_max - y_min) / res)
    W = plot_w + bar_w
    img = np.full((H, W, 3), 245, dtype=np.uint8)

    # faint 1 m grid
    for xx in np.arange(x_min + 1.0, x_max, 1.0):
        img[:, int((xx - x_min) / res)] = (215, 215, 215)
    for yy in np.arange(y_min + 1.0, y_max, 1.0):
        img[int((y_max - yy) / res), :plot_w] = (215, 215, 215)

    # points, coloured by height above ground
    keep = ((pts[:, 0] >= x_min) & (pts[:, 0] < x_max)
            & (pts[:, 1] >= y_min) & (pts[:, 1] < y_max))
    p = pts[keep]
    h = p[:, 2] + LIDAR_MOUNT_HEIGHT
    rgb = _height_colour((h - h_lo) / (h_hi - h_lo))
    col = np.clip(((p[:, 0] - x_min) / res).astype(int), 0, plot_w - 1)
    row = np.clip(((y_max - p[:, 1]) / res).astype(int), 0, H - 1)
    for dr in (0, 1):
        for dc in (0, 1):
            img[np.clip(row + dr, 0, H - 1),
                np.clip(col + dc, 0, plot_w - 1)] = rgb

    # row-detection ROI box and robot centreline
    c0 = int((-args.roi_x - x_min) / res)
    c1 = int((args.roi_x - x_min) / res)
    r0 = int((y_max - 7.0) / res)
    r1 = int((y_max - 1.5) / res)
    img[r0:r1, c0] = (0, 0, 0)
    img[r0:r1, c1] = (0, 0, 0)
    img[r0, c0:c1] = (0, 0, 0)
    img[r1, c0:c1] = (0, 0, 0)
    img[:, int((0.0 - x_min) / res)] = (120, 120, 120)

    # colour-scale bar (top = high, bottom = low)
    img[:, plot_w:] = _height_colour(np.linspace(1.0, 0.0, H))[:, None, :]

    out = Path("row_debug_bev.png")
    _write_png(out, img)
    print(f"\n Saved bird's-eye view -> {out.resolve()}  ({W}x{H} px)")
    print(f" Colour = height above ground: blue {h_lo:+.1f} m -> red {h_hi:+.1f} m.")
    print(" Robot is at bottom-centre looking up the image (+y = forward).")
    print(" Send me row_debug_bev.png and I can set the crop band precisely.")


# ---------------------------------------------------------------------------
# Main async loop
# ---------------------------------------------------------------------------

async def _run(args: argparse.Namespace, nav_ref: list) -> None:
    if args.debug:
        async with LidarDriver() as lidar:
            await _debug_loop(lidar, args)
        return

    canbus = _load_canbus_interface()

    if args.auto and canbus is None:
        print("\n[row_follow] --auto requested but no canbus available — "
              "falling back to perception-only mode.\n")
        args.auto = False

    slam = None
    if args.slam:
        # Optional field mapping alongside driving (added functionality, never
        # commands the wheels).  Runs in a separate thread fed by the navigator
        # so it cannot affect control timing; uses the SAME mount calibration as
        # the perception stack.
        from slam.slam_engine import SlamEngine
        from slam.slam_runner import SlamRunner
        slam_engine = SlamEngine(
            yaw_deg=args.lidar_yaw, tilt_deg=args.lidar_tilt,
            build_3d=args.map_3d, voxel_3d=args.voxel_3d,
            swath_m=args.swath,
        )
        slam = SlamRunner(slam_engine)
        slam.start()
        print(f" [row_follow] SLAM mapping ON (thread){'  +3D point cloud' if args.map_3d else ''}"
              f" — coverage + map saved on exit to {args.save_dir or 'maps/<ts>'}")

    left_cam = None
    right_cam = None
    vis_detector = None
    depth_left = None
    depth_right = None
    depth_pts_left = None
    depth_pts_right = None
    ekf = None
    odometry = None
    if args.camera:
        import math as _math2
        from camera.oak_driver import OakDriver
        left_cam = OakDriver(side="left", device_id=args.cam_left_id)
        right_cam = OakDriver(side="right", device_id=args.cam_right_id)

        if args.dual_row:
            # Soybean centre-residue mode: both forward-facing cameras see
            # BOTH flanking rows; each projects its green mask onto the
            # ground plane and independently estimates the residue-strip
            # centre.  The fused centre is the mean of the two estimates.
            from camera.soybean_row_tracker import DualCameraRowTracker
            vis_detector = DualCameraRowTracker(
                cam_x_left=-args.cam_x,
                cam_x_right=args.cam_x,
                row_spacing=args.row_spacing,
                roi_x_half=args.roi_x,
                cam_y_fwd=args.cam_y_fwd,
                cam_z=args.cam_height,
                cam_pitch_deg=args.cam_pitch_deg,
                canopy_z=args.cam_canopy_z,
            )
        else:
            # Single-row mode: forward green-centroid detector (onion / raised bed).
            from camera.row_detector_visual import VisualRowDetector
            vis_detector = VisualRowDetector(
                cam_x_left=-args.cam_x,
                cam_x_right=args.cam_x,
            )

        if args.no_cam_obstacles:
            pass  # cameras used for row tracking only; LiDAR handles obstacle detection
        elif args.cam_depth_3d:
            # Full 3-D depth fusion: project camera depth images to LiDAR-convention
            # point clouds and merge with LiDAR before SafetyMonitor.check().
            # Replaces the legacy 1-D strip DepthObstacleDetector.
            from camera.depth_to_points import DepthToPoints
            cam_yaw_rad = _math2.radians(args.cam_yaw_deg)
            cam_pitch_rad = _math2.radians(args.cam_pitch_deg)
            depth_pts_left = DepthToPoints(
                cam_x=-args.cam_x,
                cam_y_fwd=args.cam_y_fwd,
                cam_z=args.cam_height,
                cam_yaw=-cam_yaw_rad,   # left cam: negative = rotated left of forward axis
                cam_pitch=cam_pitch_rad,
                y_max=args.cam_stop_dist + 0.5,
            )
            depth_pts_right = DepthToPoints(
                cam_x=+args.cam_x,
                cam_y_fwd=args.cam_y_fwd,
                cam_z=args.cam_height,
                cam_yaw=+cam_yaw_rad,   # right cam: positive = rotated right of forward axis
                cam_pitch=cam_pitch_rad,
                y_max=args.cam_stop_dist + 0.5,
            )
        else:
            # 1-D depth strip obstacle detection (forward-facing cameras).
            # Disabled automatically in soybean/dual-row fields via --no-cam-obstacles
            # because the crop rows themselves appear as obstacles at 1–4 m range.
            from camera.depth_obstacle import DepthObstacleDetector
            depth_left = DepthObstacleDetector(
                stop_dist_m=args.cam_stop_dist,
                min_dist_m=args.cam_near_filter,
                col_centre_frac=0.5,
            )
            depth_right = DepthObstacleDetector(
                stop_dist_m=args.cam_stop_dist,
                min_dist_m=args.cam_near_filter,
                col_centre_frac=0.5,
            )

    if args.cam_fusion == "point" and not (args.camera and args.dual_row):
        print(" [row_follow] --cam-fusion point requires --camera --dual-row "
              "(the ground-projection tracker provides the metric points) — "
              "falling back to estimate-level fusion.")
        args.cam_fusion = "estimate"

    if args.ekf:
        from navigation.ekf_estimator import RowEKF
        ekf = RowEKF()

    # Wheel odometry: drives accurate FOLLOW row-distance and closed-loop
    # headland turns.  Reuses the canbus EventServiceConfig to subscribe to
    # measured wheel speed (AmigaTpdo1); falls back to commanded-velocity
    # dead-reckoning when telemetry is unavailable (e.g. perception-only).
    from navigation.odometry import WheelOdometry
    canbus_config = canbus.config if canbus is not None else None
    odometry = WheelOdometry(canbus_config)

    # Absolute heading (GPS+IMU) for accurate headland pivots; None → the turn
    # falls back to wheel-odometry heading.
    from navigation.filter_heading import FilterHeading
    filter_config = _load_filter_config()
    filter_heading = FilterHeading(filter_config) if filter_config is not None else None

    detector = RowDetector(
        roi_x_half=args.roi_x,
        crop_h_min=args.crop_min,
        crop_h_max=args.crop_max,
        dual_row=args.dual_row,
        row_spacing=args.row_spacing,
        ground_detrend=not args.no_ground_detrend,
    )
    tire_h = args.tire_height if args.tire_height is not None else args.obstacle_height
    safety = SafetyMonitor(
        obstacle_height=args.obstacle_height,
        tire_obstacle_height=tire_h,
        forward_dist=args.forward_dist,
        forward_half_width=args.forward_half_width,
        tire_track=args.tire_track,
    )
    if args.controller == "rl":
        # Optional learned steering (opt-in). Falls back to pure-pursuit when the
        # row fix is weak or no policy is loaded — never replaces safety/state.
        from navigation.rl_controller import RLController
        from navigation.rl_policy import MLPPolicy
        # Default to the smoothest sweep policy (as smooth as MPC, tighter
        # tracking) when no --policy is given, so `--controller rl` works
        # out of the box.
        policy_path = args.policy or "policies/follow_jerk8.0.npz"
        pol = None
        if policy_path and Path(policy_path).exists():
            pol = MLPPolicy.load(policy_path)
            default_tag = "" if args.policy else " (default)"
            print(f" [row_follow] RL steering policy: {policy_path}{default_tag} "
                  f"({pol.n_params} params)")
        else:
            print(f" [row_follow] --controller rl but no policy at '{policy_path}' "
                  f"— running pure-pursuit fallback.")
        controller = RLController(
            policy=pol, max_linear=args.speed,
            lookahead=args.lookahead, max_angular=args.max_angular)
    elif args.controller == "mpc":
        # Optional sampling-based MPC (MPPI) with an online disturbance observer
        # for slope-drift rejection (opt-in). Falls back to pure-pursuit when the
        # row fix is weak — never replaces the safety/state architecture.
        from navigation.row_mpc_controller import RowMPCController
        print(" [row_follow] MPC steering: MPPI + online disturbance observer")
        controller = RowMPCController(
            max_linear=args.speed,
            lookahead=args.lookahead, max_angular=args.max_angular)
    else:
        controller = PurePursuitController(
            max_linear=args.speed,
            lookahead=args.lookahead,
            max_angular=args.max_angular,
        )
    telemetry = None
    if args.telemetry is not None:
        from navigation.telemetry import TelemetryLogger
        tpath = (str(Path("logs") / f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl")
                 if args.telemetry == "auto" else args.telemetry)
        telemetry = TelemetryLogger(tpath)
        print(f" [row_follow] telemetry log: {tpath}" if telemetry.enabled
              else f" [row_follow] telemetry log could not be opened ({tpath}) — continuing")

    import math as _math
    navigator = RowNavigator(
        canbus, detector, safety, controller,
        auto=args.auto, rows=args.rows, headland=args.headland, slam=slam,
        telemetry=telemetry,
        self_radius=args.self_radius,
        acquire_conf=args.acquire_conf,
        row_end_min_dist=args.row_end_min_dist,
        row_spacing=args.row_spacing,
        headland_exit_dist=args.headland_exit,
        first_turn_sign=(1.0 if args.turn_dir == "right" else -1.0),
        headland_speed=args.headland_speed,
        headland_turn_rate=args.headland_turn_rate,
        headland_radius=args.headland_radius,
        turn_scrub_comp=args.turn_scrub_comp,
        reacquire_conf=args.reacquire_conf,
        approach_speed=args.approach_speed,
        approach_max_dist=args.approach_max_dist,
        post_turn_max_dist=args.post_turn_max_dist,
        heading_source=filter_heading,
        align_heading=args.align_heading,
        align_rate=args.align_speed,
        left_cam=left_cam,
        right_cam=right_cam,
        vis_detector=vis_detector,
        depth_left=depth_left,
        depth_right=depth_right,
        depth_pts_left=depth_pts_left,
        depth_pts_right=depth_pts_right,
        ekf=ekf,
        odometry=odometry,
        tilt_rad=_math.radians(args.lidar_tilt),
        yaw_rad=_math.radians(args.lidar_yaw),
        cam_block_frames=args.cam_block_frames,
        cam_self_radius=args.cam_self_radius,
        ros2_bridge=args.ros2_bridge,
        cam_fusion=args.cam_fusion,
    )
    nav_ref.append(navigator)
    if args.record is not None:
        navigator.capture_perception = True   # snapshot a real scan for figures
        # Early one-shot render: fire a few scans after a confident FOLLOW lock
        # (off the control loop) so the scan + figures land seconds into the run,
        # not at exit.  run_dir is args.save_dir (set in the --record block).
        _run_dir = args.save_dir
        navigator.perception_ready_cb = (
            lambda cap: _render_perception(_run_dir, cap, when="early"))
        if getattr(args, "_scan_every", 0):
            from navigation.scan_recorder import ScanRecorder
            navigator.scan_recorder = ScanRecorder(args._scan_dir, every=args._scan_every)

    print()
    print("=" * 68)
    det_mode = "dual-row (centre-residue / soybean)" if args.dual_row else "single-row"
    print(f"  Autonomous crop-row follower  [{det_mode}]")
    print(f"  mode    : {'AUTONOMOUS — robot WILL move' if args.auto else 'perception-only (no motion)'}")
    print(f"  rows    : {args.rows}   headland turns: {'on' if args.headland else 'off'}")
    if args.headland:
        radius_str = (f"{args.headland_radius:.2f}m" if args.headland_radius > 0.0
                      else "auto (1.00m)")
        print(f"  headland: IMU/perception arc U-turn first={args.turn_dir} "
              f"(then alternating)  radius={radius_str}  "
              f"exit={args.headland_exit:.2f}m  rate={args.headland_turn_rate:.2f}rad/s  "
              f"reacq-conf={args.reacquire_conf:.2f}  scrub-comp={args.turn_scrub_comp:.2f}")
    print(f"  speed   : {args.speed:.2f} m/s max   SLAM map: {'on' if args.slam else 'off'}")
    print(f"  crop    : h=[{args.crop_min:.2f},{args.crop_max:.2f}]m  roi_x=±{args.roi_x:.2f}m")
    print(f"  safety  : fwd_h={args.obstacle_height:.2f}m  tire_h={tire_h:.2f}m  "
          f"self_r={args.self_radius:.2f}m  tilt={args.lidar_tilt:.1f}°  "
          f"fwd_zone={args.forward_dist:.1f}m×{args.forward_half_width*2:.2f}m")
    align_str = f"align={args.align_speed:.2f}rad/s (on)" if args.align_heading else "align=off"
    print(f"  control : lookahead={args.lookahead:.1f}m  max_angular={args.max_angular:.2f}rad/s  {align_str}")
    if args.camera:
        row_mode = "dual-flanking-row tracker" if args.dual_row else "green-centroid tracker"
        cam_mode = f"OAK-D left+right ({row_mode})"
        if args.cam_depth_3d:
            cam_mode += f"  3D-fusion(h={args.cam_height}m yaw={args.cam_yaw_deg}° pitch={args.cam_pitch_deg}° self_r={args.cam_self_radius:.1f}m)"
        elif args.no_cam_obstacles:
            cam_mode += "  obstacle=LiDAR-only"
        else:
            cam_mode += "  strip-obstacle(no-height-filter)"
        if not args.no_cam_obstacles:
            cam_mode += f"  stop={args.cam_stop_dist}m"
    else:
        cam_mode = "disabled"
    ekf_str = "EKF=on" if args.ekf else "EKF=off"
    fusion_str = ("fusion=point (pooled LiDAR+camera fit)"
                  if args.cam_fusion == "point" else "fusion=estimate")
    print(f"  cameras : {cam_mode}  {ekf_str}  {fusion_str}")
    if args.ros2_bridge:
        print("  ros2    : bridge ON — writing to /dev/shm/vidalia_pts.bin + vidalia_status.json")
        print("            start Docker bridge:  bash ros2_bridge/start.sh")
    print("  Press Ctrl+C to stop the robot immediately.")
    print("=" * 68)
    print()

    async with LidarDriver() as lidar:
        if not args.no_validate:
            healthy = await validate_lidar_startup(lidar)
            if not healthy:
                print("[row_follow] LiDAR health check failed — aborting.")
                return

        # Confirm the canbus link with a harmless zero-velocity command.
        # The FIRST request_reply also establishes the gRPC channel, which can
        # take well over the 0.5 s default — so retry with a longer timeout
        # before giving up, otherwise a slow channel warm-up silently drops a
        # perfectly good link to perception-only.
        if canbus is not None:
            linked = False
            for attempt in range(1, 4):
                try:
                    await canbus.stop(timeout=2.0)
                    print(" [row_follow] canbus link OK (sent zero twist).")
                    linked = True
                    break
                except asyncio.TimeoutError:
                    print(f" [row_follow] canbus /twist no reply "
                          f"(attempt {attempt}/3) — retrying …")
                    await asyncio.sleep(0.5)
                except Exception as exc:
                    print(f" [row_follow] canbus error: {exc!r}")
                    break
            if not linked and args.auto:
                print(" [row_follow] canbus unreachable — is the Amiga in AUTO "
                      "mode with the e-stop released? Switching to perception-only.")
                navigator.auto = False

        print(" Following the centre crop row — watch the status line.\n")

        cam_tasks = []
        if args.camera:
            cam_tasks.append(asyncio.ensure_future(left_cam.run()))
            cam_tasks.append(asyncio.ensure_future(right_cam.run()))
        # Background wheel-odometry subscription (no-op if telemetry unavailable).
        if odometry is not None and canbus_config is not None:
            cam_tasks.append(asyncio.ensure_future(odometry.run()))
        # Background filter-heading subscription for accurate headland pivots.
        if filter_heading is not None:
            cam_tasks.append(asyncio.ensure_future(filter_heading.run()))
        try:
            await navigator.run(lidar)
        finally:
            for t in cam_tasks:
                t.cancel()
            if cam_tasks:
                try:
                    await asyncio.gather(*cam_tasks, return_exceptions=True)
                except RuntimeError:
                    pass  # gRPC threads may call call_soon_threadsafe on a closing loop


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _render_perception(run_dir, cap, *, when: str) -> None:
    """Save the captured real scan + render the three perception figures (3-D,
    top-down, annotated) into the run folder.  The raw scan is always saved;
    figure rendering needs matplotlib and is skipped with instructions if it is
    not installed.  Never raises.  ``when`` = 'early' (fired a few scans into the
    run, off the control loop) or 'exit' (fallback)."""
    import numpy as np
    d = Path(run_dir)
    pts_full = cap["pts"]
    try:
        np.save(d / "perception_scan.npy", pts_full)   # ALWAYS the full cloud
        (d / "perception_state.json").write_text(json.dumps({
            "lateral": cap["lateral"], "heading_deg": cap["heading_deg"],
            "spacing": cap["spacing"], "confidence": cap["confidence"],
            "n_points": cap["n"]}, indent=2))
    except Exception as exc:
        print(f"[row_follow] could not save perception scan: {exc}")
        return
    # The EARLY render runs in a background thread WHILE the robot drives — on the
    # Jetson, rendering a dense 3-D scatter can hog CPU and stutter the control
    # loop.  Decimate the cloud used for the figures (the FULL cloud is already
    # saved above, so a full-quality figure can be regenerated offline anytime).
    # The exit render (run is over) uses the full cloud.
    pts_render = pts_full
    if when == "early" and len(pts_full) > 6000:
        step = int(len(pts_full) // 6000) + 1
        pts_render = pts_full[::step]
    try:
        from scripts.viz_perception import render_all
        files = render_all(pts_render, cap["lateral"], cap["heading_deg"],
                           cap["spacing"], d)
        print(f"\n[row_follow] perception figures ({len(files)}, {when}): "
              f"{d / 'figure_perception_3d.png'}, _bev.png, _annotated.png")
    except ImportError:
        print(f"\n[row_follow] perception scan saved ({when}) → {d / 'perception_scan.npy'}.  "
              f"matplotlib not installed here; render the 3 figures on a laptop:\n"
              f"    python3 scripts/viz_perception.py --scan {d / 'perception_scan.npy'} "
              f"--telemetry {d / 'telemetry.jsonl'} --mode av3d --out {d / 'figure_perception_3d'}\n"
              f"    (repeat with --mode av_bev and --mode annotated)")
    except Exception as exc:
        print(f"\n[row_follow] perception figure rendering failed: {exc} "
              f"(raw scan saved to {d / 'perception_scan.npy'})")


def _save_perception_figures(run_record, navigator) -> None:
    """Exit-time fallback: render the perception figures only if the early
    one-shot trigger never fired (e.g. the run never reached a confident FOLLOW
    lock)."""
    if getattr(navigator, "perception_fired", False):
        return  # already saved a few seconds into the run
    cap = getattr(navigator, "perception_capture", None)
    if not cap:
        print("[row_follow] perception figures: no confident FOLLOW scan captured — skipped.")
        return
    _render_perception(run_record.dir, cap, when="exit")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Autonomous crop-row following (native stack, no ROS 2)"
    )
    parser.add_argument("--auto", action="store_true",
                        help="Enable motion (default: perception-only)")
    parser.add_argument("--rows", type=int, default=1, metavar="N",
                        help="Number of rows to cover (default: 1)")
    parser.add_argument("--headland", action="store_true",
                        help="Perform closed-loop headland U-turns between rows "
                             "(odometry feedback). Combine with --rows N to cover "
                             "multiple rows in a serpentine pattern.")
    parser.add_argument("--row-spacing", type=float, default=0.76, metavar="M",
                        help="SEED for the in-strip soybean-row separation (default: "
                             "0.76 = 30in standard). This is a detector PRIOR, not a "
                             "movement parameter: it helps pair the two flanking rows "
                             "apart from weed clutter and sets the single-side fallback "
                             "offset. The detector self-calibrates it from the data "
                             "(watch sp= in the status line), so the default is fine for "
                             "most fields — you normally only set --rows.")
    parser.add_argument("--turn-dir", choices=["right", "left"], default="right",
                        help="Direction of the FIRST headland U-turn (default: right). "
                             "Subsequent turns alternate for serpentine coverage.")
    parser.add_argument("--approach-speed", type=float, default=0.12, metavar="M",
                        help="Forward speed (m/s) of the post-turn APPROACH leg that drives "
                             "the robot into the next row until perception re-acquires it "
                             "(default: 0.12).")
    parser.add_argument("--approach-max-dist", type=float, default=3.0, metavar="M",
                        help="Max distance (m) the APPROACH leg drives searching for the next "
                             "row before stopping (field edge / overshoot guard; default: 3.0).")
    parser.add_argument("--post-turn-max-dist", type=float, default=5.0, metavar="M",
                        help="Cumulative distance (m) the robot may travel after a U-turn "
                             "(APPROACH creep + any short FOLLOW segments) without establishing "
                             "a stable down-row FOLLOW before stopping — a hard guard against "
                             "driving off the end of the field (default: 5.0).")
    parser.add_argument("--headland-exit", type=float, default=1.0, metavar="M",
                        help="Straight distance (m) driven past the row end before the "
                             "arc, to clear the body of the last plants AND give the LiDAR "
                             "room to confirm the row really ended (blind inside ~1.5 m); if "
                             "crop reappears during this leg the turn aborts back to FOLLOW "
                             "(default: 1.0).")
    parser.add_argument("--headland-speed", type=float, default=0.15, metavar="M",
                        help="Forward speed (m/s) during the straight headland EXIT phase "
                             "(default: 0.15).")
    parser.add_argument("--headland-turn-rate", type=float, default=0.30, metavar="R",
                        help="Angular rate (rad/s) of the headland U-turn arc "
                             "(default: 0.30). Arc forward speed = radius × this rate.")
    parser.add_argument("--headland-radius", type=float, default=0.0, metavar="M",
                        help="Headland U-turn arc radius (m). Default 0 = auto (1.0 m). "
                             "The turn drives a gentle large-radius arc and keeps arcing "
                             "until the LiDAR sees the next row lined up ahead (it does NOT "
                             "trust wheel heading to count 180°, which a skid-steer "
                             "over-reports). A bigger radius scrubs less and sweeps the next "
                             "row through view more slowly, making the re-lock easier.")
    parser.add_argument("--turn-scrub-comp", type=float, default=0.5, metavar="K",
                        help="Skid-steer scrub compensation for the U-turn rotation "
                             "estimate when the IMU/filter heading is NOT live (status "
                             "shows rot=…[wheel]). Wheel heading over-reports body "
                             "rotation on an arc; the real rotation ≈ wheel × this factor "
                             "(default: 0.5; heavy-slip fields ~0.45). Calibrate: watch the "
                             "rot= readout during a turn — when the robot physically "
                             "completes 180°, rot should read ~180. If it ENDED at real "
                             "~120° when rot read ~159° (wheel 265°), set ~0.45; if rot "
                             "reads LOW when truly turned, raise it. Ignored when rot=…[imu].")
    parser.add_argument("--reacquire-conf", type=float, default=0.72, metavar="C",
                        help="Row-detection confidence (0–1) required — together with small "
                             "heading error and offset, for a few consecutive scans — to end "
                             "the U-turn and enter FOLLOW on the next row (default: 0.72). "
                             "Raise if the turn ends early on a mis-aligned glimpse; lower if "
                             "it keeps arcing past a good lock.")
    parser.add_argument("--slam", action="store_true",
                        help="Build a field map in a background thread while driving "
                             "(mapping only — never commands the wheels; no effect on "
                             "control). Saved on exit (see --save-dir).")
    parser.add_argument("--no-slam", action="store_true",
                        help="Disable SLAM even under --record (skip the map / coverage / "
                             "trajectory outputs). Everything else in the record bundle "
                             "(telemetry, metrics, perception figures, raw scans) is unaffected.")
    parser.add_argument("--telemetry", nargs="?", const="auto", default=None, metavar="PATH",
                        help="Log one JSON line per scan (state, conf, crop points, lateral/"
                             "heading error, row-end conf, grade/drop, command, safety zones, "
                             "headland turn) for offline analysis. Bare --telemetry auto-names "
                             "logs/run_<ts>.jsonl; or give a PATH. Load with "
                             "pandas.read_json(path, lines=True). No effect on control.")
    parser.add_argument("--record", nargs="?", const="auto", default=None, metavar="DIR",
                        help="Record a COMPLETE, reproducible experiment folder for "
                             "publishing: manifest.json (git commit + args + calibration), "
                             "telemetry.jsonl (per-scan), summary.json + metrics_flat.csv + "
                             "per_row.csv + turns.csv (computed performance metrics), and the "
                             "SLAM coverage.png/trajectory.csv/map.png. Bundles --telemetry + "
                             "--slam into runs/run_<ts>/ (or DIR). Analyse with "
                             "scripts/analyze_run.py. Use this on EVERY run you want to analyse.")
    parser.add_argument("--save-scans", nargs="?", type=int, const=1, default=None,
                        metavar="EVERY",
                        help="With --record: also stream the raw corrected LiDAR scans to "
                             "runs/<ts>/scans/ (scan_NNNNN.npy + index.csv) so ANY moment can "
                             "be re-rendered offline into figures/animations later. Bare = every "
                             "scan (full fidelity); N = every Nth scan; 0 = off. "
                             "Default with --record: every 10th (~1 Hz).")
    parser.add_argument("--map-3d", action="store_true",
                        help="With --slam, also accumulate a 3-D point-cloud map "
                             "(map3d.ply + map3d.npz) registered with the SLAM pose.")
    parser.add_argument("--voxel-3d", type=float, default=0.15, metavar="M",
                        help="3-D map voxel size (m) for --map-3d (default: 0.15).")
    parser.add_argument("--swath", type=float, default=1.92, metavar="M",
                        help="With --slam, working width serviced per pass for the "
                             "coverage map (default: 1.92 = Amiga wheel track). Gaps "
                             "in coverage.png are missed rows.")
    parser.add_argument("--speed", type=float, default=0.30, metavar="M",
                        help="Max forward speed in m/s (default: 0.30)")
    parser.add_argument("--roi-x", type=float, default=0.80, metavar="M",
                        help="Half-width of the centre-row search band (default: 0.80). "
                             "Applies to BOTH the LiDAR detector and the dual-camera "
                             "tracker. Must exclude all rows other than the two flanking "
                             "the residue strip — e.g. with inner rows at ±0.30 m and a "
                             "second pair near the tires at ±0.85 m, use 0.55 so the "
                             "outer pair cannot be mispaired with an inner row.")
    parser.add_argument("--crop-min", type=float, default=0.03, metavar="M",
                        help="Min crop height above ground to keep (default: 0.03). "
                             "Soybean seedlings: 0.03. Mature onions: 0.05.")
    parser.add_argument("--crop-max", type=float, default=0.30, metavar="M",
                        help="Max crop height above ground to keep (default: 0.30). "
                             "Soybean seedlings: 0.30. Mature onions: 0.60.")
    parser.add_argument("--dual-row", action="store_true", default=False,
                        help="Dual-row / centre-residue detection mode (default: off). "
                             "Use for soybean fields where the robot straddles a dark centre "
                             "residue strip flanked by soybean rows on each side. The detector "
                             "finds the midpoint between the nearest left and right crop-band "
                             "histogram peaks, which is the centre of the residue strip. "
                             "Falls back to nearest-peak if only one side is visible (large offset). "
                             "In single-row mode (default) the nearest peak to the robot centreline "
                             "is used — correct for onion/raised-bed crops directly under the robot.")
    parser.add_argument("--preset", choices=["soybean", "onion"], default=None,
                        help="Apply a predefined parameter set for common crop types. "
                             "Overrides individual flags with field-tested defaults. "
                             "'soybean': dual-row mode, crop_h=[0.03,0.30]m, "
                             "obstacle_height=0.50m, tire_height=0.65m, row_spacing=0.76m. "
                             "'onion': single-row mode, crop_h=[0.05,0.60]m, "
                             "obstacle_height=0.75m, tire_height=0.85m. "
                             "Any explicit flags after --preset still override the preset.")
    parser.add_argument("--self-radius", type=float, default=1.5, metavar="M",
                        help="Discard LiDAR returns within this radius — the "
                             "robot's own frame (default: 1.5)")
    parser.add_argument("--acquire-conf", type=float, default=0.35, metavar="F",
                        help="Min row-detection confidence (0-1) to leave ACQUIRE "
                             "(default: 0.35). Lower threshold suits soybean seedlings "
                             "which give sparser LiDAR returns than mature onion canopy.")
    parser.add_argument("--obstacle-height", type=float, default=0.50, metavar="M",
                        help="Min ground-relative height (m) to count as obstacle "
                             "in the FORWARD zone (default: 0.50). "
                             "Soybean seedlings are h≤0.30 m so 0.50 passes plants but "
                             "stops humans/posts. Onion fields: use 0.75 m.")
    parser.add_argument("--tire-track", type=float, default=0.959, metavar="M",
                        help="Distance from robot centreline to wheel centre (default: 0.959 — "
                             "half of 75.5 in / 1.9177 m wheel-centre-to-wheel-centre). Sets "
                             "the lateral position of the L-TIRE / R-TIRE safety zones.")
    parser.add_argument("--tire-height", type=float, default=0.65, metavar="M",
                        help="Min ground-relative height (m) for TIRE-ZONE obstacles "
                             "(default: 0.65). Soybean fields with dried residue stalks: "
                             "0.35–0.50 m catches crop material; 0.65 m passes residue "
                             "and seedlings while still stopping real hazards (posts, animals). "
                             "Onion fields: use 0.85 m.")
    parser.add_argument("--no-ground-detrend", action="store_true",
                        help="Disable the terrain-adaptive crop band. By default the detector "
                             "estimates the forward ground grade each scan and removes its slope "
                             "so crop stays inside the height band on sloped/undulating fields; "
                             "pass this to revert to a fixed flat-ground height band.")
    parser.add_argument("--lidar-tilt", type=float, default=21.5, metavar="DEG",
                        help="Forward (nose-down) PITCH correction of the LiDAR mount in degrees, "
                             "applied AFTER --lidar-yaw (default: 21.5 — field-calibrated via "
                             "diag_birdseye.py --tilt-sweep: the 15° body lean resolves into a "
                             "~21.5° robot-frame pitch once the 71° yaw is corrected; this is the "
                             "angle that flattens the forward ground ramp).")
    parser.add_argument("--lidar-yaw", type=float, default=66.0, metavar="DEG",
                        help="Yaw of the LiDAR mount relative to robot forward, CCW positive "
                             "(degrees, default: 66 — re-calibrated 2026-06 after a mount "
                             "disturbance; was 71. Verify with diag_birdseye.py object locator "
                             "if the mount is touched). Applied FIRST (after self-filter, before "
                             "tilt) so the tilt then rotates the pitch about the robot's "
                             "left-right axis.")
    parser.add_argument("--debug", action="store_true",
                        help="Stream a LiDAR height profile instead of navigating")
    parser.add_argument("--save-dir", default="", metavar="PATH",
                        help="Map output directory (with --slam)")
    parser.add_argument("--no-validate", action="store_true",
                        help="Skip the LiDAR startup health check")
    parser.add_argument("--camera", action="store_true",
                        help="Enable OAK-D camera integration")
    parser.add_argument("--cam-left-id", type=str, default="", metavar="SERIAL",
                        help="Left OAK-D device serial (empty=auto)")
    parser.add_argument("--cam-right-id", type=str, default="", metavar="SERIAL",
                        help="Right OAK-D device serial (empty=auto)")
    parser.add_argument("--cam-x", type=float, default=0.88, metavar="M",
                        help="Camera lateral offset from robot centreline (default: 0.88 — half of 1.76 m inter-camera span; measured)")
    parser.add_argument("--cam-stop-dist", type=float, default=2.5, metavar="M",
                        help="Camera depth obstacle stop distance in metres (default: 2.5)")
    parser.add_argument("--cam-near-filter", type=float, default=0.50, metavar="M",
                        help="Camera minimum depth for obstacle detection in metres (default: 0.50). "
                             "Depths closer than this are ignored — filters out robot frame returns "
                             "and mounting hardware visible in the camera FOV.")
    parser.add_argument("--no-cam-obstacles", action="store_true",
                        help="Use cameras for row tracking only — disable camera depth "
                             "obstacle detection entirely.  Recommended for soybean/dual-row "
                             "fields where forward-facing cameras always see crop rows at "
                             "1–4 m and trigger false OBSTACLE_WAIT.  LiDAR handles all "
                             "obstacle detection in this mode.")
    parser.add_argument("--cam-block-frames", type=int, default=3, metavar="N",
                        help="Consecutive camera-blocked frames required to trigger "
                             "OBSTACLE_WAIT (default: 3). Raise to reduce false positives "
                             "from depth noise or sparse crop returns.")

    # --- 3-D depth fusion (Phase 1 sensor fusion) ---
    parser.add_argument("--cam-depth-3d", action="store_true", default=False,
                        help="Use full 3-D depth-to-point-cloud fusion from OAK-D cameras "
                             "(default: off). Camera at 0.92 m height projects near-horizontal "
                             "views to z ≈ cam_z, which exceeds the 0.75 m obstacle threshold "
                             "even in empty space — reliable only if cam_z ≤ obstacle_height + 0.1 m. "
                             "Enable only when camera is mounted low (< 0.85 m) or obstacle_height "
                             "is raised above cam_z.")
    parser.add_argument("--no-cam-depth-3d", action="store_false", dest="cam_depth_3d",
                        help="Legacy 1-D depth strip obstacle detector (default when --cam-depth-3d "
                             "is not passed).")
    parser.add_argument("--cam-height", type=float, default=0.920, metavar="M",
                        help="Camera height above ground in metres (default: 0.920 — measured). "
                             "Used with --cam-depth-3d for extrinsic transform.")
    parser.add_argument("--cam-yaw-deg", type=float, default=0.0, metavar="DEG",
                        help="Yaw of each camera from the robot forward axis in degrees "
                             "(default: 0.0 = forward-facing). Positive = rotated right. "
                             "Left cam receives negative of this value, right cam positive.")
    parser.add_argument("--cam-pitch-deg", type=float, default=15.0, metavar="DEG",
                        help="Downward pitch of the camera mount in degrees (default: 15.0 — "
                             "matches the VLP-16 nose-down tilt).")
    parser.add_argument("--cam-y-fwd", type=float, default=-0.465, metavar="M",
                        help="Camera offset from LiDAR along robot Y axis (default: -0.465 — "
                             "cameras are 46.5 cm behind the LiDAR). Negative = behind LiDAR.")
    parser.add_argument("--cam-canopy-z", type=float, default=0.10, metavar="M",
                        help="Assumed crop canopy height above ground (m) for camera IPM "
                             "ground-plane projection (default: 0.10). Each camera's midpoint "
                             "bias is proportional to cam_z/(cam_z - canopy_z); the equal-weight "
                             "dual-camera mean cancels this bias to first order regardless of the "
                             "assumed value, but a closer match reduces second-order residual. "
                             "Soybean seedlings: 0.05–0.10 m. Tall onion canopy: 0.20–0.30 m.")
    parser.add_argument("--cam-self-radius", type=float, default=1.0, metavar="M",
                        help="Planar-range self-filter for the 3-D camera depth cloud (default: 1.0 m). "
                             "Points closer than this to the LiDAR origin are assumed to be the robot's "
                             "own structure and are discarded before the safety check.")
    parser.add_argument("--cam-fusion", choices=["estimate", "point"], default="estimate",
                        help="How camera row perception is fused with the LiDAR "
                             "(default: estimate). 'estimate' = each sensor fits its own "
                             "lateral/heading, then the estimates are blended (EKF or "
                             "weighted average) — the original behaviour. 'point' = the "
                             "camera tracker's metric ground points are pooled with the "
                             "LiDAR crop points into ONE weighted row fit (single "
                             "histogram + peak pairing + PCA over all evidence); camera "
                             "mass is capped at 50%% of a full LiDAR scan so the LiDAR "
                             "always dominates on disagreement, while camera points carry "
                             "the fit through empty VLP-16 crop-ROI scans and cover the "
                             "<1.5 m self-filter blind zone. Requires --camera --dual-row.")
    parser.add_argument("--ekf", action="store_true", default=False,
                        help="Enable EKF sensor fusion (default: off). Adds Kalman filter over "
                             "LiDAR+camera estimates — useful on rough terrain with many empty scans, "
                             "but can add lag and destabilise the pure-pursuit loop on smooth rows.")
    parser.add_argument("--no-ekf", action="store_false", dest="ekf",
                        help="Disable EKF and use raw LiDAR confidence directly (default).")
    parser.add_argument("--forward-dist", type=float, default=2.5, metavar="M",
                        help="LiDAR forward safety zone depth (default: 2.5 m). "
                             "Reduce to 1.5 m for indoor or confined-space testing.")
    parser.add_argument("--forward-half-width", type=float, default=0.60, metavar="M",
                        help="LiDAR forward safety zone half-width (default: 0.60 m). "
                             "Robot body is ±0.965 m wide but 0.60 m avoids triggering on "
                             "adjacent row canopy (h≈0.80–0.84 m) when slightly off-centre. "
                             "Raise to 0.965 m only in wide open areas with no adjacent crops.")
    parser.add_argument("--controller", choices=["pursuit", "rl", "mpc"], default="pursuit",
                        help="In-row steering controller (default: pursuit). 'rl' uses a "
                             "learned policy (--policy); 'mpc' uses sampling-based MPC (MPPI) "
                             "with an online disturbance observer for slope-drift rejection. "
                             "Both touch the FOLLOW steering ONLY, with pure-pursuit as the "
                             "low-confidence fallback; the state machine, safety monitor and "
                             "headland turn are unchanged. Compare with scripts/eval_controller.py.")
    parser.add_argument("--policy", default="", metavar="PATH",
                        help="Trained steering policy .npz for --controller rl "
                             "(default: policies/follow_jerk8.0.npz, the smoothest "
                             "sweep policy; missing/absent → pure-pursuit fallback).")
    parser.add_argument("--lookahead", type=float, default=2.0, metavar="M",
                        help="Pure-pursuit lookahead distance in metres (default: 2.0). "
                             "Smaller = more aggressive lateral correction but risks oscillation. "
                             "1.0 m causes diverging limit cycles when starting >15° off heading.")
    parser.add_argument("--max-angular", type=float, default=0.40, metavar="R",
                        help="Maximum angular velocity in rad/s (default: 0.40). "
                             "Raise to 0.60 if the robot drifts and cannot steer back.")
    parser.add_argument("--align-heading", action="store_true", default=False,
                        help="Rotate in-place during ACQUIRE to reduce heading error "
                             "before FOLLOW (default: off). The LiDAR re-measures the "
                             "row each scan so rotation rarely converges; easier to just "
                             "start the robot roughly aligned with the row.")
    parser.add_argument("--align-speed", type=float, default=0.20, metavar="R",
                        help="Max rotation speed (rad/s) during heading pre-align "
                             "(default: 0.20). Raise to 0.35 for faster alignment.")
    parser.add_argument("--row-end-min-dist", type=float, default=1.5, metavar="M",
                        help="Minimum distance (m) driven before ROW_END can trigger "
                             "(default: 1.5). Increase to 5.0+ for longer rows.")
    parser.add_argument("--ros2-bridge", action="store_true",
                        help="Write scan data and nav status to /dev/shm/ every scan so the "
                             "Docker ROS2 bridge (ros2_bridge/start.sh) can publish live "
                             "topics for RViz2 visualization.")
    args = parser.parse_args()

    # Apply field presets BEFORE any further validation so explicit flags can
    # still override them (argparse has already parsed user-supplied values, but
    # we only want the preset to fill params the user did NOT explicitly specify).
    if args.preset == "soybean":
        # Dual-row centre-residue / soybean seedling defaults.
        args.dual_row = True
        if args.crop_min == 0.03:   args.crop_min = 0.03     # already correct
        if args.crop_max == 0.30:   args.crop_max = 0.30     # already correct
        if args.obstacle_height == 0.50: args.obstacle_height = 0.50
        if args.tire_height == 0.65:     args.tire_height = 0.65
        if args.row_spacing == 0.76:     args.row_spacing = 0.76
        print(" [preset] soybean — dual-row, crop_h=[0.03,0.30]m, "
              "obstacle=0.50m, tire=0.65m, row_spacing=0.76m")
    elif args.preset == "onion":
        # Single raised crop-bed / Vidalia onion defaults.
        args.dual_row = False
        args.crop_min = 0.05
        args.crop_max = 0.60
        args.obstacle_height = 0.75
        args.tire_height = 0.85
        print(" [preset] onion — single-row, crop_h=[0.05,0.60]m, "
              "obstacle=0.75m, tire=0.85m")

    # --record: bundle a complete reproducible experiment folder.  Turns on
    # telemetry + SLAM into one runs/<ts>/ dir and writes a manifest now (so a
    # crashed run is still identifiable) + a computed summary on exit.
    run_record = None
    if args.record is not None:
        from navigation.run_record import RunRecord
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Auto-named run dir includes the controller so the folder is
        # self-labelling (runs/run_rl_<ts>/, run_mpc_<ts>/, run_pursuit_<ts>/).
        run_dir = (Path(args.record) if args.record != "auto"
                   else Path("runs") / f"run_{args.controller}_{ts}")
        calibration = {
            "lidar_yaw_deg": args.lidar_yaw, "lidar_tilt_deg": args.lidar_tilt,
            "self_radius_m": args.self_radius, "roi_x_m": args.roi_x,
            "crop_band_m": [args.crop_min, args.crop_max],
            "row_spacing_seed_m": args.row_spacing, "swath_m": args.swath,
            "controller": args.controller, "max_speed_ms": args.speed,
        }
        run_record = RunRecord(run_dir, args=vars(args), calibration=calibration)
        args.telemetry = run_record.telemetry_path       # telemetry → run dir
        args.slam = not args.no_slam                     # coverage + trajectory (unless --no-slam)
        args.save_dir = str(run_dir)                     # map → run dir
        # Raw-scan streaming: default every 10th (~1 Hz); --save-scans overrides.
        scan_every = 10 if args.save_scans is None else int(args.save_scans)
        args._scan_dir = str(run_dir / "scans") if scan_every > 0 else ""
        args._scan_every = scan_every
        print(f" [row_follow] RECORDING experiment → {run_dir}/  "
              f"(manifest written; summary on exit){'  [SLAM off]' if args.no_slam else ''}")
        if scan_every > 0:
            rate = "every scan" if scan_every == 1 else f"every {scan_every}th scan"
            print(f" [row_follow] raw scans → {run_dir}/scans/  ({rate})")

    # Suppress gRPC PollerCompletionQueue noise after event-loop close (Python 3.8 + grpcio).
    import logging
    logging.getLogger("asyncio").setLevel(logging.CRITICAL)

    nav_ref: list = []
    try:
        asyncio.run(_run(args, nav_ref))
    except KeyboardInterrupt:
        pass

    print("\n[row_follow] stopping robot …")
    if nav_ref:
        navigator = nav_ref[0]
        try:
            asyncio.run(navigator.stop())
        except BaseException:
            pass

        if navigator.telemetry is not None:
            navigator.telemetry.close()
            if navigator.telemetry.enabled or navigator.telemetry.count:
                print(f"[row_follow] telemetry: {navigator.telemetry.count} scans logged "
                      f"-> {navigator.telemetry.path}")

        if navigator.scan_recorder is not None:
            rec = navigator.scan_recorder
            rec.stop()
            extra = f" ({rec.dropped} dropped)" if rec.dropped else ""
            print(f"[row_follow] raw scans: {rec.saved} saved -> {rec.dir}/{extra}")

        cov_stats = None
        if args.slam and navigator.slam is not None:
            from slam.map_io import save_map
            runner = navigator.slam
            runner.stop()
            engine = runner.engine
            state = engine.get_state()
            _cov = engine.get_coverage()
            cov_stats = _cov.stats() if _cov is not None else None
            if state.scan_count > 0:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_dir = Path(args.save_dir) if args.save_dir else Path("maps") / ts
                pts_3d = engine.get_3d_points() if engine.map3d_count > 0 else None
                out = save_map(engine._grid, state.trajectory, save_dir, points_3d=pts_3d,
                               coverage=engine.get_coverage(),
                               full_trajectory=engine.get_full_trajectory())
                extra = f" (+{engine.map3d_count:,}-pt 3D cloud)" if engine.map3d_count > 0 else ""
                print(f"[row_follow] map saved: {out / 'map.png'}  "
                      f"[{state.scan_count} scans, {engine.cell_count:,} cells{extra}]")
                print(f"[row_follow] coverage: {out / 'coverage.png'}  "
                      f"[serviced {state.covered_area_m2:.0f} m² over "
                      f"{state.path_length_m:.0f} m, redundancy "
                      f"{state.coverage_redundancy:.2f}× — gaps = missed rows]  "
                      f"+ trajectory.csv")
            else:
                print("[row_follow] SLAM: no scans mapped — nothing to save.")

        # --record: save a real captured scan + render the 3 perception figures.
        if run_record is not None:
            _save_perception_figures(run_record, navigator)
            # compute and write the run summary + CSV metric tables.
            run_record.finalize(coverage=cov_stats)
    print("[row_follow] done.")


if __name__ == "__main__":
    main()
