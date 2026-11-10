#!/usr/bin/env bash
# field_speed_test.sh — find what slows the perception loop (field 2026-11-06:
# 4 scans/s instead of ~8, and half-scans).  PERCEPTION ONLY: the robot does
# NOT move.  Runs row_follow once per configuration for SECS seconds, then
# prints one comparison table.  ~6 × 50 s ≈ 5 minutes.
#
#   source /farm_ng_image/venv/bin/activate
#   cd ~/auto_navigation_vidalia
#   bash scripts/field_speed_test.sh            # 40 s per configuration
#   SECS=60 bash scripts/field_speed_test.sh
#
# Park the robot on a bed (same spot for every configuration).  Close the GPS
# logger first if you want a clean comparison; the table also has a row with
# it running if you start it yourself in another terminal.
set -u
SECS="${SECS:-40}"
OUT="runs/speedtest_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
BASE="python3 scripts/row_follow.py --preset onion"

run() {   # run <name> <env-prefix> <extra args...>
    local name="$1"; shift
    local envp="$1"; shift
    echo
    echo "=== $name  ($SECS s)  $envp $*"
    if pgrep -f "scripts/row_follow.py" >/dev/null; then
        echo "  another row_follow is still running — stop it first (Ctrl+C in its terminal)"; exit 1
    fi
    env $envp timeout -s INT -k 20 "$((SECS + 12))" $BASE --record "$OUT/$name" "$@" \
        > "$OUT/$name.log" 2>&1
    tail -n 12 "$OUT/$name.log" | grep -E "RUN SUMMARY|scans @|scan health" || tail -n 3 "$OUT/$name.log"
    sleep 3
}

run 1_new_defaults      ""
run 2_old_inline_reader ""  --lidar-reader inline
run 3_numpy_all_cores   "OPENBLAS_NUM_THREADS=6 OMP_NUM_THREADS=6"
run 4_slam_unlimited    ""  --slam-hz 0
run 5_no_slam           ""  --no-slam
run 6_as_on_nov06       "OPENBLAS_NUM_THREADS=6 OMP_NUM_THREADS=6" \
                            --lidar-reader inline --slam-hz 0 --no-half-scan-guard

python3 - "$OUT" <<'EOF'
import json, sys
from pathlib import Path
out = Path(sys.argv[1])
print(f"\n{'configuration':22s} {'scans/s':>8s} {'loop ms':>8s} {'incompl%':>9s} {'half%':>6s} "
      f"{'skipped':>8s} {'LiDAR Hz':>9s} {'age ms':>7s}")
for d in sorted(p for p in out.iterdir() if p.is_dir()):
    f = d / "summary.json"
    if not f.exists():
        print(f"{d.name:22s} (no summary — see {d.name}.log)"); continue
    m = json.load(open(f)); h = m.get("scan_health", {})
    g = lambda k, s=1, fmt="{:.0f}": fmt.format(h[k] * s) if k in h else "-"
    print(f"{d.name:22s} {m['scan_rate_hz']:8.1f} {g('loop_ms_p50'):>8s} {g('incomplete_frac', 100):>9s} "
          f"{g('half_scan_frac', 100):>6s} {g('lidar_scans_skipped'):>8s} {g('lidar_rate_hz', 1, '{:.1f}'):>9s} "
          f"{g('scan_age_ms_p50'):>7s}")
print(f"\nLogs and run folders: {out}/   (send me this folder)")
print("Read: 'scans/s' = how many scans driving used; 'incompl%' = scans missing part of the view;")
print("1 should be near 8+ scans/s with ~0% incomplete. 6 should look like Nov 6 (≈4, many incomplete).")
EOF
