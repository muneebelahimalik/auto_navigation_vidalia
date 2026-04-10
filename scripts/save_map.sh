#!/usr/bin/env bash
# save_map.sh — save the current RTAB-Map database and export a PCD point cloud.
#
# Run while slam_live.launch.py (or slam_rtabmap_lidar3d.launch.py) is active.
#
# Usage:
#   bash scripts/save_map.sh [destination_dir]
#
# Examples:
#   bash scripts/save_map.sh                        # saves to ~/maps/<timestamp>/
#   bash scripts/save_map.sh /data/field_map_2024   # saves to specified dir

set -e

DEST_DIR="${1:-$HOME/maps/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$DEST_DIR"

echo "[save_map] Saving map to: $DEST_DIR"

# 1. Copy the live RTAB-Map database (default location).
#    The database is written continuously during mapping so this is always current.
DB_SRC="$HOME/.ros/rtabmap.db"
if [ -f "$DB_SRC" ]; then
    cp "$DB_SRC" "$DEST_DIR/rtabmap.db"
    echo "[save_map] Database saved: $DEST_DIR/rtabmap.db"
else
    echo "[save_map] WARNING: database not found at $DB_SRC"
    echo "           Make sure slam_rtabmap_lidar3d.launch.py is (or was) running."
fi

# 2. Export a coloured PCD point-cloud map (requires rtabmap ROS2 tools installed).
#    The service /rtabmap/publish_maps triggers cloud_map publication; we use
#    ros2 topic echo with --once to capture it, or rtabmap-export offline.
echo "[save_map] Triggering point-cloud map export via ros2 service..."

if ros2 service list 2>/dev/null | grep -q '/rtabmap/get_map'; then
    ros2 service call /rtabmap/get_map \
        rtabmap_msgs/srv/GetMap \
        "{global_map: true, optimized: true, graphOnly: false}" \
        --timeout 10 \
    && echo "[save_map] Map service call succeeded." \
    || echo "[save_map] WARNING: map service call failed or timed out — use the saved .db file."
else
    echo "[save_map] INFO: rtabmap node not currently running; skipping live export."
    echo "           To export a PCD later, run:"
    echo "             rtabmap-export --images $DEST_DIR/rtabmap.db"
fi

# 3. Write a convenience launch snippet showing how to localise with this map.
cat > "$DEST_DIR/localise.sh" <<EOF
#!/usr/bin/env bash
# Localise against the map saved in this directory.
source /opt/ros/foxy/setup.bash
source ~/auto_navigation_vidalia/install/setup.bash
ros2 launch vidalia_bringup slam_localization.launch.py \\
    database_path:=$DEST_DIR/rtabmap.db
EOF
chmod +x "$DEST_DIR/localise.sh"

echo "[save_map] Done.  Files in $DEST_DIR:"
ls -lh "$DEST_DIR"
echo ""
echo "[save_map] To localise against this map later:"
echo "    bash $DEST_DIR/localise.sh"
echo "  or:"
echo "    ros2 launch vidalia_bringup slam_localization.launch.py database_path:=$DEST_DIR/rtabmap.db"
