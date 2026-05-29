#!/usr/bin/env bash
# laptop_setup.sh — Install ROS2 Humble on Ubuntu 22.04 dev PC and launch RViz2.
#
# Run this ONCE on your Ubuntu 22.04 laptop to install ROS2 Humble.
# Then use the "Quick start" commands at the bottom to connect to the Amiga.
#
# Network requirement:
#   Both machines must be reachable from each other (same LAN, or Tailscale).
#   Set the same ROS_DOMAIN_ID on both the Amiga bridge and this laptop.
#   For Tailscale, you may need FastDDS unicast config (see CYCLONEDDS section).
#
# Usage:
#   bash ros2_bridge/laptop_setup.sh      # install ROS2 Humble (once)
#   bash ros2_bridge/laptop_setup.sh viz  # just launch RViz2 (after install)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
AMIGA_IP="${AMIGA_IP:-100.66.121.56}"   # Tailscale IP — change if different

# --------------------------------------------------------------------------
# Install ROS2 Humble on Ubuntu 22.04
# --------------------------------------------------------------------------
install_ros2() {
    echo "=== Installing ROS2 Humble on Ubuntu 22.04 ==="

    sudo apt update
    sudo apt install -y locales curl gnupg lsb-release software-properties-common
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo add-apt-repository universe

    # ROS2 apt source
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-desktop ros-dev-tools

    # Source in bashrc
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    # ROS_DOMAIN_ID
    if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
        echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> ~/.bashrc
    fi

    echo ""
    echo "=== ROS2 Humble installed. ==="
    echo "    Open a new terminal and run:  bash ros2_bridge/laptop_setup.sh viz"
}

# --------------------------------------------------------------------------
# Launch RViz2 pointed at the Amiga
# --------------------------------------------------------------------------
launch_viz() {
    echo "=== Launching RViz2 for Vidalia ==="
    echo ""
    echo "  ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
    echo "  Amiga IP      = $AMIGA_IP  (Tailscale)"
    echo ""

    # Source ROS2 if not already sourced
    if ! command -v rviz2 &>/dev/null; then
        source /opt/ros/humble/setup.bash
    fi

    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"

    # For Tailscale cross-node discovery, configure CycloneDDS with unicast peer.
    # This bypasses multicast (which Tailscale blocks) and connects directly.
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface autodetermine='true' priority='default' multicast='false'/>
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer Address='${AMIGA_IP}'/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>"

    echo "  CycloneDDS unicast peer: $AMIGA_IP"
    echo ""
    echo "  Check topics with:  ros2 topic list"
    echo "  Expected:           /velodyne_points  /row_viz  /safety_viz  /cmd_vel  /tf_static"
    echo ""

    rviz2 -d "$SCRIPT_DIR/rviz/vidalia.rviz"
}

# --------------------------------------------------------------------------
case "${1:-install}" in
    viz)   launch_viz   ;;
    *)     install_ros2 ;;
esac
