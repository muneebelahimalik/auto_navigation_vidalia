# L4T-optimised ROS2 Foxy base for Jetson Xavier NX — L4T R35.2.1 / Jetpack 5.1
# Matches the farm-ng Amiga Brain kernel (linux-tegra 5.10.104, aarch64).
# /var/lib/docker lives on NVMe → image persists across reboots.
FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1

# The Open Robotics ROS 2 apt signing key (F42ED6FBAB17C654) expires and must be
# refreshed before apt-get update — without this the build fails with EXPKEYSIG.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | apt-key add -

# Install bridge node deps + Foxglove WebSocket bridge for browser visualisation.
# ros-foxy-foxglove-bridge exposes all ROS2 topics over WebSocket on port 8765.
# Open https://app.foxglove.dev in any browser — zero install on any other device.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-visualization-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-std-msgs \
    ros-foxy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

COPY vidalia_node.py /workspace/vidalia_node.py
COPY entrypoint.sh  /workspace/entrypoint.sh
RUN chmod +x /workspace/entrypoint.sh

EXPOSE 8765

ENTRYPOINT ["/workspace/entrypoint.sh"]
