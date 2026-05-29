# L4T-optimised ROS2 Foxy base for Jetson Xavier NX — L4T R35.2.1 / Jetpack 5.1
# Matches the farm-ng Amiga Brain kernel (linux-tegra 5.10.104, aarch64).
# /var/lib/docker lives on NVMe → image persists across reboots.
FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1

# The Open Robotics ROS 2 signing key baked into this image has expired.
# The image uses signed-by=/usr/share/keyrings/ros-archive-keyring.gpg (not the
# legacy apt-key keyring), so overwrite that file with the refreshed key.
# Also remove ros2-latest.list if created by a previous failed build — the real
# source list is ros2.list and having both causes duplicate-repo warnings.
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg \
          /etc/apt/sources.list.d/ros2-latest.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install bridge node deps + Rosbridge WebSocket server for Foxglove visualisation.
# ros-foxy-rosbridge-suite provides the "Rosbridge WebSocket" endpoint on port 8765.
# (ros-foxy-foxglove-bridge is not published for focal/arm64; rosbridge is equivalent
#  and connects to Foxglove Studio via Open connection → Rosbridge WebSocket.)
# --allow-unauthenticated is a fallback if the refreshed key above still fails.
RUN apt-get update \
    -o Acquire::AllowInsecureRepositories=true \
    && apt-get install -y --no-install-recommends \
       --allow-unauthenticated \
    python3-pip \
    python3-numpy \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-visualization-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-std-msgs \
    ros-foxy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

COPY vidalia_node.py /workspace/vidalia_node.py
COPY entrypoint.sh  /workspace/entrypoint.sh
RUN chmod +x /workspace/entrypoint.sh

EXPOSE 8765

ENTRYPOINT ["/workspace/entrypoint.sh"]
