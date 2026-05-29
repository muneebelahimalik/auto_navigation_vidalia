# L4T-optimised ROS2 Foxy base for Jetson Xavier NX — L4T R35.2.1 / Jetpack 5.1
# Matches the farm-ng Amiga Brain kernel (linux-tegra 5.10.104, aarch64).
# /var/lib/docker lives on NVMe → image persists across reboots.
FROM dustynv/ros:foxy-ros-base-l4t-r35.2.1

# The Open Robotics ROS 2 signing key baked into this image has expired.
# This image uses signed-by= (not the legacy apt-key keyring), so we must
# overwrite /usr/share/keyrings/ros-archive-keyring.gpg with the current key
# and rebuild the sources.list entry to guarantee it points to that file.
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && printf 'deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main\n' \
       > /etc/apt/sources.list.d/ros2-latest.list

# Install bridge node deps + Foxglove WebSocket bridge for browser visualisation.
# ros-foxy-foxglove-bridge exposes all ROS2 topics over WebSocket on port 8765.
# Open https://app.foxglove.dev in any browser — zero install on any other device.
# --allow-unauthenticated is a belt-and-suspenders fallback in case the key at
# the URL above has also since expired by the time you build.
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
    ros-foxy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

COPY vidalia_node.py /workspace/vidalia_node.py
COPY entrypoint.sh  /workspace/entrypoint.sh
RUN chmod +x /workspace/entrypoint.sh

EXPOSE 8765

ENTRYPOINT ["/workspace/entrypoint.sh"]
