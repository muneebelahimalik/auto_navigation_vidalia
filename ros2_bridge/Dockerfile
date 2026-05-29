# L4T-optimised ROS2 Foxy DESKTOP for Jetson Xavier NX — L4T R35.2.1 / Jetpack 5.1
# The "desktop" variant includes RViz2, rqt, and all visualisation plugins pre-built.
# Image is ~13 GB — pulled once, stored in /var/lib/docker on NVMe (persists every reboot).
FROM dustynv/ros:foxy-desktop-l4t-r35.2.1

# Fix expired Open Robotics ROS 2 signing key (EXPKEYSIG F42ED6FBAB17C654).
# Overwrite the signed-by keyring used by this image's sources.list entry.
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg \
          /etc/apt/sources.list.d/ros2-latest.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install virtual-display stack for browser-based RViz2 access.
# x11vnc serves the Xvfb virtual display; noVNC + websockify expose it on port 6080.
# python3-numpy is the only Python dep not guaranteed in the desktop base.
RUN apt-get update \
    -o Acquire::AllowInsecureRepositories=true \
    && apt-get install -y --no-install-recommends \
       --allow-unauthenticated \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

COPY vidalia_node.py   /workspace/vidalia_node.py
COPY rviz/vidalia.rviz /workspace/vidalia.rviz
COPY entrypoint.sh     /workspace/entrypoint.sh
RUN chmod +x /workspace/entrypoint.sh

# noVNC web interface (browser-based VNC client)
EXPOSE 6080

ENTRYPOINT ["/workspace/entrypoint.sh"]
