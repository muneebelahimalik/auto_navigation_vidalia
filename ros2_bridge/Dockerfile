FROM osrf/ros:foxy-ros-base

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
