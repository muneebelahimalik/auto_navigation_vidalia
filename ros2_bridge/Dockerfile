FROM osrf/ros:foxy-ros-base

# Install only what the bridge node needs — no full desktop (runs headless on Amiga)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-numpy \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs \
    ros-foxy-visualization-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-geometry-msgs \
    ros-foxy-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
SHELL ["/bin/bash", "-c"]

COPY vidalia_node.py /workspace/vidalia_node.py

CMD source /opt/ros/foxy/setup.bash && python3 /workspace/vidalia_node.py
