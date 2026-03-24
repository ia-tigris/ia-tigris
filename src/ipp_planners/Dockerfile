
# Lean base image for workspace builds and CI
FROM ros:noetic-ros-base AS ipp_planner_core
ARG WS=/workspace
WORKDIR $WS
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Install core workspace build dependencies.
RUN echo "Installing planner_ws core dependencies... \n" && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils \
        curl \
        gdal-bin \
        libbenchmark-dev \
        libcgal-dev \
        libeigen3-dev \
        libgdal-dev \
        libusb-dev \
        python3-catkin-tools \
        python3-vcstool \
        ros-noetic-angles \
        ros-noetic-catkin \
        ros-noetic-cv-bridge \
        ros-noetic-dynamic-edt-3d \
        ros-noetic-eigen-conversions \
        ros-noetic-gazebo-msgs \
        ros-noetic-geographic-msgs \
        ros-noetic-geometry2 \
        ros-noetic-image-geometry \
        ros-noetic-image-transport \
        ros-noetic-octomap \
        ros-noetic-octomap-msgs \
        ros-noetic-pcl-ros \
        ros-noetic-tf \
        ros-noetic-tf-conversions \
        vim && \
    rm -rf /var/lib/apt/lists/*
ENV PYTHONPATH="/workspace/src/ipp_simple_sim"

# Inline entrypoint: sources ROS, optional headless display + VNC
RUN cat <<'ENTRY' > /ros_entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
export DISABLE_ROS1_EOL_WARNINGS=1
if [ -f /workspace/devel/setup.bash ]; then
  source /workspace/devel/setup.bash
fi
if [ "${ENABLE_HEADLESS_DISPLAY:-0}" = "1" ]; then
  Xvfb :1 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 &
  x11vnc -display :1 -nopw -forever -shared -quiet >/tmp/x11vnc.log 2>&1 &
  sleep 1
  export DISPLAY=:1
fi
exec "$@"
ENTRY
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# Full interactive/local image used by docker compose.
FROM ipp_planner_core AS ipp_planner
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-tk \
        ros-noetic-foxglove-bridge \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-rosbridge-suite \
        ros-noetic-rviz \
        x11vnc \
        xvfb && \
    rm -rf /var/lib/apt/lists/*

# ---- isolated test/benchmark stage (used by build_test_benchmark.sh) ----
# Build context must be the workspace root so COPY src/ works.
#   docker build --target test -f src/ipp_planners/Dockerfile .
FROM ipp_planner_core AS test
COPY src/ $WS/src/
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build -DCMAKE_BUILD_TYPE=Release --no-status --summarize
RUN catkin build --no-deps --make-args tests -- trochoids ipp_planners
RUN mkdir -p /test_results
ENTRYPOINT ["/bin/bash"]
