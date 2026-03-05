
# image target for entire workspace
FROM ros:noetic-ros-base AS ipp_planner
ARG WS=/workspace
WORKDIR $WS
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# installing workspace dependencies
RUN echo "Installing planner_ws dependencies... \n" &&\
    apt-get update && apt-get install -y apt-utils && apt-get install -y curl
RUN sudo apt-get install gdal-bin libgdal-dev libusb-dev libeigen3-dev libcgal-dev libbenchmark-dev -y 
RUN sudo apt-get install ros-noetic-image-transport ros-noetic-image-geometry -y
RUN sudo apt-get install ros-noetic-cv-bridge ros-noetic-tf-conversions ros-noetic-tf -y 
RUN sudo apt-get install ros-noetic-angles ros-noetic-geometry2 ros-noetic-pcl-ros ros-noetic-catkin python3-catkin-tools -y 
RUN apt-get update &&\
    sudo apt-get install ros-noetic-catkin -y &&\
    sudo apt-get install python3-vcstool -y &&\
    sudo apt-get install ros-noetic-geographic-msgs -y &&\
    sudo apt-get install ros-noetic-octomap-msgs -y &&\
    sudo apt-get install ros-noetic-octomap -y &&\
    sudo apt-get install ros-noetic-dynamic-edt-3d -y &&\
    sudo apt-get install ros-noetic-rosbridge-suite -y &&\
    sudo apt-get install ros-noetic-gazebo-ros-pkgs -y &&\
    sudo apt-get install ros-noetic-eigen-conversions -y &&\
    sudo apt-get install ros-noetic-rviz -y &&\
    sudo apt-get install vim -y && \
    sudo apt-get install python3-tk -y && \
    sudo apt-get install ros-noetic-foxglove-bridge -y && \
    sudo apt-get install xvfb x11vnc -y
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

# ---- isolated test/benchmark stage (used by build_test_benchmark.sh) ----
# Build context must be the workspace root so COPY src/ works.
#   docker build --target test -f src/ipp_planners/Dockerfile .
FROM ipp_planner AS test
COPY src/ $WS/src/
RUN catkin config --extend /opt/ros/$ROS_DISTRO && \
    catkin build -DCMAKE_BUILD_TYPE=Release --no-status --summarize
RUN catkin build --no-deps --make-args tests -- trochoids ipp_planners
RUN mkdir -p /test_results
ENTRYPOINT ["/bin/bash"]