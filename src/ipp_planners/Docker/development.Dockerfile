
# image target for entire workspace
FROM ros:noetic-ros-base as ipp_planner
ARG WS=/workspace
ARG ROOT=.
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
ENV PYTHONPATH="${PYTHONPATH}:/workspace/src/ipp_simple_sim"

# Add entrypoint script for Xvfb and x11vnc
COPY Docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]