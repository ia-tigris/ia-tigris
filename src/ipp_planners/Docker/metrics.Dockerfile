# image target for entire workspace
# run this from the workspace root directory so src is loaded correctly
FROM ros:noetic as ipp_planner
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
RUN sudo apt-get install ros-noetic-angles ros-noetic-pcl-ros ros-noetic-catkin python3-catkin-tools -y 
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
    sudo apt-get install vim -y &&\
    sudo apt-get install git -y &&\
    sudo apt-get install ros-noetic-tf2-geometry-msgs -y &&\
    export PYTHONPATH="${PYTHONPATH}:/{$WS}/src/ipp_simple_sim"
RUN sudo apt-get install python3-pip -y &&\
    sudo apt-get install python3-tk -y && \
    pip3 install psutil

COPY src/ $WS/src/
RUN catkin config --extend /opt/ros/$ROS_DISTRO 

RUN catkin build -DCMAKE_BUILD_TYPE=Release
RUN catkin build --make-args tests -- trochoids ipp_planners
RUN mkdir /test_results


# Build the docker using the following
# sudo docker build --memory=8g -t <tag-name> -f src/ipp_planners/Docker/Dockerfile .
# sudo docker build --memory=8g -t ipp-noetic -f src/ipp_planners/Docker/noetic.Dockerfile .



# Older build notes
# $ sudo docker build -t planner_demo --target planner_demo .

# After building, build the code, verify its working properly and then commit the changes using the following steps
# $ sudo docker run -p 9090:9090 -it  planner_demo
# $ sudo docker ps -a
# $ sudo docker commit [CONTAINER_ID] [new_image_name]

# Run the docker container in interactive mode using the following command. This command publishes the docker internal port 9090 to the host port 9090
# $ sudo docker run -p 9090:9090 -it [new_image_name]
# $ sudo docker run -p [host_port]:[docker_port] -it [new_image_name]

# Export the image as a tarball using the following command 
# $ sudo docker save onr-planner-demo-dec-21:latest | gzip > onr-planner-demo-dec-21.tar.gz

# Load docker tarball on host machine using the following command
# $ docker load < onr-planner-demo-dec-21.tar.gz

# <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />

# To run demo docker without interactive mode and auto roslaunch
# add the following to ros_entrypoint.sh
# source /workspace/devel/setup.bash
# roslaunch onr_cedric cedric.launch
# then exit the docker container and commit your changes to a new image
# sudo docker ps -a
# copy the container ID for the latest modified image
# then commit the changes
# sudo docker commit [CONTAINER_ID] [new_image_name]

# From the terminal use the following command
# $ sudo docker run -dp 9090:9090 onr-planner-demo-dec-21      