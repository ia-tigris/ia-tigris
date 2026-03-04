# image target for entire workspace
# run this from the workspace root directory so src is loaded correctly
FROM ros:melodic-ros-base as ipp_planner
ARG WS=/workspace
ARG ROOT=.
WORKDIR $WS

# installing workspace dependencies
RUN echo "Installing planner_ws dependencies... \n" &&\
    apt-get update && apt-get install --no-install-recommends -y apt-utils curl&&\
    sudo apt-get install --no-install-recommends gdal-bin libgdal-dev libopencv-dev libusb-dev libeigen3-dev libcgal-dev libbenchmark-dev -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-image-transport ros-melodic-image-geometry ros-melodic-cv-bridge ros-melodic-tf-conversions ros-melodic-tf ros-melodic-angles ros-melodic-pcl-ros ros-melodic-catkin python-catkin-tools -y &&\
    apt-get update &&\
    sudo apt-get install --no-install-recommends ros-melodic-catkin python-catkin-tools -y &&\
    sudo apt-get install --no-install-recommends python3-vcstool -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-geographic-msgs -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-octomap-msgs -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-octomap -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-dynamic-edt-3d -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-rosbridge-suite -y &&\
    sudo apt-get install --no-install-recommends ros-melodic-gazebo-ros-pkgs -y &&\
    sudo apt-get install --no-install-recommends vim -y &&\
    export PYTHONPATH="${PYTHONPATH}:/{$WS}/src/ipp_simple_sim"

COPY src/ $WS/src/
RUN catkin config --extend /opt/ros/$ROS_DISTRO 

RUN catkin build


# Copy over files (all for now) and use a smaller ROS docker image ros-core 
FROM ros:melodic-ros-core as ipp_planner_run
COPY --from=ipp_planner /workspace /workspace


# Build the docker using the following
# sudo docker build --memory=8g -t <tag-name> -f src/ipp_planners/Docker/Dockerfile .


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