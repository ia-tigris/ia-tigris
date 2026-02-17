# Planner and Map Interfaces
The repo is a stub for what we will be developing. It has all of the current messages that the planner and map with be subscribing and publishing to. Currently the node acts as a black box that will receive inputs and publish a naive plan (the average of all of the search boundary points). The interfaces document can be found [here](https://docs.google.com/document/d/1xQlAcouWq4LGp_6sIEFEx8BWmrwq555Pa5aYIgSc8-w/edit?usp=sharing)

We have also created a node for mocking a publisher that will publish example request and constraint messages which can be triggered with a service call. This node would not be necessary when integrating with the actual system.

## Add into your workspace
Adding both planner interface and mapping interface:
```
cd your_ws/src
git clone --recurse-submodules git@bitbucket.org:castacks/mapping_interface.git
git clone git@bitbucket.org:castacks/planner_map_interfaces.git
```

## Dependencies
In addition to the basic ros environment, there are several dependencies to add:
pcl-ros:
```
sudo apt-get install ros-melodic-pcl-ros
```
geographic-msgs
```
sudo apt-get install ros-melodic-geographic-msgs
```
image-geometry
```
sudo apt-get install ros-melodic-image-geometry
```

CGAL and eigen_conversion
```
sudo apt-get install libcgal-dev eigen_conversions
```

If CATKIN_ENABLE_TESTING is true, then install Google Benchmark
```
sudo apt install libbenchmark-dev
```

## Build
In your workspace run catkin build and source (maybe twice if the messages don't build on first try)
```
catkin build --continue-on-failure
source devel/setup.bash
```

## Launching the Interface Node
The interfaces node can be launched with
```
rosrun planner_map_interfaces planner_map_interfaces_node
```

## Launching the Interface and Mock Trigger Node
To run our interface node and mock trigger server, run the command
```
roslaunch planner_map_interfaces test_interface.launch
```

In order to trigger an example sensor constraint and/or planning request, call the service call with
```
rosservice call /planner_mock_trigger true true
```

where 

- arg1 is for sensor_constraint
- arg2 is for plan_request

## Launching the Mapping Interface
The simulated mapping can be launched with
```
roslaunch simple_mapping semantic_grid_onr.launch
```
then publish the SensorMeasurement type of message. Or you can play the rosbag we made as an example.

## Overview Diagram
![Interface Flow Diagram](/images/interfaceFlow.jpg)
