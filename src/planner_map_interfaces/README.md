# Planner and Map Interfaces
This repo has all of the current messages that the planner and map with be subscribing and publishing to. 

We have also created a node for mocking a publisher that will publish example request and constraint messages which can be triggered with a service call. This node would not be necessary when integrating with the actual system.

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

