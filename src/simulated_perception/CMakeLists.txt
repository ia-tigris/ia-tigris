cmake_minimum_required(VERSION 3.5)
project(simulated_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17 -lstdc++fs)  # to use #include <experimental/filesystem>

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  ipp_simple_sim
  message_generation
  planner_map_interfaces
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# moved to planner_map_interfaces
# add_message_files(
#   FILES
#   FilteredTarget.msg
#   FilteredTargets.msg
# )

# generate_messages(
#   DEPENDENCIES
#   nav_msgs 
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
 INCLUDE_DIRS 
#  include
 ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES simulated_perception
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(filter_class src/filter.cpp
)

target_link_libraries(filter_class
  ${catkin_LIBRARIES}
  stdc++fs
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(simulated_perception_node src/simulated_perception_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulated_perception_node
  filter_class
  ${catkin_LIBRARIES}
)
