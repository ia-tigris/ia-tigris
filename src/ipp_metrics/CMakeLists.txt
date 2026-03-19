cmake_minimum_required(VERSION 3.5)
project(metrics)

## Compile as C++17
add_compile_options(-std=c++17 -lstdc++fs)  # to use #include <experimental/filesystem>
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ipp_planners
  planner_map_interfaces
  ipp_simple_sim
  core_planning_state_space
  math_utils
  roscpp
  trochoids
)

find_package(CGAL REQUIRED)
find_package(ompl REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES metrics
#  CATKIN_DEPENDS ipp_planners planner_map_interfaces ipp_simple_sim
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
  ${OMPL_INCLUDE_DIRS}
  ipp_planners
  CGAL::CGAL
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/metrics.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/metrics_node.cpp)

add_executable(${PROJECT_NAME}_mc_node src/ipp_mc_runs_node.cpp)

add_dependencies(${PROJECT_NAME}_mc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_mc_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  CGAL::CGAL
)

catkin_install_python(PROGRAMS
  scripts/record_search_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)