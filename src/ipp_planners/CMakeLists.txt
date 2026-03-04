cmake_minimum_required(VERSION 3.5...3.27)
project(ipp_planners)

# Set CMake policies conditionally for compatibility
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)  # Use old Python modules for ROS compatibility
endif()

# Disable ROS1 EOL warnings
set(ENV{DISABLE_ROS1_EOL_WARNINGS} "1")

# # Compile as C++17
add_compile_options(-std=c++17 -lstdc++fs)
set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  core_planning_state_space
  eigen_conversions
  geographic_msgs
  geometry_msgs
  math_utils
  message_generation
  planner_map_interfaces
  roscpp
  std_msgs
  trochoids
)
message(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

# set the default build type
if(NOT CMAKE_BUILD_TYPE)
  # By default, use Release mode
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)
endif()

# eigen
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# OMPL
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

if(USE_MCTS)
  add_definitions(-DUSE_MCTS)
  # libtorch
  # set libtorch to be under onr_ws/libtorch
  set(TORCH_PATH "${CMAKE_SOURCE_DIR}/../../libtorch")
  message(WARNING "Appending this path to find libtorch, make sure to libtorch exists here: ${TORCH_PATH}")
  list(APPEND CMAKE_PREFIX_PATH ${TORCH_PATH})
  message(WARNING "CMAKE_PREFIX_PATH is now ${CMAKE_PREFIX_PATH}")

  # auto download libtorch if doesn't exist
  if(NOT EXISTS "${CMAKE_SOURCE_DIR}/../../libtorch")
    message(WARNING "libtorch not found, downloading...")
    execute_process(
      COMMAND bash -c
      "wget 'https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.12.1%2Bcpu.zip' \
        && unzip libtorch-cxx11-abi-shared-with-deps-1.12.1+cpu.zip -d ${CMAKE_SOURCE_DIR}/../../"
      OUTPUT_VARIABLE outVar
    )
  endif()

  find_package(Torch REQUIRED)
  include_directories(${TORCH_INCLUDE_DIRS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
endif()

# CGAL
find_package(CGAL REQUIRED)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# set(CMAKE_BUILD_TYPE Release)
# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a exec_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
# add_message_files(
# FILES
# Message1.msg
# Message2.msg
# )

# # Generate services in the 'srv' folder
add_service_files(
  FILES
  SearchMapSetup.srv
  SearchMapUpdate.srv

  # Service1.srv
  # Service2.srv
)

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# # Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs # Or other packages containing msgs
  geometry_msgs
  geographic_msgs
  planner_map_interfaces
)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
# cfg/DynReconf1.cfg
# cfg/DynReconf2.cfg
# )

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ipp_planners

  # CATKIN_DEPENDS roscpp std_psgs
  # DEPENDS system_lib
  # DEPENDS Eigen3
)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# ##########
# # Build ##
# ##########

# # Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/ipp_planners.cpp
# )
add_library(${PROJECT_NAME}
  src/Tigris.cpp
  src/SearchMap.cpp
  # src/Tracker.cpp
)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ipp_planners_node.cpp)

add_executable(${PROJECT_NAME}_search_map src/realtime_search_map_node.cpp)

target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

# # Rename C++ executable without prefix
# # The above recommended prefix causes long target names, the following renames the
# # target back to the shorter version for ease of user use
# # e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# # Add cmake target dependencies of the executable
# # same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_search_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ipp_planners
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  CGAL::CGAL
  "${TORCH_LIBRARIES}"
  stdc++fs
)

target_link_libraries(${PROJECT_NAME}_search_map
  ipp_planners
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${OpenCV_LIBS}
  CGAL::CGAL
  "${TORCH_LIBRARIES}"
  stdc++fs
)

# add_executable(ipp src/Tigris.cpp)
# target_link_libraries(ipp
# ${catkin_LIBRARIES}
# )

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/pub_dummy_image_with_pose.py
  scripts/planner_simple_waypoint_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_search_map

  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark cpp header files for installation
if(USE_MCTS)
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
else()
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN "MCTS.h" EXCLUDE
    PATTERN ".svn" EXCLUDE
  )
endif()

# # Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  # launch/main.launch
  # launch/tigris.launch
  # launch/search_belief.launch
  # launch/unit_test.launch


  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  # config/tigris.yaml
  # config/search_map.yaml
  # config/webviz_layout.json

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  config/rviz_extra/tigris_search.rviz

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# ############
# # Testing ##
# ############
# set(CATKIN_ENABLE_TESTING 0)
# # Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test 
                  test/all_tests.cpp
  )

  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} CGAL::CGAL stdc++fs)
  endif()

  find_package(benchmark REQUIRED)
  add_executable(${PROJECT_NAME}-benchmark test/all_benchmarks.cpp)
  target_link_libraries(${PROJECT_NAME}-benchmark ${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES} CGAL::CGAL benchmark::benchmark stdc++fs)

endif(CATKIN_ENABLE_TESTING)

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
