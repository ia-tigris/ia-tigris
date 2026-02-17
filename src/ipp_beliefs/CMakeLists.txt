cmake_minimum_required(VERSION 3.0.2)
project(ipp_belief)

## Compile as C++17
add_compile_options(-std=c++17 -lstdc++fs)
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH "/opt/ros/melodic/share/catkin/cmake")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

        # ros libraries
        roslaunch
        roscpp
        rospy
        std_msgs
        visualization_msgs
        geometry_msgs
        geographic_msgs
        sensor_msgs
        message_filters
        tf2
        tf2_ros
        tf2_eigen

        # custom
        planner_map_interfaces
        message_generation
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(planner_map_interfaces REQUIRED)
include_directories(${planner_map_interfaces_INCLUDE_DIRS})

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

# Generate messages in the 'msg' folder
add_message_files(
        FILES
        Particle.msg
        ParticleFiltersBelief.msg
        ParticleFilterWithCtrlParams.msg
)

# Generate services in the 'srv' folder
add_service_files(
        FILES
        AddTargetPriors.srv
        ClearTargetPriors.srv
        RemoveTargetPriors.srv
)

# # Generate actions in the 'action' folder
# add_action_files(
# FILES
# Action1.action
# Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
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
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs geometry_msgs message_runtime planner_map_interfaces

        # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(LIB_HEADERS
        include/ipp_belief/belief_manager.h
        include/ipp_belief/colors.h
        include/ipp_belief/control.h
        include/ipp_belief/observation.h
        include/ipp_belief/region.h
        include/ipp_belief/state.h
        include/ipp_belief/trackers.h
        include/ipp_belief/visualize.h
        include/ipp_belief/sampling_distributions.h
)

# ###########################################
# DECLARE THE COMMON LIBRARY
# ###########################################
# # Declare a C++ library
add_library(${PROJECT_NAME}
        src/library.cpp
        ${LIB_HEADERS}
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} planner_map_interfaces_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen stdc++fs)

# ###########################################
# DECLARE THE EXECUTABLE              #
# ###########################################
# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(
        ${PROJECT_NAME}_node
        ${LIB_HEADERS}
        include/ipp_belief/ipp_belief_node.h
        src/ipp_belief_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} planner_map_interfaces_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Eigen3::Eigen stdc++fs)
add_executable(
        ${PROJECT_NAME}_visualizer_node
        ${LIB_HEADERS}
        src/ipp_belief_visualizer_node.cpp
)
add_dependencies(${PROJECT_NAME}_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} planner_map_interfaces_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_visualizer_node ${catkin_LIBRARIES} Eigen3::Eigen stdc++fs)

#
# add_executable(
# demo_node
# src/demo_node.cpp
# ${HEADERS}
# src/library.cpp)
# add_dependencies(demo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} planner_map_interfaces_generate_messages_cpp)
# target_link_libraries(demo_node ${catkin_LIBRARIES} Eigen3::Eigen)

# ############
# # Install ##
# ############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# # Mark executable scripts (Python etc.) for installation
# # in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
# scripts/my_python_script
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # Mark executables for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
# RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_visualizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # Mark libraries for installation
# # See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
message(STATUS "LIBRARY DESTINATION is ${CATKIN_PACKAGE_LIB_DESTINATION}")
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# # Mark cpp header files for installation
message(STATUS "Installing to ${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
# # myfile1
# # myfile2
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# ############
# # Testing ##
# ############

# # Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ipp_belief.cpp)
# if(TARGET ${PROJECT_NAME}-test)
# target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)
