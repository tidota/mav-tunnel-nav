cmake_minimum_required(VERSION 2.8.3)
project(mav_tunnel_nav)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  gazebo_ros
  gazebo_plugins
  octomap_ros
  octomap_msgs
  roscpp
  rospy
  std_msgs
  message_generation

  laser_geometry
  nav_msgs
  visualization_msgs
  dynamic_reconfigure
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(gazebo REQUIRED)
find_package(ignition-transport4 REQUIRED)

# PCL
find_package(PCL 1.7 REQUIRED)

# Octomap
find_package(octomap REQUIRED)

# OpenCV (just for odometry plugin as of 12/29/2020)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Particles.msg
  Beacon.msg
  SrcDst.msg
  OctomapWithSegId.msg
  Submap.msg
  SubmapAck.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SpawnRobot.srv
#   Service1.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  octomap_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadrotor_tunnel_nav
  CATKIN_DEPENDS
    message_runtime gazebo_ros roscpp rospy std_msgs
    visualization_msgs nav_msgs sensor_msgs mav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
  INCLUDE_DIRS
  LIBRARIES
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

# Create a static library with the Protobuf messages used internally.
set(protobuf_lib_name MavTunnelNavProtobuf)
add_subdirectory(src/protobuf)
set_source_files_properties(${PROTO_SOURCES} ${PROTO_HEADERS}
                            PROPERTIES GENERATED TRUE)
add_library(${protobuf_lib_name} STATIC ${PROTO_SOURCES})
add_dependencies(${protobuf_lib_name} protobuf_compilation)
set_target_properties(${protobuf_lib_name}
  PROPERTIES POSITION_INDEPENDENT_CODE ON COMPILE_FLAGS "-Wno-unused-parameter")

# Create the libAdHocNetPlugin.so library.
set(adhocnet_plugin_name AdHocNetPlugin)
add_library(${adhocnet_plugin_name} src/sim_plugins/AdHocNetPlugin.cc)
add_dependencies(${adhocnet_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${adhocnet_plugin_name}
  ${catkin_LIBRARIES}
  ${ignition-transport4_LIBRARIES}
)

# Create the libdrift_odometry_plugin.so library.
set(drift_odometry_plugin_name drift_odometry_plugin)
add_library(${drift_odometry_plugin_name} src/sim_plugins/drift_odometry_plugin.cpp)
add_dependencies(${drift_odometry_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${drift_odometry_plugin_name}
  ${catkin_LIBRARIES}
  ${protobuf_lib_name}
  protobuf::libprotobuf
  ${ignition-transport4_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(${drift_odometry_plugin_name} protobuf_compilation)


add_executable(auto_pilot src/auto_pilot.cpp)
add_dependencies(auto_pilot
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_pilot
  ${catkin_LIBRARIES}
)

add_executable(imu_odom src/imu_odom.cpp)
add_dependencies(imu_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_odom ${catkin_LIBRARIES})

add_executable(rbpf src/rbpf.cpp src/particle.cpp)
add_dependencies(rbpf
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rbpf
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(beacon src/beacon.cpp)
add_dependencies(beacon
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(beacon
  ${catkin_LIBRARIES}
)

add_executable(map_only src/map_only.cpp)
add_dependencies(map_only ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_only
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(colorchg src/colorchg.cpp)
add_dependencies(colorchg ${catkin_EXPORTED_TARGETS})
target_link_libraries(colorchg
  ${catkin_LIBRARIES}
)

add_executable(maploadvis src/maploadvis.cpp)
add_dependencies(maploadvis ${catkin_EXPORTED_TARGETS})
target_link_libraries(maploadvis
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

add_executable(teleop_node src/teleop_node.cpp)
add_dependencies(teleop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(teleop_node ${catkin_LIBRARIES})

add_executable(map_saver src/map_saver.cpp)
add_dependencies(map_saver
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_saver ${catkin_LIBRARIES})

add_executable(save_all_server src/save_all_server.cpp)
add_dependencies(save_all_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(save_all_server ${catkin_LIBRARIES})

#add_executable(octomap_test src/octomap_test.cpp)
#add_dependencies(octomap_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(octomap_test ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

## Declare a C++ library
# add_library(quadrotor_tunnel_nav
#   src/${PROJECT_NAME}/quadrotor_tunnel_nav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(quadrotor_tunnel_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(quadrotor_tunnel_nav_node src/quadrotor_tunnel_nav_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(quadrotor_tunnel_nav_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(quadrotor_tunnel_nav_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS quadrotor_tunnel_nav quadrotor_tunnel_nav_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_quadrotor_tunnel_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
