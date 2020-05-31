cmake_minimum_required(VERSION 2.8.3)

##CHANGE THIS TO YOUR PACKAGE NAME
project(mipp_packages)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(
    catkin REQUIRED 
    COMPONENTS
    geometry_msgs
    std_msgs
    roscpp
    cmake_modules
    octomap_ros
    octomap_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_core
    base_local_planner
)
## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    DEPENDS
    LIBRARIES base_local_planner
)
## Includes headers in include folder
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## common

add_library(
    common_libs
    src/common/utils.cpp
)

### ###
# UGV #
### ###

## ugv_server_node

add_executable(
    ugv_server_node
    src/ugv/ugv_server_node.cpp
    src/ugv/UGVServer.cpp
    src/ugv/UGVServerVisualization.cpp
)

add_dependencies(
    ugv_server_node 
    ${ugv_server_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ugv_server_node
    common_libs
    ${catkin_LIBRARIES}
)

## ugv_planner_plugin
# global planner plugin for navigation

add_library(
    ugv_planner_lib
    src/ugv/UGVPlanner.cpp
    src/ugv/UGVPlannerVisualization.cpp
)

target_link_libraries(
    ugv_planner_lib
    common_libs
    ${catkin_LIBRARIES}
)

### ###
# UAV #
### ###

## uav_server_node

add_executable(
    uav_server_node
    src/uav/uav_server_node.cpp
    src/uav/UAVServer.cpp
)

add_dependencies(
    uav_server_node 
    ${uav_server_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    uav_server_node
    common_libs
    ${catkin_LIBRARIES}
)

###   ###
# Other #
###   ###

## rrt_planner_node

add_executable(
    rrt_planner_node
    src/other/rrt_planner_node.cpp
    src/other/RRTPlanner.cpp
    src/other/RRTPlannerVisualization.cpp
)

add_dependencies(
    rrt_planner_node 
    ${rrt_planner_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    rrt_planner_node
    common_libs
    ${catkin_LIBRARIES}
)

## ground_truth_handler_node

add_executable(
    ground_truth_handler_node
    src/common/ground_truth_handler_node.cpp
)

add_dependencies(
    ground_truth_handler_node 
    ${ground_truth_handler_node} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ground_truth_handler_node
    common_libs
    ${catkin_LIBRARIES}
)

## octomap_clearer_node

add_executable(
    octomap_clearer_node
    src/common/octomap_clearer_node.cpp
)

add_dependencies(
    octomap_clearer_node 
    ${octomap_clearer_node} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    octomap_clearer_node
    common_libs
    ${catkin_LIBRARIES}
)