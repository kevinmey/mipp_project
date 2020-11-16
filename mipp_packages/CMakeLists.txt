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
    message_generation
    actionlib_msgs
    mipp_msgs
    image_transport
    cv_bridge
)

###################################
## catkin specific configuration ##
###################################

## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp message_runtime actionlib_msgs mipp_msgs
    DEPENDS
    LIBRARIES base_local_planner
)

###########
## Build ##
###########

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

###                ###
# Mipp Planner #
###                ###

## mipp_planner_node

add_executable(
    mipp_planner_node
    src/joint/mipp_planner_node.cpp
    src/joint/MippPlanner.cpp
    src/joint/UGVPlanner.cpp
    src/joint/UAVPlanner.cpp
)

add_dependencies(
    mipp_planner_node
    ${mipp_planner_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    mipp_planner_node
    common_libs
    ${catkin_LIBRARIES}
)

### ###
# UGV #
### ###

## ugv_frontier_explorer_node

add_executable(
    ugv_frontier_explorer_node
    src/ugv/ugv_frontier_explorer_node.cpp
    src/ugv/UGVFrontierExplorer.cpp
)

add_dependencies(
    ugv_frontier_explorer_node
    ${ugv_frontier_explorer_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ugv_frontier_explorer_node
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

## uav_informative_explorer_node

add_executable(
    uav_informative_explorer_node
    src/uav/uav_informative_explorer_node.cpp
    src/uav/UAVInformativeExplorer.cpp
)

add_dependencies(
    uav_informative_explorer_node 
    ${uav_informative_explorer_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    uav_informative_explorer_node
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

## communication_constraint_visualizer_node

add_executable(
    communication_constraint_visualizer_node
    src/common/communication_constraint_visualizer_node.cpp
)

add_dependencies(
    communication_constraint_visualizer_node 
    ${communication_constraint_visualizer_node} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    communication_constraint_visualizer_node
    common_libs
    ${catkin_LIBRARIES}
)

## mipp_visualizer_node

add_executable(
    mipp_visualizer_node
    src/common/mipp_visualizer.cpp
)

add_dependencies(
    mipp_visualizer_node 
    ${mipp_visualizer_node} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    mipp_visualizer_node
    common_libs
    ${catkin_LIBRARIES}
)

## mipp_monitor_node

add_executable(
    mipp_monitor_node
    src/common/mipp_monitor.cpp
)

add_dependencies(
    mipp_monitor_node 
    ${mipp_monitor_node} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    mipp_monitor_node
    common_libs
    ${catkin_LIBRARIES}
)

## FSM python scripts

catkin_install_python(PROGRAMS scripts/state_machine_simple.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)