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
)
## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
    DEPENDS
)
## Includes headers in include folder
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## common

add_library(
    common_libs
    src/Node.cpp
    src/utils.cpp
)

## uav_server_node

add_executable(
    uav_server_node
    src/uav_server_node.cpp
    src/UAVServer.cpp
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

## rrt_planner_node

add_executable(
    rrt_planner_node
    src/rrt_planner_node.cpp
    src/RRTPlanner.cpp
    src/RRTPlannerVisualization.cpp
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

# ugv_planner_node

add_executable(
    ugv_planner_node
    src/ugv_planner_node.cpp
    src/UGVPlanner.cpp
    src/UGVPlannerVisualization.cpp
)

add_dependencies(
    ugv_planner_node 
    ${ugv_planner_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
    ugv_planner_node
    common_libs
    ${catkin_LIBRARIES}
)