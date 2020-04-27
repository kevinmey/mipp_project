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

## Adds a ros node with name node_name
add_executable(
    uav_server_node
    src/uav_server_node.cpp
    src/UAVServer.cpp
)

## Adds a compilation dependency to node_name
add_dependencies(
    uav_server_node 
    ${uav_server_node_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

## Links node_name together with other libraries
target_link_libraries(
    uav_server_node
    ${catkin_LIBRARIES}
)