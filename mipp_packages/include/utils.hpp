/**
* @file utils.hpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Utility functions for the mipp_packages ros package
*/

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <math.h> /* sqrt, pow */

double distanceBetweenPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Point const direction, double distance);

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Vector3 const direction, double distance);

geometry_msgs::Vector3 getDirection(geometry_msgs::Point const from_point, geometry_msgs::Point const to_point);

geometry_msgs::Point normalize(geometry_msgs::Point const point);

geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 const vector);