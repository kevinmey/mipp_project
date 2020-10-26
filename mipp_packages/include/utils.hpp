/**
* @file utils.hpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Utility functions for the mipp_packages ros package
*/

#include <ros/ros.h>

#include "mipp_msgs/ExplorationPath.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>

#include <string>
#include <sstream>
#include <sys/stat.h>
#include <math.h> /* sqrt, pow */

struct SensorCircle
{
  // Details of which vehicle the sensor circle is assigned to
  int vehicle_id;   // -1 for UGV, uav_id (0, 1, ...) for UAVs
  geometry_msgs::Pose vehicle_pose; // For UGV, pose.position = circle center
  // Geometric characteristics of circle
  geometry_msgs::Point center;
  float radius;
};

double getDistanceBetweenPoints(geometry_msgs::Point p1, geometry_msgs::Point p2);

double getDistanceBetweenAngles(double angle_1, double angle_2);

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Point const direction, double distance);

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Vector3 const direction, double distance);

double getClosestYaw(double from_yaw, double to_yaw, double max_yaw_delta);

geometry_msgs::Vector3 getDirection(geometry_msgs::Point const from_point, geometry_msgs::Point const to_point);

geometry_msgs::Point normalize(geometry_msgs::Point const point);

geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 const vector);

geometry_msgs::Vector3 makeVector3(double x, double y, double z);

geometry_msgs::Point makePoint(double x, double y, double z);

int resolveUri(std::string& uri);

// Planner functions

  float calculateSensorCoverageOverlap(SensorCircle circle_a, SensorCircle circle_b);

  nav_msgs::Path makePathFromExpPath(mipp_msgs::ExplorationPath);

  geometry_msgs::Vector3 makeRPYFromQuat(geometry_msgs::Quaternion quat);

  geometry_msgs::Quaternion makeQuatFromRPY(geometry_msgs::Vector3 rpy);

  SensorCircle makeSensorCircleFromUAVPose(geometry_msgs::Pose uav_pose, int uav_id, float sensor_range);