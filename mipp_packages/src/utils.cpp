/**
* @file utils.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Utility functions for the mipp_packages ros package
*/

#include "utils.hpp"

double getDistanceBetweenPoints(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	return sqrt( pow(p1.x - p2.x, 2.0) 
             + pow(p1.y - p2.y, 2.0)
             + pow(p1.z - p2.z, 2.0));
}

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Point const direction, double distance)
{
	geometry_msgs::Point direction_norm = normalize(direction);
  geometry_msgs::Point ray_endpoint;
  ray_endpoint.x = origin.x + distance*direction_norm.x;
  ray_endpoint.y = origin.y + distance*direction_norm.y;
  ray_endpoint.z = origin.z + distance*direction_norm.z;
  return ray_endpoint;
}

geometry_msgs::Point castRay(geometry_msgs::Point const origin, geometry_msgs::Vector3 const direction, double distance)
{
	geometry_msgs::Vector3 direction_norm = normalize(direction);
  geometry_msgs::Point ray_endpoint;
  ray_endpoint.x = origin.x + distance*direction_norm.x;
  ray_endpoint.y = origin.y + distance*direction_norm.y;
  ray_endpoint.z = origin.z + distance*direction_norm.z;
  return ray_endpoint;
}

geometry_msgs::Vector3 getDirection(geometry_msgs::Point const from_point, geometry_msgs::Point const to_point)
{
  geometry_msgs::Vector3 direction;
  direction.x = to_point.x - from_point.x;
  direction.y = to_point.y - from_point.y;
  direction.z = to_point.z - from_point.z;
  return normalize(direction);
}

geometry_msgs::Point normalize(geometry_msgs::Point const point)
{
  double norm = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));
  geometry_msgs::Point ret_point;
  ret_point.x = point.x/norm;
  ret_point.y = point.y/norm;
  ret_point.z = point.z/norm;
  return ret_point;
}

geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 const vector)
{
  double norm = sqrt(pow(vector.x, 2.0) + pow(vector.y, 2.0) + pow(vector.z, 2.0));
  geometry_msgs::Vector3 ret_vector;
  ret_vector.x = vector.x/norm;
  ret_vector.y = vector.y/norm;
  ret_vector.z = vector.z/norm;
  return ret_vector;
}