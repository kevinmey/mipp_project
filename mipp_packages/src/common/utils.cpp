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

double getClosestYaw(double from_yaw, double to_yaw, double max_yaw_delta)
{
  double max_yaw = from_yaw + max_yaw_delta;
  double min_yaw = from_yaw - max_yaw_delta;
  double yaw_distance_to_node = angles::shortest_angular_distance(from_yaw, to_yaw);
  double new_yaw = from_yaw + yaw_distance_to_node;
  if (new_yaw < min_yaw) {
    new_yaw = min_yaw;
  }
  else if (new_yaw > max_yaw) {
    new_yaw = max_yaw;
  }
  ROS_DEBUG("Utils: Previous yaw: %f, Goal yaw: %f, New yaw: %f", angles::to_degrees(from_yaw), angles::to_degrees(to_yaw), angles::to_degrees(new_yaw));
  ROS_DEBUG("Utils: Distance: %f, Limits: [%f, %f]", angles::to_degrees(yaw_distance_to_node), angles::to_degrees(min_yaw), angles::to_degrees(max_yaw));
  return angles::normalize_angle(new_yaw);
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

geometry_msgs::Point makePoint(double x, double y, double z)
{
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::Vector3 makeVector3(double x, double y, double z)
{
  geometry_msgs::Vector3 vector;
  vector.x = x;
  vector.y = y;
  vector.z = z;
  return vector;
}

int resolveUri(std::string& uri) {
  // Iterate through all locations in GAZEBO_MODEL_PATH
  char* gazebo_model_path = getenv("GAZEBO_MODEL_PATH");
  char* home = getenv("HOME");
  uri = uri.substr(7, std::string::npos);
  std::stringstream all_locations(gazebo_model_path, std::ios_base::app | std::ios_base::out | std::ios_base::in);
  all_locations << ":" << home << "/.gazebo/models";
  std::string current_location;
  while (getline(all_locations, current_location, ':')) {
    struct stat s;
    std::string temp = current_location + uri;
    if (stat(temp.c_str(), &s) == 0) {
      if (s.st_mode & S_IFREG)  // this path describes a file
      {
        uri = "file://" + current_location + uri;
        return 0;
      }
    }
  }
  return 1;
}