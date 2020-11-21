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

double getDistanceBetweenAngles(double angle_1, double angle_2)
{
	return std::abs(angles::shortest_angular_distance(angle_1, angle_2));
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
  ROS_DEBUG("Direction from (%.2f, %.2f) to (%.2f, %.2f): (%.2f, %.2f, %.2f)", from_point.x, to_point.x, from_point.y, to_point.y, direction.x, direction.y, direction.z);
  return normalize(direction);
}

geometry_msgs::Point getRotatedPoint(double rotation_angle_rad, geometry_msgs::Point point, geometry_msgs::Point rotation_axis)
{
  if (std::abs(rotation_angle_rad) < 0.01){
    // Rotation is so small that it is virtually the same point
    return point;
  }

  geometry_msgs::Point rotated_point = makePoint(point.x*std::cos(rotation_angle_rad) 
                                               - point.y*std::sin(rotation_angle_rad),
                                                 point.x*std::sin(rotation_angle_rad) 
                                               + point.y*std::cos(rotation_angle_rad),
                                                 point.z);
  return makePoint(rotated_point.x + rotation_axis.x, rotated_point.y + rotation_axis.y, point.z);
}

geometry_msgs::Point normalize(geometry_msgs::Point const point)
{
  double norm = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0) + pow(point.z, 2.0));
  if (norm < 0.0001) {
    return makePoint(1, 0, 0);
  }
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

// Planner functions

float calculateSensorCoverageOverlap(SensorCircle circle_a, SensorCircle circle_b) {
  ROS_DEBUG("calculateSensorCoverageOverlap");
  
  // First check easiest cases relating to the distance between the circles
  float d = getDistanceBetweenPoints(circle_a.center, circle_b.center);
  if (d >= circle_a.radius + circle_b.radius) {
    // Circles are not overlapping, return 0 overlap
    return 0.0;
  } 
  else if (circle_b.radius + d <= circle_a.radius) {
    // Circle B is smaller than A and is entirely contained within it
    return M_PI*pow(circle_b.radius, 2);
  }
  else if (circle_a.radius + d <= circle_b.radius) {
    // Circle A is smaller than B and is entirely contained within it
    return M_PI*pow(circle_a.radius, 2);
  }

  // Circles have partial overlap, must calculate overlap
  // Source: https://diego.assencio.com/?index=8d6ca3d82151bad815f78addf9b5c1c6
  float r_1 = (circle_a.radius > circle_b.radius) ? circle_a.radius : circle_b.radius;  // The larger circle radius
  float r_2 = (circle_a.radius > circle_b.radius) ? circle_b.radius : circle_a.radius;  // The smaller circle radius
  float d_1 = (pow(r_1, 2) - pow(r_2, 2) + pow(d, 2)) / (2.0*d);
  float d_2 = d - d_1;

  float a_intersection = pow(r_1, 2)*acos(d_1/r_1) - d_1*sqrt(pow(r_1, 2) - pow(d_1, 2))
                       + pow(r_2, 2)*acos(d_2/r_2) - d_2*sqrt(pow(r_2, 2) - pow(d_2, 2));

  return a_intersection;
}

nav_msgs::Path makePathFromExpPath(mipp_msgs::ExplorationPath exp_path) {
  ROS_DEBUG("makePathFromExpPath");
  nav_msgs::Path path;
  for (auto exp_path_it = exp_path.poses.begin(); exp_path_it != exp_path.poses.end(); ++exp_path_it) {
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = "world";
    path_pose.header.stamp = ros::Time::now();
    path_pose.pose = exp_path_it->pose;
    path.poses.push_back(path_pose);
  }
  ROS_DEBUG("Made path from: (%f, %f, %f) to (%f, %f, %f)", 
           path.poses.begin()->pose.position.x, path.poses.begin()->pose.position.y, path.poses.begin()->pose.position.z,
           path.poses.rbegin()->pose.position.x, path.poses.rbegin()->pose.position.y, path.poses.rbegin()->pose.position.z);
  return path;
}

geometry_msgs::Vector3 makeRPYFromQuat(geometry_msgs::Quaternion quat) {
  ROS_DEBUG("makeRPYFromQuat");
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  geometry_msgs::Vector3 rpy;
  tf2::Matrix3x3(tf_quat).getRPY(rpy.x, 
                                 rpy.y, 
                                 rpy.z);
  return rpy;
}

geometry_msgs::Quaternion makeQuatFromRPY(geometry_msgs::Vector3 rpy) {
  ROS_DEBUG("makeQuatFromRPY (%.2f, %.2f, %.2f)", rpy.x, rpy.y, rpy.z);
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(rpy.x, rpy.y, rpy.z);
  geometry_msgs::Quaternion quat;
  quat.x = tf_quat.x();
  quat.y = tf_quat.y();
  quat.z = tf_quat.z();
  quat.w = tf_quat.w();
  return quat;
}

SensorCircle makeSensorCircleFromUAVPose(geometry_msgs::Pose uav_pose, int uav_id, float sensor_range) {
  ROS_DEBUG("makeSensorCircleFromUAVPose");
  SensorCircle sensor_circle;
  sensor_circle.vehicle_pose = uav_pose;
  sensor_circle.vehicle_id = uav_id;
  sensor_circle.radius = sensor_range/2.0;
  
  float uav_yaw = makeRPYFromQuat(uav_pose.orientation).z;
  sensor_circle.center.x = cos(uav_yaw)*(sensor_range/2.0 + 2.0) + uav_pose.position.x;
  sensor_circle.center.y = sin(uav_yaw)*(sensor_range/2.0 + 2.0) + uav_pose.position.y;
  sensor_circle.center.z = 0.0;

  return sensor_circle;
}