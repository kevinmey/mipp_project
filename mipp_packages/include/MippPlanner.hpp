#pragma once

#include <ros/ros.h>

#include "octomap/octomap.h"

#include <utils.hpp>

#include "mipp_msgs/ExplorationResult.h"
#include "mipp_msgs/ExplorationPath.h"
#include "mipp_msgs/ExplorationPose.h"
#include <mipp_msgs/StartExplorationAction.h>
#include <mipp_msgs/MoveVehicleAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <cmath> /* sqrt, pow */
#include <algorithm>    // std::min

//

enum VehicleState { INIT, IDLE, PLANNING, MOVING, RECOVERING };

struct UGVPlanner
{
  // Publishers
  ros::Publisher pub_goal_;
  ros::Publisher pub_goal_path_;
  // Vehicle planner state
  void updateStateMachine();
  VehicleState vehicle_state;
  // Exploration (Frontier exploration using RRT)
  void sendExplorationGoal(float exploration_time);
  bool isExplorationDone();
  mipp_msgs::ExplorationResult getExplorationResult();
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* exploration_client;
  mipp_msgs::StartExplorationGoal exploration_goal;
  mipp_msgs::ExplorationResult exploration_result;
  // Navigation
  void createInitNavigationPlan();
  void getRealNavigationPlan();
  void createNavigationWaypoints(int nr_of_ugv_nav_waypoints, bool add_nav_waypoint_at_goal);
  std::vector<SensorCircle> getSensorCoverage(float ugv_sensor_radius);
  geometry_msgs::PoseStamped navigation_goal;
  nav_msgs::Path navigation_path_init;  // Initial RRT vertices making up "path" to frontier node goal
  nav_msgs::Path navigation_path;       // Plan returned from UGVPlanner which optimizes the inital path
  std::vector<geometry_msgs::Point> navigation_waypoints;  // Waypoints created from poses on path which are within a set distance
  float navigation_waypoint_max_dist;
  bool navigation_paused;
  // Communication
};

struct UAVPlanner
{
  // General
  int uav_id;
  // Vehicle planner state
  void updateStateMachine();
  VehicleState vehicle_state;
  // Exploration (Informative exploration using RRT)
  void sendExplorationGoal(float exploration_time);
  bool isExplorationDone();
  mipp_msgs::ExplorationResult getExplorationResult();
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* exploration_client;
  mipp_msgs::StartExplorationGoal exploration_goal;
  mipp_msgs::ExplorationResult exploration_result;
  // Navigation
  void createNavigationPlan(std::vector<SensorCircle> sensor_coverages, float uav_camera_range);
  void sendMoveVehicleGoal(float move_vehicle_time);
  std::vector<SensorCircle> sensor_coverage;
  actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* move_vehicle_client;
  mipp_msgs::MoveVehicleGoal move_vehicle_goal;
  geometry_msgs::PoseStamped navigation_goal;
  nav_msgs::Path navigation_path;
};

/* MOVED DEFINITION TO UTILS.HPP
struct SensorCircle
{
  // Details of which vehicle the sensor circle is assigned to
  int vehicle_id;   // -1 for UGV, uav_id (0, 1, ...) for UAVs
  geometry_msgs::Pose vehicle_pose; // For UGV, pose.position = circle center
  // Geometric characteristics of circle
  geometry_msgs::Point center;
  float radius;
};*/
 
class MippPlanner
{
public:
  // Constructor
  MippPlanner(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~MippPlanner();
  
private:
  // Functions
  // Publish functions for publishers
  void pubUGVPauseNavigation();
  // Callback functions for subscriptions
  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);
  void subUGVPlan(const nav_msgs::PathConstPtr& path_msg);
  // Planner functions
  void runStateMachine();
  void makePlanIndividual(int vehicle_id);
  void makePlanSynchronous();
  // Utility functions
  void getParams(ros::NodeHandle np);
  // Visualization functions
  void visualizeSensorCircle(SensorCircle sensor_circle);
  void visualizeSensorCoverages();
  void visualizePaths(std::vector<nav_msgs::Path> paths);
  void visualizePathFOVs(std::vector<nav_msgs::Path> paths, float ray_length);

  // Publishers
  ros::Publisher pub_ugv_pause_navigation_;
  ros::Timer pub_timer_pause_navigation_;
  ros::Publisher pub_viz_sensor_circle_;
  ros::Publisher pub_viz_sensor_coverages_;
  ros::Publisher pub_viz_uav_paths_;
  ros::Publisher pub_viz_uav_path_fovs_;
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  ros::Subscriber sub_ugv_goal_plan_;
  // Actionlib
  // Parameters
  std::string planner_world_frame_;
  //// General
  bool do_visualization_;
  //// UGV
  std::string ugv_ns_;
  int nr_of_ugv_nav_waypoints_;
  float ugv_nav_waypoint_max_distance_;
  bool add_nav_waypoint_at_goal_;
  float ugv_sensor_radius_;
  //// UAV
  int nr_of_uavs_;
  std::string uav_world_frame_;
  float uav_sensor_range_;
  float uav_camera_width_;
  float uav_camera_height_;
  float uav_camera_hfov_;
  float uav_camera_ray_resolution_;
  float uav_camera_range_;
  std::vector<tf2::Vector3> uav_camera_corner_rays_;
  // Variables
  bool running_exploration_;
  //// Collaborative
  std::vector<SensorCircle> sensor_coverages_;
  //// UGV
  UGVPlanner ugv_planner_;
  //// UAVs
  std::vector<UAVPlanner> uav_planners_;
};
