#pragma once

#include <ros/ros.h>

#include "octomap/octomap.h"

#include <utils.hpp>

#include "mipp_msgs/ExplorationResult.h"
#include "mipp_msgs/ExplorationPath.h"
#include "mipp_msgs/ExplorationPose.h"
#include "mipp_msgs/StartExplorationAction.h"
#include "mipp_msgs/MoveVehicleAction.h"
#include "mipp_msgs/StartMippAction.h"
#include "mipp_msgs/TakeoffComplete.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <angles/angles.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"

#include <string>
#include <cmath> /* sqrt, pow */
#include <algorithm>    // std::min
#include <random> 

//

enum VehicleState { INIT=-1, IDLE=0, PLANNING=1, MOVING=2, ESCORTING=3, RECOVERING=10, DONE=100 };

struct UGVPlanner
{
  // Publishers
  ros::Publisher pub_goal_;
  ros::Publisher pub_goal_path_;
  // Subscribers
  void subOdometry(const nav_msgs::OdometryConstPtr& odom_msg);
  ros::Subscriber sub_odometry;
  // General vehicle variables
  void init(ros::NodeHandle n);
  std::shared_ptr<nav_msgs::Odometry> ugv_odometry;
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
  float navigation_goal_distance;
  float navigation_waypoint_max_dist;
  bool navigation_paused;
  // Communication
};

struct UAVPlanner
{
  // Publishers
  ros::Publisher pub_position_goal;
  // Subscribers
  void subOdometry(const nav_msgs::OdometryConstPtr& odom_msg);
  ros::Subscriber sub_odometry;
  // General vehicle parameters
  void init(ros::NodeHandle n);
  float uav_altitude;
  int uav_id;
  float com_range;
  std::shared_ptr<octomap::OcTree> octomap;
  // General vehicle variables
  nav_msgs::Odometry uav_odometry;
  // Global Info (Stored in MippPlanner object)
  bool* global_run_exploration;
  bool* global_run_escorting;
  std::shared_ptr<nav_msgs::Odometry> global_ugv_odometry;
  std::vector<geometry_msgs::Point>* global_ugv_waypoints;
  std::map<int, std::vector<SensorCircle>>* global_sensor_coverages;
  std::map<int, nav_msgs::Path>* global_uav_paths;
  // Vehicle planner state
  void updateStateMachine();
  VehicleState vehicle_state;
  ros::Timer state_timer;
  ros::ServiceClient takeoff_client;
  // Recovery behaviour
  bool recoveryRequired();
  void prepareForRecovery();
  void sendRecoverVehicleGoal();
  bool no_viable_plan;
  bool out_of_com_range;
  geometry_msgs::PoseStamped recovery_goal;
  // Exploration (Informative exploration using RRT)
  void sendExplorationGoal(float exploration_time);
  bool isExplorationDone();
  mipp_msgs::ExplorationResult getExplorationResult();
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* exploration_client;
  mipp_msgs::StartExplorationGoal exploration_goal;
  mipp_msgs::ExplorationResult exploration_result;
  // Navigation
  std::vector<SensorCircle> getExistingSensorCoverages();
  void createNavigationPlan(std::vector<SensorCircle> sensor_coverages, float uav_camera_range);
  void sendMoveVehicleGoal(float move_vehicle_time);
  std::vector<SensorCircle> sensor_coverage;
  float camera_range;
  actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* move_vehicle_client;
  mipp_msgs::MoveVehicleGoal move_vehicle_goal;
  geometry_msgs::PoseStamped navigation_goal;
  nav_msgs::Path navigation_path;
  // Escort formation planner
  void initFormationPoseBank(int nr_of_uavs);
  geometry_msgs::Point getRandomCirclePoint(geometry_msgs::Point circle_center = makePoint(0,0,0), float circle_radius = 1.0);
  float getRandomYaw(float yaw_deg_center = 0.0, float yaw_deg_range = 30.0);
  geometry_msgs::PoseStamped getEscortPose(const geometry_msgs::Pose& ugv_pose, const geometry_msgs::Pose& uav_formation_pose);
  nav_msgs::Path getEscortPath(const std::vector<geometry_msgs::Point>& ugv_waypoints, const geometry_msgs::Pose& uav_formation_pose);
  nav_msgs::Path escort_path;
  geometry_msgs::Pose formation_pose;
  std::vector<geometry_msgs::Pose> formation_poses;
  int formation_pose_idx;
  // Rng
  float sample_radius;
  float sample_yaw_range;
  std::default_random_engine rng_generator;
  std::uniform_real_distribution<double> rng_unit_distribution;
  // LOS
  bool doPointsHaveLOS(const geometry_msgs::Point point_a, const geometry_msgs::Point point_b);
  // Collision check
  void initCollisionPoints();
  bool isPoseCollisionFree(const geometry_msgs::Point& pose, bool unmapped_is_collision = false);
  bool isPathCollisionFree(const nav_msgs::Path& path);
  std::vector<octomap::point3d> collision_points;
  // Info gain
  float getPoseInfoGain(geometry_msgs::Point origin, float yaw);
  float getPathInfoGain(const nav_msgs::Path& path, const std::vector<SensorCircle>& other_sensor_coverages, 
                        std::vector<SensorCircle>& path_sensor_coverages);
  float getPathInfoGain(const mipp_msgs::ExplorationPath& path, const std::vector<SensorCircle>& sensor_coverages);
  std::vector<tf2::Vector3> info_camera_rays;
  // Sampled formation reshaping
  /*geometry_msgs::Pose getSampledPose(const geometry_msgs::Pose& current_pose);
  std::default_random_engine rng_generator;
  std::uniform_real_distribution<double> rng_unit_distribution;
  float sample_radius_max;
  float sample_yaw_radian_max;*/
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
  void runUpdates();
  // Callback functions for subscriptions
  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);
  void subUGVPlan(const nav_msgs::PathConstPtr& path_msg);
  void subOctomap(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
  // Services
  bool cliIsPlannerReady(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  // Actionlib
  void actMipp(const mipp_msgs::StartMippGoalConstPtr &goal);
  // Planner functions
  void runStateMachine();
  void makePlanIndividual(int vehicle_id);
  void makePlanSynchronous();
  // Utility functions
  void getParams(ros::NodeHandle np);
  geometry_msgs::Pose getFormationPose(int uav_id);
  bool doPointsHaveLOS(const geometry_msgs::Point point_a, const geometry_msgs::Point point_b);
  // Visualization functions
  void visualizeSensorCircle(SensorCircle sensor_circle);
  void visualizeSensorCoverages(std::vector<SensorCircle> sensor_coverages);
  void visualizePaths(std::vector<nav_msgs::Path> paths);
  void visualizePathFOVs(std::vector<nav_msgs::Path> paths, float ray_length);
  void visualizeNavWaypoints();

  // Publishers
  ros::Timer tmr_run_updates_;
  ros::Timer tmr_formation_reshape_planner_;
  ros::Publisher pub_ugv_pause_navigation_;
  ros::Publisher pub_viz_sensor_circle_;
  ros::Publisher pub_viz_sensor_coverages_;
  ros::Publisher pub_viz_uav_paths_;
  ros::Publisher pub_viz_uav_path_fovs_;
  ros::Publisher pub_viz_nav_waypoints_;
  ros::Publisher pub_viz_formations_;
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  ros::Subscriber sub_ugv_goal_plan_;
  ros::Subscriber sub_octomap_;
  // Actionlib
  ros::ServiceServer cli_planner_ready_;
  actionlib::SimpleActionServer<mipp_msgs::StartMippAction> act_mipp_server_;
  mipp_msgs::StartMippGoal act_mipp_goal_;
  mipp_msgs::StartMippFeedback act_mipp_feedback_;
  mipp_msgs::StartMippResult act_mipp_result_;
  // Parameters
  std::string planner_world_frame_;
  //// General
  bool do_visualization_;
  float com_range_;
  float com_range_padding_;
  float planner_hybrid_distance_;
  //// Planner
  void reshapeFormationUpdate();
  std::vector<geometry_msgs::Pose> getRandomFormation(const std::vector<geometry_msgs::Pose>& current_formation, 
                                                      float euc_range, float yaw_range);
  std::vector<geometry_msgs::Pose> getRandomColFreeFormation(const std::vector<geometry_msgs::Pose>& current_formation, 
                                                      float euc_range, float yaw_range);
  geometry_msgs::Point getRandomCirclePoint(geometry_msgs::Point circle_center = makePoint(0,0,0), float circle_radius = 1.0);
  float getRandomYaw(float yaw_center = 0.0, float yaw_range = M_PI/6.0);
  float getFormationInfoGain(const std::vector<geometry_msgs::Pose>& formation_poses);
  bool isFormationCollisionFree(const std::vector<geometry_msgs::Pose>& formation_poses);
  float getDistanceBetweenFormations(const std::vector<geometry_msgs::Pose>& current_formation, const std::vector<geometry_msgs::Pose>& other_formation);
  void getDistanceBetweenFormations(const std::vector<geometry_msgs::Pose>& current_formation, const std::vector<geometry_msgs::Pose>& other_formation,
                                               float& ret_euc_distance, float& ret_yaw_distance);
  geometry_msgs::Pose getEscortPose(const geometry_msgs::Pose& formation_pose);
  geometry_msgs::Pose getEscortPose(const geometry_msgs::Pose& formation_pose, const geometry_msgs::Pose& ugv_pose);
  bool isFormationComConstrained(std::vector<geometry_msgs::Pose> formation);
  float getFormationUtility(const std::vector<geometry_msgs::Pose>& formation, const std::vector<geometry_msgs::Pose>& current_formation);
  void visualizeFormations(std::map<float, std::vector<geometry_msgs::Pose>, std::greater<float>> formations);
  float sample_radius_;
  float sample_yaw_range_;
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> unit_distribution_;
  std::map<float, std::vector<geometry_msgs::Pose>, std::greater<float>> formation_bank_;
  std::vector<geometry_msgs::Pose> current_formation_;
  //// Utility
  float c_info;
  float c_euc_dist; // pr m
  float c_yaw_dist; // pr PI/3 = 60 deg
  //// UGV
  std::string ugv_ns_;
  int nr_of_ugv_nav_waypoints_;
  float ugv_nav_waypoint_max_distance_;
  bool add_nav_waypoint_at_goal_;
  float ugv_sensor_radius_;
  //// UAV
  int nr_of_uavs_;
  std::string uav_world_frame_;
  float uav_camera_width_;
  float uav_camera_height_;
  float uav_camera_hfov_;
  float uav_camera_ray_resolution_;
  float uav_camera_range_;
  std::vector<tf2::Vector3> uav_camera_corner_rays_;
  // Variables
  bool run_exploration_;
  bool run_escorting_;
  bool run_hybrid_;
  std::shared_ptr<octomap::OcTree> octomap_; 
  bool received_octomap_;
  int octomap_size_;
  bool planner_initialized_;
  //// Collaborative
  std::map<int, std::vector<SensorCircle>> uav_sensor_coverages_;
  std::map<int, nav_msgs::Path> uav_paths_;
  //// UGV
  float ugv_start_x_;
  float ugv_start_y_;
  UGVPlanner ugv_planner_;
  //// UAVs
  std::vector<UAVPlanner> uav_planners_;
};
