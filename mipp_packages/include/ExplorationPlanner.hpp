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
#include <std_msgs/Bool.h>

#include <string>
#include <cmath> /* sqrt, pow */

struct UGVPlanner
{
  // Exploration (Frontier exploration using RRT)
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* exploration_client;
  mipp_msgs::StartExplorationGoal exploration_goal;
  mipp_msgs::ExplorationResult exploration_result;
  // Navigation
  geometry_msgs::PoseStamped navigation_goal;
  nav_msgs::Path navigation_path;
  bool navigation_paused;
};

struct UAVPlanner
{
  // Exploration (Frontier exploration using RRT)
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* exploration_client;
  mipp_msgs::StartExplorationGoal exploration_goal;
  mipp_msgs::ExplorationResult exploration_result;
  // Navigation
  actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* move_vehicle_client;
  mipp_msgs::MoveVehicleGoal move_vehicle_goal;
  geometry_msgs::PoseStamped navigation_goal;
  nav_msgs::Path navigation_path;
};
 
class ExplorationPlanner
{
public:
  // Constructor
  ExplorationPlanner(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~ExplorationPlanner();
  
private:
  // Functions
  // Publish functions for publishers
  void pubUGVPauseNavigation();
  // Callback functions for subscriptions
  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);
  // Planner functions
  void makePlanSynchronous();
  // Utility functions
  void getParams(ros::NodeHandle np);
  nav_msgs::Path makePathFromExpPath(mipp_msgs::ExplorationPath);

  // Variables
  // Publishers
  ros::Publisher pub_ugv_goal_;
  ros::Publisher pub_ugv_goal_path_;
  ros::Publisher pub_ugv_pause_navigation_;
  ros::Timer pub_timer_pause_navigation_;
  //// UGV
  //// UAVs
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  // Actionlib
  // Parameters
  int nr_of_uavs_;
  std::string uav_world_frame_;
  std::string ugv_ns_;
  // Variables
  UGVPlanner ugv_planner_;
  std::vector<UAVPlanner> uav_planners_;
  bool running_exploration_;
};
