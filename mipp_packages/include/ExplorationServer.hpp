#pragma once

#include <ros/ros.h>

#include "octomap/octomap.h"

#include <utils.hpp>

#include "mipp_msgs/ExplorationResult.h"
#include "mipp_msgs/ExplorationPath.h"
#include "mipp_msgs/ExplorationPose.h"
#include <mipp_msgs/StartExplorationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/PointStamped.h>

#include <string>
#include <cmath> /* sqrt, pow */
 
class ExplorationServer
{
public:
  // Constructor
  ExplorationServer(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~ExplorationServer();
  
  /* 
  *  Publish functions for publishers
  */
  
  /* 
  *  Callback functions for subscriptions
  */
  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);

  /* 
  *  Planner functions
  */
  
  /* 
  *  Utility functions
  */
  void getParams(ros::NodeHandle np);
  
private:
  // Publishers
  //// UGV
  //// UAVs
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  // Actionlib
  actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* act_ugv_exploration_client_;
  //std::vector<actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>*> act_uav_exploration_clients_;
  // Parameters
  int nr_of_uavs_;
  std::string ugv_ns_;
  // Variable
  bool running_exploration_;
};
