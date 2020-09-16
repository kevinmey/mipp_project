#pragma once

#include <ros/ros.h>

#include <UGVFrontierExplorer.hpp>
#include <UAVInformativeExplorer.hpp>
 
class JointExplorer
{
public:
  // Constructor
  JointExplorer(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~JointExplorer();
  
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
 bool startUGVExploration();
 bool startUAVExploration();
  
  /* 
  *  Utility functions
  */
  void getParams(ros::NodeHandle np);
  void initUGVExplorer(ros::NodeHandle n, ros::NodeHandle np);
  void initUAVExplorer(ros::NodeHandle n, ros::NodeHandle np, int uav_id);
  
private:
  // Publishers
  //// UGV
  //// UAVs
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  // TF
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener* tf_listener_; 
  // Random nr. generator and distributions
  // std::default_random_engine generator_;
  // std::uniform_real_distribution<double> unit_distribution_;
  // Parameters
  int nr_of_uavs_;
  // Common planner
  double planner_rate_;
  double planner_max_time_;
  double planner_max_ray_distance_;
  bool planner_unmapped_is_collision_;
  //// UGV
  bool use_dynamic_range_;
  double dynamic_range_padding_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  std::string planner_world_frame_;
  double planner_min_distance_to_frontier_;
  double planner_max_distance_to_frontier_;
  int collision_threshold_;
  //// UAV
  std::string uav_world_frame_;
  std::string uav_local_frame_;
  std::string uav_body_frame_;
  double uav_start_x_;
  double uav_start_y_;
  double uav_takeoff_z_;
  double uav_camera_width_;
  double uav_camera_height_;
  double uav_camera_hfov_;
  double uav_camera_ray_resolution_;
  double uav_camera_range_;
  double planner_sample_radius_;
  double planner_max_neighbor_distance_;
  double planner_max_neighbor_yaw_;
  // Variables
  //// UGV
  UGVFrontierExplorer* ugv_explorer_;
  bool running_joint_exploration_;
  //// UAV
  std::vector<UAVInformativeExplorer*> uav_explorers_;
};
