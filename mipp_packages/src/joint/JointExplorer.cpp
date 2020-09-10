#include <JointExplorer.hpp>

// Constructor
  
JointExplorer::JointExplorer(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_INFO("JointExplorer object is being created.");

  ros::Duration(5.0).sleep(); // sleep for half a second

  // Initialize values
  getParams(np);

  ugv_explorer_ = new UGVFrontierExplorer();
  initUGVExplorer();
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    UAVInformativeExplorer* uav_explorer = new UAVInformativeExplorer();
    uav_explorers_.push_back(uav_explorer);
    initUAVExplorer(uav_id);
  }
  ROS_WARN("Done.");
}

// Destructor
  
JointExplorer::~JointExplorer() {
  ROS_INFO("JointExplorer object is being deleted.");
}

/* 
*  Callback functions for subscriptions
*/

void JointExplorer::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_INFO("subClickedPoint");
}

// Utility functions

void JointExplorer::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  // General
  np.param<int>("nr_of_uavs", nr_of_uavs_, 3);
  np.param<double>("planner_rate", planner_rate_, 100.0);
  np.param<double>("planner_max_time", planner_max_time_, 3.0);
  np.param<double>("planner_max_ray_distance", planner_max_ray_distance_, 2.0);
  np.param<bool>("planner_unmapped_is_collision", planner_unmapped_is_collision_, true);
  // UGV
  np.param<bool>("use_dynamic_range", use_dynamic_range_, true);
  np.param<double>("dynamic_range_padding", dynamic_range_padding_, 2.0);
  np.param<double>("x_range_min", x_range_min_, -10.0);
  np.param<double>("x_range_max", x_range_max_,  10.0);
  np.param<double>("y_range_min", y_range_min_, -10.0);
  np.param<double>("y_range_max", y_range_max_,  10.0);
  np.param<std::string>("planner_world_frame", planner_world_frame_, "map");
  np.param<double>("planner_min_distance_to_frontier", planner_min_distance_to_frontier_, 4.0);
  np.param<double>("planner_max_distance_to_frontier", planner_max_distance_to_frontier_, 100.0);
  np.param<int>("collision_threshold", collision_threshold_, 0);
  // UAV
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav");
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav");
  np.param<double>("uav_start_x", uav_start_x_, 0.0);
  np.param<double>("uav_start_y", uav_start_y_, 0.0);
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<double>("uav_camera_width", uav_camera_width_, 6.0);
  np.param<double>("uav_camera_height", uav_camera_height_, 3.0);
  np.param<double>("uav_camera_hfov", uav_camera_hfov_, 1.02974);
  np.param<double>("uav_camera_ray_resolution", uav_camera_ray_resolution_, 1.0);
  np.param<double>("uav_camera_range", uav_camera_range_, 7.5);
  np.param<double>("planner_sample_radius", planner_sample_radius_, 10.0);
  np.param<double>("planner_max_neighbor_distance", planner_max_neighbor_distance_, planner_max_ray_distance_);
  np.param<double>("planner_max_neighbor_yaw", planner_max_neighbor_yaw_, angles::from_degrees(90.0));
}

void JointExplorer::initUGVExplorer() {
  ROS_DEBUG("initUGVExplorer");
  ugv_explorer_->use_dynamic_range_ = use_dynamic_range_;
  ugv_explorer_->dynamic_range_padding_ = dynamic_range_padding_;
  ugv_explorer_->x_range_min_ = x_range_min_;
  ugv_explorer_->x_range_max_ = x_range_max_;
  ugv_explorer_->y_range_min_ = y_range_min_;
  ugv_explorer_->y_range_max_ = y_range_max_;
  ugv_explorer_->planner_world_frame_ = planner_world_frame_;
  ugv_explorer_->planner_rate_ = planner_rate_;
  ugv_explorer_->planner_max_time_ = planner_max_time_;
  ugv_explorer_->planner_max_ray_distance_ = planner_max_ray_distance_;
  ugv_explorer_->planner_min_distance_to_frontier_ = planner_min_distance_to_frontier_;
  ugv_explorer_->planner_max_distance_to_frontier_ = planner_max_distance_to_frontier_;
  ugv_explorer_->collision_threshold_ = collision_threshold_;
  ugv_explorer_->unmapped_is_collision_ = planner_unmapped_is_collision_;
}

void JointExplorer::initUAVExplorer(int uav_id) {
  ROS_DEBUG("initUAVExplorer(%d)", uav_id);
  uav_explorers_[uav_id]->uav_id_ = uav_id;
  uav_explorers_[uav_id]->uav_world_frame_ = uav_world_frame_;
  uav_explorers_[uav_id]->uav_local_frame_ = uav_local_frame_+std::to_string(uav_id);
  uav_explorers_[uav_id]->uav_body_frame_ = uav_body_frame_+std::to_string(uav_id);
  uav_explorers_[uav_id]->uav_start_x_ = uav_start_x_;
  uav_explorers_[uav_id]->uav_start_y_ = uav_start_y_;
  uav_explorers_[uav_id]->uav_takeoff_z_ = uav_takeoff_z_;
  uav_explorers_[uav_id]->uav_camera_width_ = uav_camera_width_;
  uav_explorers_[uav_id]->uav_camera_height_ = uav_camera_height_;
  uav_explorers_[uav_id]->uav_camera_hfov_ = uav_camera_hfov_;
  uav_explorers_[uav_id]->uav_camera_ray_resolution_ = uav_camera_ray_resolution_;
  uav_explorers_[uav_id]->uav_camera_range_ = uav_camera_range_;
  uav_explorers_[uav_id]->planner_rate_ = planner_rate_;
  uav_explorers_[uav_id]->planner_sample_radius_ = planner_sample_radius_;
  uav_explorers_[uav_id]->planner_max_time_ = planner_max_time_;
  uav_explorers_[uav_id]->planner_max_ray_distance_ = planner_max_ray_distance_;
  uav_explorers_[uav_id]->planner_max_neighbor_distance_ = planner_max_neighbor_distance_;
  uav_explorers_[uav_id]->planner_max_neighbor_yaw_ = planner_max_neighbor_yaw_;
  uav_explorers_[uav_id]->planner_unmapped_is_collision_ = planner_unmapped_is_collision_;
  uav_explorers_[uav_id]->initVariables();
}