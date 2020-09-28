#include <ExplorationPlanner.hpp>

// Constructor
  
ExplorationPlanner::ExplorationPlanner(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_INFO("ExplorationPlanner object is being created.");

  // Initialize values
  getParams(np);

  pub_ugv_goal_ = n.advertise<geometry_msgs::PoseStamped>(ugv_ns_+"move_base_simple/goal", 1);
  pub_ugv_goal_path_ = n.advertise<nav_msgs::Path>(ugv_ns_+"UGVFrontierExplorer/goal_path", 1);
  pub_ugv_pause_navigation_ = n.advertise<std_msgs::Bool>(ugv_ns_+"pause_navigation", 1);
  pub_timer_pause_navigation_  = n.createTimer(ros::Duration(0.1), boost::bind(&ExplorationPlanner::pubUGVPauseNavigation, this));

  sub_clicked_point_ = n.subscribe("/exploration/start_collaborative", 1, &ExplorationPlanner::subClickedPoint, this);
  act_ugv_exploration_client_ = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>("/ugv/exploration_action", true);
  act_ugv_exploration_client_->waitForServer(ros::Duration(10.0));
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    std::string uav_exploration_client_name = "/uav"+std::to_string(uav_id)+"/exploration_action";
    std::string uav_move_vehicle_client_name = "/uav"+std::to_string(uav_id)+"/move_vehicle_action";
    actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* act_uav_exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>(uav_exploration_client_name, true);
    actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* act_uav_move_vehicle_client = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(uav_move_vehicle_client_name, true);
    act_uav_exploration_clients_.push_back(act_uav_exploration_client);
    act_uav_move_vehicle_clients_.push_back(act_uav_move_vehicle_client);
  }

  // Set variables
  running_exploration_ = false;
  ugv_pause_navigation_ = false;

  ROS_WARN("Done.");
}

// Destructor
  
ExplorationPlanner::~ExplorationPlanner() {
  ROS_INFO("ExplorationPlanner object is being deleted.");
}

// Publish functions

void ExplorationPlanner::pubUGVPauseNavigation() {
  ROS_DEBUG("pubUGVPauseNavigation");
  std_msgs::Bool pub_msg;
  pub_msg.data = ugv_pause_navigation_;
  pub_ugv_pause_navigation_.publish(pub_msg);
}

// Callback functions for subscriptions

void ExplorationPlanner::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_WARN("Starting collaborative exploration with 1 UGV and %d UAVs", nr_of_uavs_);

  ugv_pause_navigation_ = true;

  mipp_msgs::StartExplorationGoal exploration_goal;
  exploration_goal.max_time = 2.0;
  act_ugv_exploration_client_->sendGoal(exploration_goal);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    act_uav_exploration_clients_[uav_id]->sendGoal(exploration_goal);
  }

  actionlib::SimpleClientGoalState ugv_state = act_ugv_exploration_client_->getState();
  bool vehicles_ready = ugv_state.isDone();
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    vehicles_ready = vehicles_ready && act_uav_exploration_clients_[uav_id]->getState().isDone();
  }

  // Check if exploration action is done
  ros::Rate check_rate(10);
  while (!vehicles_ready) {
    ROS_INFO_THROTTLE(1, "UGV not finished, state: %s", ugv_state.toString().c_str());
    vehicles_ready = act_ugv_exploration_client_->getState().isDone();
    for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
      vehicles_ready = vehicles_ready && act_uav_exploration_clients_[uav_id]->getState().isDone();
    }
    ros::spinOnce();
    check_rate.sleep();
  }

  // Get results
  mipp_msgs::ExplorationResult ugv_result = act_ugv_exploration_client_->getResult().get()->result;
  ROS_INFO("First node on UGV path: (%f, %f, %f)", ugv_result.paths.begin()->poses.begin()->pose.position.x,
                                                   ugv_result.paths.begin()->poses.begin()->pose.position.y,
                                                   ugv_result.paths.begin()->poses.begin()->pose.position.z);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    mipp_msgs::ExplorationResult uav_result = act_uav_exploration_clients_[uav_id]->getResult().get()->result;
    ROS_INFO("First node on UAV%d path: (%f, %f, %f)", uav_id,
                                                    uav_result.paths.begin()->poses.begin()->pose.position.x,
                                                    uav_result.paths.begin()->poses.begin()->pose.position.y,
                                                    uav_result.paths.begin()->poses.begin()->pose.position.z);
  }
  
  // Send planning goal to UGV, UGV planner will make a plan first
  nav_msgs::Path ugv_end_goal_path = makePathFromExpPath(*ugv_result.paths.begin());
  geometry_msgs::PoseStamped ugv_end_goal = *ugv_end_goal_path.poses.rbegin();
  
  pub_ugv_goal_.publish(ugv_end_goal);
  pub_ugv_goal_path_.publish(ugv_end_goal_path);

  ugv_pause_navigation_ = false;


  ROS_WARN("Done");
}

// Utility functions

void ExplorationPlanner::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  // General
  np.param<std::string>("ugv_ns", ugv_ns_, "/ugv/");
  np.param<int>("nr_of_uavs", nr_of_uavs_, 0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
}

nav_msgs::Path ExplorationPlanner::makePathFromExpPath(mipp_msgs::ExplorationPath exp_path) {
  ROS_DEBUG("makePathFromExpPath");
  nav_msgs::Path path;
  for (auto exp_path_it = exp_path.poses.begin(); exp_path_it != exp_path.poses.end(); ++exp_path_it) {
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = "world";
    path_pose.header.stamp = ros::Time::now();
    path_pose.pose = exp_path_it->pose;
    path.poses.push_back(path_pose);
  }
  ROS_INFO("Made path from: (%f, %f, %f) to (%f, %f, %f)", 
           path.poses.begin()->pose.position.x, path.poses.begin()->pose.position.y, path.poses.begin()->pose.position.z,
           path.poses.rbegin()->pose.position.x, path.poses.rbegin()->pose.position.y, path.poses.rbegin()->pose.position.z);
  return path;
}