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

  // Make a UGVPlanner object as container for variables for the UGV
  ugv_planner_.exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>("/ugv/exploration_action", true);
  ugv_planner_.exploration_client->waitForServer(ros::Duration(10.0));
  // Make a UAVPlanner object as container for variables for each UAV
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    // Declare new UAVPlanner object
    UAVPlanner uav_planner;
    // Exploration
    std::string uav_exploration_client_name = "/uav"+std::to_string(uav_id)+"/exploration_action";
    uav_planner.exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>(uav_exploration_client_name, true);
    uav_planner.exploration_client->waitForServer(ros::Duration(10.0));
    // Navigation
    std::string uav_move_vehicle_client_name = "/uav"+std::to_string(uav_id)+"/move_vehicle_action";
    uav_planner.move_vehicle_client = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(uav_move_vehicle_client_name, true);
    uav_planner.move_vehicle_client->waitForServer(ros::Duration(10.0));
    // Add object to list
    uav_planners_.push_back(uav_planner);
  }

  // Set variables
  running_exploration_ = false;
  ugv_planner_.navigation_paused = false;

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
  pub_msg.data = ugv_planner_.navigation_paused;
  pub_ugv_pause_navigation_.publish(pub_msg);
}

// Callback functions for subscriptions

void ExplorationPlanner::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_DEBUG("subClickedPoint");

  makePlanSynchronous();
}

// Planner functions

void ExplorationPlanner::makePlanSynchronous() {
  ROS_DEBUG("makePlanSynchronous");

  ROS_WARN("Starting collaborative exploration with 1 UGV and %d UAVs", nr_of_uavs_);

  // Pause UGV navigation so it does not move until everyone has plan
  ugv_planner_.navigation_paused = true;

  // Set "start exploration" goal (max time to compute paths) and send to vehicle exploration servers
  ugv_planner_.exploration_goal.max_time = 2.0;
  ugv_planner_.exploration_client->sendGoal(ugv_planner_.exploration_goal);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    uav_planners_[uav_id].exploration_goal.max_time = 2.0;
    uav_planners_[uav_id].exploration_client->sendGoal(uav_planners_[uav_id].exploration_goal);
  }

  // Poll exploration servers and wait until every server is done
  bool vehicles_ready = ugv_planner_.exploration_client->getState().isDone();
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    vehicles_ready = vehicles_ready && uav_planners_[uav_id].exploration_client->getState().isDone();
  }
  ros::Rate check_rate(10);
  while (!vehicles_ready) {
    ROS_INFO_THROTTLE(1, "Vehicles not finished exploring, UGV state: %s", ugv_planner_.exploration_client->getState().toString().c_str());
    vehicles_ready = ugv_planner_.exploration_client->getState().isDone();
    for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
      vehicles_ready = vehicles_ready && uav_planners_[uav_id].exploration_client->getState().isDone();
    }
    ros::spinOnce();
    check_rate.sleep();
  }

  // Get results
  ugv_planner_.exploration_result = ugv_planner_.exploration_client->getResult().get()->result;
  ROS_INFO("First node on UGV path: (%f, %f, %f)", ugv_planner_.exploration_result.paths.begin()->poses.begin()->pose.position.x,
                                                   ugv_planner_.exploration_result.paths.begin()->poses.begin()->pose.position.y,
                                                   ugv_planner_.exploration_result.paths.begin()->poses.begin()->pose.position.z);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    uav_planners_[uav_id].exploration_result = uav_planners_[uav_id].exploration_client->getResult().get()->result;
    ROS_INFO("First node on UAV%d path: (%f, %f, %f)", uav_id,
                                                    uav_planners_[uav_id].exploration_result.paths.begin()->poses.begin()->pose.position.x,
                                                    uav_planners_[uav_id].exploration_result.paths.begin()->poses.begin()->pose.position.y,
                                                    uav_planners_[uav_id].exploration_result.paths.begin()->poses.begin()->pose.position.z);
  }
  
  // UGV planner will make a plan first (but is still paused)
  ugv_planner_.navigation_path = makePathFromExpPath(*(ugv_planner_.exploration_result.paths.begin()));
  ugv_planner_.navigation_goal = *(ugv_planner_.navigation_path.poses.rbegin());
  pub_ugv_goal_.publish(ugv_planner_.navigation_goal);
  pub_ugv_goal_path_.publish(ugv_planner_.navigation_path);

  // Go through UAVs and send navigation goals
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    uav_planners_[uav_id].navigation_path = makePathFromExpPath(*(uav_planners_[uav_id].exploration_result.paths.begin()));
    uav_planners_[uav_id].navigation_goal = *(uav_planners_[uav_id].navigation_path.poses.begin());
    uav_planners_[uav_id].move_vehicle_goal.goal_pose = uav_planners_[uav_id].navigation_goal;
    uav_planners_[uav_id].move_vehicle_goal.goal_reached_radius = 0.1;
    uav_planners_[uav_id].move_vehicle_goal.goal_reached_yaw = 0.1;
    uav_planners_[uav_id].move_vehicle_goal.max_time = 2.0;
    uav_planners_[uav_id].move_vehicle_client->sendGoal(uav_planners_[uav_id].move_vehicle_goal);
  }

  ugv_planner_.navigation_paused = false;

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