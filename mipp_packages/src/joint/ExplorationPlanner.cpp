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
  pub_viz_sensor_circle_ = n.advertise<visualization_msgs::Marker>("ExplorationPlanner/viz_sensor_circle_", 1);
  pub_viz_sensor_coverages_ = n.advertise<visualization_msgs::MarkerArray>("ExplorationPlanner/viz_sensor_coverages_", 1);

  sub_clicked_point_ = n.subscribe("/exploration/start_collaborative", 1, &ExplorationPlanner::subClickedPoint, this);
  sub_ugv_goal_plan_ = n.subscribe(ugv_ns_+"move_base/TebLocalPlannerROS/global_plan", 1, &ExplorationPlanner::subUGVPlan, this);

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
  ugv_planner_.naviation_beacon_max_dist = ugv_nav_waypoint_max_distance_;
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

void ExplorationPlanner::subUGVPlan(const nav_msgs::PathConstPtr& path_msg) {
  ROS_DEBUG("subUGVPlan");

  ugv_planner_.navigation_path = *path_msg;
}

// Planner functions

void ExplorationPlanner::makePlanSynchronous() {
  ROS_DEBUG("makePlanSynchronous");
  ros::Rate check_rate(10);

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
  ugv_planner_.navigation_path_init = makePathFromExpPath(*(ugv_planner_.exploration_result.paths.begin()));
  ugv_planner_.navigation_goal = *(ugv_planner_.navigation_path_init.poses.rbegin());
  ugv_planner_.navigation_path.poses.clear(); // Clear path since we will wait for a new path from the UGV planner
  pub_ugv_goal_.publish(ugv_planner_.navigation_goal);
  pub_ugv_goal_path_.publish(ugv_planner_.navigation_path_init);

  // Wait until we get the "optimized" path back 
  while (ugv_planner_.navigation_path.poses.empty()) {
    ros::spinOnce();
    check_rate.sleep();
  }

  // Work on path made by UGV planner and create communication "beacons"
  float dist_to_prev_beacon = 0.0;
  ugv_planner_.navigation_waypoints.clear();
  for (auto path_it = ugv_planner_.navigation_path.poses.begin(); path_it != ugv_planner_.navigation_path.poses.end(); ++path_it) {
    if (ugv_planner_.navigation_waypoints.empty()) {
      // Add first pose in plan (current pose) as com beacon
      ugv_planner_.navigation_waypoints.push_back(path_it->pose.position);
      ROS_WARN("Added pose nr. %d as nav. waypoint at (%.2f, %.2f)", (int)(path_it - ugv_planner_.navigation_path.poses.begin()),
                                                                   path_it->pose.position.x, path_it->pose.position.y);
    }
    else {
      dist_to_prev_beacon += getDistanceBetweenPoints(path_it->pose.position, std::prev(path_it)->pose.position);
      if (dist_to_prev_beacon > ugv_planner_.naviation_beacon_max_dist and ugv_planner_.navigation_waypoints.size() < nr_of_ugv_nav_waypoints_) {
        // Distance to previous beacon exceeds max distance, add PREVIOUS pose as beacon (since it was within max distance)
        ugv_planner_.navigation_waypoints.push_back(std::prev(path_it)->pose.position);
        dist_to_prev_beacon = getDistanceBetweenPoints(path_it->pose.position, std::prev(path_it)->pose.position);
        ROS_WARN("Added pose nr. %d as nav. waypoint at (%.2f, %.2f), with distance %.2f to previous waypoint", 
                 (int)(path_it - ugv_planner_.navigation_path.poses.begin() - 1),
                 std::prev(path_it)->pose.position.x, std::prev(path_it)->pose.position.y,
                 getDistanceBetweenPoints(ugv_planner_.navigation_waypoints.end()[-1], 
                                          ugv_planner_.navigation_waypoints.end()[-2]));
      }
    }
  }
  // Pad the com beacons with the goal position if path not long enough to fit all
  if (ugv_planner_.navigation_waypoints.size() < nr_of_ugv_nav_waypoints_) {
    ugv_planner_.navigation_waypoints.push_back(ugv_planner_.navigation_path.poses.rbegin()->pose.position);
    ROS_WARN("Padded with goal pose as nav. waypoint at (%.2f, %.2f)", ugv_planner_.navigation_path.poses.rbegin()->pose.position.x, 
                                                                     ugv_planner_.navigation_path.poses.rbegin()->pose.position.y);
  }

  // Create sensor coverage for UGV
  sensor_coverages_.clear();
  int vehicle_id = -1;
  for (auto point_it = ugv_planner_.navigation_waypoints.begin(); point_it != ugv_planner_.navigation_waypoints.end(); ++point_it) {
    SensorCircle sensor_coverage;
    sensor_coverage.vehicle_id = vehicle_id;
    sensor_coverage.radius = ugv_sensor_radius_;
    sensor_coverage.center = *point_it;
    sensor_coverages_.push_back(sensor_coverage);
    visualizeSensorCoverages();
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }

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
  np.param<std::string>("planner_world_frame", planner_world_frame_, "world");
  // UGV
  np.param<std::string>("ugv_ns", ugv_ns_, "/ugv/");
  np.param<int>("nr_of_ugv_nav_waypoints", nr_of_ugv_nav_waypoints_, 4);
  np.param<float>("ugv_nav_beacon_max_distance", ugv_nav_waypoint_max_distance_, 3.0);
  np.param<float>("ugv_sensor_radius", ugv_sensor_radius_, 7.5);
  // UAVS
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

// Visualization

void ExplorationPlanner::visualizeSensorCircle(SensorCircle sensor_circle) {
  ROS_DEBUG("visualizeSensorCircle");
  visualization_msgs::Marker marker;

  marker.header.frame_id = planner_world_frame_;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = sensor_circle.center;
  marker.scale.x = sensor_circle.radius;
  marker.scale.y = sensor_circle.radius;
  marker.scale.z = 0.1;
  marker.color.a = 0.3;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
  marker.id = 0;
  
  pub_viz_sensor_circle_.publish(marker);
}

void ExplorationPlanner::visualizeSensorCoverages() {
  ROS_DEBUG("visualizeSensorCoverages");
  visualization_msgs::MarkerArray marker_array;

  for (auto circle_it = sensor_coverages_.begin(); circle_it != sensor_coverages_.end(); ++circle_it) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = planner_world_frame_;
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = circle_it->center;
    marker.scale.x = 2*circle_it->radius;
    marker.scale.y = 2*circle_it->radius;
    marker.scale.z = 0.1;
    marker.color.a = 0.1;
    if (circle_it->vehicle_id == -1) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }
    marker.lifetime = ros::Duration();
    marker.id = marker_array.markers.size();

    marker_array.markers.push_back(marker);
  }
  pub_viz_sensor_coverages_.publish(marker_array);
}

