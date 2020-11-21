#include <MippPlanner.hpp>

// SUBMODULE FOR EXPLORATION PLANNER

void UAVPlanner::init(ros::NodeHandle n) {
  // Subscribers
  sub_odometry = n.subscribe("/gazebo/ground_truth_uav"+std::to_string(uav_id), 1, &UAVPlanner::subOdometry, this);
  // Global Info (Stored in MippPlanner object)
  (*global_sensor_coverages)[uav_id] = sensor_coverage;
  (*global_uav_paths)[uav_id] = navigation_path;
  // Recovery behaviour
  no_viable_plan = false;
  out_of_com_range = false;
  // Vehicle planner state
  vehicle_state = INIT;
  state_timer = n.createTimer(ros::Duration(0.5), boost::bind(&UAVPlanner::updateStateMachine, this));
  std::string takeoff_client_name = "/uav"+std::to_string(uav_id)+"/takeoff_complete_service";
  takeoff_client = n.serviceClient<mipp_msgs::TakeoffComplete>(takeoff_client_name);
  // Exploration
  std::string uav_exploration_client_name = "/uav"+std::to_string(uav_id)+"/exploration_action";
  exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>(uav_exploration_client_name, true);
  exploration_client->waitForServer(ros::Duration(10.0));
  // Navigation
  std::string uav_move_vehicle_client_name = "/uav"+std::to_string(uav_id)+"/move_vehicle_action";
  move_vehicle_client = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(uav_move_vehicle_client_name, true);
  move_vehicle_client->waitForServer(ros::Duration(10.0));
}

void UAVPlanner::subOdometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  uav_odometry = *odom_msg;
}

void UAVPlanner::updateStateMachine() {
  ROS_INFO("UAV%d STATE: %d", uav_id, vehicle_state);
  
  // Check first if UAV needs to be recovered
  if (recoveryRequired()) {
    ROS_WARN("UAV%d needs recovery", uav_id);
    prepareForRecovery();
    sendRecoverVehicleGoal();
    vehicle_state = RECOVERING;
  }

  // Update state machine
  switch (vehicle_state) {
    case INIT: {
      mipp_msgs::TakeoffComplete srv;
      if (takeoff_client.call(srv)) {
        if (srv.response.takeoff_complete) {
          ROS_INFO("UAV%d takeoff complete.", uav_id);
          vehicle_state = ESCORTING;
        }
        else {
          ROS_DEBUG("UAV%d still waiting for takeoff to complete.", uav_id);
        }
      }
      else {
        ROS_ERROR("UAV%d couldn't call takeoff_complete server.", uav_id);
      }
      break;}
    case IDLE: {
      if (exploration_result.paths.empty()) {
        float exploration_max_time = 1.0;
        sendExplorationGoal(exploration_max_time);
        vehicle_state = PLANNING;
      }
      else if (navigation_path.poses.empty()) {
        createNavigationPlan(getExistingSensorCoverages(), camera_range);
        global_sensor_coverages->at(uav_id) = sensor_coverage;
        global_uav_paths->at(uav_id) = navigation_path;
        sendMoveVehicleGoal(10.0);
        vehicle_state = MOVING;
      }
      else {
        ROS_WARN("UAV%d is idle", uav_id);
      }
      break;}
    case PLANNING: {
      if (exploration_client->getState().isDone()) {
        navigation_path.poses.clear();
        exploration_result = exploration_client->getResult().get()->result;
        vehicle_state = IDLE;
      }
      break;}
    case MOVING: {
      ROS_DEBUG("UAV%d: Length of sensor_coverages: (%d, %d)", uav_id, (int)global_sensor_coverages->size(), (int)global_sensor_coverages->at(uav_id).size());
      ROS_DEBUG("UAV%d: Length of uav_paths: (%d, %d)", uav_id, (int)global_uav_paths->size(), (int)global_uav_paths->at(uav_id).poses.size());
      if (move_vehicle_client->getState().isDone()) {
        exploration_result.paths.clear();
        global_sensor_coverages->at(uav_id).clear();
        global_uav_paths->at(uav_id).poses.clear();
        vehicle_state = IDLE;
      }
      break;}
    case ESCORTING: {
      geometry_msgs::PoseStamped uav_escort_pose;
      uav_escort_pose.header.frame_id = "world";
      uav_escort_pose.header.stamp = ros::Time::now();

      float ugv_yaw = makeRPYFromQuat(global_ugv_odometry->pose.pose.orientation).z;
      geometry_msgs::Point ugv_position = global_ugv_odometry->pose.pose.position;
      uav_escort_pose.pose.position = getRotatedPoint(ugv_yaw, formation_pose.position, makePoint(0,0,0));
      uav_escort_pose.pose.position.x += ugv_position.x;
      uav_escort_pose.pose.position.y += ugv_position.y;
      uav_escort_pose.pose.orientation = global_ugv_odometry->pose.pose.orientation;
      pub_position_goal.publish(uav_escort_pose);
      ROS_WARN("UGV (%.2f, %.2f) -- (%.2f, %.2f) -> (%.2f, %.2f) %s", global_ugv_odometry->pose.pose.position.x, global_ugv_odometry->pose.pose.position.y,
                                                                   formation_pose.position.x, formation_pose.position.y,
                                                                   uav_escort_pose.pose.position.x, uav_escort_pose.pose.position.y, uav_escort_pose.header.frame_id.c_str());
      break;}
    case RECOVERING: {
      ROS_WARN_THROTTLE(1.0, "Recovering UAV%d", uav_id);
      if (move_vehicle_client->getState().isDone()) {
        exploration_result.paths.clear();
        navigation_path.poses.clear();
        global_sensor_coverages->at(uav_id).clear();
        global_uav_paths->at(uav_id).poses.clear();
        vehicle_state = IDLE;
      }
      break;}
    case DONE: {
      break;}
    default: {
      ROS_ERROR("UAV%d default", uav_id);
      break;}
  }
}

bool UAVPlanner::recoveryRequired() {
  if ((no_viable_plan or out_of_com_range) and (vehicle_state != RECOVERING) and (vehicle_state != INIT)) {
    ROS_WARN("Recovery: Plan(%d), Range(%d), State(%d)", (int)no_viable_plan, (int)out_of_com_range, (int)vehicle_state);
    return true;
  }
  return false;
}

void UAVPlanner::prepareForRecovery() {
  // Recovery procedure will depend on current state
  exploration_client->cancelAllGoals();
  exploration_result.paths.clear();
  move_vehicle_client->cancelAllGoals();
  navigation_path.poses.clear();

  no_viable_plan = false;
}

void UAVPlanner::sendRecoverVehicleGoal() {
  move_vehicle_goal.goal_pose = recovery_goal;
  move_vehicle_goal.goal_path.poses.clear();
  move_vehicle_goal.goal_path_to_be_improved = true;
  move_vehicle_goal.goal_reached_radius = 2.0;
  move_vehicle_goal.goal_reached_yaw = 3.14;
  move_vehicle_goal.goal_reached_max_time = 1000.0;
  move_vehicle_client->sendGoal(move_vehicle_goal);
}

void UAVPlanner::sendExplorationGoal(float exploration_time) {
  exploration_goal.max_time = exploration_time;
  exploration_goal.init_path.clear();
  if (!navigation_path.poses.empty()) {
    navigation_path.poses.erase(navigation_path.poses.begin());
    for (auto const& pose_it : navigation_path.poses) {
      exploration_goal.init_path.push_back(pose_it);
      ROS_DEBUG("Pushed in nav. path pose: (%.2f, %.2f)", pose_it.pose.position.x, pose_it.pose.position.y);
    }
  }
  exploration_goal.sampling_centers = *global_ugv_waypoints;
  exploration_goal.sampling_radius = 7.0;
  exploration_goal.sampling_z = (double)(uav_id*0.75 + 1.5);
  exploration_goal.sampling_z_interval = 0.1;
  exploration_client->sendGoal(exploration_goal);

  navigation_path.poses.clear();
}

bool UAVPlanner::isExplorationDone() {
  return exploration_client->getState().isDone();
}

std::vector<SensorCircle> UAVPlanner::getExistingSensorCoverages() {
  std::vector<SensorCircle> existing_sensor_coverages;
  for (auto const& uav_it : *global_sensor_coverages) {
    for (auto const& coverage_it : uav_it.second) {
      existing_sensor_coverages.push_back(coverage_it);
    }
  }
  return existing_sensor_coverages;
}

mipp_msgs::ExplorationResult UAVPlanner::getExplorationResult() {
  mipp_msgs::ExplorationResult result = exploration_client->getResult().get()->result;
  ROS_DEBUG("First node on UGV path: (%f, %f, %f)", exploration_result.paths.begin()->poses.begin()->pose.position.x,
                                                    exploration_result.paths.begin()->poses.begin()->pose.position.y,
                                                    exploration_result.paths.begin()->poses.begin()->pose.position.z);
  return result;
}

void UAVPlanner::createNavigationPlan(std::vector<SensorCircle> existing_sensor_coverages, float uav_camera_range) {
  // Consider first the current pose sensor coverage
  geometry_msgs::Pose current_uav_pose = (exploration_result.paths.end()-1)->poses.begin()->pose;
  sensor_coverage.clear();
  sensor_coverage.push_back(makeSensorCircleFromUAVPose(current_uav_pose, uav_id, uav_camera_range));

  // Go through paths from the UAV explorer and pick the "best" one (most gain)
  nav_msgs::Path best_path;
  float best_path_gain = 0.0;
  int best_path_nr = -1; // Record which path was chosen in the end, for fun mostly
  int path_nr = best_path_nr;

  // Have current position (dont move) as initial "best path" in case no path is good enough
  best_path = makePathFromExpPath(*(exploration_result.paths.end()-1));

  ros::Time begin_time = ros::Time::now();
  for (auto const& path_it : exploration_result.paths) {
    path_nr++;
    
    // Check if com constraints are satisfied
    bool com_constraint_satisfied = true;
    int nav_waypoint_idx = 0;
    for (auto const& pose_it : path_it.poses) {
      com_constraint_satisfied = com_constraint_satisfied 
                             and getDistanceBetweenPoints(pose_it.pose.position, (global_ugv_waypoints->at(nav_waypoint_idx))) < com_range
                             and getDistanceBetweenPoints(pose_it.pose.position, (global_ugv_waypoints->at(nav_waypoint_idx+1))) < com_range;
      nav_waypoint_idx++;
    }

    // Compute total path gain
    if (com_constraint_satisfied) {
      float path_gain = 0.0;    // Record this paths gain (will be sum of pose gains accounting for sensor overlap)
      std::vector<SensorCircle> path_sensor_coverages;  // Will be filled with previous poses in the path
      for (auto const& pose_it : path_it.poses) {
        float pose_gain = pose_it.gain;
        SensorCircle pose_sensor_coverage = makeSensorCircleFromUAVPose(pose_it.pose, uav_id, uav_camera_range);
        float pose_sensor_coverage_overlap = 0.0; // Will record area of overlap
        for (auto const& sensor_coverage_it : existing_sensor_coverages) {
          pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
        }
        for (auto const& sensor_coverage_it : path_sensor_coverages) {
          pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
        }
        // Need ratio of sensor coverage area to sensor overlap area (capped at max 1, aka 100%)
        double pose_sensor_coverage_overlap_ratio = pose_sensor_coverage_overlap / (M_PI*pow(pose_sensor_coverage.radius, 2));
        path_gain += (1.0 - std::min(pose_sensor_coverage_overlap_ratio, 1.0))*pose_gain;
        path_sensor_coverages.push_back(pose_sensor_coverage);
      }
      if (path_gain > best_path_gain) {
        ROS_DEBUG("New path selected, gain increased from %.2f to %.2f", best_path_gain, path_gain);
        std::vector<nav_msgs::Path> viz_path;
        viz_path.push_back(makePathFromExpPath(path_it));
        viz_path.push_back(makePathFromExpPath(*exploration_result.paths.rbegin()));
        best_path_gain = path_gain;
        best_path = makePathFromExpPath(path_it);
        best_path_nr = path_nr;
      }
    }
  }
  // Best path selected, add its coverages to sensor_coverages_ and make it this UAVs navigation path
  if (best_path_nr != -1) {
    no_viable_plan = false;
    for (auto const& pose_it : best_path.poses) {
      sensor_coverage.push_back(makeSensorCircleFromUAVPose(pose_it.pose, uav_id, uav_camera_range));
    }
  }
  else {
    no_viable_plan = true;
  }
  navigation_path = best_path;
  float time_used = (ros::Time::now() - begin_time).toSec();
  ROS_INFO("UAV%d selected path nr. %d after %.2f seconds", uav_id, best_path_nr, time_used);
}

void UAVPlanner::sendMoveVehicleGoal(float move_vehicle_time) {
  navigation_goal = *(navigation_path.poses.begin());
  move_vehicle_goal.goal_pose = navigation_goal;
  move_vehicle_goal.goal_path.poses.clear();
  move_vehicle_goal.goal_path_to_be_improved = false;
  move_vehicle_goal.goal_reached_radius = 1.0;
  move_vehicle_goal.goal_reached_yaw = 0.1;
  move_vehicle_goal.goal_reached_max_time = move_vehicle_time;
  move_vehicle_client->sendGoal(move_vehicle_goal);

  exploration_result.paths.clear();
}