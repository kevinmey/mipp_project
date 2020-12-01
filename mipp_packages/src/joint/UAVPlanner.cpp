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
  state_timer = n.createTimer(ros::Duration(0.1), boost::bind(&UAVPlanner::updateStateMachine, this));
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
  // Escort
  // Rng
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  rng_generator = std::default_random_engine(rd());
  rng_unit_distribution = std::uniform_real_distribution<double>(0.0, 1.0);
  // Collision
  initCollisionPoints();
  // Info gain
}

void UAVPlanner::subOdometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  uav_odometry = *odom_msg;
}

void UAVPlanner::updateStateMachine() {
  ROS_INFO_THROTTLE(1.0, "UAV%d STATE: %d", uav_id, vehicle_state);
  ROS_INFO_THROTTLE(1.0, "UAV%d bools (%d, %d).", uav_id, (int)(*global_run_exploration), (int)(*global_run_escorting));
  
  // Check first if UAV needs to be recovered
  if (recoveryRequired()) {
    ROS_WARN_THROTTLE(1.0, "UAV%d needs recovery", uav_id);
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
          if (uav_id == 0) {
            vehicle_state = IDLE;
          }
          else {
            vehicle_state = IDLE;
          }
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
      if (*global_run_exploration) {
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
          ROS_WARN("UAV%d is idle when it should be exploring", uav_id);
          exploration_result.paths.clear();
          navigation_path.poses.clear();
        }
      }
      else if (*global_run_escorting) {
        vehicle_state = ESCORTING;
      }
      else {
        ROS_WARN_THROTTLE(1.0, "UAV%d is idle.", uav_id);
      }
      break;}
    case PLANNING: {
      if (exploration_client->getState().isDone()) {
        navigation_path.poses.clear();
        exploration_result = exploration_client->getResult().get()->result;
        if (exploration_result.paths.empty()) {
          ROS_WARN("UAV%d didn't get any paths from exploration planning, initiating recovery", uav_id);
          prepareForRecovery();
          sendRecoverVehicleGoal();
          vehicle_state = RECOVERING;
        }
        else {
          vehicle_state = IDLE;
        }
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
      if (!*(global_run_escorting)) {
        vehicle_state = IDLE;
        break;
      }
      int current_formation_pose_idx = formation_pose_idx;
      formation_pose_idx = 0;
      for (auto const& pose_it : formation_poses) {
        if (formation_pose_idx > current_formation_pose_idx + 1) {
          // Don't go beyond +1 formation in the bank at a time
          break;
        }
        else if (isPathCollisionFree(getEscortPath(*global_ugv_waypoints, pose_it))) {
          formation_pose = pose_it;
          formation_pose_idx++;
        }
        else {
          break;
        }
      }
      geometry_msgs::PoseStamped uav_escort_pose = getEscortPose(global_ugv_odometry->pose.pose, formation_pose);
      navigation_path = getEscortPath(*global_ugv_waypoints, formation_pose);
      if (!isPathCollisionFree(navigation_path)) {
        ROS_WARN_THROTTLE(1.0, "UAV%d escort path is in collision", uav_id);
        navigation_path.header.frame_id = "collision";
      }
      global_uav_paths->at(uav_id) = navigation_path;
      pub_position_goal.publish(uav_escort_pose);
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
  move_vehicle_goal.goal_path_to_be_improved = false;
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
  exploration_goal.sampling_z = uav_altitude;
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
                             and getDistanceBetweenPoints(pose_it.pose.position, (global_ugv_waypoints->at(nav_waypoint_idx+1))) < com_range
                             and doPointsHaveLOS(pose_it.pose.position, (global_ugv_waypoints->at(nav_waypoint_idx)))
                             and doPointsHaveLOS(pose_it.pose.position, (global_ugv_waypoints->at(nav_waypoint_idx+1)));
      nav_waypoint_idx++;
    }

    // Compute total path gain
    if (com_constraint_satisfied) {
      float path_gain = 0.0;    // Record this paths gain (will be sum of pose gains accounting for sensor overlap)
      std::vector<SensorCircle> path_sensor_coverages;  // Will be filled with previous poses in the path
      for (auto const& pose_it : path_it.poses) {
        float pose_gain = pose_it.gain/pose_it.pose_rank;
        SensorCircle pose_sensor_coverage = makeSensorCircleFromUAVPose(pose_it.pose, uav_id, uav_camera_range);

        /* Waypoint multiplier to encourage sensor circles close to the UGV path
        float pose_sensor_multiplier_threshold = 5.0; // m away from (at least) one waypoint to get multiplier
        float pose_sensor_multiplier = 1.0;
        for (auto const& waypoint_it : *global_ugv_waypoints) {
          if (getDistanceBetweenPoints(pose_sensor_coverage.center, waypoint_it) < pose_sensor_multiplier_threshold) {
            pose_sensor_multiplier = 3.0;
          }
        }*/
        float pose_sensor_coverage_overlap = 0.0; // Will record area of overlap
        for (auto const& sensor_coverage_it : existing_sensor_coverages) {
          pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
        }
        for (auto const& sensor_coverage_it : path_sensor_coverages) {
          pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
        }
        // Need ratio of sensor coverage area to sensor overlap area (capped at max 1, aka 100%)
        double pose_sensor_coverage_overlap_ratio = pose_sensor_coverage_overlap / (M_PI*pow(pose_sensor_coverage.radius, 2));
        path_gain += (1.0 - std::min(pose_sensor_coverage_overlap_ratio, 1.0))*pose_gain; //*pose_sensor_multiplier;
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
  ROS_DEBUG("UAV%d selected path nr. %d after %.2f seconds", uav_id, best_path_nr, time_used);
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

// Escort planning

void UAVPlanner::initFormationPoseBank(int nr_of_uavs) {
  ROS_DEBUG("initFormationPoseBank");
  // I <3 Hard coding stuff...
  int nr_of_poses = 7;
  geometry_msgs::Pose pose_near, pose_far;

  if (nr_of_uavs == 2) {
    switch (uav_id)
    {
    case 0:
      pose_near.position = makePoint(2.0, 0.0, uav_altitude);
      pose_near.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, -(30.0/180.0)*M_PI));
      pose_far.position = makePoint(0.0, -3.0, uav_altitude);
      pose_far.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, 0.0));
      formation_pose = pose_near;
      break;
    case 1:
      pose_near.position = makePoint(0.0, 0.0, uav_altitude);
      pose_near.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, (30.0/180.0)*M_PI));
      pose_far.position = makePoint(0.0, 3.0, uav_altitude);
      pose_far.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, 0.0));
      formation_pose = pose_near;
      break;
    default:
      ROS_ERROR("Couldn't find a hard coded formation for UAV%d", uav_id);
      return;
      break;
    }    
  }
  else {
    switch (uav_id)
    {
    case 0:
      // UAV0 special case only has 1 pose
      formation_pose.position = makePoint(2.0, 0.0, uav_altitude);
      formation_pose.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, 0.0));
      formation_poses.push_back(formation_pose);
      return;
      break;
    case 1:
      pose_near.position = makePoint(0.0, 0.0, uav_altitude);
      pose_near.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, -(60.0/180.0)*M_PI));
      pose_far.position = makePoint(0.0, -6.0, uav_altitude);
      pose_far.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, 0.0));
      formation_pose = pose_near;
      break;
    case 2:
      pose_near.position = makePoint(0.0, 0.0, uav_altitude);
      pose_near.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, (60.0/180.0)*M_PI));
      pose_far.position = makePoint(0.0, 6.0, uav_altitude);
      pose_far.orientation = makeQuatFromRPY(makePoint(0.0, 0.0, 0.0));
      formation_pose = pose_near;
      break;
    default:
      ROS_ERROR("Couldn't find a hard coded formation for UAV%d", uav_id);
      return;
      break;
    }
  }

  float formation_distance = getDistanceBetweenPoints(pose_near.position, pose_far.position);
  float angle_near = makeRPYFromQuat(pose_near.orientation).z;
  float angle_far = makeRPYFromQuat(pose_far.orientation).z;
  float formation_yaw = angle_far - angle_near;
  geometry_msgs::Vector3 formation_direction = getDirection(pose_near.position, pose_far.position);
  
  float formation_distance_incr = formation_distance/(float)nr_of_poses;
  float formation_yaw_incr = formation_yaw/(float)nr_of_poses;

  formation_poses.clear();
  formation_poses.push_back(pose_near);
  for (int i = 1; i < nr_of_poses - 1; i++) {
    geometry_msgs::Pose pose;
    pose.position = castRay(pose_near.position, formation_direction, (double)i*formation_distance_incr);
    pose.orientation = makeQuatFromRPY(makePoint(0, 0, angle_near + (double)i*formation_yaw_incr));
    formation_poses.push_back(pose);
  }
  formation_poses.push_back(pose_far);

  formation_pose = pose_near;
  formation_pose_idx = 0;

  for (auto const& pose : formation_poses) {
    ROS_WARN("Formation pose UAV%d: (%.2f, %.2f, %.2f)", uav_id, pose.position.x, pose.position.y, makeRPYFromQuat(pose.orientation).z);
  }
}

geometry_msgs::Point UAVPlanner::getRandomCirclePoint(geometry_msgs::Point circle_center, float circle_radius) {
  ROS_DEBUG("generateRandomCirclePoint");
  geometry_msgs::Point sample_point;

  // Sample from unit circle, uniformly https://stackoverflow.com/a/50746409
  double radius = circle_radius*sqrt(rng_unit_distribution(rng_generator));
  double theta = rng_unit_distribution(rng_generator)*2.0*M_PI;
  sample_point.x = radius*cos(theta);
  sample_point.y = radius*sin(theta);
  sample_point.z = circle_center.z;
  ROS_DEBUG("Unit circle point (x,y) = (%f,%f)",sample_point.x,sample_point.y);

  sample_point.x += circle_center.x;
  sample_point.y += circle_center.y;

  return sample_point;
}

float UAVPlanner::getRandomYaw(float yaw_deg_center, float yaw_deg_range) {
  ROS_DEBUG("generateRandomCirclePoint");
  float random_yaw_deg = yaw_deg_center - (2.0*rng_unit_distribution(rng_generator) - 1.0)*yaw_deg_range;
  return angles::normalize_angle(random_yaw_deg*(M_PI/180.0));
}

geometry_msgs::PoseStamped UAVPlanner::getEscortPose(const geometry_msgs::Pose& ugv_pose, const geometry_msgs::Pose& uav_formation_pose) {
  ROS_DEBUG("getEscortPose");
  geometry_msgs::PoseStamped uav_escort_pose;
  uav_escort_pose.header.frame_id = "world";
  uav_escort_pose.header.stamp = ros::Time::now();
  
  float ugv_yaw = makeRPYFromQuat(ugv_pose.orientation).z;
  geometry_msgs::Point ugv_position = ugv_pose.position;
  uav_escort_pose.pose.position = getRotatedPoint(ugv_yaw, uav_formation_pose.position, makePoint(0,0,0));
  uav_escort_pose.pose.position.x += ugv_position.x;
  uav_escort_pose.pose.position.y += ugv_position.y;

  uav_escort_pose.pose.orientation = makeQuatFromAddingTwoYaws(ugv_pose.orientation, uav_formation_pose.orientation);

  ROS_DEBUG("UGV (%.2f, %.2f) -- (%.2f, %.2f) -> (%.2f, %.2f) %s", ugv_pose.position.x, ugv_pose.position.y,
                                                                uav_formation_pose.position.x, uav_formation_pose.position.y,
                                                                uav_escort_pose.pose.position.x, uav_escort_pose.pose.position.y, uav_escort_pose.header.frame_id.c_str());
  return uav_escort_pose;
}

nav_msgs::Path UAVPlanner::getEscortPath(const std::vector<geometry_msgs::Point>& ugv_waypoints, const geometry_msgs::Pose& uav_formation_pose) {
  ROS_DEBUG("getEscortPath");
  if (ugv_waypoints.empty()) {
    ROS_ERROR("Got a 0 size UGV waypoint list.");
  }

  // Make path
  nav_msgs::Path escort_path;
  geometry_msgs::Vector3 escort_path_direction;
  escort_path.header.frame_id = "world";
  escort_path.header.stamp = ros::Time::now();
  geometry_msgs::Pose ugv_pose;
  geometry_msgs::Point formation_nudge;
  ugv_pose.orientation = global_ugv_odometry->pose.pose.orientation;
  for (auto ugv_waypoint_it = ugv_waypoints.begin(); ugv_waypoint_it != ugv_waypoints.end(); ++ugv_waypoint_it) {
    ugv_pose.position = *ugv_waypoint_it;

    float waypoint_distance = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDistanceBetweenPoints(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                  : getDistanceBetweenPoints(*ugv_waypoint_it, *std::prev(ugv_waypoint_it));
    geometry_msgs::Vector3 waypoint_direction_vec = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDirection(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                                        : getDirection(*std::prev(ugv_waypoint_it), *ugv_waypoint_it);
    float waypoint_direction_yaw = atan2(waypoint_direction_vec.y, waypoint_direction_vec.x);

    if (ugv_waypoint_it == ugv_waypoints.begin()) {
      // First pose should consider UGV orientation
      ugv_pose.orientation = global_ugv_odometry->pose.pose.orientation;
    }
    else if (waypoint_distance < 0.01) {
      // Waypoints basically on top of each other, ie. UGV will stand still at this point -> Cant deduce orientation, use last orientation
      ugv_pose.orientation = ugv_pose.orientation;
    }
    else {
      // Waypoints far enough that it makes sense to deduce
      geometry_msgs::Vector3 waypoint_direction_vec = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDirection(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                                          : getDirection(*std::prev(ugv_waypoint_it), *ugv_waypoint_it);
      float waypoint_direction_yaw = atan2(waypoint_direction_vec.y, waypoint_direction_vec.x);
      ugv_pose.orientation = makeQuatFromRPY(makePoint(0, 0, waypoint_direction_yaw));
    }
    ROS_DEBUG("RPY: (%.2f, %.2f, %.2f, %.2f)", ugv_pose.orientation.x, ugv_pose.orientation.y, ugv_pose.orientation.z, ugv_pose.orientation.w);
    geometry_msgs::PoseStamped escort_path_pose = getEscortPose(ugv_pose, uav_formation_pose);
    escort_path.poses.push_back(escort_path_pose);
  }
  return escort_path;
}

/*
nav_msgs::Path UAVPlanner::getEscortPath(const std::vector<geometry_msgs::Point>& ugv_waypoints, const geometry_msgs::Pose& uav_formation_pose,
                                         const std::vector<geometry_msgs::Point>& formation_nudges_def,
                                         const bool formation_nudges_zero_pad) {
  ROS_DEBUG("getEscortPath");
  if (ugv_waypoints.empty()) {
    ROS_ERROR("Got a 0 size UGV waypoint list.");
  }

  // Handle default values
  std::vector<geometry_msgs::Point> formation_nudges;
  if (formation_nudges_def.empty()) {
    for (auto const& waypoint_it : ugv_waypoints) {
      formation_nudges.push_back(makePoint(0,0,0));
    }
  }
  else if (formation_nudges_def.size() != ugv_waypoints.size()) {
    for (int waypoint_idx = 0; waypoint_idx < ugv_waypoints.size(); waypoint_idx++) {
      try {
        formation_nudges.push_back(formation_nudges_def.at(waypoint_idx));
      }
      catch (const std::out_of_range& oor) {
        if (formation_nudges_zero_pad) {
          formation_nudges.push_back(makePoint(0,0,0));
        }
        else {
          formation_nudges.push_back(*formation_nudges_def.rend());
        }
      }
    }
  }
  else {
    formation_nudges = formation_nudges_def;
  }
  for (auto const& nudge_it : formation_nudges) {
    ROS_DEBUG("Nudge: (%.2f, %.2f, %.2f)", nudge_it.x, nudge_it.y, nudge_it.z);
  }

  // Make path
  nav_msgs::Path escort_path;
  geometry_msgs::Vector3 escort_path_direction;
  escort_path.header.frame_id = "world";
  escort_path.header.stamp = ros::Time::now();
  geometry_msgs::Pose ugv_pose;
  geometry_msgs::Point formation_nudge;
  ugv_pose.orientation = global_ugv_odometry->pose.pose.orientation;
  for (auto ugv_waypoint_it = ugv_waypoints.begin(); ugv_waypoint_it != ugv_waypoints.end(); ++ugv_waypoint_it) {
    ugv_pose.position = *ugv_waypoint_it;

    float waypoint_distance = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDistanceBetweenPoints(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                  : getDistanceBetweenPoints(*ugv_waypoint_it, *std::prev(ugv_waypoint_it));
    geometry_msgs::Vector3 waypoint_direction_vec = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDirection(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                                        : getDirection(*std::prev(ugv_waypoint_it), *ugv_waypoint_it);
    float waypoint_direction_yaw = atan2(waypoint_direction_vec.y, waypoint_direction_vec.x);

    if (ugv_waypoint_it == ugv_waypoints.begin()) {
      // First pose should consider UGV orientation
      ugv_pose.orientation = global_ugv_odometry->pose.pose.orientation;
    }
    else if (waypoint_distance < 0.01) {
      // Waypoints basically on top of each other, ie. UGV will stand still at this point -> Cant deduce orientation, use last orientation
      ugv_pose.orientation = ugv_pose.orientation;
    }
    else {
      // Waypoints far enough that it makes sense to deduce
      geometry_msgs::Vector3 waypoint_direction_vec = (std::next(ugv_waypoint_it) != ugv_waypoints.end()) ? getDirection(*ugv_waypoint_it, *std::next(ugv_waypoint_it)) 
                                                                                                          : getDirection(*std::prev(ugv_waypoint_it), *ugv_waypoint_it);
      float waypoint_direction_yaw = atan2(waypoint_direction_vec.y, waypoint_direction_vec.x);
      ugv_pose.orientation = makeQuatFromRPY(makePoint(0, 0, waypoint_direction_yaw));
    }
    ROS_DEBUG("RPY: (%.2f, %.2f, %.2f, %.2f)", ugv_pose.orientation.x, ugv_pose.orientation.y, ugv_pose.orientation.z, ugv_pose.orientation.w);
    geometry_msgs::PoseStamped escort_path_pose = getEscortPose(ugv_pose, uav_formation_pose);
    escort_path.poses.push_back(escort_path_pose);
  }
  return escort_path;
}
*/

// LOS

bool UAVPlanner::doPointsHaveLOS(const geometry_msgs::Point point_a, const geometry_msgs::Point point_b) {
  ROS_DEBUG("doPointsHaveLOS");

  double occupancy_threshold = octomap->getOccupancyThres();
  float point_distance = getDistanceBetweenPoints(point_a, point_b);
  octomap::point3d om_end_point;

  octomap::point3d om_point_a(point_a.x, point_a.y, point_a.z);
  geometry_msgs::Vector3 direction_ab = getDirection(point_a, point_b);
  octomap::point3d om_direction_ab(direction_ab.x, direction_ab.y, direction_ab.z);
  bool hit_occupied_ab = octomap->castRay(om_point_a, om_direction_ab, om_end_point, true, point_distance);

  octomap::point3d om_point_b(point_b.x, point_b.y, point_b.z);
  geometry_msgs::Vector3 direction_ba = getDirection(point_b, point_a);
  octomap::point3d om_direction_ba(direction_ba.x, direction_ba.y, direction_ba.z);
  bool hit_occupied_ba = octomap->castRay(om_point_b, om_direction_ba, om_end_point, true, point_distance);

  return (!hit_occupied_ab and !hit_occupied_ba);
}

// Collision

void UAVPlanner::initCollisionPoints() {
  ROS_DEBUG("initCollisionPoints");
  float collision_radius = 0.5;
  geometry_msgs::Point collision_point = makePoint(0, 0, 0);
  //collision_points.push_back(octomap::point3d(collision_point.x, collision_point.y, collision_point.z));
  for (float point_yaw = 0.0; point_yaw < 360.0; point_yaw += 45.0) {
    collision_point = getRotatedPoint(point_yaw*(M_PI/180.0), makePoint(collision_radius, 0 ,0), makePoint(0, 0, 0));
    collision_points.push_back(octomap::point3d(collision_point.x, collision_point.y, collision_point.z));
    ROS_WARN("Collision point (%.2f,%.2f,%.2f)", collision_point.x, collision_point.y, collision_point.z);
  }
}

bool UAVPlanner::isPathCollisionFree(const nav_msgs::Path& path) {
  ROS_DEBUG("isPathCollisionFree");

  bool unmapped_is_collision = true;
  for (auto const& pose_it : path.poses) {
    if (!isPoseCollisionFree(pose_it.pose.position, unmapped_is_collision)) {
      return false;
    }
    unmapped_is_collision = false;
  }

  float check_interval = 0.75;
  for (auto pose_it = path.poses.begin(); pose_it != path.poses.end(); ++pose_it) {
    if (std::next(pose_it) == path.poses.end()) break;
    float check_interval_i = 1;
    float path_distance = getDistanceBetweenPoints(pose_it->pose.position, std::next(pose_it)->pose.position);
    geometry_msgs::Vector3 path_direction = getDirection(pose_it->pose.position, std::next(pose_it)->pose.position);
    while (check_interval*check_interval_i < path_distance) {
      geometry_msgs::Point check_point = castRay(pose_it->pose.position, path_direction, check_interval*check_interval_i);
      if (!isPoseCollisionFree(check_point)) return false;
      check_interval_i++;
    }
  }

  return true;
}

bool UAVPlanner::isPoseCollisionFree(const geometry_msgs::Point& pose, bool unmapped_is_collision) {
  ROS_DEBUG("isPoseCollisionFree");
  double occupancy_threshold = octomap->getOccupancyThres();

  // Check center point occupancy
  octomap::OcTreeNode* om_center_node = octomap->search(pose.x, pose.y, pose.z);
  if (om_center_node != NULL) {
    double node_occupancy = om_center_node->getOccupancy();
    ROS_DEBUG("Node value: %f", node_occupancy);
    if (node_occupancy > occupancy_threshold) {
      // End node is occupied
      return false;
    }
  }
  else if (unmapped_is_collision) {
    // End node is unmapped
    return false;
  }

  //
  octomap::point3d om_center_point(pose.x, pose.y, pose.z);
  octomap::point3d om_end_point;
  for (auto const& collision_point_it : collision_points) {
    bool hit_occupied = octomap->castRay(om_center_point, collision_point_it, om_end_point, true, 0.75);
    if (hit_occupied) {return false;}
    octomap::OcTreeNode* om_end_node = octomap->search(om_end_point.x(), om_end_point.y(), om_end_point.z());
    if (om_end_node != NULL) {
      double node_occupancy = om_end_node->getOccupancy();
      ROS_DEBUG("Node value: %f", node_occupancy);
      if (node_occupancy > occupancy_threshold) {
        // End node is occupied
        return false;
      }
    }
  }

  return true;
}

// Info gain

float UAVPlanner::getPoseInfoGain(geometry_msgs::Point origin, float yaw) {
  ROS_DEBUG("calculateInformationGain");
  // Return a number between 0.0 and 1.0, related to how many of the cast rays from the pose hit unmapped voxels (1.0 meaning 100% unmapped)

  float ray_distance = camera_range;
  tf2::Matrix3x3 ray_direction_rotmat;
  ray_direction_rotmat.setEulerYPR(yaw, 0.0, 0.0);

  octomap::point3d om_ray_origin = octomap::point3d(origin.x, origin.y, origin.z);
  int occupied = 0;
  int free = 0;
  int unmapped = 0;
  float occupancy_threshold = octomap->getOccupancyThres();

  for(tf2::Vector3 ray : info_camera_rays) {
    tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_distance;
    octomap::point3d om_ray_direction = octomap::point3d(ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
    octomap::point3d om_ray_end_cell;
    bool hit_occupied = octomap->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, ray_distance);
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = om_ray_end_cell.x();
    ray_end_point.y = om_ray_end_cell.y();
    ray_end_point.z = om_ray_end_cell.z();
    
    octomap::OcTreeNode* om_ray_end_node = octomap->search(om_ray_end_cell);
    if (om_ray_end_node != NULL) {
      float node_occupancy = om_ray_end_node->getOccupancy();
      ROS_DEBUG("Node value: %f", node_occupancy);
      if (node_occupancy < occupancy_threshold) {
        free++;
      }
      else {
        occupied++;
      }
    }
    else {
      unmapped++;
    }
  }
  return unmapped/(float)(info_camera_rays.size());
}

float UAVPlanner::getPathInfoGain(const mipp_msgs::ExplorationPath& path, const std::vector<SensorCircle>& sensor_coverages) {
  float path_gain = 0.0;    // Record this paths gain (will be sum of pose gains accounting for sensor overlap)
  std::vector<SensorCircle> path_sensor_coverages;  // Will be filled with previous poses in the path
  for (auto const& pose_it : path.poses) {
    float pose_gain = pose_it.gain/pose_it.pose_rank;
    SensorCircle pose_sensor_coverage = makeSensorCircleFromUAVPose(pose_it.pose, uav_id, camera_range);

    /* Waypoint multiplier to encourage sensor circles close to the UGV path
    float pose_sensor_multiplier_threshold = 5.0; // m away from (at least) one waypoint to get multiplier
    float pose_sensor_multiplier = 1.0;
    for (auto const& waypoint_it : *global_ugv_waypoints) {
      if (getDistanceBetweenPoints(pose_sensor_coverage.center, waypoint_it) < pose_sensor_multiplier_threshold) {
        pose_sensor_multiplier = 3.0;
      }
    }*/
    float pose_sensor_coverage_overlap = 0.0; // Will record area of overlap
    for (auto const& sensor_coverage_it : sensor_coverages) {
      pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
    }
    for (auto const& sensor_coverage_it : path_sensor_coverages) {
      pose_sensor_coverage_overlap += calculateSensorCoverageOverlap(pose_sensor_coverage, sensor_coverage_it);
    }
    // Need ratio of sensor coverage area to sensor overlap area (capped at max 1, aka 100%)
    double pose_sensor_coverage_overlap_ratio = pose_sensor_coverage_overlap / (M_PI*pow(pose_sensor_coverage.radius, 2));
    path_gain += (1.0 - std::min(pose_sensor_coverage_overlap_ratio, 1.0))*pose_gain; //*pose_sensor_multiplier;
    path_sensor_coverages.push_back(pose_sensor_coverage);
  }
  return path_gain;
}