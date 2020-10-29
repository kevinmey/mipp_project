#include <ExplorationPlanner.hpp>

// SUBMODULE FOR EXPLORATION PLANNER

void UAVPlanner::updateStateMachine() {
    // Check UGV first
  switch (vehicle_state) {
    case IDLE:
      if (exploration_result.paths.empty()) {
        float exploration_max_time = 2.0;
        sendExplorationGoal(exploration_max_time);
        vehicle_state = PLANNING;
      }
      else if (navigation_path.poses.empty()) {
        
      }
      else {
        vehicle_state = MOVING;
      }
      break;
    case PLANNING:
      if (exploration_client->getState().isDone()) {
        exploration_result = exploration_client->getResult().get()->result;
        vehicle_state = IDLE;
      }
      break;
  }
}

void UAVPlanner::sendExplorationGoal(float exploration_time) {
  exploration_goal.max_time = exploration_time;
  exploration_goal.init_path.clear();
  for (auto const& pose_it : navigation_path.poses) {
    exploration_goal.init_path.push_back(pose_it);
    ROS_WARN("Pushed in nav. path pose: (%.2f, %.2f)", pose_it.pose.position.x, pose_it.pose.position.y);
  }
  exploration_client->sendGoal(exploration_goal);

  navigation_path.poses.clear();
}

bool UAVPlanner::isExplorationDone() {
  return exploration_client->getState().isDone();
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

  for (auto const& path_it : exploration_result.paths) {
    // Record this paths gain (will be sum of pose gains accounting for sensor overlap)
    float path_gain = 0.0;
    path_nr++;
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
  // Best path selected, add its coverages to sensor_coverages_ and make it this UAVs navigation path
  if (best_path_nr != -1) {
    for (auto const& pose_it : best_path.poses) {
      sensor_coverage.push_back(makeSensorCircleFromUAVPose(pose_it.pose, uav_id, uav_camera_range));
    }
  }
  navigation_path = best_path;
  ROS_INFO("UAV%d selected path nr. %d", uav_id, best_path_nr);
}

void UAVPlanner::sendMoveVehicleGoal(float move_vehicle_time) {
  navigation_goal = *(navigation_path.poses.begin());
  move_vehicle_goal.goal_pose = navigation_goal;
  move_vehicle_goal.goal_reached_radius = 0.5;
  move_vehicle_goal.goal_reached_yaw = 0.1;
  move_vehicle_goal.max_time = move_vehicle_time;
  move_vehicle_client->sendGoal(move_vehicle_goal);

  exploration_result.paths.clear();
}