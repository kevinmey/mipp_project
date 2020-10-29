#include <ExplorationPlanner.hpp>

// SUBMODULE FOR EXPLORATION PLANNER

/*
void UGVPlanner::updateStateMachine() {
    // Check UGV first
  switch (vehicle_state) {
    case IDLE:
      // UGV is IDLE
      if (exploration_result.paths.empty()) {
        // UGV needs a plan, start planning
        float exploration_max_time = 2.0;
        sendExplorationGoal(exploration_max_time);
        vehicle_state = PLANNING;
      }
      else {
        // UGV has a plan, start making navigaiton plan and moving
        createInitNavigationPlan();
        pub_ugv_goal_.publish(ugv_planner_.navigation_goal);
        pub_ugv_goal_path_.publish(ugv_planner_.navigation_path_init);
        // Wait until we get the "optimized" path back 
        while (ugv_planner_.navigation_path.poses.empty()) {
          ros::spinOnce();
          check_rate.sleep();
        }

        // Work on path made by UGV planner and create communication "beacons"
        ugv_planner_.createNavigationWaypoints(nr_of_ugv_nav_waypoints_, add_nav_waypoint_at_goal_);

        // Create sensor coverage for vehicles
        sensor_coverages_.clear();

        // Create sensor coverage for UGV
        sensor_coverages_ = ugv_planner_.getSensorCoverage(ugv_sensor_radius_);
        vehicle_state = MOVING;
      }
      else {
        ROS_ERROR("Illegal state transition for UGV.");
      }
      break;
    case PLANNING:
      // UGV is PLANNING
      if (exploration_client->getState().isDone()) {
        // UGV has gotten a plan
        exploration_result = exploration_client->getResult().get()->result;
        vehicle_previous_state = vehicle_state;
        vehicle_state = IDLE;
      }
      break;
  }
}*/

void UGVPlanner::sendExplorationGoal(float exploration_time) {
  exploration_goal.max_time = exploration_time;
  exploration_client->sendGoal(exploration_goal);
}

bool UGVPlanner::isExplorationDone() {
  return exploration_client->getState().isDone();
}

mipp_msgs::ExplorationResult UGVPlanner::getExplorationResult() {
  mipp_msgs::ExplorationResult result = exploration_client->getResult().get()->result;
  ROS_DEBUG("First node on UGV path: (%f, %f, %f)", exploration_result.paths.begin()->poses.begin()->pose.position.x,
                                                    exploration_result.paths.begin()->poses.begin()->pose.position.y,
                                                    exploration_result.paths.begin()->poses.begin()->pose.position.z);
  return result;
}

void UGVPlanner::createInitNavigationPlan() {
  navigation_path_init = makePathFromExpPath(*(exploration_result.paths.begin()));
  navigation_goal = *(navigation_path_init.poses.rbegin());
  navigation_path.poses.clear(); // Clear path since we will wait for a new path from the UGV planner
  pub_goal_.publish(navigation_goal);
  pub_goal_path_.publish(navigation_path_init);
  // Wait until we get the "optimized" path back 
  while (navigation_path.poses.empty()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("UGV has plan.");
}

void UGVPlanner::createNavigationWaypoints(int nr_of_ugv_nav_waypoints, bool add_nav_waypoint_at_goal) {
  // Work on path made by UGV planner and create communication "beacons"
  float dist_to_prev_beacon = 0.0;
  navigation_waypoints.clear();
  for (auto path_it = navigation_path.poses.begin(); path_it != navigation_path.poses.end(); ++path_it) {
    if (navigation_waypoints.empty()) {
      // Add first pose in plan (current pose) as com beacon
      navigation_waypoints.push_back(path_it->pose.position);
      ROS_DEBUG("Added pose nr. %d as nav. waypoint at (%.2f, %.2f)", (int)(path_it - navigation_path.poses.begin()),
                                                                   path_it->pose.position.x, path_it->pose.position.y);
    }
    else {
      dist_to_prev_beacon += getDistanceBetweenPoints(path_it->pose.position, std::prev(path_it)->pose.position);
      if (dist_to_prev_beacon > navigation_waypoint_max_dist and navigation_waypoints.size() < nr_of_ugv_nav_waypoints) {
        // Distance to previous beacon exceeds max distance, add PREVIOUS pose as beacon (since it was within max distance)
        navigation_waypoints.push_back(std::prev(path_it)->pose.position);
        dist_to_prev_beacon = getDistanceBetweenPoints(path_it->pose.position, std::prev(path_it)->pose.position);
        ROS_DEBUG("Added pose nr. %d as nav. waypoint at (%.2f, %.2f), with distance %.2f to previous waypoint", 
                 (int)(path_it - navigation_path.poses.begin() - 1),
                 std::prev(path_it)->pose.position.x, std::prev(path_it)->pose.position.y,
                 getDistanceBetweenPoints(navigation_waypoints.end()[-1], 
                                          navigation_waypoints.end()[-2]));
      }
    }
  }
  // Pad the com beacons with the goal position if path not long enough to fit all
  if (navigation_waypoints.size() < nr_of_ugv_nav_waypoints and add_nav_waypoint_at_goal) {
    navigation_waypoints.push_back(navigation_path.poses.rbegin()->pose.position);
    ROS_DEBUG("Padded with goal pose as nav. waypoint at (%.2f, %.2f)", navigation_path.poses.rbegin()->pose.position.x, 
                                                                     navigation_path.poses.rbegin()->pose.position.y);
  }
}

std::vector<SensorCircle> UGVPlanner::getSensorCoverage(float ugv_sensor_radius) {
  std::vector<SensorCircle> sensor_coverages;
  int vehicle_id = -1;
  for (auto point_it = navigation_waypoints.begin(); point_it != navigation_waypoints.end(); ++point_it) {
    SensorCircle sensor_coverage;
    sensor_coverage.vehicle_id = vehicle_id;
    sensor_coverage.radius = ugv_sensor_radius;
    sensor_coverage.center = *point_it;
    sensor_coverages.push_back(sensor_coverage);
  }
  return sensor_coverages;
}