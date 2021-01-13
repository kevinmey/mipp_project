#include <MippPlanner.hpp>

// SUBMODULE FOR EXPLORATION PLANNER

void UGVPlanner::init(ros::NodeHandle n) {
  sub_odometry     = n.subscribe("/gazebo/ground_truth_ugv", 1, &UGVPlanner::subOdometry, this);
  vehicle_state    = INIT;
  exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>("/ugv/exploration_action", true);
  exploration_client->waitForServer(ros::Duration(10.0));
  navigation_path.poses.push_back(navigation_goal);
}

void UGVPlanner::subOdometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  //auto ugv_odom = *odom_msg;
  //ugv_odom.pose.pose.position.z = 0.6;
  *ugv_odometry = *odom_msg;
}

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
      float dist_to_prev_point = getDistanceBetweenPoints(path_it->pose.position, std::prev(path_it)->pose.position);
      dist_to_prev_beacon += dist_to_prev_point;
      if (dist_to_prev_beacon > navigation_waypoint_max_dist and navigation_waypoints.size() < nr_of_ugv_nav_waypoints) {
        // Distance to previous beacon exceeds max distance, make new point
        geometry_msgs::Point new_point = castRay(std::prev(path_it)->pose.position, 
                                                 getDirection(std::prev(path_it)->pose.position, path_it->pose.position), 
                                                 navigation_waypoint_max_dist - (dist_to_prev_beacon - dist_to_prev_point));
        navigation_waypoints.push_back(new_point);
        dist_to_prev_beacon = getDistanceBetweenPoints(path_it->pose.position, new_point);
        ROS_DEBUG("Added pose nr. %d as nav. waypoint at (%.2f, %.2f), with distance %.2f to previous waypoint", 
                 (int)(path_it - navigation_path.poses.begin() - 1),
                 std::prev(path_it)->pose.position.x, std::prev(path_it)->pose.position.y,
                 getDistanceBetweenPoints(navigation_waypoints.end()[-1], 
                                          navigation_waypoints.end()[-2]));
      }
    }
  }
  // Pad the com beacons with the goal position if path not long enough to fit all
  while (navigation_waypoints.size() < nr_of_ugv_nav_waypoints and add_nav_waypoint_at_goal) {
    navigation_waypoints.push_back(navigation_path.poses.rbegin()->pose.position);
    ROS_DEBUG("Padded with goal pose as nav. waypoint at (%.2f, %.2f)", navigation_path.poses.rbegin()->pose.position.x, 
                                                                     navigation_path.poses.rbegin()->pose.position.y);
  }

  // Increase height of waypoints
  float ugv_antenna_height = 1.0;
  for (auto& waypoint : navigation_waypoints) {
    waypoint.z = ugv_antenna_height;
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