#include <MippPlanner.hpp>

// SUBMODULE FOR MIPP PLANNER

void MippPlanner::reshapeFormationUpdate() {
  ROS_DEBUG("reshapeFormationUpdate");
  if (run_simple_formation_) return;
  if (run_escorting_ or run_hybrid_) {
    // Update current position (shouldnt have changed but)
    //if (reshaping_formation_) return;
    reshaping_formation_ = true;
    current_formation_.clear();
    for (auto& uav_it : uav_planners_) {
      current_formation_.push_back(uav_it.formation_pose);
      ROS_DEBUG("Formation UAV%d (%.2f, %.2f, %.2f).", uav_it.uav_id,
                uav_it.formation_pose.position.x, 
                uav_it.formation_pose.position.y, 
                uav_it.formation_pose.position.z);
    }
    bool formation_viable = isFormationCollisionFree(current_formation_) and isFormationComConstrained(current_formation_);
    ROS_DEBUG_COND(formation_viable, "Current formation is collision free.");

    // Update formation bank
    std::map<float, std::vector<geometry_msgs::Pose>, std::greater<float>> new_formation_bank;
    ROS_DEBUG("Size %d, updating...", (int)formation_bank_.size());
    for (auto const& formation : formation_bank_) {
      formation_viable = isFormationCollisionFree(formation.second) and isFormationComConstrained(formation.second);
      if (formation_viable) {
        float formation_utility = getFormationUtility(formation.second, current_formation_);
        new_formation_bank.insert(std::pair<float, std::vector<geometry_msgs::Pose>>(formation_utility, formation.second));
      }
    }

    // Keep only top 5 formations
    while (new_formation_bank.size() > 5) {
      ROS_DEBUG("Size %d, removing...", (int)new_formation_bank.size());
      new_formation_bank.erase(std::prev(new_formation_bank.end()));
    }

    // Fill back to 10 with new random formations
    float sample_yaw_range = sample_yaw_range_;
    float sample_yaw_limit = sample_yaw_range_;
    if (ugv_planner_.navigation_goal_distance < planner_hybrid_distance_) {
      float yaw_left = M_PI - sample_yaw_range_;
      sample_yaw_range += yaw_left*(1.0 - ugv_planner_.navigation_goal_distance/planner_hybrid_distance_);
      ROS_DEBUG("Increasing yaw range to %.2f", sample_yaw_range);
    }
    while (new_formation_bank.size() < 20) {
      ROS_DEBUG("Size %d, adding...", (int)new_formation_bank.size());
      bool make_collision_free = true;
      std::vector<geometry_msgs::Pose> random_formation = getRandomColFreeFormation(current_formation_, sample_radius_, sample_yaw_range, sample_yaw_limit);
      int counter = 0;
      //bool formation_viable = isFormationCollisionFree(random_formation) and isFormationComConstrained(random_formation);
      formation_viable = isFormationComConstrained(random_formation);
      auto sample_radius = sample_radius_;
      while (!formation_viable) {
        if (counter < 10) {
          random_formation = getRandomColFreeFormation(current_formation_, sample_radius, sample_yaw_range);
          ROS_DEBUG("Cnt %d: (%.2f, %.2f)", counter, current_formation_.front().position.x, current_formation_.front().position.y);
          //formation_viable = isFormationCollisionFree(random_formation) and isFormationComConstrained(random_formation);
          formation_viable = isFormationComConstrained(random_formation);
          ROS_DEBUG_COND(!isFormationCollisionFree(random_formation), "Formation in collision.");
          ROS_DEBUG_COND(!isFormationComConstrained(random_formation), "Formation not comm.");
          ROS_DEBUG_COND(formation_viable, "Found collision free formation after %d attempts.", counter);
          counter++;
          sample_radius += 0.2;
        }
        else {
          ROS_DEBUG("Couldn't find collision free formation after %d attempts.", counter);
          random_formation.clear();
          for (auto const& uav_planner : uav_planners_) {
            random_formation.push_back(uav_planner.formation_poses.front());
          }
          formation_viable = true;
        }
        //ros::spinOnce();
      }
      float formation_utility = getFormationUtility(random_formation, current_formation_);
      ROS_DEBUG("Utility %.2f: (%.2f, %.2f)", formation_utility, current_formation_.front().position.x, current_formation_.front().position.y);
      if (new_formation_bank.count(formation_utility) > 0) {
        formation_utility -= 0.01*new_formation_bank.size();
      }
      new_formation_bank.insert(std::pair<float, std::vector<geometry_msgs::Pose>>(formation_utility, random_formation));
    }

    // Update formation bank with new one
    formation_bank_ = new_formation_bank;
    visualizeFormations(formation_bank_);

    // Use the best formation (will at least be collision free)
    current_formation_ = formation_bank_.begin()->second;
    for (auto& uav_planner : uav_planners_) {
      uav_planner.formation_pose = current_formation_.at(uav_planner.uav_id);
    }
    ROS_DEBUG("Chosen formation: (%.2f, %.2f)", current_formation_.front().position.x, current_formation_.front().position.y);

    // Visualize
    std::vector<SensorCircle> formation_sensor_circles;
    for (auto& uav_planner : uav_planners_) {
      formation_sensor_circles.push_back(makeSensorCircleFromUAVPose(uav_planner.getEscortPose(ugv_planner_.ugv_odometry->pose.pose, uav_planner.formation_pose).pose, uav_planner.uav_id, uav_camera_range_));
    }
    visualizeSensorCoverages(formation_sensor_circles);
    reshaping_formation_ = false;
  }
}

geometry_msgs::Point MippPlanner::getRandomCirclePoint(geometry_msgs::Point circle_center, float circle_radius) {
  ROS_DEBUG("generateRandomCirclePoint");
  geometry_msgs::Point sample_point;

  // Sample from unit circle, uniformly https://stackoverflow.com/a/50746409
  double radius = circle_radius*sqrt(unit_distribution_(generator_));
  double theta = unit_distribution_(generator_)*2.0*M_PI;
  sample_point.x = radius*cos(theta);
  sample_point.y = radius*sin(theta);
  sample_point.z = circle_center.z;
  ROS_DEBUG("Unit circle point (x,y) = (%f,%f)",sample_point.x,sample_point.y);

  sample_point.x += circle_center.x;
  sample_point.y += circle_center.y;

  return sample_point;
}

float MippPlanner::getRandomYaw(float yaw_center, float yaw_range) {
  ROS_DEBUG("generateRandomCirclePoint");
  float random_yaw = yaw_center + (2.0*unit_distribution_(generator_) - 1.0)*yaw_range;
  return angles::normalize_angle(random_yaw);
}

std::vector<geometry_msgs::Pose> MippPlanner::getRandomFormation(const std::vector<geometry_msgs::Pose>& current_formation, 
                                                                 float euc_range, float yaw_range) {
  ROS_DEBUG("getRandomFormation");
  std::vector<geometry_msgs::Pose> random_formation;
  for (auto const& formation_pose : current_formation) {
    geometry_msgs::Pose random_pose;
    random_pose.position = getRandomCirclePoint(formation_pose.position, euc_range);
    float current_yaw = makeRPYFromQuat(formation_pose.orientation).z;
    float random_yaw = getRandomYaw(0.0, yaw_range);
    ROS_DEBUG("Current to random: %.2f to %.2f", angles::to_degrees(current_yaw), angles::to_degrees(random_yaw));
    random_pose.orientation = makeQuatFromYaw(random_yaw);
    random_formation.push_back(random_pose);
  }
  return random_formation;
}

std::vector<geometry_msgs::Pose> MippPlanner::getRandomColFreeFormation(const std::vector<geometry_msgs::Pose>& current_formation, 
                                                                        float euc_range, float yaw_range, float yaw_limit) {
  ROS_DEBUG("getRandomColFreeFormation");
  if (yaw_limit == -1.0) {
    yaw_limit = yaw_range;
  }

  std::vector<geometry_msgs::Pose> random_formation;
  if (current_formation.size() != uav_planners_.size()) {
    ROS_ERROR("Was given formation of size %d != nr of uavs %d", (int)current_formation.size(), (int)uav_planners_.size());
    return random_formation;
  }

  for (auto& uav_planner : uav_planners_) {
    geometry_msgs::Pose random_pose;
    geometry_msgs::Pose current_pose = current_formation.at(uav_planner.uav_id);
    int counter = 0;
    bool pose_collision_free = false;
    while (!pose_collision_free and counter < 10) {
      ROS_DEBUG("Cnt %d: (%.2f, %.2f)", counter, random_pose.position.x, random_pose.position.y);
      random_pose.position = getRandomCirclePoint(current_pose.position, euc_range);
      float current_yaw = makeRPYFromQuat(current_pose.orientation).z;
      float random_yaw = getRandomYaw(0.0, yaw_range);
      if (yaw_limit != -1.0) {
        // yaw_limit has default value -1.0, only use if another value is given
        random_yaw = getClosestYaw(current_yaw, random_yaw, yaw_limit);
      }
      random_pose.orientation = makeQuatFromYaw(random_yaw);
      
      auto escort_path = uav_planner.getEscortPath(ugv_planner_.navigation_waypoints, random_pose);
      pose_collision_free = uav_planner.isPathCollisionFree(escort_path);
      counter++;
    }
    if (!pose_collision_free) {
      random_pose = uav_planner.formation_poses.front();
      ROS_WARN("Couldn't sample collision free pose for UAV%d, using formation bank position (%.2f, %.2f)", uav_planner.uav_id, random_pose.position.x, random_pose.position.y);
    }
    random_formation.push_back(random_pose);
  }
  return random_formation;
}

float MippPlanner::getFormationInfoGain(const std::vector<geometry_msgs::Pose>& formation_poses) {
  ROS_DEBUG("getFormationInfoGain");
  float info_gain;
  std::vector<SensorCircle> existing_sensor_coverages;
  for (auto& uav_planner : uav_planners_) {
    geometry_msgs::Pose formation_pose;
    try
    {
      formation_pose = formation_poses.at(uav_planner.uav_id);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Tried to access UAV%d, out of range.", uav_planner.uav_id);
    }
    nav_msgs::Path escort_path = uav_planner.getEscortPath(ugv_planner_.navigation_waypoints, formation_pose);
    std::vector<SensorCircle> escort_path_sensor_coverages;
    info_gain += uav_planner.getPathInfoGain(escort_path, 
                                             existing_sensor_coverages, escort_path_sensor_coverages);
    existing_sensor_coverages.insert(existing_sensor_coverages.end(), escort_path_sensor_coverages.begin(), escort_path_sensor_coverages.end());
  }
  return info_gain;
}

bool MippPlanner::isFormationCollisionFree(const std::vector<geometry_msgs::Pose>& formation_poses) {
  ROS_DEBUG("isFormationCollisionFree");

  if (formation_poses.size() != uav_planners_.size()) {
    ROS_ERROR("Was given formation of size %d != nr of uavs %d", (int)formation_poses.size(), (int)uav_planners_.size());
    return false;
  }

  for (auto& uav_planner : uav_planners_) {
    geometry_msgs::Pose formation_pose;
    try
    {
      formation_pose = formation_poses.at(uav_planner.uav_id);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Tried to access UAV%d, out of range.", uav_planner.uav_id);
    }
    nav_msgs::Path escort_path = uav_planner.getEscortPath(ugv_planner_.navigation_waypoints, formation_pose);
    if (!uav_planner.isPathCollisionFree(escort_path)) return false;
  }
  return true;
}

float MippPlanner::getDistanceBetweenFormations(const std::vector<geometry_msgs::Pose>& current_formation, const std::vector<geometry_msgs::Pose>& other_formation) {
  ROS_DEBUG("getDistanceBetweenFormations");
  
  if (current_formation.size() != other_formation.size()) {
    ROS_ERROR("Current formation size %d != other formation size %d", (int)current_formation.size(), (int)other_formation.size());
    return -1.0;
  }

  float distance = 0.0;
  for (int pose_i = 0; pose_i < current_formation.size(); pose_i++) {
    distance += getDistanceBetweenPoints(current_formation[pose_i].position, other_formation[pose_i].position);
  }
  return distance;
}

void MippPlanner::getDistanceBetweenFormations(const std::vector<geometry_msgs::Pose>& current_formation, const std::vector<geometry_msgs::Pose>& other_formation,
                                               float& ret_euc_distance, float& ret_yaw_distance) {
  ROS_DEBUG("getDistanceBetweenFormations");
  
  if (current_formation.size() != other_formation.size()) {
    ROS_ERROR("Current formation size %d != other formation size %d", (int)current_formation.size(), (int)other_formation.size());
    return;
  }

  float euc_distance = 0.0;
  float yaw_distance = 0.0;
  for (int pose_i = 0; pose_i < current_formation.size(); pose_i++) {
    euc_distance += getDistanceBetweenPoints(current_formation[pose_i].position, other_formation[pose_i].position);
    yaw_distance += getDistanceBetweenAngles(makeYawFromQuat(current_formation[pose_i].orientation), makeYawFromQuat(other_formation[pose_i].orientation));
  }
  ret_euc_distance = euc_distance;
  ret_yaw_distance = yaw_distance;
  return;
}

geometry_msgs::Pose MippPlanner::getEscortPose(const geometry_msgs::Pose& formation_pose) {
  ROS_DEBUG("getEscortPose");

  geometry_msgs::Pose escort_pose;
  geometry_msgs::Pose ugv_pose = ugv_planner_.ugv_odometry->pose.pose;
  float ugv_yaw = makeRPYFromQuat(ugv_pose.orientation).z;
  escort_pose.position = getRotatedPoint(ugv_yaw, formation_pose.position, makePoint(0,0,0));
  escort_pose.position.x += ugv_pose.position.x;
  escort_pose.position.y += ugv_pose.position.y;
  escort_pose.orientation = makeQuatFromAddingTwoYaws(ugv_pose.orientation, formation_pose.orientation);
  return escort_pose;
}

geometry_msgs::Pose MippPlanner::getEscortPose(const geometry_msgs::Pose& formation_pose, const geometry_msgs::Pose& ugv_pose) {
  ROS_DEBUG("getEscortPose");

  geometry_msgs::Pose escort_pose;
  float ugv_yaw = makeRPYFromQuat(ugv_pose.orientation).z;
  escort_pose.position = getRotatedPoint(ugv_yaw, formation_pose.position, makePoint(0,0,0));
  escort_pose.position.x += ugv_pose.position.x;
  escort_pose.position.y += ugv_pose.position.y;
  escort_pose.orientation = makeQuatFromAddingTwoYaws(ugv_pose.orientation, formation_pose.orientation);
  return escort_pose;
}

bool MippPlanner::isFormationComConstrained(std::vector<geometry_msgs::Pose> formation) {
  ROS_DEBUG("isFormationComConstrained");

  // Check range
  for (auto const& formation_pose : formation) {
    float formation_distance = getDistanceBetweenPoints(makePoint(0,0,0), formation_pose.position);
    if (formation_distance > (com_range_ - com_range_padding_)) {
      ROS_DEBUG("Out of range %.2f > %.2f - %.2f ? %d", formation_distance, com_range_, com_range_padding_, (int)(formation_distance > (com_range_ - com_range_padding_)));
      return false;
    }
  }

  // Check LoS, along all waypoints
  std::vector<geometry_msgs::Pose> ugv_poses;
  for (auto const& ugv_waypoint : ugv_planner_.navigation_waypoints) {
    geometry_msgs::Pose ugv_pose;
    ugv_pose.position = ugv_waypoint;
    if (ugv_poses.size() == 0) {
      ugv_pose.orientation = ugv_planner_.ugv_odometry->pose.pose.orientation;
    }
    else if (getDistanceBetweenPoints(ugv_waypoint, ugv_poses.back().position) > 0.01) {
      geometry_msgs::Vector3 waypoint_direction_vec = getDirection(ugv_poses.back().position, ugv_waypoint);
      ugv_pose.orientation = makeQuatFromYaw(atan2(waypoint_direction_vec.y, waypoint_direction_vec.x));
    }
    else {
      ugv_pose.orientation = ugv_planner_.ugv_odometry->pose.pose.orientation;
    }
    ugv_poses.push_back(ugv_pose);
  }

  bool ignore_unknown = false;
  int counter = 0;
  for (auto const& ugv_pose : ugv_poses) {
    if (counter > 2) ignore_unknown = true;
    for (auto const& formation_pose : formation) {
      geometry_msgs::Pose escort_pose = getEscortPose(formation_pose, ugv_pose);
      if (!doPointsHaveLOS(ugv_pose.position, escort_pose.position, ignore_unknown, octomap_)) {
        ROS_WARN("No LoS on %d: (%.2f, %.2f) -> (%.2f, %.2f)", counter, ugv_pose.position.x, ugv_pose.position.y, 
                                                               escort_pose.position.x, escort_pose.position.y);
        return false;
      } 
    }
    counter++;
  }
  
  return true;
}

float MippPlanner::getFormationUtility(const std::vector<geometry_msgs::Pose>& formation, const std::vector<geometry_msgs::Pose>& current_formation) {
  ROS_DEBUG("getFormationUtility");
  float info_gain = getFormationInfoGain(formation);
  float euc_dist, yaw_dist;
  getDistanceBetweenFormations(current_formation, formation, euc_dist, yaw_dist);
  ROS_DEBUG("Utility: info(%.2f), euc(%.2f), yaw(%.2f)", info_gain, euc_dist, yaw_dist);
  return c_info*info_gain + c_euc_dist*euc_dist + c_yaw_dist*yaw_dist;
}

void MippPlanner::visualizeFormations(std::map<float, std::vector<geometry_msgs::Pose>, std::greater<float>> formations) {
  ROS_DEBUG("visualizeFormations");
  if(formations.empty()){
    return;
  }

  float alpha_max = 1.0;
  float alpha_min = 0.1;
  float alpha_current = alpha_max;

  visualization_msgs::MarkerArray formations_marker_array;
  for (auto const& formation_it : formations) {
    visualization_msgs::Marker formation_marker;
    formation_marker.header.frame_id = uav_world_frame_;
    formation_marker.header.stamp = ros::Time::now();
    formation_marker.id = formations_marker_array.markers.size();
    formation_marker.type = visualization_msgs::Marker::LINE_LIST;
    formation_marker.action = visualization_msgs::Marker::ADD;
    formation_marker.pose.orientation.w = 1.0;
    formation_marker.scale.x = 0.04;
    formation_marker.color.r = 1.0;
    formation_marker.color.g = 1.0;
    formation_marker.color.b = 0.1;

    formation_marker.color.a = std::max(alpha_current, alpha_min);
    alpha_current -= 0.2;

    // ADD FOV rays
    tf2::Matrix3x3 ray_direction_rotmat;
    for (auto const& formation_pose : formation_it.second) {
      geometry_msgs::Pose escort_pose = getEscortPose(formation_pose);

      geometry_msgs::Vector3 rpy_vector = makeRPYFromQuat(escort_pose.orientation);
      geometry_msgs::Point origin = escort_pose.position;
      ray_direction_rotmat.setEulerYPR(rpy_vector.z, rpy_vector.y, rpy_vector.x);
      std::vector<geometry_msgs::Point> ray_endpoints;
      for(tf2::Vector3 ray : uav_camera_corner_rays_) {
        formation_marker.points.push_back(origin);
        tf2::Vector3 ray_direction = ray_direction_rotmat*ray*0.8;
        geometry_msgs::Point ray_endpoint;
        ray_endpoint.x = origin.x + ray_direction.getX();
        ray_endpoint.y = origin.y + ray_direction.getY();
        ray_endpoint.z = origin.z + ray_direction.getZ();
        formation_marker.points.push_back(ray_endpoint);
        ray_endpoints.push_back(ray_endpoint);
      }
      formation_marker.points.push_back(ray_endpoints[0]);
      formation_marker.points.push_back(ray_endpoints[1]);
      formation_marker.points.push_back(ray_endpoints[1]);
      formation_marker.points.push_back(ray_endpoints[3]);
      formation_marker.points.push_back(ray_endpoints[3]);
      formation_marker.points.push_back(ray_endpoints[2]);
      formation_marker.points.push_back(ray_endpoints[2]);
      formation_marker.points.push_back(ray_endpoints[0]);
    }
    
    formations_marker_array.markers.push_back(formation_marker);
    /*
    visualization_msgs::Marker formation_marker_2 = formation_marker;
    formation_marker_2.type = visualization_msgs::Marker::LINE_STRIP;
    formation_marker_2.id = formations_marker_array.markers.size();
    for (auto const& formation_pose : formation_it.second) {
      geometry_msgs::Pose escort_pose = getEscortPose(formation_pose);
      formation_marker_2.points.push_back(escort_pose.position);
    }
    geometry_msgs::Pose escort_pose = getEscortPose(formation_it.second.front());
    formation_marker_2.points.push_back(escort_pose.position);
    formations_marker_array.markers.push_back(formation_marker_2);*/
  }



  pub_viz_formations_.publish(formations_marker_array);
}