#include <MippPlanner.hpp>

// Constructor
  
MippPlanner::MippPlanner(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_INFO("MippPlanner object is being created.");

  // Initialize values
  getParams(np);

  tmr_run_updates_              = n.createTimer(ros::Duration(0.5), boost::bind(&MippPlanner::runUpdates, this));
  pub_ugv_pause_navigation_     = n.advertise<std_msgs::Bool>(ugv_ns_+"pause_navigation", 1);
  pub_viz_sensor_circle_        = n.advertise<visualization_msgs::Marker>("MippPlanner/viz_sensor_circle_", 1);
  pub_viz_sensor_coverages_     = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_sensor_coverages_", 1);
  pub_viz_uav_paths_            = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_uav_paths", 1);
  pub_viz_uav_path_fovs_        = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_uav_path_fovs", 1);
  pub_viz_nav_waypoints_        = n.advertise<visualization_msgs::Marker>("MippPlanner/viz_nav_waypoints", 1);

  sub_clicked_point_ = n.subscribe("/exploration/start_collaborative", 1, &MippPlanner::subClickedPoint, this);
  sub_ugv_goal_plan_ = n.subscribe(ugv_ns_+"move_base/TebLocalPlannerROS/global_plan", 1, &MippPlanner::subUGVPlan, this);

  // Make a UGVPlanner object as container for variables for the UGV
  ugv_planner_.pub_goal_        = n.advertise<geometry_msgs::PoseStamped>(ugv_ns_+"move_base_simple/goal", 1);
  ugv_planner_.pub_goal_path_   = n.advertise<nav_msgs::Path>(ugv_ns_+"UGVFrontierExplorer/goal_path", 1);
  ugv_planner_.navigation_goal.pose.position.x = ugv_start_x_;
  ugv_planner_.navigation_goal.pose.position.y = ugv_start_y_;
  ugv_planner_.init(n);
  // Make a UAVPlanner object as container for variables for each UAV
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    // Declare new UAVPlanner object and init values that have to be initialized from MippPlanner object
    ROS_INFO("Making UAV%d", uav_id);
    UAVPlanner uav_planner;
    uav_planner.uav_id = uav_id;
    uav_planner.com_range = com_range_;
    uav_planner.global_ugv_waypoints = &ugv_planner_.navigation_waypoints;
    uav_planner.global_sensor_coverages = &uav_sensor_coverages_;
    uav_planner.global_uav_paths = &uav_paths_;
    uav_planner.camera_range = uav_camera_range_;
    // Add object to list
    uav_planners_.push_back(uav_planner);
  }

  // Set general variables
  running_exploration_ = false;

  // Set UGV variables
  ugv_planner_.navigation_waypoint_max_dist = ugv_nav_waypoint_max_distance_;
  ugv_planner_.navigation_paused = false;

  // Set UAV variables
  float uav_camera_vfov_ = (uav_camera_height_/uav_camera_width_)*uav_camera_hfov_;
  tf2::Matrix3x3 ray_rot_mat;
  tf2::Vector3 unit_ray_direction(1.0, 0.0, 0.0);
  for (float img_y = -uav_camera_height_/2.0; img_y <= uav_camera_height_/2.0+uav_camera_ray_resolution_/2.0; img_y+=uav_camera_ray_resolution_){
    for (float img_x = -uav_camera_width_/2.0; img_x <= uav_camera_width_/2.0+uav_camera_ray_resolution_/2.0; img_x+=uav_camera_ray_resolution_){
      float ray_yaw = (img_x/uav_camera_width_)*uav_camera_hfov_;
      float ray_pitch = (img_y/uav_camera_height_)*uav_camera_vfov_;
      ray_rot_mat.setEulerYPR(ray_yaw, ray_pitch, 0.0);
      tf2::Vector3 ray_direction = ray_rot_mat*unit_ray_direction;
      ROS_DEBUG("YPR: (%f, %f, %f)", ray_yaw, ray_pitch, 0.0);
      ROS_DEBUG("Ray: (%f, %f, %f)", ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
      if (abs(img_y) == uav_camera_height_/2.0 and abs(img_x) == uav_camera_width_/2.0) {
        uav_camera_corner_rays_.push_back(ray_direction);
      }
    }
  }

  ros::spinOnce();
  ros::Duration(10.0).sleep();
  ugv_planner_.vehicle_state = IDLE;
  for (auto& uav_planner_it : uav_planners_) {
    uav_planner_it.init(n);
  }

  ROS_WARN("Done.");
}

// Destructor
  
MippPlanner::~MippPlanner() {
  ROS_INFO("MippPlanner object is being deleted.");
}

// Publish functions

void MippPlanner::runUpdates() {
  ROS_DEBUG("runUpdates");

  // Process UGV related things
  std_msgs::Bool pub_msg;
  pub_msg.data = ugv_planner_.navigation_paused;
  pub_ugv_pause_navigation_.publish(pub_msg);
  // Make UGV navigation waypoints
  ugv_planner_.createNavigationWaypoints(nr_of_ugv_nav_waypoints_, true);
  visualizeNavWaypoints();

  // Check if UAVs are still fulfilling com constraints
  for (auto& uav_it : uav_planners_) {
    uav_it.recovery_goal.header = ugv_planner_.ugv_odometry.header;
    uav_it.recovery_goal.pose = ugv_planner_.ugv_odometry.pose.pose;
    uav_it.recovery_goal.pose.position.z = (double)(uav_it.uav_id*0.75 + 1.5);
    auto uav_distance = getDistanceBetweenPoints(ugv_planner_.ugv_odometry.pose.pose.position, uav_it.uav_odometry.pose.pose.position);
    uav_it.out_of_com_range = (uav_distance > com_range_);
  }

  // Visualize
  std::vector<nav_msgs::Path> uav_paths;
  for (auto const& uav_path_it : uav_paths_) {
    uav_paths.push_back(uav_path_it.second);
  }
  visualizePaths(uav_paths);
  visualizePathFOVs(uav_paths, 1.0);
  std::vector<SensorCircle> sensor_coverages;
  for (auto const& uav_coverages_it : uav_sensor_coverages_) {
    for (auto const& uav_coverage_it : uav_coverages_it.second) {
      sensor_coverages.push_back(uav_coverage_it);
    }
  }
  visualizeSensorCoverages(sensor_coverages);
}

// Callback functions for subscriptions

void MippPlanner::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_DEBUG("subClickedPoint");

  //makePlanSynchronous();
  std::vector<nav_msgs::Path> uav_paths;
  for (auto const& uav_path_it : uav_paths_) {
    uav_paths.push_back(uav_path_it.second);
  }
  visualizePaths(uav_paths);
  visualizePathFOVs(uav_paths, 1.0);
  std::vector<SensorCircle> sensor_coverages;
  for (auto const& uav_coverages_it : uav_sensor_coverages_) {
    for (auto const& uav_coverage_it : uav_coverages_it.second) {
      sensor_coverages.push_back(uav_coverage_it);
    }
  }
  visualizeSensorCoverages(sensor_coverages);
}

void MippPlanner::subUGVPlan(const nav_msgs::PathConstPtr& path_msg) {
  ROS_DEBUG("subUGVPlan");

  ugv_planner_.navigation_path = *path_msg;
}

// PLanner functions

void MippPlanner::makePlanIndividual(int vehicle_id) {

}

void MippPlanner::makePlanSynchronous() {
  ROS_DEBUG("makePlanSynchronous");
  ros::Rate check_rate(10);

  ROS_WARN("Starting collaborative exploration with 1 UGV and %d UAVs", nr_of_uavs_);

  // Pause UGV navigation so it does not move until everyone has plan
  ugv_planner_.navigation_paused = true;

  // Set "start exploration" goal (max time to compute paths) and send to vehicle exploration servers
  float exploration_max_time = 2.0;
  ugv_planner_.sendExplorationGoal(exploration_max_time);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    uav_planners_[uav_id].sendExplorationGoal(exploration_max_time);
  }
  ROS_INFO("UGV planner state after call: %s", ugv_planner_.exploration_client->getState().toString().c_str());

  // Poll exploration servers and wait until every server is done
  bool vehicles_ready = ugv_planner_.isExplorationDone();
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    vehicles_ready = vehicles_ready && uav_planners_[uav_id].isExplorationDone();
  }
  while (!vehicles_ready) {
    ROS_INFO_THROTTLE(1, "Vehicles not finished exploring, UGV state: %s", ugv_planner_.exploration_client->getState().toString().c_str());
    vehicles_ready = ugv_planner_.isExplorationDone();
    for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
      vehicles_ready = vehicles_ready && uav_planners_[uav_id].isExplorationDone();
    }
    ros::spinOnce();
    check_rate.sleep();
  }

  // Get results
  ugv_planner_.exploration_result = ugv_planner_.getExplorationResult();
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    uav_planners_[uav_id].exploration_result = uav_planners_[uav_id].getExplorationResult();
  }
  
  // UGV planner will make a plan first (but is still paused)
  ugv_planner_.createInitNavigationPlan();

  // Work on path made by UGV planner and create communication "beacons"
  ugv_planner_.createNavigationWaypoints(nr_of_ugv_nav_waypoints_, add_nav_waypoint_at_goal_);
  ugv_planner_.navigation_paused = false;

  ROS_WARN("Done");
}

// Utility functions

void MippPlanner::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  // General
  np.param<std::string>("planner_world_frame", planner_world_frame_, "world");
  np.param<float>("planner_com_range", com_range_, 10.0);
  // UGV
  np.param<float>("ugv_start_x", ugv_start_x_, 0.0);
  np.param<float>("ugv_start_y", ugv_start_y_, 0.0);
  np.param<std::string>("ugv_ns", ugv_ns_, "/ugv/");
  np.param<int>("nr_of_ugv_nav_waypoints", nr_of_ugv_nav_waypoints_, 4);
  np.param<float>("ugv_nav_waypoint_max_distance", ugv_nav_waypoint_max_distance_, 5.0);
  np.param<bool>("add_nav_waypoint_at_goal", add_nav_waypoint_at_goal_, true);
  np.param<float>("ugv_sensor_radius", ugv_sensor_radius_, 7.5);
  // UAVS
  np.param<int>("nr_of_uavs", nr_of_uavs_, 0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<float>("uav_camera_width", uav_camera_width_, 6.0);
  np.param<float>("uav_camera_height", uav_camera_height_, 3.0);
  np.param<float>("uav_camera_hfov", uav_camera_hfov_, 1.02974);
  np.param<float>("uav_camera_ray_resolution", uav_camera_ray_resolution_, 1.0);
  np.param<float>("uav_camera_range", uav_camera_range_, 7.5);
}

// Visualization

void MippPlanner::visualizeSensorCircle(SensorCircle sensor_circle) {
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

void MippPlanner::visualizeSensorCoverages(std::vector<SensorCircle> sensor_coverages) {
  ROS_DEBUG("visualizeSensorCoverages");
  visualization_msgs::MarkerArray marker_array;

  for (auto circle_it = sensor_coverages.begin(); circle_it != sensor_coverages.end(); ++circle_it) {
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

void MippPlanner::visualizePaths(std::vector<nav_msgs::Path> paths) {
  ROS_DEBUG("visualizePaths");
  if(paths.empty()){
    return;
  }

  bool add_current_pose_to_path = false;

  visualization_msgs::MarkerArray path_marker_array;
  for (auto const& path_it : paths) {
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = planner_world_frame_;
    path_marker.header.stamp = ros::Time::now();
    path_marker.id = path_marker_array.markers.size();
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.orientation.w = 1.0;
    path_marker.scale.x = 0.05;
    path_marker.color.a = 1.0;
    path_marker.color.r = 0.1;
    path_marker.color.g = 1.0;
    path_marker.color.b = 0.1;
    if (add_current_pose_to_path) {
      geometry_msgs::Point current_pose = path_it.poses.begin()->pose.position;
      path_marker.points.push_back(current_pose);
    }
    for (auto const& pose_it : path_it.poses) {
      path_marker.points.push_back(pose_it.pose.position);
    }
    path_marker_array.markers.push_back(path_marker);
  }
  
  pub_viz_uav_paths_.publish(path_marker_array);
}

void MippPlanner::visualizePathFOVs(std::vector<nav_msgs::Path> paths, float ray_length) {
  ROS_DEBUG("visualizePathFOVs");
  if(paths.empty()){
    return;
  }

  visualization_msgs::MarkerArray fov_marker_array;
  for (auto const& path_it : paths) {
    visualization_msgs::Marker fov_marker;
    fov_marker.header.frame_id = uav_world_frame_;
    fov_marker.header.stamp = ros::Time::now();
    fov_marker.id = fov_marker_array.markers.size();
    fov_marker.type = visualization_msgs::Marker::LINE_LIST;
    fov_marker.action = visualization_msgs::Marker::ADD;
    fov_marker.pose.orientation.w = 1.0;
    fov_marker.scale.x = 0.05;
    fov_marker.color.a = 0.8;
    fov_marker.color.r = 0.1;
    fov_marker.color.g = 1.0;
    fov_marker.color.b = 0.1;
    bool show_all_rays = false;
    tf2::Matrix3x3 ray_direction_rotmat;
    for (auto const& path_pose : path_it.poses) {
      geometry_msgs::Vector3 rpy_vector = makeRPYFromQuat(path_pose.pose.orientation);
      geometry_msgs::Point origin = path_pose.pose.position;
      ray_direction_rotmat.setEulerYPR(rpy_vector.z, rpy_vector.y, rpy_vector.x);
      std::vector<geometry_msgs::Point> ray_endpoints;
      for(tf2::Vector3 ray : uav_camera_corner_rays_) {
        fov_marker.points.push_back(origin);
        tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_length;
        geometry_msgs::Point ray_endpoint;
        ray_endpoint.x = origin.x + ray_direction.getX();
        ray_endpoint.y = origin.y + ray_direction.getY();
        ray_endpoint.z = origin.z + ray_direction.getZ();
        fov_marker.points.push_back(ray_endpoint);
        ray_endpoints.push_back(ray_endpoint);
      }
      fov_marker.points.push_back(ray_endpoints[0]);
      fov_marker.points.push_back(ray_endpoints[1]);
      fov_marker.points.push_back(ray_endpoints[1]);
      fov_marker.points.push_back(ray_endpoints[3]);
      fov_marker.points.push_back(ray_endpoints[3]);
      fov_marker.points.push_back(ray_endpoints[2]);
      fov_marker.points.push_back(ray_endpoints[2]);
      fov_marker.points.push_back(ray_endpoints[0]);
    }
    fov_marker_array.markers.push_back(fov_marker);
  }
  pub_viz_uav_path_fovs_.publish(fov_marker_array);
}

void MippPlanner::visualizeNavWaypoints() {
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = uav_world_frame_;
  line_marker.header.stamp = ros::Time::now();
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.2;
  line_marker.color.a = 0.8;
  line_marker.color.r = 0.1;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.1;
  for (auto const& nav_waypoints : ugv_planner_.navigation_waypoints) {
    line_marker.points.push_back(makePoint(nav_waypoints.x, nav_waypoints.y, 0.0));
    line_marker.points.push_back(makePoint(nav_waypoints.x, nav_waypoints.y, 2.0));
  }
  pub_viz_nav_waypoints_.publish(line_marker);
}