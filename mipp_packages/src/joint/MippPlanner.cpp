#include <MippPlanner.hpp>

// Constructor
  
MippPlanner::MippPlanner(ros::NodeHandle n, ros::NodeHandle np) 
  : act_mipp_server_(n, "/MippPlanner/start_mipp_action", boost::bind(&MippPlanner::actMipp, this, _1), false) {
  ROS_INFO("MippPlanner object is being created.");
  planner_initialized_ = false;

  // Initialize values
  getParams(np);

  tmr_run_updates_              = n.createTimer(ros::Duration(0.2), boost::bind(&MippPlanner::runUpdates, this));
  tmr_formation_reshape_planner_ = n.createTimer(ros::Duration(1.0), boost::bind(&MippPlanner::reshapeFormationUpdate, this));
  pub_ugv_pause_navigation_     = n.advertise<std_msgs::Bool>(ugv_ns_+"pause_navigation", 1);
  pub_viz_sensor_circle_        = n.advertise<visualization_msgs::Marker>("MippPlanner/viz_sensor_circle_", 1);
  pub_viz_sensor_coverages_     = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_sensor_coverages_", 1);
  pub_viz_uav_paths_            = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_uav_paths", 1);
  pub_viz_uav_path_fovs_        = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_uav_path_fovs", 1);
  pub_viz_nav_waypoints_        = n.advertise<visualization_msgs::Marker>("MippPlanner/viz_nav_waypoints", 1);
  pub_viz_formations_           = n.advertise<visualization_msgs::MarkerArray>("MippPlanner/viz_formations", 1);

  sub_clicked_point_  = n.subscribe("/exploration/start_collaborative", 1, &MippPlanner::subClickedPoint, this);
  sub_ugv_goal_plan_  = n.subscribe(ugv_ns_+"move_base/TebLocalPlannerROS/global_plan", 1, &MippPlanner::subUGVPlan, this);
  //sub_octomap_        = n.subscribe("/octomap_binary", 1, &MippPlanner::subOctomap, this);

  cli_planner_ready_ = n.advertiseService("/MippPlanner/planner_ready", &MippPlanner::cliIsPlannerReady, this);
  act_mipp_server_.start();

  cli_timer_get_octomap_ = n.createTimer(ros::Duration(1.0), boost::bind(&MippPlanner::cliGetOctomap, this));
  cli_get_octomap_ = n.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");

  // Random nr. generator and distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Set general variables
  run_exploration_ = false;
  run_escorting_ = false;
  run_hybrid_ = false;

  // Wait for map
  received_octomap_ = false;
  ros::Rate rate_wait_map(1.0);
  while (!received_octomap_) {
    ROS_WARN("No map received yet, waiting...");
    ros::spinOnce();
    rate_wait_map.sleep();
  }

  // Make a UGVPlanner object as container for variables for the UGV
  ugv_planner_.pub_goal_        = n.advertise<geometry_msgs::PoseStamped>(ugv_ns_+"move_base_simple/goal", 1);
  ugv_planner_.pub_goal_path_   = n.advertise<nav_msgs::Path>(ugv_ns_+"UGVFrontierExplorer/goal_path", 1);
  ugv_planner_.navigation_goal.pose.position.x = ugv_start_x_;
  ugv_planner_.navigation_goal.pose.position.y = ugv_start_y_;
  ugv_planner_.ugv_odometry = std::make_shared<nav_msgs::Odometry>();
  ugv_planner_.init(n);
  // Make a UAVPlanner object as container for variables for each UAV
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    // Declare new UAVPlanner object and init values that have to be initialized from MippPlanner object
    ROS_INFO("Making UAV%d", uav_id);
    UAVPlanner uav_planner;
    uav_planner.uav_id = uav_id;
    uav_planner.com_range = com_range_;
    uav_planner.uav_altitude = uav_id*0.6 + 1.4;
    uav_planner.global_ugv_waypoints = &ugv_planner_.navigation_waypoints;
    uav_planner.global_sensor_coverages = &uav_sensor_coverages_;
    uav_planner.global_uav_paths = &uav_paths_;
    uav_planner.global_run_exploration = &run_exploration_;
    uav_planner.global_run_escorting = &run_escorting_;
    uav_planner.camera_range = uav_camera_range_;
    uav_planner.use_formation_bank = run_simple_formation_;
    // Add object to list
    uav_planners_.push_back(uav_planner);
  }

  // Set UGV variables
  ugv_planner_.navigation_waypoint_max_dist = ugv_nav_waypoint_max_distance_;
  ugv_planner_.navigation_paused = false;

  // Set UAV variables
  float uav_camera_vfov_ = (uav_camera_height_/uav_camera_width_)*uav_camera_hfov_;
  tf2::Matrix3x3 ray_rot_mat;
  tf2::Vector3 unit_ray_direction(1.0, 0.0, 0.0);
  std::vector<tf2::Vector3> uav_camera_rays;
  for (float img_y = -uav_camera_height_/2.0; img_y <= uav_camera_height_/2.0+uav_camera_ray_resolution_/2.0; img_y+=uav_camera_ray_resolution_){
    for (float img_x = -uav_camera_width_/2.0; img_x <= uav_camera_width_/2.0+uav_camera_ray_resolution_/2.0; img_x+=uav_camera_ray_resolution_){
      float ray_yaw = (img_x/uav_camera_width_)*uav_camera_hfov_;
      float ray_pitch = (img_y/uav_camera_height_)*uav_camera_vfov_;
      ray_rot_mat.setEulerYPR(ray_yaw, ray_pitch, 0.0);
      tf2::Vector3 ray_direction = ray_rot_mat*unit_ray_direction;
      uav_camera_rays.push_back(ray_direction);
      ROS_DEBUG("YPR: (%f, %f, %f)", ray_yaw, ray_pitch, 0.0);
      ROS_DEBUG("Ray: (%f, %f, %f)", ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
      if (abs(img_y) == uav_camera_height_/2.0 and abs(img_x) == uav_camera_width_/2.0) {
        uav_camera_corner_rays_.push_back(ray_direction);
      }
    }
  }

  ros::spinOnce();
  ugv_planner_.vehicle_state = IDLE;
  for (auto& uav_planner_it : uav_planners_) {
    std::string uav_ns = "/uav"+std::to_string(uav_planner_it.uav_id);
    uav_planner_it.pub_position_goal = n.advertise<geometry_msgs::PoseStamped>(uav_ns+"/uav_server/position_goal", 1);
    // Global
    uav_planner_it.global_ugv_odometry = ugv_planner_.ugv_odometry;
    ROS_WARN("UAV use count %d: ugv_odometry", (int)uav_planner_it.global_ugv_odometry.use_count());
    uav_planner_it.octomap = octomap_;
    ROS_WARN("Glo use count %d: octomap", (int)octomap_.use_count());
    ROS_WARN("UAV use count %d: octomap", (int)uav_planner_it.octomap.use_count());
    uav_planner_it.formation_pose = getFormationPose(uav_planner_it.uav_id);
    uav_planner_it.sample_radius = sample_radius_;
    uav_planner_it.sample_yaw_range = sample_yaw_range_;
    // Info
    uav_planner_it.info_camera_rays = uav_camera_rays;
    // Escort
    uav_planner_it.initFormationPoseBank(nr_of_uavs_);
    // Utility
    uav_planner_it.c_info = c_info;
    uav_planner_it.c_euc_dist = c_euc_dist;
    uav_planner_it.c_yaw_dist = c_yaw_dist;
    // Init
    uav_planner_it.init(n);
  }

  planner_initialized_ = true;
  ROS_WARN("Done.");
}

// Destructor
  
MippPlanner::~MippPlanner() {
  ROS_INFO("MippPlanner object is being deleted.");
}

// Publish functions

void MippPlanner::runUpdates() {
  ROS_DEBUG("runUpdates");
  if (!planner_initialized_) {
    return;
  }

  // Process UGV related things
  std_msgs::Bool pub_msg;
  pub_msg.data = ugv_planner_.navigation_paused;
  pub_ugv_pause_navigation_.publish(pub_msg);
  // Make UGV navigation waypoints
  ugv_planner_.createNavigationWaypoints(nr_of_ugv_nav_waypoints_, true);
  visualizeNavWaypoints();
  
  // If in hybrid mode, check if switch needs to be done
  ugv_planner_.navigation_goal_distance = getDistanceBetweenPoints(ugv_planner_.ugv_odometry->pose.pose.position, ugv_planner_.navigation_goal.pose.position);
  ROS_INFO_THROTTLE(1.0, "Distance %.2f", ugv_planner_.navigation_goal_distance);
  if (run_hybrid_) {
    if (ugv_planner_.navigation_goal_distance < planner_hybrid_distance_) {
      ROS_INFO_COND(run_escorting_, "Switching to EXPLORATION_MODE");
      for (auto& uav_planner_it : uav_planners_) {
        *(uav_planner_it.global_run_exploration) = true;
        *(uav_planner_it.global_run_escorting) = false;
      }
      run_exploration_ = true;
      run_escorting_ = false;
    }
    else if (ugv_planner_.navigation_goal_distance > planner_hybrid_distance_) {
      ROS_INFO_COND(run_exploration_, "Switching to ESCORTING_MODE");
      for (auto& uav_planner_it : uav_planners_) {
        *(uav_planner_it.global_run_exploration) = false;
        *(uav_planner_it.global_run_escorting) = true;
      }
      run_exploration_ = false;
      run_escorting_ = true;
    }
    else if (!run_exploration_ and !run_escorting_) {
      run_escorting_ = true;
      run_exploration_ = false;
    }
  }

  // Check if UAVs are still fulfilling com constraints
  for (auto& uav_it : uav_planners_) {
    uav_it.recovery_goal.header = ugv_planner_.ugv_odometry->header;
    uav_it.recovery_goal.pose = ugv_planner_.ugv_odometry->pose.pose;
    uav_it.recovery_goal.pose.position.z = uav_it.uav_altitude;
    auto uav_distance = getDistanceBetweenPoints(ugv_planner_.ugv_odometry->pose.pose.position, uav_it.uav_odometry.pose.pose.position);
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
  ugv_planner_.navigation_goal = *(path_msg->poses.rbegin());
}

void MippPlanner::subOctomap(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {
  ROS_DEBUG("subOctomap");
  octomap_ = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(*octomap_msg)));
  for (auto& uav_planner : uav_planners_) {
    uav_planner.octomap = octomap_;
  }
  received_octomap_ = true;

}

// Services

bool MippPlanner::cliIsPlannerReady(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  ROS_DEBUG("cliIsPlannerReady");
  response.success = planner_initialized_;
  for (auto& uav_planner_it : uav_planners_) {
    mipp_msgs::TakeoffComplete srv;
    if (uav_planner_it.takeoff_client.call(srv)) {
      response.success = response.success and srv.response.takeoff_complete;
    }
    else {
      ROS_ERROR("UAV%d couldn't call takeoff_complete server.", uav_planner_it.uav_id);
    }
  }
  return true;
}

void MippPlanner::cliGetOctomap() {
  ROS_DEBUG("cliGetOctomap");
  octomap_msgs::GetOctomap octomap_srv;
  if (cli_get_octomap_.call(octomap_srv)) {
    received_octomap_ = true;
    octomap_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(octomap_srv.response.map)));
    for (auto& uav_planner : uav_planners_) {
      uav_planner.octomap = octomap_;
    }
    ROS_WARN("Got OctoMap");
  }
  else {
    received_octomap_ = false;
    ROS_ERROR("Failed to get OctoMap");
  }
}

// Actionlib

void MippPlanner::actMipp(const mipp_msgs::StartMippGoalConstPtr &goal) {
  ROS_WARN("actMipp");

  while (!planner_initialized_) {
    ROS_WARN("Planner still initializing...");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  bool uavs_taken_off = false;
  while (!uavs_taken_off) {
    ROS_WARN("UAVs still taking off...");
    uavs_taken_off = true;
    for (auto const& uav_planner_it :uav_planners_) {
      uavs_taken_off = uavs_taken_off and (uav_planner_it.vehicle_state != INIT);
    }
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  // Start
  ROS_INFO("Starting MIPP with mode %d.", (int)(goal->mipp_mode));
  ros::Time start_time = ros::Time::now();
  for (auto const& uav_planner_it : uav_planners_) {
    switch (goal->mipp_mode)
    {
    case mipp_msgs::StartMippGoal::EXPLORATION_MODE:
      ROS_INFO("UAV%d planner set to: EXPLORATION_MODE.", uav_planner_it.uav_id);
      run_exploration_ = true;
      run_escorting_ = false;
      run_hybrid_ = false;
      break;
    case mipp_msgs::StartMippGoal::ESCORTING_MODE:
      ROS_INFO("UAV%d planner set to: ESCORTING_MODE.", uav_planner_it.uav_id);
      run_exploration_ = false;
      run_escorting_ = true;
      run_hybrid_ = false;
      break;
    case mipp_msgs::StartMippGoal::HYBRID_MODE:
      ROS_INFO("UAV%d planner set to: HYBRID_MODE.", uav_planner_it.uav_id);
      run_exploration_ = false;
      run_escorting_ = false;
      run_hybrid_ = true;
      break;
    default:
      ROS_ERROR("Got weird value for mipp_mode: %d", (int)(goal->mipp_mode));
      break;
    }
    *(uav_planner_it.global_run_exploration) = run_exploration_;
    *(uav_planner_it.global_run_escorting) = run_escorting_;
  }
  while ((ros::Time::now() - start_time).toSec() < goal->max_time) {
    act_mipp_feedback_.voxels_discovered = octomap_size_;
    act_mipp_feedback_.time_used = (ros::Time::now() - start_time).toSec();
    act_mipp_server_.publishFeedback(act_mipp_feedback_);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
}

// PLanner functions

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
  np.param<float>("planner_com_range_padding", com_range_padding_, 1.0);
  np.param<float>("planner_hybrid_distance", planner_hybrid_distance_, 5.0);
  // Planners
  np.param<float>("planner_sample_radius", sample_radius_, 1.0);
  np.param<float>("planner_sample_yaw_range", sample_yaw_range_, M_PI/6.0);
  np.param<bool>("run_simple_formation", run_simple_formation_, true);
  // Utility
  np.param<float>("c_info", c_info, 10.0);
  np.param<float>("c_euc_dist", c_euc_dist, -1.0);
  np.param<float>("c_yaw_dist", c_yaw_dist, -1.0);
  // UGV
  np.param<float>("ugv_start_x", ugv_start_x_, 0.0);
  np.param<float>("ugv_start_y", ugv_start_y_, 0.0);
  np.param<std::string>("ugv_ns", ugv_ns_, "/ugv/");
  np.param<int>("nr_of_ugv_nav_waypoints", nr_of_ugv_nav_waypoints_, 5);
  np.param<float>("ugv_nav_waypoint_max_distance", ugv_nav_waypoint_max_distance_, 2.0);
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

geometry_msgs::Pose MippPlanner::getFormationPose(int uav_id) {
  // Hard coded, sorry
  geometry_msgs::Pose formation_pose;
  formation_pose.position.z = (double)(uav_id*0.75 + 1.5);
  formation_pose.orientation.w = 1.0;
  switch (uav_id)
  {
  case 0:
    formation_pose.position.x = 3.0;
    formation_pose.position.y = 0.0;
    break;
  case 1:
    formation_pose.position.x = -2.0;
    formation_pose.position.y = -2.0;
    break;
  case 2:
    formation_pose.position.x = -2.0;
    formation_pose.position.y = 2.0;
    break;
  default:
    ROS_ERROR("Couldn't find a hard coded formation for UAV%d", uav_id);
    break;
  }
  return formation_pose;
}

bool MippPlanner::doPointsHaveLOS(const geometry_msgs::Point point_a, const geometry_msgs::Point point_b) {
  ROS_DEBUG("doPointsHaveLOS");

  double occupancy_threshold = octomap_->getOccupancyThres();
  float point_distance = getDistanceBetweenPoints(point_a, point_b);
  octomap::point3d om_end_point;

  octomap::point3d om_point_a(point_a.x, point_a.y, point_a.z);
  geometry_msgs::Vector3 direction_ab = getDirection(point_a, point_b);
  octomap::point3d om_direction_ab(direction_ab.x, direction_ab.y, direction_ab.z);
  bool hit_occupied_ab = octomap_->castRay(om_point_a, om_direction_ab, om_end_point, false, point_distance);

  octomap::point3d om_point_b(point_b.x, point_b.y, point_b.z);
  geometry_msgs::Vector3 direction_ba = getDirection(point_b, point_a);
  octomap::point3d om_direction_ba(direction_ba.x, direction_ba.y, direction_ba.z);
  bool hit_occupied_ba = octomap_->castRay(om_point_b, om_direction_ba, om_end_point, false, point_distance);

  return (!hit_occupied_ab and !hit_occupied_ba);
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

    visualization_msgs::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.id = marker_array.markers.size();
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = circle_it->center;
    text_marker.color.a = 1.0;
    text_marker.scale.z = 1.0;
    text_marker.text = std::to_string(circle_it->vehicle_id).c_str();
    marker_array.markers.push_back(text_marker);
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
    if (path_it.header.frame_id == "collision") {
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.1;
      path_marker.color.b = 0.1;
    }
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
    if (path_it.header.frame_id == "collision") {
      fov_marker.color.r = 1.0;
      fov_marker.color.g = 0.1;
      fov_marker.color.b = 0.1;
    }
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