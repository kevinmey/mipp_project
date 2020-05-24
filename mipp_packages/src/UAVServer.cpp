#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("UAVServer object is being created.");

  // Initialize values
  getParams(np);
  uav_local_goal_.pose.position.x = uav_start_x_;
  uav_local_goal_.pose.position.y = uav_start_y_;
  uav_local_goal_.pose.position.z = uav_takeoff_z_;
  uav_local_goal_.pose.orientation.w = 1.0;
  uav_global_goal_ = uav_local_goal_;
  uav_running_exploration_ = false;
  // Establish publish timers and publishers
  pub_timer_mavros_setpoint_  = n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_        = n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  pub_global_goal_            = n.advertise<geometry_msgs::PoseStamped>("uav_server/global_goal", 10);
  pub_viz_uav_fov_            = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav_fov", 1);
  pub_viz_fov_                = n.advertise<visualization_msgs::Marker>("uav_server/viz_fov", 1);
  pub_viz_information_points_ = n.advertise<visualization_msgs::Marker>("uav_server/viz_information_points", 1);
  pub_viz_tree_               = n.advertise<visualization_msgs::Marker>("uav_server/viz_tree", 1);
  pub_viz_path_               = n.advertise<visualization_msgs::Marker>("uav_server/viz_path", 1);
  // Establish subscriptions
  sub_clicked_point_  = n.subscribe("/clicked_point", 1, &UAVServer::subClickedPoint, this);
  sub_clicked_pose_   = n.subscribe("/yeet", 1, &UAVServer::subClickedPose, this);
  sub_global_goal_    = n.subscribe("uav_server/global_goal", 1, &UAVServer::subGlobalGoal, this);
  sub_local_goal_     = n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_mavros_state_   = n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_       = n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  sub_octomap_        = n.subscribe("/octomap_binary", 1, &UAVServer::subOctomap, this);
  // Establish service clients
  cli_arm_ =      n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_ = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
  // TF
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  // Random nr. generator and distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);
  // Set variables
  double uav_camera_vfov_ = (uav_camera_height_/uav_camera_width_)*uav_camera_hfov_;
  tf2::Matrix3x3 ray_rot_mat;
  tf2::Vector3 unit_ray_direction(1.0, 0.0, 0.0);
  for (double img_y = -uav_camera_height_/2.0; img_y <= uav_camera_height_/2.0+uav_camera_ray_resolution_/2.0; img_y+=uav_camera_ray_resolution_){
    for (double img_x = -uav_camera_width_/2.0; img_x <= uav_camera_width_/2.0+uav_camera_ray_resolution_/2.0; img_x+=uav_camera_ray_resolution_){
      double ray_yaw = (img_x/uav_camera_width_)*uav_camera_hfov_;
      double ray_pitch = (img_y/uav_camera_height_)*uav_camera_vfov_;
      ray_rot_mat.setEulerYPR(ray_yaw, ray_pitch, 0.0);
      tf2::Vector3 ray_direction = ray_rot_mat*unit_ray_direction;
      uav_camera_rays_.push_back(ray_direction);
      ROS_DEBUG("UAVServer: YPR: (%f, %f, %f)", ray_yaw, ray_pitch, 0.0);
      ROS_DEBUG("UAVServer: Ray: (%f, %f, %f)", ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
      if (abs(img_y) == uav_camera_height_/2.0 and abs(img_x) == uav_camera_width_/2.0) {
        uav_camera_corner_rays_.push_back(ray_direction);
      }
    }
  }
  ROS_DEBUG("UAVServer: Camera rays has %d rays", (int)uav_camera_rays_.size());

  // Init.
  takeoff();

  // Wait for map
  ros::Rate rate_wait_map(1.0);
  while(!received_map_)
  {
    ROS_WARN("UAVServer: No map received yet, waiting...");
    ros::spinOnce();
    rate_wait_map.sleep();
  }
}

// Destructor
  
UAVServer::~UAVServer()
{
  ROS_INFO("UAVServer object is being deleted.");


}

/* 
*  Publish functions for publishers
*/

void UAVServer::pubMavrosSetpoint()
{
  ROS_DEBUG("UAVServer: pubMavrosSetpoint");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }
  /* Transform local goal back to uav local origin/odom:
  *  MAVROS takes setpoints/commands in it's own local frame (as
  *  if it started at origin). Since local goal has been trans-
  *  formed into world frame, we must transform it back before
  *  sending it as a setpoint.
  */ 
  try{
    geometry_msgs::PoseStamped mavros_setpoint;
    //tf_buffer_.transform(uav_local_goal_, mavros_setpoint, uav_local_frame_);
    geometry_msgs::TransformStamped mavros_setpoint_tf = tf_buffer_.lookupTransform(uav_local_frame_, uav_local_goal_.header.frame_id, ros::Time(0));
    tf2::doTransform(uav_local_goal_, mavros_setpoint, mavros_setpoint_tf);
    
    if(uav_global_goal_euc_dist_ < 1.0){
      mavros_setpoint.pose.orientation = uav_global_goal_.pose.orientation;
    }
    pub_mavros_setpoint_.publish(mavros_setpoint);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: pubMavrosSetpoint: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/* 
*  Callback functions for subscriptions
*/

void UAVServer::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_INFO("UAVServer: subClickedPoint");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }

  if (uav_running_exploration_) { 
    ROS_INFO("UAVServer: Stopping exploration.");
    uav_running_exploration_ = false;
  }
  else { 
    ROS_INFO("UAVServer: Starting exploration.");
    uav_running_exploration_ = true;
      runExploration();
      pub_global_goal_.publish(uav_global_goal_);
  }

  ros::Rate wait_rate(20.0);
  while (uav_running_exploration_) {
    if (isGoalReached()) {
      runExploration();
      pub_global_goal_.publish(uav_global_goal_);
    }

    ros::spinOnce();
    wait_rate.sleep();
  }
}

void UAVServer::subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg) {
  ROS_INFO("UAVServer: subClickedPose");

  geometry_msgs::Point clicked_pose;
  clicked_pose.x = clicked_pose_msg->pose.position.x;
  clicked_pose.y = clicked_pose_msg->pose.position.y;
  clicked_pose.z = uav_takeoff_z_;

  geometry_msgs::Vector3 rpy_vector;
  tf2::Quaternion tf_quat;
  tf2::fromMsg(clicked_pose_msg->pose.orientation, tf_quat);
  tf2::Matrix3x3(tf_quat).getRPY(rpy_vector.x, 
                                 rpy_vector.y, 
                                 rpy_vector.z);
  
  visualizeFOV(clicked_pose, rpy_vector);
  calculateInformationGain(clicked_pose, rpy_vector);

}

void UAVServer::subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& global_goal_msg) { 
  ROS_DEBUG("UAVServer: subGlobalGoal");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }
   
  try{
    //tf_buffer_.transform(*global_goal_msg, uav_global_goal_, uav_world_frame_);
    geometry_msgs::TransformStamped global_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, global_goal_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*global_goal_msg, uav_global_goal_, global_goal_tf);

    double roll, pitch, yaw;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_global_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_global_goal_.pose.position.x, 
              uav_global_goal_.pose.position.y,
              uav_global_goal_.pose.position.z,
              yaw);  
    uav_global_goal_yaw_ = yaw;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg) { 
  ROS_DEBUG("UAVServer: subLocalGoal");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }
  
  try{
    //tf_buffer_.transform(*local_goal_msg, uav_local_goal_, uav_world_frame_, ros::Time(0), uav_world_frame_);
    geometry_msgs::TransformStamped local_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, local_goal_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*local_goal_msg, uav_local_goal_, local_goal_tf);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subLocalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg) { 
  ROS_DEBUG("UAVServer: subMavrosState");
  uav_state_ = *mavros_state_msg;
}

void UAVServer::subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg) { 
  ROS_DEBUG("UAVServer: subOdometry");
  // Only need pose information (twist/velocities not needed)
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = odometry_msg->header;
  pose_msg.pose = odometry_msg->pose.pose;
  try{
    // Transform pose information contained in odom message to world frame and store
    //tf_buffer_.transform(pose_msg, uav_pose_, uav_world_frame_);
    geometry_msgs::TransformStamped odometry_tf = tf_buffer_.lookupTransform(uav_world_frame_, odometry_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(pose_msg, uav_pose_, odometry_tf);

    // Store orientation in pose also as roll, pitch and yaw
    uav_rpy_.header = pose_msg.header;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(pose_msg.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_rpy_.vector.x, 
                                   uav_rpy_.vector.y, 
                                   uav_rpy_.vector.z);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subOdometry: %s",ex.what());
    ros::Duration(1.0).sleep();
  }

  visualizeUAVFOV();
}

void UAVServer::subOctomap(const octomap_msgs::Octomap& octomap_msg) {
  ROS_DEBUG("UAVServer: subOctomap");
  octomap::AbstractOcTree* abstract_map = octomap_msgs::binaryMsgToMap(octomap_msg);
  if(abstract_map)
  {
    map_ = dynamic_cast<octomap::OcTree*>(abstract_map);
    received_map_ = true;
  } 
  else 
  {
    ROS_ERROR("UAVServer: Error creating octree from received message");
  }
}

// Utility functions

void UAVServer::getParams(ros::NodeHandle np) {
  ROS_DEBUG("UAVServer: getParams");
  np.param<int>("uav_id", uav_id_, 0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav"+std::to_string(uav_id_));
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav"+std::to_string(uav_id_));
  np.param<double>("uav_start_x", uav_start_x_, 0.0);
  np.param<double>("uav_start_y", uav_start_y_, 0.0);
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<double>("uav_camera_width", uav_camera_width_, 6.0);
  np.param<double>("uav_camera_height", uav_camera_height_, 3.0);
  np.param<double>("uav_camera_hfov", uav_camera_hfov_, 1.02974);
  np.param<double>("uav_camera_ray_resolution", uav_camera_ray_resolution_, 1.0);
  np.param<double>("uav_camera_range", uav_camera_range_, 7.5);
  np.param<double>("planner_rate", planner_rate_, 100.0);
  np.param<double>("planner_max_time", planner_max_time_, 1.0);
  np.param<double>("planner_sample_radius", planner_sample_radius_, 10.0);
  np.param<double>("planner_max_ray_distance", planner_max_ray_distance_, 3.0);
  np.param<double>("planner_max_neighbor_distance", planner_max_neighbor_distance_, planner_max_ray_distance_);
  np.param<double>("planner_max_neighbor_yaw", planner_max_neighbor_yaw_, angles::from_degrees(90.0));
  np.param<bool>("planner_unmapped_is_collision", planner_unmapped_is_collision_, true);
}

void UAVServer::takeoff() {
  ROS_DEBUG("UAVServer: takeoff");

  ros::Rate rate(20.0);

  // Wait for FCU connection
  while(ros::ok() && !uav_state_.connected){
    ros::spinOnce();
    rate.sleep();
  }
  ROS_DEBUG("UAVServer: FCU connected");

  // Send a few setpoints before starting procedure
  for(int i = 50; ros::ok() && i > 0; --i){
    pub_mavros_setpoint_.publish(uav_local_goal_);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_DEBUG("UAVServer: Initial setpoints sent");

  // Keep track of when last request/command was issued for timing purposes
  ros::Time last_request = ros::Time::now();
  float wait_duration = 2.0; // Give each step about 2 seconds

  // Mavros request/command procedure to get drone to take off
  uav_takeoff_complete_ = false;
  uav_clearing_rotation_complete_ = false;
  while(!uav_takeoff_complete_)
  {
    if( uav_state_.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(wait_duration)))
    {
      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";
      if( cli_set_mode_.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent)
      {
        ROS_INFO("UAVServer: Offboard enabled");
      }
      last_request = ros::Time::now();
    } 
    else 
    {
      if( !uav_state_.armed &&
          (ros::Time::now() - last_request > ros::Duration(wait_duration)))
      {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if( cli_arm_.call(arm_cmd) &&
            arm_cmd.response.success)
        {
          ROS_INFO("UAVServer: Vehicle armed");
        }
        last_request = ros::Time::now();
        uav_takeoff_complete_ = true;
      }
    }
    pub_mavros_setpoint_.publish(uav_local_goal_);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("UAVServer: Vehicle takeoff procedure complete");

  ROS_INFO("UAVServer: Performing clearing rotation");
  uav_local_goal_.pose.position.x = 0.0;
  uav_local_goal_.pose.position.y = 0.0;
  uav_local_goal_.pose.position.z = uav_takeoff_z_;
  double clearing_rotation_angle = 0.0;
  while (clearing_rotation_angle < 360.0) {
    double clearing_rotation = angles::from_degrees(clearing_rotation_angle);
    double angle_threshold = angles::from_degrees(10.0);
    uav_local_goal_.pose.orientation = makeQuatFromRPY(0.0, 0.0, clearing_rotation);
    while (angles::shortest_angular_distance(uav_rpy_.vector.z, clearing_rotation) > angle_threshold) {
      pub_mavros_setpoint_.publish(uav_local_goal_);
      ros::spinOnce();
      rate.sleep();
    }
    clearing_rotation_angle += 30;
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("UAVServer: Clearing rotations complete");
  uav_clearing_rotation_complete_ = true;
}

/* 
*  Exploration functions
*/

void UAVServer::runExploration() {
  ROS_DEBUG("UAVServer: planPathToGoal");
  // If map not initialized or in use by others, wait
  ros::Rate wait_rate(5.0);
  if (!received_map_) {
    ros::spinOnce();
    wait_rate.sleep();
  }
  tree_.clear();
  exploration_nodes_.clear();

  double start_x = uav_pose_.pose.position.x;
  double start_y = uav_pose_.pose.position.y;
  double start_z = uav_pose_.pose.position.z;

  bool root_from_global_goal = true;
  if (root_from_global_goal) {
    start_x = uav_global_goal_.pose.position.x;
    start_y = uav_global_goal_.pose.position.y;
    start_z = uav_global_goal_.pose.position.z;
  }
  const double start_yaw = tf2::getYaw(uav_pose_.pose.orientation);
  root_ = Node(start_x, start_y, start_z, start_yaw, 0.0, 0.0, 0, nullptr, false);
  tree_.push_back(root_);
  ROS_DEBUG("UAVServer: Root node added. ID: %d, Rank: %d, Cost: %f", root_.id_, root_.rank_, root_.cost_);
  ROS_DEBUG("UAVServer: Gain indiv: %f, Gain combined: %f", root_.gain_indiv_, root_.gain_);

  // Grow exploration tree
  ros::Rate rate(planner_rate_);
  ros::Time start_time = ros::Time::now();
  while((ros::Time::now() - start_time).toSec() < planner_max_time_){
    ros::Time begin = ros::Time::now();
    
    geometry_msgs::Point sample_point;
    double sample_yaw;
    if (path_.size() > 0) {
      sample_point = path_.begin()->pose.position;
      geometry_msgs::Vector3 sample_rpy = makeRPYFromQuat(path_.begin()->pose.orientation);
      sample_yaw = sample_rpy.z;
      path_.pop_front();
      ROS_DEBUG("UAVServer: Sampled previous path point: (x,y,z,y) = (%.1f,%.1f,%.1f,%.1f)",sample_point.x,sample_point.y,sample_point.z,angles::to_degrees(sample_yaw));
    }
    else {
      sample_point = generateRandomPoint();
      sample_point.x += uav_pose_.pose.position.x;
      sample_point.y += uav_pose_.pose.position.y;
      sample_yaw = M_PI - 2*M_PI*unit_distribution_(generator_);
      ROS_DEBUG("UAVServer: Sampled random point: (x,y,z,y) = (%.1f,%.1f,%.1f,%.1f)",sample_point.x,sample_point.y,sample_point.z,angles::to_degrees(sample_yaw));
    }

    // Start finding nearest node in the tree to the sampled point (init. as root)
    Node* nearest_neighbor = &root_;
    double nearest_neighbor_euc_distance = getDistanceBetweenPoints(sample_point, root_.position_);

    // Iterate through tree nodes, checking distance to sampled point
    double min_allowed_node_distance = 0.1;
    bool sample_is_too_close = false;
    for(std::list<Node>::iterator tree_node_itr = tree_.begin();
        tree_node_itr != tree_.end(); tree_node_itr++) {
      // Get distance to tree node
      double euc_distance_to_node = getDistanceBetweenPoints(tree_node_itr->position_, sample_point);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_itr->id_, euc_distance_to_node);

      if (euc_distance_to_node < min_allowed_node_distance)
      {
        ROS_DEBUG("Too close.");
        sample_is_too_close = true;
        break;
      }

      // Make tree node new nearest neighbor if it is closer than current nearest neighbor
      if (euc_distance_to_node < nearest_neighbor_euc_distance) 
      {
        ROS_DEBUG("Sampled node is closer to node %d: %f", tree_node_itr->id_, euc_distance_to_node);
        nearest_neighbor = &*tree_node_itr;
        nearest_neighbor_euc_distance = euc_distance_to_node;
      }
    }

    if (!sample_is_too_close) {
      /* Get the coordinates of the new point
      * Cast a ray from the nearest neighbor in the direction of the sampled point.
      * If nearest neighbor is within planner_max_ray_distance_ of sampled point, sampled point is new point.
      * Else the endpoint of the ray (with length planner_max_ray_distance_) is new point
      */
      geometry_msgs::Point new_point_ray_origin = nearest_neighbor->position_;
      geometry_msgs::Vector3 new_point_ray_direction = getDirection(new_point_ray_origin, sample_point);
      geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                                new_point_ray_direction, 
                                                std::min(nearest_neighbor_euc_distance, planner_max_ray_distance_));
      // Get as close to the sample yaw, without breaking the limit on neighbor yaw
      double new_yaw = getClosestYaw(nearest_neighbor->yaw_, sample_yaw, planner_max_neighbor_yaw_);


      // RRTstar algorithm
      extendTreeRRTstar(new_point, new_yaw);
    }

    visualizeTree();

    ros::spinOnce();
    rate.sleep();
  }

  std::map<double, Node>::iterator exploration_nodes_itr;
  ROS_DEBUG("\tNODE\tGAIN\tGAIN_IND"); 
  for (exploration_nodes_itr = exploration_nodes_.begin(); exploration_nodes_itr != exploration_nodes_.end(); ++exploration_nodes_itr) { 
    ROS_DEBUG("\t%d\t%f\t%f", exploration_nodes_itr->second.id_, exploration_nodes_itr->first, exploration_nodes_itr->second.gain_indiv_);
  }

  path_.clear();
  Node node_on_path = exploration_nodes_.rbegin()->second;
  while (node_on_path.getParent() != nullptr) {
    path_.push_front(makePoseStampedFromNode(node_on_path));
    ROS_DEBUG("Node: %d", (int)node_on_path.id_);

    if (node_on_path.getParent()->id_ == 0) {
      uav_global_goal_.header.frame_id = uav_world_frame_;
      uav_global_goal_.header.stamp = ros::Time::now();
      uav_global_goal_.pose.position = node_on_path.position_;
      uav_global_goal_.pose.orientation = makeQuatFromRPY(0.0, 0.0, node_on_path.yaw_);
      break;
    }
    node_on_path = *(node_on_path.getParent());
  }
  ROS_INFO("Done");
  visualizePath();
  visualizePathFOVs(1.0);
}

double UAVServer::calculateInformationGain(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy) {
  ROS_DEBUG("UAVServer: calculateInformationGain");
  uav_camera_information_points_.clear();

  double ray_distance = uav_camera_range_;
  tf2::Matrix3x3 ray_direction_rotmat;
  ray_direction_rotmat.setEulerYPR(rpy.z, rpy.y, rpy.x);

  octomap::point3d om_ray_origin = octomap::point3d(origin.x, origin.y, origin.z);
  int hits = 0;
  int occupied = 0;
  int free = 0;
  int unmapped = 0;
  double occupancy_threshold = map_->getOccupancyThres();

  for(tf2::Vector3 ray : uav_camera_rays_) {
    tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_distance;
    octomap::point3d om_ray_direction = octomap::point3d(ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
    octomap::point3d om_ray_end_cell;
    bool hit_occupied = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, ray_distance);
    if (hit_occupied) {
      hits++;
    }
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = om_ray_end_cell.x();
    ray_end_point.y = om_ray_end_cell.y();
    ray_end_point.z = om_ray_end_cell.z();
    
    octomap::OcTreeNode* om_ray_end_node = map_->search(om_ray_end_cell);
    if (om_ray_end_node != NULL) {
      double node_occupancy = om_ray_end_node->getOccupancy();
      ROS_DEBUG("Node value: %f", node_occupancy);
      uav_camera_information_points_.push_back(std::pair<double, geometry_msgs::Point>(node_occupancy, ray_end_point));
      if (node_occupancy < occupancy_threshold) {
        free++;
      }
      else {
        occupied++;
      }
    }
    else {
      //uav_camera_information_points_.push_back(std::pair<double, geometry_msgs::Point>(0.5, ray_end_point));
      unmapped++;
    }
    
  }
  
  visualizeInformationPoints();
  ROS_DEBUG("Occupied: %d", occupied);
  ROS_DEBUG("Free: %d", free);
  ROS_DEBUG("Unmapped: %d", unmapped);
  return unmapped;
}

double UAVServer::calculateInformationGain(geometry_msgs::Point origin, double yaw) {
  ROS_DEBUG("UAVServer: calculateInformationGain");
  uav_camera_information_points_.clear();

  double ray_distance = uav_camera_range_;
  tf2::Matrix3x3 ray_direction_rotmat;
  ray_direction_rotmat.setEulerYPR(yaw, 0.0, 0.0);

  octomap::point3d om_ray_origin = octomap::point3d(origin.x, origin.y, origin.z);
  int occupied = 0;
  int free = 0;
  int unmapped = 0;
  double occupancy_threshold = map_->getOccupancyThres();

  for(tf2::Vector3 ray : uav_camera_rays_) {
    tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_distance;
    octomap::point3d om_ray_direction = octomap::point3d(ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
    octomap::point3d om_ray_end_cell;
    bool hit_occupied = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, ray_distance);
    geometry_msgs::Point ray_end_point;
    ray_end_point.x = om_ray_end_cell.x();
    ray_end_point.y = om_ray_end_cell.y();
    ray_end_point.z = om_ray_end_cell.z();
    
    octomap::OcTreeNode* om_ray_end_node = map_->search(om_ray_end_cell);
    if (om_ray_end_node != NULL) {
      double node_occupancy = om_ray_end_node->getOccupancy();
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

  return unmapped;
}

geometry_msgs::Point UAVServer::generateRandomPoint()
{
  ROS_DEBUG("UAVServer: generateRandomPoint");

  geometry_msgs::Point sample_point;

  // Sample from unit circle, uniformly https://stackoverflow.com/a/50746409
  double radius = planner_sample_radius_*sqrt(unit_distribution_(generator_));
  double theta = unit_distribution_(generator_)*2.0*M_PI;
  sample_point.x = radius*cos(theta);
  sample_point.y = radius*sin(theta);
  sample_point.z = uav_takeoff_z_;
  ROS_DEBUG("UAVServer: Unit circle point (x,y) = (%f,%f)",sample_point.x,sample_point.y);

  return sample_point;
}

void UAVServer::extendTreeRRTstar(geometry_msgs::Point candidate_point, double candidate_yaw) {
  ROS_DEBUG("UAVServer: extendTreeRRTstar");

  // Make a neighbor list of nodes close enough to the candidate point
  // Store combined cost and Node pointer in map
  std::map<double, Node*> neighbor_list;
  for(std::list<Node>::iterator tree_node_itr = tree_.begin();
      tree_node_itr != tree_.end(); tree_node_itr++)
  {
    // Get distance to tree node
    double distance_to_node = getDistanceBetweenPoints(candidate_point, tree_node_itr->position_);
    double angle_to_node = abs(angles::shortest_angular_distance(candidate_yaw, tree_node_itr->yaw_));
    ROS_DEBUG("Sampled point distance to node %d: %f", tree_node_itr->id_, distance_to_node);

    // Add tree node to neighborhood if it is within range
    // Inserted into map with combined cost (tree node cost + distance to node) as key
    if(distance_to_node <= planner_max_neighbor_distance_+0.01 and angle_to_node <= planner_max_neighbor_yaw_+0.01) {
      ROS_DEBUG("Node %d is within neighborhood", tree_node_itr->id_);
      double neighbor_cost = tree_node_itr->cost_;
      double combined_cost = neighbor_cost + distance_to_node;
      neighbor_list.insert(std::pair<double, Node*>(combined_cost, &*tree_node_itr));
    }
  }
  ROS_DEBUG("Neighborhood size: %d", (int)neighbor_list.size());
  
  // Function to print neighbors
  std::map<double, Node*>::iterator neighbor_itr;
  ROS_DEBUG("\tNODE\tCOST"); 
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) { 
    ROS_DEBUG("\t%d\t%f", neighbor_itr->second->id_, neighbor_itr->first);
  }

  // Add the new node if it is collision free
  std::vector<geometry_msgs::Point> collision_tree;
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) 
  { 
    bool pathIsCollisionFree = isPathCollisionFree(neighbor_itr->second->position_, candidate_point);
    int node_rank = neighbor_itr->second->rank_ + 1;
    if(pathIsCollisionFree and node_rank < 4)
    {
      int node_id = tree_.size();
      double node_cost = neighbor_itr->second->cost_ + getDistanceBetweenPoints(candidate_point, neighbor_itr->second->position_);
      double node_gain_indiv = calculateInformationGain(candidate_point, candidate_yaw)/(double)node_rank;
      double node_gain = neighbor_itr->second->gain_ + node_gain_indiv;
      Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, candidate_yaw, node_cost, node_gain, node_id, neighbor_itr->second, node_rank, false);
      new_node.gain_indiv_ = node_gain_indiv;

      tree_.push_back(new_node);
      ROS_DEBUG("UAVServer: New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      ROS_DEBUG("UAVServer: Gain indiv: %f, Gain combined: %f", node_gain_indiv, node_gain);

      exploration_nodes_.insert(std::pair<double, Node>(node_gain+node_cost, new_node));
      
      break;
    }
    else
    {
      ROS_DEBUG("I hit something");
    }
  }
}

bool UAVServer::isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point point_b) {
  ROS_DEBUG("UAVServer: isPathCollisionFree");

  // Check first if end point is an occupied octomap node
  double occupancy_threshold = map_->getOccupancyThres();
  octomap::OcTreeNode* om_point_b_node = map_->search(point_b.x, point_b.y, point_b.z);
  if (om_point_b_node != NULL) {
    double node_occupancy = om_point_b_node->getOccupancy();
    ROS_DEBUG("Node value: %f", node_occupancy);
    if (node_occupancy > occupancy_threshold) {
      // End node is occupied
      return false;
    }
  }
  else {
    // End node is unmapped
    if (planner_unmapped_is_collision_) {
      return false;
    }
  }

  // Attempt to cast OctoMap ray
  geometry_msgs::Vector3 direction_ab = getDirection(point_a, point_b);
  double distance = getDistanceBetweenPoints(point_a, point_b);

  octomap::point3d om_ray_origin = octomap::point3d(point_a.x, point_a.y, point_a.z);
  octomap::point3d om_ray_direction = octomap::point3d(direction_ab.x, direction_ab.y, direction_ab.z);
  octomap::point3d om_ray_end = octomap::point3d(point_b.x, point_b.y, point_b.z);
  octomap::point3d om_ray_return_direction = octomap::point3d(-direction_ab.x, -direction_ab.y, -direction_ab.z);
  octomap::point3d om_ray_end_cell;
  octomap::point3d om_ray_return_cell;

  // Check if direction is valid
  if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0) {
    ROS_WARN("UAVServer: Got request to cast ray in illegal direction.");
    ROS_WARN("            Origin (x,y,z) = (%f,%f,%f)",point_a.x,point_a.y,point_a.z);
    ROS_WARN("            End    (x,y,z) = (%f,%f,%f)",point_b.x,point_b.y,point_b.z);

    // Count illegal ray cast as collision
    return false;
  }
  else {
    double om_ray_distance = std::min(distance, planner_max_ray_distance_);
    bool hit_occupied_to = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, om_ray_distance);
    bool hit_occupied_from = map_->castRay(om_ray_end, om_ray_return_direction, om_ray_return_cell, false, om_ray_distance);
    if (hit_occupied_to or hit_occupied_from) {
      return false;
    }
  }

  tf2::Matrix3x3 ray_rot_mat;
  tf2::Vector3 unit_ray_direction(1.0, 0.0, 0.0);

  double uav_radius = 0.75;
  int degrees_to_check = 360;
  while (degrees_to_check > 0) {
    double ray_yaw = angles::from_degrees(degrees_to_check);
    ray_rot_mat.setEulerYPR(ray_yaw, 0.0, 0.0);
    tf2::Vector3 ray_direction = ray_rot_mat*unit_ray_direction;
    om_ray_direction = octomap::point3d(ray_direction.x(), ray_direction.y(), ray_direction.z());
    bool hit_occupied = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, uav_radius);
    if (hit_occupied) {
      return false;
    }
    degrees_to_check -= 45;
  }

  // Return whether there was a collision or not
  return true;
}

bool UAVServer::isGoalReached() {

  // Calculate distance to global goal in 2D
  uav_global_goal_euc_dist_ = sqrt( pow(uav_global_goal_.pose.position.x - uav_pose_.pose.position.x, 2.0 ) +
                                    pow(uav_global_goal_.pose.position.y - uav_pose_.pose.position.y, 2.0 ) );

  // Calculate yaw distance to global goal yaw
  uav_global_goal_yaw_dist_ = abs(angles::shortest_angular_distance(uav_rpy_.vector.z, uav_global_goal_yaw_));

  ROS_DEBUG("Goal dist (euc, yaw) = (%.2f, %.2f)", uav_global_goal_euc_dist_, uav_global_goal_yaw_dist_);
  return (uav_global_goal_euc_dist_ < 0.5) and (uav_global_goal_yaw_dist_ < 0.2);
}

/* 
*  Utility functions
*/

geometry_msgs::PoseStamped UAVServer::makePoseStampedFromNode(Node node) {
  ROS_DEBUG("UAVServer: makePoseStampedFromNode");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = uav_world_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position = node.position_;
  tf2::Quaternion pose_quat;
  pose_quat.setRPY(0, 0, node.yaw_);
  pose.pose.orientation.x = pose_quat.x();
  pose.pose.orientation.y = pose_quat.y();
  pose.pose.orientation.z = pose_quat.z();
  pose.pose.orientation.w = pose_quat.w();
  return pose;
}

geometry_msgs::Quaternion UAVServer::makeQuatFromRPY(geometry_msgs::Vector3 rpy) {
  ROS_DEBUG("UAVServer: makeQuatFromRPY");
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(rpy.x, rpy.y, rpy.z);
  geometry_msgs::Quaternion quat;
  quat.x = tf_quat.x();
  quat.y = tf_quat.y();
  quat.z = tf_quat.z();
  quat.w = tf_quat.w();
  return quat;
}

geometry_msgs::Quaternion UAVServer::makeQuatFromRPY(double r, double p, double y) {
  ROS_DEBUG("UAVServer: makeQuatFromRPY");
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(r, p, y);
  geometry_msgs::Quaternion quat;
  quat.x = tf_quat.x();
  quat.y = tf_quat.y();
  quat.z = tf_quat.z();
  quat.w = tf_quat.w();
  return quat;
}

geometry_msgs::Vector3 UAVServer::makeRPYFromQuat(geometry_msgs::Quaternion quat) {
  ROS_DEBUG("UAVServer: makeRPYFromQuat");
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  geometry_msgs::Vector3 rpy;
  tf2::Matrix3x3(tf_quat).getRPY(rpy.x, 
                                 rpy.y, 
                                 rpy.z);
  return rpy;
}

/* 
*  Visualization functions
*/

void UAVServer::visualizeUAVFOV() {
  ROS_DEBUG("UAVServer: visualizeFOV");
  visualization_msgs::Marker fov_marker;
  fov_marker.header.frame_id = uav_world_frame_;
  fov_marker.header.stamp = ros::Time::now();
  fov_marker.id = 0;
  fov_marker.type = visualization_msgs::Marker::LINE_LIST;
  fov_marker.action = visualization_msgs::Marker::ADD;
  fov_marker.pose.orientation.w = 1.0;
  fov_marker.scale.x = 0.05;
  fov_marker.color.a = 0.5;
  fov_marker.color.r = 0.2;
  fov_marker.color.g = 0.2;
  fov_marker.color.b = 1.0;
    
  bool show_all_rays = false;
  double ray_length = uav_camera_range_;
  tf2::Matrix3x3 ray_direction_rotmat;
  ray_direction_rotmat.setEulerYPR(uav_rpy_.vector.z, uav_rpy_.vector.y, uav_rpy_.vector.x);
  if (show_all_rays) {
    for(tf2::Vector3 ray : uav_camera_rays_) {
      fov_marker.points.push_back(uav_pose_.pose.position);
      tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_length;
      geometry_msgs::Point ray_endpoint;
      ray_endpoint.x = uav_pose_.pose.position.x + ray_direction.getX();
      ray_endpoint.y = uav_pose_.pose.position.y + ray_direction.getY();
      ray_endpoint.z = uav_pose_.pose.position.z + ray_direction.getZ();
      fov_marker.points.push_back(ray_endpoint);
    }
  }
  else {
    std::vector<geometry_msgs::Point> ray_endpoints;
    for(tf2::Vector3 ray : uav_camera_corner_rays_) {
      fov_marker.points.push_back(uav_pose_.pose.position);
      tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_length;
      geometry_msgs::Point ray_endpoint;
      ray_endpoint.x = uav_pose_.pose.position.x + ray_direction.getX();
      ray_endpoint.y = uav_pose_.pose.position.y + ray_direction.getY();
      ray_endpoint.z = uav_pose_.pose.position.z + ray_direction.getZ();
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
  pub_viz_uav_fov_.publish(fov_marker);
}

void UAVServer::visualizeFOV(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy) {
  ROS_DEBUG("UAVServer: visualizeFOV");
  visualization_msgs::Marker fov_marker;
  fov_marker.header.frame_id = uav_world_frame_;
  fov_marker.header.stamp = ros::Time::now();
  fov_marker.id = 0;
  fov_marker.type = visualization_msgs::Marker::LINE_LIST;
  fov_marker.action = visualization_msgs::Marker::ADD;
  fov_marker.pose.orientation.w = 1.0;
  fov_marker.scale.x = 0.05;
  fov_marker.color.a = 0.5;
  fov_marker.color.r = 0.2;
  fov_marker.color.g = 0.2;
  fov_marker.color.b = 1.0;
  bool show_all_rays = false;
  double ray_length = uav_camera_range_;
  tf2::Matrix3x3 ray_direction_rotmat;
  ray_direction_rotmat.setEulerYPR(rpy.z, rpy.y, rpy.x);
  if (show_all_rays) {
    for(tf2::Vector3 ray : uav_camera_rays_) {
      fov_marker.points.push_back(origin);
      tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_length;
      geometry_msgs::Point ray_endpoint;
      ray_endpoint.x = origin.x + ray_direction.getX();
      ray_endpoint.y = origin.y + ray_direction.getY();
      ray_endpoint.z = origin.z + ray_direction.getZ();
      fov_marker.points.push_back(ray_endpoint);
    }
  }
  else {
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
  pub_viz_fov_.publish(fov_marker);
}

void UAVServer::visualizeInformationPoints() {
  ROS_DEBUG("UAVServer: visualizeInformationPoints");
  visualization_msgs::Marker information_marker;
  information_marker.header.frame_id = uav_world_frame_;
  information_marker.header.stamp = ros::Time::now();
  information_marker.id = 0;
  information_marker.type = visualization_msgs::Marker::POINTS;
  information_marker.action = visualization_msgs::Marker::ADD;
  information_marker.pose.orientation.w = 1.0;
  information_marker.scale.x = 0.25;
  information_marker.scale.y = 0.25;

  for(std::pair<double, geometry_msgs::Point> information_point : uav_camera_information_points_) {
    std_msgs::ColorRGBA point_color;
    point_color.a = 1.0;
    point_color.r = information_point.first;
    point_color.g = 1.0 - information_point.first;
    point_color.b = 0.0;
    information_marker.colors.push_back(point_color);
    information_marker.points.push_back(information_point.second);
  }
  pub_viz_information_points_.publish(information_marker);
}

void UAVServer::visualizeTree() {
  ROS_DEBUG("UAVServer: visualizeTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = uav_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;
  tree_marker.color.a = 0.5;
  tree_marker.color.r = 0.2;
  tree_marker.color.g = 1.0;
  tree_marker.color.b = 0.2;
  for(Node tree_node : tree_)
  {
    if(tree_node.getParent() != NULL) 
    {
      tree_marker.points.push_back(tree_node.position_);
      tree_marker.points.push_back(tree_node.getParent()->position_);
    }
  }
  pub_viz_tree_.publish(tree_marker);
}

void UAVServer::visualizePath() {
  ROS_DEBUG("UAVServer: visualizePath");
  if(path_.empty()){
    return;
  }
  
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = uav_world_frame_;
  path_marker.header.stamp = ros::Time::now();
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_LIST;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;
  path_marker.color.a = 1.0;
  path_marker.color.r = 0.1;
  path_marker.color.g = 1.0;
  path_marker.color.b = 0.1;

  path_marker.points.push_back(root_.position_);
  for (auto const& path_pose : path_) {
    path_marker.points.push_back(path_pose.pose.position);
    path_marker.points.push_back(path_pose.pose.position);
  }
  path_marker.points.pop_back();
  
  pub_viz_path_.publish(path_marker);
}

void UAVServer::visualizePathFOVs(double ray_length) {
  ROS_DEBUG("UAVServer: visualizePathFOVs");

  visualization_msgs::Marker fov_marker;
  fov_marker.header.frame_id = uav_world_frame_;
  fov_marker.header.stamp = ros::Time::now();
  fov_marker.id = 0;
  fov_marker.type = visualization_msgs::Marker::LINE_LIST;
  fov_marker.action = visualization_msgs::Marker::ADD;
  fov_marker.pose.orientation.w = 1.0;
  fov_marker.scale.x = 0.05;
  fov_marker.color.a = 0.5;
  fov_marker.color.r = 0.2;
  fov_marker.color.g = 0.2;
  fov_marker.color.b = 1.0;
  bool show_all_rays = false;
  tf2::Matrix3x3 ray_direction_rotmat;
  for (auto const& path_pose : path_) {
    geometry_msgs::Vector3 rpy_vector = makeRPYFromQuat(path_pose.pose.orientation);
    geometry_msgs::Point origin = path_pose.pose.position;
    ray_direction_rotmat.setEulerYPR(rpy_vector.z, rpy_vector.y, rpy_vector.x);
    if (show_all_rays) {
      for(tf2::Vector3 ray : uav_camera_rays_) {
        fov_marker.points.push_back(origin);
        tf2::Vector3 ray_direction = ray_direction_rotmat*ray*ray_length;
        geometry_msgs::Point ray_endpoint;
        ray_endpoint.x = origin.x + ray_direction.getX();
        ray_endpoint.y = origin.y + ray_direction.getY();
        ray_endpoint.z = origin.z + ray_direction.getZ();
        fov_marker.points.push_back(ray_endpoint);
      }
    }
    else {
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
  }
  pub_viz_fov_.publish(fov_marker);
}