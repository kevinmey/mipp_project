#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("UAVServer object is being created.");

  // Initialize values
  getParams(np);
  uav_local_goal_.header.frame_id = uav_local_frame_;
  uav_local_goal_.header.stamp = ros::Time::now();
  uav_local_goal_.pose.position.x = uav_start_x_;
  uav_local_goal_.pose.position.y = uav_start_y_;
  uav_local_goal_.pose.position.z = uav_takeoff_z_;
  uav_local_goal_.pose.orientation.w = 1.0;
  uav_global_goal_ = uav_local_goal_;
  uav_running_exploration_ = false;
  // Establish publish timers and publishers
  pub_timer_mavros_setpoint_  = n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_        = n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  pub_mavros_cmd_vel_         = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  pub_global_goal_            = n.advertise<geometry_msgs::PoseStamped>("uav_server/global_goal", 10);
  pub_viz_uav_fov_            = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav_fov", 1);
  // Establish subscriptions
  sub_clicked_pose_   = n.subscribe("uav_server/clicked_goal", 1, &UAVServer::subClickedPose, this);
  sub_global_goal_    = n.subscribe("uav_server/global_goal", 1, &UAVServer::subGlobalGoal, this);
  sub_local_goal_     = n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_mavros_state_   = n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_       = n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  // Establish service clients
  cli_arm_ =      n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_ = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
  // TF
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  // Random nr. generator and distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Init.
  takeoff();
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

  try{
    geometry_msgs::TransformStamped global_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, clicked_pose_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*clicked_pose_msg, uav_global_goal_, global_goal_tf);
    uav_global_goal_.pose.position.z = uav_takeoff_z_;

    geometry_msgs::Vector3 rpy_vector;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_global_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(rpy_vector.x, 
                                   rpy_vector.y, 
                                   rpy_vector.z);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_global_goal_.pose.position.x, 
              uav_global_goal_.pose.position.y,
              uav_global_goal_.pose.position.z,
              rpy_vector.z);  
    uav_global_goal_yaw_ = rpy_vector.z;
  
    visualizeFOV(uav_global_goal_.pose.position, rpy_vector);
    calculateInformationGain(uav_global_goal_.pose.position, rpy_vector);

    uav_global_goal_.header.frame_id = uav_world_frame_;
    uav_global_goal_.header.stamp = ros::Time::now();
    pub_global_goal_.publish(uav_global_goal_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& global_goal_msg) { 
  ROS_DEBUG("UAVServer: subGlobalGoal");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }
   
  try{
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
    uav_global_goal_.pose.orientation = makeQuatFromRPY(0.0, 0.0, uav_global_goal_yaw_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
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

  ros::Duration(5.0).sleep();

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
  while (clearing_rotation_angle <= 360.0) {
    double clearing_rotation = angles::from_degrees(clearing_rotation_angle);
    double angle_threshold = angles::from_degrees(10.0);
    uav_local_goal_.pose.orientation = makeQuatFromRPY(0.0, 0.0, clearing_rotation);
    while (angles::shortest_angular_distance(uav_rpy_.vector.z, clearing_rotation) > angle_threshold) {
      pub_mavros_setpoint_.publish(uav_local_goal_);
      ros::spinOnce();
      rate.sleep();
    }
    clearing_rotation_angle += 90;
    ros::Duration(0.5).sleep();
  }
  ROS_INFO("UAVServer: Clearing rotations complete");
  uav_clearing_rotation_complete_ = true;
  /*
  geometry_msgs::Twist test_cmd_vel;
  mavros_msgs::PositionTarget test_pos_target;
  while (true) {
    test_cmd_vel.linear.x = 0.5;
    test_cmd_vel.linear.y = 0.0;
    test_cmd_vel.linear.z = 0.0;
    test_cmd_vel.angular.x = 0.0;
    test_cmd_vel.angular.y = 0.0;
    test_cmd_vel.angular.z = 0.5;

    test_pos_target.header.frame_id = uav_body_frame_;
    test_pos_target.header.stamp = ros::Time::now();
    test_pos_target.coordinate_frame = 8; // FRAME_BODY_NED
    test_pos_target.type_mask = 0b011111000111;
    test_pos_target.velocity.x = 0.0;
    test_pos_target.velocity.y = 0.5;
    test_pos_target.velocity.z = 0.0;
    test_pos_target.yaw_rate = 0.0;
    
    pub_mavros_cmd_vel_.publish(test_pos_target);
    ros::Duration(0.05).sleep();
  }*/
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