#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("UAVServer object is being created.");

  // Initialize values
  getParams(np);
  uav_local_goal_.pose.position.z = uav_takeoff_z_;
  uav_local_goal_.pose.orientation.w = 1.0;
  uav_global_goal_ = uav_local_goal_;
  // Establish publish timers and publishers
  pub_timer_mavros_setpoint_ =  n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_ =        n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  pub_viz_fov_ =                n.advertise<visualization_msgs::Marker>("uav_server/viz_fov", 1);
  // Establish subscriptions
  sub_clicked_pose_ =   n.subscribe("/move_base_simple/goal", 1, &UAVServer::subClickedPose, this);
  sub_global_goal_ =    n.subscribe("uav_server/global_goal", 1, &UAVServer::subGlobalGoal, this);
  sub_local_goal_ =     n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_mavros_state_ =   n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_ =       n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  // Establish service clients
  cli_arm_ =      n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_ = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
  // TF
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
  // Set variables
  double uav_camera_vfov_ = (uav_camera_height_/uav_camera_width_)*uav_camera_hfov_;
  tf2::Matrix3x3 ray_rot_mat;
  tf2::Vector3 unit_ray_direction(1.0, 0.0, 0.0);
  for (double img_y = -uav_camera_height_/2.0; img_y <= uav_camera_height_/2.0; img_y+=1.0){
    for (double img_x = -uav_camera_width_/2.0; img_x <= uav_camera_width_/2.0; img_x+=1.0){
      double ray_yaw = (img_x/uav_camera_width_)*uav_camera_hfov_;
      double ray_pitch = (img_y/uav_camera_height_)*uav_camera_vfov_;
      ray_rot_mat.setEulerYPR(ray_yaw, ray_pitch, 0.0);
      tf2::Vector3 ray_direction = ray_rot_mat*unit_ray_direction;
      uav_camera_rays_.push_back(ray_direction);
      ROS_DEBUG("YPR: (%f, %f, %f)", ray_yaw, ray_pitch, 0.0);
      ROS_DEBUG("Ray: (%f, %f, %f)", ray_direction.getX(), ray_direction.getY(), ray_direction.getZ());
      if (abs(img_y) == uav_camera_height_/2.0 and abs(img_x) == uav_camera_width_/2.0) {
        uav_camera_corner_rays_.push_back(ray_direction);
      }
    }
  }
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
  if(!uav_takeoff_complete_)
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
    tf_buffer_.transform(uav_local_goal_, mavros_setpoint, uav_local_frame_);
    if(uav_global_goal_dist_ < 1.0){
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

void UAVServer::subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg)
{
  ROS_DEBUG("UAVServer: subClickedPose");
}

void UAVServer::subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& global_goal_msg)
{ 
  ROS_DEBUG("UAVServer: subGlobalGoal");  
  try{
    tf_buffer_.transform(*global_goal_msg, uav_global_goal_, uav_world_frame_);

    double roll, pitch, yaw;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_global_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_global_goal_.pose.position.x, 
              uav_global_goal_.pose.position.y,
              uav_global_goal_.pose.position.z,
              yaw);  
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void UAVServer::subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg)
{ 
  ROS_DEBUG("UAVServer: subLocalGoal");
  try{
    tf_buffer_.transform(*local_goal_msg, uav_local_goal_, uav_world_frame_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subLocalGoal: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void UAVServer::subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg)
{ 
  ROS_DEBUG("UAVServer: subMavrosState");
  uav_state_ = *mavros_state_msg;
}

void UAVServer::subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{ 
  ROS_DEBUG("UAVServer: subOdometry");
  // Only need pose information (twist/velocities not needed)
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = odometry_msg->header;
  pose_msg.pose = odometry_msg->pose.pose;
  try{
    // Transform pose information contained in odom message to world frame and store
    tf_buffer_.transform(pose_msg, uav_pose_, uav_world_frame_);

    // Store orientation in pose also as roll, pitch and yaw
    uav_rpy_.header = pose_msg.header;
    tf2::Quaternion tf_quat;
    tf2::fromMsg(pose_msg.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_rpy_.vector.x, 
                                   uav_rpy_.vector.y, 
                                   uav_rpy_.vector.z);

    // Calculate distance to global goal in 2D
    uav_global_goal_dist_ = sqrt( pow(uav_global_goal_.pose.position.x - uav_pose_.pose.position.x, 2.0 ) +
                                  pow(uav_global_goal_.pose.position.y - uav_pose_.pose.position.y, 2.0 ) );
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subOdometry: %s",ex.what());
    ros::Duration(1.0).sleep();
  }

  visualizeFOV();
}

// Utility functions

void UAVServer::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("UAVServer: getParams");
  np.param<int>("uav_id", uav_id_, 0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav"+std::to_string(uav_id_));
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav"+std::to_string(uav_id_));
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<double>("uav_camera_width", uav_camera_width_, 6.0);
  np.param<double>("uav_camera_height", uav_camera_height_, 4.0);
  np.param<double>("uav_camera_hfov", uav_camera_hfov_, 1.02974);
  np.param<double>("uav_camera_range", uav_camera_range_, 7.5);
}

void UAVServer::takeoff()
{
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
}

/* 
*  Visualization functions
*/

void UAVServer::visualizeFOV()
{
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
  pub_viz_fov_.publish(fov_marker);
}