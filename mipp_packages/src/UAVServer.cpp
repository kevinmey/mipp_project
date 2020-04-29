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
  // Establish subscriptions
  sub_global_goal_ =    n.subscribe("uav_server/global_goal", 1, &UAVServer::subGlobalGoal, this);
  sub_local_goal_ =     n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_mavros_state_ =   n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_ =       n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  // Establish service clients
  cli_arm_ =      n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_ = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
  // TF
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
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
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/* 
*  Callback functions for subscriptions
*/

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
}

// Utility functions

void UAVServer::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("UAVServer: getParams");
  np.param<int>("uav_id", uav_id_, 0);
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav"+std::to_string(uav_id_));
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav"+std::to_string(uav_id_));
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