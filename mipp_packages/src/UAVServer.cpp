#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("UAVServer object is being created.");
  
  //Initialize values
  getParams(np);
  //Establish publish timers and publishers
  pub_timer_mavros_setpoint_ =  n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_ =        n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  //Establish subscriptions
  sub_global_goal_ =    n.subscribe("uav_server/global_goal", 1, &UAVServer::subGlobalGoal, this);
  sub_local_goal_ =     n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_mavros_state_ =   n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_ =       n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  //Establish service clients
  cli_arm_ =      n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_ = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
  // Mavros related init. procedure:
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
  if(uav_takeoff_complete_)
  {
    pub_mavros_setpoint_.publish(uav_local_goal_);
  }
}

/* 
*  Callback functions for subscriptions
*/

void UAVServer::subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg)
{ 
  ROS_DEBUG("UAVServer: subGlobalGoal");
}

void UAVServer::subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg)
{ 
  ROS_DEBUG("UAVServer: subLocalGoal");
  uav_local_goal_ = *local_goal_msg;
}

void UAVServer::subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg)
{ 
  ROS_DEBUG("UAVServer: subMavrosState");
  uav_state_ = *mavros_state_msg;
}

void UAVServer::subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{ 
  ROS_DEBUG("UAVServer: subOdometry");
  uav_odometry_ = *odometry_msg;
}

// Utility functions

void UAVServer::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("UAVServer: getParams");
  np.param<int>("uav_id", uav_id_, 0);
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
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
  geometry_msgs::PoseStamped uav_takeoff_pose;
  uav_takeoff_pose.pose.position.z = uav_takeoff_z_;
  for(int i = 50; ros::ok() && i > 0; --i){
    pub_mavros_setpoint_.publish(uav_takeoff_pose);
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
    pub_mavros_setpoint_.publish(uav_takeoff_pose);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("UAVServer: Vehicle takeoff procedure complete");
}