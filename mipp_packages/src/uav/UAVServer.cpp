#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
  : act_move_vehicle_server_(n, "move_vehicle_action", boost::bind(&UAVServer::actMoveVehicle, this, _1), false) 
{
  ROS_INFO("UAVServer object is being created.");

  // Initialize values
  getParams(np);
  uav_position_goal_.header.frame_id = uav_local_frame_;
  uav_position_goal_.header.stamp = ros::Time::now();
  uav_position_goal_.pose.position.x = uav_start_x_;
  uav_position_goal_.pose.position.y = uav_start_y_;
  uav_position_goal_.pose.position.z = uav_takeoff_z_;
  uav_position_goal_.pose.orientation.w = 1.0;
  // Establish publish timers and publishers
  pub_timer_mavros_setpoint_  = n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_        = n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  pub_mavros_cmd_vel_         = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  pub_viz_uav_fov_            = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav_fov", 1);
  pub_viz_uav_              = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav", 1);
  // Establish subscriptions
  sub_clicked_pose_   = n.subscribe("uav_server/clicked_goal", 1, &UAVServer::subClickedPose, this);
  sub_position_goal_  = n.subscribe("uav_server/position_goal", 1, &UAVServer::subPositionGoal, this);
  sub_mavros_state_   = n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_       = n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  // Start service server
  act_move_vehicle_server_.start();
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
    geometry_msgs::TransformStamped mavros_setpoint_tf = tf_buffer_.lookupTransform(uav_local_frame_, uav_position_goal_.header.frame_id, ros::Time(0));
    tf2::doTransform(uav_position_goal_, mavros_setpoint, mavros_setpoint_tf);
    
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

void UAVServer::subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg) {
  ROS_INFO("UAVServer: subClickedPose");

  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, clicked_pose_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*clicked_pose_msg, uav_position_goal_, position_goal_tf);
    uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subPositionGoal(const geometry_msgs::PoseStampedConstPtr& position_goal_msg) {
  ROS_INFO("UAVServer: subPositionGoal");

  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, position_goal_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*position_goal_msg, uav_position_goal_, position_goal_tf);
    uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subGlobalGoal: %s",ex.what());
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

    // Visualize drone
    if (visualizeDrone()) {
      ROS_WARN("Failed to visualize drone in RViz");
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("UAVServer: subOdometry: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

// Actionlib callback

void UAVServer::actMoveVehicle(const mipp_msgs::MoveVehicleGoalConstPtr &goal)
{
  ROS_DEBUG("actMoveVehicle");
  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, goal->goal_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(goal->goal_pose, uav_position_goal_, position_goal_tf);
    uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_INFO("UAVServer: New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();

    ros::Time start_time = ros::Time::now();
    while ((ros::Time::now() - start_time).toSec() < goal->max_time) {
      act_move_vehicle_feedback_.goal_euc_distance = getDistanceBetweenPoints(uav_pose_.pose.position, uav_position_goal_.pose.position);
      act_move_vehicle_feedback_.goal_yaw_distance = abs(angles::shortest_angular_distance(uav_rpy_.vector.z, uav_position_goal_rpy_.z));
      act_move_vehicle_server_.publishFeedback(act_move_vehicle_feedback_);
      if (act_move_vehicle_feedback_.goal_euc_distance < goal->goal_reached_radius and 
          act_move_vehicle_feedback_.goal_yaw_distance < goal->goal_reached_yaw) {
        act_move_vehicle_result_.time_used = (ros::Time::now() - start_time).toSec();
        act_move_vehicle_server_.setSucceeded(act_move_vehicle_result_);
      }
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
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
  np.param<bool>("uav_do_clearing_rotation", uav_do_clearing_rotation_, false);
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
    pub_mavros_setpoint_.publish(uav_position_goal_);
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
    pub_mavros_setpoint_.publish(uav_position_goal_);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("UAVServer: Vehicle takeoff procedure complete");

  uav_position_goal_.pose.position.x = 0.0;
  uav_position_goal_.pose.position.y = 0.0;
  uav_position_goal_.pose.position.z = uav_takeoff_z_;
  if (uav_do_clearing_rotation_) {
    ROS_INFO("UAVServer: Performing clearing rotation");
    double clearing_rotation_angle = 0.0;
    while (clearing_rotation_angle <= 360.0) {
      double clearing_rotation = angles::from_degrees(clearing_rotation_angle);
      double angle_threshold = angles::from_degrees(10.0);
      uav_position_goal_.pose.orientation = makeQuatFromRPY(0.0, 0.0, clearing_rotation);
      while (angles::shortest_angular_distance(uav_rpy_.vector.z, clearing_rotation) > angle_threshold) {
        pub_mavros_setpoint_.publish(uav_position_goal_);
        ros::spinOnce();
        rate.sleep();
      }
      clearing_rotation_angle += 60;
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("UAVServer: Clearing rotations complete");
    uav_clearing_rotation_complete_ = true;
  }
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

int UAVServer::visualizeDrone() {
  visualization_msgs::Marker drone;
  drone.header.frame_id = uav_world_frame_;
  drone.header.stamp = ros::Time::now();
  drone.type = visualization_msgs::Marker::MESH_RESOURCE;
  drone.mesh_resource = "model://matrice_100/meshes/Matrice_100.dae";
  if (drone.mesh_resource.find("model://") != std::string::npos) {
    if (resolveUri(drone.mesh_resource)) {
      ROS_ERROR("RVIZ world loader could not find drone model");
      return 1;
    }
  }
  drone.mesh_use_embedded_materials = true;
  drone.scale.x = 1.5;
  drone.scale.y = 1.5;
  drone.scale.z = 1.5;
  drone.pose.position.x = uav_pose_.pose.position.x;
  drone.pose.position.y = uav_pose_.pose.position.y;
  drone.pose.position.z = uav_pose_.pose.position.z;
  drone.pose.orientation.x = uav_pose_.pose.orientation.x;
  drone.pose.orientation.y = uav_pose_.pose.orientation.y;
  drone.pose.orientation.z = uav_pose_.pose.orientation.z;
  drone.pose.orientation.w = uav_pose_.pose.orientation.w;
  drone.id = 0;
  drone.lifetime = ros::Duration();
  drone.action = visualization_msgs::Marker::ADD;

  pub_viz_uav_.publish(drone);

  return 0;
}