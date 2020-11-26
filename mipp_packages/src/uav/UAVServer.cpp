#include <UAVServer.hpp>

// Constructor
  
UAVServer::UAVServer(ros::NodeHandle n, ros::NodeHandle np)
  : act_move_vehicle_server_(n, "move_vehicle_action", boost::bind(&UAVServer::actMoveVehicle, this, _1), false) 
{
  ROS_INFO("UAVServer object is being created.");

  // Initialize values
  getParams(np);
  // Establish publish timers and publishers
  pub_timer_mavros_setpoint_  = n.createTimer(ros::Duration(0.1), boost::bind(&UAVServer::pubMavrosSetpoint, this));
  pub_mavros_setpoint_        = n.advertise<geometry_msgs::PoseStamped>("uav_server/mavros_setpoint", 10);
  pub_mavros_cmd_vel_         = n.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  pub_global_goal_            = n.advertise<geometry_msgs::PoseStamped>("uav_server/global_goal", 1);
  pub_viz_uav_fov_            = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav_fov", 1);
  pub_viz_uav_                = n.advertise<visualization_msgs::Marker>("uav_server/viz_uav", 1);
  pub_viz_line_to_goal_       = n.advertise<visualization_msgs::Marker>("uav_server/viz_line_to_goal", 1);
  // Establish subscriptions
  sub_clicked_pose_   = n.subscribe("uav_server/clicked_goal", 1, &UAVServer::subClickedPose, this);
  sub_position_goal_  = n.subscribe("uav_server/position_goal", 1, &UAVServer::subPositionGoal, this);
  sub_mavros_state_   = n.subscribe("uav_server/mavros_state", 1, &UAVServer::subMavrosState, this);
  sub_odometry_       = n.subscribe("uav_server/ground_truth_uav", 1, &UAVServer::subOdometry, this);
  sub_local_goal_     = n.subscribe("uav_server/local_goal", 1, &UAVServer::subLocalGoal, this);
  sub_cmd_vel_        = n.subscribe("cmd_vel", 1, &UAVServer::subCmdVel, this);
  // Start service server
  act_move_vehicle_server_.start();
  // Establish service clients
  cli_takeoff_complete_ = n.advertiseService("takeoff_complete_service", &UAVServer::cliIsTakeoffComplete, this);
  cli_arm_              = n.serviceClient<mavros_msgs::CommandBool>("uav_server/arm");
  cli_set_mode_         = n.serviceClient<mavros_msgs::SetMode>("uav_server/set_mode");
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
  ROS_DEBUG("pubMavrosSetpoint");
  // Block if UAV still initializing (taking off)
  if(!uav_takeoff_complete_ or !uav_clearing_rotation_complete_)
  {
    return;
  }

  // http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html
  mavros_msgs::PositionTarget target_cmd_vel_;
  target_cmd_vel_.header.frame_id = uav_body_frame_;
  target_cmd_vel_.header.stamp = ros::Time::now();
  target_cmd_vel_.coordinate_frame = 8; // FRAME_BODY_NED
  //if ((ros::Time::now() - uav_cmd_vel_received_time_).toSec() < 0.5) {
  if (uav_use_move_base_) {
    target_cmd_vel_.type_mask = 0b010111000111;
    target_cmd_vel_.velocity.x = uav_cmd_vel_.linear.x;
    target_cmd_vel_.velocity.y = uav_cmd_vel_.linear.y;
    float k = 0.1;
    float e = uav_position_goal_.pose.position.z - uav_pose_.pose.position.z;
    target_cmd_vel_.velocity.z = k*e;
    target_cmd_vel_.yaw_rate = uav_cmd_vel_.angular.z;
  }/*
  else if (true) {
    target_cmd_vel_.type_mask = 0b100111000111;
    double uav_max_vel_ = 3.0;
    double uav_diff_x = uav_position_goal_.pose.position.x - uav_pose_.pose.position.x;
    double uav_diff_y = uav_position_goal_.pose.position.y - uav_pose_.pose.position.y;
    double uav_diff_z = uav_position_goal_.pose.position.z - uav_pose_.pose.position.z;
    double uav_diff_dist = getDistanceBetweenPoints(uav_position_goal_.pose.position, uav_pose_.pose.position);
    target_cmd_vel_.velocity.x = (uav_diff_x/uav_diff_dist)*uav_max_vel_;
    target_cmd_vel_.velocity.y = (uav_diff_y/uav_diff_dist)*uav_max_vel_;
    target_cmd_vel_.velocity.z = (uav_diff_z/uav_diff_dist)*uav_max_vel_;
    target_cmd_vel_.yaw = uav_position_goal_rpy_.z;
  }*/
  else {
    geometry_msgs::PoseStamped mavros_setpoint;
    geometry_msgs::PoseStamped capped_position_goal = uav_position_goal_;
    bool cap_distance = true;
    float distance_cap = 3.0;
    if (cap_distance and getDistanceBetweenPoints(uav_pose_.pose.position, uav_position_goal_.pose.position) > distance_cap) {
      capped_position_goal.pose.position = castRay(uav_pose_.pose.position, getDirection(uav_pose_.pose.position, uav_position_goal_.pose.position), distance_cap);
      ROS_DEBUG("(%.2f, %.2f) -> (%.2f, %.2f) : (%.2f, %.2f)", uav_pose_.pose.position.x, uav_pose_.pose.position.y, 
                                                              uav_position_goal_.pose.position.x, uav_position_goal_.pose.position.y,
                                                              capped_position_goal.pose.position.x, capped_position_goal.pose.position.y);
    }
    geometry_msgs::TransformStamped mavros_setpoint_tf = tf_buffer_.lookupTransform(uav_local_frame_, capped_position_goal.header.frame_id, ros::Time(0));
    tf2::doTransform(capped_position_goal, mavros_setpoint, mavros_setpoint_tf);
    target_cmd_vel_.type_mask = 0b100111111000;
    target_cmd_vel_.position.x = mavros_setpoint.pose.position.y;
    target_cmd_vel_.position.y = -mavros_setpoint.pose.position.x;
    target_cmd_vel_.position.z = mavros_setpoint.pose.position.z;
    target_cmd_vel_.yaw = uav_position_goal_rpy_.z;
  }
  pub_mavros_cmd_vel_.publish(target_cmd_vel_);

  return;
  /* Transform local goal back to uav local origin/odom:
  *  MAVROS takes setpoints/commands in it's own local frame (as
  *  if it started at origin). Since local goal has been trans-
  *  formed into world frame, we must transform it back before
  *  sending it as a setpoint.
  */ 
  try{
    geometry_msgs::PoseStamped mavros_setpoint;
    if (uav_local_goal_received_) {
      mavros_setpoint = uav_local_goal_;
      if (getDistanceBetweenPoints(uav_pose_.pose.position, uav_position_goal_.pose.position) < 1.5) {
        mavros_setpoint.pose.orientation = uav_position_goal_.pose.orientation;
      }
    }
    else {
      geometry_msgs::TransformStamped mavros_setpoint_tf = tf_buffer_.lookupTransform(uav_local_frame_, uav_position_goal_.header.frame_id, ros::Time(0));
      tf2::doTransform(uav_position_goal_, mavros_setpoint, mavros_setpoint_tf);
    }
    
    pub_mavros_setpoint_.publish(mavros_setpoint);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("pubMavrosSetpoint: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/* 
*  Callback functions for subscriptions
*/

void UAVServer::subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg) {
  ROS_INFO("subClickedPose");

  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, clicked_pose_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*clicked_pose_msg, uav_position_goal_, position_goal_tf);
    uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_INFO("New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();

    pub_global_goal_.publish(uav_position_goal_);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subPositionGoal(const geometry_msgs::PoseStampedConstPtr& position_goal_msg) {
  ROS_DEBUG("subPositionGoal");

  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, position_goal_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*position_goal_msg, uav_position_goal_, position_goal_tf);
    //uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_DEBUG("New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();
    if (uav_use_move_base_) {
      pub_global_goal_.publish(uav_position_goal_);
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

void UAVServer::subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg) { 
  ROS_DEBUG("subMavrosState");
  uav_state_ = *mavros_state_msg;
}

void UAVServer::subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg) { 
  ROS_DEBUG("subOdometry");
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

    // Store absolute velocity
    uav_abs_vel_ = std::sqrt(std::pow(odometry_msg->twist.twist.linear.x, 2)
                           + std::pow(odometry_msg->twist.twist.linear.y, 2)
                           + std::pow(odometry_msg->twist.twist.linear.z, 2));
    uav_abs_vel_max_ = (uav_abs_vel_max_ < uav_abs_vel_) ? uav_abs_vel_ : uav_abs_vel_max_;
    ROS_DEBUG("Velocity: %.2f / %.2f", uav_abs_vel_, uav_abs_vel_max_);

    // Visualize drone
    if (visualizeDrone()) {
      ROS_WARN("Failed to visualize drone in RViz");
    }
    visualizeLineToGoal();
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("subOdometry: %s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void UAVServer::subLocalGoal(const geometry_msgs::PoseStampedConstPtr& local_goal_msg) {
  ROS_DEBUG("subLocalGoal");
  if (!uav_local_goal_received_) { uav_local_goal_received_ = true; }
  uav_local_goal_ = *local_goal_msg;
}

void UAVServer::subCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
  ROS_DEBUG("subCmdVel");
  uav_cmd_vel_ = *cmd_vel_msg;
  uav_cmd_vel_received_time_ = ros::Time::now();
}

// Service callback

bool UAVServer::cliIsTakeoffComplete(mipp_msgs::TakeoffComplete::Request& request, mipp_msgs::TakeoffComplete::Response& response) {
  ROS_DEBUG("cliIsTakeoffComplete");
  response.takeoff_complete = uav_takeoff_complete_ and uav_clearing_rotation_complete_;
  return true;
}

// Actionlib callback

void UAVServer::actMoveVehicle(const mipp_msgs::MoveVehicleGoalConstPtr &goal)
{
  ROS_DEBUG("actMoveVehicle");
  try{
    geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, goal->goal_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(goal->goal_pose, uav_position_goal_, position_goal_tf);
    //uav_position_goal_.pose.position.z = uav_takeoff_z_;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(uav_position_goal_.pose.orientation, tf_quat);
    tf2::Matrix3x3(tf_quat).getRPY(uav_position_goal_rpy_.x, 
                                   uav_position_goal_rpy_.y, 
                                   uav_position_goal_rpy_.z);
    ROS_INFO("New global goal: [%f,%f,%f,%f]",
              uav_position_goal_.pose.position.x, 
              uav_position_goal_.pose.position.y,
              uav_position_goal_.pose.position.z,
              uav_position_goal_rpy_.z);  

    uav_position_goal_.header.frame_id = uav_world_frame_;
    uav_position_goal_.header.stamp = ros::Time::now();
    uav_use_move_base_ = goal->goal_path_to_be_improved;
    if (uav_use_move_base_) {
      pub_global_goal_.publish(uav_position_goal_);
    }

    ros::Time start_time = ros::Time::now();
    bool goal_reached = false;
    while ((ros::Time::now() - start_time).toSec() < goal->goal_reached_max_time) {
      // Try to reach goal
      act_move_vehicle_feedback_.goal_euc_distance = getDistanceBetweenPoints(uav_pose_.pose.position, uav_position_goal_.pose.position);
      act_move_vehicle_feedback_.goal_yaw_distance = abs(angles::shortest_angular_distance(uav_rpy_.vector.z, uav_position_goal_rpy_.z));
      act_move_vehicle_server_.publishFeedback(act_move_vehicle_feedback_);
      if (act_move_vehicle_feedback_.goal_euc_distance < goal->goal_reached_radius and 
          act_move_vehicle_feedback_.goal_yaw_distance < goal->goal_reached_yaw) {
        act_move_vehicle_result_.time_used = (ros::Time::now() - start_time).toSec();
        act_move_vehicle_server_.setSucceeded(act_move_vehicle_result_);
        goal_reached = true;
        ROS_INFO("UAV %d reached its navigation goal after %.2f seconds.", uav_id_, act_move_vehicle_result_.time_used);
        break;
      }
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    if (!goal_reached) {
      act_move_vehicle_result_.time_used = (ros::Time::now() - start_time).toSec();
      act_move_vehicle_server_.setAborted();
      ROS_INFO("UAV %d didn't reach its navigation goal after %.2f seconds.", uav_id_, goal->goal_reached_max_time);
    }
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("subGlobalGoal: %s",ex.what());
    ros::Duration(0.1).sleep();
  }
}

// Utility functions

void UAVServer::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  np.param<int>("uav_id", uav_id_, 0);
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav"+std::to_string(uav_id_));
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav"+std::to_string(uav_id_));
  np.param<double>("uav_start_x", uav_start_x_, 0.0);
  np.param<double>("uav_start_y", uav_start_y_, 0.0);
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<bool>("uav_do_clearing_rotation", uav_do_clearing_rotation_, true);
  np.param<double>("uav_clearing_rotation_angle", uav_clearing_rotation_angle_, 60.0);
}

void UAVServer::takeoff() {
  ROS_DEBUG("takeoff");

  ros::Rate rate(20.0);
  uav_takeoff_complete_ = false;
  uav_clearing_rotation_complete_ = false;
  uav_use_move_base_= false;

  // Establish correct global goal
  uav_position_goal_.header.frame_id = uav_world_frame_;
  uav_position_goal_.header.stamp = ros::Time::now();
  // Set position goal to world frame
  bool got_tf = false;
  while (!got_tf) {
    try{
      geometry_msgs::TransformStamped position_goal_tf = tf_buffer_.lookupTransform(uav_world_frame_, uav_local_frame_, ros::Time(0));
      uav_position_goal_.pose.position.x = position_goal_tf.transform.translation.x;
      uav_position_goal_.pose.position.y = position_goal_tf.transform.translation.y;
      uav_position_goal_.pose.position.z = uav_takeoff_z_;
      uav_position_goal_.pose.orientation.w = 1.0;
      ROS_WARN("Global goal initialized to: [%.2f, %.2f, %.2f, %.2f]",
            uav_position_goal_.pose.position.x, 
            uav_position_goal_.pose.position.y,
            uav_position_goal_.pose.position.z,
            uav_position_goal_rpy_.z); 
      got_tf = true;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("pubMavrosSetpoint: %s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  // Establish mavros setpoint to be used in takeoff
  geometry_msgs::PoseStamped mavros_setpoint;
  mavros_setpoint.header.frame_id = uav_local_frame_;
  mavros_setpoint.pose.position.x = 0.0;
  mavros_setpoint.pose.position.y = 0.0;
  mavros_setpoint.pose.position.z = uav_takeoff_z_;
  mavros_setpoint.pose.orientation.w = 1.0;

  // Wait for FCU connection
  while(ros::ok() && !uav_state_.connected){
    ros::spinOnce();
    rate.sleep();
  }
  ROS_DEBUG("FCU connected");

  ros::Duration(5.0).sleep();

  // Send a few setpoints before starting procedure
  for(int i = 50; ros::ok() && i > 0; --i){
    pub_mavros_setpoint_.publish(mavros_setpoint);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_DEBUG("Initial setpoints sent");

  // Keep track of when last request/command was issued for timing purposes
  ros::Time last_request = ros::Time::now();
  float wait_duration = 2.0; // Give each step about 2 seconds

  // Mavros request/command procedure to get drone to take off
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
        ROS_INFO("Offboard enabled");
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
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
        uav_takeoff_complete_ = true;
      }
    }
    pub_mavros_setpoint_.publish(mavros_setpoint);
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Vehicle takeoff procedure complete");

  if (uav_do_clearing_rotation_) {
    ROS_INFO("Performing clearing rotation");
    double clearing_rotation_angle = 0.0;
    while (clearing_rotation_angle < 360.0) {
      double clearing_rotation = angles::from_degrees(clearing_rotation_angle);
      double angle_threshold = angles::from_degrees(10.0);
      mavros_setpoint.pose.orientation = makeQuatFromRPY(0.0, 0.0, clearing_rotation);
      while (angles::shortest_angular_distance(uav_rpy_.vector.z, clearing_rotation) > angle_threshold) {
        pub_mavros_setpoint_.publish(mavros_setpoint);
        ros::spinOnce();
        rate.sleep();
      }
      clearing_rotation_angle += uav_clearing_rotation_angle_;
      ros::Duration(0.5).sleep();
    }
    ROS_INFO("Clearing rotations complete");
    uav_clearing_rotation_complete_ = true;
  }
  uav_clearing_rotation_complete_ = true;
}

geometry_msgs::Quaternion UAVServer::makeQuatFromRPY(geometry_msgs::Vector3 rpy) {
  ROS_DEBUG("makeQuatFromRPY");
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
  ROS_DEBUG("makeQuatFromRPY");
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

void UAVServer::visualizeLineToGoal() {
  ROS_DEBUG("visualizeLineToGoal");
  
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = uav_world_frame_;
  line_marker.header.stamp = ros::Time::now();
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.05;
  line_marker.color.a = 1.0;
  line_marker.color.r = 0.1;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.1;
  line_marker.points.push_back(uav_pose_.pose.position);
  line_marker.points.push_back(uav_position_goal_.pose.position);
  
  pub_viz_line_to_goal_.publish(line_marker);
}