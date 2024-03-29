#include <ros/ros.h>

#include <utils.hpp>
#include <Node.hpp>

#include <actionlib/server/simple_action_server.h>
#include <mipp_msgs/MoveVehicleAction.h>
#include <mipp_msgs/TakeoffComplete.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <angles/angles.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"

#include <string>
#include <math.h> /* sqrt, pow */
#include <random> 
 
class UAVServer
{
public:
  // Constructor
  UAVServer(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~UAVServer();
  // Publish functions
  void pubMavrosSetpoint();
  // Callback functions
  void subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg);
  void subPositionGoal(const geometry_msgs::PoseStamped::ConstPtr& position_goal_msg);
  void subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg);
  void subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg);
  void subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg);
  void subCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg);
  bool cliIsTakeoffComplete(mipp_msgs::TakeoffComplete::Request& request, mipp_msgs::TakeoffComplete::Response& response);
  // Actionlib
  void actMoveVehicle(const mipp_msgs::MoveVehicleGoalConstPtr &goal);

  
  /* 
  *  Utility functions
  */
  
  void getParams(ros::NodeHandle np);
  void takeoff();

  void visualizeUAVFOV();
  void visualizeFOV(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);
  int visualizeDrone();
  void visualizeLineToGoal();
  
  geometry_msgs::Quaternion makeQuatFromRPY(geometry_msgs::Vector3 rpy);
  geometry_msgs::Quaternion makeQuatFromRPY(double r, double p, double y);
  
private:
  // Publishers    
  ros::Timer pub_timer_mavros_setpoint_;
  ros::Publisher pub_mavros_setpoint_;
  ros::Publisher pub_mavros_cmd_vel_;
  ros::Publisher pub_position_goal_;
  ros::Publisher pub_global_goal_;
  ros::Publisher pub_viz_uav_fov_;
  ros::Publisher pub_viz_fov_;
  ros::Publisher pub_viz_uav_;
  ros::Publisher pub_viz_line_to_goal_;
  // Subscribers
  ros::Subscriber sub_clicked_pose_;
  ros::Subscriber sub_position_goal_;
  ros::Subscriber sub_mavros_state_;
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_local_goal_;
  ros::Subscriber sub_cmd_vel_;
  // Service servers
  actionlib::SimpleActionServer<mipp_msgs::MoveVehicleAction> act_move_vehicle_server_;
  mipp_msgs::MoveVehicleFeedback act_move_vehicle_feedback_;
  mipp_msgs::MoveVehicleResult act_move_vehicle_result_;
  ros::ServiceServer cli_takeoff_complete_;
  // Service clients
  ros::ServiceClient cli_arm_;
  ros::ServiceClient cli_set_mode_;
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_; 
  // Parameters
  int uav_id_;
  std::string uav_world_frame_;
  std::string uav_local_frame_;
  std::string uav_body_frame_;
  double uav_start_x_;
  double uav_start_y_;
  double uav_takeoff_z_;
  bool uav_do_clearing_rotation_;
  double uav_clearing_rotation_angle_;
  // Variables
  //   Map
  octomap::OcTree* map_;
  bool received_map_;
  //   Planner
  bool uav_takeoff_complete_;
  bool uav_clearing_rotation_complete_;
  geometry_msgs::PoseStamped uav_position_goal_;
  geometry_msgs::Vector3 uav_position_goal_rpy_;
  geometry_msgs::PoseStamped uav_local_goal_;
  bool uav_local_goal_received_;
  geometry_msgs::Twist uav_cmd_vel_;
  ros::Time uav_cmd_vel_received_time_;
  bool uav_use_move_base_;
  //   State
  geometry_msgs::PoseStamped uav_pose_;
  geometry_msgs::Vector3Stamped uav_rpy_;
  double uav_abs_vel_;
  double uav_abs_vel_max_;
  mavros_msgs::State uav_state_;
  //   Sensor
};
