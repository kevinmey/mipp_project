#include <ros/ros.h>

#include <utils.hpp>
#include <Node.hpp>

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
  
  /* 
  *  Publish functions for publishers
  */
 
  /**
  * @brief Publishes uav_desired_pose_ to mavros setpoint.
  */
  void pubMavrosSetpoint();
  
  /* 
  *  Callback functions for subscriptions
  */

  void subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg);
  
  /**
  * @brief Callback for global goal, which avoidance is setting.
  * @param position_goal_msg Message sent to topic.
  */
  void subPositionGoal(const geometry_msgs::PoseStamped::ConstPtr& position_goal_msg);
  
  /**
  * @brief Callback for mavros drone state, f.ex. OFFBOARD.
  * @param mavros_state_msg Message sent to topic.
  */
  void subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg);
  
  /**
  * @brief Callback for ground truth odometry of UAV from gazebo.
  * @param odometry_msg Message sent to topic.
  */
  void subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg);
  
  /* 
  *  Utility functions
  */
  
  /**
  * @brief Get parameters from rosparam in launch.
  * @param np Private nodehandle.
  */
  void getParams(ros::NodeHandle np);
  
  /**
  * @brief Mavros procedure to takeoff the drone. Done on init.
  */
  void takeoff();

  void visualizeUAVFOV();
  void visualizeFOV(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);
  int visualizeDrone();
  
  geometry_msgs::Quaternion makeQuatFromRPY(geometry_msgs::Vector3 rpy);
  geometry_msgs::Quaternion makeQuatFromRPY(double r, double p, double y);
  
private:
  // Publishers    
  ros::Timer pub_timer_mavros_setpoint_;
  ros::Publisher pub_mavros_setpoint_;
  ros::Publisher pub_mavros_cmd_vel_;
  ros::Publisher pub_position_goal_;
  ros::Publisher pub_viz_uav_fov_;
  ros::Publisher pub_viz_fov_;
  ros::Publisher pub_viz_uav_;
  // Subscribers
  ros::Subscriber sub_clicked_pose_;
  ros::Subscriber sub_position_goal_;
  ros::Subscriber sub_mavros_state_;
  ros::Subscriber sub_odometry_;
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
  // Variables
  //   Map
  octomap::OcTree* map_;
  bool received_map_;
  //   Planner
  bool uav_takeoff_complete_;
  bool uav_clearing_rotation_complete_;
  geometry_msgs::PoseStamped uav_position_goal_;
  geometry_msgs::Vector3 uav_position_goal_rpy_;
  //   State
  geometry_msgs::PoseStamped uav_pose_;
  geometry_msgs::Vector3Stamped uav_rpy_;
  mavros_msgs::State uav_state_;
  //   Sensor
};
