#include <ros/ros.h>

#include <utils.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"

#include <string>
#include <math.h> /* sqrt, pow */
 
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
  * @param global_goal_msg Message sent to topic.
  */
  void subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& global_goal_msg);
  
  /**
  * @brief Callback for local goal/setpoint, which avoidance is setting.
  * @param local_goal_msg Message sent to topic.
  */
  void subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg);
  
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

  void subOctomap(const octomap_msgs::Octomap& octomap_msg);
  
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

  double calculateInformationGain(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);

  void visualizeUAVFOV();
  void visualizeFOV(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);
  void visualizeInformationPoints();
  
private:
  // Publishers    
  ros::Timer pub_timer_mavros_setpoint_;
  ros::Publisher pub_mavros_setpoint_;
  ros::Publisher pub_viz_uav_fov_;
  ros::Publisher pub_viz_fov_;
  ros::Publisher pub_viz_information_points_;
  // Subscribers
  ros::Subscriber sub_clicked_pose_;
  ros::Subscriber sub_global_goal_;
  ros::Subscriber sub_local_goal_;
  ros::Subscriber sub_mavros_state_;
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_octomap_;
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
  double uav_takeoff_z_;
  double uav_camera_width_;
  double uav_camera_height_;
  double uav_camera_hfov_;
  double uav_camera_range_;
  // Variables
  octomap::OcTree* map_;
  bool received_map_;
  bool uav_takeoff_complete_;
  geometry_msgs::PoseStamped uav_global_goal_;
  geometry_msgs::PoseStamped uav_local_goal_;
  float uav_global_goal_dist_;
  geometry_msgs::PoseStamped uav_pose_;
  geometry_msgs::Vector3Stamped uav_rpy_;
  mavros_msgs::State uav_state_;
  std::vector<tf2::Vector3> uav_camera_rays_;
  std::vector<tf2::Vector3> uav_camera_corner_rays_;
  std::vector<std::pair<double, geometry_msgs::Point>> uav_camera_information_points_;
};
