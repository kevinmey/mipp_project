#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include <string>
 
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
  
private:
  // Publishers    
  ros::Timer pub_timer_mavros_setpoint_;
  ros::Publisher pub_mavros_setpoint_;
  // Subscribers
  ros::Subscriber sub_global_goal_;
  ros::Subscriber sub_local_goal_;
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
  double uav_takeoff_z_;
  std::string uav_world_frame_;
  std::string uav_local_frame_;
  std::string uav_body_frame_;
  // Variables
  bool uav_takeoff_complete_;
  geometry_msgs::PoseStamped uav_global_goal_;
  geometry_msgs::PoseStamped uav_local_goal_;
  float uav_global_goal_dist;
  geometry_msgs::PoseStamped uav_pose_;
  mavros_msgs::State uav_state_;
};
