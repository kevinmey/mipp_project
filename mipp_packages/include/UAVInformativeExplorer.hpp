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
 
class UAVInformativeExplorer
{
public:
  // Constructor
  UAVInformativeExplorer();
  UAVInformativeExplorer(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~UAVInformativeExplorer();
  
  /* 
  *  Publish functions for publishers
  */
  void pubMavrosSetpoint();
  
  /* 
  *  Callback functions for subscriptions
  */

  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);
  void subClickedPose(const geometry_msgs::PoseStampedConstPtr& clicked_pose_msg);
  void subGlobalGoal(const geometry_msgs::PoseStamped::ConstPtr& position_goal_msg);
  void subLocalGoal(const geometry_msgs::PoseStamped::ConstPtr& local_goal_msg);
  void subMavrosState(const mavros_msgs::State::ConstPtr& mavros_state_msg);
  void subOdometry(const nav_msgs::Odometry::ConstPtr& odometry_msg);
  void subOctomap(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
  
  /* 
  *  Utility functions
  */
  void getParams(ros::NodeHandle np);
  void initVariables();
  
  /**
  * @brief Mavros procedure to takeoff the drone. Done on init.
  */
  void takeoff();

  // Planner
  void runExploration();
  double calculateInformationGain(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);
  double calculateInformationGain(geometry_msgs::Point origin, double yaw);
  geometry_msgs::Point generateRandomPoint();
  void extendTreeRRTstar(geometry_msgs::Point candidate_point, double candidate_yaw);
  bool isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point point_b);
  bool isGoalReached();

  geometry_msgs::PoseStamped makePoseStampedFromNode(Node node);
  geometry_msgs::Quaternion makeQuatFromRPY(geometry_msgs::Vector3 rpy);
  geometry_msgs::Quaternion makeQuatFromRPY(double r, double p, double y);
  geometry_msgs::Vector3 makeRPYFromQuat(geometry_msgs::Quaternion quat);

  void visualizeUAVFOV();
  void visualizeFOV(geometry_msgs::Point origin, geometry_msgs::Vector3 rpy);
  void visualizeInformationPoints();
  void visualizeTree();
  void visualizePath();
  void visualizePathFOVs(double ray_length);
  
private:
  // Publishers    
  ros::Timer pub_timer_mavros_setpoint_;
  ros::Publisher pub_position_goal_;
  ros::Publisher pub_viz_uav_fov_;
  ros::Publisher pub_viz_fov_;
  ros::Publisher pub_viz_information_points_;
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_path_;
  // Subscribers
  ros::Subscriber sub_clicked_point_;
  ros::Subscriber sub_position_goal_;
  ros::Subscriber sub_odometry_;
  ros::Subscriber sub_octomap_;
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_; 
  // Random nr. generator and distributions
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> unit_distribution_;
  double planner_sample_radius_;
  // Parameters
  int uav_id_;
  std::string uav_world_frame_;
  std::string uav_local_frame_;
  std::string uav_body_frame_;
  double uav_start_x_;
  double uav_start_y_;
  double uav_takeoff_z_;
  double uav_camera_width_;
  double uav_camera_height_;
  double uav_camera_hfov_;
  double uav_camera_ray_resolution_;
  double uav_camera_range_;
  double planner_rate_;
  double planner_max_time_;
  double planner_max_ray_distance_;
  double planner_max_neighbor_distance_;
  double planner_max_neighbor_yaw_;
  bool planner_unmapped_is_collision_;
  // Variables
  //   Map
  std::shared_ptr<octomap::OcTree> map_;
  bool received_map_;
  //   Planner
  bool uav_takeoff_complete_;
  bool uav_clearing_rotation_complete_;
  bool uav_running_exploration_;
  geometry_msgs::PoseStamped uav_position_goal_;
  double uav_position_goal_yaw_;
  geometry_msgs::PoseStamped uav_local_goal_;
  float uav_position_goal_euc_dist_;
  float uav_position_goal_yaw_dist_;
  Node root_;
  std::list<Node> tree_;
  std::list<geometry_msgs::PoseStamped> path_;
  std::map<double, Node> exploration_nodes_;
  //   State
  geometry_msgs::PoseStamped uav_pose_;
  geometry_msgs::Vector3Stamped uav_rpy_;
  mavros_msgs::State uav_state_;
  //   Sensor
  std::vector<tf2::Vector3> uav_camera_rays_;
  std::vector<tf2::Vector3> uav_camera_corner_rays_;
  std::vector<std::pair<double, geometry_msgs::Point>> uav_camera_information_points_;
  // Friend class
  friend class JointExplorer;
};
