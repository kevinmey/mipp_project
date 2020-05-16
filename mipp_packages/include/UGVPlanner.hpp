#pragma once

#include <ros/ros.h>
#include "octomap/octomap.h"

#include <Node.hpp>
#include <utils.hpp>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

#include <string>
#include <cmath> /* sqrt, pow */
#include <random>

namespace ugv_planner {

class UGVPlanner : public nav_core::BaseGlobalPlanner 
{
public:
  // Constructor
  UGVPlanner();
  UGVPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  // Destructor
  ~UGVPlanner();
  // Init.
  // overriden classes from interface nav_core::BaseGlobalPlanner:
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  
  /* 
  *  Subscriber callbacks
  */

 void subOdometry(const nav_msgs::Odometry& odometry_msg);
  
  /* 
  *  Planner callbacks
  */
  // overriden classes from interface nav_core::BaseGlobalPlanner:
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
  geometry_msgs::Point generateRandomPoint();
  geometry_msgs::Point generateRandomInformedPoint();
  void extendTreeRRTstar(geometry_msgs::Point candidate_point);
  bool isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point& point_b,
                           bool return_collision_free_point,
                           geometry_msgs::Vector3 direction_ab = makeVector3(0.0,0.0,0.0), 
                           double distance = -1.0);
  
  /* 
  *  Utility functions
  */
  void getParams(ros::NodeHandle np);
  geometry_msgs::PoseStamped makePoseStampedFromNode(Node node);
  geometry_msgs::Pose makePoseFromNode(Node node);
  
  /* 
  *  Visualization functions
  *  Defined in RRTPlannerVisualization.cpp
  */
  void visualizeTree();
  void visualizeTreeColored();
  void visualizeCollisionTree();
  void visualizeCollisionTree(std::vector<geometry_msgs::Point> collision_tree);
  void visualizePathToGoal();
  void visualizeRoot(geometry_msgs::Point point, double red, double green, double blue);
  void visualizeGoal(geometry_msgs::Point point, double red, double green, double blue);
  void visualizeSubgoal(geometry_msgs::Point point, double red, double green, double blue);

private:
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_collision_tree_;
  ros::Publisher pub_viz_path_to_goal_;
  ros::Publisher pub_viz_root_node_;
  ros::Publisher pub_viz_goal_node_;
  ros::Publisher pub_viz_subgoal_node_;
  ros::Publisher pub_viz_frontier_nodes_;
  ros::Subscriber sub_odometry_;
  tf::TransformListener tf_listener_;
  // Planner variables
  Node root_;
  Node goal_;
  std::list<Node> tree_;
  std::vector<geometry_msgs::Point> collision_tree_;
  std::list<geometry_msgs::Point> path_;
  std::list<geometry_msgs::Point> path_remaining_;
  // For global_planner plugin:
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  base_local_planner::WorldModel* world_model_;
  double step_size_;
  double min_dist_from_robot_;
  double footprintCost(double x_i, double y_i, double theta_i);
  // Informed RRT*
  double goal_euclidean_distance_;
  double goal_path_distance_;
  double goal_grow_distance_;
  double goal_root_rotation_[2][2];
  geometry_msgs::Point goal_root_midpoint_;
  // Random nr. generator and distributions
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
  std::uniform_real_distribution<double> unit_distribution_;
  // Collision check variables
  std::vector<geometry_msgs::Point> collision_points_;
  // Parameters
  std::string planner_world_frame_;
  double goal_sample_probability_;
  double goal_radius_;
  double ugv_midpoint_z_;
  bool use_dynamic_range_;
  double dynamic_range_padding_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  double z_range_min_;
  double z_range_max_;
  double planner_rate_;
  int planner_max_tree_nodes_;
  double planner_max_time_;
  double planner_max_ray_distance_;
};

};