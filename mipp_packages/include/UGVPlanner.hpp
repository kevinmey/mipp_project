#include <ros/ros.h>
#include "octomap/octomap.h"

#include <Node.hpp>
#include <utils.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <cmath> /* sqrt, pow */
#include <random>
 
class UGVPlanner
{
public:
  // Constructor
  UGVPlanner(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~UGVPlanner();
  
  /* 
  *  Subscriber callbacks
  */
  void subOctomap(const octomap_msgs::Octomap& octomap_msg);
  void subRoot(const geometry_msgs::PoseWithCovarianceStamped& goal_msg);
  void subGoal(const geometry_msgs::PoseStamped& goal_msg);
  
  /* 
  *  Planner callbacks
  */
  void planPathToGoal();
  geometry_msgs::Point generateRandomPoint();
  geometry_msgs::Point generateRandomInformedPoint();
  void extendTreeRRTstar(geometry_msgs::Point candidate_point);
  bool isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point point_b, 
                           geometry_msgs::Vector3 direction_ab = makeVector3(0.0,0.0,0.0), 
                           double distance = -1.0);
  
  /* 
  *  Utility functions
  */
  void getParams(ros::NodeHandle np);
  
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

private:
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_collision_tree_;
  ros::Publisher pub_viz_path_to_goal_;
  ros::Publisher pub_viz_root_node_;
  ros::Publisher pub_viz_goal_node_;
  ros::Subscriber sub_octomap_;
  ros::Subscriber sub_root_;
  ros::Subscriber sub_goal_;
  // Planner variables
  Node root_;
  Node goal_;
  std::list<Node> tree_;
  std::vector<geometry_msgs::Point> collision_tree_;
  octomap::OcTree* map_;
  bool received_map_;
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
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  double z_range_min_;
  double z_range_max_;
  double planner_rate_;
  int planner_max_tree_nodes_;
  double planner_max_time_;
  double max_ray_distance_;
};
