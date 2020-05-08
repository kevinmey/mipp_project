#include <ros/ros.h>
#include "octomap/octomap.h"

#include <Node.hpp>
#include <utils.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <random>
#include <string>
#include <math.h> /* sqrt, pow */
 
class RRTPlanner
{
public:
  RRTPlanner(ros::NodeHandle n, ros::NodeHandle np);
  ~RRTPlanner();

  void subOctomap(const octomap_msgs::Octomap& octomap_msg);
  
  /* 
  *  Utility functions
  */
  
  void getParams(ros::NodeHandle np);
  geometry_msgs::Point generateRandomPoint(bool publish_point);
  void extendTreeRRT(geometry_msgs::Point candidate_point, Node* nearest_neighbor);
  
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
  ros::Publisher pub_random_point_;
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_collision_tree_;
  ros::Publisher pub_viz_path_to_goal_;
  ros::Publisher pub_viz_root_node_;
  ros::Publisher pub_viz_goal_node_;
  ros::Subscriber sub_octomap_;
  // Planner variables
  Node root_;
  Node goal_;
  std::map<int, Node> tree_;
  std::vector<geometry_msgs::Point> collision_tree_;
  octomap::OcTree* map_;
  bool received_map_;
  // Random nr. generator and distributions
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
  std::uniform_real_distribution<double> unit_distribution_;
  // Parameters
  std::string planner_world_frame_;
  double root_x_;
  double root_y_;
  double root_z_;
  double goal_x_;
  double goal_y_;
  double goal_z_;
  double goal_sample_probability_;
  double goal_radius_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  double z_range_min_;
  double z_range_max_;
  double planner_rate_;
  int planner_algorithm_;
  int planner_max_tree_nodes_;
  double max_ray_distance_;
};
