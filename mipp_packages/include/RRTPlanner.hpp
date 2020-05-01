#include <ros/ros.h>
#include "octomap/octomap.h"

#include <Node.hpp>
#include <utils.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"
#include <visualization_msgs/Marker.h>

#include <string>
#include <math.h> /* sqrt, pow */
 
class RRTPlanner
{
public:
  // Constructor
  RRTPlanner(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~RRTPlanner();

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
  * @brief Generates a random point from generators set in constrcutor
  * @param publish_point Bool whether to publish point or not
  * @return random 3d point 
  */
  geometry_msgs::Point generateRandomPoint(bool publish_point);

  /**
  * @brief Checks if two points and the path between is collision free
  * @param point_a Origin point
  * @param point_b Destination point
  * @param direction_ab Direction vector from a to b (optional)
  * @param distance Distance between points (optional)
  * @return True if path is collision free
  */
  bool isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point point_b, 
                           geometry_msgs::Vector3 direction_ab = makeVector3(0.0,0.0,0.0), 
                           double distance = -1.0);
  
  /* 
  *  Visualization functions
  */
  void visualizeTree();
  void visualizeCollisionTree();
  void visualizeCollisionTree(std::vector<geometry_msgs::Point> collision_tree);

private:
  ros::Publisher pub_random_point_;
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_collision_tree_;
  ros::Subscriber sub_octomap_;
  // Planner variables
  Node root_;
  std::list<Node> tree_;
  std::vector<geometry_msgs::Point> collision_tree_;
  octomap::OcTree* map_;
  bool received_map_;
  // Random nr. generator and distributions
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
  std::uniform_real_distribution<double> z_distribution_;
  // Parameters
  std::string planner_world_frame_;
  double root_x_;
  double root_y_;
  double root_z_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  double z_range_min_;
  double z_range_max_;
  double planning_rate_;
  double max_ray_distance_;
};
