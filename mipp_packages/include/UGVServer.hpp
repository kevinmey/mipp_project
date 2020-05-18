#include <ros/ros.h>

#include <Node.hpp>
#include <utils.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <random>
#include <string>
 
class UGVServer
{
public:
  // Constructor
  UGVServer(ros::NodeHandle n, ros::NodeHandle np);
  // Destructor
  ~UGVServer();
  
private:
  void subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg);
  void subMap(const nav_msgs::OccupancyGridConstPtr& map_msg);
  void subMapUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map_update_msg);
  void subOdometry(const nav_msgs::Odometry odometry_msg);
  // Frontier exploration functions
  void runFrontierExploration();
  void extendTreeRRTstar(geometry_msgs::Point candidate_point);
  geometry_msgs::Point generateRandomPoint();
  // Gen. util functions
  void getParams(ros::NodeHandle np);
  // Map util. functions
  void convMapToWorld(int map_x, int map_y, double& world_x, double& world_y);
  void convWorldToMap(double world_x, double world_y, int& map_x, int& map_y);
  int getGridIndex(int map_x, int map_y);
  int getGridIndex(double world_x, double world_y);
  bool isPathCollisionFree(double world_x_a, double world_y_a, double world_x_b, double world_y_b);
  bool isPositionOutsideMap(double world_x, double world_y);
  bool isPositionCollisionFree(double world_x, double world_y);
  bool isPositionUnmapped(double world_x, double world_y);
  // Visualization
  void visualizeTree();
  void visualizeRoot(geometry_msgs::Point point, double red, double green, double blue);
  void visualizeFrontierNodes(double red, double green, double blue);
  geometry_msgs::PoseStamped makePoseStampedFromNode(Node node);
  
  ros::Publisher pub_goal_;
  ros::Publisher pub_viz_tree_;
  ros::Publisher pub_viz_root_node_;
  ros::Publisher pub_viz_frontier_nodes_;
  ros::Subscriber sub_clicked_point_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_update_;
  ros::Subscriber sub_odometry_;
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_; 
  // UGV variables
  nav_msgs::Odometry ugv_odometry_;
  // Frontier exploration variables
  Node root_;
  std::list<Node> tree_;
  std::list<geometry_msgs::Point> path_;
  std::list<geometry_msgs::Point> path_remaining_;
  std::map<double, Node> frontier_nodes_;
  geometry_msgs::PoseStamped current_frontier_goal_;
  bool running_frontier_exploration_;
  // Map variables
  bool map_initialized_;
  bool map_in_use_;
  double map_resolution_;
  double map_origin_x_;
  double map_origin_y_;
  int map_grid_width_;
  int map_grid_height_;
  double map_width_;
  double map_height_;
  nav_msgs::OccupancyGrid map_;
  // Random nr. generator and distributions
  std::default_random_engine generator_;
  std::uniform_real_distribution<double> x_distribution_;
  std::uniform_real_distribution<double> y_distribution_;
  std::uniform_real_distribution<double> unit_distribution_;
  // Params
  bool use_dynamic_range_;
  double dynamic_range_padding_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  std::string planner_world_frame_;
  double planner_rate_;
  double planner_max_ray_distance_;
  double planner_max_time_;
  double planner_min_distance_to_frontier_;
  double planner_max_distance_to_frontier_;
  int collision_threshold_;
  bool unmapped_is_collision_;
};
