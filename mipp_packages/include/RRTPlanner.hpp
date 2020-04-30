#include <ros/ros.h>

#include <Node.hpp>
#include <utils.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
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
  
  /* 
  *  Visualization functions
  */
  void visualizeTree();

private:
  ros::Publisher pub_random_point_;
  ros::Publisher pub_viz_tree_;
  Node root_;
  std::list<Node> tree_;
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
  double max_ray_distance_;
};
