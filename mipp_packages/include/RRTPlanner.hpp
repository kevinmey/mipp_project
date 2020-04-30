#include <ros/ros.h>

#include <Node.hpp>

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
  
  /* 
  *  Visualization functions
  */
  void publishTree();

private:
  ros::Publisher pub_random_point_;
  ros::Publisher pub_random_pose_;
  ros::Publisher pub_viz_tree_;
  Node root_;
  std::list<Node> tree_;
  // Parameters
  double root_x_;
  double root_y_;
  double root_z_;
  double x_range_min_;
  double x_range_max_;
  double y_range_min_;
  double y_range_max_;
  double z_range_min_;
  double z_range_max_;
};
