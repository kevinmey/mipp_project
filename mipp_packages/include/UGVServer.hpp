#include <ros/ros.h>

#include <utils.hpp>

#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
 
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

  ros::Subscriber sub_clicked_point_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_map_update_;;
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_; 

  bool map_initialized_;
  bool map_in_use_;
  nav_msgs::OccupancyGrid map_;
  // Params
  int collision_threshold_;
  bool unmapped_is_collision_;
};
