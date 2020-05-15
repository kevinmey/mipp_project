#include <UGVServer.hpp>

// Constructor
  
UGVServer::UGVServer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("UGVServer object is being created.");

  getParams(np);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  map_initialized_ = false;
  sub_clicked_point_ = n.subscribe("/clicked_point", 1, &UGVServer::subClickedPoint, this);
  sub_map_ = n.subscribe("/move_base/global_costmap/costmap", 1, &UGVServer::subMap, this);
  sub_map_update_ = n.subscribe("/move_base/global_costmap/costmap_updates", 1, &UGVServer::subMapUpdate, this);
  ros::Rate wait_rate(1.0);
  while (!map_initialized_) {
    ros::spinOnce();
    wait_rate.sleep();
  }

  ROS_INFO("UGVServer: Initialization done.");
}

// Destructor
  
UGVServer::~UGVServer()
{
  ROS_INFO("UGVServer object is being deleted.");

}

/* 
*  Subscriber callbacks
*/

void UGVServer::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg)
{
  ROS_DEBUG("UGVServer: subClickedPoint");
  int mx;
  int my;
  double wx = clicked_point_msg->point.x;
  double wy = clicked_point_msg->point.y;
  convWorldToMap(wx, wy, mx, my);
  ROS_INFO("World (%f, %f) = Map (%d, %d)",wx,wy,mx,my);
  
  int gridCost = map_.data[getGridIndex(wx, wy)];
  ROS_INFO("Cost %d, is collision: %d", gridCost, (int)!isPositionCollisionFree(wx, wy));
}

void UGVServer::subMap(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
  ROS_DEBUG("UGVServer: subMap");
  // If map in use by others, have to wait until they release it
  ros::Rate wait_rate(5.0);
  if (map_in_use_) {
    ros::spinOnce();
    wait_rate.sleep();
  }
  
  // Lock map from being operated on by others
  map_in_use_ = true;

  // Set map metadata
  map_.header = map_msg->header;
  map_.info = map_msg->info;

  // Set map data
  map_.data = map_msg->data;
  ROS_DEBUG("UGVServer: Map size %d", (int)map_.data.size());

  // Release lock
  map_in_use_ = false;

  // Map is now initialized
  if (!map_initialized_) { map_initialized_ = true; }
}

void UGVServer::subMapUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map_msg)
{
  ROS_DEBUG("UGVServer: subMapUpdate");
  // If map in use by others, have to wait until they release it
  ros::Rate wait_rate(5.0);
  if (map_in_use_) {
    ros::spinOnce();
    wait_rate.sleep();
  }
  
  // Lock map from being operated on by others
  map_in_use_ = true;

  // Set map data
  map_.data = map_msg->data;
  ROS_DEBUG("UGVServer: Map size %d", (int)map_.data.size());

  // Release lock
  map_in_use_ = false;
}

/* 
*  Utility functions
*/

void UGVServer::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("UGVServer: getParams");
  np.param<int>("collision_threshold", collision_threshold_, 0);
  np.param<bool>("unmapped_is_collision", unmapped_is_collision_, false);
}

/* 
*  Map utility functions
*/

void UGVServer::convMapToWorld(int map_x, int map_y, double& world_x, double& world_y)
{
  ROS_DEBUG("UGVServer: convMapToWorld");
  world_x = (map_x + 0.5)*map_.info.resolution + map_.info.origin.position.x;
  world_y = (map_y + 0.5)*map_.info.resolution + map_.info.origin.position.y;
}

void UGVServer::convWorldToMap(double world_x, double world_y, int& map_x, int& map_y)
{
  ROS_DEBUG("UGVServer: convWorldToMap");

  if (world_x < map_.info.origin.position.x) {
    map_x = 0;
  } 
  else if (world_x >= map_.info.resolution*map_.info.width + map_.info.origin.position.x) {
    map_x = map_.info.width - 1;
  }
  else {
    map_x = (int)((world_x - map_.info.origin.position.x)/map_.info.resolution);
  }

  if (world_y < map_.info.origin.position.y) {
    map_y = 0;
  } 
  else if (world_y >= map_.info.resolution*map_.info.height + map_.info.origin.position.y) {
    map_y = map_.info.height - 1;
  }
  else {
    map_y = (int)((world_y - map_.info.origin.position.y)/map_.info.resolution);
  }
}

int UGVServer::getGridIndex(int map_x, int map_y)
{
  ROS_DEBUG("UGVServer: getGridIndex");
  return map_y*map_.info.width + map_x;
}

int UGVServer::getGridIndex(double world_x, double world_y)
{
  ROS_DEBUG("UGVServer: getGridIndex");
  int map_x, map_y;
  convWorldToMap(world_x, world_y, map_x, map_y);
  return map_y*map_.info.width + map_x;
}

bool UGVServer::isPathCollisionFree(double world_x_a, double world_y_a, double world_x_b, double world_y_b)
{
  ROS_DEBUG("UGVServer: isPathCollisionFree");

  geometry_msgs::Point point_a = makePoint(world_x_a, world_x_b, 0.0);
  geometry_msgs::Point point_b = makePoint(world_x_b, world_x_b, 0.0);
  geometry_msgs::Vector3 direction = getDirection(point_a, point_b);
  double distance = getDistanceBetweenPoints(point_a, point_b);
  double angle_ab = atan2(point_b.y - point_a.y, point_b.x - point_a.x);
  
  // Check first if point_b is collision free configuration
  if (isPositionCollisionFree(world_x_b, world_y_b)) {
    return false;
  }

  double distance_on_line = 0.0;
  geometry_msgs::Point point_on_line = point_a;
  while (distance_on_line < distance) {
    if (isPositionCollisionFree(point_on_line.x, point_on_line.y)) {
      return false;
    }
    distance_on_line += map_.info.resolution;
    point_on_line.x = point_on_line.x + map_.info.resolution*direction.x;
    point_on_line.y = point_on_line.y + map_.info.resolution*direction.y;
  }

  return true;
}

bool UGVServer::isPositionOutsideMap(double world_x, double world_y)
{
  ROS_DEBUG("UGVServer: isPositionOutsideMap");
  return (world_x < map_.info.origin.position.x or 
          world_x >= world_x >= map_.info.resolution*map_.info.width + map_.info.origin.position.x or
          world_y < map_.info.origin.position.y or 
          world_y >= world_y >= map_.info.resolution*map_.info.height + map_.info.origin.position.y);
}

bool UGVServer::isPositionCollisionFree(double world_x, double world_y)
{
  ROS_DEBUG("UGVServer: isPositionCollisionFree");
  int gridCost = map_.data[getGridIndex(world_x, world_y)];
  if (gridCost > collision_threshold_) {
    return false;
  }
  else if ((gridCost == -1 or isPositionOutsideMap(world_x, world_y)) and unmapped_is_collision_) {
    return false;
  }
  return true;
}

bool UGVServer::isPositionUnmapped(double world_x, double world_y)
{
  ROS_DEBUG("UGVServer: isPositionUnmapped");
  int gridCost = map_.data[getGridIndex(world_x, world_y)];
  return (gridCost == -1 or isPositionOutsideMap(world_x, world_y));
}