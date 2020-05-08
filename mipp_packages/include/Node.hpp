/**
* @file Node.hpp
* @author Kevinmey
* @brief Contains the Node struct
*/

#include <geometry_msgs/Point.h>
#include <vector>

struct Node{
private:
public:
  int id_;
  int parent_id_;
  std::vector<int> child_ids_;

  int rank_;
  double cost_;

  geometry_msgs::Point position_;
  double yaw_;

  Node(int id = -1, int parent_id = -1, std::vector<int> child_ids = {},
       int rank = -1, double cost = -1.0,
       double x = 0, double y = 0, double z = 0, double yaw = 0){
    id_ = id;
    parent_id_ = parent_id;
    child_ids_ = child_ids;
    rank_ = rank;
    cost_ = cost;
    position_.x = x;
    position_.y = y;
    position_.z = z;
    yaw_ = yaw;
  }

};