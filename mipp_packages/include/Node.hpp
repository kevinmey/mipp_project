/**
* @file Node.hpp
* @author Kevinmey
* @brief Contains the Node struct
*/

#include <vector>

struct Node{
private:
public:
  int id_;
  int parent_id_;
  std::vector<int> child_ids_;

  int rank_;
  float cost_;

  float x_;
  float y_;
  float z_;
  float yaw_;

  Node(int id, int parent_id, std::vector<int> child_ids,
       int rank, float cost,
       float x, float y, float z, float yaw){
    id_ = id;
    parent_id_ = parent_id;
    child_ids_ = child_ids;
    rank_ = rank;
    cost_ = cost;
    x_ = x;
    y_ = y;
    z_ = z;
    yaw_ = yaw;
  }
};