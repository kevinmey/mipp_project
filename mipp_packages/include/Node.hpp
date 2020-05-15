/**
* @file Node.hpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include <geometry_msgs/Point.h>

class Node{
private:
  Node* parent_;
public:
  geometry_msgs::Point position_;
  double yaw_;
  double cost_;
  double gain_;
  int id_;
  int rank_;
  bool is_goal_;

  Node(double x = 0, double y = 0, double z = 0, double yaw = 0, double cost = 0, double gain = 0, int id = 0, Node* parent = NULL, int rank = 0, bool is_goal = false) {
    this->position_.x = x;
    this->position_.y = y;
    this->position_.z = z;
    this->yaw_ = yaw;
    this->cost_ = cost;
    this->gain_ = gain;
    this->id_ = id;
    this->parent_ = parent;
    this->rank_ = rank;
    this->is_goal_ = is_goal;
  }

  Node* getParent() { return this->parent_; }
  void setParent(Node* node) { this->parent_ = node; }

  void operator=(Node p) {
    this->position_ = p.position_;
    this->yaw_ = p.yaw_;
    this->cost_ = p.cost_;
    this->gain_ = p.gain_;
    this->id_ = p.id_;
    this->parent_ = p.parent_;
    this->rank_ = p.rank_;
    this->is_goal_ = p.is_goal_;
  }
  bool operator==(Node p) { return (this->id_ == p.id_); }
  bool operator!=(Node p) { return (this->id_ != p.id_); }
  bool operator<(Node p) { return (this->cost_ < p.cost_); }
};
