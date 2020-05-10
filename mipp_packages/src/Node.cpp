/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(double x, double y, double z, double yaw, double cost, double gain, int id, Node* parent, int rank, bool is_goal) {
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

Node* Node::getParent()
{
  return this->parent_;
}

void Node::setParent(Node* node)
{
  this->parent_ = node;
}

void Node::operator=(Node p){
  this->position_ = p.position_;
  this->yaw_ = p.yaw_;
  this->cost_ = p.cost_;
  this->gain_ = p.gain_;
  this->id_ = p.id_;
  this->parent_ = p.parent_;
  this->rank_ = p.rank_;
  this->is_goal_ = p.is_goal_;
}

bool Node::operator==(Node p)
{
    return (this->id_ == p.id_);
}

bool Node::operator!=(Node p)
{
    return (this->id_ != p.id_);
}

bool Node::operator<(Node p)
{
    return (this->cost_ < p.cost_);
}