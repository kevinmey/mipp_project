/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(double x, double y, double z, double yaw, double cost, double gain, int id, std::shared_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, int rank) {
  this->position_.x = x;
  this->position_.y = y;
  this->position_.z = z;
  this->yaw_ = yaw;
  this->cost_ = cost;
  this->gain_ = gain;
  this->id_ = id;
  this->parent_ = parent;
  this->rank_ = rank;

}

Node::Node(geometry_msgs::Point position, double yaw, double cost, double gain, int id, std::shared_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, int rank) {
  this->position_ = position;
  this->yaw_ = yaw;
  this->cost_ = cost;
  this->gain_ = gain;
  this->id_ = id;
  this->parent_ = parent;
  this->rank_ = rank;

}

Node::~Node()
{
  ROS_INFO("Node %d is being destroyed.", id_);
}

double Node::getDistanceToNode(Node const &node) 
{
	double distance = (double)sqrt( pow(position_.x - node.position_.x, 2.0)
																+ pow(position_.y - node.position_.y, 2.0)
																+ pow(position_.z - node.position_.z, 2.0));
	return distance;
}

std::shared_ptr<Node> Node::getParent()
{
  return this->parent_;
}

void Node::setParent(std::shared_ptr<Node> node)
{
  this->parent_ = node;
}

std::vector<std::shared_ptr<Node>> Node::getChildren()
{
  return children_;
}

void Node::setChildren(std::vector<std::shared_ptr<Node>> nodes)
{
  children_.clear();
  for(std::vector<std::shared_ptr<Node>>::iterator nodes_itr = nodes.begin();
      nodes_itr != nodes.end(); nodes_itr++){
    
    children_.push_back(*nodes_itr);
  }
}

void Node::addChild(std::shared_ptr<Node> node)
{

}



void Node::operator=(Node p){
  this->position_.x = p.position_.x;
  this->position_.y = p.position_.y;
  this->position_.z = p.position_.z;
  this->yaw_ = p.yaw_;
  this->cost_ = p.cost_;
  this->gain_ = p.gain_;
  this->id_ = p.id_;
  this->parent_ = p.parent_;
  this->rank_ = p.rank_;
}

bool Node::operator==(Node p){
    if (this->id_ == p.id_) return true;
    return false;
}

bool Node::operator!=(Node p){
    if (this->id_ != p.id_) return true;
    return false;
}

bool Node::operator<(Node p){
    return (this->cost_ < p.cost_);
}