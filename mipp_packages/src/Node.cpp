/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(std::weak_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, 
       double x = 0, double y = 0, double z = 0, double yaw = 0, 
       double cost = 0, double gain = 0, int id = 0, int rank = 0) 
{
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

Node::Node(std::weak_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, 
       geometry_msgs::Point position = makePoint(0,0,0), double yaw = 0, 
       double cost = 0, double gain = 0, int id = 0, int rank = 0)
{
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
  ROS_INFO("Destructing node %d.", id_);
  if(parent_){
    ROS_INFO("Node %d has no parent, probably root.", id_);
  } else {
    ROS_INFO("Node %d parent: %d", id_, parent_->id_);
  }
  for(std::vector<std::shared_ptr<Node>>::iterator child_itr = children_.begin();
      child_itr != children_.end(); child_itr++){
    ROS_INFO("Node %d child: %d", id_, (*child_itr)->id_);
  }
}

double Node::getDistanceToNode(Node const &node) 
{
	double distance = (double)sqrt( pow(position_.x - node.position_.x, 2.0)
																+ pow(position_.y - node.position_.y, 2.0)
																+ pow(position_.z - node.position_.z, 2.0));
	return distance;
}

std::weak_ptr<Node> Node::getParent()
{
  return this->parent_;
}

void Node::setParent(std::weak_ptr<Node> node)
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