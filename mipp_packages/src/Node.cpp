/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(double x, double y, double z, double yaw, double cost, double gain, int id, Node* parent) {
  this->x_ = x;
  this->y_ = y;
  this->z_ = z;
  this->yaw_ = yaw;
  this->cost_ = cost;
  this->gain_ = gain;
  this->id_ = id;
  this->parent_ = parent;
}

std::string Node::printStatus(){
  std::stringstream ss;
  ss << "--------------" << std::endl;
  ss << "Node ID: " << id_ << std::endl;
  ss << "Gain:    " << gain_ << std::endl;
  ss << "Parent:  " << parent_->id_ << std::endl;
  ss << "--------------" << std::endl;
  return ss.str();
}

std::stringstream Node::stringPosVec(std::vector<double> vec) {
  std::stringstream ss;
  if (!vec.empty()) {
    ss << "(" << vec[0] << ", " << vec[1] << ", " << vec[2] << "; " << vec[3] << ")" << std::endl;
  }
  else {
    ss << std::endl;
  }
  return ss;
}

double Node::findDistance(Node const &node) {
	double distance = (double)sqrt( pow((double)x_ - (double)node.x_, 2.0)
																+ pow((double)y_ - (double)node.y_, 2.0)
																+ pow((double)z_ - (double)node.z_, 2.0));
	return distance;
}

Node* Node::getParent()
{
  return this->parent_;
}

void Node::setParent(Node* node)
{
  this->parent_ = node;
}

Node Node::operator+(Node p){
  Node tmp;
  tmp.x_ = this->x_ + p.x_;
  tmp.y_ = this->y_ + p.y_;
  tmp.z_ = this->z_ + p.z_;
  tmp.yaw_ = this->yaw_ + p.yaw_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}

Node Node::operator-(Node p){
  Node tmp;
  tmp.x_ = this->x_ - p.x_;
  tmp.y_ = this->y_ - p.y_;
  tmp.z_ = this->z_ - p.z_;
  tmp.yaw_ = this->yaw_ - p.yaw_;
  tmp.cost_ = this->cost_ - p.cost_;
  tmp.gain_ = this->gain_ - p.gain_;
  return tmp;
}

void Node::operator=(Node p){
  this->x_ = p.x_;
  this->y_ = p.y_;
  this->z_ = p.z_;
  this->yaw_ = p.yaw_;
  this->cost_ = p.cost_;
  this->gain_ = p.gain_;
  this->id_ = p.id_;
  this->parent_ = p.parent_;
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