/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(double x, double y, double z, double yaw, double cost, double gain, int id, Node* parent, int rank) {
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
	double distance = (double)sqrt( pow(position_.x - node.position_.x, 2.0)
																+ pow(position_.y - node.position_.y, 2.0)
																+ pow(position_.z - node.position_.z, 2.0));
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
  tmp.position_.x = this->position_.x + p.position_.x;
  tmp.position_.y = this->position_.y + p.position_.y;
  tmp.position_.z = this->position_.z + p.position_.z;
  tmp.yaw_ = this->yaw_ + p.yaw_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}

Node Node::operator-(Node p){
  Node tmp;
  tmp.position_.x = this->position_.x - p.position_.x;
  tmp.position_.y = this->position_.y - p.position_.y;
  tmp.position_.z = this->position_.z - p.position_.z;
  tmp.yaw_ = this->yaw_ - p.yaw_;
  tmp.cost_ = this->cost_ - p.cost_;
  tmp.gain_ = this->gain_ - p.gain_;
  return tmp;
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