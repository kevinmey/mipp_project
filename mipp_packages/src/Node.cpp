/**
* @file Node.cpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include "Node.hpp"

Node::Node(double x, double y, double z, double yaw, double cost, double gain, std::vector<double> parent) {
    this->x_ = x;
    this->y_ = y;
    this->z_ = z;
    this->yaw_ = yaw;
    this->cost_ = cost;
    this->gain_ = gain;
    this->id_ = { x, y, z, yaw };
    this->parent_ = parent;
}

std::stringstream Node::printStatus(){
    std::stringstream ss;
    ss << "--------------" << std::endl;
    ss << "This:   " << this->stringPosVec(this->id_).str();
    ss << "Gain:   " << this->gain_ << std::endl;
    ss << "Parent: " << this->stringPosVec(this->parent_).str();
    ss << "--------------" << std::endl;
    //std::cout << ss.str();
    return ss;
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

double Node::findDistance(Node node) {
    double distance = (double)sqrt((((double)this->x_ - (double)node.x_) * ((double)this->x_ - (double)node.x_))
        + (((double)this->y_ - (double)node.y_) * ((double)this->y_ - (double)node.y_))
        + (((double)this->z_ - (double)node.z_) * ((double)this->z_ - (double)node.z_)));
    return distance;
}


Node Node::operator+(Node p){
    Node tmp;
    tmp.x_ = this->x_ + p.x_;
    tmp.y_ = this->y_ + p.y_;
    tmp.z_ = this->z_ + p.z_;
    tmp.yaw_ = this->yaw_ + p.yaw_;
    tmp.cost_ = this->cost_ + p.cost_;
    tmp.id_ = { this->id_[0] + p.id_[0], this->id_[1] + p.id_[1], this->id_[2] + p.id_[2], p.id_[3]};
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
    tmp.id_ = { this->id_[0] - p.id_[0], this->id_[1] - p.id_[1], this->id_[2] - p.id_[2], p.id_[3] };
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