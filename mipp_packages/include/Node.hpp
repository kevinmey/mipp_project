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

  Node(double x = 0, double y = 0, double z = 0, double yaw = 0, double cost = 0, double gain = 0, int id = 0, Node* parent = NULL, int rank = 0, bool is_goal = false);

  Node* getParent();
  void setParent(Node* node);

  void operator=(Node p);
  bool operator==(Node p);
  bool operator!=(Node p);
  bool operator<(Node p);
};
