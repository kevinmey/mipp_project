/**
* @file Node.hpp
* @author Kevinmey, based on code from Sondreab and vss2sn
* @brief Contains the Node class
*/

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <iostream>
#include <string>
#include <sstream>
#include <iterator>
#include <iomanip> // setw
#include <queue>
#include <cmath>
#include <climits>
#include <unordered_map>
#include <unordered_set>
#include <random>

#include <utils.hpp>

/**
* @brief Node class
* @param position_ (x,y,z) coordinate of point
* @param yaw_ Yaw value
* @param cost_ Cost to get to this node
* @param id_ Node's id
* @param parent_ Node's parent
* @param children_ Node's children
* @param rank_ Nr. of nodes away from root
*/
class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
private:
  /** \brief Node's parent */
  std::weak_ptr<Node> parent_;
  
  /** \brief Node's children */
  std::vector<std::shared_ptr<Node>> children_;

public:
  /** \brief Point coordinate */
  geometry_msgs::Point position_;
  /** \brief yaw angle */
  double yaw_;
  /** \brief cost to reach this node */
  double cost_;
  /** \brief exploration gain after reaching this node */
  double gain_;
  /** \brief Node's id */
  int id_;
  /** \brief Node rank, nr. of nodes away from root */
  int rank_;

  /**
  * @brief Constructor for Node class
  * @param x    X value
  * @param y    Y value
  * @param z    Z value
  * @param yaw  Yaw value
  * @param cost Cost to get to this node (usually distance to root)
  * @param id Node's id
  * @param parent_ Node's parent
  * @param children_ Node's children
  * @param rank_ Nr. of nodes away from root
  */
  Node(std::weak_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, 
       double x = 0, double y = 0, double z = 0, double yaw = 0, 
       double cost = 0, double gain = 0, int id = 0, int rank = 0);

  /**
  * @brief Constructor for Node class
  * @param position Position value
  * @param yaw  Yaw value
  * @param cost Cost to get to this node (usually distance to root)
  * @param id Node's id
  * @param parent_ Node's parent
  * @param children_ Node's children
  * @param rank_ Nr. of nodes away from root
  */
  Node(std::weak_ptr<Node> parent, std::vector<std::shared_ptr<Node>> children, 
       geometry_msgs::Point position = makePoint(0,0,0), double yaw = 0, 
       double cost = 0, double gain = 0, int id = 0, int rank = 0);

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  double getDistanceToNode(Node const &node);

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  std::weak_ptr<Node> getParent();

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  void setParent(std::weak_ptr<Node> node);

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  std::vector<std::shared_ptr<Node>> getChildren();

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  void setChildren(std::vector<std::shared_ptr<Node>> nodes);

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  void addChild(std::shared_ptr<Node> node);

  /**
  * @brief Overloading operator = for Node class
  * @param p node
  * @return void
  */
  void operator=(Node p);

  /**
  * @brief Overloading operator == for Node class
  * @param p node
  * @return bool whether current node equals input node
  */
  bool operator==(Node p);

  /**
  * @brief Overloading operator != for Node class
  * @param p node
  * @return bool whether current node is not equal to input node
  */
  bool operator!=(Node p);

  /**
  * @brief Overloading operator < for Node class
  * @param p node
  * @return bool whether current node cost is less than input node cost
  */
  bool operator<(Node p);
};
