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

/**
* @brief Node class
* @param position (x,y,z) coordinate of point
* @param y_ Y value
* @param z_ Y value
* @param cost_ Cost to get to this node
* @param id_ Node's id
* @param parent_id_ Node's parent's id
*/
class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
private:
  /** \brief Node's parent */
  Node* parent_;
  /**
  * @brief Prints the position vector
  * @return void
  */
  std::stringstream stringPosVec(std::vector<double> vec);
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

  /**
  * @brief Constructor for Node class
  * @param x    X value
  * @param y    Y value
  * @param z    Z value
  * @param yaw  Yaw value
  * @param cost Cost to get to this node (usually distance to root)
  * @param id Node's id
  * @param parent_ Node's parent
  */
  Node(double x = 0, double y = 0, double z = 0, double yaw = 0, double cost = 0, double gain = 0, int id = 0, Node* parent = NULL);

  /**
  * @brief Prints the nodes id and parent id
  * @return void
  */
  std::string printStatus();

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  double findDistance(Node const &node);

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  Node* getParent();

  /**
  * @brief Prints the position vector
  * @param node the node to find the distance to
  * @return double distance between nodes
  */
  void setParent(Node* node);


  /**
  * @brief Overloading operator + for Node class
  * @param p node
  * @return Node with current node's and input node p's values added
  */
  Node operator+(Node p);

  /**
  * @brief Overloading operator - for Node class
  * @param p node
  * @return Node with current node's and input node p's values subtracted
  */
  Node operator-(Node p);

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
