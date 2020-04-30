/**
* @file RRTPlanner.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Contains the RRTPlanner class
*/

#include <RRTPlanner.hpp>

// Constructor
  
RRTPlanner::RRTPlanner(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_INFO("RRTPlanner object is being created.");

  getParams(np);

  pub_random_point_ = n.advertise<geometry_msgs::Point>("rrt_planner/random_point", 10);
  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_tree", 10);

  // Init. random number generator distributions
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);

  // Init. tree with root node
  root_ = Node(root_x_, root_y_, root_z_, 0.0, 0.0, 0.0, 0, NULL);
  tree_.push_back(root_);

  ros::Rate rate(20.0);
  while(ros::ok)
  {
    geometry_msgs::Point sample_point = generateRandomPoint(false);

    Node* nearest_neighbor = &root_;
    double nearest_neighbor_distance = distanceBetweenPoints(sample_point, root_.position_);

    for(std::list<Node>::iterator tree_node_it = tree_.begin();
        tree_node_it != tree_.end(); tree_node_it++)
    {
      double distance_to_node = distanceBetweenPoints(sample_point, tree_node_it->position_);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_it->id_, distance_to_node);

      if(distance_to_node < nearest_neighbor_distance) 
      {
        ROS_DEBUG("Sampled node is closer to node %d: %f", tree_node_it->id_, distance_to_node);
        nearest_neighbor = &*tree_node_it;
        ROS_DEBUG("New nearest neighbor: %d.", nearest_neighbor->id_);
        nearest_neighbor_distance = distance_to_node;
      }
    }

    geometry_msgs::Point new_point_origin = nearest_neighbor->position_;
    geometry_msgs::Vector3 new_point_direction = getDirection(new_point_origin, sample_point);
    geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                             new_point_direction, 
                                             std::min(nearest_neighbor_distance, max_ray_distance_));

    int node_id = tree_.size();
    Node new_node(new_point.x, new_point.y, new_point.z, 0.0, 0.0, 0.0, node_id, nearest_neighbor);
    tree_.push_back(new_node);

    visualizeTree();

    ros::spinOnce();
    rate.sleep();
  }
}

// Destructor
  
RRTPlanner::~RRTPlanner()
{
  ROS_INFO("RRTPlanner object is being deleted.");
}

// Utility functions

void RRTPlanner::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("RRTPlanner: getParams");
  np.param<std::string>("planner_world_frame", planner_world_frame_, "world");
  np.param<double>("root_x", root_x_, 0.0);
  np.param<double>("root_y", root_y_, 0.0);
  np.param<double>("root_z", root_z_, 0.0);
  np.param<double>("x_range_min", x_range_min_, -1.0);
  np.param<double>("x_range_max", x_range_max_,  1.0);
  np.param<double>("y_range_min", y_range_min_, -1.0);
  np.param<double>("y_range_max", y_range_max_,  1.0);
  np.param<double>("z_range_min", z_range_min_,  0.0);
  np.param<double>("z_range_max", z_range_max_,  1.0);
  np.param<double>("max_ray_distance", max_ray_distance_, 3.0);
}

geometry_msgs::Point RRTPlanner::generateRandomPoint(bool publish_point)
{
  ROS_DEBUG("RRTPlanner: generateRandomPoint");
  geometry_msgs::Point sample_point;
  sample_point.x = x_distribution_(generator_);
  sample_point.y = y_distribution_(generator_);
  sample_point.z = z_distribution_(generator_);
  if(publish_point)
  {
    pub_random_point_.publish(sample_point);
  }
  return sample_point;
}

/* 
*  Visualization functions
*/
void RRTPlanner::visualizeTree()
{
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 0.4;
  tree_marker.color.g = 0.4;
  tree_marker.color.b = 0.9;
  for(Node tree_node : tree_)
  {
    if(tree_node.getParent() != NULL) 
    {
      tree_marker.points.push_back(tree_node.position_);
      tree_marker.points.push_back(tree_node.getParent()->position_);
    }
  }
  pub_viz_tree_.publish(tree_marker);
}