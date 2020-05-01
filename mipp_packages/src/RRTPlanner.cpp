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

  pub_random_point_ = n.advertise<geometry_msgs::Point>("rrt_planner/random_point", 1);
  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_tree", 1);
  pub_viz_collision_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_collision_tree", 1);

  received_map_ = false;
  sub_octomap_ = n.subscribe("octomap_binary", 1, &RRTPlanner::subOctomap, this);

  // Init. random number generator distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);

  // Init. tree with root node
  root_ = Node(root_x_, root_y_, root_z_, 0.0, 0.0, 0.0, 0, NULL, 0);
  tree_.push_back(root_);

  /*
  * TEMPORARY
  * Testing the RRT functionality by running it in a while loop
  */
  ros::Rate rate(1.0);
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

    geometry_msgs::Point new_point_ray_origin = nearest_neighbor->position_;
    geometry_msgs::Vector3 new_point_ray_direction = getDirection(new_point_ray_origin, sample_point);
    geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                             new_point_ray_direction, 
                                             std::min(nearest_neighbor_distance, max_ray_distance_));
    geometry_msgs::Vector3 new_point_return_direction = getDirection(new_point, new_point_ray_origin);

    while(!received_map_)
    {
      ros::spinOnce();
      rate.sleep();
    }
    // Attempt to cast OctoMap ray
    octomap::point3d om_ray_origin = octomap::point3d(new_point_ray_origin.x, new_point_ray_origin.y, new_point_ray_origin.z);
    octomap::point3d om_ray_direction = octomap::point3d(new_point_ray_direction.x, new_point_ray_direction.y, new_point_ray_direction.z);
    octomap::point3d om_ray_end = octomap::point3d(new_point.x, new_point.y, new_point.z);
    octomap::point3d om_ray_return_direction = octomap::point3d(new_point_return_direction.x, new_point_return_direction.y, new_point_return_direction.z);
    octomap::point3d om_ray_end_cell;
    octomap::point3d om_ray_return_cell;
    double om_ray_distance = std::min(nearest_neighbor_distance, max_ray_distance_);
    bool hit_occupied_to = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, om_ray_distance);
    bool hit_occupied_from = map_->castRay(om_ray_end, om_ray_return_direction, om_ray_return_cell, false, om_ray_distance);
    if(hit_occupied_to or hit_occupied_from)
    {
      ROS_DEBUG("I hit something");
      collision_tree_.push_back(new_point_ray_origin);
      collision_tree_.push_back(new_point);
    }
    else
    {
      int node_id = tree_.size();
      int node_rank = nearest_neighbor->rank_ + 1;
      float node_cost = nearest_neighbor->cost_ + distanceBetweenPoints(new_point, nearest_neighbor->position_);
      Node new_node(new_point.x, new_point.y, new_point.z, 0.0, node_cost, 0.0, node_id, nearest_neighbor, node_rank);
      tree_.push_back(new_node);
      ROS_DEBUG("New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, nearest_neighbor->id_, node_rank, node_cost);
    }

    visualizeTree();
    visualizeCollisionTree();

    ros::spinOnce();
    rate.sleep();
  }
}

// Destructor
  
RRTPlanner::~RRTPlanner()
{
  ROS_INFO("RRTPlanner object is being deleted.");
}

void RRTPlanner::subOctomap(const octomap_msgs::Octomap& octomap_msg)
{
  ROS_INFO("RRTPlanner: subOctomap");
  octomap::AbstractOcTree* abstract_map = octomap_msgs::binaryMsgToMap(octomap_msg);
  if(abstract_map)
  {
    map_ = dynamic_cast<octomap::OcTree*>(abstract_map);
    received_map_ = true;
  } 
  else 
  {
    ROS_ERROR("RRTPlanner: Error creating octree from received message");
  }
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

void RRTPlanner::visualizeCollisionTree()
{
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.04;
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree_;
  pub_viz_collision_tree_.publish(tree_marker);
}