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
  pub_viz_path_to_goal_ = n.advertise<visualization_msgs::Marker>("rrt_planner/path_to_goal", 1);

  received_map_ = false;
  sub_octomap_ = n.subscribe("octomap_binary", 1, &RRTPlanner::subOctomap, this);

  // Init. random number generator distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);
  goal_sample_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Init. tree with root node
  root_ = Node(root_x_, root_y_, root_z_, 0.0, 0.0, 0.0, 0, NULL, 0);
  tree_.push_back(root_);

  /*
  * TEMPORARY
  * Testing the RRT functionality by running it in a while loop
  */
  ros::Rate rate(planner_rate_);
  while(tree_.size() < planner_max_tree_nodes_)
  {
    while(!received_map_)
    {
      ROS_WARN("RRTPlanner: No map received yet, waiting...");
      ros::spinOnce();
      rate.sleep();
    }
    
    geometry_msgs::Point sample_point;
    if(goal_sample_distribution_(generator_) < goal_sample_probability_)
    {
      sample_point.x = goal_x_;
      sample_point.y = goal_y_;
      sample_point.z = goal_z_;
    }
    else
    {
      // Sample a random point within the (x,y,z)_distribution bounds
      sample_point = generateRandomPoint(false);
    }

    // Start finding nearest node in the tree to the sampled point (init. as root)
    Node* nearest_neighbor = &root_;
    double nearest_neighbor_distance = getDistanceBetweenPoints(sample_point, root_.position_);

    // Iterate through tree nodes, checking distance to sampled point
    for(std::list<Node>::iterator tree_node_itr = tree_.begin();
        tree_node_itr != tree_.end(); tree_node_itr++)
    {
      // Get distance to tree node
      double distance_to_node = getDistanceBetweenPoints(sample_point, tree_node_itr->position_);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_itr->id_, distance_to_node);

      // Make tree node new nearest neighbor if it is closer than current nearest neighbor
      if(distance_to_node < nearest_neighbor_distance) 
      {
        ROS_DEBUG("Sampled node is closer to node %d: %f", tree_node_itr->id_, distance_to_node);
        nearest_neighbor = &*tree_node_itr;
        nearest_neighbor_distance = distance_to_node;
      }
    }

    /* Get the coordinates of the new point
    * Cast a ray from the nearest neighbor in the direction of the sampled point.
    * If nearest neighbor is within max_ray_distance of sampled point, sampled point is new point.
    * Else the endpoint of the ray (with length max_ray_distance) is new point
    */
    geometry_msgs::Point new_point_ray_origin = nearest_neighbor->position_;
    geometry_msgs::Vector3 new_point_ray_direction = getDirection(new_point_ray_origin, sample_point);
    geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                             new_point_ray_direction, 
                                             std::min(nearest_neighbor_distance, max_ray_distance_));

    switch(planner_algorithm_)
    {
      case 1:
        // RRTstar algorithm
        extendTreeRRTstar(new_point);
        break;
      case 2:
        // RRT algorithm
        extendTreeRRT(new_point, nearest_neighbor);
        break;
      default:
        // RRTstar algorithm as default
        extendTreeRRTstar(new_point);
    }

    visualizeTree();
    visualizePathToGoal();

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

/* 
*  Utility functions
*/

void RRTPlanner::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("RRTPlanner: getParams");
  np.param<std::string>("planner_world_frame", planner_world_frame_, "world");
  np.param<double>("root_x", root_x_, 0.0);
  np.param<double>("root_y", root_y_, 0.0);
  np.param<double>("root_z", root_z_, 0.0);
  np.param<double>("goal_x", goal_x_, 0.0);
  np.param<double>("goal_y", goal_y_, 0.0);
  np.param<double>("goal_z", goal_z_, 2.0);
  np.param<double>("goal_sample_probability", goal_sample_probability_, 0.1);
  np.param<double>("goal_radius", goal_radius_, 0.01);
  np.param<double>("x_range_min", x_range_min_, -1.0);
  np.param<double>("x_range_max", x_range_max_,  1.0);
  np.param<double>("y_range_min", y_range_min_, -1.0);
  np.param<double>("y_range_max", y_range_max_,  1.0);
  np.param<double>("z_range_min", z_range_min_,  0.0);
  np.param<double>("z_range_max", z_range_max_,  1.0);
  np.param<double>("planner_rate", planner_rate_, 10.0);
  np.param<int>("planner_algorithm", planner_algorithm_, 1);
  np.param<int>("planner_max_tree_nodes", planner_max_tree_nodes_, 1000);
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

void RRTPlanner::extendTreeRRTstar(geometry_msgs::Point candidate_point)
{
  ROS_DEBUG("RRTPlanner: extendTreeRRTstar");

  // Make a neighbor list of nodes close enough to the candidate point
  // Store combined cost and Node pointer in map
  std::map<double, Node*> neighbor_list;
  for(std::list<Node>::iterator tree_node_itr = tree_.begin();
      tree_node_itr != tree_.end(); tree_node_itr++)
  {
    // Get distance to tree node
    double distance_to_node = getDistanceBetweenPoints(candidate_point, tree_node_itr->position_);
    ROS_DEBUG("Sampled point distance to node %d: %f", tree_node_itr->id_, distance_to_node);

    // Add tree node to neighborhood if it is within range
    // Inserted into map with combined cost (tree node cost + distance to node) as key
    if(distance_to_node < max_ray_distance_+0.01) 
    {
      ROS_DEBUG("Node %d is within neighborhood", tree_node_itr->id_);
      double neighbor_cost = tree_node_itr->cost_;
      double combined_cost = neighbor_cost + distance_to_node;
      neighbor_list.insert(std::pair<double, Node*>(combined_cost, &*tree_node_itr));
    }
  }
  ROS_DEBUG("Neighborhood size: %d", (int)neighbor_list.size());
  
  // Function to print neighbors
  std::map<double, Node*>::iterator neighbor_itr;
  ROS_DEBUG("\tNODE\tCOST"); 
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) { 
    ROS_DEBUG("\t%d\t%f", neighbor_itr->second->id_, neighbor_itr->first);
  }

  // Add the new node if it is collision free
  std::vector<geometry_msgs::Point> collision_tree;
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) 
  { 
    bool pathIsCollisionFree = isPathCollisionFree(neighbor_itr->second->position_, candidate_point);
    if(pathIsCollisionFree)
    {
      int node_id = tree_.size();
      int node_rank = neighbor_itr->second->rank_ + 1;
      float node_cost = neighbor_itr->second->cost_ + getDistanceBetweenPoints(candidate_point, neighbor_itr->second->position_);
      bool node_is_goal = (getDistanceBetweenPoints(candidate_point, makePoint(goal_x_, goal_y_, goal_z_)) < goal_radius_);
      Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, 0.0, node_cost, 0.0, node_id, neighbor_itr->second, node_rank, node_is_goal);
      if(node_is_goal)
      {
        goal_nodes_.insert(std::pair<double, Node>(node_cost, new_node));
        ROS_DEBUG("RRTPlanner: New goal node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      }
      else
      {
        tree_.push_back(new_node);
        ROS_DEBUG("RRTPlanner: New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      }
      
      break;
    }
    else
    {
      ROS_DEBUG("I hit something");
      collision_tree.push_back(neighbor_itr->second->position_);
      collision_tree.push_back(candidate_point);
    }
  }

  visualizeCollisionTree(collision_tree);
}

void RRTPlanner::extendTreeRRT(geometry_msgs::Point candidate_point, Node* nearest_neighbor)
{
  ROS_DEBUG("RRTPlanner: extendTreeRRT");

  // Add the new node if it is collision free
  bool pathIsCollisionFree = isPathCollisionFree(nearest_neighbor->position_, candidate_point);
  if(pathIsCollisionFree)
  {
    int node_id = tree_.size();
    int node_rank = nearest_neighbor->rank_ + 1;
    float node_cost = nearest_neighbor->cost_ + getDistanceBetweenPoints(candidate_point, nearest_neighbor->position_);
    Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, 0.0, node_cost, 0.0, node_id, nearest_neighbor, node_rank);
    tree_.push_back(new_node);
    ROS_DEBUG("RRTPlanner: New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, nearest_neighbor->id_, node_rank, node_cost);

  }
  else
  {
    ROS_DEBUG("I hit something");
    collision_tree_.push_back(nearest_neighbor->position_);
    collision_tree_.push_back(candidate_point);   
  }
  
  visualizeCollisionTree(collision_tree_);
}

bool RRTPlanner::isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point point_b, 
                                     geometry_msgs::Vector3 direction_ab, double distance)
{
  ROS_DEBUG("RRTPlanner: isPathCollisionFree");
  // Check if optional arguments are set (default/initialized values aren't valid)
  if((int)direction_ab.x == 0 and (int)direction_ab.y == 0 and (int)direction_ab.z == 0)
  {
    direction_ab = getDirection(point_a, point_b);
  }
  if(distance == -1.0)
  {
    distance = getDistanceBetweenPoints(point_a, point_b);
  }

  // Attempt to cast OctoMap ray
  octomap::point3d om_ray_origin = octomap::point3d(point_a.x, point_a.y, point_a.z);
  octomap::point3d om_ray_direction = octomap::point3d(direction_ab.x, direction_ab.y, direction_ab.z);
  octomap::point3d om_ray_end = octomap::point3d(point_b.x, point_b.y, point_b.z);
  octomap::point3d om_ray_return_direction = octomap::point3d(-direction_ab.x, -direction_ab.y, -direction_ab.z);
  octomap::point3d om_ray_end_cell;
  octomap::point3d om_ray_return_cell;
  double om_ray_distance = std::min(distance, max_ray_distance_);
  bool hit_occupied_to = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, om_ray_distance);
  bool hit_occupied_from = map_->castRay(om_ray_end, om_ray_return_direction, om_ray_return_cell, false, om_ray_distance);

  // Return whether there was a collision or not
  return !(hit_occupied_to or hit_occupied_from);
}