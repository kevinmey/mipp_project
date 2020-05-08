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

  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_tree", 1);
  pub_viz_collision_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_collision_tree", 1);
  pub_viz_path_to_goal_ = n.advertise<visualization_msgs::Marker>("rrt_planner/path_to_goal", 1);
  pub_viz_root_node_ = n.advertise<visualization_msgs::MarkerArray>("rrt_planner/root_node", 1);
  pub_viz_goal_node_ = n.advertise<visualization_msgs::MarkerArray>("rrt_planner/goal_node", 1);

  received_map_ = false;
  sub_octomap_ = n.subscribe("octomap_binary", 1, &RRTPlanner::subOctomap, this);

  // Init. random number generator distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Init. tree with root node
  int root_id = 1;          // First root starts with ID 1
  int root_parent_id = 0;   // Parent ID of 0 signalizes root
  std::vector<int> root_child_ids = {};  // No children yet
  int root_rank = 0;
  double root_cost = 0;
  double root_yaw = 0;
  root_ = Node(root_id, root_parent_id, root_child_ids,
               root_rank, root_cost,
               root_x_, root_y_, root_z_, root_yaw);
  tree_.clear();
  tree_.insert(std::pair<int, Node>(root_id, root_));

  // Init. goal node
  int goal_id = -1;         // Not going to be a valid node in the tree
  int goal_parent_id = -1;   // No parents yet
  std::vector<int> goal_child_ids = {};  // No children (and will not have children)
  int goal_rank = 0;
  double goal_cost = 1000000;
  double goal_yaw = 0;
  goal_ = Node(goal_id, goal_parent_id, goal_child_ids,
               goal_rank, goal_cost,
               goal_x_, goal_y_, goal_z_, goal_yaw);

  // Wait for map
  ros::Rate rate_wait_map(1.0);
  while(!received_map_)
  {
    ROS_WARN("RRTPlanner: No map received yet, waiting...");
    ros::spinOnce();
    rate_wait_map.sleep();
  }

  /*
  * TEMPORARY
  * Testing the RRT functionality by running it in a while loop
  */
  ros::Rate rate(planner_rate_);
  ros::Time algorithm_start = ros::Time::now();
  while((ros::Time::now() - algorithm_start).toSec() < 5.0)
  {
    ros::Time begin = ros::Time::now();

    geometry_msgs::Point sample_point;
    bool sample_is_goal = false;
    if(unit_distribution_(generator_) < goal_sample_probability_)
    {
      ROS_DEBUG("RRTPlanner: Sampling goal point");
      sample_point = goal_.position_;
      sample_is_goal = true;
    }
    else
    {
      sample_point = generateRandomPoint(false);
      ROS_DEBUG("RRTPlanner: Sampled random point: (x,y,z) = (%f,%f,%f)",sample_point.x,sample_point.y,sample_point.z);
    }

    // Start finding nearest node in the tree to the sampled point (init. as root)
    int nearest_neighbor_id = root_.id_;
    double nearest_neighbor_distance = getDistanceBetweenPoints(sample_point, root_.position_);

    // Iterate through tree nodes, checking distance to sampled point
    for(std::map<int, Node>::iterator tree_node_itr = tree_.begin();
        tree_node_itr != tree_.end(); tree_node_itr++)
    {
      // Get distance to tree node
      double distance_to_node = getDistanceBetweenPoints(sample_point, tree_node_itr->second.position_);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_itr->second.id_, distance_to_node);

      // Make tree node new nearest neighbor if it is closer than current nearest neighbor
      if(distance_to_node < nearest_neighbor_distance) 
      {
        ROS_DEBUG("Sampled node is closer to node %d: %f", tree_node_itr->second.id_, distance_to_node);
        nearest_neighbor_id = tree_node_itr->second.id_;
        nearest_neighbor_distance = distance_to_node;
      }
    }

    /* Get the coordinates of the new point
    * Cast a ray from the nearest neighbor in the direction of the sampled point.
    * If nearest neighbor is within max_ray_distance of sampled point, sampled point is new point.
    * Else the endpoint of the ray (with length max_ray_distance) is new point
    */
    geometry_msgs::Point new_point_ray_origin = tree_.find(nearest_neighbor_id)->second.position_;
    geometry_msgs::Vector3 new_point_ray_direction = getDirection(new_point_ray_origin, sample_point);
    geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                             new_point_ray_direction, 
                                             std::min(nearest_neighbor_distance, max_ray_distance_));

    switch(planner_algorithm_)
    {
      case 1:
        // RRT algorithm
        extendTreeRRT(new_point, nearest_neighbor);
        break;
      case 2:
        // RRTstar algorithm
        extendTreeRRTstar(new_point);
        break;
      default:
        // RRTstar algorithm as default
        extendTreeRRTstar(new_point);
    }
    }
    goal_path_distance_ = goal_.cost_;

    visualizeTree();
    visualizePathToGoal();
    visualizeRoot(root_.position_, 0.0, 1.0, 0.0);
    visualizeGoal(goal_.position_, 1.0, 0.0, 0.0);

    algorithm_time += ros::Time::now() - begin;
    algorithm_iterations++;

    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("Finished planning for planner %d. Final goal cost: %f", planner_algorithm_, goal_.cost_);
  ROS_INFO("Average iteration time was: %f second", algorithm_time.toSec()/(double)algorithm_iterations);

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
  return sample_point;
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
    bool node_is_goal = (getDistanceBetweenPoints(candidate_point, goal_.position_) < goal_radius_);
    Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, 0.0, node_cost, 0.0, node_id, nearest_neighbor, node_rank, node_is_goal);
    if(node_is_goal and (new_node.cost_ < goal_.cost_))
    {
      goal_ = new_node;
      ROS_DEBUG("RRTPlanner: New goal node. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, nearest_neighbor->id_, node_rank, node_cost);
    }
    else if(!node_is_goal)
    {
      tree_.push_back(new_node);
      ROS_DEBUG("RRTPlanner: New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, nearest_neighbor->id_, node_rank, node_cost);
    } 

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
  if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0)
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

  // Check if direction is valid
  if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0)
  {
    ROS_WARN("RRTPlanner: Got request to cast ray in illegal direction.");
    ROS_WARN("            Origin (x,y,z) = (%f,%f,%f)",point_a.x,point_a.y,point_a.z);
    ROS_WARN("            End    (x,y,z) = (%f,%f,%f)",point_b.x,point_b.y,point_b.z);

    // Count illegal ray cast as collision
    return false;
  }
  else
  {
    double om_ray_distance = std::min(distance, max_ray_distance_);
    bool hit_occupied_to = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, false, om_ray_distance);
    bool hit_occupied_from = map_->castRay(om_ray_end, om_ray_return_direction, om_ray_return_cell, false, om_ray_distance);

    // Return whether there was a collision or not
    return !(hit_occupied_to or hit_occupied_from);
  }
}