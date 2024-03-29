#include <UGVFrontierExplorer.hpp>

// Constructor
  
UGVFrontierExplorer::UGVFrontierExplorer(ros::NodeHandle n, ros::NodeHandle np)
  : act_exploration_server_(n, "exploration_action", boost::bind(&UGVFrontierExplorer::actStartExploration, this, _1), false) 
{
  ROS_INFO("UGVFrontierExplorer object is being created.");

  getParams(np);

  pub_goal_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  pub_goal_path_ = n.advertise<nav_msgs::Path>("UGVFrontierExplorer/goal_path", 1);
  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>("UGVFrontierExplorer/viz_tree", 1);
  pub_viz_root_node_ = n.advertise<visualization_msgs::MarkerArray>("UGVFrontierExplorer/viz_root_node", 1);
  pub_viz_frontier_nodes_ = n.advertise<visualization_msgs::Marker>("UGVFrontierExplorer/viz_frontier_nodes", 1);

  map_initialized_ = false;
  sub_start_exploration_indiv_ = n.subscribe("/MippMonitor/start_frontier_exploration", 1, &UGVFrontierExplorer::subStartExploration, this);
  sub_map_ = n.subscribe("move_base/global_costmap/costmap", 1, &UGVFrontierExplorer::subMap, this);
  sub_map_update_ = n.subscribe("move_base/global_costmap/costmap_updates", 1, &UGVFrontierExplorer::subMapUpdate, this);
  sub_odometry_ = n.subscribe("odometry/filtered", 1, &UGVFrontierExplorer::subOdometry, this);

  act_exploration_server_.start();

  std::string move_vehicle_client_name = "/ugv/UGVPlanner/move_vehicle_action";
  move_vehicle_client_ = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(move_vehicle_client_name, true);
  move_vehicle_client_->waitForServer(ros::Duration(10.0));

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  // Init. random number generator distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Wait for map
  ros::Rate wait_rate(1.0);
  while (!map_initialized_) {
    ros::spinOnce();
    wait_rate.sleep();
  }

  running_frontier_exploration_ = false;

  ROS_INFO("Initialization done.");
}

// Destructor
  
UGVFrontierExplorer::~UGVFrontierExplorer()
{
  ROS_INFO("UGVFrontierExplorer object is being deleted.");

}

/* 
*  Subscriber callbacks
*/

void UGVFrontierExplorer::subStartExploration(const std_msgs::Bool& msg)
{
  ROS_DEBUG("subStartExploration");
  
  if (running_frontier_exploration_) { 
    ROS_INFO("Stopping frontier exploration.");
    running_frontier_exploration_ = false;
  }
  else { 
    ROS_INFO("Starting frontier exploration.");
    running_frontier_exploration_ = true;
      runFrontierExploration();
      if (frontier_nodes_.size() > 0) {
        auto frontier_goal_node = frontier_nodes_.begin()->second.getParent();
        if (frontier_goal_node->id_ != 0) {
          frontier_goal_node = frontier_goal_node->getParent();
        }
        current_frontier_goal_ = makePoseStampedFromNode(*frontier_goal_node);

        nav_msgs::Path goal_path;
        goal_path.header = current_frontier_goal_.header;
        Node node_on_path = *frontier_nodes_.begin()->second.getParent();
        while (node_on_path.getParent() != nullptr) {
          goal_path.poses.insert(goal_path.poses.begin(), makePoseStampedFromNode(node_on_path));
          ROS_DEBUG("Node: %d", (int)node_on_path.id_);

          if (node_on_path.getParent()->id_ == 0) {
            break;
          }
          node_on_path = *(node_on_path.getParent());
        }

        bool use_actionlib = true;
        if (use_actionlib) {
          move_vehicle_goal_.goal_pose = current_frontier_goal_;
          //move_vehicle_goal_.goal_path.poses = goal_path.poses;
          move_vehicle_goal_.goal_path_to_be_improved = true;
          move_vehicle_goal_.goal_reached_radius = 1.0;
          move_vehicle_goal_.goal_reached_yaw = 0.1;
          move_vehicle_goal_.goal_reached_max_time = 600.0;
          move_vehicle_client_->sendGoal(move_vehicle_goal_);
        }
        else {
          pub_goal_.publish(current_frontier_goal_);
          pub_goal_path_.publish(goal_path);
        } 
      }
  }

  ros::Rate wait_rate(20.0);
  while (running_frontier_exploration_) {
    if (getDistanceBetweenPoints(ugv_odometry_.pose.pose.position, current_frontier_goal_.pose.position) < 0.5) {
      runFrontierExploration();
      if (frontier_nodes_.size() > 0) {
        auto frontier_goal_node = frontier_nodes_.begin()->second.getParent();
        if (frontier_goal_node->id_ != 0) {
          frontier_goal_node = frontier_goal_node->getParent();
        }
        current_frontier_goal_ = makePoseStampedFromNode(*frontier_goal_node);

        nav_msgs::Path goal_path;
        goal_path.header = current_frontier_goal_.header;
        Node node_on_path = *frontier_nodes_.begin()->second.getParent();
        while (node_on_path.getParent() != nullptr) {
          goal_path.poses.insert(goal_path.poses.begin(), makePoseStampedFromNode(node_on_path));
          ROS_DEBUG("Node: %d", (int)node_on_path.id_);

          if (node_on_path.getParent()->id_ == 0) {
            break;
          }
          node_on_path = *(node_on_path.getParent());
        }

        bool use_actionlib = true;
        if (use_actionlib) {
          move_vehicle_goal_.goal_pose = current_frontier_goal_;
          //move_vehicle_goal_.goal_path.poses = goal_path.poses;
          move_vehicle_goal_.goal_path_to_be_improved = true;
          move_vehicle_goal_.goal_reached_radius = 1.0;
          move_vehicle_goal_.goal_reached_yaw = 0.1;
          move_vehicle_goal_.goal_reached_max_time = 600.0;
          move_vehicle_client_->sendGoal(move_vehicle_goal_);
        }
        else {
          pub_goal_.publish(current_frontier_goal_);
          pub_goal_path_.publish(goal_path);
        } 
      }
    }

    ros::spinOnce();
    wait_rate.sleep();
  }
}

void UGVFrontierExplorer::subMap(const nav_msgs::OccupancyGridConstPtr& map_msg)
{
  ROS_DEBUG("subMap");
  // If map in use by others, have to wait until they release it
  ros::Rate wait_rate(5.0);
  if (map_in_use_) {
    ros::spinOnce();
    wait_rate.sleep();
  }
  
  // Lock map from being operated on by others
  map_in_use_ = true;

  // Set map metadata
  map_.header = map_msg->header;
  map_.info = map_msg->info;
  map_resolution_ = map_.info.resolution;
  map_origin_x_ = map_.info.origin.position.x;
  map_origin_y_ = map_.info.origin.position.y;
  map_grid_width_ = map_.info.width;
  map_grid_height_ = map_.info.height;
  map_width_ = map_resolution_*map_grid_width_;
  map_height_ = map_resolution_*map_grid_height_;

  // Set map data
  map_.data = map_msg->data;
  ROS_DEBUG("Map size %d", (int)map_.data.size());

  // Release lock
  map_in_use_ = false;

  // Map is now initialized
  if (!map_initialized_) { map_initialized_ = true; }
}

void UGVFrontierExplorer::subMapUpdate(const map_msgs::OccupancyGridUpdateConstPtr& map_msg)
{
  ROS_DEBUG("subMapUpdate");
  // If map not initialized or in use by others, wait
  ros::Rate wait_rate(5.0);
  if (!map_initialized_ or map_in_use_) {
    ros::spinOnce();
    wait_rate.sleep();
  }
  
  // Lock map from being operated on by others
  map_in_use_ = true;

  // Set map data
  map_.data = map_msg->data;
  ROS_DEBUG("Map size %d", (int)map_.data.size());

  // Release lock
  map_in_use_ = false;
}

void UGVFrontierExplorer::subOdometry(const nav_msgs::Odometry odometry_msg)
{
  //ROS_DEBUG("subOdometry");
  ugv_odometry_ = odometry_msg;
}

// Actionlib

void UGVFrontierExplorer::actStartExploration(const mipp_msgs::StartExplorationGoalConstPtr &goal)
{
  ROS_DEBUG("actStartExploration");

  planner_action_in_progress_ = true;
  planner_max_time_ = goal->max_time;
  runFrontierExploration();

  act_exploration_result_.result.header = current_frontier_goal_.header;
  std::map<double, Node>::iterator frontier_node_it = frontier_nodes_.begin();
  act_exploration_result_.result.paths.clear();
  while (frontier_node_it != frontier_nodes_.end()) {
    mipp_msgs::ExplorationPath frontier_path;
    Node node_on_path = *frontier_node_it->second.getParent();
    while (node_on_path.getParent() != nullptr) {
      mipp_msgs::ExplorationPose frontier_path_pose;
      frontier_path_pose.gain = 0;
      frontier_path_pose.pose = makePoseFromNode(node_on_path);
      frontier_path.poses.insert(frontier_path.poses.begin(), frontier_path_pose);
      if (node_on_path.getParent()->id_ == 0) {
        break;
      }
      node_on_path = *(node_on_path.getParent());
    }
    act_exploration_result_.result.paths.push_back(frontier_path);
    frontier_node_it++;
  }
  act_exploration_server_.setSucceeded(act_exploration_result_);
  ROS_WARN("Succeeded with %d paths", (int)act_exploration_result_.result.paths.size());
  planner_action_in_progress_ = false;
}

/* 
*  Frontier exploration functions
*/

void UGVFrontierExplorer::runFrontierExploration()
{
  ROS_DEBUG("planPathToGoal");
  // If map not initialized or in use by others, wait
  ros::Rate wait_rate(5.0);
  if (!map_initialized_ or map_in_use_) {
    ros::spinOnce();
    wait_rate.sleep();
  }

  path_.clear();
  //tree_.clear();
  frontier_nodes_.clear();

  if(use_dynamic_range_){
    ROS_DEBUG("Changing point-sampling bounds to costmap bounds + padding.");
    // Costmap origin is in lower left corner
    double map_x_min = map_origin_x_;
    double map_x_max = map_origin_x_ + map_width_;
    double map_y_min = map_origin_y_;
    double map_y_max = map_origin_y_ + map_height_;
    ROS_DEBUG("Costmap origin (%f,%f), size (%d,%d).", map_origin_x_, map_origin_y_, map_grid_width_, map_grid_height_);
    ROS_DEBUG("Costmap bounds X: (%f,%f), Y: (%f,%f).", map_x_min, map_x_max, map_y_min, map_y_max);
    x_range_min_ = map_x_min - dynamic_range_padding_;
    x_range_max_ = map_x_max + dynamic_range_padding_;
    y_range_min_ = map_y_min - dynamic_range_padding_;
    y_range_max_ = map_y_max + dynamic_range_padding_;
    ROS_DEBUG("Sampling bounds X: (%f,%f), Y: (%f,%f).", x_range_min_, x_range_max_, y_range_min_, y_range_max_);
    x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
    y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  }

  double start_x = ugv_odometry_.pose.pose.position.x;
  double start_y = ugv_odometry_.pose.pose.position.y;
  const double start_yaw = tf2::getYaw(ugv_odometry_.pose.pose.orientation);
  if (tree_.empty()) {
    root_ = Node(start_x, start_y, 0.0, start_yaw, 0.0, 0.0, 0, nullptr, false);
    tree_.push_back(root_);
  }

  // Grow frontier exploration tree
  ros::Rate rate(planner_rate_);
  ros::Time start_time = ros::Time::now();
  while((ros::Time::now() - start_time).toSec() < planner_max_time_ and frontier_nodes_.size() < planner_max_frontier_nodes_){
    ros::Time begin = ros::Time::now();
    
    geometry_msgs::Point sample_point = generateRandomPoint();
    ROS_DEBUG("Sampled random point: (x,y,z) = (%f,%f,%f)",sample_point.x,sample_point.y,sample_point.z);

    // Start finding nearest node in the tree to the sampled point (init. as root)
    Node* nearest_neighbor = &root_;
    double nearest_neighbor_distance = getDistanceBetweenPoints(sample_point, root_.position_);

    // Iterate through tree nodes, checking distance to sampled point
    double min_allowed_node_distance = 0.1;
    bool sample_is_too_close = false;
    for(std::list<Node>::iterator tree_node_itr = tree_.begin();
        tree_node_itr != tree_.end(); tree_node_itr++)
    {
      // Get distance to tree node
      double distance_to_node = getDistanceBetweenPoints(sample_point, tree_node_itr->position_);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_itr->id_, distance_to_node);

      if(distance_to_node < min_allowed_node_distance)
      {
        ROS_DEBUG("Too close.");
        sample_is_too_close = true;
        break;
      }

      // Make tree node new nearest neighbor if it is closer than current nearest neighbor
      if(distance_to_node < nearest_neighbor_distance) 
      {
        ROS_DEBUG("Sampled node is closer to node %d: %f", tree_node_itr->id_, distance_to_node);
        nearest_neighbor = &*tree_node_itr;
        nearest_neighbor_distance = distance_to_node;
      }
    }

    if(!sample_is_too_close){
      /* Get the coordinates of the new point
      * Cast a ray from the nearest neighbor in the direction of the sampled point.
      * If nearest neighbor is within planner_max_ray_distance_ of sampled point, sampled point is new point.
      * Else the endpoint of the ray (with length planner_max_ray_distance_) is new point
      */
      geometry_msgs::Point new_point_ray_origin = nearest_neighbor->position_;
      geometry_msgs::Vector3 new_point_ray_direction = getDirection(new_point_ray_origin, sample_point);
      geometry_msgs::Point new_point = castRay(nearest_neighbor->position_, 
                                                new_point_ray_direction, 
                                                std::min(nearest_neighbor_distance, planner_max_ray_distance_));

      // RRTstar algorithm
      bool is_sampling_goal = false;
      bool is_searching_frontiers = true;
      extendTreeRRTstar(new_point);
    }

    visualizeTree();
    visualizeRoot(root_.position_, 0.0, 1.0, 0.0);
    visualizeFrontierNodes(0.0, 0.0, 1.0);

    if(planner_action_in_progress_){
      act_exploration_feedback_.nodes_in_tree = tree_.size();
      act_exploration_server_.publishFeedback(act_exploration_feedback_);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void UGVFrontierExplorer::extendTreeRRTstar(geometry_msgs::Point candidate_point)
{
  ROS_DEBUG("extendTreeRRTstar");

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
    if(distance_to_node < planner_max_ray_distance_+0.01) 
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
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) { 
    bool pathIsCollisionFree = isPathCollisionFree(neighbor_itr->second->position_.x, neighbor_itr->second->position_.y, candidate_point.x, candidate_point.y);
    if(pathIsCollisionFree) {
      int node_id = tree_.size();
      int node_rank = neighbor_itr->second->rank_ + 1;
      float node_yaw = atan2(candidate_point.y - neighbor_itr->second->position_.y, candidate_point.x - neighbor_itr->second->position_.x);
      float node_cost = neighbor_itr->second->cost_ + getDistanceBetweenPoints(candidate_point, neighbor_itr->second->position_);
      bool node_is_goal = false;
      Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, node_yaw, node_cost, 0.0, node_id, neighbor_itr->second, node_rank, node_is_goal);

      if (isPositionUnmapped(candidate_point.x, candidate_point.y) and node_cost > planner_min_distance_to_frontier_ and node_cost < planner_max_distance_to_frontier_) {
        float frontier_node_cost = getDistanceBetweenPoints(ugv_odometry_.pose.pose.position, candidate_point);
        frontier_nodes_.insert(std::pair<double, Node>(frontier_node_cost, new_node));
        ROS_DEBUG("Frontier found. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      } else {
        tree_.push_back(new_node);
        ROS_DEBUG("New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      }
      
      // Since neighbor map is already sorted, the first collision free candidate is best candidate. Therefore break.
      break;
    }
    else
    {
      ROS_DEBUG("Collision detected when trying to add node at (x,y,z) = (%f,%f,%f)",candidate_point.x,candidate_point.y,candidate_point.z);
    }
  }
}

geometry_msgs::Point UGVFrontierExplorer::generateRandomPoint()
{
  ROS_DEBUG("generateRandomPoint");
  geometry_msgs::Point sample_point;
  sample_point.x = x_distribution_(generator_);
  sample_point.y = y_distribution_(generator_);
  sample_point.z = 0.0;
  return sample_point;
}

/* 
*  Utility functions
*/

void UGVFrontierExplorer::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("getParams");
  np.param<bool>("use_dynamic_range", use_dynamic_range_, true);
  np.param<double>("dynamic_range_padding", dynamic_range_padding_, 2.0);
  np.param<double>("x_range_min", x_range_min_, -10.0);
  np.param<double>("x_range_max", x_range_max_,  10.0);
  np.param<double>("y_range_min", y_range_min_, -10.0);
  np.param<double>("y_range_max", y_range_max_,  10.0);
  np.param<std::string>("planner_world_frame", planner_world_frame_, "map");
  np.param<double>("planner_rate", planner_rate_, 10.0);
  np.param<double>("planner_max_time", planner_max_time_, 5.0);
  np.param<int>("planner_max_frontier_nodes", planner_max_frontier_nodes_, 50);
  np.param<double>("planner_max_ray_distance", planner_max_ray_distance_, 2.0);
  np.param<double>("planner_min_distance_to_frontier", planner_min_distance_to_frontier_, 4.0);
  np.param<double>("planner_max_distance_to_frontier", planner_max_distance_to_frontier_, 100.0);
  np.param<int>("collision_threshold", collision_threshold_, 0);
  np.param<bool>("unmapped_is_collision", unmapped_is_collision_, false);
}

/* 
*  Map utility functions
*/

void UGVFrontierExplorer::convMapToWorld(int map_x, int map_y, double& world_x, double& world_y)
{
  ROS_DEBUG("convMapToWorld");
  world_x = (map_x + 0.5)*map_resolution_ + map_origin_x_;
  world_y = (map_y + 0.5)*map_resolution_ + map_origin_y_;
}

void UGVFrontierExplorer::convWorldToMap(double world_x, double world_y, int& map_x, int& map_y)
{
  ROS_DEBUG("convWorldToMap");

  if (world_x < map_origin_x_) {
    map_x = 0;
  } 
  else if (world_x >= map_width_ + map_origin_x_) {
    map_x = map_grid_width_ - 1;
  }
  else {
    map_x = (int)((world_x - map_origin_x_)/map_resolution_);
  }

  if (world_y < map_origin_y_) {
    map_y = 0;
  } 
  else if (world_y >= map_height_ + map_origin_y_) {
    map_y = map_grid_height_ - 1;
  }
  else {
    map_y = (int)((world_y - map_origin_y_)/map_resolution_);
  }
}

int UGVFrontierExplorer::getGridIndex(int map_x, int map_y)
{
  ROS_DEBUG("getGridIndex");
  return map_y*map_grid_width_ + map_x;
}

int UGVFrontierExplorer::getGridIndex(double world_x, double world_y)
{
  ROS_DEBUG("getGridIndex");
  int map_x, map_y;
  convWorldToMap(world_x, world_y, map_x, map_y);
  return map_y*map_grid_width_ + map_x;
}

bool UGVFrontierExplorer::isPathCollisionFree(double world_x_a, double world_y_a, double world_x_b, double world_y_b)
{
  ROS_DEBUG("isPathCollisionFree");

  geometry_msgs::Point point_a = makePoint(world_x_a, world_y_b, 0.0);
  geometry_msgs::Point point_b = makePoint(world_x_b, world_y_b, 0.0);
  geometry_msgs::Vector3 direction = getDirection(point_a, point_b);
  double distance = getDistanceBetweenPoints(point_a, point_b);
  double angle_ab = atan2(point_b.y - point_a.y, point_b.x - point_a.x);
  
  // Check first if point_b is collision free configuration
  if (!isPositionCollisionFree(world_x_b, world_y_b)) {
    ROS_DEBUG("Path endpoint is in collision");
    return false;
  }

  double distance_on_line = 0.0;
  geometry_msgs::Point point_on_line = point_a;
  while (distance_on_line < distance) {
    if (!isPositionCollisionFree(point_on_line.x, point_on_line.y)) {
      return false;
    }
    double collision_resolution = map_resolution_/2.0;
    distance_on_line += collision_resolution;
    point_on_line.x = point_on_line.x + collision_resolution*direction.x;
    point_on_line.y = point_on_line.y + collision_resolution*direction.y;
  }

  return true;
}

bool UGVFrontierExplorer::isPositionOutsideMap(double world_x, double world_y)
{
  ROS_DEBUG("isPositionOutsideMap");
  ROS_DEBUG("(%.2f, %.2f) , (%.2f, %.2f) - (%.2f, %.2f)", world_x, world_y, 
                                                         map_origin_x_, map_width_ + map_origin_x_,
                                                         map_origin_y_, map_height_ + map_origin_y_);
  return (world_x < map_origin_x_ or 
          world_x >= map_width_ + map_origin_x_ or
          world_y < map_origin_y_ or 
          world_y >= map_height_ + map_origin_y_);
}

bool UGVFrontierExplorer::isPositionCollisionFree(double world_x, double world_y)
{
  ROS_DEBUG("isPositionCollisionFree");
  int gridCost = map_.data[getGridIndex(world_x, world_y)];
  if (gridCost > collision_threshold_) {
    ROS_DEBUG("Position NOT collision free: grid cost %d above collision threshold %d", gridCost, collision_threshold_);
    return false;
  }
  else if (isPositionUnmapped(world_x, world_y) and unmapped_is_collision_) {
    ROS_DEBUG("Position NOT collision free: position is unmapped");
    return false;
  }
  return true;
}

bool UGVFrontierExplorer::isPositionUnmapped(double world_x, double world_y)
{
  ROS_DEBUG("isPositionUnmapped");
  int gridCost = map_.data[getGridIndex(world_x, world_y)];
  return (gridCost == -1 or isPositionOutsideMap(world_x, world_y));
}

geometry_msgs::PoseStamped UGVFrontierExplorer::makePoseStampedFromNode(Node node)
{
  ROS_DEBUG("makePoseStampedFromNode");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = planner_world_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.position = node.position_;
  tf2::Quaternion pose_quat;
  pose_quat.setRPY(0, 0, node.yaw_);
  pose.pose.orientation.x = pose_quat.x();
  pose.pose.orientation.y = pose_quat.y();
  pose.pose.orientation.z = pose_quat.z();
  pose.pose.orientation.w = pose_quat.w();
  return pose;
}

geometry_msgs::Pose UGVFrontierExplorer::makePoseFromNode(Node node)
{
  ROS_DEBUG("makePoseFromNode");
  geometry_msgs::Pose pose;
  pose.position = node.position_;
  tf2::Quaternion pose_quat;
  pose_quat.setRPY(0, 0, node.yaw_);
  pose.orientation.x = pose_quat.x();
  pose.orientation.y = pose_quat.y();
  pose.orientation.z = pose_quat.z();
  pose.orientation.w = pose_quat.w();
  return pose;
}

/* 
*  Visualization functions
*/

void UGVFrontierExplorer::visualizeTree()
{
  ROS_DEBUG("visualizeTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.03;
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.5;
  tree_marker.color.b = 0.0;
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

void UGVFrontierExplorer::visualizeRoot(geometry_msgs::Point point, double red, double green, double blue)
{
  ROS_DEBUG("visualizeRoot");
  visualization_msgs::MarkerArray marker_point;
  visualization_msgs::Marker m;

  m.header.frame_id = planner_world_frame_;
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.5;
  m.scale.y = 0.5;
  m.scale.z = 0.5;
  m.color.a = 1.0;
  m.color.r = red;
  m.color.g = green;
  m.color.b = blue;
  m.lifetime = ros::Duration();
  m.id = 0;
  m.pose.position = point;
  marker_point.markers.push_back(m);
  
  pub_viz_root_node_.publish(marker_point);
}

void UGVFrontierExplorer::visualizeFrontierNodes(double red, double green, double blue)
{
  ROS_DEBUG("visualizeFrontierNodes");
  visualization_msgs::Marker m;

  m.header.frame_id = planner_world_frame_;
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.25;
  m.scale.y = 0.25;
  m.scale.z = 0.5;
  m.color.a = 0.8;
  m.color.r = red;
  m.color.g = green;
  m.color.b = blue;
  m.lifetime = ros::Duration();
  m.id = 0;

  std::map<double, Node>::iterator frontier_node_itr;
  for(frontier_node_itr = frontier_nodes_.begin(); frontier_node_itr != frontier_nodes_.end(); ++frontier_node_itr){
    m.points.push_back(frontier_node_itr->second.position_);
  }
  
  pub_viz_frontier_nodes_.publish(m);
}