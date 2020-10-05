/**
* @file UGVPlanner.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Contains the UGVPlanner class
*/

#include <UGVPlanner.hpp>

namespace ugv_planner {

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ugv_planner::UGVPlanner, nav_core::BaseGlobalPlanner)

// Constructor

UGVPlanner::UGVPlanner() : costmap_ros_(NULL), initialized_(false){}

UGVPlanner::UGVPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

// Destructor
  
UGVPlanner::~UGVPlanner()
{
  ROS_INFO("UGVPlanner object is being deleted.");
}

// Init.

void UGVPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(initialized_)
  {
    ROS_WARN("This planner has already been initialized... doing nothing");
    return;
  }
  
  ROS_INFO("UGVPlanner object is being created.");

  ros::NodeHandle n("/" + name);
  ros::NodeHandle np("~/" + name);

  costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
  costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_
  world_model_ = new base_local_planner::CostmapModel(*costmap_);

  getParams(np);

  timer_replan_checker_ = n.createTimer(ros::Duration(2.0), boost::bind(&UGVPlanner::replanCheck, this));

  pub_goal_ = n.advertise<geometry_msgs::PoseStamped>("/ugv/move_base_simple/goal", 1);
  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>(robot_namespace_+"/UGVPlanner/viz_tree", 1);
  pub_viz_collision_tree_ = n.advertise<visualization_msgs::Marker>(robot_namespace_+"/UGVPlanner/viz_collision_tree", 1);
  pub_viz_path_to_goal_ = n.advertise<visualization_msgs::Marker>(robot_namespace_+"/UGVPlanner/path_to_goal", 1);
  pub_viz_root_node_ = n.advertise<visualization_msgs::MarkerArray>(robot_namespace_+"/UGVPlanner/root_node", 1);
  pub_viz_goal_node_ = n.advertise<visualization_msgs::MarkerArray>(robot_namespace_+"/UGVPlanner/goal_node", 1);
  pub_viz_subgoal_node_ = n.advertise<visualization_msgs::MarkerArray>(robot_namespace_+"/UGVPlanner/subgoal_node", 1);
  pub_viz_frontier_nodes_ = n.advertise<visualization_msgs::Marker>(robot_namespace_+"/UGVPlanner/frontier_nodes", 1);

  sub_initial_path_ = n.subscribe("/ugv/UGVFrontierExplorer/goal_path", 1, &UGVPlanner::subInitialPath, this);
  sub_odometry_ = n.subscribe("odometry/filtered", 1, &UGVPlanner::subOdometry, this);

  // Init. random number generator distributions
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  generator_ = std::default_random_engine(rd());
  x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);
  unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Init. root_ and goal_ as invalid (id -1), wait for sub callbacks
  root_ = Node(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1, nullptr, 0, false);
  goal_ = Node(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1, nullptr, 0, false);

  ROS_INFO("Initialization done. Waiting for path.");
  initialized_ = true;
}

/* 
*  Subscriber callbacks
*/

void UGVPlanner::subInitialPath(const nav_msgs::Path& path_msg)
{
  ROS_INFO("subInitialPath()");

  bool path_poses_ordered = true; // Some implementations send path in reverse order (frontier node, ..., root node), if so this is false
  path_poses_ordered = getDistanceBetweenPoints(ugv_odometry_.pose.pose.position, path_msg.poses.begin()->pose.position) 
                       < getDistanceBetweenPoints(ugv_odometry_.pose.pose.position, path_msg.poses.rbegin()->pose.position);

  initial_path_.clear();
  if (path_poses_ordered) {
    for (auto path_it = path_msg.poses.begin(); path_it != path_msg.poses.end(); ++path_it) {
      ROS_WARN("Initial path point (x,y,z) = (%f,%f,%f)", path_it->pose.position.x, path_it->pose.position.y, path_it->pose.position.z);
      initial_path_.push_back(makePoint(path_it->pose.position.x, path_it->pose.position.y, path_it->pose.position.z));
    }
  }
  else {
    for (auto path_it = path_msg.poses.begin(); path_it != path_msg.poses.end(); ++path_it) {
      ROS_WARN("Initial path unordered");
      ROS_WARN("Initial path point (x,y,z) = (%f,%f,%f)", path_it->pose.position.x, path_it->pose.position.y, path_it->pose.position.z);
      initial_path_.push_front(makePoint(path_it->pose.position.x, path_it->pose.position.y, path_it->pose.position.z));
    }
  }
}

void UGVPlanner::subOdometry(const nav_msgs::Odometry& odometry_msg)
{
  ROS_DEBUG("subOdometry()");
  ugv_odometry_ = odometry_msg;
}

/* 
*  Planner functions
*/

bool UGVPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_DEBUG("makePlan()");

  // From global_planner plugin:
  if(!initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  plan.clear();
  path_.clear();
  tree_.clear();
  costmap_ = costmap_ros_->getCostmap();

  if(use_dynamic_range_){
    ROS_DEBUG("Changing point-sampling bounds to costmap bounds + padding.");
    // Costmap origin is in lower left corner
    double costmap_x_min = costmap_->getOriginX();
    double costmap_x_max = costmap_->getOriginX() + costmap_->getSizeInMetersX();
    double costmap_y_min = costmap_->getOriginY();
    double costmap_y_max = costmap_->getOriginY() + costmap_->getSizeInMetersY();
    ROS_DEBUG("Costmap origin (%f,%f), size (%f,%f).", costmap_->getOriginX(), costmap_->getOriginY(), costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    ROS_DEBUG("Costmap bounds X: (%f,%f), Y: (%f,%f).", costmap_x_min, costmap_x_max, costmap_y_min, costmap_y_max);
    x_range_min_ = costmap_x_min - dynamic_range_padding_;
    x_range_max_ = costmap_x_max + dynamic_range_padding_;
    y_range_min_ = costmap_y_min - dynamic_range_padding_;
    y_range_max_ = costmap_y_max + dynamic_range_padding_;
    ROS_DEBUG("Sampling bounds X: (%f,%f), Y: (%f,%f).", x_range_min_, x_range_max_, y_range_min_, y_range_max_);
    x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
    y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
    z_distribution_ = std::uniform_real_distribution<double>(z_range_min_, z_range_max_);
  }

  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  root_ = Node(start_x, start_y, 0.0, start_yaw, 0.0, 0.0, 0, nullptr, false);
  tree_.push_back(root_);
  
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  goal_ = Node(goal_x, goal_y, 0.0, goal_yaw, INFINITY, 0.0, -1, nullptr, true);

  // Check first if goal can be reached with straight line
  bool return_collision_free_point = false;
  if(isPathCollisionFree(root_.position_, goal_.position_, return_collision_free_point)){
    ROS_INFO("No collision between root and goal detected. Returning straight line plan.");
    // Create "tree" of interpolated nodes on straight line between root and goal
    plan.push_back(makePoseStampedFromNode(root_));
    path_.push_back(root_.position_);
    geometry_msgs::Point point_on_path = root_.position_;
    Node node_on_path = root_;
    geometry_msgs::Vector3 path_direction = getDirection(root_.position_, goal_.position_);
    while(getDistanceBetweenPoints(point_on_path, goal_.position_) > planner_max_ray_distance_){
      point_on_path.x += planner_max_ray_distance_*path_direction.x;
      point_on_path.y += planner_max_ray_distance_*path_direction.y;
      point_on_path.z += planner_max_ray_distance_*path_direction.z;

      int node_id = tree_.size();
      int node_rank = node_on_path.rank_ + 1;
      float node_yaw = atan2(point_on_path.y - node_on_path.position_.y, point_on_path.x - node_on_path.position_.x);
      float node_cost = node_on_path.cost_ + getDistanceBetweenPoints(point_on_path, node_on_path.position_);
      bool node_is_goal = false;
      Node new_node(point_on_path.x, point_on_path.y, point_on_path.z, node_yaw, node_cost, 0.0, node_id, &(tree_.back()), node_rank, node_is_goal);
      
      tree_.push_back(new_node);
      path_.push_back(new_node.position_);
      plan.push_back(makePoseStampedFromNode(new_node));

      ROS_DEBUG("New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, new_node.getParent()->id_, node_rank, node_cost);
      node_on_path = new_node;
    }
    goal_.setParent(&(tree_.back()));
    goal_.id_ = tree_.size();
    goal_.cost_ = node_on_path.cost_ + getDistanceBetweenPoints(node_on_path.position_, goal_.position_);
    plan.push_back(makePoseStampedFromNode(goal_));
    path_.push_back(goal_.position_);

    visualizeTree();
    visualizePathToGoal();
    visualizeRoot(root_.position_, 0.0, 1.0, 0.0);
    visualizeGoal(goal_.position_, 1.0, 0.0, 0.0);

    return true;
  }

  // Init. values for informed RRT*
  goal_euclidean_distance_ = getDistanceBetweenPoints(root_.position_, goal_.position_);
  ROS_WARN("Euc dist %f", goal_euclidean_distance_);
  goal_grow_distance_ = 1.1*goal_euclidean_distance_;
  goal_path_distance_ = goal_.cost_;
  goal_root_midpoint_ = makePoint((goal_.position_.x + root_.position_.x)/2.0, (goal_.position_.y + root_.position_.y)/2.0, (goal_.position_.z + root_.position_.z)/2.0);
  double goal_root_angle = atan2(goal_.position_.y - root_.position_.y, goal_.position_.x - root_.position_.x);
  goal_root_rotation_[0][0] = cos(goal_root_angle);
  goal_root_rotation_[0][1] = -sin(goal_root_angle);
  goal_root_rotation_[1][0] = sin(goal_root_angle);
  goal_root_rotation_[1][1] = cos(goal_root_angle);

  ros::Rate rate(planner_rate_);
  ros::Time start_time = ros::Time::now();
  ros::Time last_check_time = ros::Time::now();
  double last_check_cost = goal_.cost_;
  double check_interval = 0.2;
  while((ros::Time::now() - start_time).toSec() < planner_max_time_){
    ros::Time begin = ros::Time::now();
    
    geometry_msgs::Point sample_point;
    bool sample_is_goal = false;
    if (!initial_path_.empty()){
      sample_point = initial_path_.front();
      initial_path_.erase(initial_path_.begin());
      ROS_INFO("Sampling initial path point: (x,y,z) = (%f,%f,%f)",sample_point.x,sample_point.y,sample_point.z);
    }
    else if(unit_distribution_(generator_) < goal_sample_probability_){
      sample_point = goal_.position_;
      sample_is_goal = true;
      ROS_DEBUG("Sampling goal point: (x,y,z) = (%f,%f,%f)",sample_point.x,sample_point.y,sample_point.z);
    } else {
      sample_point = generateRandomInformedPoint();
      ROS_DEBUG("Sampled informed point: (x,y,z) = (%f,%f,%f)",sample_point.x,sample_point.y,sample_point.z);
    }

    // Start finding nearest node in the tree to the sampled point (init. as root)
    Node* nearest_neighbor = &root_;
    double nearest_neighbor_distance = getDistanceBetweenPoints(sample_point, root_.position_);

    // Iterate through tree nodes, checking distance to sampled point
    double min_allowed_node_distance = 0.1;
    bool sample_is_too_close = false;
    for (std::list<Node>::iterator tree_node_itr = tree_.begin();
         tree_node_itr != tree_.end(); tree_node_itr++) {
      // Get distance to tree node
      double distance_to_node = getDistanceBetweenPoints(sample_point, tree_node_itr->position_);
      ROS_DEBUG("Sampled node distance to node %d: %f", tree_node_itr->id_, distance_to_node);

      if (distance_to_node < min_allowed_node_distance and !sample_is_goal) {
        ROS_DEBUG("Too close.");
        sample_is_too_close = true;
        break;
      }

      // Make tree node new nearest neighbor if it is closer than current nearest neighbor
      if (distance_to_node < nearest_neighbor_distance) {
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
      extendTreeRRTstar(new_point);
    }

    goal_path_distance_ = goal_.cost_;

    visualizeTree();
    visualizePathToGoal();
    visualizeRoot(root_.position_, 0.0, 1.0, 0.0);
    visualizeGoal(goal_.position_, 1.0, 0.0, 0.0);

    // Check if goal has improved much from last check time
    if((ros::Time::now() - last_check_time).toSec() > check_interval){
      if(last_check_cost - goal_.cost_ < 1.0){
        ROS_DEBUG("Finishing plan early (%f/%f s), no improvement.", (ros::Time::now() - start_time).toSec(), planner_max_time_);
        break;
      } 
      else{
        last_check_time = ros::Time::now();
        last_check_cost = goal_.cost_;
      }
    }

    // Check if path distance is close to euvlidean distance (straight line found)
    if(std::abs(goal_path_distance_ - goal_euclidean_distance_) < 0.01){
      ROS_DEBUG("Finishing plan early (%f/%f s), straight line path found.", (ros::Time::now() - start_time).toSec(), planner_max_time_);
      break;
    }

    ros::spinOnce();
    rate.sleep();
  }

  if(goal_.getParent() == nullptr){
    ROS_WARN("Could not find a global path to goal. Returning start-node as plan.");
    plan.insert(plan.begin(), makePoseStampedFromNode(root_));
    return true;
  }

  ROS_DEBUG("Finished planning. Final goal cost: %f",  goal_.cost_);

  // Iterate through path from goal to root and construct path variable
  Node node_on_goal_path = goal_;
  plan.insert(plan.begin(), makePoseStampedFromNode(node_on_goal_path));
  path_.insert(path_.begin(), node_on_goal_path.position_);
  while(node_on_goal_path.id_ != 0){
    node_on_goal_path = *node_on_goal_path.getParent();
    plan.insert(plan.begin(), makePoseStampedFromNode(node_on_goal_path));
    path_.insert(path_.begin(), node_on_goal_path.position_);
  }

  // Optimize path from goal, making it shorter if possible
  double optimization_resolution = 0.5;
  optimizePath(optimization_resolution);
  optimizePath(0.5*optimization_resolution);
  optimizePath(0.2*optimization_resolution);
  visualizePath(path_);
  plan.clear();
  for (auto path_it = path_.begin(); path_it != path_.end(); ++path_it) {
    geometry_msgs::PoseStamped path_pose;
    path_pose.header.frame_id = planner_world_frame_;
    path_pose.header.stamp = ros::Time::now();
    path_pose.pose.position = *path_it;
    tf2::Quaternion path_pose_quat;
    path_pose_quat.setRPY(0, 0, goal_.yaw_);
    path_pose.pose.orientation.x = path_pose_quat.x();
    path_pose.pose.orientation.y = path_pose_quat.y();
    path_pose.pose.orientation.z = path_pose_quat.z();
    path_pose.pose.orientation.w = path_pose_quat.w();
    plan.push_back(path_pose);
  }

  return true;
}

geometry_msgs::Point UGVPlanner::generateRandomPoint()
{
  ROS_DEBUG("generateRandomPoint()");
  geometry_msgs::Point sample_point;
  sample_point.x = x_distribution_(generator_);
  sample_point.y = y_distribution_(generator_);
  sample_point.z = z_distribution_(generator_);
  return sample_point;
}

geometry_msgs::Point UGVPlanner::generateRandomInformedPoint()
{
  // Do the Informed RRT* search in 2D (x,y), sampling z uniformly in z-range
  // Details in Informed RRT* paper: https://arxiv.org/pdf/1404.2334.pdf
  ROS_DEBUG("generateRandomInformedPoint()");

  // Revert to normal random point if grow-informed planning doesn't work
  if (goal_grow_distance_ > 2.0*goal_euclidean_distance_ and goal_.cost_ == INFINITY) {
    return generateRandomPoint();
  }

  geometry_msgs::Point sample_point;

  // Sample from unit circle, uniformly https://stackoverflow.com/a/50746409
  double radius = sqrt(unit_distribution_(generator_));
  double theta = unit_distribution_(generator_)*2.0*M_PI;
  double unit_circle_x = radius*cos(theta);
  double unit_circle_y = radius*sin(theta);
  ROS_DEBUG("Unit circle point (x,y) = (%f,%f)", unit_circle_x, unit_circle_y);

  // Define ellipse dimensions according to root, goal and minimum found goal path length
  double path_distance;
  if (goal_path_distance_ < INFINITY) {
    path_distance = goal_path_distance_;
    ROS_DEBUG("Path dist is found: %f", path_distance);
  }
  else {
    path_distance = goal_grow_distance_;
    ROS_DEBUG("Path dist is grow: %f", path_distance);
  }
  
  double r1 = path_distance/2.0;
  double r2 = sqrt(std::max(pow(path_distance,2.0) - pow(goal_euclidean_distance_,2.0),0.0))/2.0; // max in case value in sqrt is negative
  ROS_DEBUG("Ellipse dimensions (r1,r2) = (%f,%f)", r1, r2);

  // Scale, rotate and translate the unit circle point to rotated ellipse point with center between
  // goal and root
  ROS_DEBUG("Ellipse point (x,y) = (%f,%f)",r1*unit_circle_x,r2*unit_circle_y);
  sample_point.x = goal_root_rotation_[0][0]*r1*unit_circle_x + goal_root_rotation_[0][1]*r2*unit_circle_y + goal_root_midpoint_.x;
  sample_point.y = goal_root_rotation_[1][0]*r1*unit_circle_x + goal_root_rotation_[1][1]*r2*unit_circle_y + goal_root_midpoint_.y;

  // Clip/limit x and y to still be within general x- and y-range
  sample_point.x = std::max(x_range_min_, std::min(sample_point.x, x_range_max_));
  sample_point.y = std::max(y_range_min_, std::min(sample_point.y, y_range_max_));

  // Sample z from normal unit distribution, because I'm lazy
  sample_point.z = z_distribution_(generator_);

  return sample_point;
}

void UGVPlanner::extendTreeRRTstar(geometry_msgs::Point candidate_point)
{
  ROS_DEBUG("extendTreeRRTstar(%f, %f, %f)", candidate_point.x, candidate_point.y, candidate_point.z);

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
    if(distance_to_node < 2*planner_max_ray_distance_+0.01) 
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
  bool new_node_added = false;
  bool return_collision_free_point = false;
  std::vector<geometry_msgs::Point> collision_tree;
  for (neighbor_itr = neighbor_list.begin(); neighbor_itr != neighbor_list.end(); ++neighbor_itr) { 
    bool pathIsCollisionFree = isPathCollisionFree(neighbor_itr->second->position_, candidate_point, return_collision_free_point);
    if (neighbor_itr->second->id_ == 0) {
      // Neighbor is root, could be that UGV thinks it is an obstacle (observed by a UAV) and is stuck
      pathIsCollisionFree = isPathFromRootCollisionFree(neighbor_itr->second->position_, candidate_point, return_collision_free_point);
    }
    if (pathIsCollisionFree) {
      int node_id = tree_.size();
      int node_rank = neighbor_itr->second->rank_ + 1;
      float node_yaw = atan2(candidate_point.y - neighbor_itr->second->position_.y, candidate_point.x - neighbor_itr->second->position_.x);
      float node_cost = neighbor_itr->second->cost_ + getDistanceBetweenPoints(candidate_point, neighbor_itr->second->position_);
      bool node_is_goal = (getDistanceBetweenPoints(candidate_point, makePoint(goal_.position_.x, goal_.position_.y, goal_.position_.z)) < goal_radius_);
      Node new_node(candidate_point.x, candidate_point.y, candidate_point.z, node_yaw, node_cost, 0.0, node_id, neighbor_itr->second, node_rank, node_is_goal);

      if(node_is_goal and (new_node.cost_ < goal_.cost_)) {
        new_node.yaw_ = goal_.yaw_;
        goal_ = new_node;
        ROS_DEBUG("New goal node. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      } else if(!node_is_goal) {
        tree_.push_back(new_node);
        ROS_DEBUG("New node added. ID: %d, Parent: %d, Rank: %d, Cost: %f", node_id, neighbor_itr->second->id_, node_rank, node_cost);
      }

      new_node_added = true;
      
      // Since neighbor map is already sorted, the first collision free candidate is best candidate. Therefore break.
      break;
    }
    else
    {
      ROS_DEBUG("Collision detected when trying to add node at (x,y,z) = (%f,%f,%f)",candidate_point.x,candidate_point.y,candidate_point.z);
      collision_tree.push_back(neighbor_itr->second->position_);
      collision_tree.push_back(candidate_point);
    }
  }
  if (goal_path_distance_ == INFINITY and !new_node_added) {
    double max_path_distance = costmap_->getSizeInMetersX() + costmap_->getSizeInMetersY();
    goal_grow_distance_ = std::min(goal_grow_distance_*1.05, max_path_distance);
    ROS_DEBUG("Growing goal_grow_distance_, new size: %f",goal_grow_distance_);
  }

  visualizeCollisionTree(collision_tree);
}

bool UGVPlanner::isPathCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point& point_b, 
                                     bool return_collision_free_point,
                                     geometry_msgs::Vector3 direction_ab, double distance_ab)
{
  ROS_DEBUG("isPathCollisionFree()");

  // Check if optional arguments are set (default/initialized values aren't valid)
  if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0){
    direction_ab = getDirection(point_a, point_b);
  }
  if(distance_ab == -1.0){
    distance_ab = getDistanceBetweenPoints(point_a, point_b);
  }

  double angle_ab = atan2(point_b.y - point_a.y, point_b.x - point_a.x);
  
  // Check first if point_b is collision free configuration
  if(footprintCost(point_b.x, point_b.y, angle_ab) == -1.0){
    return false;
  }

  double distance_on_line = 0.0;
  geometry_msgs::Point point_on_line = point_a;
  while(distance_on_line < distance_ab){
    if(footprintCost(point_on_line.x, point_on_line.y, angle_ab) == -1.0){
      if (return_collision_free_point) {
        point_b.x = point_on_line.x - step_size_*direction_ab.x;
        point_b.y = point_on_line.y - step_size_*direction_ab.y;
        return false;
      }
      else {
        return false;
      }
    }

    distance_on_line += step_size_;
    point_on_line.x = point_on_line.x + step_size_*direction_ab.x;
    point_on_line.y = point_on_line.y + step_size_*direction_ab.y;
  }

  return true;
}

bool UGVPlanner::isPathFromRootCollisionFree(geometry_msgs::Point point_a, geometry_msgs::Point& point_b, 
                                     bool return_collision_free_point, double ignore_radius,
                                     geometry_msgs::Vector3 direction_ab, double distance_ab)
{
  ROS_DEBUG("isPathFromRootCollisionFree()");

  // Check if optional arguments are set (default/initialized values aren't valid)
  if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0){
    direction_ab = getDirection(point_a, point_b);
  }
  if(distance_ab == -1.0){
    distance_ab = getDistanceBetweenPoints(point_a, point_b);
  }

  double angle_ab = atan2(point_b.y - point_a.y, point_b.x - point_a.x);
  
  // Check first if point_b is collision free configuration
  if(footprintCost(point_b.x, point_b.y, angle_ab) == -1.0){
    return false;
  }

  double distance_on_line = 0.0;
  geometry_msgs::Point point_on_line = point_a;
  while(distance_on_line < distance_ab){
    if((footprintCost(point_on_line.x, point_on_line.y, angle_ab) == -1.0)
       and (distance_on_line > ignore_radius)){
      if (return_collision_free_point) {
        point_b.x = point_on_line.x - step_size_*direction_ab.x;
        point_b.y = point_on_line.y - step_size_*direction_ab.y;
        return false;
      }
      else {
        return false;
      }
    }

    distance_on_line += step_size_;
    point_on_line.x = point_on_line.x + step_size_*direction_ab.x;
    point_on_line.y = point_on_line.y + step_size_*direction_ab.y;
  }

  return true;
}

void UGVPlanner::optimizePath(float optimization_resolution)
{
  ROS_DEBUG("optimizePath");
  // Iterate through nodes from goal to root, trying to iteratively get a straight line path as far back as possible
  std::list<geometry_msgs::Point>::iterator point_a_it = path_.begin();
  std::list<geometry_msgs::Point>::iterator point_b_it = ++path_.begin();
  std::list<geometry_msgs::Point> optimized_path;
  optimized_path.push_back(*path_.begin());

  ROS_DEBUG("Init: Point A (%.2f, %.2f) and B (%.2f, %.2f)", point_a_it->x, point_a_it->y, point_b_it->x, point_b_it->y);

  while (point_b_it != path_.end()) {
    // If straigth path to parent of node_a is collision free, make it the new node_a
    geometry_msgs::Point point_a = *point_a_it;
    geometry_msgs::Point point_b = *point_b_it;
    ROS_DEBUG("Point A (%.2f, %.2f) and B (%.2f, %.2f)", point_a_it->x, point_a_it->y, point_b_it->x, point_b_it->y);
    if (isPathCollisionFree(point_a, point_b, false)) {
      ROS_DEBUG("Col. free");
      ++point_b_it;
      if (point_b_it == path_.end()) {
        // We iterated past the goal, go back and make a straight line to point_b (goal) from point_a
        --point_b_it;
        point_b = *point_b_it;
        // Have to interpolate points on straight line path
        geometry_msgs::Point point_on_path = point_a;
        geometry_msgs::Vector3 path_direction = getDirection(point_a, point_b);
        while (getDistanceBetweenPoints(point_on_path, point_b) > planner_max_ray_distance_) {
          point_on_path.x += optimization_resolution*planner_max_ray_distance_*path_direction.x;
          point_on_path.y += optimization_resolution*planner_max_ray_distance_*path_direction.y;
          point_on_path.z += optimization_resolution*planner_max_ray_distance_*path_direction.z;
          optimized_path.push_back(point_on_path);
          ROS_DEBUG("Made new point (%.2f, %.2f)", point_on_path.x, point_on_path.y);
        }
        // Add point_b (goal) in the end
        optimized_path.push_back(point_b);
        // move point_a and point_b "up the path" though this will end the process
        point_a_it = point_b_it;
        ++point_b_it;
      }
    }
    else {
      ROS_DEBUG("Collision");
      // New point_b was in collision, go back one and start optimizing
      --point_b_it;
      point_b = *point_b_it;
        // Have to interpolate points on straight line path
      geometry_msgs::Point point_on_path = point_a;
      geometry_msgs::Vector3 path_direction = getDirection(point_a, point_b);
      while (getDistanceBetweenPoints(point_on_path, point_b) > planner_max_ray_distance_) {
        point_on_path.x += planner_max_ray_distance_*path_direction.x;
        point_on_path.y += planner_max_ray_distance_*path_direction.y;
        point_on_path.z += planner_max_ray_distance_*path_direction.z;
        optimized_path.push_back(point_on_path);
        ROS_DEBUG("Made new point (%.2f, %.2f)", point_on_path.x, point_on_path.y);
      }
      // Add point_b in the end
      optimized_path.push_back(point_b);
      // move point_a and point_b "up the path" and repeat process
      point_a_it = point_b_it;
      ++point_b_it;
    }
  }

  // Use optimized path
  path_ = optimized_path;
}

void UGVPlanner::replanCheck()
{
  ROS_DEBUG("replanCheck()");

  if (!planner_replan_enabled_ or path_.empty()) {
    return;
  }

  bool collision_detected = false;
  geometry_msgs::Point collision_point;

  ROS_INFO("Path length %d", (int)path_.size());
  for (auto path_it = path_.begin(); path_it != path_.end(); ++path_it) {
    if (std::next(path_it,1) == path_.end()) {
      break;
    }

    ROS_DEBUG("Checking path from (%f, %f) to (%f, %f)", path_it->x, path_it->y, std::next(path_it,1)->x, std::next(path_it,1)->y);

    collision_point.x = std::next(path_it,1)->x;
    collision_point.y = std::next(path_it,1)->y;
    collision_point.z = std::next(path_it,1)->z;
    ROS_DEBUG("Point before: (%f, %f)", collision_point.x, collision_point.y);
    ROS_DEBUG("Point before: (%f, %f)", std::next(path_it,1)->x, std::next(path_it,1)->y);
    if (!isPathCollisionFree(*path_it, collision_point, true)) {
      collision_detected = true;
      visualizeSubgoal(collision_point, 1.0, 1.0, 0.0);
      ROS_DEBUG("Point after: (%f, %f)", collision_point.x, collision_point.y);
      ROS_DEBUG("Point after: (%f, %f)", std::next(path_it,1)->x, std::next(path_it,1)->y);
      break;
    }
  }

  if (collision_detected) {
    ROS_WARN("Collision detected, replanning.");
    replan();

  } else {
    ROS_DEBUG("No collision detected.");
  }
}

void UGVPlanner::replan()
{
  planner_replan_counter_++;
  if (planner_replan_counter_ >= 3) {
    ROS_WARN("Replanner exceeded threshold of %d replanning attempts for this goal, requesting new goal.", 3);
    planner_replan_counter_ = 0;
  }
  geometry_msgs::PoseStamped goal = makePoseStampedFromNode(goal_);
  pub_goal_.publish(goal);
}

/* 
*  Utility functions
*/

void UGVPlanner::getParams(ros::NodeHandle np)
{
  ROS_DEBUG("getParams()");
  np.param<std::string>("planner_world_frame", planner_world_frame_, "world");
  np.param<std::string>("robot_namespace", robot_namespace_, "/ugv");
  np.param<double>("goal_sample_probability", goal_sample_probability_, 0.1);
  np.param<double>("goal_radius", goal_radius_, 0.01);
  np.param<double>("ugv_midpoint_z", ugv_midpoint_z_, 0.35);
  np.param<bool>("use_dynamic_range", use_dynamic_range_, true);
  np.param<double>("dynamic_range_padding", dynamic_range_padding_, 2.0);
  np.param<double>("x_range_min", x_range_min_, -10.0);
  np.param<double>("x_range_max", x_range_max_,  10.0);
  np.param<double>("y_range_min", y_range_min_, -10.0);
  np.param<double>("y_range_max", y_range_max_,  10.0);
  np.param<double>("z_range_min", z_range_min_,  0.0);
  np.param<double>("z_range_max", z_range_max_,  0.0);
  np.param<double>("planner_rate", planner_rate_, 200.0);
  np.param<int>("planner_max_tree_nodes", planner_max_tree_nodes_, 1000);
  np.param<double>("planner_max_time", planner_max_time_, 2.0);
  np.param<double>("planner_max_ray_distance_", planner_max_ray_distance_, 2.0);
  np.param<bool>("planner_replan_enabled", planner_replan_enabled_, true);
  np.param("step_size", step_size_, costmap_->getResolution());
  np.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double UGVPlanner::footprintCost(double x_i, double y_i, double theta_i){
  if(!initialized_){
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
  //if we have no footprint... do nothing
  if(footprint.size() < 3)
    return -1.0;

  //check if the footprint is legal
  double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
  return footprint_cost;
}

geometry_msgs::PoseStamped UGVPlanner::makePoseStampedFromNode(Node node)
{
  ROS_DEBUG("makePoseStampedFromNode()");
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

geometry_msgs::Pose UGVPlanner::makePoseFromNode(Node node)
{
  ROS_DEBUG("makePoseFromNode()");
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

};