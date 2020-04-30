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
  pub_random_pose_ = n.advertise<geometry_msgs::PoseStamped>("rrt_planner/random_pose", 10);
  pub_viz_tree_ = n.advertise<visualization_msgs::Marker>("rrt_planner/viz_tree", 10);

  root_ = Node(root_x_, root_y_, root_z_, 0.0, 0.0, 0.0, 0, NULL);
  tree_.push_back(root_);

  std::default_random_engine generator;
  std::uniform_real_distribution<double> x_distribution(x_range_min_, x_range_max_);
  std::uniform_real_distribution<double> y_distribution(y_range_min_, y_range_max_);
  std::uniform_real_distribution<double> z_distribution(z_range_min_, z_range_max_);

  ros::Rate rate(4.0);
  while(ros::ok)
  {
    geometry_msgs::Point sample_point;
    sample_point.x = x_distribution(generator);
    sample_point.y = y_distribution(generator);
    sample_point.z = z_distribution(generator);
    pub_random_point_.publish(sample_point);

    geometry_msgs::PoseStamped sample_pose;
    sample_pose.header.frame_id = "world";
    sample_pose.header.stamp = ros::Time::now();
    sample_pose.pose.position = sample_point;
    sample_pose.pose.orientation.w = 1.0;
    pub_random_pose_.publish(sample_pose);

    int node_id = tree_.size();
    Node sample_node(sample_point.x, sample_point.y, sample_point.z, 0.0, 0.0, 0.0, node_id, &root_);
    double distance_to_parent = sample_node.findDistance(*sample_node.getParent());
    ROS_DEBUG("Sampled node distance to root: %f", distance_to_parent);
    for(std::list<Node>::iterator tree_node_it = tree_.begin();
        tree_node_it != tree_.end(); tree_node_it++)
    {
      double distance_to_node = sample_node.findDistance(*tree_node_it);
      ROS_INFO("Sampled node distance to node %d: %f", tree_node_it->id_, distance_to_node);
      if(distance_to_node < distance_to_parent) 
      {
        ROS_INFO("Sampled node is closer to node %d: %f", tree_node_it->id_, distance_to_node);
        sample_node.setParent(&*tree_node_it);
        ROS_INFO("New parent: %d.", (int)sample_node.getParent()->id_);
        distance_to_parent = distance_to_node;
      }
      ROS_INFO("parent id: %d.", (int)sample_node.getParent()->id_);
    }
    double distance_threshold = 3.0;
    if(distance_to_parent < distance_threshold)
    {
      tree_.push_back(sample_node);
      ROS_INFO("Got new node in tree. Tree size now %d.", (int)tree_.size());
      ROS_INFO("%s",sample_node.printStatus().c_str());
        ROS_INFO("Parent ID: %d.", (int)sample_node.getParent()->id_);
    }

    visualization_msgs::Marker tree_marker;
    tree_marker.header.frame_id = "world";
    tree_marker.header.stamp = ros::Time::now();
    tree_marker.id = 0;
    tree_marker.type = visualization_msgs::Marker::LINE_LIST;
    tree_marker.action = visualization_msgs::Marker::ADD;
    tree_marker.pose.orientation.w = 1.0;
    tree_marker.scale.x = 0.02;
    tree_marker.color.a = 0.8;
    tree_marker.color.r = 0.4;
    tree_marker.color.g = 0.0;
    tree_marker.color.b = 0.6;
    for(Node tree_node : tree_)
    {
      if(tree_node.getParent() != NULL) 
      {
        geometry_msgs::Point tree_node_point;
        tree_node_point.x = tree_node.x_;
        tree_node_point.y = tree_node.y_;
        tree_node_point.z = tree_node.z_;
        tree_marker.points.push_back(tree_node_point);
        geometry_msgs::Point parent_point;
        parent_point.x = tree_node.getParent()->x_;
        parent_point.y = tree_node.getParent()->y_;
        parent_point.z = tree_node.getParent()->z_;
        tree_marker.points.push_back(parent_point);
      }
    }
    pub_viz_tree_.publish(tree_marker);

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
  np.param<double>("root_x", root_x_, 0.0);
  np.param<double>("root_y", root_y_, 0.0);
  np.param<double>("root_z", root_z_, 0.0);
  np.param<double>("x_range_min", x_range_min_, -1.0);
  np.param<double>("x_range_max", x_range_max_,  1.0);
  np.param<double>("y_range_min", y_range_min_, -1.0);
  np.param<double>("y_range_max", y_range_max_,  1.0);
  np.param<double>("z_range_min", z_range_min_,  0.0);
  np.param<double>("z_range_max", z_range_max_,  1.0);
}