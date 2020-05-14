/**
* @file UGVPlannerVisualization.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Contains the visualization functions for the UGVPlanner class
*/

#include <UGVPlanner.hpp>

namespace ugv_planner {

/* 
*  Visualization functions
*/
void UGVPlanner::visualizeTree()
{
  ROS_DEBUG("UGVPlanner: visualizeTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;
  tree_marker.color.a = 0.5;
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

void UGVPlanner::visualizeTreeColored()
{
  ROS_DEBUG("UGVPlanner: visualizeTreeColored");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;
  tree_marker.color.a = 0.5;
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

void UGVPlanner::visualizeCollisionTree()
{
  ROS_DEBUG("UGVPlanner: visualizeCollisionTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.04;
  tree_marker.color.a = 0.5;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree_;
  pub_viz_collision_tree_.publish(tree_marker);
}

void UGVPlanner::visualizeCollisionTree(std::vector<geometry_msgs::Point> collision_tree)
{
  ROS_DEBUG("UGVPlanner: visualizeCollisionTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.04;
  tree_marker.color.a = 1.0;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree;
  pub_viz_collision_tree_.publish(tree_marker);
}

void UGVPlanner::visualizePathToGoal()
{
  ROS_DEBUG("UGVPlanner: visualizePathToGoal");
  if(goal_.cost_ == INFINITY){
    return;
  }
  
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.05;
  tree_marker.color.a = 1.0;
  tree_marker.color.r = 0.1;
  tree_marker.color.g = 1.0;
  tree_marker.color.b = 0.1;

  Node node_on_goal_path = goal_;
  while(node_on_goal_path.id_ != 0)
  {
    tree_marker.points.push_back(node_on_goal_path.position_);
    tree_marker.points.push_back(node_on_goal_path.getParent()->position_);
    node_on_goal_path = *node_on_goal_path.getParent();
  }
  
  pub_viz_path_to_goal_.publish(tree_marker);
}

void UGVPlanner::visualizeRoot(geometry_msgs::Point point, double red, double green, double blue)
{
  ROS_DEBUG("UGVPlanner: visualizeRoot");
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

void UGVPlanner::visualizeGoal(geometry_msgs::Point point, double red, double green, double blue)
{
  ROS_DEBUG("UGVPlanner: visualizeGoal");
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
  
  pub_viz_goal_node_.publish(marker_point);
}

void UGVPlanner::visualizeSubgoal(geometry_msgs::Point point, double red, double green, double blue)
{
  ROS_DEBUG("UGVPlanner: visualizeSubgoal");
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
  
  pub_viz_subgoal_node_.publish(marker_point);
}

void UGVPlanner::visualizeFrontierNodes(double red, double green, double blue)
{
  ROS_DEBUG("UGVPlanner: visualizeFrontierNodes");
  visualization_msgs::Marker m;

  m.header.frame_id = planner_world_frame_;
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE_LIST;
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

  std::map<double, Node>::iterator frontier_node_itr;
  for(frontier_node_itr = frontier_nodes_.begin(); frontier_node_itr != frontier_nodes_.end(); ++frontier_node_itr){
    m.points.push_back(frontier_node_itr->second.position_);
  }
  
  pub_viz_frontier_nodes_.publish(m);
}

};