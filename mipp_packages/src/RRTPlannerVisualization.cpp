/**
* @file RRTPlannerVisualization.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Contains the visualization functions for the RRTPlanner class
*/

#include <RRTPlanner.hpp>

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
  tree_marker.color.a = 0.4;
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

void RRTPlanner::visualizeTreeColored()
{
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;
  tree_marker.color.a = 0.4;
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
  tree_marker.color.a = 0.4;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree_;
  pub_viz_collision_tree_.publish(tree_marker);
}

void RRTPlanner::visualizeCollisionTree(std::vector<geometry_msgs::Point> collision_tree)
{
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.04;
  tree_marker.color.a = 0.4;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree;
  pub_viz_collision_tree_.publish(tree_marker);
}

void RRTPlanner::visualizePathToGoal()
{
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

void RRTPlanner::visualizeRoot(geometry_msgs::Point point, double red, double green, double blue)
{
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

void RRTPlanner::visualizeGoal(geometry_msgs::Point point, double red, double green, double blue)
{
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