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
  tree_marker.color.a = 0.8;
  tree_marker.color.r = 1.0;
  tree_marker.color.g = 0.0;
  tree_marker.color.b = 0.0;
  tree_marker.points = collision_tree;
  pub_viz_collision_tree_.publish(tree_marker);
}