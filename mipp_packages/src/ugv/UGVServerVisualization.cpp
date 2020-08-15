/**
* @file UGVServerVisualization.cpp
* @author Kevinmey, based on code from Ascend NTNU and vss2sn
* @brief Contains the visualization functions for the UGVServer class
*/

#include <UGVServer.hpp>

/* 
*  Visualization functions
*/
void UGVServer::visualizeTree()
{
  ROS_DEBUG("UGVServer: visualizeTree");
  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = planner_world_frame_;
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.03;
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

void UGVServer::visualizeRoot(geometry_msgs::Point point, double red, double green, double blue)
{
  ROS_DEBUG("UGVServer: visualizeRoot");
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

void UGVServer::visualizeFrontierNodes(double red, double green, double blue)
{
  ROS_DEBUG("UGVServer: visualizeFrontierNodes");
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