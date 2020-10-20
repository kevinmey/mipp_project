#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <octomap_msgs/Octomap.h>
#include "octomap_msgs/conversions.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <utils.hpp>

#include <mipp_msgs/CommunicationState.h>

class ComConstraintVisualizer
{
public:
  ComConstraintVisualizer(ros::NodeHandle n, ros::NodeHandle np);
  ~ComConstraintVisualizer();
private:
  void vehiclePoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void baseStationPoseCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg);
  void updateConstraint(mipp_msgs::CommunicationState& com_state);
  // Params
  int vehicle_id_;
  std::string world_frame_;
  std::string vehicle_pose_topic_;
  std::string base_station_odom_topic_;
  double base_station_antenna_height_;
  double communication_range_;
  double publish_rate_;
  int counter_ceiling_;
  bool unmapped_is_occupied_;
  // Variables 
  std::shared_ptr<octomap::OcTree> map_;
  bool received_map_;
  geometry_msgs::PoseStamped vehicle_pose_;
  geometry_msgs::PoseStamped base_station_pose_;
  int range_counter_;
  int los_counter_;
  bool has_range_;
  bool has_los_;
  // Publishers and transform
  ros::Publisher pub_constraint_state_;
  ros::Publisher pub_viz_constraint_;
  ros::Subscriber sub_vehicle_pose_;
  ros::Subscriber sub_base_station_pose_;
  ros::Subscriber sub_octomap_;
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_; 
};

void ComConstraintVisualizer::vehiclePoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg){
  ROS_DEBUG("ComConstraintVisualizer: vehiclePoseCallback");
  try{
    // Transform pose information contained in odom message to world frame and store
    geometry_msgs::TransformStamped odometry_tf = tf_buffer_.lookupTransform(world_frame_, pose_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(*pose_msg, vehicle_pose_, odometry_tf);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void ComConstraintVisualizer::baseStationPoseCallback(const nav_msgs::OdometryConstPtr& odom_msg){
  ROS_DEBUG("ComConstraintVisualizer: baseStationPoseCallback");
  // Only need pose information (twist/velocities not needed)
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = odom_msg->header;
  pose_msg.pose = odom_msg->pose.pose;
  pose_msg.pose.position.z = base_station_antenna_height_;
  try{
    // Transform pose information contained in odom message to world frame and store
    geometry_msgs::TransformStamped odometry_tf = tf_buffer_.lookupTransform(world_frame_, odom_msg->header.frame_id, ros::Time(0));
    tf2::doTransform(pose_msg, base_station_pose_, odometry_tf);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

void ComConstraintVisualizer::octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg) {
  ROS_DEBUG("ComConstraintVisualizer: octomapCallback");
  map_ = std::shared_ptr<octomap::OcTree> (dynamic_cast<octomap::OcTree*> (octomap_msgs::msgToMap(*octomap_msg)));
  received_map_ = true;
}

void ComConstraintVisualizer::updateConstraint(mipp_msgs::CommunicationState& com_state)
{
  geometry_msgs::Point communication_endpoint = vehicle_pose_.pose.position;

  // First update the com_state according to bool variables, since com_state as a class member variable causes errors
  if (has_range_ and has_los_) {
    com_state.state = mipp_msgs::CommunicationState::STATE_WORKING;
  }
  else if (!has_range_ and has_los_) {
    com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_BROKEN;
  }
  else if (has_range_ and !has_los_) {
    com_state.state = mipp_msgs::CommunicationState::STATE_LOS_BROKEN;
  }
  else {
    com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN;
  }
  
  // Range constraint
  double vehicle_base_station_distance = getDistanceBetweenPoints(vehicle_pose_.pose.position, base_station_pose_.pose.position);
  if (vehicle_base_station_distance < communication_range_) {
    range_counter_ = std::min(range_counter_ + 1, counter_ceiling_);
  }
  else {
    range_counter_ = std::max(range_counter_ - 1, 0);
  }
  if (range_counter_ == 0) {
    // Range constraint broken
    has_range_ = false;
    if (com_state.state ==  mipp_msgs::CommunicationState::STATE_LOS_BROKEN) {
      // LOS constraint was already broken, now both are
      com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN;
    }
    else if (com_state.state == mipp_msgs::CommunicationState::STATE_WORKING) {
      // Communication was working, so only range broken and LOS still working
      com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_BROKEN;
    }
  }
  else if (range_counter_ == 5) {
    // Range constraint regained
    has_range_ = true;
    if (com_state.state == mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN) {
      // LOS constraint is still broken
      com_state.state = mipp_msgs::CommunicationState::STATE_LOS_BROKEN;
    }
    else if (com_state.state == mipp_msgs::CommunicationState::STATE_RANGE_BROKEN) {
      // Only Range was broken, com now working fully
      com_state.state = mipp_msgs::CommunicationState::STATE_WORKING;
    }
  }

  // Line of Sight (LoS) constraint
  if (received_map_) {
    bool hit_occupied = false;
    bool hit_unmapped = false;
    geometry_msgs::Vector3 direction_ab = getDirection(base_station_pose_.pose.position, vehicle_pose_.pose.position);
    octomap::point3d om_ray_origin = octomap::point3d(base_station_pose_.pose.position.x, base_station_pose_.pose.position.y, base_station_pose_.pose.position.z);
    octomap::point3d om_ray_direction = octomap::point3d(direction_ab.x, direction_ab.y, direction_ab.z);
    octomap::point3d om_ray_end_cell;
    geometry_msgs::Point ray_endpoint;
    if(direction_ab.x == 0.0 and direction_ab.y == 0.0 and direction_ab.z == 0.0) {
      // Check if direction is valid
      ROS_WARN("UAVServer: Got request to cast ray in illegal direction.");
      ROS_WARN("            Origin (x,y,z) = (%f,%f,%f)",vehicle_pose_.pose.position.x,vehicle_pose_.pose.position.y,vehicle_pose_.pose.position.z);
      ROS_WARN("            End    (x,y,z) = (%f,%f,%f)",base_station_pose_.pose.position.x,base_station_pose_.pose.position.y,base_station_pose_.pose.position.z);
    }
    else {
      hit_occupied = map_->castRay(om_ray_origin, om_ray_direction, om_ray_end_cell, !unmapped_is_occupied_, vehicle_base_station_distance);
      ray_endpoint.x = om_ray_end_cell.x();
      ray_endpoint.y = om_ray_end_cell.y();
      ray_endpoint.z = om_ray_end_cell.z();
      octomap::OcTreeNode* om_ray_end_node = map_->search(om_ray_end_cell);
      if (om_ray_end_node == NULL) {
        hit_unmapped = true;
      }
    }
    if (hit_occupied or (hit_unmapped and unmapped_is_occupied_)) {
      los_counter_ = std::max(los_counter_ - 1, 0);
    }
    else {
      los_counter_ = std::min(los_counter_ + 1, counter_ceiling_);
    }
    
    if (los_counter_ == counter_ceiling_ and has_los_ == false) {
      has_los_ = true;
      ROS_INFO("LoS communication gained");
      if (com_state.state = mipp_msgs::CommunicationState::STATE_LOS_BROKEN) {
        com_state.state = mipp_msgs::CommunicationState::STATE_WORKING;
      }
      else if (com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN) {
        com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_BROKEN;
      }
    }
    else if (los_counter_ == 0 and has_los_ == true) {
      has_los_ = false;
      ROS_INFO("LoS communication lost");
      if (com_state.state = mipp_msgs::CommunicationState::STATE_WORKING) {
        com_state.state = mipp_msgs::CommunicationState::STATE_LOS_BROKEN;
      }
      else if (com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_BROKEN) {
        com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN;
      }
    }
    ROS_DEBUG("los_counter_ = %d\nhit_occupied = %d\nhit_unmapped = %d", los_counter_, hit_occupied, hit_unmapped);
    if (!has_los_) {
      communication_endpoint = ray_endpoint;
    }
  }

  // Publish mipp_msgs::CommunicationState::STATE
  com_state.base_station_position = base_station_pose_.pose.position;
  com_state.vehicle_position = vehicle_pose_.pose.position;
  com_state.los_lost_position = communication_endpoint;
  pub_constraint_state_.publish(com_state);

  // Visualize
  visualization_msgs::Marker constraint_marker;
  constraint_marker.header.frame_id = world_frame_;
  constraint_marker.header.stamp = ros::Time::now();
  constraint_marker.id = 0;
  constraint_marker.type = visualization_msgs::Marker::LINE_LIST;
  constraint_marker.action = visualization_msgs::Marker::ADD;
  constraint_marker.pose.orientation.w = 1.0;
  constraint_marker.scale.x = 0.03;
  constraint_marker.color.a = 0.8;
  constraint_marker.color.r = 1.0 - (1.0/(double)counter_ceiling_)*range_counter_;
  constraint_marker.color.g = 0.0 + (1.0/(double)counter_ceiling_)*range_counter_;
  constraint_marker.color.b = 0.0;
  constraint_marker.points.push_back(base_station_pose_.pose.position);
  constraint_marker.points.push_back(communication_endpoint);
  pub_viz_constraint_.publish(constraint_marker);
}

ComConstraintVisualizer::ComConstraintVisualizer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_WARN("ComConstraintVisualizer object is being created.");
  
  // Node sometimes crashes for no reason, wait a little
  ros::Duration(1.0).sleep();

  np.param<int>("vehicle_id", vehicle_id_, 0);
  np.param<std::string>("world_frame", world_frame_, "world");
  np.param<std::string>("vehicle_pose_topic", vehicle_pose_topic_, "vehicle_odometry");
  np.param<std::string>("base_station_odom_topic", base_station_odom_topic_, "base_station_odometry");
  np.param<double>("base_station_antenna_height", base_station_antenna_height_, 1.0);
  np.param<double>("communication_range", communication_range_, 10.0);
  np.param<double>("publish_rate", publish_rate_, 10.0);
  np.param<int>("counter_ceiling", counter_ceiling_, 5);
  np.param<bool>("unmapped_is_occupied", unmapped_is_occupied_, false);

  sub_vehicle_pose_ = n.subscribe(vehicle_pose_topic_, 1, &ComConstraintVisualizer::vehiclePoseCallback, this);
  sub_base_station_pose_ = n.subscribe(base_station_odom_topic_, 1, &ComConstraintVisualizer::baseStationPoseCallback, this);
  sub_octomap_        = n.subscribe("/octomap_binary", 1, &ComConstraintVisualizer::octomapCallback, this);
  pub_constraint_state_ = n.advertise<mipp_msgs::CommunicationState>("ComConstraintVisualizer/constraint_state", 1);
  pub_viz_constraint_ = n.advertise<visualization_msgs::Marker>("viz_constraint", 1);
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  mipp_msgs::CommunicationState com_state;
  com_state.state = mipp_msgs::CommunicationState::STATE_WORKING;
  com_state.vehicle_id = vehicle_id_;
  range_counter_ = 0;
  los_counter_ = 0;

  ros::Rate loop_rate(publish_rate_);
  while (ros::ok())
  {
    updateConstraint(com_state);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

ComConstraintVisualizer::~ComConstraintVisualizer()
{
  ROS_WARN("ComConstraintVisualizer object is being deleted.");
}
  

int main(int argc, char** argv){
    ROS_INFO("Starting communication_constraint_visualizer_node.");

    ros::init(argc, argv, "communication_constraint_visualizer_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    ComConstraintVisualizer comConstraintVisualizer(n, np);

    ros::spin();
    return 0;
};