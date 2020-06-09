#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <utils.hpp>

class ComConstraintVisualizer
{
public:
  ComConstraintVisualizer(ros::NodeHandle n, ros::NodeHandle np);
  ~ComConstraintVisualizer();
private:
  void vehiclePoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
  void baseStationPoseCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void visualizeConstraint();
  // Params
  std::string world_frame_;
  std::string vehicle_pose_topic_;
  std::string base_station_odom_topic_;
  double communication_range_;
  double publish_rate_;
  int counter_ceiling_;
  // Variables 
  geometry_msgs::PoseStamped vehicle_pose_;
  geometry_msgs::PoseStamped base_station_pose_;
  int range_counter_;
  int los_counter_;
  // Publishers and transform
  ros::Publisher pub_viz_constraint_;
  ros::Subscriber sub_vehicle_pose_;
  ros::Subscriber sub_base_station_pose_;
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

void ComConstraintVisualizer::visualizeConstraint()
{
  // Range constraint
  double vehicle_base_station_distance = getDistanceBetweenPoints(vehicle_pose_.pose.position, base_station_pose_.pose.position);
  if (vehicle_base_station_distance < communication_range_) {
    range_counter_ = std::min(range_counter_ + 1, counter_ceiling_);
  }
  else {
    range_counter_ = std::max(range_counter_ - 1, 0);
  }

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
  constraint_marker.points.push_back(vehicle_pose_.pose.position);
  constraint_marker.points.push_back(base_station_pose_.pose.position);
  pub_viz_constraint_.publish(constraint_marker);
}

ComConstraintVisualizer::ComConstraintVisualizer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_WARN("ComConstraintVisualizer object is being created.");

  np.param<std::string>("world_frame", world_frame_, "world");
  np.param<std::string>("vehicle_pose_topic", vehicle_pose_topic_, "vehicle_odometry");
  np.param<std::string>("base_station_odom_topic", base_station_odom_topic_, "base_station_odometry");
  np.param<double>("communication_range", communication_range_, 10.0);
  np.param<double>("publish_rate", publish_rate_, 10.0);
  np.param<int>("counter_ceiling", counter_ceiling_, 5);

  sub_vehicle_pose_ = n.subscribe(vehicle_pose_topic_, 1, &ComConstraintVisualizer::vehiclePoseCallback, this);
  sub_base_station_pose_ = n.subscribe(base_station_odom_topic_, 1, &ComConstraintVisualizer::baseStationPoseCallback, this);
  pub_viz_constraint_ = n.advertise<visualization_msgs::Marker>("viz_constraint", 1);
  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  ros::Rate loop_rate(publish_rate_);
  while (ros::ok())
  {
    visualizeConstraint();

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