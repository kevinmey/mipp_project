#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

class GroundTruthHandler
{
public:
  GroundTruthHandler(ros::NodeHandle n, ros::NodeHandle np);
  ~GroundTruthHandler();
private:
  void groundTruthCallback(const nav_msgs::OdometryConstPtr& msg);
  // Params
  std::string vehicle_name;
  std::string vehicle_map_frame;
  std::string vehicle_local_origin_frame;
  std::string vehicle_footprint_frame;
  std::string vehicle_link_frame;
  std::string vehicle_ground_truth_topic;
  std::string vehicle_pos_topic;
  std::string vehicle_vel_topic;
  double vehicle_origin_offset_x;
  double vehicle_origin_offset_y;
  bool publish_footprint_tf;
  // Publishers and transform
  ros::Publisher pub_vehicle_pos;
  ros::Publisher pub_vehicle_vel;
  ros::Subscriber sub_ground_truth;
  tf2_ros::TransformBroadcaster tf_broadcaster;
};

void GroundTruthHandler::groundTruthCallback(const nav_msgs::OdometryConstPtr& msg){
  ROS_DEBUG("GroundTruthHandler: groundTruthCallback");

  nav_msgs::Odometry local_msg;
  local_msg.header = msg->header;
  local_msg.child_frame_id = msg->child_frame_id;
  local_msg.pose = msg->pose;
  local_msg.twist = msg->twist;

  local_msg.pose.pose.position.x -= vehicle_origin_offset_x;
  local_msg.pose.pose.position.y -= vehicle_origin_offset_y;
  
  geometry_msgs::TransformStamped map_origin_tf;
  map_origin_tf.header.stamp = local_msg.header.stamp;
  map_origin_tf.header.frame_id = vehicle_map_frame;
  map_origin_tf.child_frame_id = vehicle_local_origin_frame;
  map_origin_tf.transform.translation.x = vehicle_origin_offset_x;
  map_origin_tf.transform.translation.y = vehicle_origin_offset_y;
  map_origin_tf.transform.translation.z = 0.0;
  map_origin_tf.transform.rotation.x = 0.0;
  map_origin_tf.transform.rotation.y = 0.0;
  map_origin_tf.transform.rotation.z = 0.0;
  map_origin_tf.transform.rotation.w = 1.0;

  tf_broadcaster.sendTransform(map_origin_tf);

  if (publish_footprint_tf) {
    geometry_msgs::TransformStamped origin_footprint_tf;
    origin_footprint_tf.header.stamp = local_msg.header.stamp;
    origin_footprint_tf.header.frame_id = vehicle_local_origin_frame;
    origin_footprint_tf.child_frame_id = vehicle_footprint_frame;
    origin_footprint_tf.transform.translation.x = local_msg.pose.pose.position.x;
    origin_footprint_tf.transform.translation.y = local_msg.pose.pose.position.y;
    origin_footprint_tf.transform.translation.z = 0.0;
    origin_footprint_tf.transform.rotation.x = 0.0;
    origin_footprint_tf.transform.rotation.y = 0.0;
    origin_footprint_tf.transform.rotation.z = 0.0;
    origin_footprint_tf.transform.rotation.w = 1.0;

    geometry_msgs::TransformStamped footprint_link_tf;
    footprint_link_tf.header.stamp = local_msg.header.stamp;
    footprint_link_tf.header.frame_id = vehicle_footprint_frame;
    footprint_link_tf.child_frame_id = vehicle_link_frame;
    footprint_link_tf.transform.translation.x = 0.0;
    footprint_link_tf.transform.translation.y = 0.0;
    footprint_link_tf.transform.translation.z = local_msg.pose.pose.position.z;
    footprint_link_tf.transform.rotation.x = local_msg.pose.pose.orientation.x;
    footprint_link_tf.transform.rotation.y = local_msg.pose.pose.orientation.y;
    footprint_link_tf.transform.rotation.z = local_msg.pose.pose.orientation.z;
    footprint_link_tf.transform.rotation.w = local_msg.pose.pose.orientation.w;

    tf_broadcaster.sendTransform(origin_footprint_tf);
    tf_broadcaster.sendTransform(footprint_link_tf);
  }
  else {
    geometry_msgs::TransformStamped origin_link_tf;
    origin_link_tf.header.stamp = local_msg.header.stamp;
    origin_link_tf.header.frame_id = vehicle_local_origin_frame;
    origin_link_tf.child_frame_id = vehicle_link_frame;
    origin_link_tf.transform.translation.x = local_msg.pose.pose.position.x;
    origin_link_tf.transform.translation.y = local_msg.pose.pose.position.y;
    origin_link_tf.transform.translation.z = local_msg.pose.pose.position.z;
    origin_link_tf.transform.rotation.x = local_msg.pose.pose.orientation.x;
    origin_link_tf.transform.rotation.y = local_msg.pose.pose.orientation.y;
    origin_link_tf.transform.rotation.z = local_msg.pose.pose.orientation.z;
    origin_link_tf.transform.rotation.w = local_msg.pose.pose.orientation.w;

    tf_broadcaster.sendTransform(origin_link_tf);
  }

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = local_msg.header.stamp;
  pose_msg.header.frame_id = vehicle_local_origin_frame;
  pose_msg.pose = local_msg.pose.pose;

  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = local_msg.header.stamp;
  twist_msg.header.frame_id = vehicle_local_origin_frame;
  twist_msg.twist = local_msg.twist.twist;

  pub_vehicle_pos.publish(pose_msg);
  pub_vehicle_vel.publish(twist_msg);
}

GroundTruthHandler::GroundTruthHandler(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_WARN("GroundTruthHandler object is being created.");

  np.param<std::string>("vehicle_name", vehicle_name, "");
  np.param<bool>("publish_footprint_tf", publish_footprint_tf, true);
  np.param<std::string>("vehicle_map_frame", vehicle_map_frame, "map");
  np.param<std::string>("vehicle_local_origin_frame", vehicle_local_origin_frame, "odom_"+vehicle_name);
  np.param<std::string>("vehicle_footprint_frame", vehicle_footprint_frame, "base_footprint_"+vehicle_name);
  np.param<std::string>("vehicle_link_frame", vehicle_link_frame, "base_link_"+vehicle_name);
  np.param<std::string>("vehicle_ground_truth_topic", vehicle_ground_truth_topic, vehicle_name+"/ground_truth");
  np.param<std::string>("vehicle_pos_topic", vehicle_pos_topic, vehicle_name+"/pose");
  np.param<std::string>("vehicle_vel_topic", vehicle_vel_topic, vehicle_name+"/velocity");
  np.param<double>("vehicle_origin_offset_x", vehicle_origin_offset_x, 0.0);
  np.param<double>("vehicle_origin_offset_y", vehicle_origin_offset_y, 0.0);
  
  pub_vehicle_pos = n.advertise<geometry_msgs::PoseStamped>(vehicle_pos_topic, 1);
  pub_vehicle_vel = n.advertise<geometry_msgs::TwistStamped>(vehicle_vel_topic, 1);

  sub_ground_truth = n.subscribe(vehicle_ground_truth_topic, 1, &GroundTruthHandler::groundTruthCallback, this);
}

GroundTruthHandler::~GroundTruthHandler()
{
  ROS_WARN("GroundTruthHandler object is being deleted.");
}
  

int main(int argc, char** argv){
    ROS_INFO("Starting ground_truth_handler_node.");

    ros::init(argc, argv, "ground_truth_handler_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    GroundTruthHandler groundTruthHandler(n, np);

    ros::spin();
    return 0;
};