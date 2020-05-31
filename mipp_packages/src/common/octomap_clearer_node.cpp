#include <ros/ros.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <nav_msgs/Odometry.h>
#include <string>

class OctomapClearer
{
public:
  OctomapClearer(ros::NodeHandle n, ros::NodeHandle np);
  ~OctomapClearer();
private:
  void groundTruthCallback(const nav_msgs::OdometryConstPtr& msg);
  void clearBBX();
  // Params
  std::string vehicle_name;
  std::string vehicle_ground_truth_topic;
  double vehicle_size_x;
  double vehicle_size_y;
  double vehicle_size_z;
  double clearer_rate;
  // Publishers and transform
  ros::Subscriber sub_ground_truth;
  ros::Timer cli_timer_octomap_clearer;
  ros::ServiceClient cli_octomap_clearer;
  // Variables
  geometry_msgs::Point vehicle_position;
};

void OctomapClearer::groundTruthCallback(const nav_msgs::OdometryConstPtr& msg){
  ROS_DEBUG("OctomapClearer: groundTruthCallback");

  vehicle_position = msg->pose.pose.position;
}

void OctomapClearer::clearBBX() {
  ROS_DEBUG("OctomapClearer: clearBBX");

  octomap_msgs::BoundingBoxQuery vehicle_bbx_clear_srv;
  vehicle_bbx_clear_srv.request.min.x = vehicle_position.x - vehicle_size_x/2.0;
  vehicle_bbx_clear_srv.request.min.y = vehicle_position.y - vehicle_size_y/2.0;
  vehicle_bbx_clear_srv.request.min.z = vehicle_position.z - vehicle_size_z/2.0;
  vehicle_bbx_clear_srv.request.max.x = vehicle_position.x + vehicle_size_x/2.0;
  vehicle_bbx_clear_srv.request.max.y = vehicle_position.y + vehicle_size_y/2.0;
  vehicle_bbx_clear_srv.request.max.z = vehicle_position.z + vehicle_size_z/2.0;

  if (cli_octomap_clearer.call(vehicle_bbx_clear_srv)) {
    ROS_DEBUG("Cleared octomap at (x,y,z) = (%.1f,%.1f,%.1f)", vehicle_position.x, vehicle_position.y, vehicle_position.z);
  }
  else {
    ROS_ERROR("Failed to clear octomap at (x,y,z) = (%.1f,%.1f,%.1f)", vehicle_position.x, vehicle_position.y, vehicle_position.z);
  }
}

OctomapClearer::OctomapClearer(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_WARN("OctomapClearer object is being created.");

  np.param<std::string>("vehicle_name", vehicle_name, "");
  np.param<std::string>("vehicle_ground_truth_topic", vehicle_ground_truth_topic, vehicle_name+"/ground_truth");
  np.param<double>("vehicle_size_x", vehicle_size_x, 1.0);
  np.param<double>("vehicle_size_y", vehicle_size_y, vehicle_size_x);
  np.param<double>("vehicle_size_z", vehicle_size_z, 0.5);
  np.param<double>("clearer_rate", clearer_rate, 5.0);

  sub_ground_truth = n.subscribe(vehicle_ground_truth_topic, 1, &OctomapClearer::groundTruthCallback, this);
  cli_timer_octomap_clearer = n.createTimer(ros::Duration(1.0/clearer_rate), boost::bind(&OctomapClearer::clearBBX, this));
  cli_octomap_clearer = n.serviceClient<octomap_msgs::BoundingBoxQuery>("/octomap_server/clear_bbx");
}

OctomapClearer::~OctomapClearer()
{
  ROS_WARN("OctomapClearer object is being deleted.");
}
  

int main(int argc, char** argv){
    ROS_INFO("Starting octomap_clearer_node.");

    ros::init(argc, argv, "octomap_clearer_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    OctomapClearer octomapClearer(n, np);

    ros::spin();
    return 0;
};