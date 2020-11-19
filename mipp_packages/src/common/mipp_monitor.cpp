#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "mipp_msgs/MoveVehicleAction.h"
#include "mipp_msgs/MippMonitor.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <string>
#include <utils.hpp>

enum VehicleType { UGV, UAV };

struct Vehicle
{
  ros::Subscriber sub_odom;
  // Vehicle state
  int id; // -1 of UGV, uav_id for UAVs
  VehicleType type;
  geometry_msgs::Point position;
  float yaw;
  float distance_travelled;
  // Vehicle visualization
};

struct Info
{
  ros::Time time;
  int nr_free_cells;
  int nr_occupied_cells;
  int nr_total_cells;
};

class MippMonitor
{
public:
  MippMonitor(ros::NodeHandle n, ros::NodeHandle np);
  ~MippMonitor();
private:
  void pubMonitor();
  void subOdometry(const nav_msgs::OdometryConstPtr& odom_msg, int vehicle_id);
  void subStart(const std_msgs::BoolConstPtr& start_msg);
  void subPath(const nav_msgs::PathConstPtr& path_msg);
  // Params
  int nr_of_uavs_;
  float frequency_;
  bool write_path_;
  bool read_path_;
  std::string path_bag_name_;
  rosbag::Bag path_bag_;
  // Variables
  std::vector<Vehicle> vehicles_;
  nav_msgs::Path path_;
  bool started_;
  // Publishers
  ros::Timer pub_timer_;
  ros::Publisher pub_monitor_;
  ros::Publisher pub_path_;
  // Subscribers
  ros::Subscriber sub_start_;
  ros::Subscriber sub_path_;
  // Actionlib
  actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* move_vehicle_client;
  mipp_msgs::MoveVehicleGoal move_vehicle_goal;
  // TF
};

MippMonitor::MippMonitor(ros::NodeHandle n, ros::NodeHandle np)
{
  ROS_WARN("MippMonitor object is being created.");

  np.param<int>("nr_of_uavs", nr_of_uavs_, 1);
  np.param<float>("frequency", frequency_, 10.0);
  // Write/read path for UGV for scenario 2
  np.param<bool>("write_path", write_path_, true);
  np.param<bool>("read_path", read_path_, true);
  np.param<std::string>("path_file_name", path_bag_name_, "/home/kevin/catkin_ws/src/mipp_project/mipp_launch/bags/sx2_path_1.bag");

  path_.poses.clear();
  started_ = false;

  pub_timer_    = n.createTimer(ros::Duration(1.0/frequency_), boost::bind(&MippMonitor::pubMonitor, this));
  pub_monitor_  = n.advertise<mipp_msgs::MippMonitor>("/MippMonitor/monitor", 1);
  pub_path_     = n.advertise<nav_msgs::Path>("/MippMonitor/path", 1);
  sub_start_    = n.subscribe("/MippMonitor/start", 1, &MippMonitor::subStart, this);
  sub_path_     = n.subscribe("/ugv/UGVPlanner/path", 1, &MippMonitor::subPath, this);
  std::string uav_move_vehicle_client_name = "/ugv/UGVPlanner/move_vehicle_action";
  move_vehicle_client = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(uav_move_vehicle_client_name, true);
  move_vehicle_client->waitForServer(ros::Duration(10.0));

  int nr_of_vehicles = 1 + nr_of_uavs_;
  for (int i = 0; i < nr_of_vehicles; i++) {
    Vehicle vehicle;
    vehicle.id = i-1;
    vehicle.distance_travelled = 0.0;
    if (vehicle.id != -1) {
      vehicle.type = UAV;
      vehicle.sub_odom = n.subscribe<nav_msgs::Odometry>("/gazebo/ground_truth_uav"+std::to_string(vehicle.id), 1, boost::bind(&MippMonitor::subOdometry, this, _1, vehicle.id));
    }
    else {
      vehicle.type = UGV;
      vehicle.position =  makePoint(0.0, 0.0, 0.0);
      vehicle.yaw = 0.0;
      vehicle.sub_odom = n.subscribe<nav_msgs::Odometry>("/gazebo/ground_truth_ugv", 1, boost::bind(&MippMonitor::subOdometry, this, _1, vehicle.id));
    }
    vehicles_.push_back(vehicle);
  }

  ros::Rate loop_rate(frequency_);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

MippMonitor::~MippMonitor()
{
  ROS_WARN("MippMonitor object is being deleted.");
}

void MippMonitor::pubMonitor() {
  ROS_DEBUG("pubMonitor");

  mipp_msgs::MippMonitor monitor_msg;
  monitor_msg.header.frame_id = "world";
  monitor_msg.header.stamp = ros::Time::now();
  
  for (auto const& vehicle_it : vehicles_) {
    mipp_msgs::MippMonitorVehicle monitor_vehicle;
    monitor_vehicle.id                  = vehicle_it.id;
    monitor_vehicle.position            = vehicle_it.position;
    monitor_vehicle.yaw                 = vehicle_it.yaw;
    monitor_vehicle.distance_travelled  = vehicle_it.distance_travelled;
    monitor_vehicle.fsm_state           = mipp_msgs::MippMonitorVehicle::FSM_STATE_IDLE;
    monitor_vehicle.com_state           = mipp_msgs::MippMonitorVehicle::COM_STATE_WORKING;

    if (vehicle_it.type == UGV) {
      monitor_msg.ugv = monitor_vehicle;
    }
    else {
      monitor_msg.uavs.push_back(monitor_vehicle);
    }
  }
  pub_monitor_.publish(monitor_msg);
}

void MippMonitor::subOdometry(const nav_msgs::OdometryConstPtr& odom_msg, int vehicle_id)
{
  ROS_DEBUG("subOdometry");

  // Update distance travelled before writing in new position
  vehicles_[vehicle_id+1].distance_travelled += getDistanceBetweenPoints(odom_msg->pose.pose.position, vehicles_[vehicle_id+1].position);

  // Write in new position and yaw
  vehicles_[vehicle_id+1].position = odom_msg->pose.pose.position;
  tf2::Quaternion tf_quat;
  tf2::fromMsg(odom_msg->pose.pose.orientation, tf_quat);
  geometry_msgs::Point vehicle_rpy;
  tf2::Matrix3x3(tf_quat).getRPY(vehicle_rpy.x, 
                                  vehicle_rpy.y, 
                                  vehicle_rpy.z);
  vehicles_[vehicle_id+1].yaw = vehicle_rpy.z;
  ROS_DEBUG("Got vehicle %d odom: (%.2f, %.2f, %.2f)", vehicle_id, vehicles_[vehicle_id+1].position.x, vehicles_[vehicle_id+1].position.y, vehicles_[vehicle_id+1].yaw);
}

void MippMonitor::subPath(const nav_msgs::PathConstPtr& path_msg) {
  ROS_DEBUG("subPath");

  if (write_path_ and path_.poses.empty()) {
    ROS_WARN("Writing path to bag %s", path_bag_name_.c_str());
    path_ = *path_msg;
    path_bag_.open(path_bag_name_, rosbag::bagmode::Write);
    path_bag_.write("test_path", ros::Time::now(), path_);
    path_bag_.close();
  }
}

void MippMonitor::subStart(const std_msgs::BoolConstPtr& start_msg) {
  ROS_DEBUG("subStart");

  if (!started_ and read_path_) {
    ROS_WARN("Reading path from bag %s", path_bag_name_.c_str());

    nav_msgs::Path::ConstPtr path;
    path_bag_.open(path_bag_name_, rosbag::bagmode::Read);
    for (rosbag::MessageInstance const m: rosbag::View(path_bag_)) {
      path = m.instantiate<nav_msgs::Path>();
      if (path != nullptr) {
        ROS_WARN("Read out path with length %d", (int)path->poses.size());
      }
      else {
        ROS_ERROR("Failed to read path.");
        return;
      }
    }

    move_vehicle_goal.goal_pose = *(path->poses.rbegin());
    move_vehicle_goal.goal_path.poses = path->poses;
    move_vehicle_goal.goal_path_to_be_improved = false;
    move_vehicle_goal.goal_reached_radius = 1.0;
    move_vehicle_goal.goal_reached_yaw = 0.1;
    move_vehicle_goal.goal_reached_max_time = 100.0;
    move_vehicle_client->sendGoal(move_vehicle_goal);

    started_ = true;
  }
}

int main(int argc, char** argv){
  ROS_INFO("Starting mipp_monitor_node.");

  ros::init(argc, argv, "mipp_monitor_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  MippMonitor MippMonitor(n, np);

  ros::spin();
  return 0;
};