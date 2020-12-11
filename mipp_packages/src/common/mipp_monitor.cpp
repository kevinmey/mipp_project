#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "mipp_msgs/StartMippAction.h"
#include "mipp_msgs/MoveVehicleAction.h"
#include "mipp_msgs/MippMonitor.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
  void startMipp();
  // Params
  bool auto_start_;
  int nr_of_uavs_;
  float frequency_;
  bool write_path_;
  bool read_path_;
  bool read_tour_;
  int planner_mode_;
  std::string path_bag_name_;
  rosbag::Bag path_bag_;
  std::vector<std::string> tour_bag_names_;
  std::vector<rosbag::Bag> tour_bags_;
  // Variables
  std::vector<Vehicle> vehicles_;
  nav_msgs::Path path_;
  std::vector<nav_msgs::Path> tour_;
  bool started_;
  // Publishers
  ros::Timer pub_timer_;
  ros::Publisher pub_monitor_;
  ros::Publisher pub_path_;
  ros::Publisher pub_viz_tour_;
  // Subscribers
  ros::Subscriber sub_start_;
  ros::Subscriber sub_path_;
  // Services
  ros::ServiceClient cli_planner_ready_;
  // Actionlib
  actionlib::SimpleActionClient<mipp_msgs::StartMippAction>* start_mipp_client;
  mipp_msgs::StartMippGoal start_mipp_goal;
  actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>* move_vehicle_client;
  mipp_msgs::MoveVehicleGoal move_vehicle_goal;
  // TF
};

MippMonitor::MippMonitor(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_WARN("MippMonitor object is being createdi.");

  int count = 0;

  np.param<int>("planner_mode", planner_mode_, 0);
  np.param<bool>("auto_start", auto_start_, true);
  np.param<int>("nr_of_uavs", nr_of_uavs_, 1);
  np.param<float>("frequency", frequency_, 10.0);
  // Write/read path for UGV for scenario 2
  np.param<bool>("write_path", write_path_, true);
  np.param<bool>("read_path", read_path_, true);
  np.param<std::string>("path_file_name", path_bag_name_, "$(find mipp_launch)/bags/sc2_path_1.bag");
  np.param<bool>("read_tour", read_tour_, false);
  np.param("tour_file_names", tour_bag_names_, std::vector<std::string>());

  path_.poses.clear();
  started_ = false;

  pub_timer_    = n.createTimer(ros::Duration(1.0/frequency_), boost::bind(&MippMonitor::pubMonitor, this));
  pub_monitor_  = n.advertise<mipp_msgs::MippMonitor>("/MippMonitor/monitor", 1);
  pub_path_     = n.advertise<nav_msgs::Path>("/MippMonitor/path", 1);
  pub_viz_tour_ = n.advertise<visualization_msgs::Marker>("/MippMonitor/viz_tour", 1);

  sub_start_    = n.subscribe("/MippMonitor/start", 1, &MippMonitor::subStart, this);
  sub_path_     = n.subscribe("/ugv/UGVPlanner/path", 1, &MippMonitor::subPath, this);

  std::string cli_planner_ready_name = "/MippPlanner/planner_ready";
  cli_planner_ready_ = n.serviceClient<std_srvs::SetBool>(cli_planner_ready_name);

  std::string start_mipp_client_name = "/MippPlanner/start_mipp_action";
  start_mipp_client = new actionlib::SimpleActionClient<mipp_msgs::StartMippAction>(start_mipp_client_name, true);
  start_mipp_client->waitForServer(ros::Duration(10.0));

  std::string move_vehicle_client_name = "/ugv/UGVPlanner/move_vehicle_action";
  move_vehicle_client = new actionlib::SimpleActionClient<mipp_msgs::MoveVehicleAction>(move_vehicle_client_name, true);
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
    if (auto_start_ and !started_) {
      std_srvs::SetBool srv;
      if (cli_planner_ready_.call(srv)) {
        if (srv.response.success) startMipp();
        else ROS_INFO_THROTTLE(5.0, "Mipp planner not ready... yet");
      }
      else {
        ROS_ERROR("Couldn't call planner_ready server.");
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

MippMonitor::~MippMonitor() {
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
    path_.poses.clear();
  }
}

void MippMonitor::subStart(const std_msgs::BoolConstPtr& start_msg) {
  ROS_DEBUG("subStart");

  startMipp();
}

void MippMonitor::startMipp() {

  if (started_) {
    ROS_WARN("Already started");
    return;
  }

  if (read_path_) {
    ROS_INFO("Reading path from bag %s", path_bag_name_.c_str());

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
    path_bag_.close();

    bool visualize_path = true;
    if (visualize_path) {
      visualization_msgs::Marker tour_marker;
      tour_marker.header.frame_id = "world";
      tour_marker.header.stamp = ros::Time::now();
      tour_marker.id = 0;
      tour_marker.type = visualization_msgs::Marker::LINE_STRIP;
      tour_marker.action = visualization_msgs::Marker::ADD;
      tour_marker.pose.orientation.w = 1.0;
      tour_marker.scale.x = 0.2;
      tour_marker.color.a = 0.5;
      tour_marker.color.r = 0.1;
      tour_marker.color.g = 1.0;
      tour_marker.color.b = 0.1;
      for (auto const& pose_it : path->poses) {
        tour_marker.points.push_back(pose_it.pose.position);
      }
      pub_viz_tour_.publish(tour_marker);
    }

    move_vehicle_goal.goal_pose = *(path->poses.rbegin());
    move_vehicle_goal.goal_path.poses = path->poses;
    move_vehicle_goal.goal_path_to_be_improved = false;
    move_vehicle_goal.goal_reached_radius = 1.0;
    move_vehicle_goal.goal_reached_yaw = 0.1;
    move_vehicle_goal.goal_reached_max_time = 100.0;
    move_vehicle_client->sendGoal(move_vehicle_goal);

    start_mipp_goal.max_time = 1000.0;
    start_mipp_goal.mipp_mode = planner_mode_;
    start_mipp_client->sendGoal(start_mipp_goal);

    started_ = true;
  }
  else if (read_tour_) {
    ROS_INFO("Reading tour, got %d file names", (int)tour_bag_names_.size());
    for (auto const& tour_bag_name : tour_bag_names_) {
      ROS_INFO("Reading tour bag: %s", tour_bag_name.c_str());
      nav_msgs::Path::ConstPtr path;
      rosbag::Bag tour_bag;
      tour_bag.open(tour_bag_name, rosbag::bagmode::Read);
      for (rosbag::MessageInstance const m: rosbag::View(tour_bag)) {
        path = m.instantiate<nav_msgs::Path>();
        if (path != nullptr) {
          ROS_WARN("Read out tour with length %d", (int)path->poses.size());
        }
        else {
          ROS_ERROR("Failed to read tour.");
          return;
        }
      }
      tour_bag.close();
      //tour_bags_.push_back(tour_bag);
      tour_.push_back(*path);
    }

    bool visualize_tour = true;
    if (visualize_tour) {
      visualization_msgs::Marker tour_marker;
      tour_marker.header.frame_id = "world";
      tour_marker.header.stamp = ros::Time::now();
      tour_marker.id = 0;
      tour_marker.type = visualization_msgs::Marker::LINE_STRIP;
      tour_marker.action = visualization_msgs::Marker::ADD;
      tour_marker.pose.orientation.w = 1.0;
      tour_marker.scale.x = 0.2;
      tour_marker.color.a = 0.5;
      tour_marker.color.r = 0.1;
      tour_marker.color.g = 1.0;
      tour_marker.color.b = 0.1;
      for (auto const& tour_path_it : tour_) {
        for (auto const& pose_it : tour_path_it.poses) {
          tour_marker.points.push_back(pose_it.pose.position);
        }
      }
      pub_viz_tour_.publish(tour_marker);
    }

    started_ = true;
    float goal_wait_time = 10.0;
    int goal_nr = 1;
    start_mipp_goal.max_time = 1000.0;
    start_mipp_goal.mipp_mode = planner_mode_;
    start_mipp_client->sendGoal(start_mipp_goal);
    for (auto const& tour_path : tour_) {
      ROS_DEBUG("Tour: Goal %d with path of length %d", goal_nr, (int)tour_path.poses.size());
          
      move_vehicle_goal.goal_pose = *(tour_path.poses.rbegin());
      move_vehicle_goal.goal_pose.pose.orientation = makeQuatFromYaw(M_PI/2.0);
      move_vehicle_goal.goal_path.poses = tour_path.poses;
      move_vehicle_goal.goal_path_to_be_improved = false;
      move_vehicle_goal.goal_reached_radius = 0.5;
      move_vehicle_goal.goal_reached_yaw = M_PI/6.0;
      move_vehicle_goal.goal_reached_max_time = 120.0;
      move_vehicle_client->sendGoal(move_vehicle_goal);

      while (!move_vehicle_client->getState().isDone()) {
        ROS_INFO_THROTTLE(1.0, "Moving to goal nr. %d", goal_nr);
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }

      ROS_INFO("Goal nr. %d reached, waiting %.1f seconds until next goal.", goal_nr, goal_wait_time);
      ros::Time wait_start_time = ros::Time::now();
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      while ((ros::Time::now() - wait_start_time).toSec() < goal_wait_time) {
        ROS_INFO_THROTTLE(1.0, "Waiting... %d", (int)(ros::Time::now() - wait_start_time).toSec());
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      goal_nr++;
    }
    ROS_WARN("Tour complete");
  }
  else {
    ROS_WARN("Didn't read path or tour");
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