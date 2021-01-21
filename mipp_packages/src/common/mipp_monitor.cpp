#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "mipp_msgs/StartMippAction.h"
#include "mipp_msgs/MoveVehicleAction.h"
#include "mipp_msgs/MippMonitor.h"
#include "mipp_msgs/CommunicationState.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <string>
#include <utils.hpp>
#include <fstream>
#include <chrono>
#include <ctime> 

enum VehicleType { UGV, UAV };

struct Vehicle
{
  ros::Subscriber sub_com;
  ros::Subscriber sub_odom;
  // Vehicle state
  int id; // -1 of UGV, uav_id for UAVs
  VehicleType type;
  geometry_msgs::Point position;
  geometry_msgs::Point prev_position;
  float yaw;
  float prev_yaw;
  float distance_travelled;
  float yaw_travelled;
  mipp_msgs::CommunicationState com_state;
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
  void subCom(const mipp_msgs::CommunicationStateConstPtr& com_msg, int vehicle_id);
  void subOdometry(const nav_msgs::OdometryConstPtr& odom_msg, int vehicle_id);
  void subStart(const std_msgs::BoolConstPtr& start_msg);
  void subPath(const nav_msgs::PathConstPtr& path_msg);
  void cliGetOctomap();
  void startMipp();
  // Params
  std::string world_name_;
  bool auto_start_;
  float ugv_vel_;
  int nr_of_uavs_;
  float frequency_;
  float octomap_resolution_;
  bool write_path_;
  bool read_path_;
  bool read_tour_;
  int planner_mode_;
  float com_range_;
  std::string path_bag_name_;
  rosbag::Bag path_bag_;
  std::vector<std::string> tour_bag_names_;
  std::vector<rosbag::Bag> tour_bags_;
  // Variables
  std::vector<Vehicle> vehicles_;
  nav_msgs::Path path_;
  std::vector<nav_msgs::Path> tour_;
  float init_octomap_size_;  // [m^3]
  float octomap_size_;  // [m^3]
  int nr_of_recoveries_;
  bool started_;
  bool writing_csv_;
  std::ofstream csv_file_;
  bool first_odom_msg_;
  // Publishers
  ros::Timer pub_timer_;
  ros::Publisher pub_monitor_;
  ros::Publisher pub_path_;
  ros::Publisher pub_mipp_done_;
  ros::Publisher pub_viz_tour_;
  // Subscribers
  ros::Subscriber sub_start_;
  ros::Subscriber sub_octomap_;
  ros::Subscriber sub_path_;
  // Services
  ros::ServiceClient cli_planner_ready_;
  ros::Timer cli_timer_get_octomap_;
  ros::ServiceClient cli_get_octomap_;
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

  np.param<std::string>("world_name", world_name_, "rand");
  np.param<int>("planner_mode", planner_mode_, 0);
  np.param<bool>("auto_start", auto_start_, true);
  np.param<float>("ugv_vel", ugv_vel_, 0.5);
  np.param<int>("nr_of_uavs", nr_of_uavs_, 1);
  np.param<float>("frequency", frequency_, 1.0);
  np.param<float>("octomap_resolution", octomap_resolution_, 0.2);
  np.param<float>("com_range", com_range_, 10.0);
  // Write/read path for UGV for scenario 2
  np.param<bool>("write_path", write_path_, true);
  np.param<bool>("read_path", read_path_, true);
  np.param<std::string>("path_file_name", path_bag_name_, "$(find mipp_launch)/bags/sc2_path_1.bag");
  np.param<bool>("read_tour", read_tour_, false);
  np.param("tour_file_names", tour_bag_names_, std::vector<std::string>());

  path_.poses.clear();
  started_ = false;
  writing_csv_ = false;

  pub_timer_      = n.createTimer(ros::Duration(1.0/frequency_), boost::bind(&MippMonitor::pubMonitor, this));
  pub_monitor_    = n.advertise<mipp_msgs::MippMonitor>("/MippMonitor/monitor", 1);
  pub_path_       = n.advertise<nav_msgs::Path>("/MippMonitor/path", 1);
  pub_mipp_done_  = n.advertise<std_msgs::Bool>("/MippMonitor/mipp_done", 1);
  pub_viz_tour_   = n.advertise<visualization_msgs::Marker>("/MippMonitor/viz_tour", 1);

  sub_start_    = n.subscribe("/MippMonitor/start", 1, &MippMonitor::subStart, this);
  sub_path_     = n.subscribe("/ugv/UGVPlanner/path", 1, &MippMonitor::subPath, this);

  std::string cli_planner_ready_name = "/MippPlanner/planner_ready";
  cli_planner_ready_ = n.serviceClient<std_srvs::SetBool>(cli_planner_ready_name);

  cli_timer_get_octomap_ = n.createTimer(ros::Duration(1.0), boost::bind(&MippMonitor::cliGetOctomap, this));
  cli_get_octomap_ = n.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");

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
      vehicle.com_state.state = mipp_msgs::CommunicationState::STATE_RANGE_LOS_BROKEN;
      vehicle.sub_com = n.subscribe<mipp_msgs::CommunicationState>("/uav"+std::to_string(vehicle.id)+"/ComConstraintVisualizer/constraint_state", 1, boost::bind(&MippMonitor::subCom, this, _1, vehicle.id));
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
  bool planner_ready = false;
  bool communication_ready = false;
  nr_of_recoveries_ = 0;
  while (ros::ok())
  {
    if (auto_start_ and !started_) {
      // Check if planner is ready
      std_srvs::SetBool srv;
      if (cli_planner_ready_.call(srv)) {
        if (srv.response.success) planner_ready = true;
        else ROS_INFO_THROTTLE(5.0, "Mipp planner not ready... yet");
      }
      else {
        ROS_ERROR("Couldn't call planner_ready server.");
      }

      // Check if communication is ready
      communication_ready = true;
      for (auto const& vehicle : vehicles_) {
        if (vehicle.type == UAV) {
          communication_ready = communication_ready and vehicle.com_state.state == mipp_msgs::CommunicationState::STATE_WORKING;
        }
      }
      if (!communication_ready) ROS_INFO_THROTTLE(5.0, "Communication is not running... yet");

      if (planner_ready and communication_ready) {
        startMipp();
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
  if (!started_) return;

  mipp_msgs::MippMonitor monitor_msg;
  monitor_msg.header.frame_id = "world";
  monitor_msg.header.stamp = ros::Time::now();
  monitor_msg.octomap_size = octomap_size_;
  
  for (auto& vehicle_it : vehicles_) {
    // Calculate distance travelled and save current position as "previous" position
    vehicle_it.distance_travelled += getDistanceBetweenPoints(vehicle_it.position, vehicle_it.prev_position);
    vehicle_it.yaw_travelled += getDistanceBetweenAngles(vehicle_it.prev_yaw, vehicle_it.yaw);
    ROS_DEBUG_COND(vehicle_it.type == UAV, "UAV%d: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f) = (%.2f, %.2f)", vehicle_it.id,
                                          vehicle_it.prev_position.x, vehicle_it.prev_position.y, vehicle_it.prev_yaw,
                                          vehicle_it.position.x, vehicle_it.position.y, vehicle_it.yaw,
                                          vehicle_it.distance_travelled, vehicle_it.yaw_travelled);
    vehicle_it.prev_position = vehicle_it.position;
    vehicle_it.prev_yaw = vehicle_it.yaw;


    // Fomrulate message
    mipp_msgs::MippMonitorVehicle monitor_vehicle;
    monitor_vehicle.id                  = vehicle_it.id;
    monitor_vehicle.position            = vehicle_it.position;
    monitor_vehicle.yaw                 = vehicle_it.yaw;
    monitor_vehicle.euc_dist_travelled  = vehicle_it.distance_travelled;
    monitor_vehicle.yaw_dist_travelled  = vehicle_it.yaw_travelled;
    monitor_vehicle.fsm_state           = mipp_msgs::MippMonitorVehicle::FSM_STATE_IDLE;
    monitor_vehicle.com_state           = mipp_msgs::MippMonitorVehicle::COM_STATE_WORKING;

    if (vehicle_it.type == UGV) {
      monitor_msg.ugv = monitor_vehicle;
    }
    else {
      monitor_msg.uavs.push_back(monitor_vehicle);
    }
  }
  
  if (started_ and writing_csv_) {
    ROS_INFO_THROTTLE(1.0, "Writing CSV...");
    csv_file_ << octomap_size_;
    for (auto const& vehicle : vehicles_) {
      csv_file_ << "," << vehicle.distance_travelled;
    }
    csv_file_ << "\n";
  }

  pub_monitor_.publish(monitor_msg);
}

void MippMonitor::subCom(const mipp_msgs::CommunicationStateConstPtr& com_msg, int vehicle_id)
{
  ROS_DEBUG("subCom");
  if ((vehicles_[vehicle_id+1].com_state.state != com_msg->state) and (com_msg->state != mipp_msgs::CommunicationState::STATE_WORKING)) {
    vehicles_[vehicle_id+1].com_state = *com_msg;
    ROS_WARN_COND(started_, "Recording a recovery, currently have recorded %d recoveries", nr_of_recoveries_);
     if (started_) nr_of_recoveries_++;
  }
  vehicles_[vehicle_id+1].com_state = *com_msg;
}

void MippMonitor::cliGetOctomap() {
  ROS_DEBUG("cliGetOctomap");
  octomap_msgs::GetOctomap octomap_srv;
  if (cli_get_octomap_.call(octomap_srv)) {
    auto octomap = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(octomap_srv.response.map));
    //auto octomap_size_unpruned = (int)octomap->calcNumNodes();
    octomap->expand();
    octomap_size_ = octomap->calcNumNodes()*std::pow(octomap_resolution_, 3);
    //ROS_DEBUG("Octomap size %d / %d, binary %d", (int)octomap_size_, (int)octomap_size_unpruned, (int)octomap_srv.response.map.binary);
    delete octomap;
  }
  else {
    ROS_ERROR("Failed to get OctoMap");
  }
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

  // Recording starting things
  ros::Time start_time = ros::Time::now();
  init_octomap_size_ = octomap_size_;
  ROS_DEBUG("Size: %.4f", octomap_size_);

  // Open csv file for writing
  std::string filename;
  int scenario = 0;                                               // Scenario (0:None, 1:Path, 2:Tour)
  if (read_path_) scenario = 1;
  else if (read_tour_) scenario = 2;
  filename += "sce" + std::to_string(scenario) + "/";
  filename += world_name_ + "_";                                  // World name (low, high or rand)
  filename += "pla" + std::to_string(planner_mode_) + "_";        // Planner mode (0:Exp, 1:Form, 2:Hyb, )
  filename += "ugv" + std::to_string((int)(ugv_vel_*10)) + "_";   // UGV velocity * 10 (since 0.5 is normal)
  filename += "uav" + std::to_string(nr_of_uavs_) + "_";          // Nr of UAVs (1, 2 or 3)
  filename += "com" + std::to_string((int)com_range_);            // Com range (10 or 20)
  filename += "_cost_info10dist1";                               // Com range (10 or 20)
  filename += ".csv";
  
  // Check if file exists, raise flag if it doesnt (will need a header)
  std::ifstream csv_file_check;
  csv_file_check.open("/home/kevin/csv/" + filename);
  bool file_exists = (csv_file_check) ? true : false;
  csv_file_check.close();

  ROS_WARN("Writing to CSV filename: %s", filename.c_str());
  csv_file_.open("/home/kevin/csv/" + filename, std::ios_base::app);
  // Add file header if not there (file didn't exist)
  if (!file_exists) {
    ROS_WARN("File didn't exist, adding header");
    csv_file_ << "Date_Time,Time_Used,Info";
    for (auto const& vehicle : vehicles_) {
      if (vehicle.type == UGV) {
        csv_file_ << ",UGV_Euc_Dist,UGV_Yaw_Dist";
      }
      else {
        std::string uav_name = "UAV" + std::to_string(vehicle.id);
        csv_file_ << "," + uav_name + "_Euc_Dist" << "," + uav_name + "_Yaw_Dist";
      }
    }
    // Recoveries (if not standing still scenario)
    if (scenario != 0) csv_file_ << "," << "Recoveries";
    csv_file_ << "\n";
  }
  writing_csv_ = false;

  // Update some message variables
  for (auto& vehicle : vehicles_) {
    vehicle.prev_position = vehicle.position;
    vehicle.prev_yaw = vehicle.yaw;
    vehicle.distance_travelled = 0.0;
    vehicle.yaw_travelled = 0.0;
  }

  // Start Mipp scenario
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
    move_vehicle_goal.goal_reached_max_time = 600.0;
    move_vehicle_client->sendGoal(move_vehicle_goal);

    start_mipp_goal.max_time = 600.0;
    start_mipp_goal.mipp_mode = planner_mode_;
    start_mipp_client->sendGoal(start_mipp_goal);

    started_ = true;

    while (!move_vehicle_client->getState().isDone()) {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
    ROS_WARN("UGV arrived, stopping writing");
    writing_csv_ = false;
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
    start_mipp_goal.max_time = 720.0;
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
    //writing_csv_ = false;
  }
  else {
    float no_mission_time = 30.0;

    ROS_WARN("Didn't read path or tour, doing mipp for %.2f seconds", no_mission_time);

    start_mipp_goal.max_time = no_mission_time;
    start_mipp_goal.mipp_mode = planner_mode_;
    start_mipp_client->sendGoal(start_mipp_goal);
    started_ = true;

    int counter = 0;
    while (counter < (int)no_mission_time) {
      ROS_INFO_THROTTLE(1.0, "Standing still. %d/%d", counter, (int)no_mission_time);
      ros::spinOnce();
      ros::Duration(1.0).sleep();
      counter++;
    }

    ROS_WARN("Finished Standing still");
  }

  ROS_INFO_THROTTLE(1.0, "Writing CSV...");
  // Date/Time
  time_t t = time(0);
  struct tm * now = localtime( & t );
  char buffer[80];
  strftime(buffer,80,"%Y/%m/%d-%H:%M",now);
  std::string date_time = buffer;
  csv_file_ << date_time;
  // Time used
  auto time_used = (ros::Time::now() - start_time).toSec();
  csv_file_ << "," << time_used;
  // Data
  csv_file_ << "," << octomap_size_ - init_octomap_size_;
  for (auto const& vehicle : vehicles_) {
    csv_file_ << "," << vehicle.distance_travelled << "," << vehicle.yaw_travelled;
  }
  // Recoveries (if not standing still scenario)
  if (scenario != 0) csv_file_ << "," << nr_of_recoveries_;

  csv_file_ << "\n";
  csv_file_.close();

  ROS_ERROR("\n*****************\n*\n* Finished mipp \n*\n*****************");
  std_msgs::Bool done_msg;
  done_msg.data = true;
  pub_mipp_done_.publish(done_msg);
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