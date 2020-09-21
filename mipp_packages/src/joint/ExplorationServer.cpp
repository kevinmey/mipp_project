#include <ExplorationServer.hpp>

// Constructor
  
ExplorationServer::ExplorationServer(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_INFO("ExplorationServer object is being created.");

  // Initialize values
  getParams(np);

  sub_clicked_point_ = n.subscribe("/exploration/start_collaborative", 1, &ExplorationServer::subClickedPoint, this);
  act_ugv_exploration_client_ = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>("/ugv/exploration_action", true);
  act_ugv_exploration_client_->waitForServer(ros::Duration(10.0));
  /*for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    std::string uav_exploration_client_name = "/uav"+std::to_string(uav_id)+"/exploration_action";
    actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>* act_uav_exploration_client = new actionlib::SimpleActionClient<mipp_msgs::StartExplorationAction>(uav_exploration_client_name, true);
    act_uav_exploration_clients_.push_back(act_uav_exploration_client);
  }*/

  // Set variables
  running_exploration_ = false;

  ROS_WARN("Done.");
}

// Destructor
  
ExplorationServer::~ExplorationServer() {
  ROS_INFO("ExplorationServer object is being deleted.");
}

/* 
*  Callback functions for subscriptions
*/

void ExplorationServer::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg) {
  ROS_WARN("YEET");

  mipp_msgs::StartExplorationGoal exploration_goal;
  exploration_goal.max_time = 2.0;
  act_ugv_exploration_client_->sendGoal(exploration_goal);

  bool finished_before_timeout = act_ugv_exploration_client_->waitForResult(ros::Duration(4.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = act_ugv_exploration_client_->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
}

// Utility functions

void ExplorationServer::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  // General
  np.param<int>("nr_of_uavs", nr_of_uavs_, 0);
  np.param<std::string>("ugv_ns", ugv_ns_, "/ugv/");
}