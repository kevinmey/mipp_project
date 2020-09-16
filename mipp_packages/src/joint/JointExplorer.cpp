#include <JointExplorer.hpp>

// Constructor
  
JointExplorer::JointExplorer(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_INFO("JointExplorer object is being created.");

  sub_clicked_point_ = n.subscribe("/clicked_point", 1, &JointExplorer::subClickedPoint, this);

  // Initialize values
  getParams(np);

  ugv_explorer_ = new UGVFrontierExplorer();
  initUGVExplorer(n, np);
  for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
    UAVInformativeExplorer* uav_explorer = new UAVInformativeExplorer();
    uav_explorers_.push_back(uav_explorer);
    initUAVExplorer(n, np, uav_id);
  }
  ROS_WARN("Done.");

  // Set variables
  running_joint_exploration_ = false;
}

// Destructor
  
JointExplorer::~JointExplorer() {
  ROS_INFO("JointExplorer object is being deleted.");
}

/* 
*  Callback functions for subscriptions
*/

void JointExplorer::subClickedPoint(const geometry_msgs::PointStampedConstPtr& clicked_point_msg)
{
  if (ugv_explorer_->running_frontier_exploration_) { 
    ROS_INFO("Stopping joint exploration.");
    running_joint_exploration_ = false;
    ugv_explorer_->running_frontier_exploration_ = false;
    for (int uav_id = 0; uav_id < nr_of_uavs_; uav_id++) {
      uav_explorers_[uav_id]->uav_running_exploration_ = false;
    }
  }
  else { 
    ROS_INFO("Starting joint exploration.");
    ugv_explorer_->running_frontier_exploration_ = true;
      ugv_explorer_->runFrontierExploration();
      if (ugv_explorer_->frontier_nodes_.size() > 0) {
        ugv_explorer_->current_frontier_goal_ = ugv_explorer_->makePoseStampedFromNode(*ugv_explorer_->frontier_nodes_.begin()->second.getParent());

        nav_msgs::Path goal_path;
        goal_path.header = ugv_explorer_->current_frontier_goal_.header;
        Node node_on_path = *ugv_explorer_->frontier_nodes_.begin()->second.getParent();
        while (node_on_path.getParent() != nullptr) {
          goal_path.poses.insert(goal_path.poses.begin(), ugv_explorer_->makePoseStampedFromNode(node_on_path));
          ROS_DEBUG("Node: %d", (int)node_on_path.id_);

          if (node_on_path.getParent()->id_ == 0) {
            break;
          }
          node_on_path = *(node_on_path.getParent());
        }

        ugv_explorer_->pub_goal_.publish(ugv_explorer_->current_frontier_goal_);
        ugv_explorer_->pub_goal_path_.publish(goal_path);
      }
      else {
        ROS_WARN("No Frontier nodes found.");
      }
  }

  ros::Rate wait_rate(20.0);
  while (ugv_explorer_->running_frontier_exploration_) {
    if (getDistanceBetweenPoints(ugv_explorer_->ugv_odometry_.pose.pose.position, ugv_explorer_->current_frontier_goal_.pose.position) < 0.5) {
      ugv_explorer_->runFrontierExploration();
      if (ugv_explorer_->frontier_nodes_.size() > 0) {
        ugv_explorer_->current_frontier_goal_ = ugv_explorer_->makePoseStampedFromNode(*ugv_explorer_->frontier_nodes_.begin()->second.getParent());

        nav_msgs::Path goal_path;
        goal_path.header = ugv_explorer_->current_frontier_goal_.header;
        Node node_on_path = *ugv_explorer_->frontier_nodes_.begin()->second.getParent();
        while (node_on_path.getParent() != nullptr) {
          goal_path.poses.insert(goal_path.poses.begin(), ugv_explorer_->makePoseStampedFromNode(node_on_path));
          ROS_DEBUG("Node: %d", (int)node_on_path.id_);

          if (node_on_path.getParent()->id_ == 0) {
            break;
          }
          node_on_path = *(node_on_path.getParent());
        }

        ugv_explorer_->pub_goal_.publish(ugv_explorer_->current_frontier_goal_);
        ugv_explorer_->pub_goal_path_.publish(goal_path);
      }
    }

    ros::spinOnce();
    wait_rate.sleep();
  }

}

// Utility functions

void JointExplorer::getParams(ros::NodeHandle np) {
  ROS_DEBUG("getParams");
  // General
  np.param<int>("nr_of_uavs", nr_of_uavs_, 0);
  np.param<double>("planner_rate", planner_rate_, 100.0);
  np.param<double>("planner_max_time", planner_max_time_, 3.0);
  np.param<double>("planner_max_ray_distance", planner_max_ray_distance_, 2.0);
  np.param<bool>("planner_unmapped_is_collision", planner_unmapped_is_collision_, true);
  // UGV
  np.param<bool>("use_dynamic_range", use_dynamic_range_, true);
  np.param<double>("dynamic_range_padding", dynamic_range_padding_, 2.0);
  np.param<double>("x_range_min", x_range_min_, -10.0);
  np.param<double>("x_range_max", x_range_max_,  10.0);
  np.param<double>("y_range_min", y_range_min_, -10.0);
  np.param<double>("y_range_max", y_range_max_,  10.0);
  np.param<std::string>("planner_world_frame", planner_world_frame_, "map");
  np.param<double>("planner_min_distance_to_frontier", planner_min_distance_to_frontier_, 4.0);
  np.param<double>("planner_max_distance_to_frontier", planner_max_distance_to_frontier_, 100.0);
  np.param<int>("collision_threshold", collision_threshold_, 0);
  // UAV
  np.param<std::string>("uav_world_frame", uav_world_frame_, "world");
  np.param<std::string>("uav_local_frame", uav_local_frame_, "odom_uav");
  np.param<std::string>("uav_body_frame", uav_body_frame_, "base_link_uav");
  np.param<double>("uav_takeoff_z", uav_takeoff_z_, 2.0);
  np.param<double>("uav_camera_width", uav_camera_width_, 6.0);
  np.param<double>("uav_camera_height", uav_camera_height_, 3.0);
  np.param<double>("uav_camera_hfov", uav_camera_hfov_, 1.02974);
  np.param<double>("uav_camera_ray_resolution", uav_camera_ray_resolution_, 1.0);
  np.param<double>("uav_camera_range", uav_camera_range_, 7.5);
  np.param<double>("planner_sample_radius", planner_sample_radius_, 10.0);
  np.param<double>("planner_max_neighbor_distance", planner_max_neighbor_distance_, planner_max_ray_distance_);
  np.param<double>("planner_max_neighbor_yaw", planner_max_neighbor_yaw_, angles::from_degrees(90.0));
}

void JointExplorer::initUGVExplorer(ros::NodeHandle n, ros::NodeHandle np) {
  ROS_DEBUG("initUGVExplorer");

  // Set parameters
  ugv_explorer_->use_dynamic_range_ = use_dynamic_range_;
  ugv_explorer_->dynamic_range_padding_ = dynamic_range_padding_;
  ugv_explorer_->x_range_min_ = x_range_min_;
  ugv_explorer_->x_range_max_ = x_range_max_;
  ugv_explorer_->y_range_min_ = y_range_min_;
  ugv_explorer_->y_range_max_ = y_range_max_;
  ugv_explorer_->planner_world_frame_ = planner_world_frame_;
  ugv_explorer_->planner_rate_ = planner_rate_;
  ugv_explorer_->planner_max_time_ = planner_max_time_;
  ugv_explorer_->planner_max_ray_distance_ = planner_max_ray_distance_;
  ugv_explorer_->planner_min_distance_to_frontier_ = planner_min_distance_to_frontier_;
  ugv_explorer_->planner_max_distance_to_frontier_ = planner_max_distance_to_frontier_;
  ugv_explorer_->collision_threshold_ = collision_threshold_;
  ugv_explorer_->unmapped_is_collision_ = false; //planner_unmapped_is_collision_;
  ugv_explorer_->running_frontier_exploration_ = false;

  std::string ugv_ns = "/ugv/";

  // Set variables
  //   Publishers
  ugv_explorer_->pub_goal_                = n.advertise<geometry_msgs::PoseStamped>(ugv_ns+"move_base_simple/goal", 1);
  ugv_explorer_->pub_goal_path_           = n.advertise<nav_msgs::Path>("ugv_explorer/goal_path", 1);
  ugv_explorer_->pub_viz_tree_            = n.advertise<visualization_msgs::Marker>("ugv_explorer/viz_tree", 1);
  ugv_explorer_->pub_viz_root_node_       = n.advertise<visualization_msgs::MarkerArray>("ugv_explorer/viz_root_node", 1);
  ugv_explorer_->pub_viz_frontier_nodes_  = n.advertise<visualization_msgs::Marker>("ugv_explorer/viz_frontier_nodes", 1);
  //   Subscribers
  ugv_explorer_->map_initialized_ = false;
  ugv_explorer_->sub_map_         = n.subscribe(ugv_ns+"move_base/global_costmap/costmap", 1, &UGVFrontierExplorer::subMap, ugv_explorer_);
  ugv_explorer_->sub_map_update_  = n.subscribe(ugv_ns+"move_base/global_costmap/costmap_updates", 1, &UGVFrontierExplorer::subMapUpdate, ugv_explorer_);
  ugv_explorer_->sub_odometry_    = n.subscribe(ugv_ns+"odometry/filtered", 1, &UGVFrontierExplorer::subOdometry, ugv_explorer_);
  //   TF
  ugv_explorer_->tf_listener_ = new tf2_ros::TransformListener(ugv_explorer_->tf_buffer_);
  //   RNG
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  ugv_explorer_->generator_ = std::default_random_engine(rd());
  ugv_explorer_->x_distribution_ = std::uniform_real_distribution<double>(x_range_min_, x_range_max_);
  ugv_explorer_->y_distribution_ = std::uniform_real_distribution<double>(y_range_min_, y_range_max_);
  ugv_explorer_->unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);
  // Wait for map before proceeding
  ros::Rate wait_rate(1.0);
  while (!ugv_explorer_->map_initialized_) {
    ROS_WARN("UGV waiting for map...");
    ros::spinOnce();
    wait_rate.sleep();
  }

  ROS_INFO("UGVFrontierExplorer: Initialization done.");
}

void JointExplorer::initUAVExplorer(ros::NodeHandle n, ros::NodeHandle np, int uav_id) {
  ROS_DEBUG("initUAVExplorer(%d)", uav_id);

  // Set parameters
  uav_explorers_[uav_id]->uav_id_ = uav_id;
  uav_explorers_[uav_id]->uav_world_frame_ = uav_world_frame_;
  uav_explorers_[uav_id]->uav_local_frame_ = uav_local_frame_+std::to_string(uav_id);
  uav_explorers_[uav_id]->uav_body_frame_ = uav_body_frame_+std::to_string(uav_id);
  uav_explorers_[uav_id]->uav_start_x_ = uav_start_x_;
  uav_explorers_[uav_id]->uav_start_y_ = uav_start_y_;
  uav_explorers_[uav_id]->uav_takeoff_z_ = uav_takeoff_z_;
  uav_explorers_[uav_id]->uav_camera_width_ = uav_camera_width_;
  uav_explorers_[uav_id]->uav_camera_height_ = uav_camera_height_;
  uav_explorers_[uav_id]->uav_camera_hfov_ = uav_camera_hfov_;
  uav_explorers_[uav_id]->uav_camera_ray_resolution_ = uav_camera_ray_resolution_;
  uav_explorers_[uav_id]->uav_camera_range_ = uav_camera_range_;
  uav_explorers_[uav_id]->planner_rate_ = planner_rate_;
  uav_explorers_[uav_id]->planner_sample_radius_ = planner_sample_radius_;
  uav_explorers_[uav_id]->planner_max_time_ = planner_max_time_;
  uav_explorers_[uav_id]->planner_max_ray_distance_ = planner_max_ray_distance_;
  uav_explorers_[uav_id]->planner_max_neighbor_distance_ = planner_max_neighbor_distance_;
  uav_explorers_[uav_id]->planner_max_neighbor_yaw_ = planner_max_neighbor_yaw_;
  uav_explorers_[uav_id]->planner_unmapped_is_collision_ = planner_unmapped_is_collision_;

  // Set variables
  uav_explorers_[uav_id]->uav_running_exploration_ = false;
  
  std::string uav_ns = "/uav"+std::to_string(uav_id)+"/";

  // Set variables
  //   Publishers
  uav_explorers_[uav_id]->pub_position_goal_          = n.advertise<geometry_msgs::PoseStamped>(uav_ns+"position_goal", 10);
  uav_explorers_[uav_id]->pub_viz_uav_fov_            = n.advertise<visualization_msgs::Marker>("uav"+std::to_string(uav_id)+"_explorer/viz_uav_fov", 1);
  uav_explorers_[uav_id]->pub_viz_fov_                = n.advertise<visualization_msgs::Marker>("uav"+std::to_string(uav_id)+"_explorer/viz_fov", 1);
  uav_explorers_[uav_id]->pub_viz_information_points_ = n.advertise<visualization_msgs::Marker>("uav"+std::to_string(uav_id)+"_explorer/viz_information_points", 1);
  uav_explorers_[uav_id]->pub_viz_tree_               = n.advertise<visualization_msgs::Marker>("uav"+std::to_string(uav_id)+"_explorer/viz_tree", 1);
  uav_explorers_[uav_id]->pub_viz_path_               = n.advertise<visualization_msgs::Marker>("uav"+std::to_string(uav_id)+"_explorer/viz_path", 1);
  //   Subscribers
  uav_explorers_[uav_id]->sub_odometry_ = n.subscribe("/gazebo/ground_truth_uav"+std::to_string(uav_id), 1, &UAVInformativeExplorer::subOdometry, uav_explorers_[uav_id]);
  uav_explorers_[uav_id]->sub_octomap_  = n.subscribe("/octomap_binary", 1, &UAVInformativeExplorer::subOctomap, uav_explorers_[uav_id]);
  //   TF
  uav_explorers_[uav_id]->tf_listener_ = new tf2_ros::TransformListener(uav_explorers_[uav_id]->tf_buffer_);
  //   RNG
  std::random_device rd;  // Non-deterministic random nr. to seed generator
  uav_explorers_[uav_id]->generator_ = std::default_random_engine(rd());
  uav_explorers_[uav_id]->unit_distribution_ = std::uniform_real_distribution<double>(0.0, 1.0);
  //   Other (has its own function)
  uav_explorers_[uav_id]->initVariables();
}