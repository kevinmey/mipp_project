#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include "mipp_msgs/CommunicationState.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <string>
#include <utils.hpp>

enum VehicleType { UGV, UAV };

struct Vehicle
{
  ros::Subscriber sub_com_constraint;
  ros::Subscriber sub_odom;
  // Vehicle state
  int id; // -1 of UGV, uav_id for UAVs
  VehicleType type;
  geometry_msgs::Point position;
  cv::Point2i pixel_position;
  float yaw;
  mipp_msgs::CommunicationState com_state;
  // Vehicle visualization
  cv::Scalar color;
  std::vector<cv::Point> shape;
  std::vector<geometry_msgs::Point> path;
};

class MippVisualizer
{
public:
  MippVisualizer(ros::NodeHandle n, ros::NodeHandle np, image_transport::ImageTransport it);
  ~MippVisualizer();
private:
  void publishImage();
  void comConstraintCallback(const mipp_msgs::CommunicationStateConstPtr& com_msg, int vehicle_id);
  void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid_msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg, int vehicle_id);
  void buildImage();
  cv::Point getRotatedPoint(float rotation_angle_rad, cv::Point point, cv::Point rotation_axis);
  cv::Point getPixelPosition(const geometry_msgs::Point point, const cv::Point image_origin, const float image_resolution);
  // Params
  int nr_of_uavs_;
  float frequency_;
  // Variables
  std::vector<Vehicle> vehicles_;
  nav_msgs::MapMetaData grid_info_;
  cv::Mat grid_image_;
  cv::Mat image_;
  cv::Point2f image_origin_position_;
  float image_scale_;
  bool dynamic_scaling_;
  // Publishers and transform
  ros::Timer pub_timer_;
  ros::Subscriber sub_occupancy_grid_;
  image_transport::Publisher pubit_visualization_image_;
  // TF
};

MippVisualizer::MippVisualizer(ros::NodeHandle n, ros::NodeHandle np, image_transport::ImageTransport it)
{
  ROS_WARN("MippVisualizer object is being created.");

  np.param<int>("nr_of_uavs", nr_of_uavs_, 1);
  np.param<float>("frequency", frequency_, 10.0);
  np.param<float>("image_scale", image_scale_, 4.0);
  np.param<bool>("dynamic_scaling", dynamic_scaling_, false);

  pub_timer_ = n.createTimer(ros::Duration(1.0/frequency_), boost::bind(&MippVisualizer::publishImage, this));
  sub_occupancy_grid_ = n.subscribe("/projected_map", 1, &MippVisualizer::occupancyGridCallback, this);
  pubit_visualization_image_ = it.advertise("MippVisualizer/image", 1);

  int nr_of_vehicles = 1 + nr_of_uavs_;
  for (int i = 0; i < nr_of_vehicles; i++) {
    Vehicle vehicle;
    vehicle.id = i-1;
    if (vehicle.id != -1) {
      vehicle.type = UAV;
      vehicle.position =  makePoint(0.0, 0.0, 0.0);
      vehicle.yaw = 0.0;
      vehicle.com_state.vehicle_id = -1;
      vehicle.sub_com_constraint = n.subscribe<mipp_msgs::CommunicationState>("/uav"+std::to_string(vehicle.id)+"/ComConstraintVisualizer/constraint_state", 1, boost::bind(&MippVisualizer::comConstraintCallback, this, _1, vehicle.id));
      vehicle.sub_odom = n.subscribe<nav_msgs::Odometry>("/gazebo/ground_truth_uav"+std::to_string(vehicle.id), 1, boost::bind(&MippVisualizer::odometryCallback, this, _1, vehicle.id));
      // UAV Shape is a triangle
      vehicle.shape.push_back(cv::Point2f(10.0*image_scale_, 0.0*image_scale_)); 
      vehicle.shape.push_back(cv::Point2f(-10.0*image_scale_, 5.0*image_scale_)); 
      vehicle.shape.push_back(cv::Point2f(-10.0*image_scale_, -5.0*image_scale_));
      vehicle.shape.push_back(cv::Point2f(10.0*image_scale_, 0.0*image_scale_)); 
    }
    else {
      vehicle.type = UGV;
      vehicle.position =  makePoint(0.0, 0.0, 0.0);
      vehicle.yaw = 0.0;
      vehicle.sub_odom = n.subscribe<nav_msgs::Odometry>("/gazebo/ground_truth_ugv", 1, boost::bind(&MippVisualizer::odometryCallback, this, _1, vehicle.id));
      // UGV shape is a rectangle
      vehicle.shape.push_back(cv::Point2f(10.0*image_scale_, 5.0*image_scale_)); 
      vehicle.shape.push_back(cv::Point2f(-10.0*image_scale_, 5.0*image_scale_)); 
      vehicle.shape.push_back(cv::Point2f(-10.0*image_scale_, -5.0*image_scale_));
      vehicle.shape.push_back(cv::Point2f(10.0*image_scale_, -5.0*image_scale_));
      vehicle.shape.push_back(cv::Point2f(10.0*image_scale_, 5.0*image_scale_)); 
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

MippVisualizer::~MippVisualizer()
{
  ROS_WARN("MippVisualizer object is being deleted.");
}

void MippVisualizer::publishImage()
{
  ROS_DEBUG("publishImage");

  buildImage();

  sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_).toImageMsg();
  pubit_visualization_image_.publish(pub_msg);
}

void MippVisualizer::comConstraintCallback(const mipp_msgs::CommunicationStateConstPtr& com_msg, int vehicle_id) {
  ROS_DEBUG("comConstraintCallback");

  vehicles_[vehicle_id+1].com_state = *com_msg;
}
  
void MippVisualizer::occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid_msg)
{
  ROS_DEBUG("occupancyGridCallback");

  ROS_DEBUG("Got grid with height * width: %d * %d", (int)grid_msg->info.height, (int)grid_msg->info.width);
  grid_info_ = grid_msg->info;
  int image_height = (int)grid_msg->info.height;
  int image_width = (int)grid_msg->info.width;
  grid_image_ = cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));

  int data_idx = 0;
  //for (int y = image_height-1; y >= 0; y--) {  // cv Mat has origin in down left corner
  for (int y = 0; y < image_height; y++) {  // cv Mat has origin in down left corner
    for (int x = 0; x < image_width; x++) {
      if (data_idx < 5) {
        ROS_DEBUG("x: %d / %d \t y: %d / %d", x, image_width, y, image_height);
        ROS_DEBUG("Data idx %d / %d", data_idx, (int)grid_msg->data.size());
      }
      if (data_idx > (int)grid_msg->data.size() - 5) {
        ROS_DEBUG("x: %d / %d \t y: %d / %d", x, image_width, y, image_height);
        ROS_DEBUG("Data idx %d / %d", data_idx, (int)grid_msg->data.size());
      }
      int image_pixel = 0;
      float grid_pixel = grid_msg->data[data_idx];
      if (grid_pixel >= 0) {
        // Grid pixel has value between 0-100, not -1 (which is unmapped)
        image_pixel = (grid_pixel/100.0)*200.0 + 55.0;
      }
      ROS_DEBUG("Grid pixel %d got mapped to image pixel %d", (int)grid_pixel, (int)image_pixel);
      grid_image_.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(image_pixel, image_pixel, image_pixel);
      data_idx++;
    }
  }
}

void MippVisualizer::odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg, int vehicle_id)
{
  ROS_DEBUG("odometryCallback");

  vehicles_[vehicle_id+1].position = odom_msg->pose.pose.position;
  tf2::Quaternion tf_quat;
  tf2::fromMsg(odom_msg->pose.pose.orientation, tf_quat);
  geometry_msgs::Point vehicle_rpy;
  tf2::Matrix3x3(tf_quat).getRPY(vehicle_rpy.x, 
                                  vehicle_rpy.y, 
                                  vehicle_rpy.z);
  vehicles_[vehicle_id+1].yaw = vehicle_rpy.z;
  ROS_DEBUG("Got vehicle %d odom: (%.2f, %.2f, %.2f)", vehicle_id, vehicles_[vehicle_id+1].position.x, vehicles_[vehicle_id+1].position.y, vehicles_[vehicle_id+1].yaw);

  // Path
  const auto vehicle_pos = vehicles_[vehicle_id+1].position;
  if (vehicles_[vehicle_id+1].path.empty()) {
    vehicles_[vehicle_id+1].path.push_back(vehicle_pos);
  } 
  else {
    const auto last_vehicle_pos = *vehicles_[vehicle_id+1].path.rbegin();
    const auto min_distance_to_draw = 0.5;
    const auto distance = getDistanceBetweenPoints(vehicle_pos, last_vehicle_pos);
    ROS_DEBUG("V%d, %d: Distance between path points (%.2f, %.2f) and (%.2f, %.2f) is %.2f", vehicle_id, (int)vehicles_[vehicle_id+1].path.size(), 
                                                                                            last_vehicle_pos.x, last_vehicle_pos.y, 
                                                                                            vehicle_pos.x, vehicle_pos.y, distance);
    if (distance > min_distance_to_draw) {
      ROS_DEBUG("V%d: Pushing in (%.2f, %.2f, %.2f)", vehicle_id, vehicle_pos.x, vehicle_pos.y, vehicle_pos.z);
      vehicles_[vehicle_id+1].path.push_back(vehicle_pos);
    }
  }
}

void MippVisualizer::buildImage()
{
  ROS_DEBUG("buildImage");
 
  // Copy grid image onto image
  image_ = grid_image_;
  image_origin_position_.x = grid_info_.origin.position.x;
  image_origin_position_.y = grid_info_.origin.position.y;
  ROS_DEBUG("Image origin position: (%.2f, %.2f)",image_origin_position_.x, image_origin_position_.y);

  if (image_.size().width == 0) {
    ROS_WARN("Got 0 size image");
    return;
  }
  
  // Zero-pad the image to make it square if required
  cv::Size2i image_size = image_.size();
  if (image_size.width != image_size.height) {
    int image_size_max = (image_size.width > image_size.height) ? image_size.width: image_size.height;
    int image_size_min = (image_size.width < image_size.height) ? image_size.width: image_size.height;
    int extra_padding = ((image_size_max - image_size_min) % 2 == 1 ) ? 1 : 0;  // If max - min difference not even, must add extra padding to one side
    int padding = (image_size_max - image_size_min - extra_padding) / 2;
    if (image_size.width > image_size.height) {
      cv::copyMakeBorder(image_, image_, padding, padding + extra_padding, 0, 0, cv::BORDER_CONSTANT, cv::Vec3b(0,0,0));
      image_origin_position_.y -= (padding + extra_padding)*grid_info_.resolution;
      ROS_DEBUG("New origin position: (%.2f, %.2f)", image_origin_position_.x, image_origin_position_.y);
    }
    else {
      cv::copyMakeBorder(image_, image_, 0, 0, padding + extra_padding, padding, cv::BORDER_CONSTANT, cv::Vec3b(0,0,0));
      image_origin_position_.x -= (padding + extra_padding)*grid_info_.resolution;
      ROS_DEBUG("New origin position: (%.2f, %.2f)", image_origin_position_.x, image_origin_position_.y);
    }
  }
  ROS_DEBUG("Size %d x %d -> %d x %d", image_size.width, image_size.height, image_.size().width, image_.size().height);

  // Scale image
  image_size = image_.size();
  int new_image_size = 400*image_scale_;
  float old_image_resolution = grid_info_.resolution; // [m pr pixel]
  float image_scale_ratio = (float)image_size.width / (float)new_image_size;
  cv::resize(image_, image_, cv::Size(new_image_size, new_image_size));
  float new_image_resolution = image_scale_ratio*old_image_resolution;
  ROS_DEBUG("Size %d x %d -> %d x %d", image_size.width, image_size.height, image_.size().width, image_.size().height);
  ROS_DEBUG("Resolution %.2f -> %.2f", old_image_resolution, new_image_resolution);

  // Start drawing stuff

  // Get vehicle pixel locations
  for (auto &vehicle : vehicles_) {
    vehicle.pixel_position.x = (vehicle.position.x - image_origin_position_.x)/new_image_resolution;
    vehicle.pixel_position.y = (vehicle.position.y - image_origin_position_.y)/new_image_resolution;
    ROS_DEBUG("Vehicle %d position (%.2f, %.2f) -> (%d, %d)", (int)vehicle.id, vehicle.position.x, vehicle.position.y, vehicle.pixel_position.x, vehicle.pixel_position.y);
  }
  
  // Draw com constraints
  for (auto &vehicle : vehicles_) {
    if (vehicle.type == UAV and vehicle.com_state.vehicle_id != -1) {
      cv::Vec3b constraint_color = (vehicle.com_state.state == mipp_msgs::CommunicationState::STATE_WORKING) ? cv::Vec3b(0,255,0) : cv::Vec3b(255,0,0);
      int base_station_id = vehicle.com_state.base_station_id;
      cv::Point2i point_base_station = vehicles_[base_station_id+1].pixel_position;

      
      geometry_msgs::Point com_endpoint_position = vehicle.com_state.los_lost_position;
      cv::Point2i point_com_endpoint = cv::Point2i((com_endpoint_position.x - image_origin_position_.x)/new_image_resolution, 
                                                   (com_endpoint_position.y - image_origin_position_.y)/new_image_resolution);
      cv::line(image_, point_base_station, point_com_endpoint, constraint_color, 1*image_scale_);
    }
  }

  // Draw paths
  for (const auto& vehicle : vehicles_) {
    cv::Vec3b vehicle_color = (vehicle.type == UGV) ? cv::Vec3b(0,255,0) : cv::Vec3b(vehicle.id*80,0,255);
    bool first = true;
    cv::Point prev_pixel_point;
    for (const auto& path_point : vehicle.path) {
      if (first) {
        prev_pixel_point = getPixelPosition(path_point, image_origin_position_, new_image_resolution);
        first = false;
      }
      else {
        auto path_pixel_point = getPixelPosition(path_point, image_origin_position_, new_image_resolution);;
        cv::line(image_, prev_pixel_point, path_pixel_point, vehicle_color, (int)(1.5*image_scale_));
        ROS_DEBUG("Vehicle %d draw (%d, %d) -> (%d, %d)", (int)vehicle.id, prev_pixel_point.x, prev_pixel_point.y, path_pixel_point.x, path_pixel_point.y);
        prev_pixel_point = path_pixel_point;
      }
    }
  }

  // Place vehicles on map
  for (auto &vehicle : vehicles_) {
    cv::Vec3b vehicle_color = (vehicle.type == UGV) ? cv::Vec3b(0,255,0) : cv::Vec3b(vehicle.id*50,0,255);
    image_.at<cv::Vec3b>(vehicle.pixel_position) = vehicle_color;
    for (auto point_it = vehicle.shape.begin(); point_it != vehicle.shape.end() - 1; ++point_it) {
      cv::Point2i point_from = getRotatedPoint(vehicle.yaw, *point_it, vehicle.pixel_position);
      cv::Point2i point_to = getRotatedPoint(vehicle.yaw, *(point_it+1), vehicle.pixel_position);
      cv::line(image_, point_from, point_to, vehicle_color, 3*image_scale_);
      ROS_DEBUG("Vehicle %d draw (%d, %d) -> (%d, %d)", (int)vehicle.id, point_from.x, point_from.y, point_to.x, point_to.y);
    }
  }

  // cv::Mat image has origin in upper left, while grid is lower left. We've ignored this so far, but now have to flip
  cv::flip(image_, image_, 0);
}

cv::Point MippVisualizer::getRotatedPoint(float rotation_angle_rad, cv::Point point, cv::Point rotation_axis)
{
  if (std::abs(rotation_angle_rad) < 0.01){
    // Rotation is so small that it is virtually the same point
    return cv::Point(point.x + rotation_axis.x, point.y + rotation_axis.y);;
  }

  cv::Point rotated_point = cv::Point(point.x*std::cos(rotation_angle_rad) 
                                    - point.y*std::sin(rotation_angle_rad),
                                      point.x*std::sin(rotation_angle_rad) 
                                    + point.y*std::cos(rotation_angle_rad));
  return cv::Point(rotated_point.x + rotation_axis.x, rotated_point.y + rotation_axis.y);
}

cv::Point MippVisualizer::getPixelPosition(const geometry_msgs::Point point, const cv::Point image_origin, const float image_resolution)
{
  ROS_DEBUG("Origin position (%d, %d), resolution (%.2f)", image_origin.x, image_origin.y, image_resolution);
  cv::Point ret_point;
  ret_point.x = (point.x - image_origin.x)/image_resolution;
  ret_point.y = (point.y - image_origin.y)/image_resolution;
  ROS_DEBUG("Position (%.2f, %.2f) -> (%d, %d)", point.x, point.y, (int)ret_point.x, (int)ret_point.y);
  return ret_point;
}


int main(int argc, char** argv){
  ROS_INFO("Starting mipp_visualizer_node.");

  ros::init(argc, argv, "mipp_visualizer_node");
  ros::NodeHandle n;
  ros::NodeHandle np("~");
  image_transport::ImageTransport it(n);

  MippVisualizer MippVisualizer(n, np, it);

  ros::spin();
  return 0;
};