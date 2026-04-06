#include "lidar_odometry/lidar_odometry_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options),
  is_first_frame_(true),
  cumulative_pose_(Eigen::Matrix4f::Identity()),
  prev_delta_(Eigen::Matrix4f::Identity()),
  prev_stamp_(0, 0, RCL_ROS_TIME),
  last_keyframe_pose_(Eigen::Matrix4f::Identity())
{
  // Declare parameters
  auto input_topic = this->declare_parameter<std::string>("input_topic", "/velodyne_points");
  auto ndt_resolution = this->declare_parameter<double>("ndt_resolution", 1.0);
  auto ndt_step_size = this->declare_parameter<double>("ndt_step_size", 0.1);
  ndt_max_iterations_ = this->declare_parameter<int>("ndt_max_iterations", 30);
  auto ndt_epsilon = this->declare_parameter<double>("ndt_epsilon", 0.01);
  auto ndt_num_threads = this->declare_parameter<int>("ndt_num_threads", 4);
  auto ndt_search_method = this->declare_parameter<std::string>("ndt_search_method", "DIRECT7");
  voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.5);
  min_scan_range_ = this->declare_parameter<double>("min_scan_range", 1.0);
  max_scan_range_ = this->declare_parameter<double>("max_scan_range", 100.0);
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");
  base_translation_variance_ = this->declare_parameter<double>("base_translation_variance", 0.01);
  base_rotation_variance_ = this->declare_parameter<double>("base_rotation_variance", 0.001);
  score_scale_factor_ = this->declare_parameter<double>("score_scale_factor", 1.0);

  // Local map parameters
  local_map_size_ = this->declare_parameter<int>("local_map_size", 15);
  local_map_voxel_size_ = this->declare_parameter<double>("local_map_voxel_size", 0.5);
  local_map_keyframe_distance_ = this->declare_parameter<double>(
    "local_map_keyframe_distance", 0.5);
  local_map_keyframe_angle_ = this->declare_parameter<double>(
    "local_map_keyframe_angle", 0.1);

  // IMU parameters
  auto imu_topic = this->declare_parameter<std::string>("imu_topic", "/imu/data");
  use_imu_prediction_ = this->declare_parameter<bool>("use_imu_prediction", true);

  // LiDAR to vehicle extrinsic calibration
  auto l2v_x = this->declare_parameter<double>("lidar_to_vehicle_x", 0.0);
  auto l2v_y = this->declare_parameter<double>("lidar_to_vehicle_y", 0.0);
  auto l2v_z = this->declare_parameter<double>("lidar_to_vehicle_z", 0.0);
  auto l2v_roll = this->declare_parameter<double>("lidar_to_vehicle_roll", 0.0);
  auto l2v_pitch = this->declare_parameter<double>("lidar_to_vehicle_pitch", 0.0);
  auto l2v_yaw = this->declare_parameter<double>("lidar_to_vehicle_yaw", 0.0);

  // Build LiDAR→vehicle transform matrix
  Eigen::Translation3f translation(
    static_cast<float>(l2v_x), static_cast<float>(l2v_y), static_cast<float>(l2v_z));
  Eigen::AngleAxisf roll_angle(static_cast<float>(l2v_roll), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch_angle(static_cast<float>(l2v_pitch), Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw_angle(static_cast<float>(l2v_yaw), Eigen::Vector3f::UnitZ());
  t_vehicle_lidar_ = (translation * yaw_angle * pitch_angle * roll_angle).matrix();
  t_lidar_vehicle_ = t_vehicle_lidar_.inverse();

  // Configure NDT
  ndt_.setResolution(static_cast<float>(ndt_resolution));
  ndt_.setStepSize(ndt_step_size);
  ndt_.setMaximumIterations(ndt_max_iterations_);
  ndt_.setTransformationEpsilon(ndt_epsilon);
  ndt_.setNumThreads(ndt_num_threads);

  if (ndt_search_method == "KDTREE") {
    ndt_.setNeighborhoodSearchMethod(pclomp::KDTREE);
  } else if (ndt_search_method == "DIRECT26") {
    ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT26);
  } else if (ndt_search_method == "DIRECT1") {
    ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT1);
  } else {
    ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT7);
  }

  // Configure voxel filter
  voxel_filter_.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_));

  // Publishers
  odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/lidar_odometry/odom", 10);
  trans_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/lidar_odometry/trans_pose", 10);

  // Subscribers
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&LidarOdometryNode::pointCloudCallback, this, std::placeholders::_1));

  if (use_imu_prediction_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdometryNode::imuCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(this->get_logger(), "LiDAR odometry node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  NDT resolution: %.2f, threads: %ld, method: %s",
    ndt_resolution, ndt_num_threads, ndt_search_method.c_str());
  RCLCPP_INFO(this->get_logger(), "  Local map: size=%d, voxel=%.2f, kf_dist=%.2f, kf_angle=%.2f",
    local_map_size_, local_map_voxel_size_,
    local_map_keyframe_distance_, local_map_keyframe_angle_);
  RCLCPP_INFO(this->get_logger(), "  IMU prediction: %s, topic: %s",
    use_imu_prediction_ ? "enabled" : "disabled", imu_topic.c_str());
  RCLCPP_INFO(this->get_logger(),
    "  LiDAR to vehicle: xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
    l2v_x, l2v_y, l2v_z, l2v_roll, l2v_pitch, l2v_yaw);
}

void LidarOdometryNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_buffer_.push_back(*msg);
  // Keep buffer bounded to avoid unbounded memory growth
  while (imu_buffer_.size() > 1000) {
    imu_buffer_.pop_front();
  }
}

Eigen::Matrix4f LidarOdometryNode::predictFromImu(
  const rclcpp::Time & prev_time, const rclcpp::Time & curr_time)
{
  std::lock_guard<std::mutex> lock(imu_mutex_);

  // Rotation matrix: LiDAR ← vehicle (for transforming IMU angular velocity to LiDAR frame)
  Eigen::Matrix3d r_lidar_vehicle = t_lidar_vehicle_.block<3, 3>(0, 0).cast<double>();

  Eigen::Vector3d integrated_angular(0.0, 0.0, 0.0);
  rclcpp::Time last_time = prev_time;
  bool has_data = false;

  for (const auto & imu : imu_buffer_) {
    rclcpp::Time imu_time(imu.header.stamp);
    if (imu_time <= prev_time) {
      continue;
    }
    if (imu_time > curr_time) {
      break;
    }

    double dt = (imu_time - last_time).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      last_time = imu_time;
      continue;
    }

    // Transform angular velocity from vehicle/IMU frame to LiDAR frame
    Eigen::Vector3d omega_vehicle(
      imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    Eigen::Vector3d omega_lidar = r_lidar_vehicle * omega_vehicle;

    integrated_angular += omega_lidar * dt;
    last_time = imu_time;
    has_data = true;
  }

  // Remove consumed IMU data (keep data from prev_time onward for potential reuse)
  while (!imu_buffer_.empty()) {
    rclcpp::Time t(imu_buffer_.front().header.stamp);
    if (t >= prev_time) {
      break;
    }
    imu_buffer_.pop_front();
  }

  if (!has_data) {
    // No IMU data available, fall back to constant velocity
    return prev_delta_;
  }

  // Build rotation from integrated angular velocity (small angle: ZYX order)
  Eigen::AngleAxisd roll(integrated_angular.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(integrated_angular.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(integrated_angular.z(), Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();

  // Hybrid prediction: IMU rotation + constant velocity translation
  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  prediction.block<3, 3>(0, 0) = rotation.cast<float>();
  prediction.block<3, 1>(0, 3) = prev_delta_.block<3, 1>(0, 3);

  return prediction;
}

void LidarOdometryNode::updateLocalMap(
  const PointCloud::Ptr & cloud, const Eigen::Matrix4f & pose)
{
  // Check if this frame qualifies as a new keyframe
  Eigen::Matrix4f delta_from_keyframe = last_keyframe_pose_.inverse() * pose;
  double trans_diff = delta_from_keyframe.block<3, 1>(0, 3).norm();
  double angle_diff = Eigen::AngleAxisf(
    Eigen::Matrix3f(delta_from_keyframe.block<3, 3>(0, 0))).angle();

  bool is_first_keyframe = local_map_frames_.empty();
  if (!is_first_keyframe &&
      trans_diff < local_map_keyframe_distance_ &&
      angle_diff < local_map_keyframe_angle_) {
    return;  // Not enough movement for a new keyframe
  }

  // Transform cloud to map frame
  auto transformed = std::make_shared<PointCloud>();
  pcl::transformPointCloud(*cloud, *transformed, pose);

  local_map_frames_.push_back(transformed);
  last_keyframe_pose_ = pose;

  // Remove old frames beyond the window
  while (static_cast<int>(local_map_frames_.size()) > local_map_size_) {
    local_map_frames_.pop_front();
  }

  // Rebuild local map by concatenation + downsampling
  auto combined = std::make_shared<PointCloud>();
  for (const auto & frame : local_map_frames_) {
    *combined += *frame;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(
    static_cast<float>(local_map_voxel_size_),
    static_cast<float>(local_map_voxel_size_),
    static_cast<float>(local_map_voxel_size_));
  voxel.setInputCloud(combined);

  local_map_ = std::make_shared<PointCloud>();
  voxel.filter(*local_map_);

  RCLCPP_DEBUG(this->get_logger(),
    "Local map updated: %d keyframes, %zu points",
    static_cast<int>(local_map_frames_.size()), local_map_->size());
}

void LidarOdometryNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS message to PCL
  auto raw_cloud = std::make_shared<PointCloud>();
  pcl::fromROSMsg(*msg, *raw_cloud);

  if (raw_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
    return;
  }

  // Filter and downsample
  auto filtered_cloud = filterPointCloud(raw_cloud);

  if (filtered_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Point cloud empty after filtering");
    return;
  }

  rclcpp::Time curr_stamp(msg->header.stamp);

  // First frame: initialize local map and publish identity
  if (is_first_frame_) {
    prev_cloud_ = filtered_cloud;
    prev_stamp_ = curr_stamp;
    is_first_frame_ = false;

    // Initialize local map with first frame at identity pose
    updateLocalMap(filtered_cloud, cumulative_pose_);

    std::array<double, 36> zero_cov{};
    Eigen::Matrix4f cumulative_vehicle = t_vehicle_lidar_ * cumulative_pose_ * t_lidar_vehicle_;
    auto odom_msg = matrix4fToPoseMsg(cumulative_vehicle, msg->header, zero_cov);
    auto trans_msg = matrix4fToPoseMsg(Eigen::Matrix4f::Identity(), msg->header, zero_cov);

    odom_pub_->publish(odom_msg);
    trans_pub_->publish(trans_msg);

    RCLCPP_INFO(this->get_logger(), "First frame received (%zu points)", filtered_cloud->size());
    return;
  }

  // Compute initial guess for NDT
  Eigen::Matrix4f predicted_delta;
  if (use_imu_prediction_) {
    predicted_delta = predictFromImu(prev_stamp_, curr_stamp);
  } else {
    predicted_delta = prev_delta_;
  }

  // Initial guess: predicted cumulative pose (scan-to-map)
  Eigen::Matrix4f initial_guess = cumulative_pose_ * predicted_delta;

  // Set NDT target: local map (preferred) or previous cloud (fallback)
  if (local_map_ && local_map_->size() > 100) {
    ndt_.setInputTarget(local_map_);
  } else {
    // Fallback to frame-to-frame for early frames
    ndt_.setInputTarget(prev_cloud_);
    // For frame-to-frame, initial guess is the delta, not cumulative
    initial_guess = predicted_delta;
  }
  ndt_.setInputSource(filtered_cloud);

  auto aligned = std::make_shared<PointCloud>();
  ndt_.align(*aligned, initial_guess);

  if (!ndt_.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "NDT did not converge");
    prev_cloud_ = filtered_cloud;
    prev_stamp_ = curr_stamp;
    return;
  }

  // Extract results
  Eigen::Matrix4f ndt_result = ndt_.getFinalTransformation();
  Eigen::Matrix4f delta;

  if (local_map_ && local_map_->size() > 100) {
    // Scan-to-map: NDT result is the new cumulative pose
    delta = cumulative_pose_.inverse() * ndt_result;
    cumulative_pose_ = ndt_result;
  } else {
    // Frame-to-frame fallback: NDT result is the delta
    delta = ndt_result;
    cumulative_pose_ = cumulative_pose_ * delta;
  }

  prev_delta_ = delta;
  prev_cloud_ = filtered_cloud;
  prev_stamp_ = curr_stamp;

  // Update local map (adds keyframe if sufficient movement)
  updateLocalMap(filtered_cloud, cumulative_pose_);

  // Estimate covariance
  auto covariance = estimateCovariance();

  // Transform to vehicle frame (conjugation: T_v_l * T * T_v_l^-1)
  Eigen::Matrix4f cumulative_vehicle = t_vehicle_lidar_ * cumulative_pose_ * t_lidar_vehicle_;
  Eigen::Matrix4f delta_vehicle = t_vehicle_lidar_ * delta * t_lidar_vehicle_;

  // Publish in vehicle frame
  auto odom_msg = matrix4fToPoseMsg(cumulative_vehicle, msg->header, covariance);
  odom_msg.header.frame_id = odom_frame_id_;
  odom_pub_->publish(odom_msg);

  auto trans_msg = matrix4fToPoseMsg(delta_vehicle, msg->header, covariance);
  trans_pub_->publish(trans_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "NDT converged: iter=%d, prob=%.4f, dist=%.4f, map_pts=%zu",
    ndt_.getFinalNumIteration(),
    ndt_.getTransformationProbability(),
    ndt_.getLastMeanCorrespondenceDistance(),
    local_map_ ? local_map_->size() : 0);
}

LidarOdometryNode::PointCloud::Ptr LidarOdometryNode::filterPointCloud(
  const PointCloud::Ptr & cloud) const
{
  // Range filter
  auto range_filtered = std::make_shared<PointCloud>();
  range_filtered->reserve(cloud->size());

  const double min_sq = min_scan_range_ * min_scan_range_;
  const double max_sq = max_scan_range_ * max_scan_range_;

  for (const auto & p : cloud->points) {
    double dist_sq = static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z);
    if (dist_sq >= min_sq && dist_sq <= max_sq) {
      range_filtered->push_back(p);
    }
  }

  // Voxel downsample
  auto downsampled = std::make_shared<PointCloud>();
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_),
    static_cast<float>(voxel_leaf_size_));
  voxel.setInputCloud(range_filtered);
  voxel.filter(*downsampled);

  return downsampled;
}

std::array<double, 36> LidarOdometryNode::estimateCovariance() const
{
  double probability = ndt_.getTransformationProbability();
  int iterations = ndt_.getFinalNumIteration();
  double mean_dist = ndt_.getLastMeanCorrespondenceDistance();

  // Probability-based scale (higher probability = lower covariance)
  double prob_scale = 1.0;
  if (probability > 1e-6) {
    prob_scale = score_scale_factor_ / probability;
  } else {
    prob_scale = 100.0;
  }

  // Iteration penalty
  double iter_scale = 1.0 +
    (static_cast<double>(iterations) / static_cast<double>(ndt_max_iterations_)) * 2.0;

  // Correspondence distance penalty
  double dist_scale = 1.0 + mean_dist * 2.0;

  double total_scale = std::clamp(prob_scale * iter_scale * dist_scale, 0.1, 100.0);

  // 6x6 covariance in row-major: [x, y, z, roll, pitch, yaw]
  std::array<double, 36> covariance{};
  covariance[0] = base_translation_variance_ * total_scale;   // xx
  covariance[7] = base_translation_variance_ * total_scale;   // yy
  covariance[14] = base_translation_variance_ * total_scale;  // zz
  covariance[21] = base_rotation_variance_ * total_scale;     // roll-roll
  covariance[28] = base_rotation_variance_ * total_scale;     // pitch-pitch
  covariance[35] = base_rotation_variance_ * total_scale;     // yaw-yaw

  return covariance;
}

geometry_msgs::msg::PoseWithCovarianceStamped LidarOdometryNode::matrix4fToPoseMsg(
  const Eigen::Matrix4f & mat,
  const std_msgs::msg::Header & header,
  const std::array<double, 36> & covariance) const
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header = header;

  // Translation
  msg.pose.pose.position.x = static_cast<double>(mat(0, 3));
  msg.pose.pose.position.y = static_cast<double>(mat(1, 3));
  msg.pose.pose.position.z = static_cast<double>(mat(2, 3));

  // Rotation matrix to quaternion
  Eigen::Matrix3f rot = mat.block<3, 3>(0, 0);
  Eigen::Quaternionf quat(rot);
  quat.normalize();
  msg.pose.pose.orientation.x = static_cast<double>(quat.x());
  msg.pose.pose.orientation.y = static_cast<double>(quat.y());
  msg.pose.pose.orientation.z = static_cast<double>(quat.z());
  msg.pose.pose.orientation.w = static_cast<double>(quat.w());

  // Covariance
  for (size_t i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = covariance[i];
  }

  return msg;
}

}  // namespace lidar_odometry
