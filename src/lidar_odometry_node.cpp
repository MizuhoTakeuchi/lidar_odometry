#include "lidar_odometry/lidar_odometry_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>

namespace lidar_odometry
{

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions & options)
: Node("lidar_odometry", options),
  is_first_frame_(true),
  cumulative_pose_(Eigen::Matrix4f::Identity()),
  prev_delta_(Eigen::Matrix4f::Identity())
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

  // Subscriber
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&LidarOdometryNode::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LiDAR odometry node initialized");
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  NDT resolution: %.2f, threads: %ld, method: %s",
    ndt_resolution, ndt_num_threads, ndt_search_method.c_str());
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

  // First frame: store and publish identity
  if (is_first_frame_) {
    prev_cloud_ = filtered_cloud;
    is_first_frame_ = false;

    std::array<double, 36> zero_cov{};
    auto odom_msg = matrix4fToPoseMsg(cumulative_pose_, msg->header, zero_cov);
    auto trans_msg = matrix4fToPoseMsg(Eigen::Matrix4f::Identity(), msg->header, zero_cov);

    odom_pub_->publish(odom_msg);
    trans_pub_->publish(trans_msg);

    RCLCPP_INFO(this->get_logger(), "First frame received (%zu points)", filtered_cloud->size());
    return;
  }

  // NDT alignment
  ndt_.setInputTarget(prev_cloud_);
  ndt_.setInputSource(filtered_cloud);

  auto aligned = std::make_shared<PointCloud>();
  ndt_.align(*aligned, prev_delta_);

  if (!ndt_.hasConverged()) {
    RCLCPP_WARN(this->get_logger(), "NDT did not converge");
    // Still update prev_cloud but keep previous delta
    prev_cloud_ = filtered_cloud;
    return;
  }

  // Get results
  Eigen::Matrix4f delta = ndt_.getFinalTransformation();
  cumulative_pose_ = cumulative_pose_ * delta;
  prev_delta_ = delta;
  prev_cloud_ = filtered_cloud;

  // Estimate covariance
  auto covariance = estimateCovariance();

  // Publish
  auto odom_msg = matrix4fToPoseMsg(cumulative_pose_, msg->header, covariance);
  odom_msg.header.frame_id = odom_frame_id_;
  odom_pub_->publish(odom_msg);

  auto trans_msg = matrix4fToPoseMsg(delta, msg->header, covariance);
  trans_pub_->publish(trans_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "NDT converged: iter=%d, prob=%.4f, dist=%.4f",
    ndt_.getFinalNumIteration(),
    ndt_.getTransformationProbability(),
    ndt_.getLastMeanCorrespondenceDistance());
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
