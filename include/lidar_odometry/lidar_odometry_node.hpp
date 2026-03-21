#ifndef LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
#define LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace lidar_odometry
{

class LidarOdometryNode : public rclcpp::Node
{
public:
  explicit LidarOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<PointT>;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  PointCloud::Ptr filterPointCloud(const PointCloud::Ptr & cloud) const;

  std::array<double, 36> estimateCovariance() const;

  geometry_msgs::msg::PoseWithCovarianceStamped matrix4fToPoseMsg(
    const Eigen::Matrix4f & mat,
    const std_msgs::msg::Header & header,
    const std::array<double, 36> & covariance) const;

  // NDT
  pclomp::NormalDistributionsTransform<PointT, PointT> ndt_;

  // Voxel filter
  pcl::VoxelGrid<PointT> voxel_filter_;

  // State
  bool is_first_frame_;
  PointCloud::Ptr prev_cloud_;
  Eigen::Matrix4f cumulative_pose_;
  Eigen::Matrix4f prev_delta_;

  // Publishers / Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr trans_pub_;

  // Parameters
  double min_scan_range_;
  double max_scan_range_;
  double voxel_leaf_size_;
  int ndt_max_iterations_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double base_translation_variance_;
  double base_rotation_variance_;
  double score_scale_factor_;
};

}  // namespace lidar_odometry

#endif  // LIDAR_ODOMETRY__LIDAR_ODOMETRY_NODE_HPP_
