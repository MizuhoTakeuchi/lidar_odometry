#include <rclcpp/rclcpp.hpp>
#include "lidar_odometry/lidar_odometry_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_odometry::LidarOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
