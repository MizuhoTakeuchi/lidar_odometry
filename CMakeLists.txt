cmake_minimum_required(VERSION 3.8)
project(lidar_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "RELEASE")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)

add_executable(lidar_odometry_node
  src/main.cpp
  src/lidar_odometry_node.cpp
  src/ndt_omp_instantiation.cpp
)

target_include_directories(lidar_odometry_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_odometry_node
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  ndt_omp_ros2
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
