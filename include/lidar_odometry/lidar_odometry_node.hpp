#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <deque>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>

// ---------- Point type definitions (from LIO-SAM) ----------

using PointType = pcl::PointXYZI;

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint32_t, t, t)(uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

using PointXYZIRT = VelodynePointXYZIRT;

// ---------- Feature extraction helpers ----------

struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(const smoothness_t& left, const smoothness_t& right) {
        return left.value < right.value;
    }
};

// ---------- Sensor type ----------

enum class SensorType { VELODYNE, OUSTER, LIVOX };

// ---------- Keyframe ----------

struct Keyframe {
    Eigen::Affine3f pose;
    pcl::PointCloud<PointType>::Ptr corner_cloud;
    pcl::PointCloud<PointType>::Ptr surface_cloud;
    double timestamp;
};

// ---------- LidarOdometryNode ----------

class LidarOdometryNode : public rclcpp::Node
{
public:
    explicit LidarOdometryNode(const rclcpp::NodeOptions& options);

private:
    // --- Parameters ---
    SensorType sensor_type_;
    int n_scan_;
    int horizon_scan_;
    int downsample_rate_;
    float lidar_min_range_;
    float lidar_max_range_;
    float edge_threshold_;
    float surf_threshold_;
    int edge_feature_min_valid_num_;
    int surf_feature_min_valid_num_;
    float corner_leaf_size_;
    float surf_leaf_size_;
    int max_iterations_;
    float degeneracy_threshold_;
    float degeneracy_covariance_;
    float keyframe_dist_threshold_;
    float keyframe_angle_threshold_;
    int max_local_map_keyframes_;
    int num_threads_;
    std::string lidar_frame_;
    std::string odom_frame_;
    bool publish_tf_;
    Eigen::Affine3f T_lidar_to_vehicle_;
    std::string vehicle_frame_;

    // --- ROS interfaces ---
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_relative_pose_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_corners_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_surfaces_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Point cloud data ---
    pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud_in_;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmp_ouster_cloud_in_;
    pcl::PointCloud<PointType>::Ptr full_cloud_;
    pcl::PointCloud<PointType>::Ptr extracted_cloud_;
    cv::Mat range_mat_;

    // CloudInfo equivalent (internal arrays)
    std::vector<int> start_ring_index_;
    std::vector<int> end_ring_index_;
    std::vector<int> point_col_ind_;
    std::vector<float> point_range_;

    // --- Feature extraction ---
    pcl::PointCloud<PointType>::Ptr corner_cloud_;
    pcl::PointCloud<PointType>::Ptr surface_cloud_;
    std::vector<smoothness_t> cloud_smoothness_;
    std::vector<float> cloud_curvature_;
    std::vector<int> cloud_neighbor_picked_;
    std::vector<int> cloud_label_;
    pcl::VoxelGrid<PointType> down_size_filter_surf_;

    // --- Scan-to-map optimization ---
    pcl::PointCloud<PointType>::Ptr laser_cloud_corner_last_ds_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_last_ds_;
    int laser_cloud_corner_last_ds_num_ = 0;
    int laser_cloud_surf_last_ds_num_ = 0;

    pcl::PointCloud<PointType>::Ptr laser_cloud_corner_from_map_ds_;
    pcl::PointCloud<PointType>::Ptr laser_cloud_surf_from_map_ds_;
    int laser_cloud_corner_from_map_ds_num_ = 0;
    int laser_cloud_surf_from_map_ds_num_ = 0;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_from_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_from_map_;

    pcl::PointCloud<PointType>::Ptr laser_cloud_ori_;
    pcl::PointCloud<PointType>::Ptr coeff_sel_;
    std::vector<PointType> laser_cloud_ori_corner_vec_;
    std::vector<PointType> coeff_sel_corner_vec_;
    std::vector<bool> laser_cloud_ori_corner_flag_;
    std::vector<PointType> laser_cloud_ori_surf_vec_;
    std::vector<PointType> coeff_sel_surf_vec_;
    std::vector<bool> laser_cloud_ori_surf_flag_;

    pcl::VoxelGrid<PointType> down_size_filter_corner_;
    pcl::VoxelGrid<PointType> down_size_filter_surf_map_;

    Eigen::Affine3f trans_point_associate_to_map_;

    // transformTobeMapped[6]: [roll, pitch, yaw, x, y, z]  (LIO-SAM convention)
    float transform_to_be_mapped_[6] = {};

    bool is_degenerate_ = false;
    Eigen::Matrix<float, 6, 6> mat_p_;

    // --- Keyframe / local map management ---
    std::deque<Keyframe> keyframes_;
    bool has_prev_pose_ = false;
    Eigen::Affine3f prev_keyframe_pose_;

    // --- Velocity model ---
    bool has_prev_prev_pose_ = false;
    Eigen::Affine3f prev_pose_;
    Eigen::Affine3f prev_prev_pose_;

    // --- Covariance output ---
    std::array<double, 36> covariance_ros_;

    // --- Ring flag for first-time check ---
    int ring_flag_ = 0;

    // --- Livox column count ---
    std::vector<int> column_idn_count_vec_;

    // --- Frame counter ---
    int frame_count_ = 0;

    // --- Timestamp ---
    rclcpp::Time current_cloud_stamp_;

    // ========== Methods ==========

    void declareParameters();

    // Main callback
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // Pipeline steps
    bool convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void projectToRangeImage();
    void extractCloudFromRangeImage();
    void extractFeatures();
    void computeInitialGuess();
    void buildLocalMap();
    void scanToMapOptimization();
    void computeCovariance(const cv::Mat& matA, const cv::Mat& matB, int n);
    void publishResults();
    void updateKeyframes();

    // Feature extraction sub-steps
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeaturePoints();

    // Optimization sub-steps
    void updatePointAssociateToMap();
    void pointAssociateToMap(const PointType* pi, PointType* po);
    void cornerOptimization();
    void surfOptimization();
    void combineOptimizationCoeffs();
    bool lmOptimization(int iter_count);

    // Utility
    Eigen::Affine3f trans2Affine3f(const float transform_in[]);
    pcl::PointCloud<PointType>::Ptr transformPointCloud(
        pcl::PointCloud<PointType>::Ptr cloud_in, const Eigen::Affine3f& trans);
    void resetPerFrameData();

    static float pointDistance(const PointType& p);
    static float pointDistance(const PointType& p1, const PointType& p2);
};
