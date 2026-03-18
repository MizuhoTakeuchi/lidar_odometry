#include "lidar_odometry/lidar_odometry_node.hpp"

// ============================================================
// Constructor
// ============================================================

LidarOdometryNode::LidarOdometryNode(const rclcpp::NodeOptions& options)
    : Node("lidar_odometry_node", options)
{
    declareParameters();

    // --- QoS ---
    rmw_qos_profile_t qos_profile_lidar{
        RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5,
        RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        RMW_QOS_POLICY_DURABILITY_VOLATILE,
        RMW_QOS_DEADLINE_DEFAULT, RMW_QOS_LIFESPAN_DEFAULT,
        RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT, false};
    auto qos_lidar = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile_lidar.history, qos_profile_lidar.depth),
        qos_profile_lidar);

    // --- Subscribers ---
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/cloud", qos_lidar,
        std::bind(&LidarOdometryNode::cloudCallback, this, std::placeholders::_1));

    // --- Publishers ---
    pub_relative_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "~/output/relative_pose", 10);
    pub_odometry_ = create_publisher<nav_msgs::msg::Odometry>(
        "~/output/odometry", 10);
    pub_local_map_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/debug/local_map", 1);
    pub_corners_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/debug/corners", 1);
    pub_surfaces_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/debug/surfaces", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // --- Allocate memory ---
    laser_cloud_in_.reset(new pcl::PointCloud<PointXYZIRT>());
    tmp_ouster_cloud_in_.reset(new pcl::PointCloud<OusterPointXYZIRT>());
    full_cloud_.reset(new pcl::PointCloud<PointType>());
    extracted_cloud_.reset(new pcl::PointCloud<PointType>());
    corner_cloud_.reset(new pcl::PointCloud<PointType>());
    surface_cloud_.reset(new pcl::PointCloud<PointType>());

    laser_cloud_corner_last_ds_.reset(new pcl::PointCloud<PointType>());
    laser_cloud_surf_last_ds_.reset(new pcl::PointCloud<PointType>());
    laser_cloud_corner_from_map_ds_.reset(new pcl::PointCloud<PointType>());
    laser_cloud_surf_from_map_ds_.reset(new pcl::PointCloud<PointType>());

    kdtree_corner_from_map_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_surf_from_map_.reset(new pcl::KdTreeFLANN<PointType>());

    laser_cloud_ori_.reset(new pcl::PointCloud<PointType>());
    coeff_sel_.reset(new pcl::PointCloud<PointType>());

    full_cloud_->points.resize(n_scan_ * horizon_scan_);

    start_ring_index_.assign(n_scan_, 0);
    end_ring_index_.assign(n_scan_, 0);
    point_col_ind_.assign(n_scan_ * horizon_scan_, 0);
    point_range_.assign(n_scan_ * horizon_scan_, 0);

    cloud_smoothness_.resize(n_scan_ * horizon_scan_);
    cloud_curvature_.resize(n_scan_ * horizon_scan_, 0.0f);
    cloud_neighbor_picked_.resize(n_scan_ * horizon_scan_, 0);
    cloud_label_.resize(n_scan_ * horizon_scan_, 0);

    laser_cloud_ori_corner_vec_.resize(n_scan_ * horizon_scan_);
    coeff_sel_corner_vec_.resize(n_scan_ * horizon_scan_);
    laser_cloud_ori_corner_flag_.resize(n_scan_ * horizon_scan_, false);
    laser_cloud_ori_surf_vec_.resize(n_scan_ * horizon_scan_);
    coeff_sel_surf_vec_.resize(n_scan_ * horizon_scan_);
    laser_cloud_ori_surf_flag_.resize(n_scan_ * horizon_scan_, false);

    down_size_filter_surf_.setLeafSize(surf_leaf_size_, surf_leaf_size_, surf_leaf_size_);
    down_size_filter_corner_.setLeafSize(corner_leaf_size_, corner_leaf_size_, corner_leaf_size_);
    down_size_filter_surf_map_.setLeafSize(surf_leaf_size_, surf_leaf_size_, surf_leaf_size_);

    mat_p_.setZero();
    covariance_ros_.fill(0.0);

    column_idn_count_vec_.assign(n_scan_, 0);

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    RCLCPP_INFO(get_logger(), "LiDAR Odometry Node initialized. sensor=%s, n_scan=%d, horizon_scan=%d",
                (sensor_type_ == SensorType::OUSTER ? "ouster" :
                 sensor_type_ == SensorType::VELODYNE ? "velodyne" : "livox"),
                n_scan_, horizon_scan_);

    {
        float ex, ey, ez, eroll, epitch, eyaw;
        pcl::getTranslationAndEulerAngles(T_lidar_to_vehicle_, ex, ey, ez, eroll, epitch, eyaw);
        RCLCPP_INFO(get_logger(),
                    "Extrinsic LiDAR->Vehicle: t=[%.3f, %.3f, %.3f] rpy=[%.3f, %.3f, %.3f] vehicle_frame=%s",
                    ex, ey, ez, eroll, epitch, eyaw, vehicle_frame_.c_str());
    }
}

// ============================================================
// Parameters
// ============================================================

void LidarOdometryNode::declareParameters()
{
    std::string sensor_str;
    declare_parameter("sensor", "ouster");
    get_parameter("sensor", sensor_str);
    if (sensor_str == "velodyne")       sensor_type_ = SensorType::VELODYNE;
    else if (sensor_str == "ouster")    sensor_type_ = SensorType::OUSTER;
    else if (sensor_str == "livox")     sensor_type_ = SensorType::LIVOX;
    else {
        RCLCPP_ERROR(get_logger(), "Invalid sensor type: %s", sensor_str.c_str());
        rclcpp::shutdown();
    }

    declare_parameter("n_scan", 64);           get_parameter("n_scan", n_scan_);
    declare_parameter("horizon_scan", 512);    get_parameter("horizon_scan", horizon_scan_);
    declare_parameter("downsample_rate", 1);   get_parameter("downsample_rate", downsample_rate_);
    declare_parameter("lidar_min_range", 1.0); get_parameter("lidar_min_range", lidar_min_range_);
    declare_parameter("lidar_max_range", 1000.0); get_parameter("lidar_max_range", lidar_max_range_);
    declare_parameter("edge_threshold", 1.0);  get_parameter("edge_threshold", edge_threshold_);
    declare_parameter("surf_threshold", 0.1);  get_parameter("surf_threshold", surf_threshold_);
    declare_parameter("edge_feature_min_valid_num", 10); get_parameter("edge_feature_min_valid_num", edge_feature_min_valid_num_);
    declare_parameter("surf_feature_min_valid_num", 100); get_parameter("surf_feature_min_valid_num", surf_feature_min_valid_num_);
    declare_parameter("corner_leaf_size", 0.2); get_parameter("corner_leaf_size", corner_leaf_size_);
    declare_parameter("surf_leaf_size", 0.4);  get_parameter("surf_leaf_size", surf_leaf_size_);
    declare_parameter("max_iterations", 30);   get_parameter("max_iterations", max_iterations_);
    declare_parameter("degeneracy_threshold", 100.0); get_parameter("degeneracy_threshold", degeneracy_threshold_);
    declare_parameter("degeneracy_covariance", 1e4); get_parameter("degeneracy_covariance", degeneracy_covariance_);
    declare_parameter("keyframe_dist_threshold", 1.0); get_parameter("keyframe_dist_threshold", keyframe_dist_threshold_);
    declare_parameter("keyframe_angle_threshold", 0.2); get_parameter("keyframe_angle_threshold", keyframe_angle_threshold_);
    declare_parameter("max_local_map_keyframes", 50); get_parameter("max_local_map_keyframes", max_local_map_keyframes_);
    declare_parameter("num_threads", 4);       get_parameter("num_threads", num_threads_);
    declare_parameter("lidar_frame", "lidar_link"); get_parameter("lidar_frame", lidar_frame_);
    declare_parameter("odom_frame", "odom");   get_parameter("odom_frame", odom_frame_);
    declare_parameter("publish_tf", true);     get_parameter("publish_tf", publish_tf_);

    // Extrinsic calibration: LiDAR → Vehicle
    declare_parameter("lidar_to_vehicle_translation", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("lidar_to_vehicle_rotation", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("vehicle_frame", "base_link");

    std::vector<double> ext_trans, ext_rot;
    get_parameter("lidar_to_vehicle_translation", ext_trans);
    get_parameter("lidar_to_vehicle_rotation", ext_rot);
    get_parameter("vehicle_frame", vehicle_frame_);

    T_lidar_to_vehicle_ = pcl::getTransformation(
        ext_trans[0], ext_trans[1], ext_trans[2],
        ext_rot[0], ext_rot[1], ext_rot[2]);
}

// ============================================================
// Utility
// ============================================================

float LidarOdometryNode::pointDistance(const PointType& p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float LidarOdometryNode::pointDistance(const PointType& p1, const PointType& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                     (p1.y - p2.y) * (p1.y - p2.y) +
                     (p1.z - p2.z) * (p1.z - p2.z));
}

Eigen::Affine3f LidarOdometryNode::trans2Affine3f(const float transform_in[])
{
    // transform_in: [roll, pitch, yaw, x, y, z]
    return pcl::getTransformation(transform_in[3], transform_in[4], transform_in[5],
                                  transform_in[0], transform_in[1], transform_in[2]);
}

pcl::PointCloud<PointType>::Ptr LidarOdometryNode::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloud_in, const Eigen::Affine3f& trans)
{
    pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>());
    int cloud_size = cloud_in->size();
    cloud_out->resize(cloud_size);

    #pragma omp parallel for num_threads(num_threads_)
    for (int i = 0; i < cloud_size; ++i)
    {
        const auto& pf = cloud_in->points[i];
        cloud_out->points[i].x = trans(0,0)*pf.x + trans(0,1)*pf.y + trans(0,2)*pf.z + trans(0,3);
        cloud_out->points[i].y = trans(1,0)*pf.x + trans(1,1)*pf.y + trans(1,2)*pf.z + trans(1,3);
        cloud_out->points[i].z = trans(2,0)*pf.x + trans(2,1)*pf.y + trans(2,2)*pf.z + trans(2,3);
        cloud_out->points[i].intensity = pf.intensity;
    }
    return cloud_out;
}

void LidarOdometryNode::resetPerFrameData()
{
    laser_cloud_in_->clear();
    extracted_cloud_->clear();
    corner_cloud_->clear();
    surface_cloud_->clear();
    laser_cloud_ori_->clear();
    coeff_sel_->clear();

    range_mat_ = cv::Mat(n_scan_, horizon_scan_, CV_32F, cv::Scalar::all(FLT_MAX));
    column_idn_count_vec_.assign(n_scan_, 0);
}

// ============================================================
// Main Callback
// ============================================================

void LidarOdometryNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    resetPerFrameData();

    // (1) Convert point cloud
    if (!convertPointCloud(msg))
        return;

    // (2) Project to range image
    projectToRangeImage();

    // (3) Extract cloud from range image
    extractCloudFromRangeImage();

    // (4) Extract features
    extractFeatures();

    // (4.5) Transform features from LiDAR frame to vehicle frame
    corner_cloud_ = transformPointCloud(corner_cloud_, T_lidar_to_vehicle_);
    surface_cloud_ = transformPointCloud(surface_cloud_, T_lidar_to_vehicle_);

    // (5) Compute initial guess
    computeInitialGuess();

    // (6) Build local map
    buildLocalMap();

    // (7) Scan-to-map optimization
    scanToMapOptimization();

    // (9) Publish results
    publishResults();

    // (10) Update keyframes
    updateKeyframes();
}

// ============================================================
// (1) Convert Point Cloud
// ============================================================

bool LidarOdometryNode::convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    current_cloud_stamp_ = msg->header.stamp;

    if (sensor_type_ == SensorType::VELODYNE || sensor_type_ == SensorType::LIVOX)
    {
        pcl::fromROSMsg(*msg, *laser_cloud_in_);
    }
    else if (sensor_type_ == SensorType::OUSTER)
    {
        pcl::fromROSMsg(*msg, *tmp_ouster_cloud_in_);
        laser_cloud_in_->points.resize(tmp_ouster_cloud_in_->size());
        laser_cloud_in_->is_dense = tmp_ouster_cloud_in_->is_dense;
        for (size_t i = 0; i < tmp_ouster_cloud_in_->size(); i++)
        {
            auto& src = tmp_ouster_cloud_in_->points[i];
            auto& dst = laser_cloud_in_->points[i];
            dst.x = src.x; dst.y = src.y; dst.z = src.z;
            dst.intensity = src.intensity;
            dst.ring = src.ring;
            dst.time = src.t * 1e-9f;
        }
    }

    // Remove NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laser_cloud_in_, *laser_cloud_in_, indices);

    // Check ring channel (first time only)
    if (ring_flag_ == 0)
    {
        ring_flag_ = -1;
        for (const auto& field : msg->fields)
        {
            if (field.name == "ring") { ring_flag_ = 1; break; }
        }
        if (ring_flag_ == -1)
        {
            if (sensor_type_ == SensorType::VELODYNE) {
                ring_flag_ = 2;  // compute from vertical angle
            } else {
                RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available!");
                return false;
            }
        }
    }

    return true;
}

// ============================================================
// (2) Project to Range Image (from imageProjection.cpp:558-618)
// ============================================================

void LidarOdometryNode::projectToRangeImage()
{
    int cloud_size = laser_cloud_in_->points.size();
    for (int i = 0; i < cloud_size; ++i)
    {
        PointType this_point;
        this_point.x = laser_cloud_in_->points[i].x;
        this_point.y = laser_cloud_in_->points[i].y;
        this_point.z = laser_cloud_in_->points[i].z;
        this_point.intensity = laser_cloud_in_->points[i].intensity;

        float range = pointDistance(this_point);
        if (range < lidar_min_range_ || range > lidar_max_range_)
            continue;

        int row_idn = laser_cloud_in_->points[i].ring;
        // Velodyne without ring field: compute from vertical angle
        if (ring_flag_ == 2) {
            float vertical_angle = std::atan2(this_point.z,
                std::sqrt(this_point.x * this_point.x + this_point.y * this_point.y)) * 180.0f / M_PI;
            row_idn = static_cast<int>((vertical_angle + (n_scan_ - 1)) / 2.0);
        }

        if (row_idn < 0 || row_idn >= n_scan_)
            continue;
        if (row_idn % downsample_rate_ != 0)
            continue;

        int column_idn = -1;
        if (sensor_type_ == SensorType::VELODYNE || sensor_type_ == SensorType::OUSTER)
        {
            float horizon_angle = std::atan2(this_point.x, this_point.y) * 180.0f / M_PI;
            float ang_res_x = 360.0f / float(horizon_scan_);
            column_idn = -std::round((horizon_angle - 90.0f) / ang_res_x) + horizon_scan_ / 2;
            if (column_idn >= horizon_scan_)
                column_idn -= horizon_scan_;
        }
        else if (sensor_type_ == SensorType::LIVOX)
        {
            column_idn = column_idn_count_vec_[row_idn];
            column_idn_count_vec_[row_idn] += 1;
        }

        if (column_idn < 0 || column_idn >= horizon_scan_)
            continue;
        if (range_mat_.at<float>(row_idn, column_idn) != FLT_MAX)
            continue;

        // No deskew: use point as-is
        range_mat_.at<float>(row_idn, column_idn) = range;
        int index = column_idn + row_idn * horizon_scan_;
        full_cloud_->points[index] = this_point;
    }
}

// ============================================================
// (3) Extract Cloud from Range Image (from imageProjection.cpp:621-643)
// ============================================================

void LidarOdometryNode::extractCloudFromRangeImage()
{
    int count = 0;
    for (int i = 0; i < n_scan_; ++i)
    {
        start_ring_index_[i] = count - 1 + 5;
        for (int j = 0; j < horizon_scan_; ++j)
        {
            if (range_mat_.at<float>(i, j) != FLT_MAX)
            {
                point_col_ind_[count] = j;
                point_range_[count] = range_mat_.at<float>(i, j);
                extracted_cloud_->push_back(full_cloud_->points[j + i * horizon_scan_]);
                ++count;
            }
        }
        end_ring_index_[i] = count - 1 - 5;
    }
}

// ============================================================
// (4) Feature Extraction (from featureExtraction.cpp:87-241)
// ============================================================

void LidarOdometryNode::extractFeatures()
{
    calculateSmoothness();
    markOccludedPoints();
    extractFeaturePoints();
}

void LidarOdometryNode::calculateSmoothness()
{
    int cloud_size = extracted_cloud_->points.size();
    for (int i = 5; i < cloud_size - 5; i++)
    {
        float diff_range = point_range_[i-5] + point_range_[i-4]
                         + point_range_[i-3] + point_range_[i-2]
                         + point_range_[i-1] - point_range_[i] * 10
                         + point_range_[i+1] + point_range_[i+2]
                         + point_range_[i+3] + point_range_[i+4]
                         + point_range_[i+5];

        cloud_curvature_[i] = diff_range * diff_range;
        cloud_neighbor_picked_[i] = 0;
        cloud_label_[i] = 0;
        cloud_smoothness_[i].value = cloud_curvature_[i];
        cloud_smoothness_[i].ind = i;
    }
}

void LidarOdometryNode::markOccludedPoints()
{
    int cloud_size = extracted_cloud_->points.size();
    for (int i = 5; i < cloud_size - 6; ++i)
    {
        float depth1 = point_range_[i];
        float depth2 = point_range_[i+1];
        int column_diff = std::abs(int(point_col_ind_[i+1] - point_col_ind_[i]));
        if (column_diff < 10) {
            if (depth1 - depth2 > 0.3) {
                cloud_neighbor_picked_[i - 5] = 1;
                cloud_neighbor_picked_[i - 4] = 1;
                cloud_neighbor_picked_[i - 3] = 1;
                cloud_neighbor_picked_[i - 2] = 1;
                cloud_neighbor_picked_[i - 1] = 1;
                cloud_neighbor_picked_[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                cloud_neighbor_picked_[i + 1] = 1;
                cloud_neighbor_picked_[i + 2] = 1;
                cloud_neighbor_picked_[i + 3] = 1;
                cloud_neighbor_picked_[i + 4] = 1;
                cloud_neighbor_picked_[i + 5] = 1;
                cloud_neighbor_picked_[i + 6] = 1;
            }
        }
        float diff1 = std::abs(point_range_[i-1] - point_range_[i]);
        float diff2 = std::abs(point_range_[i+1] - point_range_[i]);
        if (diff1 > 0.02f * point_range_[i] && diff2 > 0.02f * point_range_[i])
            cloud_neighbor_picked_[i] = 1;
    }
}

void LidarOdometryNode::extractFeaturePoints()
{
    corner_cloud_->clear();
    surface_cloud_->clear();

    pcl::PointCloud<PointType>::Ptr surface_cloud_scan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surface_cloud_scan_ds(new pcl::PointCloud<PointType>());

    for (int i = 0; i < n_scan_; i++)
    {
        surface_cloud_scan->clear();

        for (int j = 0; j < 6; j++)
        {
            int sp = (start_ring_index_[i] * (6 - j) + end_ring_index_[i] * j) / 6;
            int ep = (start_ring_index_[i] * (5 - j) + end_ring_index_[i] * (j + 1)) / 6 - 1;

            if (sp >= ep)
                continue;

            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, by_value());

            int largest_picked_num = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloud_smoothness_[k].ind;
                if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > edge_threshold_)
                {
                    largest_picked_num++;
                    if (largest_picked_num <= 20) {
                        cloud_label_[ind] = 1;
                        corner_cloud_->push_back(extracted_cloud_->points[ind]);
                    } else {
                        break;
                    }

                    cloud_neighbor_picked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int col_diff = std::abs(int(point_col_ind_[ind + l] - point_col_ind_[ind + l - 1]));
                        if (col_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int col_diff = std::abs(int(point_col_ind_[ind + l] - point_col_ind_[ind + l + 1]));
                        if (col_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                int ind = cloud_smoothness_[k].ind;
                if (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < surf_threshold_)
                {
                    cloud_label_[ind] = -1;
                    cloud_neighbor_picked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        int col_diff = std::abs(int(point_col_ind_[ind + l] - point_col_ind_[ind + l - 1]));
                        if (col_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        int col_diff = std::abs(int(point_col_ind_[ind + l] - point_col_ind_[ind + l + 1]));
                        if (col_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloud_label_[k] <= 0)
                    surface_cloud_scan->push_back(extracted_cloud_->points[k]);
            }
        }

        surface_cloud_scan_ds->clear();
        down_size_filter_surf_.setInputCloud(surface_cloud_scan);
        down_size_filter_surf_.filter(*surface_cloud_scan_ds);
        *surface_cloud_ += *surface_cloud_scan_ds;
    }
}

// ============================================================
// (5) Compute Initial Guess
// ============================================================

void LidarOdometryNode::computeInitialGuess()
{
    if (frame_count_ == 0)
    {
        // First frame: identity pose
        for (int i = 0; i < 6; ++i)
            transform_to_be_mapped_[i] = 0;
        return;
    }

    // Constant velocity model: apply previous inter-frame delta
    if (has_prev_prev_pose_)
    {
        Eigen::Affine3f prev_delta = prev_prev_pose_.inverse() * prev_pose_;
        Eigen::Affine3f current_guess = prev_pose_ * prev_delta;
        pcl::getTranslationAndEulerAngles(current_guess,
            transform_to_be_mapped_[3], transform_to_be_mapped_[4], transform_to_be_mapped_[5],
            transform_to_be_mapped_[0], transform_to_be_mapped_[1], transform_to_be_mapped_[2]);
    }
    // else: keep transform_to_be_mapped_ from previous frame (zero-velocity model)
}

// ============================================================
// (6) Build Local Map
// ============================================================

void LidarOdometryNode::buildLocalMap()
{
    if (keyframes_.empty())
        return;

    pcl::PointCloud<PointType>::Ptr corner_from_map(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surf_from_map(new pcl::PointCloud<PointType>());

    for (const auto& kf : keyframes_)
    {
        *corner_from_map += *transformPointCloud(kf.corner_cloud, kf.pose);
        *surf_from_map   += *transformPointCloud(kf.surface_cloud, kf.pose);
    }

    // Downsample
    laser_cloud_corner_from_map_ds_->clear();
    down_size_filter_corner_.setInputCloud(corner_from_map);
    down_size_filter_corner_.filter(*laser_cloud_corner_from_map_ds_);
    laser_cloud_corner_from_map_ds_num_ = laser_cloud_corner_from_map_ds_->size();

    laser_cloud_surf_from_map_ds_->clear();
    down_size_filter_surf_map_.setInputCloud(surf_from_map);
    down_size_filter_surf_map_.filter(*laser_cloud_surf_from_map_ds_);
    laser_cloud_surf_from_map_ds_num_ = laser_cloud_surf_from_map_ds_->size();

    // Publish debug local map
    if (pub_local_map_->get_subscription_count() != 0)
    {
        pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>());
        *local_map = *laser_cloud_corner_from_map_ds_ + *laser_cloud_surf_from_map_ds_;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*local_map, cloud_msg);
        cloud_msg.header.stamp = current_cloud_stamp_;
        cloud_msg.header.frame_id = odom_frame_;
        pub_local_map_->publish(cloud_msg);
    }
}

// ============================================================
// (7) Scan-to-Map Optimization (from mapOptmization.cpp:976-1313)
// ============================================================

void LidarOdometryNode::updatePointAssociateToMap()
{
    trans_point_associate_to_map_ = trans2Affine3f(transform_to_be_mapped_);
}

void LidarOdometryNode::pointAssociateToMap(const PointType* pi, PointType* po)
{
    po->x = trans_point_associate_to_map_(0,0)*pi->x + trans_point_associate_to_map_(0,1)*pi->y + trans_point_associate_to_map_(0,2)*pi->z + trans_point_associate_to_map_(0,3);
    po->y = trans_point_associate_to_map_(1,0)*pi->x + trans_point_associate_to_map_(1,1)*pi->y + trans_point_associate_to_map_(1,2)*pi->z + trans_point_associate_to_map_(1,3);
    po->z = trans_point_associate_to_map_(2,0)*pi->x + trans_point_associate_to_map_(2,1)*pi->y + trans_point_associate_to_map_(2,2)*pi->z + trans_point_associate_to_map_(2,3);
    po->intensity = pi->intensity;
}

void LidarOdometryNode::cornerOptimization()
{
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(num_threads_)
    for (int i = 0; i < laser_cloud_corner_last_ds_num_; i++)
    {
        PointType point_ori, point_sel, coeff;
        std::vector<int> point_search_ind;
        std::vector<float> point_search_sq_dis;

        point_ori = laser_cloud_corner_last_ds_->points[i];
        pointAssociateToMap(&point_ori, &point_sel);
        kdtree_corner_from_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_sq_dis);

        cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

        if (point_search_sq_dis[4] < 1.0) {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++) {
                cx += laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].x;
                cy += laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].y;
                cz += laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].z;
            }
            cx /= 5; cy /= 5; cz /= 5;

            float a11=0, a12=0, a13=0, a22=0, a23=0, a33=0;
            for (int j = 0; j < 5; j++) {
                float ax = laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].x - cx;
                float ay = laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].y - cy;
                float az = laser_cloud_corner_from_map_ds_->points[point_search_ind[j]].z - cz;
                a11 += ax*ax; a12 += ax*ay; a13 += ax*az;
                a22 += ay*ay; a23 += ay*az; a33 += az*az;
            }
            a11/=5; a12/=5; a13/=5; a22/=5; a23/=5; a33/=5;

            matA1.at<float>(0,0)=a11; matA1.at<float>(0,1)=a12; matA1.at<float>(0,2)=a13;
            matA1.at<float>(1,0)=a12; matA1.at<float>(1,1)=a22; matA1.at<float>(1,2)=a23;
            matA1.at<float>(2,0)=a13; matA1.at<float>(2,1)=a23; matA1.at<float>(2,2)=a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0,0) > 3 * matD1.at<float>(0,1)) {
                float x0 = point_sel.x, y0 = point_sel.y, z0 = point_sel.z;
                float x1 = cx + 0.1f*matV1.at<float>(0,0);
                float y1 = cy + 0.1f*matV1.at<float>(0,1);
                float z1 = cz + 0.1f*matV1.at<float>(0,2);
                float x2 = cx - 0.1f*matV1.at<float>(0,0);
                float y2 = cy - 0.1f*matV1.at<float>(0,1);
                float z2 = cz - 0.1f*matV1.at<float>(0,2);

                float a012 = std::sqrt(
                    ((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1)) * ((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1))
                  + ((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1)) * ((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))
                  + ((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1)) * ((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1)));

                float l12 = std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));

                float la = ((y1-y2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1))
                           + (z1-z2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))) / a012 / l12;
                float lb = -((x1-x2)*((x0-x1)*(y0-y2)-(x0-x2)*(y0-y1))
                            - (z1-z2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12;
                float lc = -((x1-x2)*((x0-x1)*(z0-z2)-(x0-x2)*(z0-z1))
                            + (y1-y2)*((y0-y1)*(z0-z2)-(y0-y2)*(z0-z1))) / a012 / l12;

                float ld2 = a012 / l12;
                float s = 1 - 0.9f * std::fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1) {
                    laser_cloud_ori_corner_vec_[i] = point_ori;
                    coeff_sel_corner_vec_[i] = coeff;
                    laser_cloud_ori_corner_flag_[i] = true;
                }
            }
        }
    }
}

void LidarOdometryNode::surfOptimization()
{
    updatePointAssociateToMap();

    #pragma omp parallel for num_threads(num_threads_)
    for (int i = 0; i < laser_cloud_surf_last_ds_num_; i++)
    {
        PointType point_ori, point_sel, coeff;
        std::vector<int> point_search_ind;
        std::vector<float> point_search_sq_dis;

        point_ori = laser_cloud_surf_last_ds_->points[i];
        pointAssociateToMap(&point_ori, &point_sel);
        kdtree_surf_from_map_->nearestKSearch(point_sel, 5, point_search_ind, point_search_sq_dis);

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;
        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        if (point_search_sq_dis[4] < 1.0) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].x;
                matA0(j, 1) = laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].y;
                matA0(j, 2) = laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0,0), pb = matX0(1,0), pc = matX0(2,0), pd = 1;
            float ps = std::sqrt(pa*pa + pb*pb + pc*pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool plane_valid = true;
            for (int j = 0; j < 5; j++) {
                if (std::fabs(pa * laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].x +
                              pb * laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].y +
                              pc * laser_cloud_surf_from_map_ds_->points[point_search_ind[j]].z + pd) > 0.2) {
                    plane_valid = false;
                    break;
                }
            }

            if (plane_valid) {
                float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
                float s = 1 - 0.9f * std::fabs(pd2) / std::sqrt(std::sqrt(
                    point_ori.x*point_ori.x + point_ori.y*point_ori.y + point_ori.z*point_ori.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laser_cloud_ori_surf_vec_[i] = point_ori;
                    coeff_sel_surf_vec_[i] = coeff;
                    laser_cloud_ori_surf_flag_[i] = true;
                }
            }
        }
    }
}

void LidarOdometryNode::combineOptimizationCoeffs()
{
    for (int i = 0; i < laser_cloud_corner_last_ds_num_; ++i) {
        if (laser_cloud_ori_corner_flag_[i]) {
            laser_cloud_ori_->push_back(laser_cloud_ori_corner_vec_[i]);
            coeff_sel_->push_back(coeff_sel_corner_vec_[i]);
        }
    }
    for (int i = 0; i < laser_cloud_surf_last_ds_num_; ++i) {
        if (laser_cloud_ori_surf_flag_[i]) {
            laser_cloud_ori_->push_back(laser_cloud_ori_surf_vec_[i]);
            coeff_sel_->push_back(coeff_sel_surf_vec_[i]);
        }
    }
    std::fill(laser_cloud_ori_corner_flag_.begin(), laser_cloud_ori_corner_flag_.end(), false);
    std::fill(laser_cloud_ori_surf_flag_.begin(), laser_cloud_ori_surf_flag_.end(), false);
}

bool LidarOdometryNode::lmOptimization(int iter_count)
{
    // Coordinate transformation from LIO-SAM (LOAM legacy):
    // lidar -> camera: x=z, y=x, z=y
    // roll=yaw, pitch=roll, yaw=pitch
    float srx = std::sin(transform_to_be_mapped_[1]);
    float crx = std::cos(transform_to_be_mapped_[1]);
    float sry = std::sin(transform_to_be_mapped_[2]);
    float cry = std::cos(transform_to_be_mapped_[2]);
    float srz = std::sin(transform_to_be_mapped_[0]);
    float crz = std::cos(transform_to_be_mapped_[0]);

    int laser_cloud_sel_num = laser_cloud_ori_->size();
    if (laser_cloud_sel_num < 50)
        return false;

    cv::Mat matA(laser_cloud_sel_num, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laser_cloud_sel_num, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laser_cloud_sel_num, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

    PointType point_ori, coeff;

    for (int i = 0; i < laser_cloud_sel_num; i++) {
        // lidar -> camera
        point_ori.x = laser_cloud_ori_->points[i].y;
        point_ori.y = laser_cloud_ori_->points[i].z;
        point_ori.z = laser_cloud_ori_->points[i].x;
        coeff.x = coeff_sel_->points[i].y;
        coeff.y = coeff_sel_->points[i].z;
        coeff.z = coeff_sel_->points[i].x;
        coeff.intensity = coeff_sel_->points[i].intensity;

        float arx = (crx*sry*srz*point_ori.x + crx*crz*sry*point_ori.y - srx*sry*point_ori.z) * coeff.x
                  + (-srx*srz*point_ori.x - crz*srx*point_ori.y - crx*point_ori.z) * coeff.y
                  + (crx*cry*srz*point_ori.x + crx*cry*crz*point_ori.y - cry*srx*point_ori.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*point_ori.x
                  + (sry*srz + cry*crz*srx)*point_ori.y + crx*cry*point_ori.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*point_ori.x
                  + (cry*srz - crz*srx*sry)*point_ori.y - crx*sry*point_ori.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*point_ori.x + (-cry*crz-srx*sry*srz)*point_ori.y) * coeff.x
                  + (crx*crz*point_ori.x - crx*srz*point_ori.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*point_ori.x + (crz*sry-cry*srx*srz)*point_ori.y) * coeff.z;

        // lidar -> camera order in matA columns
        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = arx;
        matA.at<float>(i, 2) = ary;
        matA.at<float>(i, 3) = coeff.z;
        matA.at<float>(i, 4) = coeff.x;
        matA.at<float>(i, 5) = coeff.y;
        matB.at<float>(i, 0) = -coeff.intensity;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iter_count == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        is_degenerate_ = false;
        for (int i = 5; i >= 0; i--) {
            if (matE.at<float>(0, i) < degeneracy_threshold_) {
                for (int j = 0; j < 6; j++)
                    matV2.at<float>(i, j) = 0;
                is_degenerate_ = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (is_degenerate_) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transform_to_be_mapped_[0] += matX.at<float>(0, 0);
    transform_to_be_mapped_[1] += matX.at<float>(1, 0);
    transform_to_be_mapped_[2] += matX.at<float>(2, 0);
    transform_to_be_mapped_[3] += matX.at<float>(3, 0);
    transform_to_be_mapped_[4] += matX.at<float>(4, 0);
    transform_to_be_mapped_[5] += matX.at<float>(5, 0);

    float deltaR = std::sqrt(
        std::pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
        std::pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
        std::pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = std::sqrt(
        std::pow(matX.at<float>(3, 0) * 100, 2) +
        std::pow(matX.at<float>(4, 0) * 100, 2) +
        std::pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05)
        return true;  // converged

    // Store matA and matB for covariance computation on last iteration
    // (done in scanToMapOptimization)
    return false;
}

void LidarOdometryNode::scanToMapOptimization()
{
    // First frame: no local map yet, skip optimization
    if (keyframes_.empty())
    {
        if (frame_count_ == 0)
        {
            // Publish debug feature clouds
            if (pub_corners_->get_subscription_count() != 0) {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*corner_cloud_, cloud_msg);
                cloud_msg.header.stamp = current_cloud_stamp_;
                cloud_msg.header.frame_id = lidar_frame_;
                pub_corners_->publish(cloud_msg);
            }
            if (pub_surfaces_->get_subscription_count() != 0) {
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*surface_cloud_, cloud_msg);
                cloud_msg.header.stamp = current_cloud_stamp_;
                cloud_msg.header.frame_id = lidar_frame_;
                pub_surfaces_->publish(cloud_msg);
            }
        }
        return;
    }

    // Downsample current scan
    laser_cloud_corner_last_ds_->clear();
    down_size_filter_corner_.setInputCloud(corner_cloud_);
    down_size_filter_corner_.filter(*laser_cloud_corner_last_ds_);
    laser_cloud_corner_last_ds_num_ = laser_cloud_corner_last_ds_->size();

    laser_cloud_surf_last_ds_->clear();
    down_size_filter_surf_map_.setInputCloud(surface_cloud_);
    down_size_filter_surf_map_.filter(*laser_cloud_surf_last_ds_);
    laser_cloud_surf_last_ds_num_ = laser_cloud_surf_last_ds_->size();

    if (laser_cloud_corner_last_ds_num_ > edge_feature_min_valid_num_ &&
        laser_cloud_surf_last_ds_num_ > surf_feature_min_valid_num_)
    {
        kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_from_map_ds_);
        kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_from_map_ds_);

        // Store matA/matB from the last iteration for covariance
        cv::Mat last_matA, last_matB;
        int last_n = 0;

        for (int iter = 0; iter < max_iterations_; iter++)
        {
            laser_cloud_ori_->clear();
            coeff_sel_->clear();

            cornerOptimization();
            surfOptimization();
            combineOptimizationCoeffs();

            // Rebuild matA/matB before LM update for covariance
            int sel_num = laser_cloud_ori_->size();
            if (sel_num >= 50)
            {
                // We need to capture the Jacobian and residuals before the LM step
                // for covariance computation. We'll recompute after convergence.
                last_n = sel_num;
            }

            if (lmOptimization(iter))
                break;
        }

        // (8) Compute covariance from last optimization state
        // Recompute Jacobian at converged pose
        {
            laser_cloud_ori_->clear();
            coeff_sel_->clear();
            cornerOptimization();
            surfOptimization();
            combineOptimizationCoeffs();

            int n = laser_cloud_ori_->size();
            if (n >= 50)
            {
                // Build matA (Jacobian) and matB (residuals) at converged pose
                float srx = std::sin(transform_to_be_mapped_[1]);
                float crx = std::cos(transform_to_be_mapped_[1]);
                float sry = std::sin(transform_to_be_mapped_[2]);
                float cry = std::cos(transform_to_be_mapped_[2]);
                float srz = std::sin(transform_to_be_mapped_[0]);
                float crz = std::cos(transform_to_be_mapped_[0]);

                cv::Mat matA_cov(n, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matB_cov(n, 1, CV_32F, cv::Scalar::all(0));

                for (int i = 0; i < n; i++) {
                    PointType po, co;
                    po.x = laser_cloud_ori_->points[i].y;
                    po.y = laser_cloud_ori_->points[i].z;
                    po.z = laser_cloud_ori_->points[i].x;
                    co.x = coeff_sel_->points[i].y;
                    co.y = coeff_sel_->points[i].z;
                    co.z = coeff_sel_->points[i].x;
                    co.intensity = coeff_sel_->points[i].intensity;

                    float arx = (crx*sry*srz*po.x + crx*crz*sry*po.y - srx*sry*po.z) * co.x
                              + (-srx*srz*po.x - crz*srx*po.y - crx*po.z) * co.y
                              + (crx*cry*srz*po.x + crx*cry*crz*po.y - cry*srx*po.z) * co.z;
                    float ary = ((cry*srx*srz - crz*sry)*po.x
                              + (sry*srz + cry*crz*srx)*po.y + crx*cry*po.z) * co.x
                              + ((-cry*crz - srx*sry*srz)*po.x
                              + (cry*srz - crz*srx*sry)*po.y - crx*sry*po.z) * co.z;
                    float arz = ((crz*srx*sry - cry*srz)*po.x + (-cry*crz-srx*sry*srz)*po.y) * co.x
                              + (crx*crz*po.x - crx*srz*po.y) * co.y
                              + ((sry*srz + cry*crz*srx)*po.x + (crz*sry-cry*srx*srz)*po.y) * co.z;

                    matA_cov.at<float>(i, 0) = arz;
                    matA_cov.at<float>(i, 1) = arx;
                    matA_cov.at<float>(i, 2) = ary;
                    matA_cov.at<float>(i, 3) = co.z;
                    matA_cov.at<float>(i, 4) = co.x;
                    matA_cov.at<float>(i, 5) = co.y;
                    matB_cov.at<float>(i, 0) = -co.intensity;
                }

                computeCovariance(matA_cov, matB_cov, n);
            }
        }
    } else {
        RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.",
                    laser_cloud_corner_last_ds_num_, laser_cloud_surf_last_ds_num_);
    }

    // Publish debug feature clouds
    if (pub_corners_->get_subscription_count() != 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*corner_cloud_, cloud_msg);
        cloud_msg.header.stamp = current_cloud_stamp_;
        cloud_msg.header.frame_id = lidar_frame_;
        pub_corners_->publish(cloud_msg);
    }
    if (pub_surfaces_->get_subscription_count() != 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*surface_cloud_, cloud_msg);
        cloud_msg.header.stamp = current_cloud_stamp_;
        cloud_msg.header.frame_id = lidar_frame_;
        pub_surfaces_->publish(cloud_msg);
    }
}

// ============================================================
// (8) Compute Covariance (new implementation)
// ============================================================

void LidarOdometryNode::computeCovariance(const cv::Mat& matA, const cv::Mat& matB, int n)
{
    // matA: N×6 Jacobian, matB: N×1 residuals
    // Column order in matA (LIO-SAM internal): [roll, pitch, yaw, x, y, z]

    cv::Mat JtJ = matA.t() * matA;  // 6×6 Fisher information matrix

    // Residual variance estimate: σ² = Σ(residuals²) / (N - 6)
    float residual_ss = static_cast<float>(cv::norm(matB, cv::NORM_L2SQR));
    float sigma_sq = residual_ss / std::max(n - 6, 1);

    // SVD-based pseudo-inverse for numerical stability
    cv::Mat JtJ_inv;
    cv::invert(JtJ, JtJ_inv, cv::DECOMP_SVD);

    // Covariance = σ² × (JᵀJ)⁻¹ in LIO-SAM order [roll, pitch, yaw, x, y, z]
    cv::Mat cov = sigma_sq * JtJ_inv;

    // Degeneracy handling: inflate covariance for degenerate directions
    if (is_degenerate_)
    {
        cv::Mat eigenvalues, eigenvectors;
        cv::eigen(JtJ, eigenvalues, eigenvectors);

        for (int i = 0; i < 6; i++)
        {
            if (eigenvalues.at<float>(i, 0) < degeneracy_threshold_)
            {
                // Inflate covariance in this eigenvector direction
                // cov += large_var * v * v^T
                cv::Mat v = eigenvectors.row(i).t();  // 6×1
                cov += degeneracy_covariance_ * (v * v.t());
            }
        }
    }

    // Reorder from LIO-SAM [roll, pitch, yaw, x, y, z] → ROS [x, y, z, roll, pitch, yaw]
    // LIO-SAM index: 0=roll, 1=pitch, 2=yaw, 3=x, 4=y, 5=z
    // ROS index:     0=x,    1=y,     2=z,   3=roll, 4=pitch, 5=yaw
    const int lio2ros[6] = {3, 4, 5, 0, 1, 2};

    covariance_ros_.fill(0.0);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            covariance_ros_[lio2ros[i] * 6 + lio2ros[j]] = static_cast<double>(cov.at<float>(i, j));
        }
    }
}

// ============================================================
// (9) Publish Results
// ============================================================

void LidarOdometryNode::publishResults()
{
    Eigen::Affine3f current_pose = trans2Affine3f(transform_to_be_mapped_);

    // --- Publish relative pose (prev_pose⁻¹ × current_pose) ---
    if (has_prev_pose_)
    {
        Eigen::Affine3f relative = prev_pose_.inverse() * current_pose;

        geometry_msgs::msg::PoseWithCovarianceStamped rel_msg;
        rel_msg.header.stamp = current_cloud_stamp_;
        rel_msg.header.frame_id = odom_frame_;

        float rx, ry, rz, rroll, rpitch, ryaw;
        pcl::getTranslationAndEulerAngles(relative, rx, ry, rz, rroll, rpitch, ryaw);

        tf2::Quaternion q;
        q.setRPY(rroll, rpitch, ryaw);

        rel_msg.pose.pose.position.x = rx;
        rel_msg.pose.pose.position.y = ry;
        rel_msg.pose.pose.position.z = rz;
        rel_msg.pose.pose.orientation.x = q.x();
        rel_msg.pose.pose.orientation.y = q.y();
        rel_msg.pose.pose.orientation.z = q.z();
        rel_msg.pose.pose.orientation.w = q.w();

        for (int i = 0; i < 36; i++)
            rel_msg.pose.covariance[i] = covariance_ros_[i];

        pub_relative_pose_->publish(rel_msg);
    }

    // --- Publish cumulative odometry ---
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_cloud_stamp_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = vehicle_frame_;

    odom_msg.pose.pose.position.x = transform_to_be_mapped_[3];
    odom_msg.pose.pose.position.y = transform_to_be_mapped_[4];
    odom_msg.pose.pose.position.z = transform_to_be_mapped_[5];

    tf2::Quaternion q_odom;
    q_odom.setRPY(transform_to_be_mapped_[0], transform_to_be_mapped_[1], transform_to_be_mapped_[2]);
    odom_msg.pose.pose.orientation.x = q_odom.x();
    odom_msg.pose.pose.orientation.y = q_odom.y();
    odom_msg.pose.pose.orientation.z = q_odom.z();
    odom_msg.pose.pose.orientation.w = q_odom.w();

    for (int i = 0; i < 36; i++)
        odom_msg.pose.covariance[i] = covariance_ros_[i];

    pub_odometry_->publish(odom_msg);

    // --- Publish TF ---
    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_cloud_stamp_;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = vehicle_frame_;
        tf_msg.transform.translation.x = transform_to_be_mapped_[3];
        tf_msg.transform.translation.y = transform_to_be_mapped_[4];
        tf_msg.transform.translation.z = transform_to_be_mapped_[5];
        tf_msg.transform.rotation.x = q_odom.x();
        tf_msg.transform.rotation.y = q_odom.y();
        tf_msg.transform.rotation.z = q_odom.z();
        tf_msg.transform.rotation.w = q_odom.w();
        tf_broadcaster_->sendTransform(tf_msg);
    }
}

// ============================================================
// (10) Update Keyframes
// ============================================================

void LidarOdometryNode::updateKeyframes()
{
    Eigen::Affine3f current_pose = trans2Affine3f(transform_to_be_mapped_);

    bool add_keyframe = false;

    if (keyframes_.empty())
    {
        // First frame is always a keyframe
        add_keyframe = true;
    }
    else
    {
        Eigen::Affine3f delta = prev_keyframe_pose_.inverse() * current_pose;
        float dx, dy, dz, droll, dpitch, dyaw;
        pcl::getTranslationAndEulerAngles(delta, dx, dy, dz, droll, dpitch, dyaw);

        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dist >= keyframe_dist_threshold_ ||
            std::fabs(droll) >= keyframe_angle_threshold_ ||
            std::fabs(dpitch) >= keyframe_angle_threshold_ ||
            std::fabs(dyaw) >= keyframe_angle_threshold_)
        {
            add_keyframe = true;
        }
    }

    if (add_keyframe)
    {
        Keyframe kf;
        kf.pose = current_pose;
        kf.corner_cloud.reset(new pcl::PointCloud<PointType>());
        kf.surface_cloud.reset(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*corner_cloud_, *kf.corner_cloud);
        pcl::copyPointCloud(*surface_cloud_, *kf.surface_cloud);
        kf.timestamp = rclcpp::Time(current_cloud_stamp_).seconds();

        keyframes_.push_back(kf);

        // FIFO eviction
        while (static_cast<int>(keyframes_.size()) > max_local_map_keyframes_)
            keyframes_.pop_front();

        prev_keyframe_pose_ = current_pose;
        has_prev_pose_ = true;
    }

    // Update velocity model
    if (frame_count_ > 0)
    {
        has_prev_prev_pose_ = true;
        prev_prev_pose_ = prev_pose_;
    }
    prev_pose_ = current_pose;
    has_prev_pose_ = true;

    frame_count_++;
}

// ============================================================
// Main
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor exec;

    auto node = std::make_shared<LidarOdometryNode>(options);
    exec.add_node(node);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> LiDAR Odometry Node Started.\033[0m");

    exec.spin();
    rclcpp::shutdown();
    return 0;
}
