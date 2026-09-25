#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include "dog_prior_map_localization/external_ndt_transaction_server.hpp"

namespace dog_prior_map_localization
{
namespace
{
// 更新点云尺寸和稠密属性，确保后续 PCL 算法获得合法的组织信息。
void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

// 将位置和旋转矩阵组合为 PCL/NDT 使用的四阶齐次变换矩阵。
Eigen::Matrix4d poseToMatrix(const Eigen::Vector3d &p, const Eigen::Matrix3d &R)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

// 提取旋转矩阵对应的最小旋转角，并转换为角度制。
double rotationAngleDeg(const Eigen::Matrix3d &R)
{
  Eigen::AngleAxisd angle_axis(R);
  return std::abs(angle_axis.angle()) * 180.0 / M_PI;
}

}  // namespace

class DogPriorMapNdtNode
{
public:
  // 初始化独立 NDT 定位节点：读取参数、加载地图并建立 ROS 接口。
  DogPriorMapNdtNode() : nh_(), pnh_("~")
  {
    map_frame_ = getParam<std::string>("frames/map_frame", "camera_init");
    base_frame_ = getParam<std::string>("frames/base_frame", "livox_frame");
    lidar_topic_ = getParam<std::string>("topics/lidar", "/livox/lidar");
    lidar_msg_type_ = getParam<std::string>("topics/lidar_msg_type", "livox");
    imu_topic_ = getParam<std::string>("topics/imu", "/livox/imu");
    filtered_points_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
    diagnostics_topic_ = getParam<std::string>("topics/diagnostics", "/dog_livo/diagnostics");
    ndt_odom_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
    ndt_pose_topic_ = getParam<std::string>("topics/ndt_pose", "/dog_livo/ndt_pose");
    ndt_path_topic_ = getParam<std::string>("topics/ndt_path", "/dog_livo/ndt_path");
    points_aligned_topic_ = getParam<std::string>("topics/points_aligned", "/dog_livo/points_aligned");
    deskew_enable_ = getParam<bool>("lidar_update/deskew_enable", false);
    lidar_offset_time_scale_ = getParam<double>("lidar_update/offset_time_scale", 1e-9);
    scan_reference_time_ = getParam<std::string>("lidar_update/scan_reference_time", "start");
    if (scan_reference_time_ != "start" && scan_reference_time_ != "mid" && scan_reference_time_ != "end")
    {
      ROS_WARN("[DogPriorMap NDT] invalid lidar_update/scan_reference_time='%s'; using start",
               scan_reference_time_.c_str());
      scan_reference_time_ = "start";
    }
    imu_history_keep_sec_ = getParam<double>("imu/history_keep_sec", 2.0);

    map_pcd_path_ = getParam<std::string>("map/pcd_fallback_path", "");
    map_voxel_size_ = getParam<double>("map/voxel_size", 0.30);
    map_voxel_z_size_ = getParam<double>("map/voxel_z_size", map_voxel_size_);
    scan_voxel_size_ = getParam<double>("lidar_update/ndt_source_voxel_size", 0.35);
    source_voxel_z_size_ = getParam<double>("lidar_update/ndt_source_voxel_z_size", std::max(scan_voxel_size_ * 0.5, 0.05));
    max_scan_points_ = getParam<int>("lidar_update/ndt_max_source_points", 900);
    target_voxel_size_ = getParam<double>("lidar_update/ndt_target_voxel_size", 0.30);
    target_voxel_z_size_ = getParam<double>("lidar_update/ndt_target_voxel_z_size", std::max(target_voxel_size_ * 0.8, 0.05));
    max_target_points_ = getParam<int>("lidar_update/ndt_max_target_points", 0);
    min_range_ = getParam<double>("lidar_update/scan_min_range", 0.5);
    max_range_ = getParam<double>("lidar_update/scan_max_range", 80.0);
    min_z_ = getParam<double>("lidar_update/scan_min_z", -std::numeric_limits<double>::infinity());
    max_z_ = getParam<double>("lidar_update/scan_max_z", std::numeric_limits<double>::infinity());
    min_effective_points_ = getParam<int>("lidar_update/min_effective_points", 50);
    ndt_resolution_ = getParam<double>("lidar_update/ndt_resolution", 1.0);
    ndt_step_size_ = getParam<double>("lidar_update/ndt_step_size", 0.1);
    ndt_transformation_epsilon_ = getParam<double>("lidar_update/ndt_transformation_epsilon", 0.001);
    ndt_max_iterations_ = getParam<int>("lidar_update/ndt_max_iterations", 30);
    ndt_step_limit_enable_ = getParam<bool>("lidar_update/ndt_step_limit_enable", false);
    ndt_step_limit_max_translation_ = getParam<double>("lidar_update/ndt_step_limit_max_translation", 0.5);
    ndt_step_limit_max_rotation_deg_ = getParam<double>("lidar_update/ndt_step_limit_max_rotation_deg", 5.0);
    local_imu_rotation_prior_enable_ = getParam<bool>(
        "lidar_update/local_imu_rotation_prior_enable", false);
    pending_lidar_max_frames_ = static_cast<size_t>(std::max(1,
        getParam<int>("lidar_update/pending_lidar_max_frames", 50)));
    lidar_subscriber_queue_size_ = static_cast<uint32_t>(std::max(1,
        getParam<int>("lidar_update/lidar_subscriber_queue_size", 100)));
    const bool determinism_requested = getParam<bool>("output/diagnostic_determinism_enable", false);
    determinism_diagnostic_enable_ = determinism_requested;
    determinism_csv_path_ = getParam<std::string>("output/ndt_determinism_csv_path", "");
    if (determinism_csv_path_.empty())
    {
      determinism_csv_path_ = getParam<std::string>("output/ndt_diagnostics_csv_path", "");
    }
    // Supplying an explicit CSV path is itself an opt-in for this observation
    // path.  With no path, the default remains completely behavior-neutral.
    determinism_diagnostic_enable_ = determinism_requested || !determinism_csv_path_.empty();
    if (determinism_diagnostic_enable_ && !determinism_csv_path_.empty())
    {
      determinism_csv_.open(determinism_csv_path_, std::ios::out);
      if (determinism_csv_.is_open())
      {
        determinism_csv_ << determinismCsvHeader() << "\n";
        determinism_csv_.flush();
      }
      else
      {
        ROS_WARN("[DogPriorMap NDT] failed to write determinism CSV: %s", determinism_csv_path_.c_str());
      }
    }
    else if (determinism_diagnostic_enable_)
    {
      ROS_WARN("[DogPriorMap NDT] determinism diagnostics enabled without an output CSV path");
      determinism_diagnostic_enable_ = false;
    }
    publish_tf_ = getParam<bool>("output/ndt_publish_tf", false);
    path_max_length_ = std::max(1, getParam<int>("output/path_max_length", 5000));
    path_sample_rate_hz_ = getParam<double>("output/path_sample_rate_hz", 2.0);
    path_publish_rate_hz_ = getParam<double>("output/path_publish_rate_hz", 1.0);
    if (!std::isfinite(path_sample_rate_hz_) || path_sample_rate_hz_ <= 0.0)
      path_sample_rate_hz_ = 2.0;
    if (!std::isfinite(path_publish_rate_hz_) || path_publish_rate_hz_ <= 0.0)
      path_publish_rate_hz_ = 1.0;
    publish_path_ = getParam<bool>("output/publish_path", false);
    publish_filtered_points_ = getParam<bool>("output/publish_filtered_points", true);
    publish_diagnostics_ = getParam<bool>("output/publish_diagnostics", true);

    loadMap();

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(ndt_odom_topic_, 20);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 20);
    pub_path_ = nh_.advertise<nav_msgs::Path>(ndt_path_topic_, 5);
    pub_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>(points_aligned_topic_, 5);
    if (publish_filtered_points_)
    {
      pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 5);
    }
    if (publish_diagnostics_)
    {
      pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    }
    path_.header.frame_id = map_frame_;

    if (lidar_msg_type_ == "pointcloud2")
    {
      sub_pc2_ = nh_.subscribe(lidar_topic_, lidar_subscriber_queue_size_,
                                &DogPriorMapNdtNode::pointCloud2Callback, this);
    }
    else
    {
      sub_livox_ = nh_.subscribe(lidar_topic_, lidar_subscriber_queue_size_,
                                 &DogPriorMapNdtNode::livoxCallback, this);
    }
    sub_imu_ = nh_.subscribe(imu_topic_, 500, &DogPriorMapNdtNode::imuCallback, this);

    ROS_INFO("[DogPriorMap NDT] started: map=%s target=%zu lidar=%s output=%s",
             map_pcd_path_.c_str(), target_cloud_->size(), lidar_topic_.c_str(), ndt_odom_topic_.c_str());
  }

  void imuCallback(const sensor_msgs::ImuConstPtr &msg)
  {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_imu_watermark_ || msg->header.stamp > latest_imu_stamp_)
    {
      latest_imu_stamp_ = msg->header.stamp;
      has_imu_watermark_ = true;
    }
    imu_history_.push_back(ImuSample{msg->header.stamp.toSec(),
                                     Eigen::Vector3d(msg->linear_acceleration.x,
                                                     msg->linear_acceleration.y,
                                                     msg->linear_acceleration.z),
                                     Eigen::Vector3d(msg->angular_velocity.x,
                                                     msg->angular_velocity.y,
                                                     msg->angular_velocity.z)});
    while (!imu_history_.empty() &&
           msg->header.stamp.toSec() - imu_history_.front().stamp > imu_history_keep_sec_)
    {
      imu_history_.pop_front();
    }
    if (local_imu_rotation_prior_enable_)
      processPendingCloudsLocked();
  }

private:
  struct PendingLidarFrame
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    ros::Time stamp;
    ros::Time reference_stamp;
    ros::Time scan_mid_stamp;
    ros::Time scan_end_stamp;
    double scan_min_offset_sec = 0.0;
    double scan_max_offset_sec = 0.0;
    ros::WallTime callback_start;
    uint32_t header_seq = 0;
  };

  struct ScanTiming
  {
    ros::Time start_stamp;
    ros::Time mid_stamp;
    ros::Time end_stamp;
    ros::Time reference_stamp;
    double min_offset_sec = 0.0;
    double max_offset_sec = 0.0;
  };

  struct DeterminismRow
  {
    uint64_t frame_index = 0;
    double lidar_header_stamp = std::numeric_limits<double>::quiet_NaN();
    double ros_now = std::numeric_limits<double>::quiet_NaN();
    double wall_time = std::numeric_limits<double>::quiet_NaN();
    uint32_t cloud_seq = 0;
    size_t cloud_size_raw = 0;
    size_t cloud_size_after_filter = 0;
    uint64_t cloud_hash = 0;
    std::string initial_guess_source = "not_evaluated";
    std::string initial_guess_reason = "not_evaluated";
    bool local_imu_prior_enabled = false;
    bool local_imu_prior_used = false;
    std::string local_imu_prior_reason = "not_evaluated";
    double local_imu_t0 = std::numeric_limits<double>::quiet_NaN();
    double local_imu_t1 = std::numeric_limits<double>::quiet_NaN();
    double local_imu_dt = std::numeric_limits<double>::quiet_NaN();
    double local_imu_history_first_stamp = std::numeric_limits<double>::quiet_NaN();
    double local_imu_history_last_stamp = std::numeric_limits<double>::quiet_NaN();
    double local_imu_delta_rotation_deg = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix4d baseline_guess = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d imu_rotation_guess = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    double baseline_to_imu_translation_diff = std::numeric_limits<double>::quiet_NaN();
    double baseline_to_imu_rotation_diff_deg = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d previous_pose = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d delta_pose = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d raw_ndt = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> raw_delta =
        Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    double raw_delta_translation = std::numeric_limits<double>::quiet_NaN();
    double raw_delta_rotation_deg = std::numeric_limits<double>::quiet_NaN();
    double ndt_fitness = std::numeric_limits<double>::quiet_NaN();
    bool ndt_has_converged = false;
    int ndt_iterations = 0;
    Eigen::Matrix4d used_before_step_limit = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    bool translation_limited = false;
    bool rotation_limited = false;
    Eigen::Matrix4d final_used = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    double scan_start_stamp = std::numeric_limits<double>::quiet_NaN();
    double scan_mid_stamp = std::numeric_limits<double>::quiet_NaN();
    double scan_end_stamp = std::numeric_limits<double>::quiet_NaN();
    double scan_min_offset_sec = std::numeric_limits<double>::quiet_NaN();
    double scan_max_offset_sec = std::numeric_limits<double>::quiet_NaN();
    double scan_duration_ms = std::numeric_limits<double>::quiet_NaN();
    size_t pending_lidar_count = 0;
    uint64_t pending_overflow_count = 0;
  };

  static std::string determinismCsvHeader()
  {
    return "frame_index,lidar_header_stamp,ros_now,wall_time,cloud_seq,cloud_size_raw,cloud_size_after_filter,cloud_hash,"
           "initial_guess_source,initial_guess_reason,"
           "local_imu_prior_enabled,local_imu_prior_used,local_imu_prior_reason,local_imu_t0,local_imu_t1,local_imu_dt,"
           "local_imu_history_first_stamp,local_imu_history_last_stamp,local_imu_delta_rotation_deg,"
           "baseline_guess_tx,baseline_guess_ty,baseline_guess_tz,baseline_guess_qx,baseline_guess_qy,baseline_guess_qz,baseline_guess_qw,"
           "imu_rotation_guess_tx,imu_rotation_guess_ty,imu_rotation_guess_tz,imu_rotation_guess_qx,imu_rotation_guess_qy,imu_rotation_guess_qz,imu_rotation_guess_qw,"
           "baseline_to_imu_translation_diff,baseline_to_imu_rotation_diff_deg,"
           "initial_guess_tx,initial_guess_ty,initial_guess_tz,initial_guess_qx,initial_guess_qy,initial_guess_qz,initial_guess_qw,"
           "previous_pose_tx,previous_pose_ty,previous_pose_tz,previous_pose_qx,previous_pose_qy,previous_pose_qz,previous_pose_qw,"
           "delta_pose_tx,delta_pose_ty,delta_pose_tz,delta_pose_qx,delta_pose_qy,delta_pose_qz,delta_pose_qw,"
           "raw_ndt_tx,raw_ndt_ty,raw_ndt_tz,raw_ndt_qx,raw_ndt_qy,raw_ndt_qz,raw_ndt_qw,"
           "raw_delta_tx,raw_delta_ty,raw_delta_tz,raw_delta_rx,raw_delta_ry,raw_delta_rz,"
           "raw_delta_from_guess_translation,raw_delta_from_guess_rotation_deg,ndt_fitness,ndt_has_converged,ndt_iterations,"
           "used_before_step_limit_tx,used_before_step_limit_ty,used_before_step_limit_tz,used_before_step_limit_qx,"
           "used_before_step_limit_qy,used_before_step_limit_qz,used_before_step_limit_qw,"
           "translation_limited,rotation_limited,final_used_tx,final_used_ty,final_used_tz,final_used_qx,final_used_qy,"
           "final_used_qz,final_used_qw,"
           "scan_start_stamp,scan_mid_stamp,scan_end_stamp,scan_min_offset_sec,scan_max_offset_sec,scan_duration_ms,"
           "pending_lidar_count,pending_overflow_count";
  }

  static void appendPoseCsv(std::ostream &out, const Eigen::Matrix4d &pose)
  {
    if (!pose.allFinite())
    {
      out << "nan,nan,nan,nan,nan,nan,nan";
      return;
    }
    Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    if (!q.coeffs().allFinite() || q.norm() < 1e-12)
    {
      out << "nan,nan,nan,nan,nan,nan,nan";
      return;
    }
    q.normalize();
    out << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
  }

  static uint64_t stableCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    constexpr uint64_t kOffset = 1469598103934665603ULL;
    constexpr uint64_t kPrime = 1099511628211ULL;
    uint64_t hash = kOffset;
    const auto mixByte = [&hash](uint8_t byte) {
      hash ^= static_cast<uint64_t>(byte);
      hash *= kPrime;
    };
    const auto mixU32 = [&mixByte](uint32_t value) {
      for (int shift = 0; shift < 32; shift += 8)
      {
        mixByte(static_cast<uint8_t>((value >> shift) & 0xffU));
      }
    };
    if (!cloud) return hash;
    mixU32(cloud->width);
    mixU32(cloud->height);
    mixU32(cloud->is_dense ? 1U : 0U);
    mixU32(static_cast<uint32_t>(cloud->size()));
    for (const auto &point : cloud->points)
    {
      uint32_t bits = 0;
      std::memcpy(&bits, &point.x, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.y, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.z, sizeof(bits));
      mixU32(bits);
    }
    return hash;
  }

  void writeDeterminismRow(const DeterminismRow &row)
  {
    if (!determinism_diagnostic_enable_ || !determinism_csv_.is_open()) return;
    std::ostringstream out;
    out << std::setprecision(17)
        << row.frame_index << "," << row.lidar_header_stamp << "," << row.ros_now << ","
        << row.wall_time << "," << row.cloud_seq << "," << row.cloud_size_raw << ","
        << row.cloud_size_after_filter << ","
        << std::hex << std::setw(16) << std::setfill('0') << row.cloud_hash << std::dec << std::setfill('0') << ","
        << row.initial_guess_source << "," << row.initial_guess_reason << ","
        << (row.local_imu_prior_enabled ? 1 : 0) << ","
        << (row.local_imu_prior_used ? 1 : 0) << ","
        << row.local_imu_prior_reason << ","
        << row.local_imu_t0 << "," << row.local_imu_t1 << "," << row.local_imu_dt << ","
        << row.local_imu_history_first_stamp << "," << row.local_imu_history_last_stamp << ","
        << row.local_imu_delta_rotation_deg << ",";
    appendPoseCsv(out, row.baseline_guess); out << ",";
    appendPoseCsv(out, row.imu_rotation_guess); out << ","
        << row.baseline_to_imu_translation_diff << ","
        << row.baseline_to_imu_rotation_diff_deg << ",";
    appendPoseCsv(out, row.initial_guess); out << ",";
    appendPoseCsv(out, row.previous_pose); out << ",";
    appendPoseCsv(out, row.delta_pose); out << ",";
    appendPoseCsv(out, row.raw_ndt); out << ","
        << row.raw_delta(0) << "," << row.raw_delta(1) << "," << row.raw_delta(2) << ","
        << row.raw_delta(3) << "," << row.raw_delta(4) << "," << row.raw_delta(5) << ","
        << row.raw_delta_translation << "," << row.raw_delta_rotation_deg << ","
        << row.ndt_fitness << "," << (row.ndt_has_converged ? 1 : 0) << "," << row.ndt_iterations << ",";
    appendPoseCsv(out, row.used_before_step_limit);
    out << "," << (row.translation_limited ? 1 : 0) << "," << (row.rotation_limited ? 1 : 0) << ",";
    appendPoseCsv(out, row.final_used);
    out << "," << row.scan_start_stamp << "," << row.scan_mid_stamp << "," << row.scan_end_stamp << ","
        << row.scan_min_offset_sec << "," << row.scan_max_offset_sec << "," << row.scan_duration_ms << ","
        << row.pending_lidar_count << "," << row.pending_overflow_count;
    out << "\n";
    determinism_csv_ << out.str();
  }

  // 优先读取全局参数，再读取节点私有参数，不存在时使用默认值。
  template <typename T>
  T getParam(const std::string &name, const T &default_value)
  {
    T value;
    if (nh_.getParam(name, value)) return value;
    if (pnh_.getParam(name, value)) return value;
    return default_value;
  }

  // 加载先验 PCD 地图，去除非法点并生成 NDT 使用的降采样目标点云。
  void loadMap()
  {
    if (map_pcd_path_.empty()) throw std::runtime_error("map/pcd_fallback_path is empty");

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(map_pcd_path_, *raw) != 0)
    {
      throw std::runtime_error("failed to load map pcd: " + map_pcd_path_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
    xyz->reserve(raw->size());
    for (const auto &pt : raw->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) xyz->push_back(pt);
    }
    finalizeCloud(xyz);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud =
        voxelDown(xyz, map_voxel_size_, map_voxel_z_size_, 0);
    target_cloud_ = voxelDown(map_cloud, target_voxel_size_, target_voxel_z_size_, max_target_points_);
    ndt_.setInputTarget(target_cloud_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
  }

  // 按 XYZ 分辨率执行各向异性体素降采样，并可限制最终点数。
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                double voxel_size,
                                                double voxel_z_size,
                                                int max_points) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    if (voxel_size > 0.01)
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(voxel_size, voxel_size, voxel_z_size);
      voxel.setInputCloud(cloud);
      voxel.filter(*down);
    }
    else
    {
      *down = *cloud;
    }

    if (max_points > 0 && static_cast<int>(down->size()) > max_points)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
      sampled->reserve(max_points);
      const double step = static_cast<double>(down->size() - 1) / static_cast<double>(std::max(max_points - 1, 1));
      for (int i = 0; i < max_points; ++i)
      {
        sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
      }
      finalizeCloud(sampled);
      return sampled;
    }
    finalizeCloud(down);
    return down;
  }

  // 对输入扫描执行有限值、距离和高度过滤，再进行体素降采样。
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->size());
    for (const auto &pt : cloud->points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
      const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r < min_range_ || r > max_range_) continue;
      if (pt.z < min_z_ || pt.z > max_z_) continue;
      filtered->push_back(pt);
    }
    finalizeCloud(filtered);
    return voxelDown(filtered, scan_voxel_size_, source_voxel_z_size_, max_scan_points_);
  }

  ScanTiming makeScanTiming(const ros::Time &start_stamp,
                            double min_offset_sec,
                            double max_offset_sec) const
  {
    ScanTiming timing;
    timing.start_stamp = start_stamp;
    timing.min_offset_sec = std::isfinite(min_offset_sec) ? min_offset_sec : 0.0;
    timing.max_offset_sec = std::isfinite(max_offset_sec) ? max_offset_sec : 0.0;
    timing.end_stamp = start_stamp + ros::Duration(timing.max_offset_sec);
    timing.mid_stamp = start_stamp + ros::Duration(0.5 * timing.max_offset_sec);
    if (scan_reference_time_ == "mid") timing.reference_stamp = timing.mid_stamp;
    else if (scan_reference_time_ == "end") timing.reference_stamp = timing.end_stamp;
    else timing.reference_stamp = timing.start_stamp;
    return timing;
  }

  void enqueueCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const ScanTiming &timing,
                    const ros::WallTime &callback_start,
                    uint32_t header_seq)
  {
    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_imu_rotation_prior_enable_)
    {
      handleCloudLocked(cloud, timing.start_stamp, callback_start, timing, header_seq);
      return;
    }

    PendingLidarFrame frame;
    frame.cloud = cloud;
    frame.stamp = timing.start_stamp;
    frame.reference_stamp = timing.reference_stamp;
    frame.scan_mid_stamp = timing.mid_stamp;
    frame.scan_end_stamp = timing.end_stamp;
    frame.scan_min_offset_sec = timing.min_offset_sec;
    frame.scan_max_offset_sec = timing.max_offset_sec;
    frame.callback_start = callback_start;
    frame.header_seq = header_seq;
    auto insert_at = pending_lidar_frames_.begin();
    while (insert_at != pending_lidar_frames_.end() &&
           (insert_at->reference_stamp < frame.reference_stamp ||
            (insert_at->reference_stamp == frame.reference_stamp && insert_at->stamp <= frame.stamp)))
      ++insert_at;
    pending_lidar_frames_.insert(insert_at, frame);

    while (pending_lidar_frames_.size() > pending_lidar_max_frames_)
    {
      PendingLidarFrame overflow = pending_lidar_frames_.front();
      pending_lidar_frames_.pop_front();
      ++pending_overflow_count_;
      handleCloudLocked(overflow.cloud, overflow.stamp, overflow.callback_start,
                        ScanTiming{overflow.stamp, overflow.scan_mid_stamp, overflow.scan_end_stamp,
                                   overflow.reference_stamp, overflow.scan_min_offset_sec,
                                   overflow.scan_max_offset_sec},
                        overflow.header_seq);
    }
    processPendingCloudsLocked();
  }

  void processPendingCloudsLocked()
  {
    while (!pending_lidar_frames_.empty())
    {
      const PendingLidarFrame &front = pending_lidar_frames_.front();
      const bool imu_watermark_ready = !local_imu_rotation_prior_enable_ ||
          (has_imu_watermark_ && latest_imu_stamp_ >= front.reference_stamp);
      if (!imu_watermark_ready)
      {
        break;
      }

      PendingLidarFrame ready = front;
      pending_lidar_frames_.pop_front();
      handleCloudLocked(ready.cloud, ready.stamp, ready.callback_start,
                        ScanTiming{ready.stamp, ready.scan_mid_stamp, ready.scan_end_stamp,
                                   ready.reference_stamp, ready.scan_min_offset_sec,
                                   ready.scan_max_offset_sec},
                        ready.header_seq);
    }
  }

  // 将 Livox 自定义消息转换为 XYZ 点云并进入统一 NDT 处理流程。
  void livoxCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
  {
    const ros::WallTime callback_start = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(msg->points.size());
    double min_offset_sec = std::numeric_limits<double>::infinity();
    double max_offset_sec = -std::numeric_limits<double>::infinity();
    for (const auto &pt : msg->points)
    {
      const double offset_sec = static_cast<double>(pt.offset_time) * lidar_offset_time_scale_;
      min_offset_sec = std::min(min_offset_sec, offset_sec);
      max_offset_sec = std::max(max_offset_sec, offset_sec);
    }
    if (!std::isfinite(min_offset_sec) || !std::isfinite(max_offset_sec))
    {
      min_offset_sec = 0.0;
      max_offset_sec = 0.0;
    }
    const ScanTiming timing = makeScanTiming(msg->header.stamp, min_offset_sec, max_offset_sec);
    const double frame_end = timing.end_stamp.toSec();
    for (const auto &pt : msg->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        if (deskew_enable_ && frame_end > msg->header.stamp.toSec())
        {
          const double point_time = msg->header.stamp.toSec() +
              static_cast<double>(pt.offset_time) * lidar_offset_time_scale_;
          p = integrateImuRotation(point_time, frame_end) * p;
        }
        cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
    }
    finalizeCloud(cloud);
    enqueueCloud(cloud, timing, callback_start, msg->header.seq);
  }

  bool integrateImuRotationFromHistoryLocked(double t0,
                                              double t1,
                                              Eigen::Matrix3d &rotation,
                                              std::string &reason) const
  {
    rotation = Eigen::Matrix3d::Identity();
    if (!std::isfinite(t0) || !std::isfinite(t1) || t1 <= t0)
    {
      reason = "invalid_interval";
      return false;
    }
    if (imu_history_.size() < 2)
    {
      reason = "insufficient_imu_history";
      return false;
    }
    if (!std::isfinite(imu_history_.front().stamp) ||
        !std::isfinite(imu_history_.back().stamp) ||
        imu_history_.front().stamp > t0 || imu_history_.back().stamp < t1)
    {
      reason = "imu_history_coverage";
      return false;
    }
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    double prev_t = t0;
    Eigen::Vector3d prev_g = imu_history_.front().gyro;
    for (size_t i = 1; i < imu_history_.size(); ++i)
    {
      if (imu_history_[i].stamp <= t0) { prev_g = imu_history_[i].gyro; continue; }
      const double seg_end = std::min(imu_history_[i].stamp, t1);
      if (seg_end > prev_t)
      {
        const Eigen::Vector3d g = 0.5 * (prev_g + imu_history_[i].gyro);
        const double dt = seg_end - prev_t;
        if (!g.allFinite() || !std::isfinite(dt) || dt <= 0.0)
        {
          reason = "nonfinite_imu_sample";
          return false;
        }
        const double angle = g.norm() * dt;
        if (!std::isfinite(angle))
        {
          reason = "nonfinite_imu_integration";
          return false;
        }
        if (angle > 1e-12) R = R * Eigen::AngleAxisd(angle, g.normalized()).toRotationMatrix();
        prev_t = seg_end;
      }
      prev_g = imu_history_[i].gyro;
      if (prev_t >= t1) break;
    }
    if (prev_t < t1 - 1e-9 || !R.allFinite())
    {
      reason = "imu_integration_incomplete";
      return false;
    }
    rotation = R;
    reason = "applied";
    return true;
  }

  Eigen::Matrix3d integrateImuRotation(double t0, double t1) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    std::string reason;
    integrateImuRotationFromHistoryLocked(t0, t1, rotation, reason);
    return rotation;
  }

  // 将标准 PointCloud2 转为 PCL 点云并进入统一 NDT 处理流程。
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    const ros::WallTime callback_start = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    enqueueCloud(cloud, makeScanTiming(msg->header.stamp, 0.0, 0.0), callback_start, msg->header.seq);
  }

  // 独立 NDT 主流程：预处理、确定性初值、配准、步长审核及结果发布。
  void handleCloudLocked(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         const ros::Time &stamp,
                         const ros::WallTime &callback_start,
                         const ScanTiming &timing,
                         uint32_t header_seq)
  {
    if (!cloud || cloud->empty()) return;

    DeterminismRow diagnostic_row;
    diagnostic_row.local_imu_prior_enabled = local_imu_rotation_prior_enable_;
    if (determinism_diagnostic_enable_)
    {
      diagnostic_row.frame_index = ++determinism_frame_index_;
      diagnostic_row.lidar_header_stamp = stamp.toSec();
      diagnostic_row.scan_start_stamp = timing.start_stamp.toSec();
      diagnostic_row.scan_mid_stamp = timing.mid_stamp.toSec();
      diagnostic_row.scan_end_stamp = timing.end_stamp.toSec();
      diagnostic_row.scan_min_offset_sec = timing.min_offset_sec;
      diagnostic_row.scan_max_offset_sec = timing.max_offset_sec;
      diagnostic_row.scan_duration_ms = (timing.end_stamp - timing.start_stamp).toSec() * 1000.0;
      diagnostic_row.ros_now = ros::Time::now().toSec();
      diagnostic_row.wall_time = ros::WallTime::now().toSec();
      diagnostic_row.cloud_seq = header_seq;
      diagnostic_row.cloud_size_raw = cloud->size();
      diagnostic_row.pending_lidar_count = pending_lidar_frames_.size();
      diagnostic_row.pending_overflow_count = pending_overflow_count_;
      if (!imu_history_.empty())
      {
        diagnostic_row.local_imu_history_first_stamp = imu_history_.front().stamp;
        diagnostic_row.local_imu_history_last_stamp = imu_history_.back().stamp;
      }
      if (has_previous_pose_)
      {
        diagnostic_row.previous_pose = previous_pose_;
        diagnostic_row.delta_pose = delta_pose_;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source = preprocess(cloud);
    if (determinism_diagnostic_enable_)
    {
      diagnostic_row.cloud_size_after_filter = source->size();
      diagnostic_row.cloud_hash = stableCloudHash(source);
    }
    const double preprocess_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    publishCloud(source, stamp, base_frame_, pub_filtered_);
    if (static_cast<int>(source->size()) < min_effective_points_)
    {
      const double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishDiagnostics(timing.reference_stamp, false, 0.0, preprocess_ms, localization_ms,
                         static_cast<int>(source->size()), target_cloud_->size(), 0.0, 0);
      diagnostic_row.initial_guess_source = "not_evaluated";
      diagnostic_row.initial_guess_reason = "insufficient_points";
      writeDeterminismRow(diagnostic_row);
      return;
    }

    Eigen::Matrix4d initial_guess = poseToMatrix(p_, R_);
    Eigen::Matrix4d baseline_guess = initial_guess;
    bool baseline_from_previous_pose = false;
    if (has_previous_pose_)
    {
      // The delivery initial guess is deliberately independent of EKF output:
      // use the previous NDT pose and its last accepted increment.
      initial_guess = previous_pose_ * delta_pose_;
      baseline_guess = initial_guess;
      baseline_from_previous_pose = true;
      diagnostic_row.initial_guess_source = "previous_pose_delta";
      diagnostic_row.initial_guess_reason = "applied";
    }
    else
    {
      diagnostic_row.initial_guess_source = "current_pose";
      diagnostic_row.initial_guess_reason = "no_previous_pose";
    }

    diagnostic_row.baseline_guess = baseline_guess;
    if (!local_imu_rotation_prior_enable_)
    {
      diagnostic_row.local_imu_prior_reason = "disabled";
    }
    else if (!baseline_from_previous_pose)
    {
      diagnostic_row.local_imu_prior_reason = "no_previous_pose";
      diagnostic_row.initial_guess_source = "current_pose";
      diagnostic_row.initial_guess_reason = diagnostic_row.local_imu_prior_reason;
    }
    else if (!has_previous_pose_reference_stamp_)
    {
      diagnostic_row.local_imu_prior_reason = "no_previous_reference_stamp";
      diagnostic_row.initial_guess_source = "previous_pose_delta";
      diagnostic_row.initial_guess_reason = diagnostic_row.local_imu_prior_reason;
    }
    else
    {
      diagnostic_row.local_imu_t0 = previous_pose_reference_stamp_.toSec();
      diagnostic_row.local_imu_t1 = timing.reference_stamp.toSec();
      diagnostic_row.local_imu_dt = diagnostic_row.local_imu_t1 - diagnostic_row.local_imu_t0;
      Eigen::Matrix3d delta_rotation = Eigen::Matrix3d::Identity();
      std::string imu_reason;
      const bool imu_ok = integrateImuRotationFromHistoryLocked(
          diagnostic_row.local_imu_t0, diagnostic_row.local_imu_t1,
          delta_rotation, imu_reason);
      diagnostic_row.local_imu_prior_reason = imu_reason;
      if (imu_ok)
      {
        initial_guess = baseline_guess;
        initial_guess.block<3, 3>(0, 0) =
            previous_pose_.block<3, 3>(0, 0) * delta_rotation;
        diagnostic_row.local_imu_prior_used = true;
        diagnostic_row.initial_guess_source = "local_imu_rotation_prior";
        diagnostic_row.initial_guess_reason = "applied";
        diagnostic_row.imu_rotation_guess = initial_guess;
        diagnostic_row.local_imu_delta_rotation_deg = rotationAngleDeg(delta_rotation);
        diagnostic_row.baseline_to_imu_translation_diff =
            (initial_guess.block<3, 1>(0, 3) - baseline_guess.block<3, 1>(0, 3)).norm();
        diagnostic_row.baseline_to_imu_rotation_diff_deg = rotationAngleDeg(
            baseline_guess.block<3, 3>(0, 0).transpose() *
            initial_guess.block<3, 3>(0, 0));
      }
      else
      {
        diagnostic_row.initial_guess_source = "previous_pose_delta";
        diagnostic_row.initial_guess_reason = imu_reason;
      }
    }
    diagnostic_row.initial_guess = initial_guess;

    ndt_.setInputSource(source);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    const ros::WallTime align_start = ros::WallTime::now();
    ndt_.align(aligned, initial_guess.cast<float>());
    const double align_ms = (ros::WallTime::now() - align_start).toSec() * 1000.0;
    const bool ok = ndt_.hasConverged();
    const double score = ndt_.getFitnessScore();
    const int iterations = ndt_.getFinalNumIteration();
    diagnostic_row.ndt_has_converged = ok;
    diagnostic_row.ndt_fitness = score;
    diagnostic_row.ndt_iterations = iterations;

    bool step_limited = false;
    double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    if (ok)
    {
      const Eigen::Matrix4d result = ndt_.getFinalTransformation().cast<double>();
      Eigen::Matrix4d used_result = result;
      diagnostic_row.raw_ndt = result;
      const Eigen::Matrix4d raw_delta = initial_guess.inverse() * result;
      const Eigen::AngleAxisd raw_delta_angle(raw_delta.block<3, 3>(0, 0));
      diagnostic_row.raw_delta.head<3>() = raw_delta.block<3, 1>(0, 3);
      diagnostic_row.raw_delta.tail<3>() = raw_delta_angle.axis() * raw_delta_angle.angle();
      diagnostic_row.raw_delta_translation = raw_delta.block<3, 1>(0, 3).norm();
      diagnostic_row.raw_delta_rotation_deg = rotationAngleDeg(raw_delta.block<3, 3>(0, 0));
      diagnostic_row.used_before_step_limit = used_result;
      step_limited = limitNdtStep(result, used_result,
                                  diagnostic_row.translation_limited,
                                  diagnostic_row.rotation_limited);
      diagnostic_row.final_used = used_result;
      if (has_previous_pose_)
      {
        delta_pose_ = previous_pose_.inverse() * used_result;
      }
      else
      {
        has_previous_pose_ = true;
        delta_pose_.setIdentity();
      }
      previous_pose_ = used_result;
      previous_pose_reference_stamp_ = timing.reference_stamp;
      has_previous_pose_reference_stamp_ = true;
      R_ = used_result.block<3, 3>(0, 0);
      p_ = used_result.block<3, 1>(0, 3);
      localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishPose(timing.reference_stamp);
      if (step_limited)
      {
        pcl::PointCloud<pcl::PointXYZ> limited_aligned = transformCloud(source, used_result);
        publishAlignedCloud(limited_aligned, stamp);
      }
      else
      {
        publishAlignedCloud(aligned, stamp);
      }
    }

    publishDiagnostics(timing.reference_stamp, ok, align_ms, preprocess_ms, localization_ms,
                       static_cast<int>(source->size()), target_cloud_->size(), score, iterations, step_limited);
    writeDeterminismRow(diagnostic_row);
    ROS_INFO_THROTTLE(1.0,
                      "[DogPriorMap NDT] conv=%d limited=%d source=%zu target=%zu align=%.2fms score=%.4f iter=%d p=(%.2f %.2f %.2f)",
                      ok ? 1 : 0, step_limited ? 1 : 0, source->size(), target_cloud_->size(), align_ms, score, iterations, p_.x(), p_.y(), p_.z());
  }

  // 使用给定齐次变换把源点云转换到地图坐标系。
  pcl::PointCloud<pcl::PointXYZ> transformCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                                 const Eigen::Matrix4d &pose) const
  {
    pcl::PointCloud<pcl::PointXYZ> transformed;
    if (!source) return transformed;
    transformed.reserve(source->size());
    const Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    const Eigen::Vector3d p = pose.block<3, 1>(0, 3);
    for (const auto &pt : source->points)
    {
      const Eigen::Vector3d q = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + p;
      transformed.push_back(pcl::PointXYZ(static_cast<float>(q.x()),
                                          static_cast<float>(q.y()),
                                          static_cast<float>(q.z())));
    }
    transformed.width = static_cast<uint32_t>(transformed.points.size());
    transformed.height = 1;
    transformed.is_dense = true;
    return transformed;
  }

  // 限制相邻 NDT 位姿跳变；超限时按比例截断平移和旋转增量。
  bool limitNdtStep(const Eigen::Matrix4d &raw_result,
                    Eigen::Matrix4d &used_result,
                    bool &translation_limited,
                    bool &rotation_limited) const
  {
    translation_limited = false;
    rotation_limited = false;
    if (!ndt_step_limit_enable_ || !has_previous_pose_) return false;

    bool limited = false;
    const Eigen::Vector3d previous_p = previous_pose_.block<3, 1>(0, 3);
    const Eigen::Vector3d raw_p = raw_result.block<3, 1>(0, 3);
    const Eigen::Vector3d dp = raw_p - previous_p;
    const double dist = dp.norm();

    const Eigen::Matrix3d previous_R = previous_pose_.block<3, 3>(0, 0);
    const Eigen::Matrix3d raw_R = raw_result.block<3, 3>(0, 0);
    const double angle_deg = rotationAngleDeg(previous_R.transpose() * raw_R);

    if (ndt_step_limit_max_translation_ > 0.0 && dist > ndt_step_limit_max_translation_)
    {
      used_result.block<3, 1>(0, 3) = previous_p + dp.normalized() * ndt_step_limit_max_translation_;
      limited = true;
      translation_limited = true;
    }
    if (ndt_step_limit_max_rotation_deg_ > 0.0 && angle_deg > ndt_step_limit_max_rotation_deg_)
    {
      const double ratio = ndt_step_limit_max_rotation_deg_ / std::max(angle_deg, 1e-6);
      Eigen::Quaterniond q_prev(previous_R);
      Eigen::Quaterniond q_raw(raw_R);
      used_result.block<3, 3>(0, 0) = q_prev.normalized().slerp(ratio, q_raw.normalized()).toRotationMatrix();
      limited = true;
      rotation_limited = true;
    }
    return limited;
  }

  // 发布 NDT 位姿、里程计、可选轨迹以及 map 到 base 的 TF。
  void publishPose(const ros::Time &stamp)
  {
    Eigen::Quaterniond q(R_);
    q.normalize();

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p_.x();
    odom.pose.pose.position.y = p_.y();
    odom.pose.pose.position.z = p_.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pub_odom_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pub_pose_.publish(pose);

    if (publish_path_)
    {
      const double sensor_stamp = stamp.toSec();
      const double sample_interval = 1.0 / path_sample_rate_hz_;
      if (std::isfinite(sensor_stamp) &&
          (next_ndt_path_sample_stamp_ < 0.0 ||
           sensor_stamp >= next_ndt_path_sample_stamp_))
      {
        if (last_ndt_path_sample_stamp_ < 0.0 ||
            sensor_stamp > last_ndt_path_sample_stamp_)
        {
          last_ndt_path_sample_stamp_ = sensor_stamp;
          path_.header.stamp = stamp;
          path_.header.frame_id = map_frame_;
          path_.poses.push_back(pose);
          if (static_cast<int>(path_.poses.size()) > path_max_length_)
            path_.poses.erase(path_.poses.begin(),
                              path_.poses.begin() + (path_.poses.size() - path_max_length_));
        }
        if (next_ndt_path_sample_stamp_ < 0.0)
          next_ndt_path_sample_stamp_ = sensor_stamp + sample_interval;
        else
        {
          do
          {
            next_ndt_path_sample_stamp_ += sample_interval;
          } while (next_ndt_path_sample_stamp_ <= sensor_stamp);
        }
      }
      const double publish_interval = 1.0 / path_publish_rate_hz_;
      if (std::isfinite(sensor_stamp) &&
          (next_ndt_path_publish_stamp_ < 0.0 ||
           sensor_stamp >= next_ndt_path_publish_stamp_))
      {
        if (last_ndt_path_publish_stamp_ < 0.0 ||
            sensor_stamp > last_ndt_path_publish_stamp_)
        {
          last_ndt_path_publish_stamp_ = sensor_stamp;
          path_.header.stamp = stamp;
          path_.header.frame_id = map_frame_;
          pub_path_.publish(path_);
        }
        if (next_ndt_path_publish_stamp_ < 0.0)
          next_ndt_path_publish_stamp_ = sensor_stamp + publish_interval;
        else
        {
          do
          {
            next_ndt_path_publish_stamp_ += publish_interval;
          } while (next_ndt_path_publish_stamp_ <= sensor_stamp);
        }
      }
    }

    if (publish_tf_)
    {
      tf::Transform tf_msg;
      tf_msg.setOrigin(tf::Vector3(p_.x(), p_.y(), p_.z()));
      tf_msg.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
      tf_broadcaster_.sendTransform(tf::StampedTransform(tf_msg, stamp, map_frame_, base_frame_));
    }
  }

  // 将指定坐标系中的 PCL 点云转换为 ROS 消息并发布。
  void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const ros::Time &stamp,
                    const std::string &frame_id,
                    ros::Publisher &publisher)
  {
    if (!publisher || !cloud) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    publisher.publish(msg);
  }

  // 发布已经变换到地图系的 NDT 对齐点云，用于观察配准效果。
  void publishAlignedCloud(const pcl::PointCloud<pcl::PointXYZ> &aligned, const ros::Time &stamp)
  {
    if (!pub_aligned_) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(aligned, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    pub_aligned_.publish(msg);
  }

  // 发布 NDT 收敛状态、耗时、点数、适应度和当前位姿等诊断量。
  void publishDiagnostics(const ros::Time &stamp,
                          bool converged,
                          double align_ms,
                          double preprocess_ms,
                          double localization_ms,
                          int scan_points,
                          size_t map_points,
                          double score,
                          int iterations,
                          bool step_limited = false)
  {
    if (!publish_diagnostics_ || !pub_diagnostics_) return;
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "dog_prior_map_ndt";
    status.hardware_id = base_frame_;
    status.level = converged ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = converged ? "NDT converged" : "NDT not converged";

    addDiagnosticValue(status, "ndt_converged", converged ? "true" : "false");
    addDiagnosticValue(status, "align_time_ms", std::to_string(align_ms));
    addDiagnosticValue(status, "preprocess_time_ms", std::to_string(preprocess_ms));
    addDiagnosticValue(status, "localization_time_ms", std::to_string(localization_ms));
    addDiagnosticValue(status, "scan_points", std::to_string(scan_points));
    addDiagnosticValue(status, "map_points", std::to_string(map_points));
    addDiagnosticValue(status, "fitness_score", std::to_string(score));
    addDiagnosticValue(status, "iterations", std::to_string(iterations));
    addDiagnosticValue(status, "pose_xyz", std::to_string(p_.x()) + "," +
                                  std::to_string(p_.y()) + "," +
                                  std::to_string(p_.z()));
    array.status.push_back(status);
    pub_diagnostics_.publish(array);
  }

  // 向 DiagnosticStatus 追加一个键值字段，统一诊断消息构造方式。
  void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus &status,
                          const std::string &key,
                          const std::string &value)
  {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_livox_;
  ros::Subscriber sub_pc2_;
  ros::Subscriber sub_imu_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_path_;
  ros::Publisher pub_filtered_;
  ros::Publisher pub_aligned_;
  ros::Publisher pub_diagnostics_;
  tf::TransformBroadcaster tf_broadcaster_;
  mutable std::mutex mutex_;

  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_topic_;
  std::string lidar_msg_type_;
  std::string filtered_points_topic_;
  std::string diagnostics_topic_;
  std::string ndt_odom_topic_;
  std::string ndt_pose_topic_;
  std::string ndt_path_topic_;
  std::string points_aligned_topic_;
  std::string map_pcd_path_;
  std::string imu_topic_;

  struct ImuSample
  {
    double stamp = 0.0;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  };
  std::deque<ImuSample> imu_history_;
  bool has_imu_watermark_ = false;
  ros::Time latest_imu_stamp_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  nav_msgs::Path path_;

  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool has_previous_pose_ = false;
  Eigen::Matrix4d previous_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose_ = Eigen::Matrix4d::Identity();
  bool has_previous_pose_reference_stamp_ = false;
  ros::Time previous_pose_reference_stamp_;
  std::deque<PendingLidarFrame> pending_lidar_frames_;
  uint64_t pending_overflow_count_ = 0;

  double map_voxel_size_ = 0.30;
  double map_voxel_z_size_ = 0.30;
  double target_voxel_size_ = 0.30;
  double target_voxel_z_size_ = 0.30;
  double scan_voxel_size_ = 0.35;
  double source_voxel_z_size_ = 0.35;
  int max_scan_points_ = 900;
  int max_target_points_ = 0;
  double min_range_ = 0.5;
  double max_range_ = 80.0;
  double min_z_ = -std::numeric_limits<double>::infinity();
  double max_z_ = std::numeric_limits<double>::infinity();
  int min_effective_points_ = 50;
  int ndt_max_iterations_ = 30;
  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_transformation_epsilon_ = 0.001;
  bool ndt_step_limit_enable_ = false;
  double ndt_step_limit_max_translation_ = 0.5;
  double ndt_step_limit_max_rotation_deg_ = 5.0;
  bool local_imu_rotation_prior_enable_ = false;
  size_t pending_lidar_max_frames_ = 50;
  uint32_t lidar_subscriber_queue_size_ = 100;
  bool determinism_diagnostic_enable_ = false;
  std::string determinism_csv_path_;
  std::ofstream determinism_csv_;
  uint64_t determinism_frame_index_ = 0;
  bool deskew_enable_ = false;
  double lidar_offset_time_scale_ = 1e-9;
  double imu_history_keep_sec_ = 2.0;
  bool publish_tf_ = true;
  bool publish_path_ = false;
  int path_max_length_ = 5000;
  double path_sample_rate_hz_ = 2.0;
  double path_publish_rate_hz_ = 1.0;
  double last_ndt_path_sample_stamp_ = -1.0;
  double last_ndt_path_publish_stamp_ = -1.0;
  double next_ndt_path_sample_stamp_ = -1.0;
  double next_ndt_path_publish_stamp_ = -1.0;
  bool publish_filtered_points_ = true;
  bool publish_diagnostics_ = true;
  std::string scan_reference_time_ = "start";
};
}  // namespace dog_prior_map_localization

// 独立 NDT 可执行程序入口：初始化节点并持续处理点云回调。
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dog_prior_map_ndt");
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    bool external_transaction_enabled = false;
    if (!nh.getParam("external_transaction/enabled", external_transaction_enabled))
      pnh.param("external_transaction/enabled", external_transaction_enabled, false);
    if (external_transaction_enabled)
    {
      dog_prior_map_localization::ExternalNdtTransactionServer node;
      ros::spin();
      return 0;
    }
    dog_prior_map_localization::DogPriorMapNdtNode node;
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[DogPriorMap NDT] node exited: %s", e.what());
    return 1;
  }
  return 0;
}
