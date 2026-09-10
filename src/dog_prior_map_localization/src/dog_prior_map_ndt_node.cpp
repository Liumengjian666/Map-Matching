#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

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
#include <tf/transform_broadcaster.h>

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

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
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
    filtered_points_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
    diagnostics_topic_ = getParam<std::string>("topics/diagnostics", "/dog_livo/diagnostics");
    prior_map_topic_ = getParam<std::string>("topics/prior_map", "/dog_livo/prior_map");
    ndt_odom_topic_ = getParam<std::string>("topics/ndt_odom", "/dog_livo/ndt_odom");
    ndt_pose_topic_ = getParam<std::string>("topics/ndt_pose", "/dog_livo/ndt_pose");
    ndt_path_topic_ = getParam<std::string>("topics/ndt_path", "/dog_livo/ndt_path");
    points_aligned_topic_ = getParam<std::string>("topics/points_aligned", "/dog_livo/points_aligned");
    prediction_topic_ = getParam<std::string>("topics/odom_high_rate", "/dog_livo/odom_high_rate");

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
    ndt_hard_reject_max_translation_ =
        getParam<double>("lidar_update/ndt_hard_reject_max_translation", 0.5);
    ndt_hard_reject_max_fitness_ =
        getParam<double>("lidar_update/ndt_hard_reject_max_fitness", 5.0);
    reliability_fitness_scale_ = getParam<double>("reliability/fitness_scale", 2.0);
    reliability_translation_scale_ = getParam<double>("reliability/translation_scale", 0.50);
    reliability_rotation_scale_deg_ = getParam<double>("reliability/rotation_scale_deg", 5.0);
    reliability_temporal_translation_scale_ = getParam<double>("reliability/temporal_translation_scale", 0.35);
    reliability_temporal_rotation_scale_deg_ = getParam<double>("reliability/temporal_rotation_scale_deg", 3.0);
    reliability_geometry_ratio_scale_ = getParam<double>("reliability/geometry_ratio_scale", 0.10);
    reliability_iteration_scale_ = getParam<double>("reliability/iteration_scale", 10.0);
    reliability_position_std_ = getParam<double>("reliability/position_std_m", 0.08);
    reliability_rotation_std_ = getParam<double>("reliability/rotation_std_deg", 1.0) * M_PI / 180.0;
    reliability_min_score_ = getParam<double>("reliability/min_score", 0.10);
    publish_tf_ = getParam<bool>("output/ndt_publish_tf", false);
    publish_path_ = getParam<bool>("output/publish_path", false);
    publish_filtered_points_ = getParam<bool>("output/publish_filtered_points", true);
    publish_diagnostics_ = getParam<bool>("output/publish_diagnostics", true);

    loadMap();

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(ndt_odom_topic_, 20);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 20);
    pub_path_ = nh_.advertise<nav_msgs::Path>(ndt_path_topic_, 5);
    pub_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>(points_aligned_topic_, 5);
    pub_prior_map_ = nh_.advertise<sensor_msgs::PointCloud2>(prior_map_topic_, 1, true);
    if (publish_filtered_points_)
    {
      pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 5);
    }
    if (publish_diagnostics_)
    {
      pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    }
    path_.header.frame_id = map_frame_;
    publishCloud(map_cloud_, ros::Time::now(), map_frame_, pub_prior_map_);

    if (lidar_msg_type_ == "pointcloud2")
    {
      sub_pc2_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapNdtNode::pointCloud2Callback, this);
    }
    else
    {
      sub_livox_ = nh_.subscribe(lidar_topic_, 5, &DogPriorMapNdtNode::livoxCallback, this);
    }
    sub_prediction_ = nh_.subscribe(prediction_topic_, 10,
                                    &DogPriorMapNdtNode::predictionCallback, this);

    ROS_INFO("[DogPriorMap NDT] started: map=%s target=%zu lidar=%s output=%s",
             map_pcd_path_.c_str(), target_cloud_->size(), lidar_topic_.c_str(), ndt_odom_topic_.c_str());
  }

private:
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
    map_cloud_ = voxelDown(xyz, map_voxel_size_, map_voxel_z_size_, 0);
    target_cloud_ = voxelDown(map_cloud_, target_voxel_size_, target_voxel_z_size_, max_target_points_);

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

  // XY协方差最小/最大特征值之比；越接近零，平面内结构越接近单方向退化。
  double computeXyGeometryRatio(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
  {
    if (!cloud || cloud->size() < 3) return 0.0;
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto &point : cloud->points) mean += Eigen::Vector2d(point.x, point.y);
    mean /= static_cast<double>(cloud->size());
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto &point : cloud->points)
    {
      const Eigen::Vector2d centered = Eigen::Vector2d(point.x, point.y) - mean;
      covariance.noalias() += centered * centered.transpose();
    }
    covariance /= static_cast<double>(cloud->size() - 1);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success) return 0.0;
    const double smallest = std::max(0.0, solver.eigenvalues()(0));
    const double largest = std::max(0.0, solver.eigenvalues()(1));
    return largest > 1e-9 ? smallest / largest : 0.0;
  }

  // 将 Livox 自定义消息转换为 XYZ 点云并进入统一 NDT 处理流程。
  void predictionCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    const Eigen::Vector3d p(msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            msg->pose.pose.position.z);
    if (!p.allFinite() || q.norm() < 1e-9) return;
    imu_prediction_pose_ = poseToMatrix(p, q.normalized().toRotationMatrix());
    has_imu_prediction_ = true;
  }

  // 将 Livox 自定义消息转换为 XYZ 点云并进入统一 NDT 处理流程。
  void livoxCallback(const livox_ros_driver2::CustomMsgConstPtr &msg)
  {
    const ros::WallTime callback_start = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(msg->points.size());
    for (const auto &pt : msg->points)
    {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
      {
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
      }
    }
    finalizeCloud(cloud);
    handleCloud(cloud, msg->header.stamp, callback_start);
  }

  // 将标准 PointCloud2 转为 PCL 点云并进入统一 NDT 处理流程。
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    const ros::WallTime callback_start = ros::WallTime::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    handleCloud(cloud, msg->header.stamp, callback_start);
  }

  // 独立 NDT 主流程：预处理、预测初值、配准、步长审核及结果发布。
  void handleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                   const ros::Time &stamp,
                   const ros::WallTime &callback_start)
  {
    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lock(mutex_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source = preprocess(cloud);
    const double preprocess_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    publishCloud(source, stamp, base_frame_, pub_filtered_);
    if (static_cast<int>(source->size()) < min_effective_points_)
    {
      const double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishDiagnostics(stamp, false, 0.0, preprocess_ms, localization_ms,
                         static_cast<int>(source->size()), target_cloud_->size(), 0.0, 0);
      return;
    }

    Eigen::Matrix4d initial_guess = poseToMatrix(p_, R_);
    if (has_imu_prediction_)
    {
      // NDT 的优化初值来自 IMU/EKF 传播，而不是 NDT 自己的历史外推。
      initial_guess = imu_prediction_pose_;
    }
    else if (has_previous_pose_)
    {
      initial_guess = previous_pose_ * delta_pose_;
    }

    ndt_.setInputSource(source);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    const ros::WallTime align_start = ros::WallTime::now();
    ndt_.align(aligned, initial_guess.cast<float>());
    const double align_ms = (ros::WallTime::now() - align_start).toSec() * 1000.0;
    const bool ok = ndt_.hasConverged();
    const double score = ndt_.getFitnessScore();
    const int iterations = ndt_.getFinalNumIteration();

    double innovation_translation = 0.0;
    double innovation_rotation_deg = 0.0;
    double raw_step_translation = 0.0;
    double raw_step_rotation_deg = 0.0;
    double temporal_translation = 0.0;
    double temporal_rotation_deg = 0.0;
    Eigen::Matrix4d raw_result = initial_guess;
    if (ok)
    {
      raw_result = ndt_.getFinalTransformation().cast<double>();
      const Eigen::Matrix4d innovation = initial_guess.inverse() * raw_result;
      innovation_translation = innovation.block<3, 1>(0, 3).norm();
      innovation_rotation_deg = rotationAngleDeg(innovation.block<3, 3>(0, 0));
      if (has_previous_pose_)
      {
        const Eigen::Matrix4d raw_delta = previous_pose_.inverse() * raw_result;
        raw_step_translation = raw_delta.block<3, 1>(0, 3).norm();
        raw_step_rotation_deg = rotationAngleDeg(raw_delta.block<3, 3>(0, 0));
        const Eigen::Matrix4d temporal_error = delta_pose_.inverse() * raw_delta;
        temporal_translation = temporal_error.block<3, 1>(0, 3).norm();
        temporal_rotation_deg = rotationAngleDeg(temporal_error.block<3, 3>(0, 0));
      }
    }

    if (ok && ndt_hard_reject_max_translation_ > 0.0)
    {
      const Eigen::Vector3d raw_p = raw_result.block<3, 1>(0, 3);
      const double prediction_jump = has_imu_prediction_
          ? (raw_p - imu_prediction_pose_.block<3, 1>(0, 3)).norm()
          : std::numeric_limits<double>::infinity();
      const double previous_jump = has_previous_pose_
          ? (raw_p - previous_pose_.block<3, 1>(0, 3)).norm()
          : prediction_jump;
      if ((has_imu_prediction_ && prediction_jump > ndt_hard_reject_max_translation_) ||
          (!has_imu_prediction_ && has_previous_pose_ && previous_jump > ndt_hard_reject_max_translation_))
      {
        ROS_WARN_THROTTLE(1.0,
                          "[DogPriorMap NDT] drop candidate: jump_to_imu=%.3f jump_to_previous=%.3f limit=%.3f",
                          prediction_jump, previous_jump, ndt_hard_reject_max_translation_);
        publishDiagnostics(stamp, false, align_ms, preprocess_ms,
                           (ros::WallTime::now() - callback_start).toSec() * 1000.0,
                           static_cast<int>(source->size()), target_cloud_->size(), score, iterations);
        return;
      }
    }
    if (ok && ndt_hard_reject_max_fitness_ > 0.0 &&
        (!std::isfinite(score) || score > ndt_hard_reject_max_fitness_))
    {
      ROS_WARN_THROTTLE(1.0,
                        "[DogPriorMap NDT] drop candidate: fitness=%.3f > %.3f",
                        score, ndt_hard_reject_max_fitness_);
      publishDiagnostics(stamp, false, align_ms, preprocess_ms,
                         (ros::WallTime::now() - callback_start).toSec() * 1000.0,
                         static_cast<int>(source->size()), target_cloud_->size(), score, iterations);
      return;
    }
    const double geometry_ratio = computeXyGeometryRatio(source);
    const double fitness_quality = ok && std::isfinite(score)
        ? 1.0 / (1.0 + std::max(0.0, score) / std::max(reliability_fitness_scale_, 1e-6))
        : 0.0;
    const double innovation_quality = std::exp(-0.5 * std::pow(
        innovation_translation / std::max(reliability_translation_scale_, 1e-6), 2.0) -
        0.5 * std::pow(innovation_rotation_deg / std::max(reliability_rotation_scale_deg_, 1e-6), 2.0));
    const double temporal_quality = has_previous_pose_ ? std::exp(-0.5 * std::pow(
        temporal_translation / std::max(reliability_temporal_translation_scale_, 1e-6), 2.0) -
        0.5 * std::pow(temporal_rotation_deg / std::max(reliability_temporal_rotation_scale_deg_, 1e-6), 2.0)) : 1.0;
    const double geometry_quality = clamp01(geometry_ratio /
        std::max(reliability_geometry_ratio_scale_, 1e-6));
    const double iteration_quality = std::exp(-static_cast<double>(iterations) /
        std::max(reliability_iteration_scale_, 1e-6));
    const double reliability_score = ok ? std::pow(std::max(
        fitness_quality * innovation_quality * temporal_quality * geometry_quality * iteration_quality,
        1e-12), 0.2) : 0.0;

    bool step_limited = false;
    double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    if (ok)
    {
      Eigen::Matrix4d used_result = raw_result;
      step_limited = limitNdtStep(raw_result, used_result);
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
      R_ = used_result.block<3, 3>(0, 0);
      p_ = used_result.block<3, 1>(0, 3);
      localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishPose(stamp, reliability_score);
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

    publishDiagnostics(stamp, ok, align_ms, preprocess_ms, localization_ms,
                       static_cast<int>(source->size()), target_cloud_->size(), score, iterations, step_limited,
                       innovation_translation, innovation_rotation_deg,
                       raw_step_translation, raw_step_rotation_deg,
                       temporal_translation, temporal_rotation_deg,
                       geometry_ratio, fitness_quality, innovation_quality,
                       temporal_quality, geometry_quality, iteration_quality, reliability_score);
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
  bool limitNdtStep(const Eigen::Matrix4d &raw_result, Eigen::Matrix4d &used_result) const
  {
    if (!ndt_step_limit_enable_ || !has_previous_pose_) return false;

    bool limited = false;
    used_result = raw_result;

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
    }
    if (ndt_step_limit_max_rotation_deg_ > 0.0 && angle_deg > ndt_step_limit_max_rotation_deg_)
    {
      const double ratio = ndt_step_limit_max_rotation_deg_ / std::max(angle_deg, 1e-6);
      Eigen::Quaterniond q_prev(previous_R);
      Eigen::Quaterniond q_raw(raw_R);
      used_result.block<3, 3>(0, 0) = q_prev.normalized().slerp(ratio, q_raw.normalized()).toRotationMatrix();
      limited = true;
    }
    return limited;
  }

  // 发布 NDT 位姿、里程计、可选轨迹以及 map 到 base 的 TF。
  void publishPose(const ros::Time &stamp, double reliability_score)
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
    const double bounded_score = std::max(reliability_min_score_, clamp01(reliability_score));
    const double position_variance = std::pow(reliability_position_std_ / bounded_score, 2.0);
    const double rotation_variance = std::pow(reliability_rotation_std_ / bounded_score, 2.0);
    odom.pose.covariance[0] = position_variance;
    odom.pose.covariance[7] = position_variance;
    odom.pose.covariance[14] = position_variance;
    odom.pose.covariance[21] = rotation_variance;
    odom.pose.covariance[28] = rotation_variance;
    odom.pose.covariance[35] = rotation_variance;
    pub_odom_.publish(odom);

    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    pub_pose_.publish(pose);

    if (publish_path_)
    {
      path_.header.stamp = stamp;
      path_.header.frame_id = map_frame_;
      path_.poses.push_back(pose);
      if (path_.poses.size() > 5000) path_.poses.erase(path_.poses.begin());
      pub_path_.publish(path_);
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
                          bool step_limited = false,
                          double innovation_translation = 0.0,
                          double innovation_rotation_deg = 0.0,
                          double raw_step_translation = 0.0,
                          double raw_step_rotation_deg = 0.0,
                          double temporal_translation = 0.0,
                          double temporal_rotation_deg = 0.0,
                          double geometry_ratio = 0.0,
                          double fitness_quality = 0.0,
                          double innovation_quality = 0.0,
                          double temporal_quality = 0.0,
                          double geometry_quality = 0.0,
                          double iteration_quality = 0.0,
                          double reliability_score = 0.0)
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
    addDiagnosticValue(status, "step_limited", step_limited ? "true" : "false");
    addDiagnosticValue(status, "innovation_translation_m", std::to_string(innovation_translation));
    addDiagnosticValue(status, "innovation_rotation_deg", std::to_string(innovation_rotation_deg));
    addDiagnosticValue(status, "raw_step_translation_m", std::to_string(raw_step_translation));
    addDiagnosticValue(status, "raw_step_rotation_deg", std::to_string(raw_step_rotation_deg));
    addDiagnosticValue(status, "temporal_translation_m", std::to_string(temporal_translation));
    addDiagnosticValue(status, "temporal_rotation_deg", std::to_string(temporal_rotation_deg));
    addDiagnosticValue(status, "xy_geometry_ratio", std::to_string(geometry_ratio));
    addDiagnosticValue(status, "fitness_quality", std::to_string(fitness_quality));
    addDiagnosticValue(status, "innovation_quality", std::to_string(innovation_quality));
    addDiagnosticValue(status, "temporal_quality", std::to_string(temporal_quality));
    addDiagnosticValue(status, "geometry_quality", std::to_string(geometry_quality));
    addDiagnosticValue(status, "iteration_quality", std::to_string(iteration_quality));
    addDiagnosticValue(status, "reliability_score", std::to_string(reliability_score));
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
  ros::Subscriber sub_prediction_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_path_;
  ros::Publisher pub_filtered_;
  ros::Publisher pub_aligned_;
  ros::Publisher pub_prior_map_;
  ros::Publisher pub_diagnostics_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;

  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_topic_;
  std::string lidar_msg_type_;
  std::string filtered_points_topic_;
  std::string diagnostics_topic_;
  std::string prior_map_topic_;
  std::string ndt_odom_topic_;
  std::string ndt_pose_topic_;
  std::string ndt_path_topic_;
  std::string points_aligned_topic_;
  std::string prediction_topic_;
  std::string map_pcd_path_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  nav_msgs::Path path_;

  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
  bool has_previous_pose_ = false;
  Eigen::Matrix4d previous_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d delta_pose_ = Eigen::Matrix4d::Identity();
  bool has_imu_prediction_ = false;
  Eigen::Matrix4d imu_prediction_pose_ = Eigen::Matrix4d::Identity();

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
  double ndt_hard_reject_max_translation_ = 0.5;
  double ndt_hard_reject_max_fitness_ = 5.0;
  double reliability_fitness_scale_ = 2.0;
  double reliability_translation_scale_ = 0.50;
  double reliability_rotation_scale_deg_ = 5.0;
  double reliability_temporal_translation_scale_ = 0.35;
  double reliability_temporal_rotation_scale_deg_ = 3.0;
  double reliability_geometry_ratio_scale_ = 0.10;
  double reliability_iteration_scale_ = 10.0;
  double reliability_position_std_ = 0.08;
  double reliability_rotation_std_ = M_PI / 180.0;
  double reliability_min_score_ = 0.10;
  bool publish_tf_ = true;
  bool publish_path_ = false;
  bool publish_filtered_points_ = true;
  bool publish_diagnostics_ = true;
};
}  // namespace dog_prior_map_localization

// 独立 NDT 可执行程序入口：初始化节点并持续处理点云回调。
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dog_prior_map_ndt");
  try
  {
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
