#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <deque>
#include <cstring>
#include <ctime>
#include <fstream>
#include <fcntl.h>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <dog_prior_map_interfaces/VisualRelativeMeasurement.h>
#include <dog_prior_map_interfaces/VisualDegeneracy.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include "dog_prior_map_localization/external_ndt_transaction_server.hpp"
#include "dog_prior_map_localization/core/frame_conversions.hpp"
#include "dog_prior_map_localization/core/causal_pose_prediction.hpp"
#include "dog_prior_map_localization/core/ekf_translation_prior.hpp"
#include "dog_prior_map_localization/visual_aided_initial_guess.hpp"
#include "dog_prior_map_localization/experimental_source_capture.hpp"
#include "dog_prior_map_localization/local_map_tiles.hpp"
#include "dog_prior_map_localization/mature_ndt_algorithms.hpp"
#include "dog_prior_map_localization/skf_visual_measurement.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"

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

struct GeometryStats
{
  std::size_t points = 0;
  double median = std::numeric_limits<double>::quiet_NaN();
  double p90 = std::numeric_limits<double>::quiet_NaN();
  double overlap_05 = std::numeric_limits<double>::quiet_NaN();
  double overlap_2 = std::numeric_limits<double>::quiet_NaN();
  double overlap_8 = std::numeric_limits<double>::quiet_NaN();
};

double sortedPercentile(const std::vector<double> &values, double fraction)
{
  if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
  const std::size_t index = static_cast<std::size_t>(
      std::floor(fraction * static_cast<double>(values.size() - 1)));
  return values[index];
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
    if (getParam<bool>("initialization/ndt_seed_enable", false))
    {
      const auto translation = getParam<std::vector<double>>(
          "initialization/ndt_seed_map_T_lidar_translation_m", std::vector<double>());
      const auto rotation = getParam<std::vector<double>>(
          "initialization/ndt_seed_map_T_lidar_rotation_row_major", std::vector<double>());
      if (translation.size() != 3 || rotation.size() != 9)
        throw std::runtime_error("NDT startup seed requires 3 translation and 9 rotation values");
      Eigen::Matrix3d seed_rotation;
      for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
          seed_rotation(row, column) = rotation[static_cast<std::size_t>(row * 3 + column)];
      const Eigen::Vector3d seed_translation(translation[0], translation[1], translation[2]);
      Eigen::Matrix4d seed_pose = Eigen::Matrix4d::Identity();
      seed_pose.block<3, 3>(0, 0) = seed_rotation;
      seed_pose.block<3, 1>(0, 3) = seed_translation;
      if (!seed_pose.allFinite() || !validRigidPose(seed_pose))
        throw std::runtime_error("NDT startup seed is not a finite proper SE(3) pose");
      R_ = seed_rotation;
      p_ = seed_translation;
      ROS_WARN("[DogPriorMap NDT] using explicit startup pose seed (diagnostic/Oracle if configured): p=(%.6f %.6f %.6f)",
               p_.x(), p_.y(), p_.z());
    }
    lidar_topic_ = getParam<std::string>("topics/lidar", "/livox/lidar");
    lidar_msg_type_ = getParam<std::string>("topics/lidar_msg_type", "livox");
    imu_topic_ = getParam<std::string>("topics/imu", "/livox/imu");
    filtered_points_topic_ = getParam<std::string>("topics/filtered_points", "/dog_livo/filtered_points");
    diagnostics_topic_ = getParam<std::string>("topics/diagnostics", "/dog_livo/diagnostics");
    imu_watermark_topic_ = getParam<std::string>("output/imu_watermark_topic", "");
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
    local_map_manifest_path_ = getParam<std::string>("map/tile_manifest_path", "");
    local_map_safety_margin_m_ = getParam<double>("map/local_safety_margin_m", 10.0);
    local_map_hysteresis_margin_m_ = getParam<double>("map/local_hysteresis_margin_m", 30.0);
    local_map_enable_ = !local_map_manifest_path_.empty();
    if (local_map_enable_ &&
        (!std::isfinite(local_map_safety_margin_m_) || local_map_safety_margin_m_ < 0.0 ||
         !std::isfinite(local_map_hysteresis_margin_m_) ||
         local_map_hysteresis_margin_m_ < local_map_safety_margin_m_))
      throw std::runtime_error("invalid local map safety/hysteresis margins");
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
    coupled_mode_ = getParam<std::string>("coupled_mode", "CONTROL");
    if (coupled_mode_ != "CONTROL" && coupled_mode_ != "COUPLED_SHADOW" &&
        coupled_mode_ != "WEAK_ONLY_FEEDBACK" && coupled_mode_ != "COUPLED_FEEDBACK")
      throw std::runtime_error("coupled_mode must be CONTROL, COUPLED_SHADOW, WEAK_ONLY_FEEDBACK, or COUPLED_FEEDBACK");
    coupled_enabled_ = coupled_mode_ != "CONTROL";
    coupled_causal_record_only_enable_ = getParam<bool>(
        "coupled_causal_record_only_enable", false);
    coupled_causal_record_directory_ = getParam<std::string>(
        "coupled_causal_record_directory", "");
    if (coupled_causal_record_only_enable_ && coupled_mode_ != "CONTROL")
      throw std::runtime_error("causal Record-Only requires coupled_mode=CONTROL");
    if (coupled_causal_record_only_enable_ && coupled_causal_record_directory_.empty())
      throw std::runtime_error("causal Record-Only requires an explicit fresh output directory");
    visual_initial_guess_enable_ = getParam<bool>("visual_initial_guess_enable", false);
    if (coupled_causal_record_only_enable_ && visual_initial_guess_enable_)
      throw std::runtime_error("causal Record-Only forbids visual initial-guess branches");
    experimental_ekf_translation_fallback_enable_ = getParam<bool>(
        "lidar_update/experimental_ekf_translation_fallback_enable", false);
    if (experimental_ekf_translation_fallback_enable_ &&
        (!visual_initial_guess_enable_ || coupled_enabled_ || !local_imu_rotation_prior_enable_ ||
         scan_reference_time_ != "start" ||
         getParam<std::string>("deskew/mode", "") != "ekf_imu_fastlio" ||
         !getParam<bool>("imu/publish_high_rate", true) ||
         !getParam<bool>("ndt_observation/oosm_fixed_lag_enable", false) ||
         getParam<double>("ndt_observation/oosm_fixed_lag_sec", 0.0) != 0.15 ||
         getParam<bool>("ndt_observation/future_deferral_enable", false)))
      throw std::runtime_error("P21 requires frozen CONTROL/visual/IMU-rotation/LiDAR-body-twist/fixed-lag contract");
    if (visual_initial_guess_enable_)
      visual_initial_guess_.load(getParam<std::string>("visual_initial_guess_csv", ""));
    const std::string geometry_path = getParam<std::string>("visual_geometry_csv", "");
    if (!geometry_path.empty()) {
      if (coupled_causal_record_only_enable_)
        throw std::runtime_error("causal Record-Only forbids visual geometry jet diagnostics");
      visual_geometry_csv_.open(geometry_path);
      if (!visual_geometry_csv_) throw std::runtime_error("visual geometry log unavailable");
      visual_geometry_csv_ << "scan_ns,jet_valid,jet_ms";
      for (int i=0; i<6; ++i) visual_geometry_csv_ << ",lambda" << i;
      for (int i=0; i<6; ++i) for (int j=0; j<6; ++j)
        visual_geometry_csv_ << ",basis" << i << j;
      visual_geometry_csv_ << "\n";
    }
    ekf_high_rate_topic_ = getParam<std::string>("topics/odom_high_rate", "/dog_livo/odom_high_rate");
    // Frozen to the existing 20 ms OOSM timestamp-alignment contract.
    coupled_prediction_max_age_sec_ = 0.02;
    coupled_ekf_history_keep_sec_ = 60.0;
    local_imu_gyro_to_lidar_enable_ = getParam<bool>(
        "lidar_update/local_imu_gyro_to_lidar_enable", false);
    if (local_imu_gyro_to_lidar_enable_)
    {
      const std::vector<double> rotation = getParam<std::vector<double>>(
          "lidar_update/T_imu_lidar_rotation_row_major", std::vector<double>());
      if (!local_imu_rotation_prior_enable_ || rotation.size() != 9)
        throw std::runtime_error("local IMU gyro frame conversion requires a 9-value T_imu_lidar and enabled rotation prior");
      Eigen::Matrix3d R_imu_lidar;
      for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
          R_imu_lidar(row, column) = rotation[static_cast<std::size_t>(row * 3 + column)];
      if (!R_imu_lidar.allFinite() || R_imu_lidar.determinant() <= 0.0 ||
          (R_imu_lidar.transpose() * R_imu_lidar - Eigen::Matrix3d::Identity()).norm() > 1e-5 ||
          std::abs(R_imu_lidar.determinant() - 1.0) > 1e-5)
        throw std::runtime_error("local IMU gyro T_imu_lidar rotation is not proper SO(3)");
      T_imu_lidar_rotation_ = Eigen::Isometry3d::Identity();
      T_imu_lidar_rotation_.linear() = R_imu_lidar;
    }
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
    if (experimental_ekf_translation_fallback_enable_)
    {
      if (determinism_csv_path_.empty()) throw std::runtime_error("P21 requires per-scan diagnostics");
      ekf_translation_samples_csv_.open(determinism_csv_path_ + ".ekf_samples.csv");
      if (!ekf_translation_samples_csv_) throw std::runtime_error("P21 sample receipt log unavailable");
      ekf_translation_samples_csv_ << "sample_ns,header_seq,publisher,callback_wall_sec,receipt_ros_sec,callback_ros_sec,"
          "frame_id,child_frame_id,status,validation_ms";
      for (const char* key : {"tx","ty","tz","qx","qy","qz","qw","vx","vy","vz","wx","wy","wz"})
        ekf_translation_samples_csv_ << ',' << key;
      ekf_translation_samples_csv_ << '\n';
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
        if (experimental_ekf_translation_fallback_enable_)
          throw std::runtime_error("P21 per-scan diagnostics unavailable");
        ROS_WARN("[DogPriorMap NDT] failed to write determinism CSV: %s", determinism_csv_path_.c_str());
      }
    }
    else if (determinism_diagnostic_enable_)
    {
      ROS_WARN("[DogPriorMap NDT] determinism diagnostics enabled without an output CSV path");
      determinism_diagnostic_enable_ = false;
    }
    if (coupled_causal_record_only_enable_)
    {
      if (!local_map_enable_)
        throw std::runtime_error("causal Record-Only requires the frozen Local Target manager");
      if (experimental_ekf_translation_fallback_enable_)
        throw std::runtime_error("causal Record-Only forbids the experimental EKF fallback branch");
      if (!determinism_diagnostic_enable_ || !determinism_csv_.is_open())
        throw std::runtime_error("causal Record-Only requires live per-scan CONTROL diagnostics");
      openFreshCausalRecordCsv(causal_prediction_csv_, "prediction_records.csv",
          "frame_index,scan_stamp_ns,scan_reference_stamp_ns,ndt_align_complete_wall_sec,prediction_query_wall_sec,ndt_odom_publish_wall_sec,query_before_same_frame_ndt_publish,ekf_prediction_status,prediction_valid,causal_delivery_valid,ekf_sample_stamp_ns,ekf_message_seq,ekf_publisher,ekf_frame_id,ekf_child_frame_id,ekf_receipt_ros_sec,ekf_callback_receive_wall_sec,ekf_callback_accept_wall_sec,subscriber_history_revision,ekf_sample_state_revision,ekf_pose_tx,ekf_pose_ty,ekf_pose_tz,ekf_pose_qx,ekf_pose_qy,ekf_pose_qz,ekf_pose_qw,ekf_linear_velocity_x,ekf_linear_velocity_y,ekf_linear_velocity_z,ekf_angular_velocity_x,ekf_angular_velocity_y,ekf_angular_velocity_z,twist_frame_definition,prediction_age_ms,prediction_target_stamp_ns,predicted_map_T_lidar_row_major,source_hash64,target_tile_ids,target_cloud_hash64,target_points,ndt_converged");
      openFreshCausalRecordCsv(causal_anchor_csv_, "anchor_records.csv",
          "frame_index,scan_stamp_ns,anchor_before_valid,anchor_before_frozen,anchor_before_status,anchor_origin_stamp_ns,anchor_propagated_stamp_ns,anchor_age_sec,anchor_prediction_row_major,propagation_interval_start_ns,propagation_interval_end_ns,propagation_interval_sec,propagation_attempted,propagation_interval_valid,propagation_reason,anchor_after_propagation_valid,anchor_after_propagation_frozen,anchor_after_propagation_status,anchor_after_propagation_origin_stamp_ns,anchor_after_propagation_stamp_ns,anchor_after_propagation_prediction_row_major,anchor_after_settle_valid,anchor_after_settle_frozen,anchor_after_settle_status,anchor_after_settle_origin_stamp_ns,anchor_after_settle_stamp_ns,anchor_after_settle_prediction_row_major,anchor_invalid_reason");
      openFreshCausalRecordCsv(causal_trigger_csv_, "trigger_eligibility.csv",
          "frame_index,scan_stamp_ns,ndt_status,ndt_effective,innovation_translation_m,innovation_rotation_deg,trigger_translation_threshold_m,trigger_rotation_threshold_deg,triggered,prediction_valid,causal_delivery_valid,anchor_valid_before_refinement,anchor_frozen,anchor_origin_stamp_ns,anchor_propagated_stamp_ns,anchor_age_sec,first_frame,pair_eligible,eligibility_reason,nominal_ndt_pose_row_major,causal_prediction_pose_row_major");
      openFreshCausalRecordCsv(causal_record_overhead_csv_, "record_overhead.csv",
          "frame_index,scan_stamp_ns,record_write_wall_ms,record_write_cpu_ms");
    }
    experimental_source_capture_enable_ = getParam<bool>(
        "output/experimental_ndt_source_capture_enable", false);
    if (experimental_source_capture_enable_) {
      if (!determinism_diagnostic_enable_ || !determinism_csv_.is_open())
        throw std::runtime_error("Source capture requires live per-scan diagnostics");
      experimental_source_capture_.configure(
          getParam<std::string>("output/experimental_ndt_source_capture_directory", ""),
          getParam<std::vector<std::string>>("output/experimental_ndt_source_capture_requests", {}));
    }
    mature_integration_enable_ = getParam<bool>("mature_integration/enable", false);
    if (coupled_causal_record_only_enable_ && mature_integration_enable_)
      throw std::runtime_error("causal Record-Only forbids mature extra-align branches");
    mature_dcreg_enable_ = mature_integration_enable_ &&
        getParam<bool>("mature_integration/dcreg_enable", false);
    mature_multi_ndt_enable_ = mature_integration_enable_ &&
        getParam<bool>("mature_integration/multi_ndt_enable", false);
    mature_skf_enable_ = mature_integration_enable_ &&
        getParam<bool>("mature_integration/skf_enable", false);
    p25_relative_visual_contract_ = mature_integration_enable_ &&
        getParam<bool>("mature_integration/p25_relative_visual_contract", false);
    mature_ndt_config_.degeneracy_eigen_ratio = getParam<double>(
        "mature_integration/degeneracy_eigen_ratio", 0.03);
    mature_ndt_config_.weak_translation_step_m = getParam<double>(
        "mature_integration/weak_translation_step_m", 0.20);
    mature_ndt_config_.weak_rotation_step_deg = getParam<double>(
        "mature_integration/weak_rotation_step_deg", 2.0);
    mature_ndt_config_.score_improvement_margin = getParam<double>(
        "mature_integration/score_improvement_margin", 2.747604276e-4);
    mature_ndt_config_.motion_weight = getParam<double>(
        "mature_integration/motion_weight", 0.05);
    mature_ndt_config_.motion_translation_scale_m = getParam<double>(
        "mature_integration/motion_translation_scale_m", 2.0);
    mature_ndt_config_.motion_rotation_scale_deg = getParam<double>(
        "mature_integration/motion_rotation_scale_deg", 15.0);
    mature_ndt_config_.suspect_fitness_threshold = getParam<double>(
        "mature_integration/suspect_fitness_threshold", 10.0);
    mature_ndt_config_.maximum_quick_candidates = getParam<int>(
        "mature_integration/maximum_quick_candidates", 4);
    mature_ndt_config_.maximum_extra_aligns = getParam<int>(
        "mature_integration/maximum_extra_aligns", 2);
    if (mature_integration_enable_)
    {
      if (coupled_enabled_ || experimental_ekf_translation_fallback_enable_ ||
          (!visual_initial_guess_enable_ && !p25_relative_visual_contract_ &&
           !getParam<bool>("mature_visual/time_adapter_enable", false)) ||
          (p25_relative_visual_contract_ && (visual_initial_guess_enable_ ||
           scan_reference_time_ != "start" ||
           !getParam<bool>("ndt_observation/oosm_fixed_lag_enable", false) ||
           std::abs(getParam<double>("ndt_observation/oosm_fixed_lag_sec", 0.0) - 0.15) > 1e-9)) ||
          mature_ndt_config_.degeneracy_eigen_ratio <= 0.0 ||
          mature_ndt_config_.degeneracy_eigen_ratio >= 1.0 ||
          !std::isfinite(mature_ndt_config_.score_improvement_margin) ||
          mature_ndt_config_.score_improvement_margin < 0.0 ||
          !std::isfinite(mature_ndt_config_.motion_weight) || mature_ndt_config_.motion_weight < 0.0 ||
          !std::isfinite(mature_ndt_config_.motion_translation_scale_m) ||
          mature_ndt_config_.motion_translation_scale_m <= 0.0 ||
          !std::isfinite(mature_ndt_config_.motion_rotation_scale_deg) ||
          mature_ndt_config_.motion_rotation_scale_deg <= 0.0 ||
          !std::isfinite(mature_ndt_config_.suspect_fitness_threshold) ||
          mature_ndt_config_.suspect_fitness_threshold <= 0.0 ||
          mature_ndt_config_.weak_translation_step_m <= 0.0 ||
          mature_ndt_config_.weak_translation_step_m > 0.20 ||
          mature_ndt_config_.weak_rotation_step_deg <= 0.0 ||
          mature_ndt_config_.weak_rotation_step_deg > 2.0 ||
          mature_ndt_config_.maximum_quick_candidates < 0 ||
          mature_ndt_config_.maximum_quick_candidates > 4 ||
          mature_ndt_config_.maximum_extra_aligns < 0 ||
          mature_ndt_config_.maximum_extra_aligns > 2)
        throw std::runtime_error("P24 requires CONTROL+Visual, no P21 fallback, and bounded mature-module parameters");
      const std::string mature_csv_path = getParam<std::string>(
          "output/mature_integration_diagnostics_csv_path", "");
      if (mature_csv_path.empty()) throw std::runtime_error("P24 requires mature integration diagnostics path");
      std::ifstream existing_mature_csv(mature_csv_path.c_str());
      if (existing_mature_csv.good()) throw std::runtime_error("P24 diagnostics output already exists");
      mature_diagnostics_csv_.open(mature_csv_path, std::ios::out);
      if (!mature_diagnostics_csv_) throw std::runtime_error("P24 diagnostics output unavailable");
      mature_diagnostics_csv_ << "frame_index,scan_ns,dcreg_status,translation_degenerate,rotation_degenerate,"
          "translation_nonconvex,rotation_nonconvex,"
          "translation_ratio,rotation_ratio,translation_eigenvalues,rotation_eigenvalues,translation_weak_axis,"
          "rotation_weak_axis,recovery_suspicious,recovery_trigger_status,skf_status,skf_direction_selected,"
          "quick_candidate_count,quick_score_calls,jet_calls,preview_ms,jet_ms,extra_aligns,extra_align_ms,"
          "terminal_score_calls,candidate_details,selected_candidate,full_align_count,frame_module_ms\n";
    }
    if (getParam<bool>("mature_visual/time_adapter_enable", false))
      pub_visual_degeneracy_ = nh_.advertise<dog_prior_map_interfaces::VisualDegeneracy>(
          "/dog_livo/visual_degeneracy", 32);
    if (p25_relative_visual_contract_ && mature_skf_enable_)
    {
      const std::string visual_path = getParam<std::string>(
          "mature_integration/skf_visual_measurement_csv", "");
      skf_visual_measurements_.load(visual_path);
      skf_visual_topic_ = getParam<std::string>("topics/skf_visual_measurement",
          "/dog_livo/skf_visual_measurement");
      const std::string measurement_path = getParam<std::string>(
          "output/skf_measurement_csv_path", "");
      if (measurement_path.empty()) throw std::runtime_error("P25 requires SKF measurement output path");
      std::ifstream existing_skf_csv(measurement_path.c_str());
      if (existing_skf_csv.good()) throw std::runtime_error("P25 SKF diagnostic output already exists");
      skf_update_csv_.open(measurement_path, std::ios::out);
      if (!skf_update_csv_) throw std::runtime_error("P25 SKF diagnostics output unavailable");
      skf_update_csv_ << "scan_ns,reference_ns,current_ns,pnp_inliers,inlier_ratio,reprojection_rmse_px,"
          "covariance_model,translation_degenerate,rotation_degenerate,published\n";
      pub_skf_visual_ = nh_.advertise<dog_prior_map_interfaces::VisualRelativeMeasurement>(
          skf_visual_topic_, 32);
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
    geometry_diagnostics_csv_path_ = getParam<std::string>(
        "output/geometry_diagnostics_csv_path", "");
    geometry_gate_enable_ = getParam<bool>("p41/geometry_gate_enable", false);
    geometry_gate_max_median_m_ = getParam<double>("p41/geometry_gate_max_median_m", 1.0);
    geometry_gate_max_p90_m_ = getParam<double>("p41/geometry_gate_max_p90_m", 3.0);
    geometry_gate_min_overlap_05_ = getParam<double>("p41/geometry_gate_min_overlap_05", 0.50);
    if (geometry_gate_enable_ && geometry_diagnostics_csv_path_.empty())
      throw std::runtime_error("P41 geometry gate requires output/geometry_diagnostics_csv_path");
    if (local_map_enable_ && (!geometry_gate_enable_ || geometry_diagnostics_csv_path_.empty()))
      throw std::runtime_error("P45 local map requires the P41 geometry coverage gate and its diagnostic tree");
    if (!geometry_diagnostics_csv_path_.empty())
    {
      std::ifstream existing_geometry_csv(geometry_diagnostics_csv_path_.c_str());
      if (existing_geometry_csv.good())
        throw std::runtime_error("geometry diagnostic output already exists");
      geometry_diagnostics_csv_.open(geometry_diagnostics_csv_path_, std::ios::out);
      if (!geometry_diagnostics_csv_)
        throw std::runtime_error("geometry diagnostic output unavailable");
      geometry_diagnostics_csv_ << "scan_ns,source_points,target_points,gate_status,"
          "initial_nn_median_m,initial_nn_p90_m,initial_overlap_0p5m,initial_overlap_2m,initial_overlap_8m,"
          "ndt_converged,fitness,iterations,align_ms,raw_nn_median_m,raw_nn_p90_m,"
          "raw_overlap_0p5m,raw_overlap_2m,raw_overlap_8m,used_nn_median_m,used_nn_p90_m,"
          "used_overlap_0p5m,used_overlap_2m,used_overlap_8m,step_limited\n";
      geometry_diagnostics_csv_.flush();
    }
    local_map_diagnostics_csv_path_ = getParam<std::string>(
        "output/local_map_diagnostics_csv_path", "");
    if (local_map_enable_ && local_map_diagnostics_csv_path_.empty())
      throw std::runtime_error("P45 local map requires output/local_map_diagnostics_csv_path");
    if (!local_map_diagnostics_csv_path_.empty())
    {
      std::ifstream existing_local_csv(local_map_diagnostics_csv_path_.c_str());
      if (existing_local_csv.good()) throw std::runtime_error("local map diagnostics output already exists");
      local_map_diagnostics_csv_.open(local_map_diagnostics_csv_path_.c_str(), std::ios::out);
      if (!local_map_diagnostics_csv_)
        throw std::runtime_error("local map diagnostics output unavailable");
      local_map_diagnostics_csv_
          << "scan_ns,local_mode,map_updated,update_reason,update_error,coverage_status,loaded_tile_count,loaded_tile_ids,"
             "source_points,target_points,projected_x_min,projected_x_max,projected_y_min,projected_y_max,"
             "projected_z_min,projected_z_max,tile_io_ms,target_build_ms,target_rebuild_ms,"
             "initial_nn_median_m,initial_nn_p90_m,initial_overlap_0p5m,"
             "estimated_target_payload_mib,vmrss_kib,vmhwm_kib\n";
      local_map_diagnostics_csv_.flush();
    }

    loadMap();

    pub_odom_ = nh_.advertise<nav_msgs::Odometry>(ndt_odom_topic_, 20);
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_, 20);
    pub_path_ = nh_.advertise<nav_msgs::Path>(ndt_path_topic_, 5);
    pub_aligned_ = nh_.advertise<sensor_msgs::PointCloud2>(points_aligned_topic_, 5);
    publish_local_target_ = getParam<bool>("output/publish_local_target", false);
    if (publish_local_target_ && local_map_enable_)
      pub_local_target_ = nh_.advertise<sensor_msgs::PointCloud2>(
          getParam<std::string>("topics/local_target", "/newer_college/local_ndt_target"), 1, true);
    if (publish_filtered_points_)
    {
      pub_filtered_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_points_topic_, 5);
    }
    if (publish_diagnostics_)
    {
      pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    }
    if (!imu_watermark_topic_.empty())
      pub_imu_watermark_ = nh_.advertise<std_msgs::Header>(imu_watermark_topic_, 1000, false);
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
    if (coupled_enabled_ || experimental_ekf_translation_fallback_enable_ ||
        coupled_causal_record_only_enable_)
    {
      sub_ekf_high_rate_ = nh_.subscribe(ekf_high_rate_topic_, 10000,
          &DogPriorMapNdtNode::ekfHighRateCallback, this);
    }

    ROS_INFO("[DogPriorMap NDT] started: mode=%s map=%s target=%zu lidar=%s output=%s EKF=%s",
             coupled_mode_.c_str(), local_map_enable_ ? local_map_manifest_path_.c_str() : map_pcd_path_.c_str(),
             target_cloud_->size(), lidar_topic_.c_str(),
             ndt_odom_topic_.c_str(), (coupled_enabled_ || experimental_ekf_translation_fallback_enable_ ||
                                      coupled_causal_record_only_enable_) ?
             ekf_high_rate_topic_.c_str() : "disabled");
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
    Eigen::Vector3d gyro(msg->angular_velocity.x,
                         msg->angular_velocity.y,
                         msg->angular_velocity.z);
    if (local_imu_gyro_to_lidar_enable_)
      gyro = imuGyroToLidarFrame(gyro, T_imu_lidar_rotation_);
    imu_history_.push_back(ImuSample{msg->header.stamp.toSec(),
                                     Eigen::Vector3d(msg->linear_acceleration.x,
                                                     msg->linear_acceleration.y,
                                                     msg->linear_acceleration.z),
                                     gyro});
    while (!imu_history_.empty() &&
           msg->header.stamp.toSec() - imu_history_.front().stamp > imu_history_keep_sec_)
    {
      imu_history_.pop_front();
    }
    if (local_imu_rotation_prior_enable_ || coupled_enabled_)
      processPendingCloudsLocked();
    if (pub_imu_watermark_)
    {
      std_msgs::Header watermark;
      watermark.stamp = msg->header.stamp;
      watermark.frame_id = msg->header.frame_id;
      pub_imu_watermark_.publish(watermark);
    }
  }

  void ekfHighRateCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
  {
    const ros::WallTime callback_receive_wall = ros::WallTime::now();
    const auto msg = event.getMessage();
    if (!msg || (!coupled_enabled_ && !experimental_ekf_translation_fallback_enable_ &&
                 !coupled_causal_record_only_enable_)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto callback_start = ros::WallTime::now();
    const uint64_t stamp_ns = msg->header.stamp.toNSec();
    if (stamp_ns == 0) return;
    if (experimental_ekf_translation_fallback_enable_)
    {
      const Eigen::Vector3d p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      const Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
          msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      const Eigen::Vector3d v(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
      const Eigen::Vector3d w(msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);
      Eigen::Matrix4d accepted_pose = Eigen::Matrix4d::Identity();
      const double quaternion_norm = q.norm();
      if (std::isfinite(quaternion_norm) && quaternion_norm > 1e-9)
        accepted_pose.block<3,3>(0,0) = q.normalized().toRotationMatrix();
      accepted_pose.block<3,1>(0,3) = p;
      std::string status = "ACCEPTED_CURRENT_EKF_SAMPLE";
      if (event.getPublisherName() != "/dog_prior_map_ekf") status = "UNVERIFIED_EKF_PROVENANCE";
      else if (msg->header.frame_id != map_frame_ || msg->child_frame_id != base_frame_) status = "INVALID_FRAME";
      else if (!p.allFinite() || !q.coeffs().allFinite() || !std::isfinite(quaternion_norm) ||
               quaternion_norm <= 1e-9 || !validRigidPose(accepted_pose) ||
               !v.allFinite() || !w.allFinite()) status = "NONFINITE_PREDICTION";
      else if (!orderedEkfSample(latest_ekf_high_rate_stamp_ns_, stamp_ns)) status = "AMBIGUOUS_EKF_STREAM_ORDER";
      ekf_translation_samples_csv_ << std::setprecision(17) << stamp_ns << ',' << msg->header.seq << ','
          << event.getPublisherName() << ',' << callback_start.toSec() << ',' << event.getReceiptTime().toSec()
          << ',' << ros::Time::now().toSec()
          << ',' << msg->header.frame_id << ',' << msg->child_frame_id << ',' << status << ','
          << (ros::WallTime::now()-callback_start).toSec()*1000.0 << ','
          << p.x() << ',' << p.y() << ',' << p.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w()
          << ',' << v.x() << ',' << v.y() << ',' << v.z() << ',' << w.x() << ',' << w.y() << ',' << w.z() << '\n';
      if (status != "ACCEPTED_CURRENT_EKF_SAMPLE")
        throw std::runtime_error("P21 causal EKF stream rejected: " + status);
    }
    if (!has_ekf_high_rate_watermark_ || stamp_ns > latest_ekf_high_rate_stamp_ns_)
    {
      latest_ekf_high_rate_stamp_ns_ = stamp_ns;
      has_ekf_high_rate_watermark_ = true;
    }
    const Eigen::Vector3d position(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
    const Eigen::Quaterniond orientation(msg->pose.pose.orientation.w,
                                         msg->pose.pose.orientation.x,
                                         msg->pose.pose.orientation.y,
                                         msg->pose.pose.orientation.z);
    const Eigen::Vector3d linear_velocity_lidar(msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const Eigen::Vector3d angular_velocity_lidar(msg->twist.twist.angular.x,
        msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    if (msg->header.frame_id == map_frame_ && msg->child_frame_id == base_frame_ &&
        position.allFinite() && orientation.coeffs().allFinite() && orientation.norm() > 1e-9 &&
        linear_velocity_lidar.allFinite() && angular_velocity_lidar.allFinite())
    {
      Eigen::Matrix4d map_T_lidar = Eigen::Matrix4d::Identity();
      map_T_lidar.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
      map_T_lidar.block<3, 1>(0, 3) = position;
      if (validRigidPose(map_T_lidar))
      {
        EkfPoseSample sample;
        sample.stamp_ns = stamp_ns;
        sample.map_T_lidar = map_T_lidar;
        sample.linear_velocity_lidar = linear_velocity_lidar;
        sample.angular_velocity_lidar = angular_velocity_lidar;
        sample.message_seq = msg->header.seq;
        sample.publisher = event.getPublisherName();
        sample.frame_id = msg->header.frame_id;
        sample.child_frame_id = msg->child_frame_id;
        sample.receipt_ros_sec = event.getReceiptTime().toSec();
        sample.callback_receive_wall_sec = callback_receive_wall.toSec();
        sample.callback_accept_wall_sec = ros::WallTime::now().toSec();
        sample.subscriber_history_revision = ++ekf_pose_history_revision_;
        if (ekf_pose_history_.empty() || stamp_ns > ekf_pose_history_.back().stamp_ns)
          ekf_pose_history_.push_back(sample);
        else if (stamp_ns == ekf_pose_history_.back().stamp_ns)
          ekf_pose_history_.back() = sample;
        else
          --ekf_pose_history_revision_;
      }
    }
    while (!ekf_pose_history_.empty() &&
           latest_ekf_high_rate_stamp_ns_ > ekf_pose_history_.front().stamp_ns &&
           (latest_ekf_high_rate_stamp_ns_ - ekf_pose_history_.front().stamp_ns) / 1e9 >
               coupled_ekf_history_keep_sec_)
      ekf_pose_history_.pop_front();
    if (local_imu_rotation_prior_enable_ || coupled_enabled_ ||
        experimental_ekf_translation_fallback_enable_)
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

  struct EkfPoseSample
  {
    uint64_t stamp_ns = 0;
    Eigen::Matrix4d map_T_lidar = Eigen::Matrix4d::Identity();
    Eigen::Vector3d linear_velocity_lidar = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_lidar = Eigen::Vector3d::Zero();
    uint32_t message_seq = 0;
    uint64_t subscriber_history_revision = 0;
    double receipt_ros_sec = std::numeric_limits<double>::quiet_NaN();
    double callback_receive_wall_sec = std::numeric_limits<double>::quiet_NaN();
    double callback_accept_wall_sec = std::numeric_limits<double>::quiet_NaN();
    std::string publisher;
    std::string frame_id;
    std::string child_frame_id;
  };

  struct EkfPrediction
  {
    bool valid = false;
    uint64_t sample_stamp_ns = 0;
    uint64_t stamp_ns = 0;
    double age_ms = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix4d map_T_lidar = Eigen::Matrix4d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    EkfPoseSample sample;
    bool sample_available = false;
    std::string status = "NOT_REQUESTED";
  };

  struct DeterminismRow
  {
    uint64_t frame_index = 0;
    double lidar_header_stamp = std::numeric_limits<double>::quiet_NaN();
    double ros_now = std::numeric_limits<double>::quiet_NaN();
    double wall_time = std::numeric_limits<double>::quiet_NaN();
    double ndt_align_start_wall_sec = std::numeric_limits<double>::quiet_NaN();
    double ndt_align_complete_wall_sec = std::numeric_limits<double>::quiet_NaN();
    double ndt_odom_publish_wall_sec = std::numeric_limits<double>::quiet_NaN();
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
    std::string coupled_mode = "CONTROL";
    std::string ekf_prediction_status = "NOT_REQUESTED";
    double ekf_prediction_stamp = std::numeric_limits<double>::quiet_NaN();
    double ekf_prediction_target_stamp = std::numeric_limits<double>::quiet_NaN();
    double ekf_prediction_age_ms = std::numeric_limits<double>::quiet_NaN();
    Eigen::Matrix4d ekf_prediction_pose = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    std::string anchor_status = "NOT_REQUESTED";
    bool anchor_valid = false;
    bool anchor_valid_after = false;
    double anchor_age_s = std::numeric_limits<double>::quiet_NaN();
    double anchor_stamp = std::numeric_limits<double>::quiet_NaN();
    std::string anchor_status_after = "NOT_REQUESTED";
    std::string coupled_status = "NOT_RUN";
    bool coupled_triggered = false;
    bool coupled_attempted = false;
    bool coupled_recommended = false;
    bool coupled_feedback_applied = false;
    bool coupled_feedback_changed_observation = false;
    bool coupled_candidate_step_limited = false;
    bool coupled_candidate_translation_limited = false;
    bool coupled_candidate_rotation_limited = false;
    bool coupled_strong_selected = false;
    int coupled_weak_dimension = 0;
    int coupled_jet_calls = 0;
    int coupled_value_calls = 0;
    Eigen::Matrix4d coupled_candidate = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d coupled_weak_pose = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix4d coupled_full_pose = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> coupled_weak_eta = Eigen::Matrix<double, 6, 1>::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Matrix<double, 6, 1> coupled_strong_eta = Eigen::Matrix<double, 6, 1>::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double coupled_score_sum = std::numeric_limits<double>::quiet_NaN();
    double coupled_total_ms = 0.0;
    std::string ekf_translation_reason = "DISABLED";
    bool visual_applied = false;
    bool ekf_translation_applied = false;
    uint64_t ekf_sample_ns = 0;
    uint64_t ekf_target_ns = 0;
    uint64_t ekf_watermark_ns = 0;
    double ekf_prediction_query_ms = 0.0;
    double ekf_application_wall_sec = 0.0;
    double ekf_translation_change_m = 0.0;
  };

  struct AnchorPropagationRecord
  {
    uint64_t interval_start_ns = 0;
    uint64_t interval_end_ns = 0;
    double interval_sec = std::numeric_limits<double>::quiet_NaN();
    bool attempted = false;
    bool interval_valid = false;
    std::string reason = "NOT_ATTEMPTED";
    Eigen::Matrix4d relative_prediction = Eigen::Matrix4d::Identity();
  };

  struct CausalRecordFrame
  {
    uint64_t frame_index = 0;
    uint64_t scan_stamp_ns = 0;
    uint64_t scan_reference_stamp_ns = 0;
    uint64_t source_hash64 = 0;
    uint64_t target_cloud_hash64 = 0;
    std::size_t source_points = 0;
    std::size_t target_points = 0;
    std::vector<std::string> target_tile_ids;
    bool ndt_effective = false;
    bool ndt_converged = false;
    std::string ndt_status = "NOT_RUN";
    double prediction_query_wall_sec = std::numeric_limits<double>::quiet_NaN();
    bool query_before_same_frame_ndt_publish = false;
    bool causal_delivery_valid = false;
    bool anchor_valid_for_pair = false;
    bool pair_eligible = false;
    std::string eligibility_reason = "PREDICTION_UNAVAILABLE";
    dog_prior_map_fastlio2_frontend_exp::WeakCoupledInnovation innovation;
    Eigen::Matrix4d nominal_ndt_pose = Eigen::Matrix4d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    EkfPrediction prediction;
    dog_prior_map_fastlio2_frontend_exp::CoupledAnchorState anchor_before;
    dog_prior_map_fastlio2_frontend_exp::CoupledAnchorState anchor_after_propagation;
    dog_prior_map_fastlio2_frontend_exp::CoupledAnchorState anchor_after_settle;
    AnchorPropagationRecord propagation;
  };

  struct MatureCandidateRecord
  {
    dog_prior_map_localization::MatureNdtCandidate candidate;
    bool attempted = false;
    bool converged = false;
    bool accepted = false;
    bool translation_limited = false;
    bool rotation_limited = false;
    double terminal_score = std::numeric_limits<double>::quiet_NaN();
    double fitness = std::numeric_limits<double>::quiet_NaN();
    double align_ms = 0.0;
    int iterations = 0;
    std::string status = "PREVIEW_ONLY";
  };

  struct MatureFrameRecord
  {
    uint64_t frame_index = 0;
    uint64_t scan_ns = 0;
    dog_prior_map_localization::DcregAdaptedReport dcreg;
    std::string recovery_status = "NOT_RUN";
    std::string skf_status = "DISABLED";
    bool recovery_suspicious = false;
    bool skf_direction_selected = false;
    int quick_score_calls = 0;
    int terminal_score_calls = 0;
    int jet_calls = 0;
    int extra_aligns = 0;
    int selected_candidate = -1;
    double jet_ms = 0.0;
    double preview_ms = 0.0;
    double extra_align_ms = 0.0;
    double module_ms = 0.0;
    std::vector<MatureCandidateRecord> candidates;
  };

  static bool validRigidPose(const Eigen::Matrix4d &pose)
  {
    return rigidMapPose(pose);
  }

  static std::string vectorText(const Eigen::Vector3d& value)
  {
    std::ostringstream out;
    out << std::setprecision(9) << value.x() << ':' << value.y() << ':' << value.z();
    return out.str();
  }

  static std::string candidateText(const MatureCandidateRecord& record)
  {
    std::ostringstream out;
    out << std::setprecision(9) << record.candidate.id << ':' << record.candidate.kind << ':'
        << record.candidate.score << ':' << record.candidate.merit << ':'
        << (record.attempted ? 1 : 0) << ':' << record.status << ':'
        << record.terminal_score << ':' << record.fitness << ':' << record.iterations << ':'
        << record.align_ms << ':' << (record.translation_limited ? 1 : 0) << ':'
        << (record.rotation_limited ? 1 : 0) << ':' << (record.accepted ? 1 : 0);
    return out.str();
  }

  void runMatureIntegration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const Eigen::Matrix4d& prediction, uint64_t frame_index, uint64_t scan_ns, uint64_t scan_end_ns,
      bool visual_applied, Eigen::Matrix4d* result,
      pcl::PointCloud<pcl::PointXYZ>* aligned, bool* converged, double* fitness,
      int* iterations, double* total_align_ms)
  {
    if (!mature_integration_enable_ || !source || !result || !aligned || !converged ||
        !fitness || !iterations || !total_align_ms) return;
    namespace frontend = dog_prior_map_fastlio2_frontend_exp;
    MatureFrameRecord record;
    record.frame_index = frame_index;
    record.scan_ns = scan_ns;
    const auto module_start = ros::WallTime::now();

    if (mature_dcreg_enable_ && result->allFinite() && validRigidPose(*result))
    {
      frontend::CoupledNdtBackend backend;
      backend.jet = [this, source](const Eigen::Matrix4f& pose,
          const frontend::CoupledVector6& parameters) {
        return ndt_.nativeJet(source, pose, parameters);
      };
      const auto jet_start = ros::WallTime::now();
      const frontend::CoupledLocalJet jet = frontend::coupledNominalJet(
          result->cast<float>(), static_cast<double>(source->size()), backend,
          mature_ndt_config_.translation_scale_m);
      record.jet_ms = (ros::WallTime::now() - jet_start).toSec() * 1000.0;
      record.jet_calls = 1;
      record.dcreg = analyzeDcregAdapted(jet.H, mature_ndt_config_.degeneracy_eigen_ratio);
      if (!jet.valid) record.dcreg.status = "NDT_JET_INVALID";
    }
    else if (!mature_dcreg_enable_)
    {
      record.dcreg.status = "DISABLED";
    }
    else
    {
      record.dcreg.status = "INVALID_NOMINAL_POSE";
    }

    if (pub_visual_degeneracy_)
    {
      dog_prior_map_interfaces::VisualDegeneracy message;
      message.header.stamp.fromNSec(scan_ns);
      message.header.frame_id = map_frame_;
      message.scan_end_ns = scan_end_ns;
      message.valid = record.dcreg.valid;
      message.translation_degenerate = record.dcreg.translation_degenerate;
      message.rotation_degenerate = record.dcreg.rotation_degenerate;
      for (int i=0; i<3; ++i) {
        message.translation_eigenvalues[i] = record.dcreg.translation_eigenvalues(i);
        message.rotation_eigenvalues[i] = record.dcreg.rotation_eigenvalues(i);
        for (int j=0; j<3; ++j) {
          message.translation_axes[3*i+j] = record.dcreg.translation_axes(i,j);
          message.rotation_axes[3*i+j] = record.dcreg.rotation_axes(i,j);
        }
      }
      pub_visual_degeneracy_.publish(message);
    }
    const SkfVisualMeasurement* relative_visual = p25_relative_visual_contract_ && mature_skf_enable_ ?
        skf_visual_measurements_.find(scan_ns) : nullptr;
    const bool valid_visual = p25_relative_visual_contract_ ? relative_visual != nullptr :
        (visual_initial_guess_enable_ && visual_initial_guess_.hasValidMeasurement(scan_ns));
    const auto skf = evaluateSkfGate(mature_skf_enable_, record.dcreg,
        valid_visual, visual_applied, relative_visual != nullptr);
    record.skf_status = skf.status;
    record.skf_direction_selected = skf.direction_selected;
    if (relative_visual && skf.status == "READY_RELATIVE_VISUAL_MEASUREMENT")
      publishSkfVisual(*relative_visual, record.dcreg);

    const bool quality_suspect = !*converged || !std::isfinite(*fitness) ||
        *fitness > mature_ndt_config_.suspect_fitness_threshold || *iterations <= 1;
    record.recovery_suspicious = quality_suspect;
    const bool weak = record.dcreg.valid &&
        (record.dcreg.translation_degenerate || record.dcreg.rotation_degenerate);
    if (!mature_multi_ndt_enable_)
    {
      record.recovery_status = "MULTI_NDT_DISABLED";
    }
    else if (!record.dcreg.valid)
    {
      record.recovery_status = "NO_VALID_DCREG_REPORT";
    }
    else if (!weak)
    {
      record.recovery_status = "NO_DEGENERATE_SUBSPACE";
    }
    else if (!quality_suspect)
    {
      record.recovery_status = "NOMINAL_QUALITY_NOT_SUSPECT";
    }
    else if (!prediction.allFinite() || !validRigidPose(prediction))
    {
      record.recovery_status = "INVALID_PREDICTION";
    }
    else
    {
      record.recovery_status = "TRIGGERED";
      std::vector<MatureNdtCandidate> candidates = makeWeakDirectionCandidates(
          record.dcreg, prediction.cast<float>(), mature_ndt_config_.weak_translation_step_m,
          mature_ndt_config_.weak_rotation_step_deg);
      if (static_cast<int>(candidates.size()) > mature_ndt_config_.maximum_quick_candidates)
        candidates.resize(static_cast<std::size_t>(mature_ndt_config_.maximum_quick_candidates));

      const double nominal_score = ndt_.dynamicScore(source, result->cast<float>());
      ++record.terminal_score_calls;
      const double nominal_energy_scale = std::max(1.0,
          std::abs(nominal_score / static_cast<double>(source->size())));
      const double nominal_merit = candidateMotionMerit(nominal_score, source->size(),
          result->cast<float>(), prediction.cast<float>(), nominal_energy_scale, mature_ndt_config_);
      const auto preview_start = ros::WallTime::now();
      for (auto& candidate : candidates)
      {
        MatureCandidateRecord item;
        item.candidate = candidate;
        item.candidate.score = ndt_.dynamicScore(source, candidate.pose);
        ++record.quick_score_calls;
        item.candidate.merit = candidateMotionMerit(item.candidate.score, source->size(),
            candidate.pose, prediction.cast<float>(), nominal_energy_scale, mature_ndt_config_);
        item.candidate.finite = item.candidate.finite && std::isfinite(item.candidate.score) &&
            std::isfinite(item.candidate.merit);
        record.candidates.push_back(item);
      }
      record.preview_ms = (ros::WallTime::now() - preview_start).toSec() * 1000.0;
      std::vector<MatureNdtCandidate> ranked;
      for (const auto& item : record.candidates)
        if (item.candidate.finite && item.candidate.score >=
            nominal_score + mature_ndt_config_.score_improvement_margin &&
            item.candidate.merit < nominal_merit)
          ranked.push_back(item.candidate);
      rankMatureCandidates(&ranked);

      Eigen::Matrix4d nominal_used = *result;
      bool nominal_translation_limited = false, nominal_rotation_limited = false;
      limitNdtStep(*result, nominal_used, nominal_translation_limited, nominal_rotation_limited);
      const double nominal_used_score = ndt_.dynamicScore(source, nominal_used.cast<float>());
      ++record.terminal_score_calls;
      const double nominal_used_merit = candidateMotionMerit(nominal_used_score, source->size(),
          nominal_used.cast<float>(), prediction.cast<float>(), nominal_energy_scale, mature_ndt_config_);
      double selected_merit = nominal_used_merit;
      for (int rank = 0; rank < static_cast<int>(ranked.size()) &&
           record.extra_aligns < mature_ndt_config_.maximum_extra_aligns; ++rank)
      {
        auto found = std::find_if(record.candidates.begin(), record.candidates.end(),
            [&](const MatureCandidateRecord& item) { return item.candidate.id == ranked[rank].id; });
        if (found == record.candidates.end()) continue;
        found->attempted = true;
        const auto align_start = ros::WallTime::now();
        ndt_.setInputSource(source);
        pcl::PointCloud<pcl::PointXYZ> candidate_aligned;
        ndt_.align(candidate_aligned, found->candidate.pose);
        found->align_ms = (ros::WallTime::now() - align_start).toSec() * 1000.0;
        record.extra_align_ms += found->align_ms;
        *total_align_ms += found->align_ms;
        ++record.extra_aligns;
        found->converged = ndt_.hasConverged();
        found->fitness = ndt_.getFitnessScore();
        found->iterations = ndt_.getFinalNumIteration();
        Eigen::Matrix4d candidate_raw = ndt_.getFinalTransformation().cast<double>();
        found->terminal_score = candidate_raw.allFinite() ?
            ndt_.dynamicScore(source, candidate_raw.cast<float>()) :
            std::numeric_limits<double>::quiet_NaN();
        ++record.terminal_score_calls;
        found->status = "REFINED_REJECTED";
        if (!found->converged || !std::isfinite(found->fitness) ||
            !candidate_raw.allFinite() || !validRigidPose(candidate_raw) ||
            !std::isfinite(found->terminal_score))
        {
          found->status = "REFINEMENT_INVALID";
          continue;
        }
        Eigen::Matrix4d candidate_used = candidate_raw;
        limitNdtStep(candidate_raw, candidate_used, found->translation_limited,
                     found->rotation_limited);
        const double used_score = ndt_.dynamicScore(source, candidate_used.cast<float>());
        ++record.terminal_score_calls;
        const double used_merit = candidateMotionMerit(used_score, source->size(),
            candidate_used.cast<float>(), prediction.cast<float>(), nominal_energy_scale, mature_ndt_config_);
        const bool objective_gain = found->terminal_score >=
            nominal_score + mature_ndt_config_.score_improvement_margin &&
            used_score >= nominal_used_score + mature_ndt_config_.score_improvement_margin;
        const bool motion_gain = std::isfinite(used_merit) && used_merit < selected_merit;
        if (objective_gain && motion_gain)
        {
          *result = candidate_raw;
          *aligned = candidate_aligned;
          *converged = true;
          *fitness = found->fitness;
          *iterations = found->iterations;
          selected_merit = used_merit;
          record.selected_candidate = found->candidate.id;
          found->accepted = true;
          found->status = "ACCEPTED_AFTER_FULL_NDT_AND_STEP_LIMIT";
        }
        else if (!objective_gain)
        {
          found->status = "TERMINAL_OBJECTIVE_GAIN_NOT_PRESERVED";
        }
        else
        {
          found->status = "MOTION_MERIT_NOT_IMPROVED";
        }
      }
      if (record.extra_aligns == 0) record.recovery_status = "TRIGGERED_NO_CANDIDATE_PASSED_PREVIEW";
      else if (record.selected_candidate >= 0) record.recovery_status = "ALTERNATIVE_SELECTED";
      else record.recovery_status = "REFINED_NOMINAL_RETAINED";
    }

    record.module_ms = (ros::WallTime::now() - module_start).toSec() * 1000.0;
    writeMatureDiagnostics(record);
  }

  void publishSkfVisual(const SkfVisualMeasurement& measurement,
                        const DcregAdaptedReport& dcreg)
  {
    if (!pub_skf_visual_ || !skf_update_csv_.is_open()) return;
    dog_prior_map_interfaces::VisualRelativeMeasurement message;
    message.header.stamp.fromNSec(measurement.current_ns);
    message.header.frame_id = "imu_cur";
    message.reference_stamp.fromNSec(measurement.reference_ns);
    message.depth_max_point_stamp.fromNSec(measurement.depth_max_point_ns);
    message.depth_max_imu_stamp.fromNSec(measurement.depth_max_imu_ns);
    message.scan_ns = measurement.scan_ns;
    message.reference_frame = "imu_ref";
    message.current_frame = "imu_cur";
    message.relative_pose.position.x = measurement.relative_pose(0, 3);
    message.relative_pose.position.y = measurement.relative_pose(1, 3);
    message.relative_pose.position.z = measurement.relative_pose(2, 3);
    Eigen::Quaterniond q(measurement.relative_pose.block<3, 3>(0, 0));
    q.normalize();
    message.relative_pose.orientation.x = q.x();
    message.relative_pose.orientation.y = q.y();
    message.relative_pose.orientation.z = q.z();
    message.relative_pose.orientation.w = q.w();
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        message.covariance_left_imu_ref[static_cast<std::size_t>(i * 6 + j)] = measurement.covariance(i, j);
    message.translation_degenerate = dcreg.translation_degenerate;
    message.rotation_degenerate = dcreg.rotation_degenerate;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
      {
        message.translation_axes_map[static_cast<std::size_t>(i * 3 + j)] = dcreg.translation_axes(i, j);
        message.rotation_axes_map[static_cast<std::size_t>(i * 3 + j)] = dcreg.rotation_axes(i, j);
      }
    message.pnp_inliers = measurement.pnp_inliers;
    message.inlier_ratio = measurement.inlier_ratio;
    message.reprojection_rmse_px = measurement.reprojection_rmse_px;
    message.grid_occupancy = measurement.grid_occupancy;
    message.hull_fraction = measurement.hull_fraction;
    message.covariance_model = measurement.covariance_model;
    pub_skf_visual_.publish(message);
    skf_update_csv_ << std::setprecision(17) << measurement.scan_ns << ','
        << measurement.reference_ns << ',' << measurement.current_ns << ','
        << measurement.pnp_inliers << ',' << measurement.inlier_ratio << ','
        << measurement.reprojection_rmse_px << ',' << measurement.covariance_model << ','
        << (dcreg.translation_degenerate ? 1 : 0) << ','
        << (dcreg.rotation_degenerate ? 1 : 0) << ",1\n";
    skf_update_csv_.flush();
  }

  void writeMatureDiagnostics(const MatureFrameRecord& record)
  {
    if (!mature_diagnostics_csv_.is_open()) return;
    std::ostringstream out;
    out << std::setprecision(17) << record.frame_index << ',' << record.scan_ns << ','
        << record.dcreg.status << ',' << (record.dcreg.translation_degenerate ? 1 : 0) << ','
        << (record.dcreg.rotation_degenerate ? 1 : 0) << ','
        << (record.dcreg.translation_nonconvex ? 1 : 0) << ','
        << (record.dcreg.rotation_nonconvex ? 1 : 0) << ',' << record.dcreg.translation_ratio << ','
        << record.dcreg.rotation_ratio << ',' << vectorText(record.dcreg.translation_eigenvalues) << ','
        << vectorText(record.dcreg.rotation_eigenvalues) << ','
        << vectorText(record.dcreg.translation_axes.col(0)) << ','
        << vectorText(record.dcreg.rotation_axes.col(0)) << ','
        << (record.recovery_suspicious ? 1 : 0) << ',' << record.recovery_status << ','
        << record.skf_status << ',' << (record.skf_direction_selected ? 1 : 0) << ','
        << record.candidates.size() << ',' << record.quick_score_calls << ',' << record.jet_calls << ','
        << record.preview_ms << ',' << record.jet_ms << ',' << record.extra_aligns << ','
        << record.extra_align_ms << ',' << record.terminal_score_calls << ',';
    for (std::size_t i = 0; i < record.candidates.size(); ++i)
    {
      if (i) out << ';';
      out << candidateText(record.candidates[i]);
    }
    out << ',' << record.selected_candidate << ',' << (1 + record.extra_aligns) << ','
        << record.module_ms << '\n';
    mature_diagnostics_csv_ << out.str();
    mature_diagnostics_csv_.flush();
  }

  EkfPrediction ekfPredictionAtOrBeforeLocked(uint64_t reference_stamp_ns) const
  {
    EkfPrediction prediction;
    prediction.status = "NO_CAUSAL_EKF_SAMPLE";
    if (!reference_stamp_ns || ekf_pose_history_.empty()) return prediction;
    const auto after = std::upper_bound(ekf_pose_history_.begin(), ekf_pose_history_.end(),
        reference_stamp_ns, [](uint64_t stamp, const EkfPoseSample &sample) {
          return stamp < sample.stamp_ns;
        });
    if (after == ekf_pose_history_.begin())
    {
      if (experimental_ekf_translation_fallback_enable_) prediction.status = "FUTURE_SAMPLE_REJECTED";
      return prediction;
    }
    const EkfPoseSample &sample = *std::prev(after);
    if (sample.stamp_ns > reference_stamp_ns || !validRigidPose(sample.map_T_lidar))
    {
      prediction.status = "INVALID_CAUSAL_EKF_SAMPLE";
      return prediction;
    }
    prediction.sample_stamp_ns = sample.stamp_ns;
    prediction.sample = sample;
    prediction.sample_available = true;
    prediction.stamp_ns = reference_stamp_ns;
    prediction.age_ms = static_cast<double>(reference_stamp_ns - sample.stamp_ns) / 1e6;
    if (!std::isfinite(prediction.age_ms) ||
        prediction.age_ms > 1000.0 * coupled_prediction_max_age_sec_)
    {
      prediction.status = "STALE_CAUSAL_EKF_SAMPLE";
      return prediction;
    }
    if (!predictMapTLidarWithBodyTwist(sample.map_T_lidar, sample.linear_velocity_lidar,
        sample.angular_velocity_lidar, prediction.age_ms / 1000.0,
        coupled_prediction_max_age_sec_, &prediction.map_T_lidar))
    {
      prediction.status = "CAUSAL_EKF_TWIST_EXTRAPOLATION_INVALID";
      return prediction;
    }
    prediction.valid = true;
    prediction.status = prediction.age_ms > 0.0 ?
        "CAUSAL_EKF_BODY_TWIST_TO_SCAN" : "CAUSAL_EKF_EXACT_SCAN_STAMP";
    return prediction;
  }

  void advanceCoupledAnchorFromPrediction(uint64_t reference_stamp_ns,
      const EkfPrediction &prediction, AnchorPropagationRecord *record)
  {
    if (record) *record = AnchorPropagationRecord{};
    if (prediction.valid)
    {
      if (has_previous_coupled_ekf_pose_)
      {
        const uint64_t previous_stamp_ns = previous_coupled_ekf_pose_.stamp_ns;
        const double interval_s = previous_stamp_ns < prediction.stamp_ns ?
            (prediction.stamp_ns - previous_stamp_ns) / 1e9 : 0.0;
        if (record)
        {
          record->interval_start_ns = previous_stamp_ns;
          record->interval_end_ns = prediction.stamp_ns;
          record->interval_sec = interval_s;
        }
        if (interval_s > 0.0 && interval_s <= 0.25 &&
            reference_stamp_ns > coupled_anchor_.propagated_stamp_ns)
        {
          const Eigen::Matrix4d causal_interval =
              previous_coupled_ekf_pose_.map_T_lidar.inverse() * prediction.map_T_lidar;
          if (record)
          {
            record->attempted = true;
            record->relative_prediction = causal_interval;
            record->interval_valid = validRigidPose(causal_interval);
            record->reason = record->interval_valid ? "ADVANCE_ATTEMPTED" : "INVALID_RELATIVE_POSE";
          }
          dog_prior_map_fastlio2_frontend_exp::advanceCoupledAnchor(
              &coupled_anchor_, reference_stamp_ns, causal_interval);
        }
        else if (coupled_anchor_.valid)
        {
          coupled_anchor_.valid = false;
          coupled_anchor_.frozen = false;
          coupled_anchor_.status = "EKF_INTERVAL_INVALID";
          coupled_anchor_.invalidated_stamp_ns = reference_stamp_ns;
          if (record) record->reason = "EKF_INTERVAL_INVALID";
        }
        else if (record)
          record->reason = interval_s > 0.25 ? "EKF_INTERVAL_TOO_LARGE" : "EKF_INTERVAL_NOT_INCREASING";
      }
      else if (coupled_anchor_.valid)
      {
        coupled_anchor_.valid = false;
        coupled_anchor_.frozen = false;
        coupled_anchor_.status = "EKF_INTERVAL_UNAVAILABLE";
        coupled_anchor_.invalidated_stamp_ns = reference_stamp_ns;
        if (record) record->reason = "EKF_INTERVAL_UNAVAILABLE";
      }
      else if (record)
        record->reason = "NO_PREVIOUS_CAUSAL_PREDICTION";

      previous_coupled_ekf_pose_ = EkfPoseSample{};
      previous_coupled_ekf_pose_.stamp_ns = prediction.stamp_ns;
      previous_coupled_ekf_pose_.map_T_lidar = prediction.map_T_lidar;
      has_previous_coupled_ekf_pose_ = true;
    }
    else
    {
      coupled_anchor_.valid = false;
      coupled_anchor_.frozen = false;
      coupled_anchor_.status = prediction.status;
      coupled_anchor_.invalidated_stamp_ns = reference_stamp_ns;
      has_previous_coupled_ekf_pose_ = false;
      if (record) record->reason = prediction.status;
    }
  }

  static std::string determinismCsvHeader()
  {
    return "frame_index,lidar_header_stamp,ros_now,wall_time,ndt_align_complete_wall_sec,ndt_odom_publish_wall_sec,"
           "cloud_seq,cloud_size_raw,cloud_size_after_filter,cloud_hash,"
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
           "pending_lidar_count,pending_overflow_count,coupled_mode,ekf_prediction_status,ekf_prediction_stamp,"
           "ekf_prediction_target_stamp,ekf_prediction_age_ms,ekf_prediction_tx,ekf_prediction_ty,ekf_prediction_tz,ekf_prediction_qx,"
           "ekf_prediction_qy,ekf_prediction_qz,ekf_prediction_qw,anchor_status,anchor_valid,anchor_stamp,anchor_age_s,"
           "anchor_status_after,anchor_valid_after,coupled_status,coupled_triggered,coupled_attempted,"
           "coupled_recommended,coupled_feedback_applied,coupled_feedback_changed_observation,"
           "coupled_weak_dimension,coupled_weak_eta0,coupled_weak_eta1,coupled_weak_eta2,coupled_weak_eta3,"
           "coupled_weak_eta4,coupled_weak_eta5,coupled_strong_eta0,coupled_strong_eta1,coupled_strong_eta2,"
           "coupled_strong_eta3,coupled_strong_eta4,coupled_strong_eta5,coupled_weak_pose_tx,coupled_weak_pose_ty,"
           "coupled_weak_pose_tz,coupled_weak_pose_qx,coupled_weak_pose_qy,coupled_weak_pose_qz,coupled_weak_pose_qw,"
           "coupled_full_pose_tx,coupled_full_pose_ty,coupled_full_pose_tz,coupled_full_pose_qx,coupled_full_pose_qy,"
           "coupled_full_pose_qz,coupled_full_pose_qw,coupled_candidate_tx,coupled_candidate_ty,coupled_candidate_tz,"
           "coupled_candidate_qx,coupled_candidate_qy,coupled_candidate_qz,coupled_candidate_qw,coupled_score_sum,"
           "coupled_jet_calls,coupled_value_calls,coupled_total_ms,coupled_strong_selected,coupled_candidate_step_limited,"
           "coupled_candidate_translation_limited,coupled_candidate_rotation_limited,"
           "ekf_translation_reason,visual_applied,ekf_translation_applied,ekf_sample_ns,ekf_target_ns,"
           "ekf_watermark_ns,ekf_prediction_query_ms,ekf_application_wall_sec,ekf_translation_change_m";
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

  void openFreshCausalRecordCsv(std::ofstream &stream, const std::string &name,
                                std::string header)
  {
    if (name == "prediction_records.csv")
    {
      const std::string marker = "scan_reference_stamp_ns,";
      const std::size_t position = header.find(marker);
      if (position != std::string::npos)
        header.insert(position + marker.size(), "ndt_align_start_wall_sec,");
    }
    const std::string path = coupled_causal_record_directory_ +
        (coupled_causal_record_directory_.back() == '/' ? "" : "/") + name;
    const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (descriptor < 0)
      throw std::runtime_error("cannot create fresh causal record output " + path +
          ": " + std::strerror(errno));
    ::close(descriptor);
    stream.open(path.c_str(), std::ios::out | std::ios::app);
    if (!stream) throw std::runtime_error("causal record output unavailable: " + path);
    stream << header << '\n';
    stream.flush();
    if (!stream) throw std::runtime_error("failed to initialize causal record output: " + path);
  }

  static std::string hash64Text(uint64_t value)
  {
    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << value;
    return out.str();
  }

  static std::string tileIdsText(const std::vector<std::string> &ids)
  {
    std::ostringstream out;
    for (std::size_t i = 0; i < ids.size(); ++i)
    {
      if (i) out << ';';
      out << ids[i];
    }
    return out.str();
  }

  static void appendMatrixCsv(std::ostream &out, const Eigen::Matrix4d &pose)
  {
    out << '"';
    for (int row = 0; row < 4; ++row)
      for (int col = 0; col < 4; ++col)
      {
        if (row || col) out << ';';
        out << (pose.allFinite() ? pose(row, col) :
            std::numeric_limits<double>::quiet_NaN());
      }
    out << '"';
  }

  void writeCausalRecord(const CausalRecordFrame &record,
                         const DeterminismRow &diagnostic_row)
  {
    if (!coupled_causal_record_only_enable_) return;
    const ros::WallTime write_start = ros::WallTime::now();
    const std::clock_t cpu_start = std::clock();
    std::ostringstream prediction, anchor, trigger;
    prediction << std::setprecision(17)
        << record.frame_index << ',' << record.scan_stamp_ns << ',' << record.scan_reference_stamp_ns << ','
        << diagnostic_row.ndt_align_start_wall_sec << ',' << diagnostic_row.ndt_align_complete_wall_sec << ','
        << record.prediction_query_wall_sec << ','
        << diagnostic_row.ndt_odom_publish_wall_sec << ','
        << (record.query_before_same_frame_ndt_publish ? 1 : 0) << ','
        << record.prediction.status << ',' << (record.prediction.valid ? 1 : 0) << ','
        << (record.causal_delivery_valid ? 1 : 0) << ',';
    if (record.prediction.sample_available)
    {
      const EkfPoseSample &sample = record.prediction.sample;
      Eigen::Quaterniond q(sample.map_T_lidar.block<3,3>(0,0));
      q.normalize();
      prediction << sample.stamp_ns << ',' << sample.message_seq << ',' << sample.publisher << ','
          << sample.frame_id << ',' << sample.child_frame_id << ',' << sample.receipt_ros_sec << ','
          << sample.callback_receive_wall_sec << ',' << sample.callback_accept_wall_sec << ','
          << sample.subscriber_history_revision << ",NOT_EXPOSED," << sample.map_T_lidar(0,3) << ','
          << sample.map_T_lidar(1,3) << ',' << sample.map_T_lidar(2,3) << ',' << q.x() << ','
          << q.y() << ',' << q.z() << ',' << q.w() << ','
          << sample.linear_velocity_lidar.x() << ',' << sample.linear_velocity_lidar.y() << ','
          << sample.linear_velocity_lidar.z() << ',' << sample.angular_velocity_lidar.x() << ','
          << sample.angular_velocity_lidar.y() << ',' << sample.angular_velocity_lidar.z() << ','
          << "child_frame_id_axes (nav_msgs/Odometry twist convention)";
    }
    else
    {
      prediction << std::string(9, ',') << "NOT_EXPOSED" << std::string(14, ',')
                 << "child_frame_id_axes (nav_msgs/Odometry twist convention)";
    }
    prediction << ',' << record.prediction.age_ms << ',' << record.prediction.stamp_ns << ',';
    appendMatrixCsv(prediction, record.prediction.map_T_lidar);
    prediction << ',' << hash64Text(record.source_hash64) << ',' << tileIdsText(record.target_tile_ids) << ','
        << hash64Text(record.target_cloud_hash64) << ',' << record.target_points << ','
        << (record.ndt_converged ? 1 : 0) << '\n';

    const auto anchorAge = [&record](const dog_prior_map_fastlio2_frontend_exp::CoupledAnchorState &state) {
      return state.origin_stamp_ns && record.scan_reference_stamp_ns >= state.origin_stamp_ns ?
          (record.scan_reference_stamp_ns - state.origin_stamp_ns) / 1e9 : 0.0;
    };
    anchor << std::setprecision(17) << record.frame_index << ',' << record.scan_stamp_ns << ','
        << (record.anchor_before.valid ? 1 : 0) << ',' << (record.anchor_before.frozen ? 1 : 0) << ','
        << record.anchor_before.status << ',' << record.anchor_before.origin_stamp_ns << ','
        << record.anchor_before.propagated_stamp_ns << ',' << anchorAge(record.anchor_before) << ',';
    appendMatrixCsv(anchor, record.anchor_before.prediction);
    anchor << ',' << record.propagation.interval_start_ns << ',' << record.propagation.interval_end_ns << ','
        << record.propagation.interval_sec << ',' << (record.propagation.attempted ? 1 : 0) << ','
        << (record.propagation.interval_valid ? 1 : 0) << ',' << record.propagation.reason << ','
        << (record.anchor_after_propagation.valid ? 1 : 0) << ','
        << (record.anchor_after_propagation.frozen ? 1 : 0) << ','
        << record.anchor_after_propagation.status << ','
        << record.anchor_after_propagation.origin_stamp_ns << ','
        << record.anchor_after_propagation.propagated_stamp_ns << ',';
    appendMatrixCsv(anchor, record.anchor_after_propagation.prediction);
    anchor << ',' << (record.anchor_after_settle.valid ? 1 : 0) << ','
        << (record.anchor_after_settle.frozen ? 1 : 0) << ',' << record.anchor_after_settle.status << ','
        << record.anchor_after_settle.origin_stamp_ns << ','
        << record.anchor_after_settle.propagated_stamp_ns << ',';
    appendMatrixCsv(anchor, record.anchor_after_settle.prediction);
    anchor << ',' << (record.anchor_after_settle.valid ? "" : record.anchor_after_settle.status) << '\n';

    trigger << std::setprecision(17) << record.frame_index << ',' << record.scan_stamp_ns << ','
        << record.ndt_status << ',' << (record.ndt_effective ? 1 : 0) << ','
        << record.innovation.translation_m << ',' << record.innovation.rotation_deg << ",0.12,3.0,"
        << (record.innovation.valid && record.innovation.triggered ? 1 : 0) << ','
        << (record.prediction.valid ? 1 : 0) << ',' << (record.causal_delivery_valid ? 1 : 0) << ','
        << (record.anchor_valid_for_pair ? 1 : 0) << ','
        << (record.anchor_after_propagation.frozen ? 1 : 0) << ','
        << record.anchor_after_propagation.origin_stamp_ns << ','
        << record.anchor_after_propagation.propagated_stamp_ns << ','
        << anchorAge(record.anchor_after_propagation) << ',' << (record.frame_index == 1 ? 1 : 0) << ','
        << (record.pair_eligible ? 1 : 0) << ',' << record.eligibility_reason << ',';
    appendMatrixCsv(trigger, record.nominal_ndt_pose);
    trigger << ',';
    appendMatrixCsv(trigger, record.prediction.map_T_lidar);
    trigger << '\n';

    causal_prediction_csv_ << prediction.str();
    causal_anchor_csv_ << anchor.str();
    causal_trigger_csv_ << trigger.str();
    causal_prediction_csv_.flush();
    causal_anchor_csv_.flush();
    causal_trigger_csv_.flush();
    const double wall_ms = (ros::WallTime::now() - write_start).toSec() * 1000.0;
    const double cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
    causal_record_overhead_csv_ << std::setprecision(17) << record.frame_index << ','
        << record.scan_stamp_ns << ',' << wall_ms << ',' << cpu_ms << '\n';
    causal_record_overhead_csv_.flush();
  }

  void writeDeterminismRow(const DeterminismRow &row)
  {
    if (!determinism_diagnostic_enable_ || !determinism_csv_.is_open()) return;
    std::ostringstream out;
    out << std::setprecision(17)
        << row.frame_index << "," << row.lidar_header_stamp << "," << row.ros_now << ","
        << row.wall_time << "," << row.ndt_align_complete_wall_sec << ","
        << row.ndt_odom_publish_wall_sec << "," << row.cloud_seq << "," << row.cloud_size_raw << ","
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
        << row.pending_lidar_count << "," << row.pending_overflow_count << ","
        << row.coupled_mode << "," << row.ekf_prediction_status << "," << row.ekf_prediction_stamp << ","
        << row.ekf_prediction_target_stamp << "," << row.ekf_prediction_age_ms << ",";
    appendPoseCsv(out, row.ekf_prediction_pose);
    out << "," << row.anchor_status << "," << (row.anchor_valid ? 1 : 0) << ","
        << row.anchor_stamp << "," << row.anchor_age_s << "," << row.anchor_status_after << ","
        << (row.anchor_valid_after ? 1 : 0) << "," << row.coupled_status << ","
        << (row.coupled_triggered ? 1 : 0) << "," << (row.coupled_attempted ? 1 : 0) << ","
        << (row.coupled_recommended ? 1 : 0) << "," << (row.coupled_feedback_applied ? 1 : 0) << ","
        << (row.coupled_feedback_changed_observation ? 1 : 0) << ","
        << row.coupled_weak_dimension;
    for (int i = 0; i < 6; ++i) out << "," << row.coupled_weak_eta(i);
    for (int i = 0; i < 6; ++i) out << "," << row.coupled_strong_eta(i);
    out << ","; appendPoseCsv(out, row.coupled_weak_pose);
    out << ","; appendPoseCsv(out, row.coupled_full_pose);
    out << ","; appendPoseCsv(out, row.coupled_candidate);
    out << "," << row.coupled_score_sum << "," << row.coupled_jet_calls << ","
        << row.coupled_value_calls << "," << row.coupled_total_ms << ","
        << (row.coupled_strong_selected ? 1 : 0) << ","
        << (row.coupled_candidate_step_limited ? 1 : 0) << ","
        << (row.coupled_candidate_translation_limited ? 1 : 0) << ","
        << (row.coupled_candidate_rotation_limited ? 1 : 0) << ','
        << row.ekf_translation_reason << ',' << row.visual_applied << ',' << row.ekf_translation_applied << ','
        << row.ekf_sample_ns << ',' << row.ekf_target_ns << ',' << row.ekf_watermark_ns << ','
        << row.ekf_prediction_query_ms << ',' << row.ekf_application_wall_sec << ',' << row.ekf_translation_change_m;
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
    if (local_map_enable_)
    {
      std::string error;
      if (!local_map_tiles_.loadManifest(local_map_manifest_path_, &error))
        throw std::runtime_error("failed to load local map manifest: " + error);
      target_cloud_ = local_map_tiles_.target();
      ROS_INFO("[DogPriorMap NDT] local map metadata loaded: manifest=%s tiles=%zu tile=%.1fm; global XYZ was not loaded",
               local_map_manifest_path_.c_str(), local_map_tiles_.tileCount(), local_map_tiles_.tileSize());
    }
    else
    {
      if (map_pcd_path_.empty()) throw std::runtime_error("map/pcd_fallback_path is empty");
      pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>());
      if (pcl::io::loadPCDFile(map_pcd_path_, *raw) != 0)
        throw std::runtime_error("failed to load map pcd: " + map_pcd_path_);

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
    }
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(ndt_transformation_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iterations_);
    if (!geometry_diagnostics_csv_path_.empty() && !local_map_enable_)
    {
      map_geometry_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
      map_geometry_tree_->setInputCloud(target_cloud_);
    }
  }

  std::pair<long, long> processMemoryKiB() const
  {
    long rss = -1, hwm = -1;
    std::ifstream status("/proc/self/status");
    std::string key;
    while (status >> key)
    {
      if (key == "VmRSS:") status >> rss;
      else if (key == "VmHWM:") status >> hwm;
      std::string rest;
      std::getline(status, rest);
      if (rss >= 0 && hwm >= 0) break;
    }
    return std::make_pair(rss, hwm);
  }

  void writeLocalMapDiagnostics(uint64_t scan_ns,
                                const LocalMapUpdate &update,
                                std::size_t source_points,
                                const std::string &coverage_status,
                                const GeometryStats &geometry,
                                double target_rebuild_ms)
  {
    if (!local_map_diagnostics_csv_.is_open()) return;
    std::ostringstream ids;
    for (std::size_t i = 0; i < update.loaded_tile_ids.size(); ++i)
    {
      if (i) ids << ';';
      ids << update.loaded_tile_ids[i];
    }
    const std::pair<long, long> memory = processMemoryKiB();
    const std::size_t point_count = update.target ? update.target->size() :
        (target_cloud_ ? target_cloud_->size() : 0);
    const double payload_mib = static_cast<double>(point_count * sizeof(pcl::PointXYZ)) /
        (1024.0 * 1024.0);
    local_map_diagnostics_csv_ << std::setprecision(17) << scan_ns << ",1,"
        << (update.updated ? 1 : 0) << ',' << update.reason << ',' << update.error << ','
        << coverage_status << ',' << update.loaded_tile_ids.size() << ',' << ids.str() << ','
        << source_points << ',' << point_count << ','
        << update.projected_min.x() << ',' << update.projected_max.x() << ','
        << update.projected_min.y() << ',' << update.projected_max.y() << ','
        << update.projected_min.z() << ',' << update.projected_max.z() << ','
        << update.tile_io_ms << ',' << update.target_merge_ms << ',' << target_rebuild_ms << ','
        << geometry.median << ',' << geometry.p90 << ',' << geometry.overlap_05 << ','
        << payload_mib << ',' << memory.first << ',' << memory.second << '\n';
    local_map_diagnostics_csv_.flush();
  }

  GeometryStats measureGeometry(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                const Eigen::Matrix4d &map_T_lidar) const
  {
    GeometryStats stats;
    if (!source || source->empty() || !map_geometry_tree_ || !map_T_lidar.allFinite()) return stats;
    std::vector<double> distances;
    distances.reserve(source->size());
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    std::size_t within_05 = 0, within_2 = 0, within_8 = 0;
    const Eigen::Matrix3d R = map_T_lidar.block<3, 3>(0, 0);
    const Eigen::Vector3d p = map_T_lidar.block<3, 1>(0, 3);
    for (const auto &point : source->points)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const Eigen::Vector3d map_point = R * Eigen::Vector3d(point.x, point.y, point.z) + p;
      const pcl::PointXYZ query(static_cast<float>(map_point.x()),
                                static_cast<float>(map_point.y()),
                                static_cast<float>(map_point.z()));
      if (map_geometry_tree_->nearestKSearch(query, 1, indices, squared_distances) != 1 ||
          squared_distances[0] < 0.0f) continue;
      const double distance = std::sqrt(static_cast<double>(squared_distances[0]));
      distances.push_back(distance);
      within_05 += distance <= 0.5 ? 1 : 0;
      within_2 += distance <= 2.0 ? 1 : 0;
      within_8 += distance <= 8.0 ? 1 : 0;
    }
    if (distances.empty()) return stats;
    std::sort(distances.begin(), distances.end());
    stats.points = distances.size();
    stats.median = sortedPercentile(distances, 0.50);
    stats.p90 = sortedPercentile(distances, 0.90);
    stats.overlap_05 = static_cast<double>(within_05) / distances.size();
    stats.overlap_2 = static_cast<double>(within_2) / distances.size();
    stats.overlap_8 = static_cast<double>(within_8) / distances.size();
    return stats;
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
    if (!local_imu_rotation_prior_enable_ && !coupled_enabled_ &&
        !experimental_ekf_translation_fallback_enable_)
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
      if (experimental_ekf_translation_fallback_enable_)
        throw std::runtime_error("P21 pending overflow: refusing an unwatermarked scan");
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
      const bool ekf_watermark_ready = !(coupled_enabled_ || experimental_ekf_translation_fallback_enable_) ||
          (has_ekf_high_rate_watermark_ &&
           latest_ekf_high_rate_stamp_ns_ >= front.reference_stamp.toNSec());
      if (!imu_watermark_ready || !ekf_watermark_ready)
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
    CausalRecordFrame causal_record;
    causal_record.scan_stamp_ns = stamp.toNSec();
    causal_record.scan_reference_stamp_ns = timing.reference_stamp.toNSec();
    causal_record.anchor_before = coupled_anchor_;
    causal_record.anchor_after_propagation = coupled_anchor_;
    causal_record.anchor_after_settle = coupled_anchor_;
    causal_record.target_tile_ids = local_map_tiles_.activeTileIds();
    causal_record.target_cloud_hash64 = active_target_hash64_;
    causal_record.target_points = target_cloud_ ? target_cloud_->size() : 0;
    diagnostic_row.local_imu_prior_enabled = local_imu_rotation_prior_enable_;
    diagnostic_row.coupled_mode = coupled_mode_;
    if (determinism_diagnostic_enable_)
    {
      diagnostic_row.frame_index = ++determinism_frame_index_;
      causal_record.frame_index = diagnostic_row.frame_index;
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
    causal_record.source_points = source->size();
    causal_record.source_hash64 = diagnostic_row.cloud_hash;
    const double preprocess_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    publishCloud(source, stamp, base_frame_, pub_filtered_);
    if (static_cast<int>(source->size()) < min_effective_points_)
    {
      const double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishDiagnostics(timing.reference_stamp, false, 0.0, preprocess_ms, localization_ms,
                         static_cast<int>(source->size()), target_cloud_->size(), 0.0, 0);
      diagnostic_row.initial_guess_source = "not_evaluated";
      diagnostic_row.initial_guess_reason = "insufficient_points";
      diagnostic_row.coupled_status = coupled_enabled_ ? "INSUFFICIENT_POINTS" : "NOT_RUN";
      if (local_map_enable_)
      {
        LocalMapUpdate update;
        update.ok = true;
        update.reason = "insufficient_source_points";
        update.loaded_tile_ids = local_map_tiles_.activeTileIds();
        update.target = target_cloud_;
        writeLocalMapDiagnostics(stamp.toNSec(), update, source->size(),
            "NOT_EVALUATED", GeometryStats(), 0.0);
      }
      writeDeterminismRow(diagnostic_row);
      causal_record.ndt_status = "INSUFFICIENT_SOURCE_POINTS";
      causal_record.anchor_after_propagation = coupled_anchor_;
      causal_record.anchor_after_settle = coupled_anchor_;
      causal_record.eligibility_reason = "NDT_NOT_EFFECTIVE";
      writeCausalRecord(causal_record, diagnostic_row);
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
    if (visual_initial_guess_enable_ && has_previous_pose_reference_stamp_ &&
        diagnostic_row.local_imu_prior_used) {
      std::string reason;
      if (visual_initial_guess_.apply(previous_pose_reference_stamp_.toNSec(),
            timing.reference_stamp.toNSec(), previous_pose_, initial_guess, reason))
      {
        diagnostic_row.visual_applied = true;
        diagnostic_row.initial_guess_source = "p4_visual_translation_imu_rotation";
      }
      diagnostic_row.initial_guess_reason = reason;
    }
    if (experimental_ekf_translation_fallback_enable_)
    {
      EkfPrediction prediction;
      const auto query_start = ros::WallTime::now();
      if (!diagnostic_row.visual_applied && has_previous_pose_ && diagnostic_row.local_imu_prior_used)
        prediction = ekfPredictionAtOrBeforeLocked(timing.reference_stamp.toNSec());
      diagnostic_row.ekf_target_ns = timing.reference_stamp.toNSec();
      diagnostic_row.ekf_sample_ns = prediction.sample_stamp_ns;
      diagnostic_row.ekf_watermark_ns = latest_ekf_high_rate_stamp_ns_;
      diagnostic_row.ekf_prediction_status = prediction.status;
      diagnostic_row.ekf_prediction_stamp = prediction.sample_stamp_ns / 1e9;
      diagnostic_row.ekf_prediction_target_stamp = timing.reference_stamp.toSec();
      diagnostic_row.ekf_prediction_age_ms = prediction.age_ms;
      diagnostic_row.ekf_prediction_pose = prediction.map_T_lidar;
      const Eigen::Vector3d old_translation = initial_guess.block<3,1>(0,3);
      diagnostic_row.ekf_translation_reason = applyEkfTranslationPrior(true, visual_initial_guess_enable_,
          diagnostic_row.visual_applied, has_previous_pose_, diagnostic_row.local_imu_prior_used,
          has_ekf_high_rate_watermark_ && latest_ekf_high_rate_stamp_ns_ >= timing.reference_stamp.toNSec(),
          true, prediction.sample_stamp_ns, timing.reference_stamp.toNSec(), prediction.map_T_lidar, &initial_guess);
      diagnostic_row.ekf_translation_applied = diagnostic_row.ekf_translation_reason == "EKF_TRANSLATION_APPLIED";
      diagnostic_row.ekf_translation_change_m = (initial_guess.block<3,1>(0,3)-old_translation).norm();
      diagnostic_row.ekf_prediction_query_ms = (ros::WallTime::now()-query_start).toSec()*1000.0;
      diagnostic_row.ekf_application_wall_sec = ros::WallTime::now().toSec();
      if (diagnostic_row.ekf_translation_applied)
      {
        diagnostic_row.initial_guess_source = "causal_ekf_translation_imu_rotation";
        diagnostic_row.initial_guess_reason = diagnostic_row.ekf_translation_reason;
      }
    }
    diagnostic_row.initial_guess = initial_guess;

    LocalMapUpdate local_map_update;
    double local_target_rebuild_ms = 0.0;
    if (local_map_enable_)
    {
      local_map_update = local_map_tiles_.ensureCoverage(
          source, initial_guess, local_map_safety_margin_m_, local_map_hysteresis_margin_m_);
      if (!local_map_update.ok)
      {
        ROS_ERROR("[DogPriorMap NDT] local map update failed; refusing align: %s",
                  local_map_update.error.c_str());
        local_map_update.reason = "selection_or_tile_load_error";
        if (geometry_diagnostics_csv_.is_open())
          geometry_diagnostics_csv_ << std::setprecision(17) << stamp.toNSec() << ','
              << source->size() << ',' << target_cloud_->size() << ",LOCAL_MAP_ERROR,nan,nan,nan,nan,nan,"
              << "0,nan,0,0,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,0\n";
        writeLocalMapDiagnostics(stamp.toNSec(), local_map_update, source->size(),
            "ERROR", GeometryStats(), 0.0);
        const double elapsed_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
        publishDiagnostics(timing.reference_stamp, false, 0.0, preprocess_ms, elapsed_ms,
                           static_cast<int>(source->size()), target_cloud_->size(), 0.0, 0);
        diagnostic_row.initial_guess_reason = "p45_local_map_update_failed";
        writeDeterminismRow(diagnostic_row);
        causal_record.ndt_status = "LOCAL_MAP_UPDATE_FAILED";
        causal_record.eligibility_reason = "NDT_NOT_EFFECTIVE";
        causal_record.target_tile_ids = local_map_tiles_.activeTileIds();
        causal_record.target_cloud_hash64 = active_target_hash64_;
        causal_record.target_points = target_cloud_ ? target_cloud_->size() : 0;
        causal_record.anchor_after_propagation = coupled_anchor_;
        causal_record.anchor_after_settle = coupled_anchor_;
        writeCausalRecord(causal_record, diagnostic_row);
        return;
      }
      if (local_map_update.updated)
      {
        const ros::WallTime target_rebuild_start = ros::WallTime::now();
        map_geometry_tree_.reset();
        target_cloud_ = local_map_update.target;
        ndt_.setInputTarget(target_cloud_);
        if (coupled_causal_record_only_enable_)
          active_target_hash64_ = stableCloudHash(target_cloud_);
        map_geometry_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        map_geometry_tree_->setInputCloud(target_cloud_);
        local_target_rebuild_ms = (ros::WallTime::now() - target_rebuild_start).toSec() * 1000.0;
        if (publish_local_target_ && pub_local_target_)
          publishCloud(target_cloud_, timing.reference_stamp, map_frame_, pub_local_target_);
      }
    }
    causal_record.target_tile_ids = local_map_enable_ ? local_map_tiles_.activeTileIds() :
        std::vector<std::string>();
    causal_record.target_cloud_hash64 = active_target_hash64_;
    causal_record.target_points = target_cloud_ ? target_cloud_->size() : 0;

    const GeometryStats initial_geometry = map_geometry_tree_ ?
        measureGeometry(source, initial_guess) : GeometryStats();
    const bool geometry_gate_passed = !geometry_gate_enable_ ||
        (std::isfinite(initial_geometry.median) && std::isfinite(initial_geometry.p90) &&
         std::isfinite(initial_geometry.overlap_05) &&
         initial_geometry.median <= geometry_gate_max_median_m_ &&
         initial_geometry.p90 <= geometry_gate_max_p90_m_ &&
         initial_geometry.overlap_05 >= geometry_gate_min_overlap_05_);
    if (local_map_enable_)
      writeLocalMapDiagnostics(stamp.toNSec(), local_map_update, source->size(),
          geometry_gate_passed ? "COVERAGE_PASS" : "GEOMETRY_GUARD_REJECTED",
          initial_geometry, local_target_rebuild_ms);
    if (!geometry_gate_passed)
    {
      ROS_ERROR("[DogPriorMap NDT] startup geometry gate rejected scan: median=%.3f m p90=%.3f m overlap0.5=%.3f; NDT align was not called",
                initial_geometry.median, initial_geometry.p90, initial_geometry.overlap_05);
      if (geometry_diagnostics_csv_.is_open())
      {
        geometry_diagnostics_csv_ << std::setprecision(17) << stamp.toNSec() << ','
            << source->size() << ',' << target_cloud_->size() << ",REJECTED,"
            << initial_geometry.median << ',' << initial_geometry.p90 << ','
            << initial_geometry.overlap_05 << ',' << initial_geometry.overlap_2 << ','
            << initial_geometry.overlap_8 << ",0,nan,0,0,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,0\n";
        geometry_diagnostics_csv_.flush();
      }
      const double elapsed_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
      publishDiagnostics(timing.reference_stamp, false, 0.0, elapsed_ms, elapsed_ms,
                         static_cast<int>(source->size()), target_cloud_->size(), 0.0, 0);
      diagnostic_row.initial_guess_reason = "p41_initial_geometry_gate_rejected";
      writeDeterminismRow(diagnostic_row);
      causal_record.ndt_status = "GEOMETRY_GATE_REJECTED";
      causal_record.eligibility_reason = "NDT_NOT_EFFECTIVE";
      causal_record.anchor_after_propagation = coupled_anchor_;
      causal_record.anchor_after_settle = coupled_anchor_;
      writeCausalRecord(causal_record, diagnostic_row);
      return;
    }

    ndt_.setInputSource(source);
    if (experimental_source_capture_enable_ && experimental_source_capture_.requested(stamp.toNSec()))
      experimental_source_capture_.save(diagnostic_row.frame_index, stamp.toNSec(), *source, diagnostic_row.cloud_hash);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    const ros::WallTime align_start = ros::WallTime::now();
    diagnostic_row.ndt_align_start_wall_sec = align_start.toSec();
    ndt_.align(aligned, initial_guess.cast<float>());
    diagnostic_row.ndt_align_complete_wall_sec = ros::WallTime::now().toSec();
    double align_ms = (ros::WallTime::now() - align_start).toSec() * 1000.0;
    bool ok = ndt_.hasConverged();
    double score = ndt_.getFitnessScore();
    int iterations = ndt_.getFinalNumIteration();
    Eigen::Matrix4d result = ndt_.getFinalTransformation().cast<double>();
    causal_record.ndt_effective = ok;
    causal_record.ndt_converged = ok;
    causal_record.ndt_status = ok ? "NDT_CONVERGED" : "NDT_NOT_CONVERGED";
    causal_record.nominal_ndt_pose = result;
    const GeometryStats raw_geometry = map_geometry_tree_ && result.allFinite() ?
        measureGeometry(source, result) : GeometryStats();
    if (mature_integration_enable_)
    {
      runMatureIntegration(source, initial_guess, diagnostic_row.frame_index,
          timing.reference_stamp.toNSec(), timing.end_stamp.toNSec(), diagnostic_row.visual_applied, &result,
          &aligned, &ok, &score, &iterations, &align_ms);
      diagnostic_row.ndt_align_complete_wall_sec = ros::WallTime::now().toSec();
    }
    diagnostic_row.ndt_has_converged = ok;
    diagnostic_row.ndt_fitness = score;
    diagnostic_row.ndt_iterations = iterations;

    bool step_limited = false;
    double localization_ms = (ros::WallTime::now() - callback_start).toSec() * 1000.0;
    if (ok)
    {
      diagnostic_row.raw_ndt = result;
      if (visual_geometry_csv_.is_open()) {
        namespace frontend = dog_prior_map_fastlio2_frontend_exp;
        const auto start = ros::WallTime::now();
        frontend::CoupledNdtBackend backend;
        backend.jet = [this, source](const Eigen::Matrix4f& pose,
            const frontend::CoupledVector6& parameters) {
          return ndt_.nativeJet(source, pose, parameters);
        };
        backend.score = [this, source](const Eigen::Matrix4f& pose) {
          return ndt_.dynamicScore(source, pose);
        };
        const auto jet = frontend::coupledNominalJet(result.cast<float>(), source->size(), backend, .8);
        Eigen::SelfAdjointEigenSolver<frontend::CoupledMatrix6> solver;
        if (jet.valid) solver.compute(jet.H);
        const bool valid = jet.valid && solver.info() == Eigen::Success &&
            solver.eigenvalues().allFinite() && solver.eigenvectors().allFinite();
        visual_geometry_csv_ << std::setprecision(17) << timing.reference_stamp.toNSec()
            << ',' << valid << ',' << (ros::WallTime::now()-start).toSec()*1000;
        for (int i=0; i<6; ++i) visual_geometry_csv_ << ',' << (valid ? solver.eigenvalues()(i) : NAN);
        for (int i=0; i<6; ++i) for (int j=0; j<6; ++j)
          visual_geometry_csv_ << ',' << (valid ? solver.eigenvectors()(i,j) : NAN);
        visual_geometry_csv_ << "\n"; visual_geometry_csv_.flush();
      }
      const Eigen::Matrix4d raw_delta = initial_guess.inverse() * result;
      const Eigen::AngleAxisd raw_delta_angle(raw_delta.block<3, 3>(0, 0));
      diagnostic_row.raw_delta.head<3>() = raw_delta.block<3, 1>(0, 3);
      diagnostic_row.raw_delta.tail<3>() = raw_delta_angle.axis() * raw_delta_angle.angle();
      diagnostic_row.raw_delta_translation = raw_delta.block<3, 1>(0, 3).norm();
      diagnostic_row.raw_delta_rotation_deg = rotationAngleDeg(raw_delta.block<3, 3>(0, 0));

      EkfPrediction ekf_prediction;
      dog_prior_map_fastlio2_frontend_exp::WeakCoupledResult coupled_result;
      bool have_coupled_result = false;
      bool feedback_candidate = false;
      const uint64_t reference_stamp_ns = timing.reference_stamp.toNSec();
      if (coupled_enabled_)
      {
        ekf_prediction = ekfPredictionAtOrBeforeLocked(reference_stamp_ns);
        diagnostic_row.ekf_prediction_status = ekf_prediction.status;
        diagnostic_row.ekf_prediction_stamp = ekf_prediction.sample_stamp_ns / 1e9;
        diagnostic_row.ekf_prediction_target_stamp = reference_stamp_ns / 1e9;
        diagnostic_row.ekf_prediction_age_ms = ekf_prediction.age_ms;
        diagnostic_row.ekf_prediction_pose = ekf_prediction.map_T_lidar;
        advanceCoupledAnchorFromPrediction(reference_stamp_ns, ekf_prediction, nullptr);

        diagnostic_row.anchor_status = coupled_anchor_.status;
        diagnostic_row.anchor_valid = coupled_anchor_.valid;
        diagnostic_row.anchor_stamp = coupled_anchor_.origin_stamp_ns / 1e9;
        diagnostic_row.anchor_age_s = coupled_anchor_.origin_stamp_ns &&
            reference_stamp_ns >= coupled_anchor_.origin_stamp_ns ?
            (reference_stamp_ns - coupled_anchor_.origin_stamp_ns) / 1e9 : 0.0;

        if (ekf_prediction.valid)
        {
          dog_prior_map_fastlio2_frontend_exp::CoupledNdtBackend backend;
          backend.jet = [this, source](const Eigen::Matrix4f &pose,
              const dog_prior_map_fastlio2_frontend_exp::CoupledVector6 &native_parameters) {
            return ndt_.nativeJet(source, pose, native_parameters);
          };
          backend.score = [this, source](const Eigen::Matrix4f &pose) {
            return ndt_.dynamicScore(source, pose);
          };
          dog_prior_map_fastlio2_frontend_exp::WeakCoupledConfig refinement_config;
          refinement_config.coupled = coupled_mode_ != "WEAK_ONLY_FEEDBACK";
          coupled_result = dog_prior_map_fastlio2_frontend_exp::runWeakCoupledRefinement(
              result.cast<float>(), ekf_prediction.map_T_lidar.cast<float>(), reference_stamp_ns,
              source->size(), true, coupled_anchor_, backend, refinement_config);
          diagnostic_row.coupled_total_ms = coupled_result.total_ms;
          have_coupled_result = true;
          diagnostic_row.coupled_status = coupled_result.status;
          diagnostic_row.coupled_triggered = coupled_result.triggered;
          diagnostic_row.coupled_attempted = coupled_result.attempted;
          diagnostic_row.coupled_recommended = coupled_result.recommended;
          diagnostic_row.coupled_strong_selected = coupled_result.strong_selected;
          diagnostic_row.coupled_weak_dimension = coupled_result.weak_dimension;
          diagnostic_row.coupled_jet_calls = coupled_result.jet_calls;
          diagnostic_row.coupled_value_calls = coupled_result.value_calls;
          diagnostic_row.coupled_weak_pose = coupled_result.weak_pose.cast<double>();
          diagnostic_row.coupled_full_pose = coupled_result.coupled_pose.cast<double>();
          diagnostic_row.coupled_candidate = coupled_result.candidate.cast<double>();
          diagnostic_row.coupled_weak_eta = coupled_result.weak_eta;
          diagnostic_row.coupled_strong_eta = coupled_result.strong_eta;
          diagnostic_row.coupled_score_sum = coupled_result.candidate_score;
          const bool feedback_mode = coupled_mode_ == "WEAK_ONLY_FEEDBACK" ||
                                     coupled_mode_ == "COUPLED_FEEDBACK";
          feedback_candidate = feedback_mode && coupled_result.recommended &&
                               validRigidPose(coupled_result.candidate.cast<double>());
          if (feedback_mode && coupled_result.recommended && !feedback_candidate)
            diagnostic_row.coupled_status = "INVALID_RECOMMENDED_POSE_RETAIN_NOMINAL";
        }
        else
        {
          diagnostic_row.coupled_status = ekf_prediction.status;
        }
      }
      else if (coupled_causal_record_only_enable_)
      {
        causal_record.anchor_before = coupled_anchor_;
        causal_record.prediction_query_wall_sec = ros::WallTime::now().toSec();
        causal_record.prediction = ekfPredictionAtOrBeforeLocked(reference_stamp_ns);
        diagnostic_row.ekf_prediction_status = causal_record.prediction.status;
        diagnostic_row.ekf_prediction_stamp = causal_record.prediction.sample_stamp_ns / 1e9;
        diagnostic_row.ekf_prediction_target_stamp = reference_stamp_ns / 1e9;
        diagnostic_row.ekf_prediction_age_ms = causal_record.prediction.age_ms;
        diagnostic_row.ekf_prediction_pose = causal_record.prediction.map_T_lidar;

        const EkfPoseSample &sample = causal_record.prediction.sample;
        causal_record.causal_delivery_valid = causal_record.prediction.valid &&
            causal_record.prediction.sample_available && sample.stamp_ns <= reference_stamp_ns &&
            std::isfinite(sample.callback_receive_wall_sec) &&
            std::isfinite(sample.callback_accept_wall_sec) &&
            sample.callback_receive_wall_sec <= sample.callback_accept_wall_sec &&
            sample.callback_accept_wall_sec <= diagnostic_row.ndt_align_start_wall_sec &&
            diagnostic_row.ndt_align_start_wall_sec <= causal_record.prediction_query_wall_sec &&
            sample.publisher == "/dog_prior_map_ekf" && sample.frame_id == map_frame_ &&
            sample.child_frame_id == base_frame_;

        EkfPrediction anchor_prediction = causal_record.prediction;
        if (!causal_record.causal_delivery_valid)
        {
          // An unverified or late-delivered sample must never establish an Anchor.
          // Reuse the production invalid-prediction lifecycle to invalidate stale state.
          anchor_prediction.valid = false;
          anchor_prediction.status = "CAUSAL_EKF_DELIVERY_INVALID";
        }
        advanceCoupledAnchorFromPrediction(reference_stamp_ns, anchor_prediction,
                                            &causal_record.propagation);
        causal_record.anchor_after_propagation = coupled_anchor_;
        diagnostic_row.anchor_status = coupled_anchor_.status;
        diagnostic_row.anchor_valid = coupled_anchor_.valid;
        diagnostic_row.anchor_stamp = coupled_anchor_.origin_stamp_ns / 1e9;
        diagnostic_row.anchor_age_s = coupled_anchor_.origin_stamp_ns &&
            reference_stamp_ns >= coupled_anchor_.origin_stamp_ns ?
            (reference_stamp_ns - coupled_anchor_.origin_stamp_ns) / 1e9 : 0.0;

        const bool valid_nominal = validRigidPose(result);
        const bool valid_prediction_pose = causal_record.causal_delivery_valid &&
            validRigidPose(causal_record.prediction.map_T_lidar);
        if (causal_record.causal_delivery_valid && valid_nominal && valid_prediction_pose)
          causal_record.innovation = dog_prior_map_fastlio2_frontend_exp::evaluateWeakCoupledInnovation(
              result.cast<float>(), causal_record.prediction.map_T_lidar.cast<float>());

        causal_record.anchor_valid_for_pair = causal_record.causal_delivery_valid &&
            coupled_anchor_.valid && !coupled_anchor_.frozen && coupled_anchor_.origin_stamp_ns &&
            coupled_anchor_.propagated_stamp_ns == reference_stamp_ns &&
            reference_stamp_ns >= coupled_anchor_.origin_stamp_ns &&
            (reference_stamp_ns - coupled_anchor_.origin_stamp_ns) / 1e9 <=
                dog_prior_map_fastlio2_frontend_exp::kCoupledAnchorLifetimeS;

        if (causal_record.causal_delivery_valid && causal_record.prediction.valid &&
            (!causal_record.innovation.valid || !causal_record.innovation.triggered))
          dog_prior_map_fastlio2_frontend_exp::settleWeakRefinementAnchor(
              &coupled_anchor_, reference_stamp_ns, causal_record.prediction.map_T_lidar, true);
        causal_record.anchor_after_settle = coupled_anchor_;
        diagnostic_row.anchor_status_after = coupled_anchor_.status;
        diagnostic_row.anchor_valid_after = coupled_anchor_.valid;

        causal_record.nominal_ndt_pose = result;
        if (!causal_record.prediction.valid)
          causal_record.eligibility_reason = causal_record.prediction.status == "NO_CAUSAL_EKF_SAMPLE" ?
              "PREDICTION_UNAVAILABLE" :
              (causal_record.prediction.status == "STALE_CAUSAL_EKF_SAMPLE" ?
                  "PREDICTION_STALE" : causal_record.prediction.status);
        else if (!causal_record.causal_delivery_valid)
          causal_record.eligibility_reason = "PREDICTION_CAUSALITY_INVALID";
        else if (!causal_record.innovation.valid)
          causal_record.eligibility_reason = "NDT_PREDICTION_INVALID";
        else if (!causal_record.innovation.triggered)
          causal_record.eligibility_reason = "NOT_TRIGGERED";
        else if (coupled_anchor_.frozen)
          causal_record.eligibility_reason = "ANCHOR_FROZEN";
        else if (!causal_record.anchor_valid_for_pair)
          causal_record.eligibility_reason = "TRIGGERED_BUT_ANCHOR_UNAVAILABLE";
        else if (causal_record.frame_index == 1)
          causal_record.eligibility_reason = "FIRST_FRAME_TRIGGER_NO_PAIR";
        else
        {
          causal_record.pair_eligible = causal_record.ndt_effective;
          causal_record.eligibility_reason = causal_record.pair_eligible ?
              "LEGAL_CAUSAL_TRIGGER" : "NDT_NOT_EFFECTIVE";
        }
      }

      Eigen::Matrix4d measurement_before_limit = feedback_candidate ?
          coupled_result.candidate.cast<double>() : result;
      Eigen::Matrix4d used_result = measurement_before_limit;
      diagnostic_row.used_before_step_limit = measurement_before_limit;
      step_limited = limitNdtStep(measurement_before_limit, used_result,
                                  diagnostic_row.translation_limited,
                                  diagnostic_row.rotation_limited);
      diagnostic_row.coupled_feedback_applied = feedback_candidate;
      diagnostic_row.coupled_candidate_step_limited = feedback_candidate && step_limited;
      diagnostic_row.coupled_candidate_translation_limited = feedback_candidate &&
          diagnostic_row.translation_limited;
      diagnostic_row.coupled_candidate_rotation_limited = feedback_candidate &&
          diagnostic_row.rotation_limited;
      if (feedback_candidate)
      {
        Eigen::Matrix4d nominal_limited = result;
        bool nominal_translation_limited = false, nominal_rotation_limited = false;
        limitNdtStep(result, nominal_limited, nominal_translation_limited, nominal_rotation_limited);
        const double translation_difference =
            (used_result.block<3, 1>(0, 3) - nominal_limited.block<3, 1>(0, 3)).norm();
        const double rotation_difference_deg = rotationAngleDeg(
            nominal_limited.block<3, 3>(0, 0).transpose() * used_result.block<3, 3>(0, 0));
        diagnostic_row.coupled_feedback_changed_observation =
            translation_difference > 1e-9 || rotation_difference_deg > 1e-8;
        dog_prior_map_fastlio2_frontend_exp::settleCoupledAnchor(
            &coupled_anchor_, reference_stamp_ns, used_result, false, true);
      }
      else if (coupled_enabled_ && ekf_prediction.valid && have_coupled_result &&
               !coupled_result.triggered && coupled_result.recommended == false)
      {
        dog_prior_map_fastlio2_frontend_exp::settleWeakRefinementAnchor(
            &coupled_anchor_, reference_stamp_ns,
            ekf_prediction.map_T_lidar, true);
      }
      diagnostic_row.final_used = used_result;
      diagnostic_row.anchor_status_after = coupled_anchor_.status;
      diagnostic_row.anchor_valid_after = coupled_anchor_.valid;
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
      publishPose(timing.reference_stamp, &diagnostic_row.ndt_odom_publish_wall_sec);
      if (step_limited)
      {
        pcl::PointCloud<pcl::PointXYZ> limited_aligned = transformCloud(source, used_result);
        publishAlignedCloud(limited_aligned, stamp);
      }
      else
      {
        publishAlignedCloud(aligned, stamp);
      }

    if (!ok)
    {
      causal_record.eligibility_reason = "NDT_NOT_EFFECTIVE";
      causal_record.anchor_after_propagation = coupled_anchor_;
      causal_record.anchor_after_settle = coupled_anchor_;
    }
    }

    const GeometryStats used_geometry = map_geometry_tree_ && ok && diagnostic_row.final_used.allFinite() ?
        measureGeometry(source, diagnostic_row.final_used) : GeometryStats();
    if (geometry_diagnostics_csv_.is_open())
    {
      geometry_diagnostics_csv_ << std::setprecision(17) << stamp.toNSec() << ','
          << source->size() << ',' << target_cloud_->size() << ",PASS,"
          << initial_geometry.median << ',' << initial_geometry.p90 << ','
          << initial_geometry.overlap_05 << ',' << initial_geometry.overlap_2 << ','
          << initial_geometry.overlap_8 << ',' << (ok ? 1 : 0) << ',' << score << ','
          << iterations << ',' << align_ms << ',' << raw_geometry.median << ','
          << raw_geometry.p90 << ',' << raw_geometry.overlap_05 << ','
          << raw_geometry.overlap_2 << ',' << raw_geometry.overlap_8 << ','
          << used_geometry.median << ',' << used_geometry.p90 << ','
          << used_geometry.overlap_05 << ',' << used_geometry.overlap_2 << ','
          << used_geometry.overlap_8 << ',' << (step_limited ? 1 : 0) << '\n';
      geometry_diagnostics_csv_.flush();
    }

    publishDiagnostics(timing.reference_stamp, ok, align_ms, preprocess_ms, localization_ms,
                       static_cast<int>(source->size()), target_cloud_->size(), score, iterations, step_limited);
    if (experimental_source_capture_enable_ && experimental_source_capture_.requested(stamp.toNSec()))
      experimental_source_capture_.savePoses(diagnostic_row.frame_index,
          {diagnostic_row.previous_pose, diagnostic_row.initial_guess, diagnostic_row.imu_rotation_guess,
           diagnostic_row.raw_ndt, diagnostic_row.final_used});
    writeDeterminismRow(diagnostic_row);
    causal_record.query_before_same_frame_ndt_publish =
        std::isfinite(causal_record.prediction_query_wall_sec) &&
        std::isfinite(diagnostic_row.ndt_odom_publish_wall_sec) &&
        causal_record.prediction_query_wall_sec < diagnostic_row.ndt_odom_publish_wall_sec;
    if (!causal_record.query_before_same_frame_ndt_publish && causal_record.prediction.valid)
    {
      causal_record.causal_delivery_valid = false;
      causal_record.pair_eligible = false;
      causal_record.eligibility_reason = "PREDICTION_QUERY_NOT_BEFORE_NDT_PUBLISH";
    }
    writeCausalRecord(causal_record, diagnostic_row);
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
  void publishPose(const ros::Time &stamp, double *odom_publish_wall_sec = nullptr)
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
    if (odom_publish_wall_sec)
      *odom_publish_wall_sec = ros::WallTime::now().toSec();
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
  ros::Subscriber sub_ekf_high_rate_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_pose_;
  ros::Publisher pub_path_;
  ros::Publisher pub_filtered_;
  ros::Publisher pub_aligned_;
  ros::Publisher pub_local_target_;
  ros::Publisher pub_diagnostics_;
  ros::Publisher pub_imu_watermark_;
  tf::TransformBroadcaster tf_broadcaster_;
  mutable std::mutex mutex_;

  std::string map_frame_;
  std::string base_frame_;
  std::string lidar_topic_;
  std::string lidar_msg_type_;
  std::string filtered_points_topic_;
  std::string diagnostics_topic_;
  std::string imu_watermark_topic_;
  std::string ndt_odom_topic_;
  std::string ndt_pose_topic_;
  std::string ndt_path_topic_;
  std::string points_aligned_topic_;
  std::string map_pcd_path_;
  std::string local_map_manifest_path_;
  std::string local_map_diagnostics_csv_path_;
  std::string imu_topic_;
  std::string coupled_mode_ = "CONTROL";
  std::string ekf_high_rate_topic_ = "/dog_livo/odom_high_rate";
  bool coupled_enabled_ = false;
  bool coupled_causal_record_only_enable_ = false;
  std::string coupled_causal_record_directory_;
  std::ofstream causal_prediction_csv_;
  std::ofstream causal_anchor_csv_;
  std::ofstream causal_trigger_csv_;
  std::ofstream causal_record_overhead_csv_;
  bool experimental_ekf_translation_fallback_enable_ = false;
  std::ofstream ekf_translation_samples_csv_;
  double coupled_prediction_max_age_sec_ = 0.02;
  double coupled_ekf_history_keep_sec_ = 60.0;

  struct ImuSample
  {
    double stamp = 0.0;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  };
  std::deque<ImuSample> imu_history_;
  bool has_imu_watermark_ = false;
  ros::Time latest_imu_stamp_;
  std::deque<EkfPoseSample> ekf_pose_history_;
  uint64_t ekf_pose_history_revision_ = 0;
  bool has_ekf_high_rate_watermark_ = false;
  uint64_t latest_ekf_high_rate_stamp_ns_ = 0;
  bool has_previous_coupled_ekf_pose_ = false;
  EkfPoseSample previous_coupled_ekf_pose_;
  dog_prior_map_fastlio2_frontend_exp::CoupledAnchorState coupled_anchor_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_geometry_tree_;
  LocalMapTiles local_map_tiles_;
  uint64_t active_target_hash64_ = 0;
  bool local_map_enable_ = false;
  bool publish_local_target_ = false;
  double local_map_safety_margin_m_ = 10.0;
  double local_map_hysteresis_margin_m_ = 30.0;
  std::string geometry_diagnostics_csv_path_;
  std::ofstream geometry_diagnostics_csv_;
  std::ofstream local_map_diagnostics_csv_;
  bool geometry_gate_enable_ = false;
  double geometry_gate_max_median_m_ = 1.0;
  double geometry_gate_max_p90_m_ = 3.0;
  double geometry_gate_min_overlap_05_ = 0.50;
  bool visual_initial_guess_enable_ = false;
  VisualAidedInitialGuess visual_initial_guess_;
  std::ofstream visual_geometry_csv_;
  dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<pcl::PointXYZ> ndt_;
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
  bool local_imu_gyro_to_lidar_enable_ = false;
  Eigen::Isometry3d T_imu_lidar_rotation_ = Eigen::Isometry3d::Identity();
  size_t pending_lidar_max_frames_ = 50;
  uint32_t lidar_subscriber_queue_size_ = 100;
  bool determinism_diagnostic_enable_ = false;
  bool experimental_source_capture_enable_ = false;
  ExperimentalSourceCapture experimental_source_capture_;
  bool mature_integration_enable_ = false;
  bool mature_dcreg_enable_ = false;
  bool mature_multi_ndt_enable_ = false;
  bool mature_skf_enable_ = false;
  bool p25_relative_visual_contract_ = false;
  SkfVisualMeasurementCatalog skf_visual_measurements_;
  ros::Publisher pub_skf_visual_;
  ros::Publisher pub_visual_degeneracy_;
  std::string skf_visual_topic_;
  std::ofstream skf_update_csv_;
  MatureNdtConfig mature_ndt_config_;
  std::ofstream mature_diagnostics_csv_;
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
