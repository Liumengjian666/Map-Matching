#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include "dog_prior_map_localization/core/estimator_types.hpp"
#include "dog_prior_map_localization/core/state_history.hpp"

namespace dog_prior_map_localization
{

/// 将三维向量转换为叉乘对应的反对称矩阵。
Eigen::Matrix3d skew(const Eigen::Vector3d &v);
/// 将角度制的 roll、pitch、yaw 转换为旋转矩阵。
Eigen::Matrix3d rpyDegToRot(const std::vector<double> &rpy_deg);
/// 将向量模长限制在给定上限内，方向保持不变。
Eigen::Vector3d limitVector(const Eigen::Vector3d &v, double max_norm);

class DogPriorMapEkfNode
{
public:
  /// 读取参数、初始化状态并建立 ROS 订阅发布关系。
  DogPriorMapEkfNode();

private:
  /// 优先从全局命名空间读取参数，再读取私有命名空间，均不存在时返回默认值。
  template <typename T>
  T getParam(const std::string &name, const T &default_value)
  {
    T value;
    if (nh_.getParam(name, value)) return value;
    if (pnh_.getParam(name, value)) return value;
    return default_value;
  }

  /// 接收 IMU 数据，完成初始化、状态传播和高频里程计发布。
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);
  /// 使用一次 IMU 测量传播姿态、速度、位置和误差状态协方差。
  void propagateImu(const Eigen::Vector3d &acc_m, const Eigen::Vector3d &gyr_m, double dt);
  void saveStateSnapshot(double stamp);
  void restoreStateSnapshot(const FilterStateSnapshot &snapshot);
  void pruneStateHistory(double current_stamp);
  bool findStateSnapshotAtOrBefore(double target_stamp,
                                   size_t &index,
                                   double &alignment_error) const;
  void eraseStateHistoryAfter(double stamp);

  /// 将位置和小角度姿态修正反馈到当前导航状态。
  void applyPoseCorrection(const Eigen::Vector3d &dp, const Eigen::Vector3d &dtheta);
  /// 将独立 NDT 节点输出作为低频外部观测融合到 EKF 状态中。
  void ndtObservationCallback(const nav_msgs::OdometryConstPtr &msg);
  /// 在调用方已持有 mutex_ 时执行一条 NDT 观测的完整校正流程。
  void processNdtObservationLocked(const nav_msgs::OdometryConstPtr &msg);
  /// 在 IMU 状态推进后按传感器时间处理已到达 watermark 的 NDT 观测。
  void processReadyDeferredNdtLocked();
  void writeDeferredDiagnostic(const std::string &event,
                              double ndt_stamp,
                              double state_now_stamp,
                              double future_lead_sec,
                              std::size_t queue_size,
                              double wait_ms,
                              const std::string &result);
  /// 发布当前里程计，并按配置同步发布路径、TF 和兼容话题。
  void publishState(const ros::Time &stamp, bool corrected);
  /// 向轨迹消息追加一个位姿，并限制轨迹缓存长度。
  void appendPath(nav_msgs::Path &path, const nav_msgs::Odometry &odom);
  bool shouldSamplePath(const ros::Time &stamp, double &last_sample_stamp) const;
  void publishPathsIfDue(const ros::Time &stamp);
  /// 按固定周期打印并记录各传感器和匹配模块的运行统计。
  void maybePrintRuntime(const ros::Time &stamp);
  void writeEkfPredictionLineage(const std::string &event_type,
                                 double event_ros_stamp,
                                 uint64_t publish_seq = 0,
                                 double ndt_measurement_stamp = std::numeric_limits<double>::quiet_NaN(),
                                 double state_now_before_ndt = std::numeric_limits<double>::quiet_NaN(),
                                 double state_now_after_replay = std::numeric_limits<double>::quiet_NaN(),
                                 double rollback_stamp = std::numeric_limits<double>::quiet_NaN(),
                                 size_t replay_imu_count = 0,
                                 double lag_ms = std::numeric_limits<double>::quiet_NaN(),
                                 double rewrite_translation_m = std::numeric_limits<double>::quiet_NaN(),
                                 double rewrite_rotation_deg = std::numeric_limits<double>::quiet_NaN(),
                                 double rewrite_velocity_mps = std::numeric_limits<double>::quiet_NaN());
  void writeOosmDiagnostic(double ndt_stamp,
                           double state_now_stamp,
                           double rollback_stamp,
                           double lag_sec,
                           double alignment_error_sec,
                           size_t replay_imu_count,
                           size_t state_history_count,
                           size_t imu_history_count,
                           const std::string &result);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_ndt_observation_;
  ros::Publisher pub_high_;
  ros::Publisher pub_imu_propagate_;
  ros::Publisher pub_corr_;
  ros::Publisher pub_path_high_;
  ros::Publisher pub_path_corr_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::mutex mutex_;
  bool has_last_imu_ = false;
  double last_imu_time_ = 0.0;
  bool has_state_stamp_ = false;
  double state_stamp_ = 0.0;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_topic_;
  std::string ndt_observation_topic_;
  std::string odom_high_rate_topic_;
  std::string imu_propagate_topic_;
  std::string odom_corrected_topic_;
  std::string path_high_rate_topic_;
  std::string path_corrected_topic_;

  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d g_;
  Matrix15d P_;

  double gravity_norm_ = 9.80665;
  double max_imu_dt_ = 0.05;
  bool publish_high_rate_ = true;
  bool publish_imu_propagate_alias_ = true;
  bool use_acc_for_position_ = false;
  double velocity_damping_ = 0.98;
  bool initialize_gravity_from_imu_ = true;
  bool gravity_initialized_ = false;
  int init_imu_samples_ = 200;
  int imu_init_count_ = 0;
  Eigen::Vector3d imu_acc_sum_ = Eigen::Vector3d::Zero();
  bool continuous_gravity_correction_enable_ = true;
  double gravity_correction_expected_acc_norm_ = 1.0;
  double gravity_correction_gain_ = 0.01;
  double gravity_correction_max_angle_ = 0.5 * M_PI / 180.0;
  double gravity_correction_acc_tolerance_ = 1.5;
  double gravity_correction_gyro_max_ = 0.8;
  double acc_noise_ = 2.0;
  double gyro_noise_ = 0.1;
  double acc_bias_noise_ = 0.0001;
  double gyro_bias_noise_ = 0.0001;
  std::deque<ImuSample> imu_history_;
  double imu_history_keep_sec_ = 2.0;
  StateHistory state_history_;

  bool ndt_observation_enable_ = false;
  bool oosm_enable_ = false;
  double oosm_max_alignment_sec_ = 0.02;
  double ndt_observation_apply_ratio_ = 0.8;
  double ndt_observation_z_apply_ratio_ = 1.0;
  double ndt_observation_roll_pitch_apply_ratio_ = 1.0;
  double ndt_observation_max_translation_correction_ = 1.0;
  double ndt_observation_max_rotation_correction_ = 5.0 * M_PI / 180.0;
  double ndt_observation_velocity_blend_ = 0.6;
  bool future_deferral_enable_ = false;
  double future_deferral_max_sec_ = 0.010;
  std::size_t future_deferral_max_queue_ = 20;

  struct DeferredNdtObservation
  {
    nav_msgs::OdometryConstPtr msg;
    double stamp = std::numeric_limits<double>::quiet_NaN();
    double received_state_stamp = std::numeric_limits<double>::quiet_NaN();
    double future_lead_sec = std::numeric_limits<double>::quiet_NaN();
    double received_wall_sec = 0.0;
  };
  std::deque<DeferredNdtObservation> deferred_ndt_observations_;
  uint64_t deferred_received_count_ = 0;
  uint64_t deferred_processed_count_ = 0;
  uint64_t deferred_over_limit_count_ = 0;
  uint64_t deferred_queue_full_count_ = 0;
  std::size_t max_deferred_queue_size_ = 0;
  std::string deferred_csv_path_;
  std::ofstream deferred_csv_;
  double last_ndt_observation_time_ = 0.0;
  Eigen::Vector3d last_ndt_observation_p_map_ = Eigen::Vector3d::Zero();

  int path_max_length_ = 5000;
  double path_sample_rate_hz_ = 2.0;
  double path_publish_rate_hz_ = 1.0;
  double last_path_high_sample_stamp_ = -1.0;
  double last_path_corr_sample_stamp_ = -1.0;
  double last_path_publish_stamp_ = -1.0;
  bool publish_path_ = false;
  bool publish_tf_ = true;
  bool print_debug_ = true;
  double debug_interval_sec_ = 2.0;
  std::string runtime_csv_path_;
  std::ofstream runtime_csv_;
  std::string ekf_prediction_diagnostics_csv_path_;
  std::ofstream ekf_prediction_diagnostics_csv_;
  uint64_t ekf_prediction_event_index_ = 0;
  uint64_t ekf_state_revision_ = 0;
  uint64_t high_rate_publish_seq_ = 0;
  std::string oosm_csv_path_;
  std::ofstream oosm_csv_;
  uint64_t oosm_frame_index_ = 0;
  bool ekf_determinism_diagnostic_enable_ = false;
  std::string ekf_ndt_feedback_csv_path_;
  std::ofstream ekf_ndt_feedback_csv_;
  uint64_t ekf_ndt_feedback_frame_index_ = 0;
  double last_debug_time_ = 0.0;
  uint64_t imu_msg_count_ = 0;
  uint64_t lidar_update_ok_count_ = 0;
  uint64_t lidar_update_fail_count_ = 0;
  uint64_t last_debug_imu_count_ = 0;
  uint64_t last_debug_update_ok_count_ = 0;
  double lidar_update_time_sum_ms_ = 0.0;
  double lidar_update_time_max_ms_ = 0.0;
  double icp_update_time_sum_ms_ = 0.0;
  double icp_update_time_max_ms_ = 0.0;
  uint64_t icp_update_ok_count_ = 0;
  uint64_t icp_update_fail_count_ = 0;
  int last_used_points_ = 0;
  double last_mean_residual_ = 0.0;

  nav_msgs::Path path_high_;
  nav_msgs::Path path_corr_;
};

}  // namespace dog_prior_map_localization
