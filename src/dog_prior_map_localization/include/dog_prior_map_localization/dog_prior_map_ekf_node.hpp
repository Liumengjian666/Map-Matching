#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>

namespace dog_prior_map_localization
{

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix3x15d = Eigen::Matrix<double, 3, 15>;

struct ImuSample
{
  // IMU 样本时间戳，单位为秒。
  double stamp = 0.0;
  // 加速度计和陀螺仪原始测量，均位于 IMU/机体系。
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

struct FilterStateSnapshot
{
  double stamp = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Matrix15d P = Matrix15d::Identity();
};

/// 将三维向量转换为叉乘对应的反对称矩阵。
Eigen::Matrix3d skew(const Eigen::Vector3d &v);
/// 将角度制的 roll、pitch、yaw 转换为旋转矩阵。
Eigen::Matrix3d rpyDegToRot(const std::vector<double> &rpy_deg);
/// 将向量模长限制在给定上限内，方向保持不变。
Eigen::Vector3d limitVector(const Eigen::Vector3d &v, double max_norm);

class DogPriorMapEkfNode
{
public:
  // Direction-selective fusion and the hysteresis state machine are opt-in.
  // The default configuration remains the existing full NDT + IMU + EKF path.
  enum class LocalizationMode
  {
    NORMAL,
    LIDAR_DEGRADED,
    VISION_ASSISTED,
    BOTH_DEGRADED,
    RECOVERY
  };

  /// 读取参数、初始化状态与地图，并建立 ROS 订阅发布关系。
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

  /// 读取 double 数组参数，读取失败时返回调用者提供的默认数组。
  std::vector<double> getParamVec(const std::string &name, const std::vector<double> &default_value);

  /// 加载并预处理先验点云地图，同时建立最近邻搜索树。
  void loadPriorMap();
  /// 从 PCD 文件中兼容读取 XYZ/XYZI 点并统一转换为 XYZ 点云。
  pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcdXyzOnly(const std::string &pcd_path);

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

  /// 接收 Livox 自定义点云并按配置决定是否执行帧内去畸变。
  void livoxCallback(const livox_ros_driver2::CustomMsgConstPtr &msg);
  /// 利用 IMU 历史将 Livox 一帧内各点补偿到统一的帧尾时刻。
  pcl::PointCloud<pcl::PointXYZ>::Ptr deskewLivoxCloud(const livox_ros_driver2::CustomMsgConstPtr &msg);
  /// 对指定时间区间的陀螺仪数据积分，得到相对旋转。
  Eigen::Matrix3d integrateImuRotation(double t0, double t1) const;
  /// 对指定时间区间的 IMU 数据积分，得到相对旋转和平移。
  bool integrateImuDelta(double t0, double t1, Eigen::Matrix3d &R_delta, Eigen::Vector3d &p_delta) const;
  /// 接收标准 PointCloud2 点云并转入统一 LiDAR 处理流程。
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  /// 调度点云预处理、局部地图构建、匹配更新和结果发布。
  void handleLidarCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_lidar, const ros::Time &stamp);
  /// 将雷达点转换到机体系，并执行范围过滤、降采样和离群点剔除。
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_lidar);
  /// 使用点到点或点到平面残差迭代估计当前帧的六自由度位姿修正量。
  bool lidarMapUpdate(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &match_map,
                      int &used,
                      double &mean_residual);
  /// 按当前位置从先验地图截取用于本帧匹配的局部子地图。
  pcl::PointCloud<pcl::PointXYZ>::Ptr buildLocalSubmap();
  /// 以当前状态为初值执行 NDT 精配准，并按门限决定是否接收结果。
  bool runNdtRefinement(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map);
  /// 将位置和小角度姿态修正反馈到当前导航状态。
  void applyPoseCorrection(const Eigen::Vector3d &dp, const Eigen::Vector3d &dtheta);
  /// 根据匹配信息矩阵与地图几何分布更新 LiDAR 退化状态。
  bool updateLidarDegeneracyStatus(const Eigen::Matrix<double, 6, 6> &information_matrix,
                                   double geometry_degeneracy_score);

  /// 评估图像质量、跟踪角点，并在 LiDAR 退化时触发视觉航向约束。
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  /// 从相邻图像的特征运动估计相对旋转，并仅反馈受限的 yaw 修正。
  bool applyVisualYawCorrection(const cv::Mat &prev_gray,
                                const cv::Mat &curr_gray,
                                const std::vector<cv::Point2f> &prev_pts,
                                const std::vector<cv::Point2f> &curr_pts,
                                double weight_scale,
                                bool apply_correction);
  /// 仅用于诊断：比较视觉相对旋转与同一图像时间窗内的陀螺仪积分。
  void updateVisualImuDiagnostic(const ros::Time &stamp);
  /// 将独立 NDT 节点输出作为低频外部观测融合到 EKF 状态中。
  void ndtObservationCallback(const nav_msgs::OdometryConstPtr &msg);
  void lidarDegeneracyCallback(const std_msgs::Float64ConstPtr &msg);
  /// 接收 NDT 发布的 6DoF 信息矩阵特征系统，并构造退化/可靠子空间。
  void lidarInformationCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
  /// 带滞回地更新 NORMAL/DEGRADED/RECOVERY 状态；默认只记录，不参与更新。
  void updateLocalizationMode(bool lidar_event, bool visual_event);
  /// 根据特征值比例构造 P_d 与 P_r，状态顺序为 [x,y,z,roll,pitch,yaw]。
  void rebuildDirectionalProjectors();

  /// 发布当前里程计，并按配置同步发布路径、TF 和兼容话题。
  void publishState(const ros::Time &stamp, bool corrected);
  /// 将预处理后的机体系点云转换到地图系并发布调试点云。
  void publishFilteredCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_body,
                            const ros::Time &stamp);
  /// 发布本次地图匹配的耗时、点数、残差和收敛状态。
  void publishDiagnostics(const ros::Time &stamp,
                          bool converged,
                          double match_time_ms,
                          int scan_points,
                          int map_points,
                          double score);
  /// 向轨迹消息追加一个位姿，并限制轨迹缓存长度。
  void appendPath(nav_msgs::Path &path, const nav_msgs::Odometry &odom);
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
  ros::Subscriber sub_livox_;
  ros::Subscriber sub_pc2_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_ndt_observation_;
  ros::Subscriber sub_lidar_degeneracy_;
  ros::Subscriber sub_lidar_information_;
  ros::Publisher pub_high_;
  ros::Publisher pub_imu_propagate_;
  ros::Publisher pub_corr_;
  ros::Publisher pub_path_high_;
  ros::Publisher pub_path_corr_;
  ros::Publisher pub_filtered_points_;
  ros::Publisher pub_prior_map_;
  ros::Publisher pub_diagnostics_;
  tf::TransformBroadcaster tf_broadcaster_;

  std::mutex mutex_;
  bool has_last_imu_ = false;
  double last_imu_time_ = 0.0;
  bool has_state_stamp_ = false;
  double state_stamp_ = 0.0;
  int scan_count_ = 0;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_topic_;
  std::string lidar_topic_;
  std::string lidar_msg_type_;
  std::string image_topic_;
  std::string ndt_observation_topic_;
  std::string lidar_degeneracy_topic_;
  std::string lidar_information_topic_;
  std::string odom_high_rate_topic_;
  std::string imu_propagate_topic_;
  std::string odom_corrected_topic_;
  std::string path_high_rate_topic_;
  std::string path_corrected_topic_;
  std::string filtered_points_topic_;
  std::string prior_map_topic_;
  std::string diagnostics_topic_;

  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d ba_;
  Eigen::Vector3d bg_;
  Eigen::Vector3d g_;
  Matrix15d P_;

  Eigen::Vector3d T_base_lidar_;
  Eigen::Matrix3d R_base_lidar_;
  Eigen::Vector3d T_base_camera_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_base_camera_ = Eigen::Matrix3d::Identity();

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
  bool lidar_deskew_enable_ = true;
  bool lidar_deskew_translation_enable_ = true;
  double lidar_offset_time_scale_ = 1e-9;
  double imu_history_keep_sec_ = 2.0;
  std::deque<FilterStateSnapshot> state_history_;

  bool ndt_observation_enable_ = false;
  bool oosm_enable_ = false;
  double oosm_max_alignment_sec_ = 0.02;
  double ndt_observation_apply_ratio_ = 0.8;
  double ndt_observation_z_apply_ratio_ = 1.0;
  double ndt_observation_roll_pitch_apply_ratio_ = 1.0;
  double ndt_observation_max_translation_correction_ = 1.0;
  double ndt_observation_max_rotation_correction_ = 5.0 * M_PI / 180.0;
  double ndt_observation_velocity_blend_ = 0.6;
  double last_ndt_observation_time_ = 0.0;
  Eigen::Vector3d last_ndt_observation_p_map_ = Eigen::Vector3d::Zero();

  // Direction-selective fusion is deliberately opt-in.  Until a replay
  // validates the eigensystem convention and visual relative motion, the
  // existing full NDT correction remains the only active measurement path.
  bool directional_fusion_enable_ = false;
  bool state_machine_enable_ = false;
  double directional_eigen_ratio_ = 0.03;
  int lidar_degraded_enter_frames_ = 5;
  int visual_assisted_enter_frames_ = 3;
  int both_degraded_enter_frames_ = 3;
  int recovery_exit_frames_ = 5;
  double recovery_weak_weight_start_ = 0.0;
  bool skip_updates_when_both_degraded_ = true;
  bool legacy_visual_yaw_enable_ = false;
  double lidar_information_max_age_sec_ = 0.05;
  // Internal information coordinates are [m,m,m,scale*m,scale*m,scale*m]
  // so translation and rotation eigenvalues are comparable.  The public
  // telemetry still reports roll/pitch/yaw components in radians.
  double information_rotation_scale_m_ = 1.0;
  bool local_vio_diagnostic_enable_ = true;
  // Experimental diagnostic only: use the norm of short-window IMU
  // preintegration to scale the monocular essential-matrix translation
  // direction.  This is not connected to the EKF update path.
  bool local_vio_metric_enable_ = false;
  bool visual_imu_consistency_gate_enable_ = false;
  double visual_imu_consistency_max_deg_ = 20.0;
  bool lidar_directional_valid_ = false;
  bool lidar_directional_degenerate_ = false;
  bool lidar_information_received_ = false;
  bool lidar_information_stale_ = true;
  bool lidar_projector_valid_ = false;
  double lidar_information_stamp_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 6, 6> lidar_information_eigenvectors_ =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> lidar_degenerate_projector_ =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> lidar_reliable_projector_ =
      Eigen::Matrix<double, 6, 6>::Identity();
  int lidar_degraded_count_ = 0;
  int lidar_recovery_count_ = 0;
  int visual_good_count_ = 0;
  int visual_bad_count_ = 0;

  bool lidar_enable_ = true;
  bool prior_map_update_enable_ = true;
  std::string registration_method_ = "point_to_point";
  int update_every_n_scans_ = 1;
  double scan_voxel_size_ = 0.25;
  int max_scan_points_ = 2500;
  double scan_min_range_ = 0.5;
  double scan_max_range_ = 80.0;
  double scan_min_z_ = -std::numeric_limits<double>::infinity();
  double scan_max_z_ = std::numeric_limits<double>::infinity();
  bool scan_radius_outlier_enable_ = false;
  double scan_radius_outlier_radius_ = 0.35;
  int scan_radius_outlier_min_neighbors_ = 2;
  double max_match_distance_ = 1.0;
  int min_effective_points_ = 80;
  double point_noise_ = 0.20;
  double hybrid_point_noise_ = 0.60;
  double plane_noise_ = 0.12;
  int plane_neighbor_k_ = 5;
  double plane_min_eigen_ratio_ = 0.08;
  double fastlio_plane_threshold_ = 0.10;
  double fastlio_residual_gate_ = 0.90;
  bool fastlivo_outlier_reject_enable_ = true;
  double plane_radius_gate_scale_ = 3.0;
  double plane_sigma_gate_ = 3.0;
  double plane_min_radius_ = 0.05;
  double plane_min_sigma_ = 0.03;
  double range_noise_per_meter_ = 0.003;
  bool fastlivo_body_cov_enable_ = true;
  double lidar_depth_noise_ = 0.05;
  double lidar_beam_noise_deg_ = 0.02;
  bool horizontal_plane_z_observation_enable_ = true;
  double horizontal_plane_normal_z_min_ = 0.85;
  double horizontal_plane_weight_scale_ = 2.0;
  double horizontal_plane_residual_gate_ = 0.35;
  double huber_threshold_ = 0.6;
  int max_iterations_ = 3;
  double max_translation_update_ = 0.25;
  double max_rotation_update_ = 3.0 * M_PI / 180.0;
  double max_update_time_ms_ = 25.0;
  bool match_accept_gate_enable_ = true;
  double match_accept_max_mean_residual_ = 0.20;
  double match_accept_max_residual_increase_ratio_ = 1.02;
  double match_accept_min_improvement_ratio_ = 0.0;
  double match_accept_min_effective_ratio_ = 0.10;
  double match_accept_max_translation_ = 0.35;
  double match_accept_max_rotation_ = 2.0 * M_PI / 180.0;
  double match_accept_degenerate_max_translation_ = 0.12;
  double match_accept_degenerate_max_rotation_ = 0.8 * M_PI / 180.0;
  double local_radius_ = 18.0;
  double local_radius_max_ = 18.0;
  double local_radius_step_ = 2.0;
  bool local_submap_enable_ = true;
  int local_submap_max_points_ = 12000;
  int local_submap_min_points_ = 500;
  double ndt_source_voxel_size_ = 0.35;
  double ndt_target_voxel_size_ = 0.30;
  int ndt_max_source_points_ = 900;
  int ndt_max_target_points_ = 25000;
  int ndt_max_iterations_ = 15;
  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_transformation_epsilon_ = 0.01;
  double ndt_max_fitness_score_ = 0.6;
  double ndt_accept_max_translation_ = 0.6;
  double ndt_accept_max_rotation_ = 8.0 * M_PI / 180.0;
  bool ndt_absolute_pose_mode_ = false;
  bool ndt_use_full_map_target_ = false;
  bool ndt_has_previous_pose_ = false;
  bool ndt_full_map_target_ready_ = false;
  Eigen::Matrix4d ndt_previous_pose_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d ndt_delta_pose_ = Eigen::Matrix4d::Identity();
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_full_map_;
  bool degeneracy_check_enable_ = true;
  double degeneracy_min_eigenvalue_ = 1e-3;
  double degeneracy_max_condition_number_ = 1e5;
  double degeneracy_geometry_ratio_ = 0.08;
  bool degeneracy_project_update_enable_ = true;
  double degeneracy_project_eigen_ratio_ = 0.03;
  double degeneracy_project_min_scale_ = 0.10;
  bool lidar_degenerate_ = false;
  double lidar_degeneracy_score_ = 0.0;
  bool lidar_information_valid_ = false;
  bool lidar_information_degenerate_ = false;
  double lidar_information_condition_ = 1.0;
  double lidar_geometry_degeneracy_score_ = 0.0;
  bool lidar_geometry_degeneracy_valid_ = false;
  Eigen::Matrix<double, 6, 1> lidar_information_eigenvalues_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> lidar_information_weak_eigenvector_ = Eigen::Matrix<double, 6, 1>::Zero();

  LocalizationMode localization_mode_ = LocalizationMode::NORMAL;

  bool camera_enable_ = true;
  bool visual_feature_update_enable_ = true;
  double max_over_exposure_ratio_ = 0.25;
  double max_under_exposure_ratio_ = 0.35;
  int max_features_ = 300;
  int min_tracked_features_ = 40;
  double visual_max_flow_residual_px_ = 3.0;
  double min_feature_ratio_ = 0.15;
  double max_visual_yaw_update_ = 0.5 * M_PI / 180.0;
  double good_image_weight_scale_ = 1.0;
  double bad_image_weight_scale_ = 0.4;
  double visual_degenerate_weight_scale_ = 2.0;
  double min_degeneracy_score_for_visual_ = 0.15;
  bool camera_intrinsic_valid_ = false;
  double cam_fx_ = 0.0;
  double cam_fy_ = 0.0;
  double cam_cx_ = 0.0;
  double cam_cy_ = 0.0;
  bool image_quality_good_ = true;
  double last_feature_ratio_ = 0.0;
  double visual_constraint_weight_scale_ = 1.0;
  double visual_update_time_sum_ms_ = 0.0;
  double visual_update_time_max_ms_ = 0.0;
  uint64_t image_msg_count_ = 0;
  uint64_t visual_update_ok_count_ = 0;
  uint64_t visual_update_fail_count_ = 0;
  cv::Mat last_gray_;
  std::vector<cv::Point2f> last_features_;
  bool has_last_image_pose_ = false;
  Eigen::Matrix3d last_image_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_image_p_ = Eigen::Vector3d::Zero();
  ros::Time last_image_stamp_;
  bool last_visual_tracking_good_ = false;
  Eigen::Matrix3d last_visual_relative_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d last_visual_translation_direction_base_ = Eigen::Vector3d::Zero();
  bool last_visual_translation_direction_valid_ = false;
  double last_visual_translation_scale_m_ = std::numeric_limits<double>::quiet_NaN();
  double last_visual_imu_rotation_residual_deg_ = std::numeric_limits<double>::quiet_NaN();
  bool last_visual_imu_rotation_valid_ = false;
  Eigen::Matrix<double, 6, 1> last_visual_covariance_diag_ =
      Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
  int last_visual_feature_count_ = 0;
  int last_visual_tracked_count_ = 0;
  int last_visual_inlier_count_ = 0;
  double last_visual_flow_residual_px_ = std::numeric_limits<double>::quiet_NaN();
  bool last_visual_flow_residual_valid_ = false;
  double last_visual_reprojection_error_px_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 6, 1> last_visual_relative_pose_ =
      Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
  bool last_visual_relative_pose_valid_ = false;
  bool last_visual_metric_translation_valid_ = false;
  bool last_visual_reprojection_valid_ = false;
  bool last_visual_covariance_valid_ = false;
  std::string last_visual_update_reason_ = "not_initialized";

  int path_max_length_ = 5000;
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
  uint64_t lidar_msg_count_ = 0;
  uint64_t lidar_update_ok_count_ = 0;
  uint64_t lidar_update_fail_count_ = 0;
  uint64_t last_debug_imu_count_ = 0;
  uint64_t last_debug_lidar_count_ = 0;
  uint64_t last_debug_update_ok_count_ = 0;
  uint64_t last_debug_image_count_ = 0;
  double lidar_update_time_sum_ms_ = 0.0;
  double lidar_update_time_max_ms_ = 0.0;
  double icp_update_time_sum_ms_ = 0.0;
  double icp_update_time_max_ms_ = 0.0;
  uint64_t icp_update_ok_count_ = 0;
  uint64_t icp_update_fail_count_ = 0;
  int last_used_points_ = 0;
  double last_mean_residual_ = 0.0;
  double last_registration_score_ = 0.0;
  int last_map_points_ = 0;
  bool publish_filtered_points_ = true;
  bool publish_diagnostics_ = true;

  nav_msgs::Path path_high_;
  nav_msgs::Path path_corr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_kdtree_;
};

}  // namespace dog_prior_map_localization
