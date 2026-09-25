#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct Pose3d {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct ImuSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp_ns = 0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct ImuPoseSample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp_ns = 0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d world_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d unbiased_gyro = Eigen::Vector3d::Zero();
};

struct RuntimeParameters {
  int static_init_samples = 200;
  double gravity_mps2 = 9.809;
  double max_static_gyro_std_rad_s = 0.05;
  double max_static_accel_std_m_s2 = 0.50;
  Eigen::Vector3d initial_accel_bias = Eigen::Vector3d::Zero();
  double gyro_noise_std_rad_s = 0.1;
  double accel_noise_std_m_s2 = 2.0;
  double gyro_bias_rw_std_rad_s2 = 0.0001;
  double accel_bias_rw_std_m_s3 = 0.0001;
  double pose_position_sigma_m = 0.20;
  double pose_rotation_sigma_rad = 0.10;
};

struct FilterSnapshot {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp_ns = 0;
  Pose3d map_T_imu;
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Matrix3d T_imu_lidar_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d T_imu_lidar_translation = Eigen::Vector3d::Zero();
  Eigen::MatrixXd covariance;
};

struct PoseCorrectionDelta {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector2d gravity_tangent = Eigen::Vector2d::Zero();
};

enum class RuntimeDisposition : uint8_t {
  SUCCESS = 0,
  REJECT_INSUFFICIENT_POINTS = 1,
  REJECT_NOT_CONVERGED = 2,
  FATAL = 255
};

struct RuntimeCounters {
  uint64_t last_committed_transaction = 0;
  uint64_t prediction_only_commits = 0;
  uint64_t measurement_updates = 0;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
