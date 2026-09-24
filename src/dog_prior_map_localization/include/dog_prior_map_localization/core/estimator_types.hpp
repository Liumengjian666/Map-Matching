#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dog_prior_map_localization
{

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Vector15d = Eigen::Matrix<double, 15, 1>;
using Matrix3x15d = Eigen::Matrix<double, 3, 15>;

// IMU sample used by the estimator's existing sensor-time history.
// The acceleration and angular-rate vectors remain in the current IMU/body
// convention; no frame conversion is introduced by this type extraction.
struct ImuSample
{
  // IMU sample timestamp, in seconds.
  double stamp = 0.0;
  // Raw accelerometer and gyroscope measurements in the IMU/body frame.
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

// Snapshot of the estimator's existing nominal state and covariance.
// The state is p, v, R, ba, bg, P.  The associated error-state dimension is
// [p, v, theta, ba, bg] = 15 DoF, following the current estimator convention.
struct FilterStateSnapshot
{
  double stamp = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  // Raw IMU sensor sample at exactly `stamp` (not an interval average).
  Eigen::Vector3d acc_measurement = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_measurement = Eigen::Vector3d::Zero();
  // Input used for the completed interval ending at `stamp`. These fields are
  // diagnostic/replay metadata; the next interval is formed from raw samples.
  Eigen::Vector3d interval_acc_input = Eigen::Vector3d::Zero();
  Eigen::Vector3d interval_gyro_input = Eigen::Vector3d::Zero();
  bool has_interval_input = false;
  Eigen::Vector3d acc_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_unbiased = Eigen::Vector3d::Zero();
  Matrix15d P = Matrix15d::Identity();
};

}  // namespace dog_prior_map_localization
