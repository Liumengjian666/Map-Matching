#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

inline Eigen::Matrix3d lidarSkew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

inline Eigen::Vector3d lidarSo3Log(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond q(rotation);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sine_half = q.vec().norm();
  if (sine_half < 1e-12) return 2.0 * q.vec();
  const double angle = 2.0 * std::atan2(
      sine_half, std::clamp(q.w(), -1.0, 1.0));
  return q.vec() * (angle / sine_half);
}

inline bool lidarSo3LeftJacobianInverse(const Eigen::Vector3d& phi,
                                        Eigen::Matrix3d* result,
                                        std::string* reason = nullptr) {
  if (reason) reason->clear();
  if (!result || !phi.allFinite()) {
    if (reason) *reason = result ? "NONFINITE_ROTATION_RESIDUAL" :
                                  "NULL_SO3_JACOBIAN_OUTPUT";
    return false;
  }
  const double theta = phi.norm();
  if (!std::isfinite(theta) || theta > M_PI - 1e-4) {
    if (reason) *reason = "ROTATION_RESIDUAL_NEAR_PI";
    return false;
  }
  const Eigen::Matrix3d K = lidarSkew(phi);
  const Eigen::Matrix3d K2 = K * K;
  if (theta < 1e-4) {
    *result = Eigen::Matrix3d::Identity() - 0.5 * K +
        (1.0 / 12.0 + theta * theta / 720.0) * K2;
  } else {
    const double half = 0.5 * theta;
    const double sine = std::sin(half);
    if (std::abs(sine) < 1e-12) {
      if (reason) *reason = "INVALID_ROTATION_RESIDUAL_HALF_ANGLE";
      return false;
    }
    const double coefficient = 1.0 / (theta * theta) -
        (std::cos(half) / sine) / (2.0 * theta);
    *result = Eigen::Matrix3d::Identity() - 0.5 * K + coefficient * K2;
  }
  if (!result->allFinite()) {
    if (reason) *reason = "NONFINITE_SO3_LEFT_JACOBIAN_INVERSE";
    return false;
  }
  return true;
}

// Exact map-spatial normalized NDT perturbation -> window residual Jacobian.
// Columns are [left map rotation, scaled map translation].
inline bool normalizedLidarResidualJacobian(
    const WindowState& state, const Eigen::Matrix3d& measured_map_R_imu,
    const Eigen::Vector3d& imu_T_lidar_translation,
    double translation_length_scale_m, Matrix6d* jacobian,
    std::string* reason = nullptr) {
  if (reason) reason->clear();
  if (!jacobian || !state.rotation.allFinite() ||
      !measured_map_R_imu.allFinite() ||
      !imu_T_lidar_translation.allFinite() ||
      !std::isfinite(translation_length_scale_m) ||
      translation_length_scale_m <= 0.0) {
    if (reason) *reason = "INVALID_EXACT_RESIDUAL_JACOBIAN_INPUT";
    return false;
  }
  const Eigen::Vector3d phi =
      lidarSo3Log(state.rotation.transpose() * measured_map_R_imu);
  Eigen::Matrix3d left_inverse;
  if (!lidarSo3LeftJacobianInverse(phi, &left_inverse, reason)) return false;
  jacobian->setZero();
  jacobian->block<3, 3>(0, 0) =
      lidarSkew(measured_map_R_imu * imu_T_lidar_translation);
  jacobian->block<3, 3>(0, 3) =
      translation_length_scale_m * Eigen::Matrix3d::Identity();
  jacobian->block<3, 3>(3, 0) = left_inverse * state.rotation.transpose();
  return jacobian->allFinite();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
