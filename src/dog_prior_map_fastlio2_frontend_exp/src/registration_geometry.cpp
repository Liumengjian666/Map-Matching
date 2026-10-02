#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

}  // namespace

Eigen::Matrix3d skew3(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond q(rotation);
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sine_half = q.vec().norm();
  if (sine_half < 1e-12) return 2.0 * q.vec();
  const double angle = 2.0 * std::atan2(
      sine_half, std::max(-1.0, std::min(1.0, q.w())));
  return q.vec() * (angle / sine_half);
}

bool so3LeftJacobianInverse(const Eigen::Vector3d& phi,
                            Eigen::Matrix3d* result,
                            std::string* reason) {
  if (reason) reason->clear();
  if (!result) return fail(reason, "NULL_SO3_JACOBIAN_OUTPUT");
  if (!phi.allFinite()) return fail(reason, "NONFINITE_ROTATION_RESIDUAL");
  const double theta = phi.norm();
  if (!std::isfinite(theta))
    return fail(reason, "NONFINITE_ROTATION_RESIDUAL");
  if (theta > M_PI - 1e-4)
    return fail(reason, "ROTATION_RESIDUAL_NEAR_PI");
  const Eigen::Matrix3d K = skew3(phi);
  const Eigen::Matrix3d K2 = K * K;
  if (theta < 1e-4) {
    *result = Eigen::Matrix3d::Identity() - 0.5 * K +
        (1.0 / 12.0 + theta * theta / 720.0) * K2;
  } else {
    const double half = 0.5 * theta;
    const double sine = std::sin(half);
    if (!std::isfinite(sine) || std::abs(sine) < 1e-12)
      return fail(reason, "INVALID_ROTATION_RESIDUAL_HALF_ANGLE");
    const double cotangent = std::cos(half) / sine;
    const double coefficient = 1.0 / (theta * theta) -
        cotangent / (2.0 * theta);
    *result = Eigen::Matrix3d::Identity() - 0.5 * K + coefficient * K2;
  }
  if (!result->allFinite())
    return fail(reason, "NONFINITE_SO3_LEFT_JACOBIAN_INVERSE");
  return true;
}

bool normalizedRegistrationToPoseResidualJacobian(
    const Eigen::Matrix3d& predicted_map_R_imu,
    const Eigen::Matrix3d& measured_map_R_imu,
    const Eigen::Vector3d& imu_T_lidar_translation,
    double translation_length_scale_m,
    Eigen::Matrix<double, 6, 6>* jacobian,
    std::string* reason) {
  if (reason) reason->clear();
  if (!jacobian || !predicted_map_R_imu.allFinite() ||
      !measured_map_R_imu.allFinite() ||
      !imu_T_lidar_translation.allFinite() ||
      !std::isfinite(translation_length_scale_m) ||
      translation_length_scale_m <= 0.0)
    return fail(reason, "INVALID_EXACT_RESIDUAL_JACOBIAN_INPUT");
  const Eigen::Vector3d phi =
      so3Log(predicted_map_R_imu.transpose() * measured_map_R_imu);
  Eigen::Matrix3d left_inverse;
  if (!so3LeftJacobianInverse(phi, &left_inverse, reason)) return false;
  jacobian->setZero();
  jacobian->block<3, 3>(0, 0) =
      skew3(measured_map_R_imu * imu_T_lidar_translation);
  jacobian->block<3, 3>(0, 3) =
      translation_length_scale_m * Eigen::Matrix3d::Identity();
  jacobian->block<3, 3>(3, 0) =
      left_inverse * predicted_map_R_imu.transpose();
  return jacobian->allFinite();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
