#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace dog_prior_map_localization {

inline bool rigidMapPose(const Eigen::Matrix4d& pose) {
  if (!pose.allFinite() ||
      (pose.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-6)
    return false;
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-4 &&
      std::abs(rotation.determinant() - 1.0) <= 1e-4;
}

// Causally advance map_T_lidar from a stamped EKF sample using the child-frame
// Odometry twist over a short, bounded age. For constant body twist the exact
// relative motion is right-multiplied as Exp_SE3([v_lidar, omega_lidar] * dt).
inline bool predictMapTLidarWithBodyTwist(const Eigen::Matrix4d& sample_map_T_lidar,
    const Eigen::Vector3d& linear_velocity_lidar,
    const Eigen::Vector3d& angular_velocity_lidar, double dt,
    double maximum_age_sec, Eigen::Matrix4d* predicted_map_T_lidar) {
  if (!predicted_map_T_lidar || !rigidMapPose(sample_map_T_lidar) ||
      !linear_velocity_lidar.allFinite() || !angular_velocity_lidar.allFinite() ||
      !std::isfinite(dt) || !std::isfinite(maximum_age_sec) || dt < 0.0 ||
      maximum_age_sec < 0.0 || dt > maximum_age_sec)
    return false;

  const Eigen::Vector3d phi = angular_velocity_lidar * dt;
  Eigen::Matrix3d phi_hat;
  phi_hat << 0.0, -phi.z(), phi.y(),
             phi.z(), 0.0, -phi.x(),
             -phi.y(), phi.x(), 0.0;
  const Eigen::Matrix3d phi_hat_squared = phi_hat * phi_hat;
  const double angle = phi.norm();
  Eigen::Matrix3d delta_rotation;
  Eigen::Matrix3d left_jacobian;
  if (angle < 1e-6) {
    delta_rotation = Eigen::Matrix3d::Identity() + phi_hat + 0.5 * phi_hat_squared;
    left_jacobian = Eigen::Matrix3d::Identity() + 0.5 * phi_hat +
        (1.0 / 6.0) * phi_hat_squared;
  } else {
    delta_rotation = Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
    left_jacobian = Eigen::Matrix3d::Identity() +
        ((1.0 - std::cos(angle)) / (angle * angle)) * phi_hat +
        ((angle - std::sin(angle)) / (angle * angle * angle)) * phi_hat_squared;
  }

  Eigen::Matrix4d body_delta = Eigen::Matrix4d::Identity();
  body_delta.block<3, 3>(0, 0) = delta_rotation;
  body_delta.block<3, 1>(0, 3) = left_jacobian * (linear_velocity_lidar * dt);
  *predicted_map_T_lidar = sample_map_T_lidar * body_delta;
  return rigidMapPose(*predicted_map_T_lidar);
}

}  // namespace dog_prior_map_localization
