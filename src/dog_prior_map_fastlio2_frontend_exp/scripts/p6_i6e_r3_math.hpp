#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>

namespace p6_i6e_r3 {

using dog_prior_map_fastlio2_frontend_exp::Pose3d;

inline Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
            vector.z(), 0.0, -vector.x(),
            -vector.y(), vector.x(), 0.0;
  return result;
}

inline Eigen::Matrix<double, 6, 1> poseResidual(
    const Pose3d& prior, const Pose3d& measurement) {
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = measurement.position - prior.position;
  // Keep the legacy operand order and normalization sequence exactly.  The
  // filter state is already normalized; normalizing the prior once more here
  // perturbs finite-difference diagnostics at the last few printed digits.
  Eigen::Quaterniond error = (prior.orientation.conjugate() *
                              measurement.orientation.normalized()).normalized();
  if (error.w() < 0.0) error.coeffs() *= -1.0;
  const Eigen::AngleAxisd angle_axis(error);
  residual.tail<3>() = angle_axis.axis() * angle_axis.angle();
  return residual;
}

inline bool finitePose(const Pose3d& pose) {
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
      std::isfinite(pose.orientation.norm()) && pose.orientation.norm() > 1e-12;
}

inline bool normalizedLidarToExactResidualJacobianFromImuMeasurement(
    const Pose3d& prior_map_T_imu, const Pose3d& nominal_map_T_imu,
    const Pose3d& T_imu_lidar, double length_scale_m,
    Eigen::Matrix<double, 6, 6>* jacobian, std::string* reason = nullptr) {
  if (reason) reason->clear();
  const auto reject = [reason](const char* message) {
    if (reason) *reason = message;
    return false;
  };
  if (!jacobian) return reject("NULL_EXACT_RESIDUAL_JACOBIAN_OUTPUT");
  if (!finitePose(prior_map_T_imu) || !finitePose(nominal_map_T_imu) ||
      !finitePose(T_imu_lidar) || !std::isfinite(length_scale_m) ||
      length_scale_m <= 0.0)
    return reject("INVALID_EXACT_RESIDUAL_JACOBIAN_INPUT");
  const Eigen::Matrix<double, 6, 1> residual =
      poseResidual(prior_map_T_imu, nominal_map_T_imu);
  Eigen::Matrix3d left_inverse;
  if (!dog_prior_map_fastlio2_frontend_exp::so3LeftJacobianInverse(
          residual.tail<3>(), &left_inverse, reason))
    return false;
  const Eigen::Matrix3d prior_rotation =
      prior_map_T_imu.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d measurement_rotation =
      nominal_map_T_imu.orientation.normalized().toRotationMatrix();
  const Eigen::Vector3d lever =
      -(measurement_rotation * T_imu_lidar.position);
  jacobian->setZero();
  jacobian->block<3, 3>(0, 0) = -skewMatrix(lever);
  jacobian->block<3, 3>(0, 3) =
      length_scale_m * Eigen::Matrix3d::Identity();
  jacobian->block<3, 3>(3, 0) = left_inverse * prior_rotation.transpose();
  if (!jacobian->allFinite())
    return reject("NONFINITE_EXACT_RESIDUAL_JACOBIAN");
  return true;
}

inline bool makeResidualConsistentMeasurementBasisFromImuMeasurement(
    const Pose3d& prior_map_T_imu, const Pose3d& nominal_map_T_imu,
    const Pose3d& T_imu_lidar, double length_scale_m,
    const dog_prior_map_fastlio2_frontend_exp::reliability::Matrix6d&
        normalized_weak_basis,
    int weak_dimension,
    dog_prior_map_fastlio2_frontend_exp::reliability::Matrix6d*
        measurement_basis,
    int* reliable_rank, std::string* reason = nullptr) {
  dog_prior_map_fastlio2_frontend_exp::reliability::Matrix6d exact_jacobian;
  if (!normalizedLidarToExactResidualJacobianFromImuMeasurement(
          prior_map_T_imu, nominal_map_T_imu, T_imu_lidar, length_scale_m,
          &exact_jacobian, reason))
    return false;
  if (!dog_prior_map_fastlio2_frontend_exp::reliability::
          buildReliableMeasurementBasisFromWeak(
              exact_jacobian, normalized_weak_basis, weak_dimension,
              measurement_basis, reliable_rank, reason))
    return false;
  if (*reliable_rank != 6 - weak_dimension) {
    if (reason) *reason = "EXACT_RELIABLE_RANK_MISMATCH";
    return false;
  }
  if (weak_dimension > 0 && *reliable_rank > 0) {
    const Eigen::MatrixXd weak = normalized_weak_basis.leftCols(weak_dimension);
    const Eigen::MatrixXd reliable = measurement_basis->leftCols(*reliable_rank);
    const Eigen::MatrixXd mapped = exact_jacobian * weak;
    const double leakage = (reliable.transpose() * mapped).norm() /
        (1.0 + mapped.norm());
    if (!std::isfinite(leakage) || leakage >= 1e-9) {
      if (reason) *reason = "EXACT_RESIDUAL_WEAK_DIRECTION_LEAKAGE";
      return false;
    }
  }
  return true;
}

}  // namespace p6_i6e_r3
