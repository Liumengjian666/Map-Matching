#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace p6_i4 {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Pose = Eigen::Matrix4d;

struct PoseCovarianceSpectrum {
  bool valid = false;
  std::string reason;
  Matrix6d covariance = Matrix6d::Zero();
  Matrix6d eigenvectors = Matrix6d::Identity();
  Vector6d eigenvalues = Vector6d::Zero();
  Matrix6d pseudoinverse = Matrix6d::Zero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> whitened_basis;
  int effective_rank = 0;
  double numerical_tolerance = 0.0;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(),
            value.z(), 0.0, -value.x(),
            -value.y(), value.x(), 0.0;
  return result;
}

inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& angle_axis) {
  const double angle = angle_axis.norm();
  if (angle < 1e-12) {
    Eigen::Quaterniond quaternion(1.0, 0.5 * angle_axis.x(),
                                  0.5 * angle_axis.y(), 0.5 * angle_axis.z());
    quaternion.normalize();
    return quaternion.toRotationMatrix();
  }
  return Eigen::AngleAxisd(angle, angle_axis / angle).toRotationMatrix();
}

inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    throw std::runtime_error("invalid_rotation_for_log");
  return angle_axis.angle() * angle_axis.axis();
}

// Product-manifold perturbation required by the P6-I4 protocol:
// R_seed=Exp(delta_phi_map)R_pred and p_seed=p_pred+delta_p_map.
inline Pose boxplusMapPose(const Pose& nominal, const Vector6d& delta) {
  if (!nominal.allFinite() || !delta.allFinite())
    throw std::runtime_error("nonfinite_pose_or_tangent");
  Pose result = nominal;
  result.block<3, 3>(0, 0) = expSO3(delta.head<3>()) *
                             nominal.block<3, 3>(0, 0);
  result.block<3, 1>(0, 3) = nominal.block<3, 1>(0, 3) + delta.tail<3>();
  return result;
}

inline double translationSeparation(const Pose& a, const Pose& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

inline double rotationSeparationDeg(const Pose& a, const Pose& b) {
  const Eigen::Matrix3d relative = a.block<3, 3>(0, 0).transpose() *
                                  b.block<3, 3>(0, 0);
  return Eigen::AngleAxisd(relative).angle() * 180.0 / M_PI;
}

struct ModeComparison {
  double translation_separation_m = std::numeric_limits<double>::quiet_NaN();
  double rotation_separation_deg = std::numeric_limits<double>::quiet_NaN();
  bool same_mode = false;
};

inline ModeComparison compareOperationalMode(const Pose& a, bool a_converged,
                                             const Pose& b, bool b_converged) {
  ModeComparison result;
  if (!a.allFinite() || !b.allFinite()) return result;
  result.translation_separation_m = translationSeparation(a, b);
  result.rotation_separation_deg = rotationSeparationDeg(a, b);
  if (a_converged && b_converged) {
    // The tiny angular allowance only absorbs radians/degrees roundoff at the
    // specified inclusive 2-degree boundary; it is not a tunable mode gate.
    constexpr double kAngularRoundoffDeg = 1e-12;
    result.same_mode = result.translation_separation_m <= 0.20 &&
        result.rotation_separation_deg <= 2.0 + kAngularRoundoffDeg;
  }
  return result;
}

inline Eigen::MatrixXd buildPoseErrorJacobian(
    const Eigen::Matrix3d& nominal_rotation, int state_dof,
    int position_error_index, int rotation_error_index) {
  if (state_dof <= 0 || position_error_index < 0 || rotation_error_index < 0 ||
      position_error_index + 3 > state_dof || rotation_error_index + 3 > state_dof)
    throw std::runtime_error("pose_error_indices_out_of_range");
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, state_dof);
  jacobian.block<3, 3>(0, rotation_error_index) = nominal_rotation;
  jacobian.block<3, 3>(3, position_error_index).setIdentity();
  return jacobian;
}

inline PoseCovarianceSpectrum analyzePoseCovariance(const Matrix6d& input) {
  PoseCovarianceSpectrum result;
  if (!input.allFinite()) {
    result.reason = "NONFINITE";
    return result;
  }
  result.covariance = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(result.covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    result.reason = "EIGENSOLVE_FAILED";
    return result;
  }
  result.eigenvalues = solver.eigenvalues();
  result.eigenvectors = solver.eigenvectors();
  const double lambda_max = result.eigenvalues.maxCoeff();
  result.numerical_tolerance = 1e-10 * std::max(lambda_max, 1e-18);
  for (int i = 0; i < 6; ++i) {
    if (result.eigenvalues(i) < -result.numerical_tolerance) {
      result.reason = "COVARIANCE_INVALID_NEGATIVE_EIGENVALUE";
      return result;
    }
    if (result.eigenvalues(i) < 0.0) result.eigenvalues(i) = 0.0;
    if (result.eigenvalues(i) > 0.0) ++result.effective_rank;
  }
  result.covariance = result.eigenvectors * result.eigenvalues.asDiagonal() *
                      result.eigenvectors.transpose();
  result.whitened_basis.resize(6, result.effective_rank);
  int active_column = 0;
  for (int i = 0; i < 6; ++i) {
    const double lambda = result.eigenvalues(i);
    if (lambda <= 0.0) continue;
    const Eigen::VectorXd direction = result.eigenvectors.col(i);
    result.pseudoinverse.noalias() +=
        (direction * direction.transpose()) / lambda;
    result.whitened_basis.col(active_column++) =
        result.eigenvectors.col(i) * std::sqrt(lambda);
  }
  result.valid = true;
  result.reason = "PASS";
  return result;
}

inline double priorMetricRadius(const Vector6d& delta,
                                const PoseCovarianceSpectrum& spectrum) {
  if (!spectrum.valid) return std::numeric_limits<double>::quiet_NaN();
  const double squared = delta.dot(spectrum.pseudoinverse * delta);
  if (!std::isfinite(squared) || squared < -1e-10)
    return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(std::max(0.0, squared));
}

inline Vector6d whitenedPerturbation(
    const PoseCovarianceSpectrum& spectrum,
    const Eigen::VectorXd& unit_direction, double alpha) {
  if (!spectrum.valid || unit_direction.size() != spectrum.effective_rank ||
      !unit_direction.allFinite() || !std::isfinite(alpha))
    throw std::runtime_error("invalid_whitened_perturbation_input");
  if (std::abs(unit_direction.norm() - 1.0) > 1e-10)
    throw std::runtime_error("whitened_direction_not_unit");
  return alpha * spectrum.whitened_basis * unit_direction;
}

}  // namespace p6_i4
