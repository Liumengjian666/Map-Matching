#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond q(rotation);
  if (!rotation.allFinite() || !q.coeffs().allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  q.normalize();
  if (q.w() < 0.0) q.coeffs() *= -1.0;
  const double sine_half = q.vec().norm();
  if (sine_half < 1e-12) return 2.0 * q.vec();
  const double angle = 2.0 * std::atan2(sine_half,
      std::clamp(q.w(), -1.0, 1.0));
  return q.vec() * (angle / sine_half);
}

bool validBasis(const LidarWindowMeasurement& measurement) {
  if (!measurement.valid || measurement.reliable_rank < 1 ||
      measurement.reliable_rank > 6 || !measurement.measurement_basis.allFinite() ||
      !measurement.covariance.allFinite()) return false;
  const Eigen::MatrixXd basis = measurement.measurement_basis.leftCols(
      measurement.reliable_rank);
  return (basis.transpose() * basis -
      Eigen::MatrixXd::Identity(measurement.reliable_rank,
                                 measurement.reliable_rank)).norm() < 1e-8;
}

Eigen::Matrix<double, 6, 1> rawPoseResidual(
    const WindowState& state, const LidarWindowMeasurement& measurement);

bool freezeBasisAtState(const WindowState& state,
                        const LidarWindowMeasurement& measurement,
                        LidarWindowMeasurement* frozen,
                        std::string* reason) {
  if (!frozen) return fail(reason, "null_frozen_lidar_measurement");
  *frozen = measurement;
  // Full-rank measurement coordinates span the entire residual and are
  // invariant to a change of orthonormal basis. Degenerate factors must be
  // relinearized by the upstream U_obs/R3 adapter.
  if (measurement.reliable_rank < 6) {
    if (!measurement.basis_relinearizer)
      return fail(reason, "lidar_basis_relinearizer_missing");
    Matrix6d basis = Matrix6d::Zero();
    int rank = 0;
    std::string local_reason;
    if (!measurement.basis_relinearizer(state, &basis, &rank, &local_reason)) {
      if (reason) *reason = local_reason.empty()
          ? "lidar_basis_relinearization_failed" : local_reason;
      return false;
    }
    frozen->measurement_basis = basis;
    frozen->reliable_rank = rank;
  }
  if (!validBasis(*frozen))
    return fail(reason, "invalid_relinearized_lidar_basis");
  return true;
}

bool buildLidarResidualFrozen(const WindowState& state,
                              const LidarWindowMeasurement& measurement,
                              Eigen::VectorXd* residual,
                              std::string* reason) {
  if (!residual) return fail(reason, "null_lidar_residual");
  if (state.stamp_ns != measurement.stamp_ns)
    return fail(reason, "lidar_state_timestamp_mismatch");
  if (!validBasis(measurement))
    return fail(reason, measurement.skipped_reason.empty()
                         ? "invalid_lidar_basis_or_measurement"
                         : measurement.skipped_reason.c_str());
  const Eigen::Matrix<double, 6, 1> raw = rawPoseResidual(state, measurement);
  if (!raw.allFinite()) return fail(reason, "nonfinite_lidar_pose_residual");
  *residual = measurement.measurement_basis.leftCols(measurement.reliable_rank)
      .transpose() * raw;
  if (!residual->allFinite()) return fail(reason, "nonfinite_lidar_factor_residual");
  return true;
}

Eigen::Matrix<double, 6, 1> rawPoseResidual(
    const WindowState& state, const LidarWindowMeasurement& measurement) {
  Eigen::Matrix<double, 6, 1> result;
  result.head<3>() = measurement.measured_position - state.position;
  result.tail<3>() = so3Log(state.rotation.transpose() *
                             measurement.measured_rotation);
  return result;
}

}  // namespace

bool buildLidarResidual(const WindowState& state,
                        const LidarWindowMeasurement& measurement,
                        Eigen::VectorXd* residual, std::string* reason) {
  if (reason) reason->clear();
  LidarWindowMeasurement frozen;
  if (!freezeBasisAtState(state, measurement, &frozen, reason)) return false;
  return buildLidarResidualFrozen(state, frozen, residual, reason);
}

bool linearizeLidarFactor(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    Eigen::VectorXd* residual, Eigen::MatrixXd* jacobian,
    Eigen::MatrixXd* covariance, std::string* reason) {
  if (!jacobian || !covariance)
    return fail(reason, "null_lidar_linearization_output");
  LidarWindowMeasurement frozen;
  if (!freezeBasisAtState(state, measurement, &frozen, reason) ||
      !buildLidarResidualFrozen(state, frozen, residual, reason)) return false;
  const int rank = frozen.reliable_rank;
  jacobian->setZero(rank, 15);
  constexpr double h = 1e-7;
  for (int column = 0; column < 6; ++column) {
    Vector15d increment = Vector15d::Zero();
    increment(column) = h;
    WindowState plus = state, minus = state;
    applyLocalIncrement(&plus, increment);
    applyLocalIncrement(&minus, -increment);
    Eigen::VectorXd residual_plus, residual_minus;
    if (!buildLidarResidualFrozen(plus, frozen, &residual_plus, reason) ||
        !buildLidarResidualFrozen(minus, frozen, &residual_minus, reason))
      return false;
    jacobian->col(column) = (residual_plus - residual_minus) / (2.0 * h);
  }
  const Eigen::MatrixXd basis = frozen.measurement_basis.leftCols(rank);
  *covariance = basis.transpose() * frozen.covariance * basis;
  *covariance = 0.5 * (*covariance + covariance->transpose());
  Eigen::LLT<Eigen::MatrixXd> factor(*covariance);
  if (factor.info() != Eigen::Success || !jacobian->allFinite() ||
      !covariance->allFinite())
    return fail(reason, "lidar_factor_covariance_not_spd");
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
