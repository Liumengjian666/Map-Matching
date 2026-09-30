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

bool freezeLidarProjection(const WindowState& state,
                           const LidarWindowMeasurement& measurement,
                           FrozenLidarProjection* output,
                           std::string* reason) {
  if (reason) reason->clear();
  if (!output) return fail(reason, "null_frozen_lidar_projection");
  *output = FrozenLidarProjection();
  if (!measurement.valid || state.stamp_ns != measurement.stamp_ns ||
      !measurement.covariance.allFinite() ||
      measurement.reliable_rank < 1 || measurement.reliable_rank > 6)
    return fail(reason, "invalid_lidar_projection_input");

  Matrix6d basis = measurement.measurement_basis;
  int rank = measurement.reliable_rank;
  if (rank < 6) {
    if (!measurement.basis_relinearizer)
      return fail(reason, "lidar_basis_relinearizer_missing");
    std::string callback_reason;
    if (!measurement.basis_relinearizer(state, &basis, &rank,
                                         &callback_reason)) {
      if (reason) *reason = callback_reason.empty()
          ? "lidar_basis_relinearization_failed" : callback_reason;
      return false;
    }
  }
  if (rank < 1 || rank > 6 || !basis.allFinite())
    return fail(reason, "invalid_frozen_lidar_basis");
  const Eigen::MatrixXd selected_basis = basis.leftCols(rank);
  const Eigen::MatrixXd gram = selected_basis.transpose() * selected_basis;
  if (!gram.allFinite() ||
      (gram - Eigen::MatrixXd::Identity(rank, rank)).norm() > 1e-8)
    return fail(reason, "frozen_lidar_basis_not_orthonormal");

  Eigen::MatrixXd selected_covariance = selected_basis.transpose() *
      measurement.covariance * selected_basis;
  selected_covariance =
      (0.5 * (selected_covariance + selected_covariance.transpose())).eval();
  if (!selected_covariance.allFinite())
    return fail(reason, "nonfinite_frozen_lidar_covariance");
  Eigen::LLT<Eigen::MatrixXd> factor(selected_covariance);
  if (factor.info() != Eigen::Success ||
      !factor.matrixL().toDenseMatrix().diagonal().allFinite() ||
      (factor.matrixL().toDenseMatrix().diagonal().array() <= 0.0).any())
    return fail(reason, "frozen_lidar_covariance_not_spd");

  output->observation_id = measurement.observation_id;
  output->stamp_ns = measurement.stamp_ns;
  output->basis = basis;
  output->reliable_rank = rank;
  output->selected_covariance = std::move(selected_covariance);
  output->valid = true;
  return true;
}

bool linearizeLidarFactorWithFrozenProjection(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    const FrozenLidarProjection& projection, Eigen::VectorXd* residual,
    Eigen::MatrixXd* jacobian, Eigen::MatrixXd* covariance,
    std::string* reason) {
  if (reason) reason->clear();
  if (!residual || !jacobian || !covariance)
    return fail(reason, "null_lidar_linearization_output");
  if (!measurement.valid || !projection.valid ||
      projection.observation_id != measurement.observation_id ||
      projection.stamp_ns != measurement.stamp_ns ||
      state.stamp_ns != measurement.stamp_ns ||
      projection.reliable_rank < 1 || projection.reliable_rank > 6 ||
      !projection.basis.allFinite() ||
      projection.selected_covariance.rows() != projection.reliable_rank ||
      projection.selected_covariance.cols() != projection.reliable_rank ||
      !projection.selected_covariance.allFinite())
    return fail(reason, "frozen_lidar_projection_identity_or_shape_mismatch");
  const Eigen::MatrixXd selected_basis =
      projection.basis.leftCols(projection.reliable_rank);
  const Eigen::MatrixXd gram = selected_basis.transpose() * selected_basis;
  if (!gram.allFinite() ||
      (gram - Eigen::MatrixXd::Identity(projection.reliable_rank,
                                        projection.reliable_rank)).norm() > 1e-8 ||
      (projection.selected_covariance -
       projection.selected_covariance.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return fail(reason, "invalid_frozen_lidar_projection");
  Eigen::LLT<Eigen::MatrixXd> covariance_factor(
      projection.selected_covariance);
  if (covariance_factor.info() != Eigen::Success)
    return fail(reason, "frozen_lidar_covariance_not_spd");

  const Eigen::Matrix<double, 6, 1> raw = rawPoseResidual(state, measurement);
  if (!raw.allFinite()) return fail(reason, "nonfinite_lidar_pose_residual");
  *residual = selected_basis.transpose() * raw;

  const Eigen::Vector3d phi = raw.tail<3>();
  const double theta = phi.norm();
  if (std::abs(theta - std::acos(-1.0)) < 1e-12)
    return fail(reason, "lidar_rotation_log_branch_cut");
  Eigen::Matrix3d hat;
  hat << 0, -phi.z(), phi.y(), phi.z(), 0, -phi.x(), -phi.y(), phi.x(), 0;
  const double coefficient = theta < 1e-5
      ? 1.0 / 12.0 + theta * theta / 720.0
      : (1.0 - 0.5 * theta / std::tan(0.5 * theta)) / (theta * theta);
  Eigen::Matrix<double, 6, 15> raw_jacobian =
      Eigen::Matrix<double, 6, 15>::Zero();
  raw_jacobian.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  raw_jacobian.block<3, 3>(3, 0) =
      -(Eigen::Matrix3d::Identity() - 0.5 * hat + coefficient * hat * hat);
  *jacobian = selected_basis.transpose() * raw_jacobian;
  *covariance = projection.selected_covariance;
  if (!residual->allFinite() || !jacobian->allFinite())
    return fail(reason, "nonfinite_frozen_lidar_factor");
  return true;
}

bool linearizeLidarFactorFiniteDifferenceReference(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    Eigen::VectorXd* residual, Eigen::MatrixXd* jacobian,
    Eigen::MatrixXd* covariance, std::string* reason) {
  if (!jacobian || !covariance)
    return fail(reason, "null_lidar_linearization_output");
  FrozenLidarProjection projection;
  if (!freezeLidarProjection(state, measurement, &projection, reason))
    return false;
  LidarWindowMeasurement frozen = measurement;
  frozen.measurement_basis = projection.basis;
  frozen.reliable_rank = projection.reliable_rank;
  if (!buildLidarResidualFrozen(state, frozen, residual, reason)) return false;
  const int rank = projection.reliable_rank;
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
  *covariance = projection.selected_covariance;
  if (!jacobian->allFinite() || !covariance->allFinite())
    return fail(reason, "nonfinite_frozen_lidar_factor");
  return true;
}

bool linearizeLidarFactor(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    Eigen::VectorXd* residual, Eigen::MatrixXd* jacobian,
    Eigen::MatrixXd* covariance, std::string* reason) {
  FrozenLidarProjection projection;
  if (!freezeLidarProjection(state, measurement, &projection, reason))
    return false;
  return linearizeLidarFactorWithFrozenProjection(
      state, measurement, projection, residual, jacobian, covariance, reason);
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
