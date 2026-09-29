#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <utility>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

bool finitePsd(const Eigen::MatrixXd& matrix, double tolerance) {
  if (!matrix.allFinite() ||
      (matrix - matrix.transpose()).cwiseAbs().maxCoeff() > 1e-8)
    return false;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(matrix);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
      solver.eigenvalues().minCoeff() >= -tolerance;
}

}  // namespace

bool FixedLagWindow::marginalizeOldest(std::string* reason) {
  if (reason) reason->clear();
  if (states_.size() < 2) return fail(reason, "cannot_marginalize_single_state");
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  double cost = 0.0;
  // Assemble only information that will be consumed by this Schur operation:
  // the existing prior plus factors incident on the oldest state. Factors
  // whose endpoints are all retained remain active and relinearizable.
  if (!linearizeMarginalizationSubgraph(&hessian, &gradient, &cost, reason))
    return false;
  const Eigen::Index marginalized_dimension = 15;
  const Eigen::Index retained_dimension = hessian.rows() - marginalized_dimension;
  if (retained_dimension <= 0)
    return fail(reason, "no_retained_window_state_after_marginalization");
  const Eigen::MatrixXd hmm = hessian.topLeftCorner(
      marginalized_dimension, marginalized_dimension);
  const Eigen::MatrixXd hmr = hessian.topRightCorner(
      marginalized_dimension, retained_dimension);
  const Eigen::MatrixXd hrr = hessian.bottomRightCorner(
      retained_dimension, retained_dimension);
  const Eigen::VectorXd bm = gradient.head(marginalized_dimension);
  const Eigen::VectorXd br = gradient.tail(retained_dimension);

  // A tiny solve-only jitter is used when the oldest state is gauge-like. It
  // is never added to the stored prior, so numerical damping is not treated
  // as measurement information.
  Eigen::MatrixXd hmm_solve = 0.5 * (hmm + hmm.transpose());
  double jitter = 0.0;
  Eigen::LDLT<Eigen::MatrixXd> factor(hmm_solve);
  if (factor.info() != Eigen::Success ||
      factor.vectorD().cwiseAbs().minCoeff() < 1e-12) {
    jitter = 1e-9 * std::max(1.0, hmm_solve.diagonal().cwiseAbs().maxCoeff());
    hmm_solve.diagonal().array() += jitter;
    factor.compute(hmm_solve);
  }
  if (factor.info() != Eigen::Success ||
      factor.vectorD().cwiseAbs().minCoeff() < 1e-14)
    return fail(reason, "marginalization_oldest_block_solve_failed");
  const Eigen::MatrixXd correction_h = factor.solve(hmr);
  const Eigen::VectorXd correction_b = factor.solve(bm);
  Eigen::MatrixXd new_information = hrr - hmr.transpose() * correction_h;
  Eigen::VectorXd new_gradient = br - hmr.transpose() * correction_b;
  new_information = 0.5 * (new_information + new_information.transpose());
  if (!finitePsd(new_information, 1e-6) || !new_gradient.allFinite())
    return fail(reason, "marginalized_prior_not_finite_psd");

  double jitter_information_delta_norm = 0.0;
  double jitter_gradient_delta_norm = 0.0;
  if (jitter > 0.0) {
    // Report the indirect information change introduced by the solve-only
    // jitter against the unjittered Moore-Penrose Schur reference.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> unjittered_solver(
        0.5 * (hmm + hmm.transpose()));
    if (unjittered_solver.info() == Eigen::Success &&
        unjittered_solver.eigenvalues().allFinite()) {
      const double threshold = 1e-12 * std::max(
          1.0, unjittered_solver.eigenvalues().cwiseAbs().maxCoeff());
      Eigen::VectorXd inverse_eigenvalues =
          Eigen::VectorXd::Zero(unjittered_solver.eigenvalues().size());
      for (Eigen::Index index = 0; index < inverse_eigenvalues.size(); ++index) {
        const double eigenvalue = unjittered_solver.eigenvalues()(index);
        if (std::abs(eigenvalue) > threshold)
          inverse_eigenvalues(index) = 1.0 / eigenvalue;
      }
      const Eigen::MatrixXd pseudoinverse =
          unjittered_solver.eigenvectors() * inverse_eigenvalues.asDiagonal() *
          unjittered_solver.eigenvectors().transpose();
      Eigen::MatrixXd alternate_information =
          hrr - hmr.transpose() * pseudoinverse * hmr;
      const Eigen::VectorXd alternate_gradient =
          br - hmr.transpose() * pseudoinverse * bm;
      alternate_information =
          0.5 * (alternate_information + alternate_information.transpose());
      jitter_information_delta_norm =
          (new_information - alternate_information).norm();
      jitter_gradient_delta_norm =
          (new_gradient - alternate_gradient).norm();
    }
  }

  double retained_cross_information_norm = 0.0;
  if (retained_dimension >= 30) {
    const Eigen::Index block_count = retained_dimension / 15;
    for (Eigen::Index row = 0; row < block_count; ++row) {
      for (Eigen::Index column = row + 1; column < block_count; ++column) {
        retained_cross_information_norm +=
            new_information.block(row * 15, column * 15, 15, 15).norm();
      }
    }
  }

  const std::uint64_t removed_stamp = states_.front().stamp_ns;
  const bool feedback_was_current = summary_.prediction_feedback_ready &&
      optimized_revision_ == window_revision_;
  prior_.information = std::move(new_information);
  prior_.gradient = std::move(new_gradient);
  prior_.reference_states.assign(states_.begin() + 1, states_.end());
  prior_.valid = true;
  states_.erase(states_.begin());
  imu_factors_.erase(std::remove_if(imu_factors_.begin(), imu_factors_.end(),
      [this, removed_stamp](const ImuFactorRecord& factor_record) {
        const bool remove = factor_record.from_stamp_ns == removed_stamp ||
            factor_record.to_stamp_ns == removed_stamp;
        if (remove) retireObservationId(factor_record.observation_id);
        return remove;
      }), imu_factors_.end());
  lidar_factors_.erase(std::remove_if(lidar_factors_.begin(), lidar_factors_.end(),
      [this, removed_stamp](const LidarFactorRecord& factor_record) {
        const bool remove = factor_record.measurement.stamp_ns == removed_stamp;
        if (remove) retireObservationId(factor_record.measurement.observation_id);
        return remove;
      }), lidar_factors_.end());
  visual_factors_.erase(std::remove_if(visual_factors_.begin(), visual_factors_.end(),
      [this, removed_stamp](const VisualFactorRecord& factor_record) {
        const bool remove =
            factor_record.measurement.reference_stamp_ns == removed_stamp ||
            factor_record.measurement.current_stamp_ns == removed_stamp;
        if (remove) retireObservationId(factor_record.measurement.observation_id);
        return remove;
      }), visual_factors_.end());
  ++window_revision_;
  if (feedback_was_current) {
    optimized_revision_ = window_revision_;
    summary_.prediction_feedback_ready = true;
    summary_.prediction_feedback_status = "LATEST_OPTIMIZED_STATE_READY";
  } else {
    summary_.prediction_feedback_ready = false;
    summary_.prediction_feedback_status = "STALE_WINDOW_REVISION";
  }
  summary_.retained_prior_cross_information_norm =
      retained_cross_information_norm;
  summary_.marginalization_solve_jitter = jitter;
  summary_.marginalization_jitter_information_delta_norm =
      jitter_information_delta_norm;
  summary_.marginalization_jitter_gradient_delta_norm =
      jitter_gradient_delta_norm;
  summary_.marginalization_performed = true;
  summary_.marginalization_psd = true;
  summary_.marginalization_status = jitter > 0.0
      ? "PASS_SCHUR_WITH_SOLVE_ONLY_JITTER" : "PASS_SCHUR";
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
