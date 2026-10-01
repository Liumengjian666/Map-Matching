#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
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

std::uint64_t fnv1aUpdate(std::uint64_t hash, const void* bytes,
                          std::size_t size) {
  const auto* data = static_cast<const unsigned char*>(bytes);
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= static_cast<std::uint64_t>(data[i]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t priorHash(const Eigen::MatrixXd& information,
                        const Eigen::VectorXd& gradient) {
  std::uint64_t hash = 14695981039346656037ULL;
  const Eigen::Index rows = information.rows();
  const Eigen::Index cols = information.cols();
  const Eigen::Index gradient_size = gradient.size();
  hash = fnv1aUpdate(hash, &rows, sizeof(rows));
  hash = fnv1aUpdate(hash, &cols, sizeof(cols));
  hash = fnv1aUpdate(hash, &gradient_size, sizeof(gradient_size));
  if (information.size() > 0)
    hash = fnv1aUpdate(hash, information.data(),
        static_cast<std::size_t>(information.size()) * sizeof(double));
  if (gradient.size() > 0)
    hash = fnv1aUpdate(hash, gradient.data(),
        static_cast<std::size_t>(gradient.size()) * sizeof(double));
  return hash;
}

bool diagnosticPsdPass(const MarginalizationMatrixStats& stats) {
  // Intermediate blocks are forensic components, not production validator
  // inputs. Treat scale-relative round-off as such; M7 below uses the exact
  // production absolute checks so the first rejecting stage remains exact.
  constexpr double kIntermediateRelativeTolerance = 1e-10;
  return stats.available && stats.finite &&
      stats.symmetry_frobenius_norm /
          std::max(stats.spectral_scale,
                   std::numeric_limits<double>::epsilon()) <=
          kIntermediateRelativeTolerance &&
      stats.relative_negative_ratio <= kIntermediateRelativeTolerance;
}

bool productionPsdPass(const MarginalizationMatrixStats& stats) {
  return stats.available && stats.finite && stats.symmetry_max_abs <= 1e-8 &&
      stats.lambda_min >= -1e-6;
}

const char* firstPsdFailureStage(
    const MarginalizationTraceRecord& trace) {
  if (trace.incoming_prior_valid && !diagnosticPsdPass(trace.incoming_prior))
    return "M0_INCOMING_PRIOR";
  if (trace.charted_prior.available &&
      !diagnosticPsdPass(trace.charted_prior))
    return "M1_CHARTED_PRIOR";
  if (trace.touching_factors.available &&
      !diagnosticPsdPass(trace.touching_factors))
    return "M2_TOUCHING_FACTORS";
  if (trace.consumed_system.available &&
      !diagnosticPsdPass(trace.consumed_system))
    return "M3_CONSUMED_SYSTEM";
  if (trace.hmm.available && !diagnosticPsdPass(trace.hmm))
    return "M4_HMM";
  if (trace.raw_schur.available && !trace.raw_schur.finite)
    return "M6_RAW_SCHUR";
  if (trace.symmetrized_schur.available &&
      !productionPsdPass(trace.symmetrized_schur))
    return "M7_SYMMETRIZED_SCHUR";
  if (trace.new_gradient_evaluated && !trace.new_gradient_finite)
    return "M8_NEW_GRADIENT";
  return "NONE";
}

void appendStateStamps(const WindowStateVector& states,
                       std::vector<std::uint64_t>* output) {
  if (!output) return;
  output->clear();
  output->reserve(states.size());
  for (const auto& state : states) output->push_back(state.stamp_ns);
}

}  // namespace

MarginalizationMatrixStats marginalizationMatrixStatsForDiagnostics(
    const Eigen::MatrixXd& matrix) {
  MarginalizationMatrixStats result;
  if (matrix.rows() != matrix.cols() || matrix.rows() == 0) {
    result.status = "NON_SQUARE_OR_EMPTY";
    return result;
  }
  result.available = true;
  result.dimension = matrix.rows();
  result.finite = matrix.allFinite();
  if (!result.finite) {
    result.status = "NONFINITE";
    return result;
  }
  result.symmetry_frobenius_norm = (matrix - matrix.transpose()).norm();
  result.symmetry_max_abs =
      (matrix - matrix.transpose()).cwiseAbs().maxCoeff();
  result.frobenius_norm = matrix.norm();
  const Eigen::MatrixXd symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    result.status = "EIGENSOLVER_FAILED";
    return result;
  }
  result.lambda_min = solver.eigenvalues().minCoeff();
  result.lambda_max = solver.eigenvalues().maxCoeff();
  result.min_abs_eigenvalue = solver.eigenvalues().cwiseAbs().minCoeff();
  result.spectral_scale = solver.eigenvalues().cwiseAbs().maxCoeff();
  result.relative_negative_ratio =
      std::max(0.0, -result.lambda_min) /
      std::max(result.spectral_scale, std::numeric_limits<double>::epsilon());
  result.negative_eigenvalue_count =
      (solver.eigenvalues().array() < 0.0).count();
  const double rank_threshold = std::numeric_limits<double>::epsilon() *
      static_cast<double>(matrix.rows()) * result.spectral_scale;
  result.numerical_rank = rank_threshold == 0.0 ? 0 :
      (solver.eigenvalues().array().abs() > rank_threshold).count();
  result.status = "OK";
  return result;
}

std::uint64_t marginalizationPriorHashForDiagnostics(
    const Eigen::MatrixXd& information, const Eigen::VectorXd& gradient) {
  return priorHash(information, gradient);
}

MarginalizationLdltStats marginalizationLdltStatsForDiagnostics(
    const Eigen::VectorXd& diagonal) {
  MarginalizationLdltStats result;
  if (diagonal.size() == 0 || !diagonal.allFinite()) return result;
  result.available = true;
  result.min_abs_d = diagonal.cwiseAbs().minCoeff();
  result.max_abs_d = diagonal.cwiseAbs().maxCoeff();
  result.pivot_ratio = result.max_abs_d > 0.0
      ? result.min_abs_d / result.max_abs_d : 0.0;
  result.near_zero_threshold = std::numeric_limits<double>::epsilon() *
      static_cast<double>(diagonal.size()) * result.max_abs_d;
  for (Eigen::Index i = 0; i < diagonal.size(); ++i) {
    if (diagonal(i) > 0.0) ++result.positive_d_count;
    if (diagonal(i) < 0.0) ++result.negative_d_count;
    if (std::abs(diagonal(i)) <= result.near_zero_threshold)
      ++result.near_zero_d_count;
  }
  return result;
}

bool FixedLagWindow::marginalizeOldest(std::string* reason) {
  return options_.marginalization_backend == MarginalizationBackend::SQUARE_ROOT_QR
      ? marginalizeOldestSquareRoot(reason) : marginalizeOldestInformation(reason);
}

bool FixedLagWindow::marginalizeOldestInformation(std::string* reason) {
  if (reason) reason->clear();
  const bool capture = options_.capture_marginalization_diagnostics;
  MarginalizationTraceRecord trace;
  Eigen::MatrixXd incoming_prior_information;
  Eigen::VectorXd incoming_prior_gradient;
  std::vector<std::uint64_t> state_stamps_at_attempt;
  if (capture) {
    trace.marginalization_enforcement_index =
        marginalization_enforcement_index_;
    trace.attempt_index_within_enforcement =
        marginalization_attempt_index_;
    trace.latest_state_stamp_ns = states_.back().stamp_ns;
    trace.oldest_state_stamp_ns = states_.front().stamp_ns;
    trace.nodes_before_attempt = states_.size();
    trace.span_before_attempt_s = static_cast<double>(
        states_.back().stamp_ns - states_.front().stamp_ns) * 1e-9;
    trace.removed_state_stamp_ns = states_.front().stamp_ns;
    trace.nodes_after_attempt = states_.size();
    trace.span_after_attempt_s = trace.span_before_attempt_s;
    trace.trigger_duration_limit =
        marginalization_trigger_duration_limit_;
    trace.trigger_node_limit = marginalization_trigger_node_limit_;
    trace.incoming_prior_valid = prior_.valid;
    trace.incoming_prior_hash_fnv1a64 =
        priorHash(prior_.information, prior_.gradient);
    if (prior_.valid) {
      incoming_prior_information = prior_.information;
      incoming_prior_gradient = prior_.gradient;
      trace.incoming_prior =
          marginalizationMatrixStatsForDiagnostics(prior_.information);
    }
    for (const auto& factor_record : imu_factors_)
      if (factor_record.from_stamp_ns == states_.front().stamp_ns ||
          factor_record.to_stamp_ns == states_.front().stamp_ns)
        ++trace.incident_imu_factor_count;
    for (const auto& factor_record : lidar_factors_)
      if (factor_record.measurement.stamp_ns == states_.front().stamp_ns)
        ++trace.incident_lidar_factor_count;
    for (const auto& factor_record : visual_factors_)
      if (factor_record.measurement.reference_stamp_ns == states_.front().stamp_ns ||
          factor_record.measurement.current_stamp_ns == states_.front().stamp_ns)
        ++trace.incident_visual_factor_count;
    appendStateStamps(states_, &state_stamps_at_attempt);
  }
  if (states_.size() < 2) {
    if (capture) {
      trace.first_bad_stage = "M3_CONSUMED_SYSTEM";
      trace.marginalization_result = "FAIL:cannot_marginalize_single_state";
      trace.nodes_after_attempt = states_.size();
      trace.span_after_attempt_s = 0.0;
      marginalization_trace_.push_back(trace);
    }
    return fail(reason, "cannot_marginalize_single_state");
  }

  const auto recordFailure = [&](const std::string& failure_reason,
                                 const char* fallback_stage,
                                 const Eigen::MatrixXd* hessian,
                                 const Eigen::VectorXd* gradient,
                                 const MarginalizationAssemblyDiagnostics*
                                     assembly,
                                 const Eigen::MatrixXd* correction_h,
                                 const Eigen::VectorXd* correction_b) {
    if (!capture) return;
    if (trace.first_bad_stage == "NONE") {
      const char* first_bad = firstPsdFailureStage(trace);
      trace.first_bad_stage = std::string(first_bad) == "NONE"
          ? fallback_stage : first_bad;
    }
    trace.marginalization_result = "FAIL:" + failure_reason;
    marginalization_trace_.push_back(trace);
    MarginalizationFailureCapsule capsule;
    capsule.valid = true;
    capsule.trace = trace;
    capsule.state_stamps_before_enforcement = enforcement_state_stamps_before_;
    capsule.state_stamps_at_attempt = state_stamps_at_attempt;
    capsule.prior_hash_before_enforcement_fnv1a64 =
        enforcement_prior_hash_before_;
    capsule.factors_before_enforcement = enforcement_factor_counts_before_;
    capsule.prior_hash_at_failure_fnv1a64 =
        priorHash(prior_.information, prior_.gradient);
    appendStateStamps(states_, &capsule.state_stamps_at_failure);
    capsule.factors_at_failure = {imu_factors_.size(), lidar_factors_.size(),
                                  visual_factors_.size()};
    capsule.incoming_prior_information = incoming_prior_information;
    capsule.incoming_prior_gradient = incoming_prior_gradient;
    if (assembly) {
      capsule.charted_prior_information = assembly->prior_hessian;
      capsule.charted_prior_gradient = assembly->prior_gradient;
      capsule.imu_hessian = assembly->imu_hessian;
      capsule.lidar_hessian = assembly->lidar_hessian;
      capsule.visual_hessian = assembly->visual_hessian;
    }
    if (hessian) capsule.consumed_hessian = *hessian;
    if (gradient) capsule.consumed_gradient = *gradient;
    if (correction_h) capsule.correction_h = *correction_h;
    if (correction_b) capsule.correction_b = *correction_b;
    marginalization_failure_capsule_ = std::move(capsule);
  };

  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  double cost = 0.0;
  MarginalizationAssemblyDiagnostics assembly;
  // Assemble only information that will be consumed by this Schur operation:
  // the existing prior plus factors incident on the oldest state. Factors
  // whose endpoints are all retained remain active and relinearizable.
  if (!linearizeMarginalizationSubgraph(&hessian, &gradient, &cost, reason,
                                         capture ? &assembly : nullptr)) {
    recordFailure(reason ? *reason : "marginalization_subgraph_assembly_failed",
                  "M2_TOUCHING_FACTORS", nullptr, nullptr,
                  capture ? &assembly : nullptr, nullptr, nullptr);
    return false;
  }
  if (capture) {
    trace.charted_prior = marginalizationMatrixStatsForDiagnostics(
        assembly.prior_hessian);
    const Eigen::MatrixXd touching_hessian = hessian - assembly.prior_hessian;
    trace.touching_factors =
        marginalizationMatrixStatsForDiagnostics(touching_hessian);
    trace.imu_contribution =
        marginalizationMatrixStatsForDiagnostics(assembly.imu_hessian);
    trace.lidar_contribution =
        marginalizationMatrixStatsForDiagnostics(assembly.lidar_hessian);
    trace.visual_contribution =
        marginalizationMatrixStatsForDiagnostics(assembly.visual_hessian);
    trace.consumed_system = marginalizationMatrixStatsForDiagnostics(hessian);
  }
  const Eigen::Index marginalized_dimension = 15;
  const Eigen::Index retained_dimension = hessian.rows() - marginalized_dimension;
  if (retained_dimension <= 0) {
    recordFailure("no_retained_window_state_after_marginalization",
                  "M3_CONSUMED_SYSTEM", &hessian, &gradient,
                  capture ? &assembly : nullptr, nullptr, nullptr);
    return fail(reason, "no_retained_window_state_after_marginalization");
  }
  const Eigen::MatrixXd hmm = hessian.topLeftCorner(
      marginalized_dimension, marginalized_dimension);
  const Eigen::MatrixXd hmr = hessian.topRightCorner(
      marginalized_dimension, retained_dimension);
  const Eigen::MatrixXd hrr = hessian.bottomRightCorner(
      retained_dimension, retained_dimension);
  const Eigen::VectorXd bm = gradient.head(marginalized_dimension);
  const Eigen::VectorXd br = gradient.tail(retained_dimension);
  if (capture) {
    trace.hmm = marginalizationMatrixStatsForDiagnostics(hmm);
    trace.hmm_condition_proxy = trace.hmm.spectral_scale /
        std::max(std::numeric_limits<double>::epsilon(),
                 trace.hmm.min_abs_eigenvalue);
  }

  Eigen::MatrixXd correction_h;
  Eigen::VectorXd correction_b;

  // A tiny solve-only jitter is used when the oldest state is gauge-like. It
  // is never added to the stored prior, so numerical damping is not treated
  // as measurement information.
  Eigen::MatrixXd hmm_solve = 0.5 * (hmm + hmm.transpose());
  double jitter = 0.0;
  Eigen::LDLT<Eigen::MatrixXd> factor(hmm_solve);
  const auto capturePivots = [&](const Eigen::VectorXd& diagonal,
                                 bool initial) {
    if (!capture) return;
    const auto pivots = marginalizationLdltStatsForDiagnostics(diagonal);
    if (!pivots.available) return;
    if (initial) {
      trace.ldlt_initial_min_abs_d = pivots.min_abs_d;
      trace.ldlt_initial_max_abs_d = pivots.max_abs_d;
      trace.ldlt_initial_pivot_ratio = pivots.pivot_ratio;
      trace.ldlt_initial_positive_d_count = pivots.positive_d_count;
      trace.ldlt_initial_negative_d_count = pivots.negative_d_count;
      trace.ldlt_initial_near_zero_d_count = pivots.near_zero_d_count;
    } else {
      trace.ldlt_solve_min_abs_d = pivots.min_abs_d;
      trace.ldlt_solve_max_abs_d = pivots.max_abs_d;
      trace.ldlt_solve_pivot_ratio = pivots.pivot_ratio;
      trace.ldlt_solve_positive_d_count = pivots.positive_d_count;
      trace.ldlt_solve_negative_d_count = pivots.negative_d_count;
      trace.ldlt_solve_near_zero_d_count = pivots.near_zero_d_count;
    }
  };
  if (capture && factor.vectorD().size() > 0)
    capturePivots(factor.vectorD(), true);
  if (factor.info() != Eigen::Success ||
      factor.vectorD().cwiseAbs().minCoeff() < 1e-12) {
    jitter = 1e-9 * std::max(1.0, hmm_solve.diagonal().cwiseAbs().maxCoeff());
    hmm_solve.diagonal().array() += jitter;
    factor.compute(hmm_solve);
  }
  if (capture && factor.vectorD().size() > 0)
    capturePivots(factor.vectorD(), false);
  if (capture) trace.solve_jitter = jitter;
  if (factor.info() != Eigen::Success ||
      factor.vectorD().cwiseAbs().minCoeff() < 1e-14) {
    recordFailure("marginalization_oldest_block_solve_failed",
                  "M5_SOLVE", &hessian, &gradient,
                  capture ? &assembly : nullptr, nullptr, nullptr);
    return fail(reason, "marginalization_oldest_block_solve_failed");
  }
  correction_h = factor.solve(hmr);
  correction_b = factor.solve(bm);
  Eigen::MatrixXd new_information = hrr - hmr.transpose() * correction_h;
  Eigen::VectorXd new_gradient = br - hmr.transpose() * correction_b;
  if (capture) {
    trace.solve_finite = correction_h.allFinite() && correction_b.allFinite();
    const double epsilon = std::numeric_limits<double>::epsilon();
    trace.solve_h_backward_error = (hmm_solve * correction_h - hmr).norm() /
        std::max(hmr.norm(), epsilon);
    trace.solve_b_backward_error = (hmm_solve * correction_b - bm).norm() /
        std::max(bm.norm(), epsilon);
    trace.raw_schur_asymmetry_frobenius_norm =
        (new_information - new_information.transpose()).norm();
    trace.raw_schur =
        marginalizationMatrixStatsForDiagnostics(new_information);
    trace.new_gradient_evaluated = true;
    trace.new_gradient_finite = new_gradient.allFinite();
    trace.new_gradient_norm = new_gradient.norm();
    trace.new_gradient_max_abs = new_gradient.size() > 0 &&
        new_gradient.allFinite() ? new_gradient.cwiseAbs().maxCoeff() :
        std::numeric_limits<double>::infinity();
  }
  // A transpose expression must not read from a matrix as it is overwritten.
  const Eigen::MatrixXd symmetric_information =
      evaluateSymmetricInformation(new_information);
  new_information = symmetric_information;
  if (capture) {
    trace.symmetrized_schur =
        marginalizationMatrixStatsForDiagnostics(new_information);
    trace.new_prior = trace.symmetrized_schur;
  }
  if (!finitePsd(new_information, 1e-6)) {
    if (capture) {
      const char* first_bad = firstPsdFailureStage(trace);
      trace.first_bad_stage = std::string(first_bad) == "NONE"
          ? "M7_SYMMETRIZED_SCHUR" : first_bad;
    }
    recordFailure("marginalized_prior_not_finite_psd",
                  "M7_SYMMETRIZED_SCHUR", &hessian, &gradient,
                  capture ? &assembly : nullptr, &correction_h,
                  &correction_b);
    return fail(reason, "marginalized_prior_not_finite_psd");
  }
  if (!new_gradient.allFinite()) {
    if (capture) {
      const char* first_bad = firstPsdFailureStage(trace);
      trace.first_bad_stage = std::string(first_bad) == "NONE"
          ? "M8_NEW_GRADIENT" : first_bad;
    }
    recordFailure("marginalized_gradient_nonfinite", "M8_NEW_GRADIENT",
                  &hessian, &gradient, capture ? &assembly : nullptr,
                  &correction_h, &correction_b);
    return fail(reason, "marginalized_prior_not_finite_psd");
  }

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
          (0.5 * (alternate_information + alternate_information.transpose())).eval();
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

  PriorInformation replacement;
  replacement.information = std::move(new_information);
  replacement.gradient = std::move(new_gradient);
  commitMarginalPrior(std::move(replacement));
  summary_.retained_prior_cross_information_norm = retained_cross_information_norm;
  summary_.marginalization_solve_jitter = jitter;
  summary_.marginalization_jitter_information_delta_norm = jitter_information_delta_norm;
  summary_.marginalization_jitter_gradient_delta_norm = jitter_gradient_delta_norm;
  summary_.marginalization_performed = true;
  summary_.marginalization_psd = true;
  summary_.marginalization_status = jitter > 0.0
      ? "PASS_SCHUR_WITH_SOLVE_ONLY_JITTER" : "PASS_SCHUR";
  if (capture) {
    trace.oldest_state_removed = true;
    trace.nodes_after_attempt = states_.size();
    trace.span_after_attempt_s = states_.size() < 2 ? 0.0 :
        static_cast<double>(states_.back().stamp_ns - states_.front().stamp_ns) * 1e-9;
    trace.first_bad_stage = "NONE";
    trace.marginalization_result = "SUCCESS";
    marginalization_trace_.push_back(std::move(trace));
  }
  return true;
}

void FixedLagWindow::commitMarginalPrior(PriorInformation&& replacement) {
  const std::uint64_t removed_stamp = states_.front().stamp_ns;
  const bool feedback_was_current = summary_.prediction_feedback_ready &&
      optimized_revision_ == window_revision_;
  replacement.reference_states.assign(states_.begin() + 1, states_.end());
  replacement.valid = true;
  prior_ = std::move(replacement);
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
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
