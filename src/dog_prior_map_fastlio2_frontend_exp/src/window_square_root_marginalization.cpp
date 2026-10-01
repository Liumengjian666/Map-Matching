#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <chrono>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

const SquareRootRows& FixedLagWindow::priorSquareRootRows() const {
  return prior_.square_root;
}

bool FixedLagWindow::linearizeSquareRootPriorRows(SquareRootRows* output,
                                                 std::string* reason) const {
  const auto fail = [&](const char* text) { if (reason) *reason = text; return false; };
  const Eigen::Index dimension = states_.size() * 15;
  if (!output || !prior_.valid || !prior_.square_root_authoritative ||
      prior_.reference_states.size() != states_.size() ||
      prior_.square_root.a.cols() != dimension ||
      prior_.square_root.a.rows() != prior_.square_root.b.size() ||
      !prior_.square_root.a.allFinite() || !prior_.square_root.b.allFinite())
    return fail("square_root_prior_invalid_or_not_authoritative");
  SquareRootRows current;
  current.a = prior_.square_root.a;
  Eigen::VectorXd displacement(dimension);
  const double h = std::max(1e-8, options_.finite_difference_step);
  for (std::size_t index = 0; index < states_.size(); ++index) {
    displacement.segment<15>(index*15) = localDifference(states_[index], prior_.reference_states[index]);
    Matrix15d chart = Matrix15d::Identity();
    for (int axis = 0; axis < 3; ++axis) {
      Vector15d delta = Vector15d::Zero(); delta(axis) = h;
      WindowState plus = states_[index], minus = states_[index];
      if (!applyLocalIncrement(&plus, delta, reason) ||
          !applyLocalIncrement(&minus, -delta, reason)) return false;
      chart.col(axis) = (localDifference(plus, prior_.reference_states[index]) -
          localDifference(minus, prior_.reference_states[index])) / (2*h);
    }
    current.a.middleCols(index*15, 15) = prior_.square_root.a.middleCols(index*15, 15) * chart;
  }
  current.b = prior_.square_root.a * displacement + prior_.square_root.b;
  if (!current.a.allFinite() || !current.b.allFinite()) return fail("square_root_chart_nonfinite");
  *output = std::move(current);
  return true;
}

bool FixedLagWindow::marginalizeOldestSquareRoot(std::string* reason) {
  std::string local_reason;
  if (!reason) reason = &local_reason;
  if (reason) reason->clear();
  const auto start = std::chrono::steady_clock::now();
  const bool capture = options_.capture_marginalization_diagnostics;
  MarginalizationTraceRecord trace;
  trace.backend = "SQUARE_ROOT_QR";
  trace.marginalization_enforcement_index = marginalization_enforcement_index_;
  trace.attempt_index_within_enforcement = marginalization_attempt_index_;
  trace.nodes_before_attempt = states_.size();
  trace.nodes_after_attempt = states_.size();
  if (!states_.empty()) {
    trace.oldest_state_stamp_ns = states_.front().stamp_ns;
    trace.latest_state_stamp_ns = states_.back().stamp_ns;
    trace.span_before_attempt_s = (states_.back().stamp_ns-states_.front().stamp_ns)*1e-9;
    trace.span_after_attempt_s = trace.span_before_attempt_s;
  }
  trace.trigger_duration_limit = marginalization_trigger_duration_limit_;
  trace.trigger_node_limit = marginalization_trigger_node_limit_;
  SquareRootRows stack;
  const auto fail = [&](const std::string& text) {
    if (reason) *reason = text;
    summary_.marginalization_status = text;
    if (capture) {
      trace.first_bad_stage = "SQUARE_ROOT_QR";
      trace.marginalization_result = "FAIL:" + text;
      marginalization_trace_.push_back(trace);
      auto& capsule = marginalization_failure_capsule_;
      capsule = MarginalizationFailureCapsule();
      capsule.valid = true; capsule.trace = trace;
      capsule.state_stamps_before_enforcement = enforcement_state_stamps_before_;
      for (const auto& state : states_) {
        capsule.state_stamps_at_attempt.push_back(state.stamp_ns);
        capsule.state_stamps_at_failure.push_back(state.stamp_ns);
      }
      capsule.prior_hash_before_enforcement_fnv1a64 = enforcement_prior_hash_before_;
      capsule.prior_hash_at_failure_fnv1a64 = marginalizationPriorHashForDiagnostics(prior_.information, prior_.gradient);
      capsule.factors_before_enforcement = enforcement_factor_counts_before_;
      capsule.factors_at_failure = {imu_factors_.size(),lidar_factors_.size(),visual_factors_.size()};
      capsule.square_root_stack = stack;
      capsule.incoming_square_root_prior = prior_.square_root;
      capsule.incoming_prior_information = prior_.information;
      capsule.incoming_prior_gradient = prior_.gradient;
    }
    return false;
  };
  if (states_.size() < 2) return fail("cannot_marginalize_single_state");
  trace.incoming_prior_valid = prior_.valid;
  trace.incoming_prior_hash_fnv1a64 = marginalizationPriorHashForDiagnostics(prior_.information, prior_.gradient);
  std::vector<SquareRootRows> pieces;
  FrozenLidarProjectionVector incident_projections;
  if (prior_.valid) {
    SquareRootRows rows;
    if (!linearizeSquareRootPriorRows(&rows, reason)) return fail(reason ? *reason : "square_root_prior_assembly_failed");
    pieces.push_back(std::move(rows));
  }
  const std::uint64_t oldest = states_.front().stamp_ns;
  const Eigen::Index dimension = states_.size() * 15;
  const auto append = [&](std::size_t i, const Eigen::MatrixXd& ji,
      std::size_t j, const Eigen::MatrixXd* jj, const Eigen::VectorXd& r,
      const Eigen::MatrixXd& covariance) {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(r.size(), dimension);
    jacobian.middleCols(i*15,15) = ji;
    if (jj) jacobian.middleCols(j*15,15) = *jj;
    SquareRootRows rows;
    if (!whitenSquareRootRows(jacobian,r,covariance,&rows,reason)) return false;
    pieces.push_back(std::move(rows));
    return true;
  };
  for (const auto& factor : imu_factors_) {
    if (factor.from_stamp_ns != oldest && factor.to_stamp_ns != oldest) continue;
    ++trace.incident_imu_factor_count;
    std::size_t i=0,j=0;
    if (!findStateIndex(factor.from_stamp_ns,&i) || !findStateIndex(factor.to_stamp_ns,&j))
      return fail("square_root_imu_endpoint_missing");
    Matrix15d ji,jj; Vector15d residual;
    if (!linearizeImuFactor(states_[i],states_[j],factor.measurement,imu_noise_,&ji,&jj,&residual,reason))
      return fail(reason ? *reason : "square_root_imu_linearization_failed");
    const Eigen::MatrixXd j_dynamic = jj;
    if (!append(i,ji,j,&j_dynamic,residual,factor.measurement.covariance)) return fail(*reason);
  }
  for (const auto& factor : lidar_factors_) {
    if (factor.measurement.stamp_ns != oldest) continue;
    ++trace.incident_lidar_factor_count;
    std::size_t i=0;
    if (!findStateIndex(factor.measurement.stamp_ns,&i)) return fail("square_root_lidar_endpoint_missing");
    Eigen::MatrixXd jacobian,covariance; Eigen::VectorXd residual;
    // Same current-state relinearization as the legacy marginalization path.
    FrozenLidarProjection projection;
    if (!freezeLidarProjection(states_[i],factor.measurement,&projection,reason) ||
        !linearizeLidarFactorWithFrozenProjection(states_[i],factor.measurement,
            projection,&residual,&jacobian,&covariance,reason))
      return fail(reason ? *reason : "square_root_lidar_linearization_failed");
    incident_projections.push_back(std::move(projection));
    if (!append(i,jacobian,0,nullptr,residual,covariance)) return fail(*reason);
  }
  for (const auto& factor : visual_factors_) {
    if (factor.measurement.reference_stamp_ns != oldest && factor.measurement.current_stamp_ns != oldest) continue;
    ++trace.incident_visual_factor_count;
    std::size_t i=0,j=0;
    if (!findStateIndex(factor.measurement.reference_stamp_ns,&i) || !findStateIndex(factor.measurement.current_stamp_ns,&j))
      return fail("square_root_visual_endpoint_missing");
    Eigen::MatrixXd ji,jj,covariance; Eigen::VectorXd residual;
    if (!linearizeSelectedVisualFactor(states_[i],states_[j],factor.measurement,&residual,&ji,&jj,&covariance,reason))
      return fail(reason ? *reason : "square_root_visual_linearization_failed");
    if (!append(i,ji,j,&jj,residual,covariance)) return fail(*reason);
  }
  Eigen::Index rows=0;
  for (const auto& piece : pieces) rows += piece.a.rows();
  stack.a = Eigen::MatrixXd::Zero(rows,dimension); stack.b.resize(rows);
  Eigen::Index offset=0;
  for (const auto& piece : pieces) {
    stack.a.middleRows(offset,piece.a.rows()) = piece.a;
    stack.b.segment(offset,piece.b.size()) = piece.b;
    offset += piece.a.rows();
  }
  PriorInformation replacement;
  if (!eliminateSquareRootOldest(stack,15,&replacement.square_root,&trace.qr,reason))
    return fail(reason ? *reason : "square_root_elimination_failed");
  replacement.square_root_authoritative = true;
  // These caches are NOT read by the production square-root assembly above.
  replacement.information = evaluateSymmetricInformation(replacement.square_root.a.transpose()*replacement.square_root.a);
  replacement.gradient = replacement.square_root.a.transpose()*replacement.square_root.b;
  if (!replacement.information.allFinite() || !replacement.gradient.allFinite())
    return fail("square_root_derived_cache_nonfinite");
  trace.qr_ms = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  if (capture) {
    trace.new_prior = marginalizationMatrixStatsForDiagnostics(replacement.information);
    trace.incoming_prior = marginalizationMatrixStatsForDiagnostics(prior_.information);
    // Explicit trace-only oracle on a complete copy. No shadow result is used
    // for acceptance, rank, lifecycle or the production prior.
    FixedLagWindow shadow = *this;
    shadow.options_.marginalization_backend = MarginalizationBackend::LEGACY_INFORMATION_SCHUR;
    // std::function copies can share captured external state. The shadow must
    // not re-invoke the production reliability callback, even on a window copy.
    for (auto& factor : shadow.lidar_factors_) {
      if (factor.measurement.stamp_ns != oldest) continue;
      for (const auto& projection : incident_projections) {
        if (projection.observation_id != factor.measurement.observation_id ||
            projection.stamp_ns != factor.measurement.stamp_ns) continue;
        factor.measurement.measurement_basis = projection.basis;
        factor.measurement.reliable_rank = projection.reliable_rank;
        factor.measurement.basis_relinearizer =
            [projection](const WindowState&,Matrix6d* basis,int* rank,std::string*) {
              *basis=projection.basis; *rank=projection.reliable_rank; return true;
            };
      }
    }
    std::string shadow_reason;
    const bool shadow_ok = shadow.marginalizeOldestInformation(&shadow_reason);
    trace.legacy_shadow_status = shadow_ok ? "SUCCESS" : "FAIL:"+shadow_reason;
    if (!shadow.marginalization_trace_.empty() &&
        shadow.marginalization_trace_.back().new_prior.available)
      trace.legacy_shadow_lambda_min = shadow.marginalization_trace_.back().new_prior.lambda_min;
  }
  commitMarginalPrior(std::move(replacement));
  summary_.marginalization_performed = true;
  summary_.marginalization_psd = true;  // finite A^T A by construction, no clamp.
  summary_.marginalization_status = "PASS_SQUARE_ROOT_QR";
  summary_.qr_marginalization_ms = trace.qr_ms;
  summary_.marginalization_solve_jitter = 0;
  summary_.marginalization_jitter_information_delta_norm = 0;
  summary_.marginalization_jitter_gradient_delta_norm = 0;
  if (capture) {
    trace.oldest_state_removed = true; trace.removed_state_stamp_ns = oldest;
    trace.nodes_after_attempt = states_.size();
    trace.span_after_attempt_s = (states_.back().stamp_ns-states_.front().stamp_ns)*1e-9;
    trace.marginalization_result = "SUCCESS";
    marginalization_trace_.push_back(std::move(trace));
  }
  return true;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
