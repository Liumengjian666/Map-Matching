#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

bool finiteState(const WindowState& state) {
  return state.stamp_ns > 0 && state.rotation.allFinite() &&
      state.position.allFinite() && state.velocity.allFinite() &&
      state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
      (state.rotation.transpose() * state.rotation -
       Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
      std::abs(state.rotation.determinant() - 1.0) < 1e-7;
}

bool finiteSpd(const Eigen::MatrixXd& covariance) {
  if (!covariance.allFinite() ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8)
    return false;
  Eigen::LLT<Eigen::MatrixXd> factor(covariance);
  return factor.info() == Eigen::Success;
}

void addBlock(Eigen::MatrixXd* hessian, Eigen::VectorXd* gradient,
              const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& residual,
              const Eigen::MatrixXd& covariance, Eigen::Index offset) {
  Eigen::LDLT<Eigen::MatrixXd> factor(covariance);
  const Eigen::MatrixXd information = factor.solve(
      Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
  const Eigen::MatrixXd weighted_jacobian = information * jacobian;
  hessian->block(offset, offset, jacobian.cols(), jacobian.cols()).noalias() +=
      jacobian.transpose() * weighted_jacobian;
  gradient->segment(offset, jacobian.cols()).noalias() +=
      jacobian.transpose() * information * residual;
}

void addCrossBlock(Eigen::MatrixXd* hessian, const Eigen::MatrixXd& left,
                   const Eigen::MatrixXd& right, const Eigen::MatrixXd& info,
                   Eigen::Index left_offset, Eigen::Index right_offset) {
  const Eigen::MatrixXd block = left.transpose() * info * right;
  hessian->block(left_offset, right_offset, left.cols(), right.cols()).noalias() += block;
  hessian->block(right_offset, left_offset, right.cols(), left.cols()).noalias() += block.transpose();
}

}  // namespace

const char* toString(OptimizerStatus status) {
  switch (status) {
    case OptimizerStatus::ACCEPTED_UPDATE:
      return "ACCEPTED_UPDATE";
    case OptimizerStatus::CONVERGED_WITHOUT_STEP:
      return "CONVERGED_WITHOUT_STEP";
    case OptimizerStatus::FAILED_ALL_CANDIDATES:
      return "FAILED_ALL_CANDIDATES";
    case OptimizerStatus::INVALID_LINEAR_SYSTEM:
      return "INVALID_LINEAR_SYSTEM";
    case OptimizerStatus::NOT_RUN:
    default:
      return "NOT_RUN";
  }
}

FixedLagWindow::FixedLagWindow(const FixedLagOptions& options,
                               const ImuNoiseParameters& imu_noise)
    : options_(options), imu_noise_(imu_noise) {}

bool FixedLagWindow::initializeWithPriorAtomic(
    const WindowState& state, const Matrix15d& information,
    const Vector15d& gradient, std::string* reason) {
  if (reason) reason->clear();
  if (!states_.empty() || prior_.valid)
    return fail(reason, "window_already_initialized");
  if (!finiteState(state) || !information.allFinite() || !gradient.allFinite() ||
      (information - information.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return fail(reason, "invalid_atomic_initial_state_or_prior");
  Eigen::SelfAdjointEigenSolver<Matrix15d> solver(information);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() < -1e-10)
    return fail(reason, "atomic_initial_prior_not_psd");
  states_.push_back(state);
  prior_.information = information;
  prior_.gradient = gradient;
  prior_.reference_states = states_;
  prior_.valid = true;
  summary_.latest_state_timestamp = state.stamp_ns;
  markWindowMutated();
  return true;
}

bool FixedLagWindow::addStateWithImuFactorAtomic(
    const WindowState& state, std::uint64_t observation_id,
    std::uint64_t from_stamp_ns,
    const ImuPreintegratedMeasurement& measurement, std::string* reason) {
  if (reason) reason->clear();
  if (!finiteState(state) || states_.empty() ||
      state.stamp_ns <= states_.back().stamp_ns ||
      from_stamp_ns != states_.back().stamp_ns ||
      measurement.start_stamp_ns != from_stamp_ns ||
      measurement.end_stamp_ns != state.stamp_ns || !measurement.valid ||
      !finiteSpd(measurement.covariance))
    return fail(reason, "invalid_atomic_state_or_imu_factor");
  if (!observationIdAvailable(observation_id, reason)) return false;
  if (prior_.valid) {
    const Eigen::Index dimension = static_cast<Eigen::Index>(states_.size() * 15);
    if (prior_.reference_states.size() != states_.size() ||
        prior_.information.rows() != dimension ||
        prior_.information.cols() != dimension ||
        prior_.gradient.size() != dimension)
      return fail(reason, "window_prior_invalid_before_atomic_extension");
  }

  if (prior_.valid) {
    const Eigen::Index old_dimension = prior_.information.rows();
    prior_.information.conservativeResize(old_dimension + 15,
                                          old_dimension + 15);
    prior_.information.rightCols(15).setZero();
    prior_.information.bottomRows(15).setZero();
    prior_.gradient.conservativeResize(old_dimension + 15);
    prior_.gradient.tail(15).setZero();
    prior_.reference_states.push_back(state);
  }
  states_.push_back(state);
  imu_factors_.push_back(
      {observation_id, from_stamp_ns, state.stamp_ns, measurement});
  active_observation_ids_.insert(observation_id);
  summary_.latest_state_timestamp = state.stamp_ns;
  markWindowMutated();
  return true;
}

void FixedLagWindow::markWindowMutated() {
  ++window_revision_;
  summary_.prediction_feedback_ready = false;
  summary_.prediction_feedback_status = "STALE_WINDOW_REVISION";
  summary_.optimizer_success = false;
  summary_.optimizer_status = "NOT_RUN";
}

bool FixedLagWindow::observationIdAvailable(std::uint64_t observation_id,
                                            std::string* reason) {
  if (observation_id == 0)
    return fail(reason, "invalid_measurement_id");
  if (observation_id <= retired_observation_id_watermark_ ||
      active_observation_ids_.count(observation_id) != 0) {
    ++summary_.duplicate_measurement_count;
    return fail(reason, observation_id <= retired_observation_id_watermark_
                            ? "retired_measurement_id"
                            : "duplicate_measurement_id");
  }
  if (active_observation_ids_.size() >=
      options_.maximum_active_observation_ids)
    return fail(reason, "active_observation_id_capacity_exceeded");
  return true;
}

void FixedLagWindow::retireObservationId(std::uint64_t observation_id) {
  active_observation_ids_.erase(observation_id);
  retired_observation_id_watermark_ =
      std::max(retired_observation_id_watermark_, observation_id);
}

bool FixedLagWindow::findStateIndex(std::uint64_t stamp_ns,
                                    std::size_t* index) const {
  if (!index) return false;
  for (std::size_t i = 0; i < states_.size(); ++i) {
    if (states_[i].stamp_ns == stamp_ns) {
      *index = i;
      return true;
    }
  }
  return false;
}

bool FixedLagWindow::addState(const WindowState& state, std::string* reason) {
  if (reason) reason->clear();
  if (!finiteState(state)) return fail(reason, "invalid_window_state");
  if (!states_.empty() && state.stamp_ns <= states_.back().stamp_ns)
    return fail(reason, "window_state_timestamps_not_strictly_increasing");
  if (prior_.valid) {
    const Eigen::Index old_dimension =
        static_cast<Eigen::Index>(prior_.reference_states.size() * 15);
    if (prior_.reference_states.size() != states_.size() ||
        prior_.information.rows() != old_dimension ||
        prior_.information.cols() != old_dimension ||
        prior_.gradient.size() != old_dimension)
      return fail(reason, "window_prior_invalid_before_state_extension");
    prior_.information.conservativeResize(old_dimension + 15,
                                          old_dimension + 15);
    prior_.information.rightCols(15).setZero();
    prior_.information.bottomRows(15).setZero();
    prior_.gradient.conservativeResize(old_dimension + 15);
    prior_.gradient.tail(15).setZero();
    prior_.reference_states.push_back(state);
  }
  states_.push_back(state);
  summary_.latest_state_timestamp = state.stamp_ns;
  markWindowMutated();
  return true;
}

bool FixedLagWindow::addImuFactor(
    std::uint64_t observation_id, std::uint64_t from_stamp_ns,
    std::uint64_t to_stamp_ns, const ImuPreintegratedMeasurement& measurement,
    std::string* reason) {
  if (reason) reason->clear();
  if (!observationIdAvailable(observation_id, reason)) return false;
  std::size_t from_index = 0, to_index = 0;
  if (!findStateIndex(from_stamp_ns, &from_index) ||
      !findStateIndex(to_stamp_ns, &to_index) || from_index >= to_index ||
      !measurement.valid)
    return fail(reason, "invalid_imu_factor_state_or_measurement");
  imu_factors_.push_back({observation_id, from_stamp_ns, to_stamp_ns, measurement});
  active_observation_ids_.insert(observation_id);
  markWindowMutated();
  return true;
}

bool FixedLagWindow::addLidarFactor(const LidarWindowMeasurement& measurement,
                                    std::string* reason) {
  if (reason) reason->clear();
  if (!observationIdAvailable(measurement.observation_id, reason)) return false;
  std::size_t index = 0;
  if (!findStateIndex(measurement.stamp_ns, &index))
    return fail(reason, "lidar_factor_state_not_in_window");
  if (!measurement.valid) {
    ++summary_.lidar_factor_skipped_count;
    summary_.lidar_factor_skipped_reason = measurement.skipped_reason;
    return fail(reason, measurement.skipped_reason.empty()
                         ? "lidar_factor_skipped_invalid_observation"
                         : measurement.skipped_reason.c_str());
  }
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian, covariance;
  std::string validation_reason;
  if (!linearizeLidarFactor(states_[index], measurement, &residual, &jacobian,
                            &covariance, &validation_reason)) {
    ++summary_.lidar_factor_skipped_count;
    summary_.lidar_factor_skipped_reason = validation_reason;
    if (reason) *reason = validation_reason;
    return false;
  }
  lidar_factors_.push_back({measurement});
  active_observation_ids_.insert(measurement.observation_id);
  summary_.last_lidar_reliable_rank = measurement.reliable_rank;
  markWindowMutated();
  return true;
}

bool FixedLagWindow::addVisualFactor(
    const VisualRelativeMeasurement& measurement, std::string* reason) {
  if (reason) reason->clear();
  if (!observationIdAvailable(measurement.observation_id, reason)) return false;
  std::size_t reference_index = 0, current_index = 0;
  if (!findStateIndex(measurement.reference_stamp_ns, &reference_index) ||
      !findStateIndex(measurement.current_stamp_ns, &current_index) ||
      reference_index >= current_index || !measurement.valid ||
      measurement.mode == VisualFactorMode::NOT_TRIGGERED ||
      measurement.selected_rank < 1 || measurement.selected_rank > 3)
    return fail(reason, "invalid_visual_factor_state_or_measurement");
  if (measurement.source_semantic != "METRIC_PNP_RELATIVE_TRANSLATION_FACTOR")
    return fail(reason, "unsupported_visual_factor_semantic");
  visual_factors_.push_back({measurement});
  active_observation_ids_.insert(measurement.observation_id);
  markWindowMutated();
  return true;
}

bool FixedLagWindow::setInitialPrior(std::uint64_t stamp_ns,
                                     const Matrix15d& information,
                                     const Vector15d& gradient,
                                     std::string* reason) {
  if (reason) reason->clear();
  if (states_.empty() || stamp_ns != states_.front().stamp_ns)
    return fail(reason, "initial_prior_must_target_oldest_state");
  if (prior_.valid) return fail(reason, "initial_prior_already_set");
  if (!information.allFinite() || !gradient.allFinite() ||
      (information - information.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return fail(reason, "invalid_initial_prior");
  Eigen::SelfAdjointEigenSolver<Matrix15d> solver(information);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().minCoeff() < -1e-10)
    return fail(reason, "initial_prior_not_psd");
  const Eigen::Index dimension = static_cast<Eigen::Index>(states_.size() * 15);
  prior_.information = Eigen::MatrixXd::Zero(dimension, dimension);
  prior_.gradient = Eigen::VectorXd::Zero(dimension);
  prior_.information.topLeftCorner<15, 15>() = information;
  prior_.gradient.head<15>() = gradient;
  prior_.reference_states = states_;
  prior_.valid = true;
  markWindowMutated();
  return true;
}

bool FixedLagWindow::linearizeSelected(LinearizationScope scope,
                                       Eigen::MatrixXd* hessian,
                                       Eigen::VectorXd* gradient, double* cost,
                                       std::string* reason) const {
  if (reason) reason->clear();
  if (!hessian || !gradient || !cost)
    return fail(reason, "null_window_linearization_output");
  const Eigen::Index dimension = static_cast<Eigen::Index>(states_.size() * 15);
  if (dimension == 0) return fail(reason, "empty_window");
  *hessian = Eigen::MatrixXd::Zero(dimension, dimension);
  *gradient = Eigen::VectorXd::Zero(dimension);
  *cost = 0.0;
  const std::uint64_t oldest_stamp = states_.front().stamp_ns;
  if (prior_.valid) {
    if (prior_.reference_states.size() != states_.size() ||
        prior_.information.rows() != dimension ||
        prior_.information.cols() != dimension ||
        prior_.gradient.size() != dimension)
      return fail(reason, "window_prior_dimension_mismatch");
    Eigen::VectorXd displacement(dimension);
    Eigen::MatrixXd coordinate_jacobian =
        Eigen::MatrixXd::Identity(dimension, dimension);
    const double h = std::max(1e-8, options_.finite_difference_step);
    for (std::size_t index = 0; index < states_.size(); ++index) {
      displacement.segment<15>(static_cast<Eigen::Index>(index * 15)) =
          localDifference(states_[index], prior_.reference_states[index]);
      for (int axis = 0; axis < 3; ++axis) {
        Vector15d increment = Vector15d::Zero();
        increment(axis) = h;
        WindowState plus = states_[index];
        WindowState minus = states_[index];
        if (!applyLocalIncrement(&plus, increment, reason) ||
            !applyLocalIncrement(&minus, -increment, reason))
          return false;
        coordinate_jacobian.block<15, 1>(
            static_cast<Eigen::Index>(index * 15),
            static_cast<Eigen::Index>(index * 15 + axis)) =
            (localDifference(plus, prior_.reference_states[index]) -
             localDifference(minus, prior_.reference_states[index])) /
            (2.0 * h);
      }
    }
    const Eigen::VectorXd prior_force =
        prior_.information * displacement + prior_.gradient;
    *hessian += coordinate_jacobian.transpose() * prior_.information *
        coordinate_jacobian;
    *gradient += coordinate_jacobian.transpose() * prior_force;
    *cost += displacement.dot(prior_.information * displacement) +
        2.0 * prior_.gradient.dot(displacement);
  }
  for (const ImuFactorRecord& factor_record : imu_factors_) {
    if (scope == LinearizationScope::FACTORS_TOUCHING_OLDEST &&
        factor_record.from_stamp_ns != oldest_stamp &&
        factor_record.to_stamp_ns != oldest_stamp)
      continue;
    std::size_t from_index = 0, to_index = 0;
    if (!findStateIndex(factor_record.from_stamp_ns, &from_index) ||
        !findStateIndex(factor_record.to_stamp_ns, &to_index))
      return fail(reason, "imu_factor_state_removed_without_marginalization");
    Eigen::Matrix<double, 15, 15> jacobian_from, jacobian_to;
    Vector15d residual;
    if (!linearizeImuFactor(states_[from_index], states_[to_index],
                            factor_record.measurement, imu_noise_,
                            &jacobian_from, &jacobian_to, &residual, reason))
      return false;
    if (!finiteSpd(factor_record.measurement.covariance))
      return fail(reason, "imu_factor_covariance_not_spd");
    Eigen::LDLT<Matrix15d> factor(factor_record.measurement.covariance);
    const Matrix15d information = factor.solve(Matrix15d::Identity());
    const Eigen::Index from_offset = static_cast<Eigen::Index>(from_index * 15);
    const Eigen::Index to_offset = static_cast<Eigen::Index>(to_index * 15);
    addBlock(hessian, gradient, jacobian_from, residual,
             factor_record.measurement.covariance, from_offset);
    addBlock(hessian, gradient, jacobian_to, residual,
             factor_record.measurement.covariance, to_offset);
    addCrossBlock(hessian, jacobian_from, jacobian_to, information,
                  from_offset, to_offset);
    *cost += residual.dot(information * residual);
  }
  for (const LidarFactorRecord& factor_record : lidar_factors_) {
    if (scope == LinearizationScope::FACTORS_TOUCHING_OLDEST &&
        factor_record.measurement.stamp_ns != oldest_stamp)
      continue;
    std::size_t state_index = 0;
    if (!findStateIndex(factor_record.measurement.stamp_ns, &state_index))
      return fail(reason, "lidar_factor_state_removed_without_marginalization");
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian, covariance;
    if (!linearizeLidarFactor(states_[state_index], factor_record.measurement,
                              &residual, &jacobian, &covariance, reason))
      return false;
    if (!finiteSpd(covariance)) return fail(reason, "lidar_factor_covariance_not_spd");
    addBlock(hessian, gradient, jacobian, residual, covariance,
             static_cast<Eigen::Index>(state_index * 15));
    *cost += residual.dot(covariance.ldlt().solve(residual));
  }
  for (const VisualFactorRecord& factor_record : visual_factors_) {
    if (scope == LinearizationScope::FACTORS_TOUCHING_OLDEST &&
        factor_record.measurement.reference_stamp_ns != oldest_stamp &&
        factor_record.measurement.current_stamp_ns != oldest_stamp)
      continue;
    std::size_t reference_index = 0, current_index = 0;
    if (!findStateIndex(factor_record.measurement.reference_stamp_ns,
                        &reference_index) ||
        !findStateIndex(factor_record.measurement.current_stamp_ns,
                        &current_index))
      return fail(reason, "visual_factor_state_removed_without_marginalization");
    Eigen::VectorXd residual;
    Eigen::MatrixXd jacobian_reference, jacobian_current, covariance;
    if (!linearizeSelectedVisualFactor(
            states_[reference_index], states_[current_index],
            factor_record.measurement, &residual, &jacobian_reference,
            &jacobian_current, &covariance, reason))
      return false;
    if (!finiteSpd(covariance))
      return fail(reason, "visual_factor_covariance_not_spd");
    Eigen::LDLT<Eigen::MatrixXd> factor(covariance);
    const Eigen::MatrixXd information = factor.solve(
        Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
    const Eigen::Index reference_offset = static_cast<Eigen::Index>(reference_index * 15);
    const Eigen::Index current_offset = static_cast<Eigen::Index>(current_index * 15);
    addBlock(hessian, gradient, jacobian_reference, residual,
             covariance, reference_offset);
    addBlock(hessian, gradient, jacobian_current, residual,
             covariance, current_offset);
    addCrossBlock(hessian, jacobian_reference, jacobian_current, information,
                  reference_offset, current_offset);
    *cost += residual.dot(information * residual);
  }
  *hessian = 0.5 * (*hessian + hessian->transpose());
  if (!hessian->allFinite() || !gradient->allFinite() || !std::isfinite(*cost))
    return fail(reason, "nonfinite_window_linear_system");
  return true;
}

bool FixedLagWindow::linearize(Eigen::MatrixXd* hessian,
                               Eigen::VectorXd* gradient, double* cost,
                               std::string* reason) const {
  return linearizeSelected(LinearizationScope::ALL_FACTORS, hessian, gradient,
                           cost, reason);
}

bool FixedLagWindow::linearizeMarginalizationSubgraph(
    Eigen::MatrixXd* hessian, Eigen::VectorXd* gradient, double* cost,
    std::string* reason) const {
  return linearizeSelected(LinearizationScope::FACTORS_TOUCHING_OLDEST,
                           hessian, gradient, cost, reason);
}

double FixedLagWindow::objective(std::string* reason) const {
  WindowLinearSystem system;
  if (!blockLinearizedSystem(&system, reason))
    return std::numeric_limits<double>::infinity();
  return system.cost;
}

bool applyGlobalIncrementAtomically(
    std::vector<WindowState, Eigen::aligned_allocator<WindowState>>* states,
    const Eigen::VectorXd& increment, std::string* reason) {
  if (!states || increment.size() !=
          static_cast<Eigen::Index>(states->size() * 15) ||
      !increment.allFinite())
    return fail(reason, "invalid_global_window_increment");
  const auto backup = *states;
  for (std::size_t index = 0; index < states->size(); ++index) {
    if (!applyLocalIncrement(&(*states)[index],
                             increment.segment<15>(index * 15), reason)) {
      *states = backup;
      return false;
    }
  }
  return true;
}

bool FixedLagWindow::applyGlobalIncrement(const Eigen::VectorXd& increment,
                                          std::string* reason) {
  return applyGlobalIncrementAtomically(&states_, increment, reason);
}

bool FixedLagWindow::optimize(std::string* reason) {
  if (reason) reason->clear();
  if (states_.empty()) return fail(reason, "cannot_optimize_empty_window");
  const auto optimization_start_states = states_;
  summary_.prediction_feedback_ready = false;
  summary_.prediction_feedback_status = "OPTIMIZATION_IN_PROGRESS";
  summary_.optimizer_status = toString(OptimizerStatus::NOT_RUN);
  summary_.solver_status = "NOT_RUN";
  summary_.linearization_ms = summary_.solve_ms = summary_.rank_diagnostic_ms = 0;
  summary_.hessian_numerical_rank = -1;  // Not measured when diagnostics are off.
  double current_cost = objective(reason);
  if (!std::isfinite(current_cost)) {
    states_ = optimization_start_states;
    summary_.optimizer_status =
        toString(OptimizerStatus::INVALID_LINEAR_SYSTEM);
    summary_.optimizer_success = false;
    summary_.prediction_feedback_status = "INVALID_LINEAR_SYSTEM";
    return false;
  }
  summary_.optimizer_initial_cost = current_cost;
  summary_.optimizer_final_cost = current_cost;
  summary_.optimizer_iterations = 0;
  summary_.optimizer_success = false;
  bool accepted_update = false;
  bool converged_without_step = false;
  double damping = std::max(1e-12, options_.initial_damping);
  for (int iteration = 0; iteration < options_.maximum_optimizer_iterations;
       ++iteration) {
    const auto linearization_start = std::chrono::steady_clock::now();
    WindowLinearSystem system;
    if (!blockLinearizedSystem(&system, reason)) {
      states_ = optimization_start_states;
      summary_.optimizer_status =
          toString(OptimizerStatus::INVALID_LINEAR_SYSTEM);
      summary_.optimizer_success = false;
      summary_.prediction_feedback_status = "INVALID_LINEAR_SYSTEM";
      return false;
    }
    summary_.linearization_ms += std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-linearization_start).count();
    const auto& gradient = system.gradient;
    current_cost = system.cost;
    summary_.hessian_dimension = gradient.size();
    if (options_.debug_rank_diagnostic) {
      const auto rank_start = std::chrono::steady_clock::now();
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> hessian_solver(system.dense());
      if (hessian_solver.info() == Eigen::Success &&
          hessian_solver.eigenvalues().allFinite()) {
        const double scale = std::max(1.0,
            hessian_solver.eigenvalues().cwiseAbs().maxCoeff());
        summary_.hessian_numerical_rank =
            (hessian_solver.eigenvalues().array() > 1e-9 * scale).count();
      }
      summary_.rank_diagnostic_ms += std::chrono::duration<double,std::milli>(
          std::chrono::steady_clock::now()-rank_start).count();
    }
    if (gradient.lpNorm<Eigen::Infinity>() <=
        options_.gradient_convergence_tolerance) {
      converged_without_step = !accepted_update;
      break;
    }
    const auto solve_start = std::chrono::steady_clock::now();
    Eigen::VectorXd step;
    const bool solved = solveWindowLinearSystem(system, damping,
        options_.solver_backend, &step, &summary_.solver_status);
    summary_.solve_ms += std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-solve_start).count();
    if (summary_.solver_status == "SPARSE_SOLVER_FALLBACK_DENSE")
      ++summary_.sparse_solver_fallback_count;
    if (!solved) {
      states_ = optimization_start_states;
      summary_.optimizer_status =
          toString(OptimizerStatus::INVALID_LINEAR_SYSTEM);
      summary_.optimizer_success = false;
      summary_.prediction_feedback_status = "INVALID_LINEAR_SYSTEM";
      return fail(reason, "window_normal_equation_factorization_failed");
    }
    if (!step.allFinite()) {
      states_ = optimization_start_states;
      summary_.optimizer_status =
          toString(OptimizerStatus::INVALID_LINEAR_SYSTEM);
      summary_.optimizer_success = false;
      summary_.prediction_feedback_status = "INVALID_LINEAR_SYSTEM";
      return fail(reason, "nonfinite_window_optimizer_step");
    }
    if (step.norm() > options_.maximum_step_norm)
      step *= options_.maximum_step_norm / step.norm();
    const auto backup = states_;
    if (!applyGlobalIncrement(step, reason)) {
      states_ = optimization_start_states;
      summary_.optimizer_status =
          toString(OptimizerStatus::INVALID_LINEAR_SYSTEM);
      summary_.optimizer_success = false;
      summary_.prediction_feedback_status = "INVALID_LINEAR_SYSTEM";
      return false;
    }
    const double candidate_cost = objective(reason);
    if (std::isfinite(candidate_cost) && candidate_cost < current_cost) {
      current_cost = candidate_cost;
      damping = std::max(1e-12, damping * 0.3);
      accepted_update = true;
    } else {
      states_ = backup;
      damping = std::min(1e12, damping * 10.0);
      if (reason) reason->clear();
    }
    summary_.optimizer_iterations = iteration + 1;
    summary_.optimizer_final_cost = current_cost;
    if (step.norm() < 1e-8) break;
  }
  summary_.latest_state_timestamp = states_.back().stamp_ns;
  if (accepted_update || converged_without_step) {
    summary_.optimizer_success = true;
    summary_.optimizer_status = toString(accepted_update
        ? OptimizerStatus::ACCEPTED_UPDATE
        : OptimizerStatus::CONVERGED_WITHOUT_STEP);
    optimized_revision_ = window_revision_;
    summary_.prediction_feedback_ready = finiteState(states_.back());
    summary_.prediction_feedback_status = summary_.prediction_feedback_ready
        ? "LATEST_OPTIMIZED_STATE_READY" : "LATEST_STATE_INVALID";
    return summary_.prediction_feedback_ready;
  }
  summary_.optimizer_success = false;
  summary_.optimizer_status =
      toString(OptimizerStatus::FAILED_ALL_CANDIDATES);
  summary_.prediction_feedback_ready = false;
  summary_.prediction_feedback_status = "FAILED_ALL_CANDIDATES";
  return fail(reason, "all_optimizer_candidates_rejected");
}

bool FixedLagWindow::marginalizeIfNeeded(std::string* reason) {
  if (reason) reason->clear();
  if (states_.empty()) return fail(reason, "cannot_marginalize_empty_window");
  while (states_.size() > options_.maximum_nodes ||
         (states_.back().stamp_ns - states_.front().stamp_ns) * 1e-9 >
             options_.maximum_duration_s) {
    if (!marginalizeOldest(reason)) return false;
  }
  summary_.window_node_count = states_.size();
  summary_.window_time_span_s = states_.size() < 2 ? 0.0 :
      static_cast<double>(states_.back().stamp_ns - states_.front().stamp_ns) * 1e-9;
  summary_.imu_factor_count = imu_factors_.size();
  summary_.lidar_factor_count = lidar_factors_.size();
  summary_.visual_factor_count = visual_factors_.size();
  return true;
}

const std::vector<WindowState, Eigen::aligned_allocator<WindowState>>&
FixedLagWindow::states() const { return states_; }

const WindowState* FixedLagWindow::stateAt(std::uint64_t stamp_ns) const {
  std::size_t index = 0;
  return findStateIndex(stamp_ns, &index) ? &states_[index] : nullptr;
}

const WindowState* FixedLagWindow::latestState() const {
  return states_.empty() ? nullptr : &states_.back();
}

WindowSummary FixedLagWindow::summary() const {
  WindowSummary result = summary_;
  result.window_node_count = states_.size();
  result.window_time_span_s = states_.size() < 2 ? 0.0 :
      static_cast<double>(states_.back().stamp_ns - states_.front().stamp_ns) * 1e-9;
  result.imu_factor_count = imu_factors_.size();
  result.lidar_factor_count = lidar_factors_.size();
  result.visual_factor_count = visual_factors_.size();
  result.active_observation_id_count = active_observation_ids_.size();
  result.retired_observation_id_watermark =
      retired_observation_id_watermark_;
  result.window_revision = window_revision_;
  result.optimized_revision = optimized_revision_;
  result.latest_state_timestamp = states_.empty() ? 0 : states_.back().stamp_ns;
  return result;
}

const Eigen::MatrixXd& FixedLagWindow::priorInformation() const {
  return prior_.information;
}

const Eigen::VectorXd& FixedLagWindow::priorGradient() const {
  return prior_.gradient;
}

bool FixedLagWindow::linearizedSystem(Eigen::MatrixXd* hessian,
                                      Eigen::VectorXd* gradient, double* cost,
                                      std::string* reason) const {
  return linearize(hessian, gradient, cost, reason);
}

namespace {
bool finishMarginalCovariance(const Eigen::MatrixXd& columns,
    const Eigen::Matrix3d& rotation, WindowMarginalCovariance* output,
    std::string* reason) {
  const auto unavailable = [&](const char* detail) {
    if (reason) *reason = output->status + ":" + detail;
    return false;
  };
  output->covariance15 = 0.5 *
      (columns.bottomRows(15) + columns.bottomRows(15).transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Matrix15d> eigen(output->covariance15);
  if (eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() < -1e-10 ||
      !output->covariance15.allFinite()) return unavailable("MARGINAL_NOT_PSD");
  Eigen::Matrix<double, 6, 15> map_product = Eigen::Matrix<double, 6, 15>::Zero();
  map_product.block<3, 3>(0, 0) = rotation;
  map_product.block<3, 3>(3, 3).setIdentity();
  output->map_pose_covariance6 = map_product * output->covariance15 * map_product.transpose();
  output->map_pose_covariance6 = 0.5 * (output->map_pose_covariance6 +
      output->map_pose_covariance6.transpose()).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6d> map_eigen(output->map_pose_covariance6);
  if (!output->map_pose_covariance6.allFinite() || map_eigen.info()!=Eigen::Success ||
      map_eigen.eigenvalues().minCoeff() < -1e-10)
    return unavailable("MAP_COVARIANCE_NOT_PSD");
  output->valid = true;
  output->status = "WINDOW_MARGINAL_COVARIANCE_AVAILABLE";
  return true;
}
}  // namespace

bool FixedLagWindow::latestMarginalCovariance(
    WindowMarginalCovariance* output, std::string* reason) const {
  const auto started = std::chrono::steady_clock::now();
  struct Timer {
    double* milliseconds;
    std::chrono::steady_clock::time_point started;
    ~Timer() { *milliseconds = std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-started).count(); }
  } timer{&summary_.marginal_covariance_ms, started};
  ++summary_.marginal_covariance_requests;
  if (reason) reason->clear();
  if (!output) return fail(reason, "null_window_marginal_covariance_output");
  *output = WindowMarginalCovariance();
  WindowLinearSystem system;
  std::string detail;
  if (!blockLinearizedSystem(&system, &detail)) {
    if (reason) *reason = output->status + ":" + detail;
    return false;
  }
  Eigen::MatrixXd columns;
  if (!solveLatestMarginalColumnsSparse(system, &columns,
      &output->normalized_backward_error, &detail)) {
    if (reason) *reason = output->status + ":" + detail;
    return false;
  }
  return finishMarginalCovariance(columns, states_.back().rotation, output, reason);
}

bool FixedLagWindow::latestMarginalCovarianceDenseReferenceForTest(
    WindowMarginalCovariance* output, std::string* reason) const {
  ++summary_.dense_marginal_reference_requests;
  if (reason) reason->clear();
  if (!output) return fail(reason, "null_window_marginal_covariance_output");
  *output = WindowMarginalCovariance();
  const auto unavailable = [&](const char* detail) {
    output->status = "WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE";
    if (reason) *reason = output->status + ":" + detail;
    return false;
  };
  if (states_.empty()) return unavailable("EMPTY_WINDOW");
  Eigen::MatrixXd hessian;
  Eigen::VectorXd gradient;
  double cost = 0.0;
  if (!linearize(&hessian, &gradient, &cost, nullptr) ||
      hessian.rows() != static_cast<Eigen::Index>(15 * states_.size()) ||
      !hessian.allFinite()) return unavailable("INVALID_LINEARIZATION");
  // No optimizer damping or artificial diagonal floor is admitted here.
  hessian = 0.5 * (hessian + hessian.transpose()).eval();
  if ((hessian.diagonal().array() <= 0.0).any())
    return unavailable("NONPOSITIVE_HESSIAN_DIAGONAL");
  // Equilibrate coordinates, without changing the information matrix. Tight
  // IMU factors and weak priors otherwise have very different numeric scales.
  const Eigen::VectorXd scale = hessian.diagonal().array().sqrt().inverse();
  const Eigen::MatrixXd balanced =
      scale.asDiagonal() * hessian * scale.asDiagonal();
  Eigen::LLT<Eigen::MatrixXd> factor(balanced);
  if (factor.info() != Eigen::Success) return unavailable("HESSIAN_NOT_SPD");
  if (factor.matrixL().toDenseMatrix().diagonal().array().square().minCoeff() <= 1e-12)
    return unavailable("NUMERICALLY_SINGULAR_HESSIAN");
  Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(hessian.rows(), 15);
  selector.bottomRows(15).setIdentity();
  const Eigen::MatrixXd columns = scale.asDiagonal() *
      factor.solve(scale.asDiagonal() * selector);
  const double backward_error = (hessian * columns - selector).norm() /
      (hessian.norm() * columns.norm() + selector.norm());
  if (factor.info() != Eigen::Success || !columns.allFinite() ||
      !std::isfinite(backward_error) || backward_error > 1e-10)
    return unavailable("SOLVE_RESIDUAL");
  output->normalized_backward_error = backward_error;
  return finishMarginalCovariance(columns, states_.back().rotation, output, reason);
}

bool FixedLagWindow::predictionFeedbackSeed(WindowState* output,
                                             std::string* reason) const {
  if (reason) reason->clear();
  if (!output || states_.empty()) return fail(reason, "prediction_feedback_not_ready");
  if (!summary_.prediction_feedback_ready ||
      optimized_revision_ != window_revision_)
    return fail(reason, "window_revision_not_optimized");
  if (!finiteState(states_.back()))
    return fail(reason, "latest_optimized_state_not_finite");
  *output = states_.back();
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
