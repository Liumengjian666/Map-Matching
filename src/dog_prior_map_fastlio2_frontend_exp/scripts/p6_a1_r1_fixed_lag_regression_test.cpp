#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {

bool require(bool condition, const std::string& label,
             const std::string& reason = {}) {
  if (condition) return true;
  std::cerr << "FAIL: " << label;
  if (!reason.empty()) std::cerr << ": " << reason;
  std::cerr << '\n';
  return false;
}

Eigen::Matrix3d exp3(const Eigen::Vector3d& value) {
  const double angle = value.norm();
  if (angle < 1e-14) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, value / angle).toRotationMatrix();
}

WindowState makeState(std::uint64_t stamp_ns, double x,
                      const Eigen::Vector3d& rotation_vector =
                          Eigen::Vector3d::Zero()) {
  WindowState state;
  state.stamp_ns = stamp_ns;
  state.rotation = exp3(rotation_vector);
  state.position = Eigen::Vector3d(x, 0.03 * x, -0.02 * x);
  state.velocity = Eigen::Vector3d(1.0, 0.03, -0.02);
  return state;
}

ImuPreintegratedMeasurement makeImu(std::uint64_t from_stamp_ns,
                                    std::uint64_t to_stamp_ns,
                                    double delta_x) {
  ImuPreintegratedMeasurement measurement;
  measurement.start_stamp_ns = from_stamp_ns;
  measurement.end_stamp_ns = to_stamp_ns;
  measurement.dt_s = static_cast<double>(to_stamp_ns - from_stamp_ns) * 1e-9;
  measurement.delta_position = Eigen::Vector3d(delta_x, 0.03 * delta_x,
                                                -0.02 * delta_x);
  measurement.delta_velocity.setZero();
  measurement.covariance = 0.04 * Matrix15d::Identity();
  measurement.valid = true;
  measurement.status = "SYNTHETIC_A1_R1";
  return measurement;
}

LidarWindowMeasurement makeLidar(std::uint64_t observation_id,
                                 const WindowState& state,
                                 const Eigen::Vector3d& position_offset =
                                     Eigen::Vector3d::Zero()) {
  LidarWindowMeasurement measurement;
  measurement.observation_id = observation_id;
  measurement.stamp_ns = state.stamp_ns;
  measurement.measured_position = state.position + position_offset;
  measurement.measured_rotation = state.rotation;
  measurement.covariance =
      0.03 * Eigen::Matrix<double, 6, 6>::Identity();
  measurement.reliable_rank = 6;
  measurement.valid = true;
  return measurement;
}

bool schurOldest(const Eigen::MatrixXd& hessian,
                 const Eigen::VectorXd& gradient,
                 Eigen::MatrixXd* information,
                 Eigen::VectorXd* reduced_gradient) {
  if (!information || !reduced_gradient || hessian.rows() <= 15) return false;
  const Eigen::Index retained = hessian.rows() - 15;
  const Eigen::MatrixXd hmm = hessian.topLeftCorner(15, 15);
  const Eigen::MatrixXd hmr = hessian.topRightCorner(15, retained);
  Eigen::LDLT<Eigen::MatrixXd> factor(hmm);
  if (factor.info() != Eigen::Success ||
      factor.vectorD().cwiseAbs().minCoeff() < 1e-12)
    return false;
  *information = hessian.bottomRightCorner(retained, retained) -
      hmr.transpose() * factor.solve(hmr);
  *information = 0.5 * (*information + information->transpose());
  *reduced_gradient = gradient.tail(retained) -
      hmr.transpose() * factor.solve(gradient.head(15));
  return information->allFinite() && reduced_gradient->allFinite();
}

bool informationConservationCase(bool conflicting_observation,
                                 double* hessian_error,
                                 double* gradient_error) {
  FixedLagOptions options;
  options.maximum_nodes = 2;
  options.maximum_duration_s = 100.0;
  ImuNoiseParameters noise;
  noise.gravity.setZero();
  FixedLagWindow window(options, noise);
  const WindowState x0 = makeState(1000000000ULL, 0.0,
                                   Eigen::Vector3d(0.08, -0.03, 0.02));
  WindowState x1 = makeState(2000000000ULL, 1.08,
                             Eigen::Vector3d(0.09, -0.025, 0.025));
  WindowState x2 = makeState(3000000000ULL, 2.02,
                             Eigen::Vector3d(0.10, -0.02, 0.03));
  std::string reason;
  if (!window.addState(x0, &reason) || !window.addState(x1, &reason) ||
      !window.addState(x2, &reason))
    return require(false, "information case state insertion", reason);
  Matrix15d prior_information = Matrix15d::Identity();
  prior_information.diagonal().segment<3>(0).setConstant(80.0);
  prior_information.diagonal().segment<3>(3).setConstant(50.0);
  Vector15d prior_gradient = Vector15d::Zero();
  prior_gradient.segment<3>(0) = Eigen::Vector3d(0.15, -0.08, 0.04);
  prior_gradient.segment<3>(3) = Eigen::Vector3d(-0.3, 0.2, -0.1);
  if (!window.setInitialPrior(x0.stamp_ns, prior_information, prior_gradient,
                              &reason) ||
      !window.addImuFactor(1, x0.stamp_ns, x1.stamp_ns,
                           makeImu(x0.stamp_ns, x1.stamp_ns, 1.0), &reason) ||
      !window.addImuFactor(2, x1.stamp_ns, x2.stamp_ns,
                           makeImu(x1.stamp_ns, x2.stamp_ns, 1.0), &reason))
    return require(false, "information case graph construction", reason);
  if (conflicting_observation &&
      !window.addLidarFactor(
          makeLidar(3, x2, Eigen::Vector3d(0.37, -0.19, 0.11)), &reason))
    return require(false, "conflicting retained observation", reason);

  Eigen::MatrixXd full_hessian;
  Eigen::VectorXd full_gradient;
  double full_cost = 0.0;
  if (!window.linearizedSystem(&full_hessian, &full_gradient, &full_cost,
                               &reason))
    return require(false, "full graph linearization", reason);
  Eigen::MatrixXd expected_hessian;
  Eigen::VectorXd expected_gradient;
  if (!schurOldest(full_hessian, full_gradient, &expected_hessian,
                   &expected_gradient))
    return require(false, "full graph Schur solve");
  if (conflicting_observation && expected_gradient.norm() < 1e-3)
    return require(false, "conflicting observation produced nonzero gradient");

  if (!window.marginalizeIfNeeded(&reason))
    return require(false, "subgraph marginalization", reason);
  Eigen::MatrixXd retained_hessian;
  Eigen::VectorXd retained_gradient;
  double retained_cost = 0.0;
  if (!window.linearizedSystem(&retained_hessian, &retained_gradient,
                               &retained_cost, &reason))
    return require(false, "retained graph linearization", reason);
  *hessian_error = (retained_hessian - expected_hessian).norm();
  *gradient_error = (retained_gradient - expected_gradient).norm();
  const WindowSummary summary = window.summary();
  return require(*hessian_error <= 1e-8 * std::max(1.0, expected_hessian.norm()),
                 "Schur Hessian information conservation") &&
      require(*gradient_error <=
                  1e-8 * std::max(1.0, expected_gradient.norm()),
              "Schur gradient information conservation") &&
      require(summary.imu_factor_count == 1,
              "retained X1-X2 factor remains active") &&
      require(summary.active_observation_id_count ==
                  summary.imu_factor_count + summary.lidar_factor_count +
                      summary.visual_factor_count,
              "active observation index equals active factors") &&
      require(summary.retired_observation_id_watermark == 1,
              "removed X0-X1 factor retired at watermark");
}

bool optimizerAndFeedbackTransactions() {
  std::string reason;
  FixedLagOptions converged_options;
  converged_options.maximum_nodes = 4;
  FixedLagWindow converged(converged_options);
  const WindowState state = makeState(1000000000ULL, 0.0);
  if (!converged.addState(state, &reason) ||
      !converged.setInitialPrior(state.stamp_ns, Matrix15d::Identity(),
                                 Vector15d::Zero(), &reason) ||
      !converged.optimize(&reason))
    return require(false, "zero-gradient optimizer convergence", reason);
  WindowState feedback;
  if (!require(converged.summary().optimizer_status ==
                   "CONVERGED_WITHOUT_STEP",
               "explicit converged-without-step status") ||
      !require(converged.predictionFeedbackSeed(&feedback, &reason),
               "feedback after validated convergence", reason))
    return false;
  if (!converged.addState(makeState(2000000000ULL, 1.0), &reason) ||
      !require(!converged.predictionFeedbackSeed(&feedback, &reason) &&
                   reason == "window_revision_not_optimized",
               "new state invalidates optimized feedback", reason))
    return false;

  FixedLagOptions failed_options;
  failed_options.maximum_step_norm = 0.0;
  failed_options.maximum_optimizer_iterations = 3;
  FixedLagWindow failed(failed_options);
  Vector15d nonzero_gradient = Vector15d::Zero();
  nonzero_gradient(3) = 1.0;
  if (!failed.addState(state, &reason) ||
      !failed.setInitialPrior(state.stamp_ns, Matrix15d::Identity(),
                              nonzero_gradient, &reason))
    return require(false, "failed optimizer construction", reason);
  const WindowState failed_before = *failed.latestState();
  if (!require(!failed.optimize(&reason), "all candidates must fail") ||
      !require(failed.summary().optimizer_status == "FAILED_ALL_CANDIDATES",
               "explicit failed-all-candidates status") ||
      !require((failed.latestState()->position - failed_before.position).norm() == 0.0,
               "failed optimizer leaves state unchanged") ||
      !require(!failed.predictionFeedbackSeed(&feedback, &reason),
               "failed optimizer cannot export feedback"))
    return false;

  FixedLagWindow invalid;
  int basis_calls = 0;
  LidarWindowMeasurement invalid_during_optimization = makeLidar(10, state);
  invalid_during_optimization.reliable_rank = 5;
  invalid_during_optimization.basis_relinearizer =
      [&basis_calls](const WindowState&, Eigen::Matrix<double, 6, 6>* basis,
                     int* rank, std::string* local_reason) {
        ++basis_calls;
        if (basis_calls > 1) {
          if (local_reason) *local_reason = "injected_linearization_failure";
          return false;
        }
        if (!basis || !rank) return false;
        *basis = Eigen::Matrix<double, 6, 6>::Identity();
        *rank = 5;
        return true;
      };
  if (!invalid.addState(state, &reason) ||
      !invalid.addLidarFactor(invalid_during_optimization, &reason) ||
      !require(!invalid.optimize(&reason), "invalid linear system rejection") ||
      !require(invalid.summary().optimizer_status == "INVALID_LINEAR_SYSTEM",
               "explicit invalid-linear-system status"))
    return false;

  std::vector<WindowState, Eigen::aligned_allocator<WindowState>> states{
      makeState(1, 0.0), makeState(2, 0.0)};
  states[1].position.x() = std::numeric_limits<double>::max();
  const auto backup = states;
  Eigen::VectorXd increment = Eigen::VectorXd::Zero(30);
  increment(3) = 0.25;
  increment(18) = std::numeric_limits<double>::max();
  if (!require(!applyGlobalIncrementAtomically(&states, increment, &reason),
               "mid-transaction increment failure") ||
      !require(states[0].position == backup[0].position &&
                   states[1].position == backup[1].position &&
                   states[0].rotation == backup[0].rotation &&
                   states[1].rotation == backup[1].rotation,
               "global increment rollback restores every state"))
    return false;
  return true;
}

bool repeatedMarginalizationAndIdLifecycle(double* maximum_active_ids) {
  std::string reason;
  FixedLagOptions capacity_options;
  capacity_options.maximum_active_observation_ids = 1;
  FixedLagWindow capacity_window(capacity_options);
  const WindowState capacity_state = makeState(1, 0.0);
  if (!capacity_window.addState(capacity_state, &reason) ||
      !capacity_window.addLidarFactor(makeLidar(1, capacity_state), &reason) ||
      !require(!capacity_window.addLidarFactor(makeLidar(2, capacity_state),
                                               &reason) &&
                   reason == "active_observation_id_capacity_exceeded",
               "active observation index hard bound", reason))
    return false;

  FixedLagOptions options;
  options.maximum_nodes = 3;
  options.maximum_duration_s = 100.0;
  options.maximum_optimizer_iterations = 5;
  ImuNoiseParameters noise;
  noise.gravity.setZero();
  FixedLagWindow window(options, noise);
  WindowState first = makeState(1000000000ULL, 0.12);
  if (!window.addState(first, &reason) ||
      !window.setInitialPrior(first.stamp_ns, 20.0 * Matrix15d::Identity(),
                              Vector15d::Zero(), &reason))
    return require(false, "repeated marginalization initialization", reason);
  std::uint64_t observation_id = 1000;
  for (int index = 1; index <= 12; ++index) {
    const std::uint64_t stamp = 1000000000ULL +
        static_cast<std::uint64_t>(index) * 100000000ULL;
    WindowState state = makeState(stamp, 0.1 * index + 0.02);
    if (!window.addState(state, &reason) ||
        !window.addImuFactor(++observation_id,
                             stamp - 100000000ULL, stamp,
                             makeImu(stamp - 100000000ULL, stamp, 0.1),
                             &reason) ||
        !window.addLidarFactor(makeLidar(++observation_id, state), &reason) ||
        !window.optimize(&reason))
      return require(false, "repeated optimize/marginalize cycle", reason);
    Eigen::MatrixXd expected_hessian;
    Eigen::VectorXd expected_gradient;
    if (window.states().size() > options.maximum_nodes) {
      Eigen::MatrixXd full_hessian;
      Eigen::VectorXd full_gradient;
      double full_cost = 0.0;
      if (!window.linearizedSystem(&full_hessian, &full_gradient, &full_cost,
                                   &reason) ||
          !schurOldest(full_hessian, full_gradient, &expected_hessian,
                       &expected_gradient))
        return require(false, "repeated pre-marginalization Schur", reason);
    }
    if (!window.marginalizeIfNeeded(&reason))
      return require(false, "repeated optimize/marginalize cycle", reason);
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
    double cost = 0.0;
    if (!window.linearizedSystem(&hessian, &gradient, &cost, &reason) ||
        !hessian.allFinite() || !gradient.allFinite())
      return require(false, "repeated retained linear system", reason);
    if (expected_hessian.size() != 0 &&
        (!require((hessian - expected_hessian).norm() <=
                      1e-8 * std::max(1.0, expected_hessian.norm()),
                  "repeated Schur information conservation") ||
         !require((gradient - expected_gradient).norm() <=
                      1e-8 * std::max(1.0, expected_gradient.norm()),
                  "repeated Schur gradient conservation")))
      return false;
    const WindowSummary summary = window.summary();
    *maximum_active_ids = std::max(
        *maximum_active_ids,
        static_cast<double>(summary.active_observation_id_count));
    if (!require(summary.active_observation_id_count ==
                     summary.imu_factor_count + summary.lidar_factor_count +
                         summary.visual_factor_count,
                 "repeated active factor/index accounting") ||
        !require(summary.prediction_feedback_ready,
                 "marginalization preserves validated feedback revision"))
      return false;
  }

  FixedLagOptions lifecycle_options;
  lifecycle_options.maximum_nodes = 1;
  lifecycle_options.maximum_duration_s = 100.0;
  FixedLagWindow lifecycle(lifecycle_options);
  WindowState state = makeState(1, 0.0);
  if (!lifecycle.addState(state, &reason) ||
      !lifecycle.setInitialPrior(state.stamp_ns, Matrix15d::Identity(),
                                 Vector15d::Zero(), &reason))
    return require(false, "ID lifecycle initialization", reason);
  constexpr std::uint64_t first_lifecycle_id = 100000;
  for (std::uint64_t index = 1; index <= 10000; ++index) {
    state = makeState(index + 1, 0.001 * static_cast<double>(index));
    if (!lifecycle.addState(state, &reason) ||
        !lifecycle.addLidarFactor(
            makeLidar(first_lifecycle_id + index, state), &reason) ||
        !lifecycle.marginalizeIfNeeded(&reason))
      return require(false, "10000 observation lifecycle", reason);
    const WindowSummary summary = lifecycle.summary();
    if (summary.active_observation_id_count > 1 ||
        summary.lidar_factor_count > 1)
      return require(false, "bounded active observation index");
  }
  LidarWindowMeasurement replay = makeLidar(first_lifecycle_id + 1,
                                             *lifecycle.latestState());
  if (!require(!lifecycle.addLidarFactor(replay, &reason) &&
                   reason == "retired_measurement_id",
               "retired observation cannot re-enter", reason))
    return false;
  return true;
}

bool priorCoordinateAndJitter(double* prior_gradient_error,
                              WindowSummary* jitter_summary) {
  std::string reason;
  FixedLagOptions options;
  options.maximum_optimizer_iterations = 1;
  FixedLagWindow prior_window(options);
  WindowState reference = makeState(10, 0.0,
      Eigen::Vector3d(0.35, -0.22, 0.18));
  Matrix15d information = 3.0 * Matrix15d::Identity();
  Vector15d gradient = Vector15d::Zero();
  gradient.head<3>() = Eigen::Vector3d(-0.25, 0.14, -0.08);
  if (!prior_window.addState(reference, &reason) ||
      !prior_window.setInitialPrior(reference.stamp_ns, information, gradient,
                                    &reason) ||
      !prior_window.optimize(&reason))
    return require(false, "nonzero rotation prior optimization", reason);
  const WindowState current = *prior_window.latestState();
  const Vector15d displacement = localDifference(current, reference);
  Matrix15d coordinate_jacobian = Matrix15d::Identity();
  constexpr double h = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    Vector15d increment = Vector15d::Zero();
    increment(axis) = h;
    WindowState plus = current;
    WindowState minus = current;
    if (!applyLocalIncrement(&plus, increment, &reason) ||
        !applyLocalIncrement(&minus, -increment, &reason))
      return require(false, "prior coordinate finite difference", reason);
    coordinate_jacobian.col(axis) =
        (localDifference(plus, reference) -
         localDifference(minus, reference)) / (2.0 * h);
  }
  Eigen::MatrixXd actual_hessian;
  Eigen::VectorXd actual_gradient;
  double actual_cost = 0.0;
  if (!prior_window.linearizedSystem(&actual_hessian, &actual_gradient,
                                     &actual_cost, &reason))
    return require(false, "nonzero rotation prior linearization", reason);
  const Matrix15d expected_hessian =
      coordinate_jacobian.transpose() * information * coordinate_jacobian;
  const Vector15d expected_gradient = coordinate_jacobian.transpose() *
      (information * displacement + gradient);
  *prior_gradient_error = (actual_gradient - expected_gradient).norm();
  if (!require((actual_hessian - expected_hessian).norm() < 1e-9,
               "fixed prior local-coordinate Hessian Jacobian") ||
      !require(*prior_gradient_error < 1e-9,
               "fixed prior local-coordinate gradient Jacobian"))
    return false;

  FixedLagOptions jitter_options;
  jitter_options.maximum_nodes = 1;
  jitter_options.maximum_duration_s = 100.0;
  FixedLagWindow jitter_window(jitter_options);
  const WindowState x0 = makeState(100, 0.0);
  const WindowState x1 = makeState(200, 0.2);
  VisualRelativeMeasurement visual;
  visual.observation_id = 1;
  visual.reference_stamp_ns = x0.stamp_ns;
  visual.current_stamp_ns = x1.stamp_ns;
  visual.reference_imu_translation = Eigen::Vector3d(0.1, 0.04, -0.03);
  visual.covariance = 0.01 * Eigen::Matrix3d::Identity();
  visual.valid = true;
  if (!jitter_window.addState(x0, &reason) ||
      !jitter_window.addState(x1, &reason) ||
      !jitter_window.addVisualFactor(visual, &reason) ||
      !jitter_window.marginalizeIfNeeded(&reason))
    return require(false, "solve-only jitter marginalization", reason);
  *jitter_summary = jitter_window.summary();
  return require(jitter_summary->marginalization_solve_jitter > 0.0,
                 "rank-deficient oldest block uses reported jitter") &&
      require(std::isfinite(
                  jitter_summary->marginalization_jitter_information_delta_norm) &&
                  std::isfinite(
                  jitter_summary->marginalization_jitter_gradient_delta_norm),
              "jitter sensitivity is finite and reported");
}

}  // namespace

int main() {
  double clean_hessian_error = 0.0;
  double clean_gradient_error = 0.0;
  if (!informationConservationCase(false, &clean_hessian_error,
                                   &clean_gradient_error))
    return 1;
  double noisy_hessian_error = 0.0;
  double noisy_gradient_error = 0.0;
  if (!informationConservationCase(true, &noisy_hessian_error,
                                   &noisy_gradient_error))
    return 2;
  if (!optimizerAndFeedbackTransactions()) return 3;
  double maximum_active_ids = 0.0;
  if (!repeatedMarginalizationAndIdLifecycle(&maximum_active_ids)) return 4;
  double prior_gradient_error = 0.0;
  WindowSummary jitter_summary;
  if (!priorCoordinateAndJitter(&prior_gradient_error, &jitter_summary))
    return 5;

  std::cout << "P6_A1_R1_FIXED_LAG_REGRESSION_PASS"
            << " clean_H_error=" << clean_hessian_error
            << " clean_g_error=" << clean_gradient_error
            << " noisy_H_error=" << noisy_hessian_error
            << " noisy_g_error=" << noisy_gradient_error
            << " max_active_ids=" << maximum_active_ids
            << " prior_gradient_error=" << prior_gradient_error
            << " jitter=" << jitter_summary.marginalization_solve_jitter
            << " jitter_H_delta="
            << jitter_summary.marginalization_jitter_information_delta_norm
            << " jitter_g_delta="
            << jitter_summary.marginalization_jitter_gradient_delta_norm
            << '\n';
  return 0;
}
