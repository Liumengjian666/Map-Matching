#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using dog_prior_map_fastlio2_frontend_exp::Matrix6d;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

WindowState makeState(double x = 0.2) {
  WindowState state;
  state.stamp_ns = 1'000'000'000ULL;
  state.position = Eigen::Vector3d(x, -0.1, 0.04);
  state.rotation = Eigen::AngleAxisd(0.22, Eigen::Vector3d::UnitZ())
                       .toRotationMatrix();
  return state;
}

struct WindowFixture {
  FixedLagWindow window;
  std::shared_ptr<int> basis_calls;
};

WindowFixture makeDirectionalWindow(bool capture_trace, double initial_x = 0.2) {
  FixedLagOptions options;
  options.maximum_duration_s = 10.0;
  options.maximum_optimizer_iterations = 5;
  options.maximum_step_norm = 0.5;
  options.gradient_convergence_tolerance = 1e-12;
  options.capture_optimizer_trace = capture_trace;
  FixedLagWindow window(options);

  const WindowState initial = makeState(initial_x);
  Matrix15d prior_information = 0.2 * Matrix15d::Identity();
  prior_information(3, 3) = 1.4;
  prior_information(3, 4) = prior_information(4, 3) = 0.12;
  Vector15d prior_gradient = Vector15d::Zero();
  prior_gradient(3) = -0.18;
  prior_gradient(4) = 0.04;
  require(window.initializeWithPriorAtomic(initial, prior_information,
                                            prior_gradient),
          "directional window prior initialization");

  LidarWindowMeasurement lidar;
  lidar.observation_id = 501;
  lidar.stamp_ns = initial.stamp_ns;
  lidar.measured_position = Eigen::Vector3d(0.72, 0.13, -0.05);
  lidar.measured_rotation =
      Eigen::AngleAxisd(0.43, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  lidar.covariance = Matrix6d::Zero();
  lidar.covariance.diagonal() << 0.08, 0.12, 0.2, 0.03, 0.07, 0.8;
  lidar.reliable_rank = 5;
  lidar.valid = true;
  auto calls = std::make_shared<int>(0);
  lidar.basis_relinearizer = [calls](const WindowState& state,
      Matrix6d* basis, int* rank, std::string* reason) {
    ++*calls;
    if (!basis || !rank) {
      if (reason) *reason = "null_basis_output";
      return false;
    }
    basis->setIdentity();
    const double angle = 0.7 * state.position.x();
    (*basis)(3, 3) = std::cos(angle);
    (*basis)(5, 3) = std::sin(angle);
    (*basis)(3, 5) = -std::sin(angle);
    (*basis)(5, 5) = std::cos(angle);
    *rank = 5;
    return true;
  };
  std::string reason;
  require(window.addLidarFactor(lidar, &reason),
          "directional LiDAR factor: " + reason);
  return {std::move(window), std::move(calls)};
}

double relativeError(double value, double reference) {
  return std::abs(value - reference) /
      std::max({1e-12, std::abs(value), std::abs(reference)});
}

void frozenSurrogateDerivativeAndCovariance() {
  auto fixture = makeDirectionalWindow(false);
  auto& window = fixture.window;
  const int calls_before_snapshot = *fixture.basis_calls;
  LidarIterationSnapshot snapshot;
  std::string reason;
  require(window.buildLidarIterationSnapshot(&snapshot, &reason, 17),
          "build initial projection snapshot: " + reason);
  require(snapshot.generation == 17 && snapshot.projections.size() == 1,
          "snapshot generation or factor count mismatch");
  require(*fixture.basis_calls == calls_before_snapshot + 1,
          "degenerate basis callback was not called exactly once at freeze");

  WindowLinearSystem system;
  require(window.blockLinearizedSystemWithLidarSnapshot(snapshot, &system,
              &reason), "frozen snapshot assembly: " + reason);
  const Eigen::VectorXd gradient = system.gradient;
  require(std::abs(gradient(3)) > 1e-5,
          "synthetic direction has no position-x gradient");
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(15);
  direction(3) = 1.0;
  const double model = 2.0 * gradient.dot(direction);
  constexpr double epsilon = 1e-6;
  WindowStateVector plus = window.states();
  WindowStateVector minus = window.states();
  Vector15d step = Vector15d::Zero();
  step(3) = epsilon;
  require(applyLocalIncrement(&plus.front(), step, &reason) &&
              applyLocalIncrement(&minus.front(), -step, &reason),
          "directional candidate state construction: " + reason);
  ObjectiveBreakdown frozen_plus, frozen_minus;
  require(window.objectiveAtStatesWithLidarSnapshotForDebug(
              plus, snapshot, &frozen_plus, &reason) &&
              window.objectiveAtStatesWithLidarSnapshotForDebug(
                  minus, snapshot, &frozen_minus, &reason),
          "frozen candidate objective: " + reason);
  require(*fixture.basis_calls == calls_before_snapshot + 1,
          "candidate objective relinearized the frozen basis");
  const double frozen_fd = (frozen_plus.total_cost - frozen_minus.total_cost) /
      (2.0 * epsilon);
  require(relativeError(frozen_fd, model) <= 1e-8,
          "frozen surrogate FD does not match 2*g^T*d");

  ObjectiveBreakdown dynamic_plus, dynamic_minus;
  require(window.objectiveBreakdownAtStatesForDebug(plus, false,
              &dynamic_plus, &reason) &&
              window.objectiveBreakdownAtStatesForDebug(minus, false,
              &dynamic_minus, &reason),
          "legacy dynamic-B diagnostic objective: " + reason);
  const double dynamic_fd = (dynamic_plus.total_cost - dynamic_minus.total_cost) /
      (2.0 * epsilon);
  require(relativeError(dynamic_fd, model) > 1e-5,
          "state-dependent synthetic basis failed to reproduce the old mismatch");

  const auto projection_before = snapshot.projections.front();
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian, covariance_before, covariance_after;
  const auto& measurement_id = projection_before.observation_id;
  require(measurement_id == 501,
          "unexpected snapshot measurement identity");
  const WindowState& current = plus.front();
  LidarWindowMeasurement measurement;
  measurement.observation_id = 501;
  measurement.stamp_ns = current.stamp_ns;
  measurement.measured_position = Eigen::Vector3d(0.72, 0.13, -0.05);
  measurement.measured_rotation =
      Eigen::AngleAxisd(0.43, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  measurement.covariance = Matrix6d::Zero();
  measurement.covariance.diagonal() << 0.08, 0.12, 0.2, 0.03, 0.07, 0.8;
  measurement.reliable_rank = 5;
  measurement.valid = true;
  measurement.basis_relinearizer = [fixture_calls = fixture.basis_calls](
      const WindowState&, Matrix6d*, int*, std::string*) {
    ++*fixture_calls;
    return false;
  };
  require(linearizeLidarFactorWithFrozenProjection(current, measurement,
              projection_before, &residual, &jacobian, &covariance_before,
              &reason), "candidate frozen factor: " + reason);
  require((covariance_before - projection_before.selected_covariance).norm() == 0.0,
          "factor covariance differs from frozen selected covariance");
  require(*fixture.basis_calls == calls_before_snapshot + 3,
          "diagnostic dynamic objective callback count is unexpected");
  require(linearizeLidarFactorWithFrozenProjection(current, measurement,
              projection_before, &residual, &jacobian, &covariance_after,
              &reason), "repeat candidate frozen factor: " + reason);
  require((covariance_after - covariance_before).norm() == 0.0 &&
              (projection_before.basis - snapshot.projections.front().basis).norm() == 0.0,
          "candidate evaluation changed frozen basis or covariance");
  require(*fixture.basis_calls == calls_before_snapshot + 3,
          "candidate frozen linearizer invoked basis callback");
}

void outerIterationAndDiagnosticsParity() {
  auto off = makeDirectionalWindow(false, -0.05);
  auto on = makeDirectionalWindow(true, -0.05);
  const int off_calls_before = *off.basis_calls;
  const int on_calls_before = *on.basis_calls;
  std::string reason;
  require(off.window.optimize(&reason), "trace-off optimizer: " + reason);
  require(on.window.optimize(&reason), "trace-on optimizer: " + reason);
  require(localDifference(*off.window.latestState(),
                          *on.window.latestState()).norm() == 0.0,
          "trace diagnostics changed optimized result");
  const auto off_summary = off.window.summary();
  const auto on_summary = on.window.summary();
  require(off_summary.optimizer_status == on_summary.optimizer_status &&
              off_summary.optimizer_iterations == on_summary.optimizer_iterations &&
              off_summary.optimizer_initial_cost == on_summary.optimizer_initial_cost &&
              off_summary.optimizer_final_cost == on_summary.optimizer_final_cost,
          "trace diagnostics changed optimizer outcome");
  const auto& trace = on.window.optimizerTraceForDebug();
  require(trace.size() >= 2,
          "fixture did not create a second outer iteration");
  require(*off.basis_calls - off_calls_before ==
              static_cast<int>(off_summary.optimizer_iterations) &&
          *on.basis_calls - on_calls_before ==
              static_cast<int>(on_summary.optimizer_iterations),
          "optimizer candidate evaluation invoked a basis callback outside snapshot creation");
  bool projector_relinearized = false;
  std::uint64_t prior_generation = 0;
  for (const auto& row : trace) {
    require(row.candidate_basis_relinearization_calls == 0,
            "candidate acceptance called basis relinearizer");
    require(row.lidar_snapshot_generation > prior_generation,
            "snapshot generation did not advance per outer iteration");
    prior_generation = row.lidar_snapshot_generation;
    projector_relinearized = projector_relinearized ||
        row.max_projector_change_from_previous_outer > 1e-12;
    require(row.surrogate_current_cost == row.current_cost &&
                row.surrogate_candidate_cost == row.candidate_cost,
            "legacy trace aliases differ from explicit surrogate costs");
    if (!row.accepted && &row != &trace.back()) {
      const auto next = &row + 1;
      require(next->max_projector_change_from_previous_outer < 1e-12,
              "unchanged retry state did not reproduce its projector snapshot");
    }
  }
  require(projector_relinearized,
          "accepted outer update did not permit a new reliability projector");

  auto previous_outer = makeDirectionalWindow(false, -0.05);
  auto next_outer = makeDirectionalWindow(false, 0.06);
  LidarIterationSnapshot prior_snapshot;
  require(previous_outer.window.buildLidarIterationSnapshot(
              &prior_snapshot, &reason, 1),
          "first explicit outer snapshot: " + reason);
  LidarIterationSnapshot next_snapshot;
  require(next_outer.window.buildLidarIterationSnapshot(
              &next_snapshot, &reason, 2),
          "next outer snapshot: " + reason);
  const auto& basis_before = prior_snapshot.projections.front().basis;
  const auto& basis_after = next_snapshot.projections.front().basis;
  const Eigen::MatrixXd projector_before = basis_before.leftCols(5) *
      basis_before.leftCols(5).transpose();
  const Eigen::MatrixXd projector_after = basis_after.leftCols(5) *
      basis_after.leftCols(5).transpose();
  require((projector_after - projector_before).norm() > 1e-12,
          "new outer iteration failed to relinearize the state-dependent projector");
  WindowLinearSystem next_system;
  require(next_outer.window.blockLinearizedSystemWithLidarSnapshot(
              next_snapshot, &next_system, &reason),
          "next-outer frozen assembly: " + reason);
  const Eigen::VectorXd next_direction =
      next_system.gradient / next_system.gradient.norm();
  const int callback_count_after_freeze = *next_outer.basis_calls;
  WindowStateVector plus_states = next_outer.window.states();
  WindowStateVector minus_states = next_outer.window.states();
  Vector15d plus_step = 1e-6 * next_direction;
  require(applyLocalIncrement(&plus_states.front(), plus_step, &reason) &&
              applyLocalIncrement(&minus_states.front(), -plus_step, &reason),
          "next-outer FD candidate states: " + reason);
  ObjectiveBreakdown plus_cost, minus_cost;
  require(next_outer.window.objectiveAtStatesWithLidarSnapshotForDebug(
              plus_states, next_snapshot, &plus_cost, &reason) &&
              next_outer.window.objectiveAtStatesWithLidarSnapshotForDebug(
              minus_states, next_snapshot, &minus_cost, &reason),
          "next-outer frozen candidate objective: " + reason);
  const double next_fd = (plus_cost.total_cost - minus_cost.total_cost) / 2e-6;
  const double next_model = 2.0 * next_system.gradient.dot(next_direction);
  require(relativeError(next_fd, next_model) <= 1e-8 &&
              *next_outer.basis_calls == callback_count_after_freeze,
          "next-outer H/g and candidate objective do not share the new frozen projector");
}

void fullRankParity() {
  WindowState state = makeState();
  LidarWindowMeasurement measurement;
  measurement.observation_id = 900;
  measurement.stamp_ns = state.stamp_ns;
  measurement.measured_position = Eigen::Vector3d(0.4, 0.2, 0.1);
  measurement.measured_rotation =
      Eigen::AngleAxisd(-0.13, Eigen::Vector3d::UnitY()).toRotationMatrix();
  measurement.covariance = Matrix6d::Identity();
  measurement.reliable_rank = 6;
  measurement.valid = true;
  std::string reason;
  FrozenLidarProjection projection;
  require(freezeLidarProjection(state, measurement, &projection, &reason),
          "freeze full-rank projection: " + reason);
  Eigen::VectorXd new_residual;
  Eigen::MatrixXd new_jacobian, new_covariance;
  require(linearizeLidarFactorWithFrozenProjection(state, measurement,
              projection, &new_residual, &new_jacobian, &new_covariance,
              &reason), "new full-rank linearizer: " + reason);

  const auto independent_residual = [&](const WindowState& candidate) {
    Eigen::Matrix<double, 6, 1> raw;
    raw.head<3>() = measurement.measured_position - candidate.position;
    Eigen::Quaterniond q(candidate.rotation.transpose() *
                         measurement.measured_rotation);
    q.normalize();
    if (q.w() < 0.0) q.coeffs() *= -1.0;
    const double half_sine = q.vec().norm();
    if (half_sine < 1e-12) {
      raw.tail<3>() = 2.0 * q.vec();
    } else {
      raw.tail<3>() = q.vec() *
          (2.0 * std::atan2(half_sine, q.w()) / half_sine);
    }
    const Eigen::Matrix<double, 6, 1> projected =
        projection.basis.transpose() * raw;
    return projected;
  };
  const Eigen::VectorXd expected_residual = independent_residual(state);
  Eigen::MatrixXd expected_jacobian = Eigen::MatrixXd::Zero(6, 15);
  constexpr double fd_step = 2e-7;
  for (int column = 0; column < 6; ++column) {
    Vector15d perturbation = Vector15d::Zero();
    perturbation(column) = fd_step;
    WindowState plus = state;
    WindowState minus = state;
    require(applyLocalIncrement(&plus, perturbation, &reason) &&
                applyLocalIncrement(&minus, -perturbation, &reason),
            "independent full-rank FD state construction: " + reason);
    expected_jacobian.col(column) =
        (independent_residual(plus) - independent_residual(minus)) /
        (2.0 * fd_step);
  }
  const Eigen::MatrixXd expected_covariance =
      projection.basis.transpose() * measurement.covariance * projection.basis;
  require((expected_residual - new_residual).norm() < 1e-14 &&
              (expected_jacobian - new_jacobian).norm() < 2e-7 &&
              (expected_covariance - new_covariance).norm() < 1e-14,
          "full-rank independent oracle mismatch residual=" +
              std::to_string((expected_residual - new_residual).norm()) +
              " jacobian=" +
              std::to_string((expected_jacobian - new_jacobian).norm()) +
              " covariance=" +
              std::to_string((expected_covariance - new_covariance).norm()));

  FixedLagWindow window;
  Matrix15d prior_information = 2.0 * Matrix15d::Identity();
  Vector15d prior_gradient = Vector15d::Zero();
  require(window.initializeWithPriorAtomic(state, prior_information,
                                            prior_gradient),
          "full-rank parity window init");
  require(window.addLidarFactor(measurement, &reason),
          "full-rank parity add factor: " + reason);
  LidarIterationSnapshot snapshot;
  require(window.buildLidarIterationSnapshot(&snapshot, &reason),
          "full-rank explicit snapshot: " + reason);
  WindowLinearSystem implicit_system, explicit_system;
  require(window.blockLinearizedSystem(&implicit_system, &reason) &&
              window.blockLinearizedSystemWithLidarSnapshot(
                  snapshot, &explicit_system, &reason),
          "full-rank block assembly: " + reason);
  const Eigen::MatrixXd expected_information =
      Eigen::MatrixXd::Identity(6, 6);
  const Eigen::MatrixXd expected_hessian = prior_information +
      expected_jacobian.transpose() * expected_information * expected_jacobian;
  const Eigen::VectorXd expected_gradient = prior_gradient +
      expected_jacobian.transpose() * expected_information * expected_residual;
  const double expected_cost = expected_residual.dot(
      expected_information * expected_residual);
  require((implicit_system.dense() - expected_hessian).norm() < 2e-7 &&
              (implicit_system.gradient - expected_gradient).norm() < 2e-7 &&
              std::abs(implicit_system.cost - expected_cost) < 1e-13 &&
              (implicit_system.dense() - explicit_system.dense()).norm() == 0.0 &&
              (implicit_system.gradient - explicit_system.gradient).norm() == 0.0 &&
              implicit_system.cost == explicit_system.cost,
          "full-rank assembly differs from independent H/g/cost oracle");
}

}  // namespace

int main() {
  try {
    frozenSurrogateDerivativeAndCovariance();
    outerIterationAndDiagnosticsParity();
    fullRankParity();
    std::cout << "A3B_R2_FROZEN_PROJECTION_SYNTHETIC_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "A3B_R2_FROZEN_PROJECTION_SYNTHETIC_FAIL: "
              << error.what() << '\n';
    return 1;
  }
}
