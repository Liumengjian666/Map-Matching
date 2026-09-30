#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

WindowState stateAt(std::uint64_t stamp, const Eigen::Vector3d& position) {
  WindowState state;
  state.stamp_ns = stamp;
  state.position = position;
  return state;
}

FixedLagWindow makeScalarWindow(bool capture_trace, double maximum_step_norm) {
  FixedLagOptions options;
  options.maximum_duration_s = 10.0;
  options.maximum_optimizer_iterations = 4;
  options.maximum_step_norm = maximum_step_norm;
  options.capture_optimizer_trace = capture_trace;
  FixedLagWindow window(options);
  const WindowState initial = stateAt(1'000'000'000ULL, Eigen::Vector3d::Zero());
  Matrix15d information = 2.0 * Matrix15d::Identity();
  Vector15d gradient = Vector15d::Zero();
  gradient(3) = 0.2;
  require(window.initializeWithPriorAtomic(initial, information, gradient),
          "scalar window initialization");
  return window;
}

FixedLagWindow makeFactorWindow() {
  FixedLagOptions options;
  options.maximum_duration_s = 10.0;
  options.maximum_optimizer_iterations = 5;
  FixedLagWindow window(options);

  const WindowState reference = stateAt(1'000'000'000ULL,
                                         Eigen::Vector3d(0.0, 0.0, 0.0));
  Matrix15d prior_information = 3.0 * Matrix15d::Identity();
  prior_information(3, 6) = prior_information(6, 3) = 0.15;
  Vector15d prior_gradient = Vector15d::Zero();
  prior_gradient(3) = -0.04;
  prior_gradient(4) = 0.03;
  require(window.initializeWithPriorAtomic(reference, prior_information,
                                            prior_gradient),
          "factor window prior initialization");

  WindowState current = stateAt(1'100'000'000ULL,
                                Eigen::Vector3d(0.18, -0.04, 0.02));
  ImuPreintegratedMeasurement imu;
  imu.start_stamp_ns = reference.stamp_ns;
  imu.end_stamp_ns = current.stamp_ns;
  imu.dt_s = 0.1;
  imu.valid = true;
  imu.delta_rotation = Eigen::Matrix3d::Identity();
  imu.delta_position.setZero();
  imu.delta_velocity.setZero();
  imu.covariance = 0.2 * Matrix15d::Identity();
  require(window.addStateWithImuFactorAtomic(current, 1,
              reference.stamp_ns, imu), "factor window IMU factor");

  LidarWindowMeasurement lidar;
  lidar.observation_id = 2;
  lidar.stamp_ns = current.stamp_ns;
  lidar.measured_position = Eigen::Vector3d(0.65, 0.12, -0.08);
  lidar.measured_rotation = Eigen::AngleAxisd(0.35,
      Eigen::Vector3d::UnitZ()).toRotationMatrix();
  lidar.covariance = 0.04 * Matrix6d::Identity();
  lidar.reliable_rank = 5;
  lidar.valid = true;
  lidar.basis_relinearizer = [](const WindowState& state, Matrix6d* basis,
                                int* rank, std::string* reason) {
    if (!basis || !rank) {
      if (reason) *reason = "null_dynamic_basis_output";
      return false;
    }
    const double angle = 0.8 * state.position.x();
    basis->setIdentity();
    (*basis)(3, 3) = std::cos(angle);
    (*basis)(3, 5) = -std::sin(angle);
    (*basis)(5, 3) = std::sin(angle);
    (*basis)(5, 5) = std::cos(angle);
    *rank = 5;
    return true;
  };
  require(window.addLidarFactor(lidar), "factor window directional LiDAR");

  VisualRelativeMeasurement visual;
  visual.observation_id = 3;
  visual.reference_stamp_ns = reference.stamp_ns;
  visual.current_stamp_ns = current.stamp_ns;
  visual.reference_imu_translation = Eigen::Vector3d(0.35, -0.02, 0.01);
  visual.covariance = 0.08 * Eigen::Matrix3d::Identity();
  visual.valid = true;
  require(window.addVisualFactor(visual), "factor window visual factor");
  return window;
}

void traceBehaviorParity() {
  FixedLagWindow without_trace = makeScalarWindow(false, 2.0);
  FixedLagWindow with_trace = makeScalarWindow(true, 2.0);
  std::string reason;
  require(without_trace.optimize(&reason), "trace-off successful optimize: " + reason);
  require(with_trace.optimize(&reason), "trace-on successful optimize: " + reason);
  require((localDifference(*without_trace.latestState(),
                           *with_trace.latestState())).norm() == 0.0,
          "trace capture changed optimized state");
  const auto off_summary = without_trace.summary();
  const auto on_summary = with_trace.summary();
  require(off_summary.optimizer_status == on_summary.optimizer_status &&
              off_summary.optimizer_iterations == on_summary.optimizer_iterations &&
              off_summary.optimizer_initial_cost == on_summary.optimizer_initial_cost &&
              off_summary.optimizer_final_cost == on_summary.optimizer_final_cost,
          "trace capture changed optimizer outcome");
  require(without_trace.optimizerTraceForDebug().empty(),
          "disabled tracing retained trace rows");
  require(!with_trace.optimizerTraceForDebug().empty() &&
              with_trace.optimizerTraceForDebug().front().accepted,
          "enabled tracing did not record accepted optimizer candidate");
}

void breakdownAndFrozenBasis() {
  FixedLagWindow window = makeFactorWindow();
  std::string reason;
  ObjectiveBreakdown before;
  require(window.objectiveBreakdownForDebug(&before, &reason),
          "objective breakdown: " + reason);
  const double component_sum = before.prior_cost + before.imu_cost +
      before.lidar_cost + before.visual_cost;
  require(std::abs(component_sum - before.total_cost) <=
              1e-12 * std::max(1.0, std::abs(before.total_cost)),
          "objective component sum mismatch");
  require(before.imu_cost > 0.0 && before.lidar_cost > 0.0 &&
              before.visual_cost > 0.0 && before.latest_lidar_factor_cost > 0.0,
          "synthetic graph failed to exercise factor cost categories");

  WindowLinearSystem system_before, system_after;
  require(window.blockLinearizedSystem(&system_before, &reason),
          "pre-debug system: " + reason);
  WindowStateVector candidate = window.states();
  Vector15d increment = Vector15d::Zero();
  increment(3) = 0.6;
  require(applyLocalIncrement(&candidate.back(), increment, &reason),
          "synthetic candidate state: " + reason);
  ObjectiveBreakdown production_candidate, frozen_candidate;
  require(window.objectiveBreakdownAtStatesForDebug(candidate, false,
              &production_candidate, &reason),
          "relinearized candidate objective: " + reason);
  require(window.objectiveBreakdownAtStatesForDebug(candidate, true,
              &frozen_candidate, &reason),
          "frozen-basis candidate objective: " + reason);
  require(std::abs(production_candidate.lidar_cost - frozen_candidate.lidar_cost) >
              1e-8,
          "synthetic state-dependent basis did not distinguish objectives");
  ObjectiveBreakdown diagnostic_start;
  std::vector<DirectionalDerivativeTrace> forensic_derivatives;
  std::vector<DampingSweepTrace> forensic_sweep;
  double forensic_state_difference = -1.0;
  require(window.diagnoseOptimizerFailureForDebug(
              {1e-8, 1e-7, 1e-6, 1e-5, 1e-4},
              {1e-6}, &diagnostic_start, &forensic_derivatives,
              &forensic_sweep, &forensic_state_difference, &reason),
          "state-dependent basis forensic derivative: " + reason);
  bool frozen_fd_matches_model = false;
  bool relinearized_fd_exposes_mismatch = false;
  for (const auto& row : forensic_derivatives) {
    require(row.valid && row.status == "OK",
            "state-dependent basis forensic row invalid: " + row.status);
    if (row.epsilon >= 1e-6 && row.epsilon <= 1e-4) {
      frozen_fd_matches_model = frozen_fd_matches_model ||
          row.frozen_basis_relative_error < 1e-4;
      relinearized_fd_exposes_mismatch =
          relinearized_fd_exposes_mismatch ||
          row.diagnostic_relinearized_basis_relative_error >
              std::max(1e-3, 10.0 * row.frozen_basis_relative_error);
    }
  }
  require(frozen_fd_matches_model && relinearized_fd_exposes_mismatch &&
              forensic_state_difference == 0.0,
          "forensic FD did not retain frozen-consistent/relinearized-inconsistent evidence transactionally");
  require(window.blockLinearizedSystem(&system_after, &reason),
          "post-debug system: " + reason);
  require((system_before.dense() - system_after.dense()).norm() == 0.0 &&
              (system_before.gradient - system_after.gradient).norm() == 0.0 &&
              system_before.cost == system_after.cost,
          "frozen-basis debug evaluation mutated original factors/window");
  ObjectiveBreakdown after;
  require(window.objectiveBreakdownForDebug(&after, &reason),
          "post-debug breakdown: " + reason);
  require(after.total_cost == before.total_cost,
          "frozen-basis debug evaluation changed production objective");
}

void failureTraceAndTransactionalDiagnosis() {
  FixedLagWindow failed = makeScalarWindow(true, 0.0);
  const WindowState before = *failed.latestState();
  std::string reason;
  require(!failed.optimize(&reason), "zero applied step should reject all candidates");
  require(reason == "all_optimizer_candidates_rejected" &&
              failed.summary().optimizer_status == "FAILED_ALL_CANDIDATES",
          "failure trace fixture did not reach all-candidates-rejected");
  const auto trace = failed.optimizerTraceForDebug();
  require(trace.size() == static_cast<std::size_t>(
              failed.summary().optimizer_iterations) && trace.size() == 1,
          "failure optimizer trace did not retain the rejected attempt");
  for (const auto& row : trace)
    require(!row.accepted && row.raw_step_norm > 0.0 && row.applied_step_norm == 0.0,
            "failure trace step fields are inconsistent");

  ObjectiveBreakdown breakdown;
  std::vector<DirectionalDerivativeTrace> derivatives;
  std::vector<DampingSweepTrace> sweep;
  double state_difference = -1.0;
  require(failed.diagnoseOptimizerFailureForDebug(
              {1e-8, 1e-7, 1e-6, 1e-5, 1e-4},
              {1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0,
               100.0, 1e3, 1e4, 1e5, 1e6},
              &breakdown, &derivatives, &sweep, &state_difference, &reason),
          "transactional failure diagnosis: " + reason);
  require(derivatives.size() == 10 && sweep.size() == 13,
          "diagnostic direction or damping sweep row count mismatch");
  require(state_difference == 0.0 &&
              localDifference(*failed.latestState(), before).norm() == 0.0,
          "directional derivative/damping sweep changed the failed window state");
  for (const auto& row : derivatives) {
    require(row.valid && row.status == "OK",
            "directional derivative row invalid: " + row.status);
  }
  for (const auto& row : sweep)
    require(row.solved && row.applied_step_norm == 0.0,
            "synthetic damping sweep solve/clip mismatch");
}

}  // namespace

int main() {
  try {
    traceBehaviorParity();
    breakdownAndFrozenBasis();
    failureTraceAndTransactionalDiagnosis();
    std::cout << "A3B_R1_OPTIMIZER_DIAGNOSTICS_SYNTHETIC_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "A3B_R1_OPTIMIZER_DIAGNOSTICS_SYNTHETIC_FAIL: "
              << error.what() << '\n';
    return 1;
  }
}
