#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/optimizer_termination.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

FixedLagWindow fixture(bool trace, double maximum_step_norm = 2.0) {
  FixedLagOptions options;
  options.capture_optimizer_trace = trace;
  options.maximum_step_norm = maximum_step_norm;
  FixedLagWindow window(options);
  WindowState state;
  state.stamp_ns = 1'000'000'000;
  Vector15d gradient = Vector15d::Zero();
  gradient(3) = 1e-8;
  require(window.initializeWithPriorAtomic(state, Matrix15d::Identity(), gradient),
          "initialize tiny nonzero gradient fixture");
  // Two conflicting observations provide a finite nonzero cost, while their
  // gradients cancel. The tiny prior step's strict decrease rounds to zero.
  for (int side : {-1, 1}) {
    LidarWindowMeasurement lidar;
    lidar.observation_id = side < 0 ? 1 : 2;
    lidar.stamp_ns = state.stamp_ns;
    lidar.valid = true;
    lidar.measured_position.x() = side * 10.0;
    require(window.addLidarFactor(lidar), "add conflicting finite observation");
  }
  return window;
}

void naturalSmallStep() {
  auto window = fixture(true);
  const auto before = window.states();
  std::string reason;
  const bool success = window.optimize(&reason);
  const auto& rows = window.optimizerTraceForDebug();
  require(rows.size() == 1, "expected single natural small-step attempt");
  const auto& row = rows.front();
  std::cout << std::setprecision(17) << "natural raw=" << row.raw_step_norm
            << " applied=" << row.applied_step_norm << " clipped=" << row.step_clipped
            << " current=" << row.current_cost << " candidate=" << row.candidate_cost
            << " accepted=" << row.accepted << " status="
            << window.summary().optimizer_status << '\n';
  require(row.gradient_inf_norm > 1e-9 && row.raw_step_norm > 0 &&
              row.raw_step_norm < 1e-8 && !row.step_clipped &&
              std::isfinite(row.candidate_cost) && !row.accepted &&
              row.candidate_cost >= row.current_cost,
          "fixture must isolate natural unclipped non-decreasing finite candidate");
  require(success && window.summary().optimizer_status == "CONVERGED_WITHOUT_STEP",
          "natural small rejected step must be convergence, not FAILED_ALL_CANDIDATES");
  require(localDifference(before.front(), window.states().front()).norm() == 0,
          "rejected tiny candidate must not commit state");
  require(row.small_step_termination &&
              row.termination_reason == "NATURAL_SMALL_STEP_NO_ACCEPTED_UPDATE" &&
              row.candidate_rollback_state_difference == 0 &&
              window.summary().optimizer_success &&
              window.summary().prediction_feedback_ready,
          "natural small-step diagnostics/feedback must certify rolled-back state");
  auto trace_off = fixture(false);
  require(trace_off.optimize(&reason) &&
              localDifference(*trace_off.latestState(), *window.latestState()).norm() == 0,
          "trace OFF/ON natural convergence parity");
  require(trace_off.summary().optimizer_termination_reason ==
              window.summary().optimizer_termination_reason &&
              trace_off.summary().optimizer_raw_step_norm == row.raw_step_norm,
          "termination diagnostics must not depend on trace capture");
}

void clippedNegativeControls() {
  for (double maximum_step_norm : {0.0, 1e-10}) {
    auto window = fixture(true, maximum_step_norm);
    const auto before = window.states();
    std::string reason;
    require(!window.optimize(&reason), "policy-clipped tiny step must not converge");
    const auto& row = window.optimizerTraceForDebug().front();
    require(row.raw_step_norm > 0 && row.step_clipped &&
                row.applied_step_norm <= maximum_step_norm && !row.accepted &&
                window.summary().optimizer_status == "FAILED_ALL_CANDIDATES" &&
                !window.summary().prediction_feedback_ready &&
                localDifference(before.front(), window.states().front()).norm() == 0,
            "clipped small step masked a failure or committed rejected state");
    std::cout << "clipped maximum=" << maximum_step_norm << " status="
              << window.summary().optimizer_status << '\n';
    auto trace_off = fixture(false, maximum_step_norm);
    require(!trace_off.optimize(&reason) &&
                trace_off.summary().optimizer_status == window.summary().optimizer_status &&
                trace_off.summary().optimizer_termination_reason == window.summary().optimizer_termination_reason &&
                localDifference(*trace_off.latestState(), *window.latestState()).norm() == 0,
            "clipped negative control trace OFF/ON parity");
  }
}

void nonfiniteClassifierNegativeControls() {
  // Exercise the exact production predicate without inventing a new estimator
  // objective or adding a production candidate-injection hook.
  for (double cost : {std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity(),
                      -std::numeric_limits<double>::infinity()})
    require(!isNaturalSmallStepConvergence(3e-9, false, cost),
            "nonfinite objective must not classify as natural convergence");
  require(!isNaturalSmallStepConvergence(0, false, 200) &&
              !isNaturalSmallStepConvergence(1e-8, false, 200) &&
              !isNaturalSmallStepConvergence(3e-9, true, 200),
          "zero/boundary/clipped step convergence guard");
  std::cout << "nonfinite candidate classifier=NOT_CONVERGED\n";
}
}  // namespace

int main() {
  try {
    naturalSmallStep();
    clippedNegativeControls();
    nonfiniteClassifierNegativeControls();
    std::cout << "A3D_R1_TERMINATION_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "A3D_R1_TERMINATION_FAIL: " << error.what() << '\n';
    return 1;
  }
}
