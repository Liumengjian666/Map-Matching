#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_experiment.hpp"

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

}  // namespace

const char* toString(WindowExecutionMode mode) {
  return mode == WindowExecutionMode::FULL_FIXED_LAG_EXPERIMENTAL
      ? "FULL_FIXED_LAG_EXPERIMENTAL" : "FORMAL_FULL_LEGACY";
}

FixedLagExperimentalController::FixedLagExperimentalController(
    WindowExecutionMode mode, const FixedLagOptions& options,
    const ImuNoiseParameters& imu_noise)
    : mode_(mode), window_(options, imu_noise) {}

bool FixedLagExperimentalController::enabled() const {
  return mode_ == WindowExecutionMode::FULL_FIXED_LAG_EXPERIMENTAL;
}

bool FixedLagExperimentalController::addState(const WindowState& state,
                                               std::string* reason) {
  return enabled() ? window_.addState(state, reason) :
      fail(reason, "fixed_lag_experimental_mode_disabled");
}

bool FixedLagExperimentalController::addImuFactor(
    std::uint64_t observation_id, std::uint64_t from_stamp_ns,
    std::uint64_t to_stamp_ns, const ImuPreintegratedMeasurement& measurement,
    std::string* reason) {
  return enabled() ? window_.addImuFactor(observation_id, from_stamp_ns,
                                          to_stamp_ns, measurement, reason) :
      fail(reason, "fixed_lag_experimental_mode_disabled");
}

bool FixedLagExperimentalController::addLidarFactor(
    const LidarWindowMeasurement& measurement, std::string* reason) {
  return enabled() ? window_.addLidarFactor(measurement, reason) :
      fail(reason, "fixed_lag_experimental_mode_disabled");
}

bool FixedLagExperimentalController::addVisualFactor(
    const VisualRelativeMeasurement& measurement, std::string* reason) {
  return enabled() ? window_.addVisualFactor(measurement, reason) :
      fail(reason, "fixed_lag_experimental_mode_disabled");
}

bool FixedLagExperimentalController::optimizeAndMarginalize(std::string* reason) {
  if (!enabled()) return fail(reason, "fixed_lag_experimental_mode_disabled");
  if (!window_.optimize(reason)) return false;
  return window_.marginalizeIfNeeded(reason);
}

bool FixedLagExperimentalController::predictionFeedbackSeed(
    WindowState* output, std::string* reason) const {
  if (!enabled()) return fail(reason, "fixed_lag_experimental_mode_disabled");
  return window_.predictionFeedbackSeed(output, reason);
}

WindowSummary FixedLagExperimentalController::summary() const {
  return window_.summary();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
