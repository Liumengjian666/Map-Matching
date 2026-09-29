#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"

#include <string>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

enum class WindowExecutionMode {
  FORMAL_FULL_LEGACY,
  FULL_FIXED_LAG_EXPERIMENTAL,
};

const char* toString(WindowExecutionMode mode);

// Explicit mode boundary: the formal FULL path cannot accidentally call the
// joint optimizer. The experimental controller owns one FixedLagWindow and
// exposes only its latest optimized state as the next prediction seed.
class FixedLagExperimentalController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FixedLagExperimentalController(
      WindowExecutionMode mode = WindowExecutionMode::FORMAL_FULL_LEGACY,
      const FixedLagOptions& options = {},
      const ImuNoiseParameters& imu_noise = {});

  bool enabled() const;
  bool addState(const WindowState& state, std::string* reason = nullptr);
  bool addImuFactor(std::uint64_t observation_id, std::uint64_t from_stamp_ns,
                    std::uint64_t to_stamp_ns,
                    const ImuPreintegratedMeasurement& measurement,
                    std::string* reason = nullptr);
  bool addLidarFactor(const LidarWindowMeasurement& measurement,
                      std::string* reason = nullptr);
  bool addVisualFactor(const VisualRelativeMeasurement& measurement,
                       std::string* reason = nullptr);
  bool setInitialPrior(std::uint64_t stamp_ns, const Matrix15d& information,
                       const Vector15d& gradient,
                       std::string* reason = nullptr);
  bool optimizeAndMarginalize(std::string* reason = nullptr);
  bool stateAt(std::uint64_t stamp_ns, WindowState* output,
               std::string* reason = nullptr) const;
  bool latestState(WindowState* output, std::string* reason = nullptr) const;
  bool predictionFeedbackSeed(WindowState* output,
                              std::string* reason = nullptr) const;
  WindowSummary summary() const;

 private:
  WindowExecutionMode mode_;
  FixedLagWindow window_;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
