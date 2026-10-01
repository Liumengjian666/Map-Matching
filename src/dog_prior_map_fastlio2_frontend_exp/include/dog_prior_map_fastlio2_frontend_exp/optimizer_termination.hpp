#pragma once

#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

// The existing optimizer termination threshold, not an acceptance tolerance.
inline constexpr double kOptimizerSmallStepThreshold = 1e-8;

// Called only after strict candidate rejection and complete state rollback.
inline bool isNaturalSmallStepConvergence(double raw_step_norm,
                                          bool step_was_clipped,
                                          double candidate_cost) {
  return std::isfinite(candidate_cost) && raw_step_norm > 0.0 &&
      raw_step_norm < kOptimizerSmallStepThreshold && !step_was_clipped;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
