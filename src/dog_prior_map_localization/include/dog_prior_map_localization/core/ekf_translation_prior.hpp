#pragma once

#include "dog_prior_map_localization/core/causal_pose_prediction.hpp"
#include <cstdint>
#include <string>

namespace dog_prior_map_localization {

// Selection only: prediction mathematics stays in causal_pose_prediction.hpp.
// All failure paths leave the caller's original guess bit-for-bit unchanged.
inline std::string applyEkfTranslationPrior(bool enabled, bool visual_enabled,
    bool visual_applied, bool previous_valid, bool imu_rotation_valid,
    bool causal_stream_valid, bool frame_valid, uint64_t sample_ns,
    uint64_t target_ns, const Eigen::Matrix4d& predicted, Eigen::Matrix4d* guess) {
  if (!enabled) return "DISABLED";
  if (visual_applied) return "VISUAL_APPLIED";
  if (!visual_enabled || !previous_valid || !imu_rotation_valid)
    return "NDT_HISTORY_FALLBACK";
  if (!causal_stream_valid) return "UNVERIFIED_EKF_PROVENANCE";
  if (!frame_valid) return "INVALID_FRAME";
  if (!sample_ns || !target_ns) return "NO_CAUSAL_EKF_SAMPLE";
  if (sample_ns > target_ns) return "FUTURE_SAMPLE_REJECTED";
  if (target_ns - sample_ns > 20000000ULL) return "STALE_EKF_SAMPLE";
  if (!guess || !rigidMapPose(predicted) || !guess->allFinite())
    return "NONFINITE_PREDICTION";
  guess->block<3, 1>(0, 3) = predicted.block<3, 1>(0, 3);
  return "EKF_TRANSLATION_APPLIED";
}

// A watermark is only trustworthy for a single ordered current-time stream.
// Equality/reversal is rejected, never replaced with an arrival-order choice.
inline bool orderedEkfSample(uint64_t previous_ns, uint64_t incoming_ns) {
  return incoming_ns != 0 && (!previous_ns || incoming_ns > previous_ns);
}

}  // namespace dog_prior_map_localization
