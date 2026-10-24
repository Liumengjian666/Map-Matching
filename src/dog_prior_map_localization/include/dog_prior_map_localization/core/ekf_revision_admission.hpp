#pragma once
#include <cstdint>
#include <string>
namespace dog_prior_map_localization {
// Exact integer sensor-time identity. No tolerance, resampling or time rewrites.
inline std::string correctedHistoryAdmission(bool provenance, bool finite,
    uint64_t incoming, uint64_t latest, bool already_corrected) {
  if (!provenance) return "INVALID_SOURCE_OR_FRAME";
  if (!finite) return "INVALID_POSE_OR_TWIST";
  if (!incoming || !latest) return "NO_LATEST_HIGH_RATE_SAMPLE";
  if (latest > incoming) return "LATER_HISTORY_EXISTS";
  if (latest < incoming) return "EXACT_STAMP_MISMATCH_CORRECTED_LATER";
  if (already_corrected) return "DUPLICATE_CORRECTED_SAMPLE";
  return "REPLACED_EXACT_LATEST";
}
}
