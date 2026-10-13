#pragma once

#include <stdexcept>
#include "dog_prior_map_localization/core/imu_propagation.hpp"

namespace dog_prior_map_localization
{
enum class DeskewTrajectoryMode { LEGACY_HISTORY, HISTORY_NOMINAL_CONSISTENT, SCAN_LOCAL };

inline DeskewTrajectoryMode deskewTrajectoryMode(bool scan_local, bool history_consistent)
{
  if (scan_local && history_consistent)
    throw std::runtime_error("DESKEW_TRAJECTORY_FLAGS_MUTUALLY_EXCLUSIVE");
  if (scan_local) return DeskewTrajectoryMode::SCAN_LOCAL;
  if (history_consistent) return DeskewTrajectoryMode::HISTORY_NOMINAL_CONSISTENT;
  return DeskewTrajectoryMode::LEGACY_HISTORY;
}

inline const char *deskewTrajectoryModeName(DeskewTrajectoryMode mode)
{
  switch (mode) {
    case DeskewTrajectoryMode::SCAN_LOCAL: return "SCAN_LOCAL";
    case DeskewTrajectoryMode::HISTORY_NOMINAL_CONSISTENT: return "HISTORY_NOMINAL_CONSISTENT";
    default: return "LEGACY_HISTORY";
  }
}

// Deskew-only copy: never change the filter's nominal propagation or noise.
inline ImuKinematicsConfig deskewTrajectoryConfig(const ImuKinematicsConfig &global,
                                                 DeskewTrajectoryMode mode)
{
  auto local = global;
  if (mode != DeskewTrajectoryMode::LEGACY_HISTORY) {
    local.midpoint_interval_input_enable = false;
    local.continuous_gravity_correction_enable = false;
  }
  return local;
}
}  // namespace dog_prior_map_localization
