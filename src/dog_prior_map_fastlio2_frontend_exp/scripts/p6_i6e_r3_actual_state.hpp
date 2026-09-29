#pragma once

#include <cstdint>
#include <string>

namespace p6_i6e_r3 {

struct ActualFusionLedger {
  std::uint64_t initialization_stamp_ns = 0;
  std::uint64_t last_external_commit_ns = 0;
  bool ever_committed_external = false;
  bool relocalization_pending_validation = false;
  std::uint64_t relocalization_requested_ns = 0;
  std::uint64_t verified_recovery_ns = 0;
  std::uint64_t lidar_commit_count = 0;
  std::uint64_t visual_commit_count = 0;
  std::uint64_t imu_only_interval_count = 0;
};

inline void initializeActualFusionLedger(std::uint64_t stamp_ns,
                                         ActualFusionLedger* ledger) {
  *ledger = ActualFusionLedger();
  ledger->initialization_stamp_ns = stamp_ns;
  ledger->last_external_commit_ns = stamp_ns;
}

inline void recordVisualCommit(std::uint64_t stamp_ns,
                               ActualFusionLedger* ledger) {
  ledger->last_external_commit_ns = stamp_ns;
  ledger->ever_committed_external = true;
  ++ledger->visual_commit_count;
}

inline void recordLidarCommit(std::uint64_t stamp_ns,
                              ActualFusionLedger* ledger) {
  ledger->last_external_commit_ns = stamp_ns;
  ledger->ever_committed_external = true;
  ++ledger->lidar_commit_count;
}

inline double actualExternalGapSeconds(std::uint64_t stamp_ns,
                                       const ActualFusionLedger& ledger) {
  return stamp_ns >= ledger.last_external_commit_ns
      ? static_cast<double>(stamp_ns - ledger.last_external_commit_ns) * 1e-9
      : 0.0;
}

inline void updateRelocalizationPending(bool provisional_required,
                                        std::uint64_t stamp_ns,
                                        double actual_gap_s,
                                        double coast_limit_s,
                                        ActualFusionLedger* ledger) {
  if (!ledger->relocalization_pending_validation &&
      (provisional_required || actual_gap_s > coast_limit_s)) {
    ledger->relocalization_pending_validation = true;
    ledger->relocalization_requested_ns = stamp_ns;
  }
  // There is no independently validated global relocalizer in this project.
  // Deliberately never clear the latch or manufacture verified_recovery_ns.
}

inline std::string actualHealthStatus(bool lidar_committed,
                                      bool visual_committed,
                                      bool partial_directional,
                                      ActualFusionLedger* ledger) {
  if (ledger->relocalization_pending_validation)
    return "RELOCALIZATION_PENDING_VALIDATION";
  if (lidar_committed && partial_directional)
    return "PARTIAL_DIRECTIONAL_CONSTRAINT";
  if (lidar_committed) return "EXTERNAL_UPDATE_COMMITTED";
  if (visual_committed) return "VISUAL_ONLY";
  ++ledger->imu_only_interval_count;
  return "IMU_ONLY";
}

}  // namespace p6_i6e_r3
