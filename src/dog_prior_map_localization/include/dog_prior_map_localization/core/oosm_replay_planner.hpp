#pragma once

#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

#include "dog_prior_map_localization/core/state_history.hpp"

namespace dog_prior_map_localization
{

// Preparation status for the sensor-time OOSM rollback/replay plan.  The
// caller maps these values to the existing diagnostic strings; the planner
// itself has no ROS or output dependency.
enum class OosmPlanStatus
{
  kReady,
  kFutureMeasurement,
  kNoHistory,
  kAlignmentTooLarge,
  kReplayIncomplete,
  kInvalidTimestamp
};

const char *oosmPlanStatusName(OosmPlanStatus status) noexcept;

struct OosmReplayPlan
{
  OosmPlanStatus status = OosmPlanStatus::kInvalidTimestamp;
  std::size_t rollback_index = 0;
  double measurement_stamp = std::numeric_limits<double>::quiet_NaN();
  double state_now_stamp = std::numeric_limits<double>::quiet_NaN();
  double rollback_stamp = std::numeric_limits<double>::quiet_NaN();
  double alignment_error_sec = std::numeric_limits<double>::quiet_NaN();
  std::vector<ImuSample> replay_samples;
};

// Stateless, ROS-free preparation of the rollback and replay interval.
// StateHistory and IMU history remain owned by DogPriorMapEkfNode; only the
// samples selected for the returned plan are copied.
class OosmReplayPlanner
{
public:
  OosmReplayPlan makePlan(const StateHistory &state_history,
                          const std::deque<ImuSample> &imu_history,
                          double measurement_stamp,
                          double state_now_stamp,
                          double max_alignment_sec,
                          double max_imu_dt_sec) const;
};

}  // namespace dog_prior_map_localization
