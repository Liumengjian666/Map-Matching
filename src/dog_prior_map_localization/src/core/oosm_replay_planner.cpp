#include "dog_prior_map_localization/core/oosm_replay_planner.hpp"

#include <algorithm>
#include <cmath>

namespace dog_prior_map_localization
{

const char *oosmPlanStatusName(OosmPlanStatus status) noexcept
{
  switch (status)
  {
    case OosmPlanStatus::kReady: return "READY";
    case OosmPlanStatus::kFutureMeasurement: return "FUTURE_MEASUREMENT";
    case OosmPlanStatus::kNoHistory: return "NO_HISTORY";
    case OosmPlanStatus::kAlignmentTooLarge: return "ALIGNMENT_TOO_LARGE";
    case OosmPlanStatus::kReplayIncomplete: return "REPLAY_INCOMPLETE";
    case OosmPlanStatus::kInvalidTimestamp: return "INVALID_TIMESTAMP";
  }
  return "INVALID_TIMESTAMP";
}

OosmReplayPlan OosmReplayPlanner::makePlan(
    const StateHistory &state_history,
    const std::deque<ImuSample> &imu_history,
    double measurement_stamp,
    double state_now_stamp,
    double max_alignment_sec,
    double max_imu_dt_sec) const
{
  OosmReplayPlan plan;
  plan.measurement_stamp = measurement_stamp;
  plan.state_now_stamp = state_now_stamp;

  if (!std::isfinite(measurement_stamp))
  {
    plan.status = OosmPlanStatus::kInvalidTimestamp;
    return plan;
  }
  if (!std::isfinite(state_now_stamp))
  {
    plan.status = OosmPlanStatus::kNoHistory;
    return plan;
  }
  if (measurement_stamp > state_now_stamp + 1e-9)
  {
    plan.status = OosmPlanStatus::kFutureMeasurement;
    return plan;
  }

  if (!state_history.findAtOrBefore(measurement_stamp,
                                    plan.rollback_index,
                                    plan.alignment_error_sec))
  {
    plan.status = OosmPlanStatus::kNoHistory;
    return plan;
  }

  plan.rollback_stamp = state_history.at(plan.rollback_index).stamp;
  if (plan.alignment_error_sec > max_alignment_sec + 1e-9)
  {
    plan.status = OosmPlanStatus::kAlignmentTooLarge;
    return plan;
  }

  plan.replay_samples.reserve(imu_history.size());
  for (const ImuSample &sample : imu_history)
  {
    // Preserve the existing half-open sensor-time interval exactly:
    // rollback_stamp < imu_stamp <= state_now_stamp.  The epsilon on the
    // lower bound is part of the current callback's boundary semantics.
    if (sample.stamp > plan.rollback_stamp + 1e-9 &&
        sample.stamp <= state_now_stamp)
    {
      plan.replay_samples.push_back(sample);
    }
  }
  std::sort(plan.replay_samples.begin(), plan.replay_samples.end(),
            [](const ImuSample &a, const ImuSample &b)
            {
              return a.stamp < b.stamp;
            });

  // Validate before the node mutates its nominal state.  Empty replay is
  // intentionally valid because it is the existing behavior when the
  // rollback snapshot already reaches the current state time.
  double replay_stamp = plan.rollback_stamp;
  for (const ImuSample &sample : plan.replay_samples)
  {
    const double dt = sample.stamp - replay_stamp;
    if (!std::isfinite(sample.stamp) || !sample.acc.allFinite() ||
        !sample.gyro.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
        dt > max_imu_dt_sec + 1e-9)
    {
      plan.status = OosmPlanStatus::kReplayIncomplete;
      return plan;
    }
    replay_stamp = sample.stamp;
  }

  plan.status = OosmPlanStatus::kReady;
  return plan;
}

}  // namespace dog_prior_map_localization
