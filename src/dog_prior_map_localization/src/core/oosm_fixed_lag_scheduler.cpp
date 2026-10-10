#include "dog_prior_map_localization/core/oosm_fixed_lag_scheduler.hpp"

#include <algorithm>
#include <cmath>

namespace dog_prior_map_localization
{

const char *fixedLagEnqueueStatusName(FixedLagEnqueueStatus status) noexcept
{
  switch (status)
  {
    case FixedLagEnqueueStatus::kQueued: return "QUEUED";
    case FixedLagEnqueueStatus::kLate: return "LATE_NDT_FOR_FIXED_LAG";
    case FixedLagEnqueueStatus::kInvalid: return "INVALID_FIXED_LAG_OBSERVATION";
    case FixedLagEnqueueStatus::kQueueFull: return "FIXED_LAG_QUEUE_FULL";
    case FixedLagEnqueueStatus::kDuplicateId: return "DUPLICATE_FIXED_LAG_ID";
  }
  return "INVALID_FIXED_LAG_OBSERVATION";
}

bool OosmFixedLagScheduler::configure(double lag_sec, std::size_t max_queue_size) noexcept
{
  if (!std::isfinite(lag_sec) || lag_sec <= 0.0 || max_queue_size == 0) return false;
  lag_sec_ = lag_sec;
  max_queue_size_ = max_queue_size;
  pending_.clear();
  last_advanced_state_stamp_ = -std::numeric_limits<double>::infinity();
  configured_ = true;
  return true;
}

FixedLagEnqueueStatus OosmFixedLagScheduler::enqueue(uint64_t id,
    double measurement_stamp, double arrival_state_stamp, double arrival_wall_sec) noexcept
{
  if (!configured_ || id == 0 || !std::isfinite(measurement_stamp) ||
      !std::isfinite(arrival_state_stamp) || !std::isfinite(arrival_wall_sec))
    return FixedLagEnqueueStatus::kInvalid;
  const double deadline = measurement_stamp + lag_sec_;
  if (!std::isfinite(deadline)) return FixedLagEnqueueStatus::kInvalid;
  if (arrival_state_stamp >= deadline - 1e-9 ||
      last_advanced_state_stamp_ >= deadline - 1e-9)
    return FixedLagEnqueueStatus::kLate;
  if (pending_.size() >= max_queue_size_) return FixedLagEnqueueStatus::kQueueFull;
  if (std::any_of(pending_.begin(), pending_.end(), [id](const FixedLagObservation &item) {
        return item.id == id;
      }))
    return FixedLagEnqueueStatus::kDuplicateId;

  FixedLagObservation event;
  event.id = id;
  event.measurement_stamp = measurement_stamp;
  event.deadline_stamp = deadline;
  event.arrival_state_stamp = arrival_state_stamp;
  event.arrival_wall_sec = arrival_wall_sec;
  auto at = std::lower_bound(pending_.begin(), pending_.end(), event,
      [](const FixedLagObservation &lhs, const FixedLagObservation &rhs) {
        if (lhs.measurement_stamp != rhs.measurement_stamp)
          return lhs.measurement_stamp < rhs.measurement_stamp;
        return lhs.id < rhs.id;
      });
  pending_.insert(at, event);
  return FixedLagEnqueueStatus::kQueued;
}

FixedLagAdvanceResult OosmFixedLagScheduler::advance(double state_stamp) noexcept
{
  FixedLagAdvanceResult result;
  if (!configured_ || !std::isfinite(state_stamp))
  {
    result.valid = false;
    result.status = "INVALID_FIXED_LAG_WATERMARK";
    return result;
  }
  if (state_stamp + 1e-9 < last_advanced_state_stamp_)
  {
    result.valid = false;
    result.status = "FIXED_LAG_WATERMARK_REGRESSION";
    return result;
  }
  last_advanced_state_stamp_ = std::max(last_advanced_state_stamp_, state_stamp);
  while (!pending_.empty() && pending_.front().deadline_stamp <= state_stamp + 1e-9)
  {
    FixedLagObservation event = pending_.front();
    pending_.pop_front();
    event.release_state_stamp = state_stamp;
    result.ready.push_back(event);
  }
  return result;
}

}  // namespace dog_prior_map_localization
