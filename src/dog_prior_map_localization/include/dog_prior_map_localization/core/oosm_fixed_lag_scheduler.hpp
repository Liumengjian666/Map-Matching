#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace dog_prior_map_localization
{

enum class FixedLagEnqueueStatus { kQueued, kLate, kInvalid, kQueueFull, kDuplicateId };
const char *fixedLagEnqueueStatusName(FixedLagEnqueueStatus status) noexcept;

struct FixedLagObservation
{
  uint64_t id = 0;
  double measurement_stamp = std::numeric_limits<double>::quiet_NaN();
  double deadline_stamp = std::numeric_limits<double>::quiet_NaN();
  double arrival_state_stamp = std::numeric_limits<double>::quiet_NaN();
  double arrival_wall_sec = std::numeric_limits<double>::quiet_NaN();
  double release_state_stamp = std::numeric_limits<double>::quiet_NaN();
};

struct FixedLagAdvanceResult
{
  bool valid = true;
  const char *status = "OK";
  std::vector<FixedLagObservation> ready;
};

// Release is selected only by the first propagated sensor-time watermark at
// or beyond stamp+lag.  Callback arrival time only decides whether it is late.
class OosmFixedLagScheduler
{
public:
  bool configure(double lag_sec, std::size_t max_queue_size) noexcept;
  FixedLagEnqueueStatus enqueue(uint64_t id, double measurement_stamp,
      double arrival_state_stamp, double arrival_wall_sec) noexcept;
  FixedLagAdvanceResult advance(double state_stamp) noexcept;

  double lagSec() const noexcept { return lag_sec_; }
  std::size_t maxQueueSize() const noexcept { return max_queue_size_; }
  std::size_t pendingCount() const noexcept { return pending_.size(); }
  const std::deque<FixedLagObservation> &pending() const noexcept { return pending_; }

private:
  double lag_sec_ = 0.0;
  std::size_t max_queue_size_ = 0;
  double last_advanced_state_stamp_ = -std::numeric_limits<double>::infinity();
  bool configured_ = false;
  std::deque<FixedLagObservation> pending_;
};

}  // namespace dog_prior_map_localization
