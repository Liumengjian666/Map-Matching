#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

#include "dog_prior_map_localization/core/estimator_types.hpp"

namespace dog_prior_map_localization
{

// ROS-free storage for timestamped estimator snapshots.  This class owns no
// nominal EKF state and deliberately contains no OOSM orchestration policy.
class StateHistory
{
public:
  void insertMonotonic(const FilterStateSnapshot &snapshot);

  void pruneOlderThan(double current_stamp, double keep_sec);

  bool findAtOrBefore(double target_stamp,
                      std::size_t &index,
                      double &alignment_error) const;

  void eraseAfter(double stamp);

  const FilterStateSnapshot &at(std::size_t index) const;

  std::size_t size() const noexcept { return snapshots_.size(); }
  bool empty() const noexcept { return snapshots_.empty(); }

private:
  std::deque<FilterStateSnapshot> snapshots_;
};

}  // namespace dog_prior_map_localization
