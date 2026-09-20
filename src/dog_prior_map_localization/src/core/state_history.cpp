#include "dog_prior_map_localization/core/state_history.hpp"

#include <algorithm>

namespace dog_prior_map_localization
{

void StateHistory::insertMonotonic(const FilterStateSnapshot &snapshot)
{
  if (!std::isfinite(snapshot.stamp)) return;
  if (!snapshots_.empty() &&
      snapshot.stamp < snapshots_.back().stamp - 1e-9)
  {
    return;
  }

  while (!snapshots_.empty() &&
         std::abs(snapshots_.back().stamp - snapshot.stamp) <= 1e-9)
  {
    snapshots_.pop_back();
  }
  snapshots_.push_back(snapshot);
}

void StateHistory::pruneOlderThan(double current_stamp, double keep_sec)
{
  if (!std::isfinite(current_stamp) || !std::isfinite(keep_sec)) return;
  while (!snapshots_.empty() &&
         current_stamp - snapshots_.front().stamp > keep_sec)
  {
    snapshots_.pop_front();
  }
}

bool StateHistory::findAtOrBefore(double target_stamp,
                                  std::size_t &index,
                                  double &alignment_error) const
{
  index = 0;
  alignment_error = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(target_stamp) || snapshots_.empty()) return false;

  // Keep the original reverse linear search and its <= target + epsilon
  // boundary semantics.  This is intentionally not lower_bound.
  for (std::size_t i = snapshots_.size(); i > 0; --i)
  {
    const FilterStateSnapshot &snapshot = snapshots_[i - 1];
    if (snapshot.stamp <= target_stamp + 1e-9)
    {
      index = i - 1;
      alignment_error = std::max(0.0, target_stamp - snapshot.stamp);
      return std::isfinite(alignment_error);
    }
  }
  return false;
}

void StateHistory::eraseAfter(double stamp)
{
  if (!std::isfinite(stamp)) return;
  while (!snapshots_.empty() &&
         snapshots_.back().stamp > stamp + 1e-9)
  {
    snapshots_.pop_back();
  }
}

const FilterStateSnapshot &StateHistory::at(std::size_t index) const
{
  return snapshots_.at(index);
}

}  // namespace dog_prior_map_localization
