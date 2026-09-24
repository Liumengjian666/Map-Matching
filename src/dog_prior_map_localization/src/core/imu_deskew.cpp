#include "dog_prior_map_localization/core/imu_deskew.hpp"

#include <algorithm>
#include <cmath>

namespace dog_prior_map_localization
{
namespace
{
constexpr double kStampEpsilon = 1e-9;

ImuDeskewPose toPose(const FilterStateSnapshot &snapshot)
{
  ImuDeskewPose pose;
  pose.stamp = snapshot.stamp;
  pose.p = snapshot.p;
  pose.v = snapshot.v;
  pose.R = snapshot.R;
  pose.ba = snapshot.ba;
  pose.bg = snapshot.bg;
  pose.acc_measurement = snapshot.acc_measurement;
  pose.gyro_measurement = snapshot.gyro_measurement;
  pose.interval_acc_input = snapshot.interval_acc_input;
  pose.interval_gyro_input = snapshot.interval_gyro_input;
  pose.has_interval_input = snapshot.has_interval_input;
  pose.acc_world = snapshot.acc_world;
  pose.gyro_unbiased = snapshot.gyro_unbiased;
  pose.latest_source_stamp = snapshot.stamp;
  return pose;
}
}  // namespace

bool inspectImuDeskewCoverage(const StateHistory &history,
                             double scan_start,
                             double scan_end,
                             double max_gap_sec,
                             ImuDeskewCoverage &coverage,
                             std::string &reason)
{
  coverage = ImuDeskewCoverage{};
  if (!std::isfinite(scan_start) || !std::isfinite(scan_end) ||
      !std::isfinite(max_gap_sec) || scan_end < scan_start || max_gap_sec <= 0.0)
  {
    reason = "invalid_interval_or_gap_limit";
    return false;
  }

  const auto &samples = history.samples();
  if (samples.empty())
  {
    reason = "no_state_history";
    return false;
  }
  coverage.history_first_stamp = samples.front().stamp;
  coverage.history_last_stamp = samples.back().stamp;
  if (samples.front().stamp > scan_start + kStampEpsilon)
  {
    reason = "missing_start_coverage";
    return false;
  }
  if (samples.back().stamp < scan_end - kStampEpsilon)
  {
    reason = "waiting_for_end_coverage";
    return false;
  }

  auto left = std::lower_bound(samples.begin(), samples.end(), scan_start,
      [](const FilterStateSnapshot &sample, double stamp) { return sample.stamp < stamp; });
  if (left != samples.begin() &&
      (left == samples.end() || left->stamp > scan_start + kStampEpsilon))
    --left;
  if (left == samples.end())
  {
    reason = "missing_start_bracket";
    return false;
  }

  auto right = std::lower_bound(samples.begin(), samples.end(), scan_end,
      [](const FilterStateSnapshot &sample, double stamp) { return sample.stamp < stamp; });
  if (right == samples.end())
  {
    reason = "waiting_for_end_coverage";
    return false;
  }

  std::size_t first_index = static_cast<std::size_t>(std::distance(samples.begin(), left));
  std::size_t last_index = static_cast<std::size_t>(std::distance(samples.begin(), right));
  coverage.samples_in_scan = 0;
  for (std::size_t i = 0; i < samples.size(); ++i)
  {
    if (samples[i].stamp >= scan_start - kStampEpsilon &&
        samples[i].stamp <= scan_end + kStampEpsilon)
      ++coverage.samples_in_scan;
  }
  for (std::size_t i = first_index + 1; i <= last_index; ++i)
  {
    const double gap = samples[i].stamp - samples[i - 1].stamp;
    if (!std::isfinite(gap) || gap <= 0.0)
    {
      reason = "nonmonotonic_state_history";
      return false;
    }
    coverage.max_gap_sec = std::max(coverage.max_gap_sec, gap);
    if (gap > max_gap_sec + kStampEpsilon)
    {
      reason = "state_gap_exceeds_limit";
      return false;
    }
  }
  reason = "covered";
  return true;
}

bool interpolateImuDeskewPose(const StateHistory &history,
                              double stamp,
                              double max_gap_sec,
                              const Eigen::Vector3d &gravity_world,
                              const ImuKinematicsConfig &propagation_config,
                              ImuDeskewPose &pose,
                              std::string &reason,
                              double max_source_stamp)
{
  if (!std::isfinite(stamp) || !std::isfinite(max_gap_sec) || max_gap_sec <= 0.0 ||
      std::isnan(max_source_stamp))
  {
    reason = "invalid_interpolation_time";
    return false;
  }
  const auto &samples = history.samples();
  if (samples.empty())
  {
    reason = "no_state_history";
    return false;
  }
  auto upper = std::lower_bound(samples.begin(), samples.end(), stamp,
      [](const FilterStateSnapshot &sample, double target) { return sample.stamp < target; });
  if (upper != samples.end() && std::abs(upper->stamp - stamp) <= kStampEpsilon)
  {
    pose = toPose(*upper);
    reason = "exact_state_sample";
    return pose.p.allFinite() && pose.v.allFinite() && pose.R.allFinite();
  }
  if (upper == samples.begin() || upper == samples.end())
  {
    reason = "interpolation_would_extrapolate";
    return false;
  }
  const auto lower = upper - 1;
  const double gap = upper->stamp - lower->stamp;
  if (!std::isfinite(gap) || gap <= 0.0 || gap > max_gap_sec + kStampEpsilon)
  {
    reason = "interpolation_gap_exceeds_limit";
    return false;
  }
  const double dt = stamp - lower->stamp;
  if (!std::isfinite(dt) || dt < 0.0 || dt > gap + kStampEpsilon)
  {
    reason = "invalid_partial_propagation_time";
    return false;
  }

  // Snapshot measurements are raw samples at their own timestamps. Use the
  // mature head/tail mean only when its tail is within the caller's admitted
  // data horizon; the final scan-boundary fragment must not consume an IMU
  // sample after scan_end.
  pose = toPose(*lower);
  const bool tail_is_within_horizon = upper->stamp <= max_source_stamp + kStampEpsilon;
  const bool midpoint = propagation_config.midpoint_interval_input_enable &&
                        tail_is_within_horizon;
  const ImuIntervalInput interval = makeImuIntervalInput(
      lower->acc_measurement, lower->gyro_measurement,
      upper->acc_measurement, upper->gyro_measurement,
      midpoint ? ImuIntervalInputPolicy::kMidpointAverage :
                 ImuIntervalInputPolicy::kHeadSample);
  pose.interval_acc_input = interval.acc;
  pose.interval_gyro_input = interval.gyro;
  pose.has_interval_input = true;
  propagateImuKinematics(pose.p, pose.v, pose.R, pose.ba, pose.bg,
                         interval.acc, interval.gyro, dt,
                         gravity_world, propagation_config,
                         pose.acc_world, pose.gyro_unbiased);
  pose.stamp = stamp;
  pose.latest_source_stamp = midpoint ? upper->stamp : lower->stamp;
  if (!pose.p.allFinite() || !pose.v.allFinite() || !pose.R.allFinite() ||
      !pose.acc_world.allFinite() || !pose.gyro_unbiased.allFinite())
  {
    reason = "nonfinite_interpolated_state";
    return false;
  }
  reason = "causal_partial_propagation";
  return true;
}

bool transformPointToReferenceLidar(const ImuDeskewPose &point_pose,
                                     const ImuDeskewPose &reference_pose,
                                     const Eigen::Isometry3d &T_imu_lidar,
                                     const Eigen::Vector3d &point_lidar,
                                     Eigen::Vector3d &point_reference,
                                     std::string &reason)
{
  if (!point_pose.p.allFinite() || !point_pose.R.allFinite() ||
      !reference_pose.p.allFinite() || !reference_pose.R.allFinite() ||
      !T_imu_lidar.matrix().allFinite() || !point_lidar.allFinite())
  {
    reason = "nonfinite_transform_input";
    return false;
  }
  Eigen::Isometry3d T_world_imu_point = Eigen::Isometry3d::Identity();
  T_world_imu_point.linear() = point_pose.R;
  T_world_imu_point.translation() = point_pose.p;
  Eigen::Isometry3d T_world_imu_reference = Eigen::Isometry3d::Identity();
  T_world_imu_reference.linear() = reference_pose.R;
  T_world_imu_reference.translation() = reference_pose.p;
  const Eigen::Isometry3d T_world_lidar_point = T_world_imu_point * T_imu_lidar;
  const Eigen::Isometry3d T_world_lidar_reference = T_world_imu_reference * T_imu_lidar;
  point_reference = T_world_lidar_reference.inverse() * T_world_lidar_point * point_lidar;
  if (!point_reference.allFinite())
  {
    reason = "nonfinite_transformed_point";
    return false;
  }
  reason = "transformed";
  return true;
}

}  // namespace dog_prior_map_localization
