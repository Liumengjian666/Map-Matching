#pragma once

#include <Eigen/Core>
#include <cmath>

namespace superloc_corridor_adapter
{

struct Pose3d
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
};

// Relative transform mapping coordinates in the moving LiDAR frame into the
// scan-start LiDAR frame: T_L0_Li = inverse(T_W_L0) * T_W_Li.
inline Pose3d relativePose(const Pose3d &world_lidar_start,
                           const Pose3d &world_lidar_point)
{
  Pose3d relative;
  relative.rotation = world_lidar_start.rotation.transpose() *
                      world_lidar_point.rotation;
  relative.translation = world_lidar_start.rotation.transpose() *
                        (world_lidar_point.translation -
                         world_lidar_start.translation);
  return relative;
}

inline Pose3d composePose(const Pose3d &a, const Pose3d &b)
{
  Pose3d result;
  result.rotation = a.rotation * b.rotation;
  result.translation = a.translation + a.rotation * b.translation;
  return result;
}

inline Eigen::Vector3d transformPoint(const Pose3d &transform,
                                      const Eigen::Vector3d &point)
{
  return transform.rotation * point + transform.translation;
}

// Convert an IMU-relative orientation increment to a LiDAR-relative increment.
// R_i_l maps LiDAR coordinates into IMU; R_l_i is its inverse.
inline Eigen::Matrix3d lidarRelativeRotation(
    const Eigen::Matrix3d &R_i_l,
    const Eigen::Matrix3d &R_imu_start_imu_point)
{
  const Eigen::Matrix3d R_l_i = R_i_l.transpose();
  return R_l_i * R_imu_start_imu_point * R_i_l;
}

inline bool coversWithoutExtrapolation(double start, double end,
                                       double first_sample,
                                       double last_sample)
{
  return std::isfinite(start) && std::isfinite(end) &&
         std::isfinite(first_sample) && std::isfinite(last_sample) &&
         start <= end && first_sample <= start && last_sample >= end;
}

inline bool validScanStartRelativeTimeWindow(double time_min, double time_max,
                                             double max_scan_duration)
{
  return std::isfinite(time_min) && std::isfinite(time_max) &&
         std::isfinite(max_scan_duration) && time_min >= 0.0 &&
         time_max > time_min && time_max <= max_scan_duration;
}

enum class ImuCoverageState
{
  INVALID_INTERVAL,
  WAIT_FOR_FIRST_SAMPLE,
  START_NOT_COVERED,
  WAIT_FOR_END_SAMPLE,
  COVERED
};

inline ImuCoverageState classifyImuCoverage(double start, double end,
                                            bool has_samples,
                                            double first_sample,
                                            double last_sample)
{
  if (!std::isfinite(start) || !std::isfinite(end) || start > end)
    return ImuCoverageState::INVALID_INTERVAL;
  if (!has_samples)
    return ImuCoverageState::WAIT_FOR_FIRST_SAMPLE;
  if (!std::isfinite(first_sample) || !std::isfinite(last_sample) ||
      first_sample > start)
    return ImuCoverageState::START_NOT_COVERED;
  if (last_sample < end)
    return ImuCoverageState::WAIT_FOR_END_SAMPLE;
  return ImuCoverageState::COVERED;
}

}  // namespace superloc_corridor_adapter
