#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dog_prior_map_localization/core/state_history.hpp"
#include "dog_prior_map_localization/core/imu_propagation.hpp"

namespace dog_prior_map_localization
{

struct ImuDeskewPose
{
  double stamp = 0.0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  // Raw IMU measurement at `stamp`; never an interval average.
  Eigen::Vector3d acc_measurement = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_measurement = Eigen::Vector3d::Zero();
  // Input used for the interval that ended at this pose timestamp.
  Eigen::Vector3d interval_acc_input = Eigen::Vector3d::Zero();
  Eigen::Vector3d interval_gyro_input = Eigen::Vector3d::Zero();
  bool has_interval_input = false;
  Eigen::Vector3d acc_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_unbiased = Eigen::Vector3d::Zero();
  double latest_source_stamp = 0.0;
};

struct ImuDeskewCoverage
{
  double history_first_stamp = 0.0;
  double history_last_stamp = 0.0;
  double max_gap_sec = 0.0;
  std::size_t samples_in_scan = 0;
};

// Checks that propagated EKF states bracket the full scan interval without
// extrapolation and that every state interval used is below max_gap_sec.
bool inspectImuDeskewCoverage(const StateHistory &history,
                             double scan_start,
                             double scan_end,
                             double max_gap_sec,
                             ImuDeskewCoverage &coverage,
                             std::string &reason);

// Reconstructs a partial state between bracketing EKF snapshots using their raw
// head/tail IMU samples. It never extrapolates outside stored state coverage.
bool interpolateImuDeskewPose(const StateHistory &history,
                              double stamp,
                              double max_gap_sec,
                              const Eigen::Vector3d &gravity_world,
                              const ImuKinematicsConfig &propagation_config,
                              ImuDeskewPose &pose,
                              std::string &reason,
                              double max_source_stamp = std::numeric_limits<double>::infinity());

// Transforms one LiDAR-frame point from its acquisition pose to the selected
// reference LiDAR pose using T_imu_lidar (p_imu = T_imu_lidar * p_lidar).
bool transformPointToReferenceLidar(const ImuDeskewPose &point_pose,
                                     const ImuDeskewPose &reference_pose,
                                     const Eigen::Isometry3d &T_imu_lidar,
                                     const Eigen::Vector3d &point_lidar,
                                     Eigen::Vector3d &point_reference,
                                     std::string &reason);

}  // namespace dog_prior_map_localization
