#pragma once

#include <cstddef>
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
  Eigen::Vector3d acc_measurement = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_measurement = Eigen::Vector3d::Zero();
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

// Interpolates only between existing EKF snapshots. This does not re-integrate
// IMU data and never extrapolates before/after the stored trajectory.
bool interpolateImuDeskewPose(const StateHistory &history,
                              double stamp,
                              double max_gap_sec,
                              const Eigen::Vector3d &gravity_world,
                              const ImuKinematicsConfig &propagation_config,
                              ImuDeskewPose &pose,
                              std::string &reason);

// Transforms one LiDAR-frame point from its acquisition pose to the selected
// reference LiDAR pose using T_imu_lidar (p_imu = T_imu_lidar * p_lidar).
bool transformPointToReferenceLidar(const ImuDeskewPose &point_pose,
                                     const ImuDeskewPose &reference_pose,
                                     const Eigen::Isometry3d &T_imu_lidar,
                                     const Eigen::Vector3d &point_lidar,
                                     Eigen::Vector3d &point_reference,
                                     std::string &reason);

}  // namespace dog_prior_map_localization
