#include "dog_prior_map_localization/core/imu_deskew.hpp"

#include <cmath>
#include <iostream>

using dog_prior_map_localization::FilterStateSnapshot;
using dog_prior_map_localization::ImuDeskewCoverage;
using dog_prior_map_localization::ImuDeskewPose;
using dog_prior_map_localization::ImuKinematicsConfig;
using dog_prior_map_localization::StateHistory;

namespace
{
bool near(double a, double b, double tolerance = 1e-9)
{
  return std::abs(a - b) <= tolerance;
}

FilterStateSnapshot snapshot(double stamp, const Eigen::Vector3d &p,
                             const Eigen::Matrix3d &R = Eigen::Matrix3d::Identity())
{
  FilterStateSnapshot s;
  s.stamp = stamp;
  s.p = p;
  s.R = R;
  return s;
}
}  // namespace

int main()
{
  StateHistory history;
  history.insertMonotonic(snapshot(1.0, Eigen::Vector3d(0.0, 0.0, 0.0)));
  history.insertMonotonic(snapshot(1.01, Eigen::Vector3d(0.1, 0.0, 0.0)));
  history.insertMonotonic(snapshot(1.02, Eigen::Vector3d(0.2, 0.0, 0.0),
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix()));

  ImuDeskewCoverage coverage;
  std::string reason;
  if (!dog_prior_map_localization::inspectImuDeskewCoverage(
          history, 1.002, 1.018, 0.011, coverage, reason) ||
      coverage.samples_in_scan != 1 || !near(coverage.max_gap_sec, 0.01))
  {
    std::cerr << "coverage contract failed: " << reason << "\n";
    return 1;
  }
  if (dog_prior_map_localization::inspectImuDeskewCoverage(
          history, 0.99, 1.018, 0.011, coverage, reason))
  {
    std::cerr << "missing-start coverage was accepted\n";
    return 1;
  }
  if (dog_prior_map_localization::inspectImuDeskewCoverage(
          history, 1.002, 1.03, 0.011, coverage, reason) ||
      reason != "waiting_for_end_coverage")
  {
    std::cerr << "future-state extrapolation/coverage contract failed: " << reason << "\n";
    return 1;
  }

  ImuDeskewPose at_1_005, at_1_015;
  ImuKinematicsConfig model;
  model.continuous_gravity_correction_enable = false;
  if (!dog_prior_map_localization::interpolateImuDeskewPose(
          history, 1.005, 0.011, Eigen::Vector3d(0.0, 0.0, -9.80665), model,
          at_1_005, reason) ||
      !near(at_1_005.p.x(), 0.0) ||
      !dog_prior_map_localization::interpolateImuDeskewPose(
          history, 1.015, 0.011, Eigen::Vector3d(0.0, 0.0, -9.80665), model,
          at_1_015, reason))
  {
    std::cerr << "state interpolation contract failed: " << reason << "\n";
    return 1;
  }

  // A point between two IMU timestamps may use the lower (already received)
  // sample, but changing the upper sample must not change that point pose.
  StateHistory causal_history_a;
  StateHistory causal_history_b;
  FilterStateSnapshot lower = snapshot(2.0, Eigen::Vector3d::Zero());
  lower.acc_measurement = Eigen::Vector3d(1.0, 0.0, 9.80665);
  lower.gyro_measurement = Eigen::Vector3d(0.0, 0.0, 0.2);
  FilterStateSnapshot upper_a = snapshot(2.01, Eigen::Vector3d(5.0, -2.0, 1.0));
  upper_a.acc_measurement = Eigen::Vector3d::Zero();
  upper_a.gyro_measurement = Eigen::Vector3d::Zero();
  FilterStateSnapshot upper_b = upper_a;
  upper_b.acc_measurement = Eigen::Vector3d(1000.0, -2000.0, 3000.0);
  upper_b.gyro_measurement = Eigen::Vector3d(-20.0, 30.0, 40.0);
  causal_history_a.insertMonotonic(lower);
  causal_history_a.insertMonotonic(upper_a);
  causal_history_b.insertMonotonic(lower);
  causal_history_b.insertMonotonic(upper_b);

  ImuKinematicsConfig causal_model;
  causal_model.use_acc_for_position = true;
  causal_model.continuous_gravity_correction_enable = false;
  ImuDeskewPose causal_pose_a, causal_pose_b;
  if (!dog_prior_map_localization::interpolateImuDeskewPose(
          causal_history_a, 2.005, 0.011, Eigen::Vector3d(0.0, 0.0, -9.80665),
          causal_model, causal_pose_a, reason) ||
      !dog_prior_map_localization::interpolateImuDeskewPose(
          causal_history_b, 2.005, 0.011, Eigen::Vector3d(0.0, 0.0, -9.80665),
          causal_model, causal_pose_b, reason) ||
      causal_pose_a.latest_source_stamp > 2.005 + 1e-9 ||
      !near(causal_pose_a.latest_source_stamp, 2.0) ||
      !near(causal_pose_a.p.x(), 0.5 * 1.0 * 0.005 * 0.005) ||
      !near(causal_pose_a.v.norm(), 0.005, 1e-8) ||
      (causal_pose_a.p - causal_pose_b.p).norm() > 1e-12 ||
      (causal_pose_a.v - causal_pose_b.v).norm() > 1e-12 ||
      (causal_pose_a.R - causal_pose_b.R).norm() > 1e-12)
  {
    std::cerr << "future IMU sample affected point-time pose: " << reason
              << " source=" << causal_pose_a.latest_source_stamp
              << " p_a=" << causal_pose_a.p.transpose()
              << " p_b=" << causal_pose_b.p.transpose()
              << " v_a=" << causal_pose_a.v.transpose()
              << " v_b=" << causal_pose_b.v.transpose()
              << " R_diff=" << (causal_pose_a.R - causal_pose_b.R).norm()
              << " expected_px=" << 0.5 * 1.0 * 0.005 * 0.005 << "\n";
    return 1;
  }

  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.translation() = Eigen::Vector3d(0.0, 1.0, 0.0);
  Eigen::Vector3d point_reference;
  ImuDeskewPose start_pose;
  start_pose.p.setZero();
  start_pose.R.setIdentity();
  ImuDeskewPose end_pose;
  end_pose.p = Eigen::Vector3d(1.0, 0.0, 0.0);
  end_pose.R = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (!dog_prior_map_localization::transformPointToReferenceLidar(
          start_pose, end_pose, T_imu_lidar, Eigen::Vector3d::Zero(),
          point_reference, reason))
  {
    std::cerr << "SE(3) transform failed: " << reason << "\n";
    return 1;
  }
  // A stationary LiDAR-origin point expressed in the end LiDAR frame must
  // include both world translation and the rotated IMU/LiDAR lever arm.
  const Eigen::Vector3d expected(1.0, 0.0, 0.0);
  if ((point_reference - expected).norm() > 1e-9)
  {
    std::cerr << "SE(3) lever-arm transform mismatch: " << point_reference.transpose() << "\n";
    return 1;
  }
  std::cout << "IMU_DESKEW_CONTRACT_PASS\n";
  return 0;
}
