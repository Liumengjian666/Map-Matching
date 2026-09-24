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

  // TEST-A: an IMU sample after scan_end must not affect a point in the scan.
  StateHistory bounded_history_a;
  StateHistory bounded_history_b;
  FilterStateSnapshot lower = snapshot(2.0, Eigen::Vector3d::Zero());
  lower.acc_measurement = Eigen::Vector3d(1.0, 0.0, 0.0);
  lower.gyro_measurement = Eigen::Vector3d(0.0, 0.0, 0.2);
  FilterStateSnapshot upper_a = snapshot(2.01, Eigen::Vector3d(5.0, -2.0, 1.0));
  upper_a.acc_measurement = Eigen::Vector3d(3.0, 0.0, 0.0);
  upper_a.gyro_measurement = Eigen::Vector3d(0.0, 0.0, 0.4);
  FilterStateSnapshot upper_b = upper_a;
  upper_b.acc_measurement = Eigen::Vector3d(1000.0, -2000.0, 3000.0);
  upper_b.gyro_measurement = Eigen::Vector3d(-20.0, 30.0, 40.0);
  FilterStateSnapshot post_scan_a = snapshot(2.02, Eigen::Vector3d::Zero());
  post_scan_a.acc_measurement = Eigen::Vector3d::Zero();
  post_scan_a.gyro_measurement = Eigen::Vector3d::Zero();
  FilterStateSnapshot post_scan_b = post_scan_a;
  post_scan_b.acc_measurement = Eigen::Vector3d(-1e6, 2e6, -3e6);
  post_scan_b.gyro_measurement = Eigen::Vector3d(1e4, -2e4, 3e4);
  bounded_history_a.insertMonotonic(lower);
  bounded_history_a.insertMonotonic(upper_a);
  bounded_history_a.insertMonotonic(post_scan_a);
  bounded_history_b.insertMonotonic(lower);
  bounded_history_b.insertMonotonic(upper_b);
  bounded_history_b.insertMonotonic(post_scan_b);

  ImuKinematicsConfig midpoint_model;
  midpoint_model.use_acc_for_position = true;
  midpoint_model.midpoint_interval_input_enable = true;
  midpoint_model.continuous_gravity_correction_enable = false;
  ImuDeskewPose bounded_pose_a, bounded_pose_b;
  if (!dog_prior_map_localization::interpolateImuDeskewPose(
          bounded_history_a, 2.005, 0.011, Eigen::Vector3d::Zero(),
          midpoint_model, bounded_pose_a, reason, 2.007) ||
      !dog_prior_map_localization::interpolateImuDeskewPose(
          bounded_history_b, 2.005, 0.011, Eigen::Vector3d::Zero(),
          midpoint_model, bounded_pose_b, reason, 2.007) ||
      bounded_pose_a.latest_source_stamp > 2.007 + 1e-9 ||
      !near(bounded_pose_a.latest_source_stamp, 2.0) ||
      (bounded_pose_a.p - bounded_pose_b.p).norm() > 1e-12 ||
      (bounded_pose_a.v - bounded_pose_b.v).norm() > 1e-12 ||
      (bounded_pose_a.R - bounded_pose_b.R).norm() > 1e-12)
  {
    std::cerr << "TEST-A post-scan-end IMU influenced scan: " << reason
              << " source=" << bounded_pose_a.latest_source_stamp
              << " p_diff=" << (bounded_pose_a.p - bounded_pose_b.p).norm()
              << " v_diff=" << (bounded_pose_a.v - bounded_pose_b.v).norm()
              << " R_diff=" << (bounded_pose_a.R - bounded_pose_b.R).norm() << "\n";
    return 1;
  }

  // TEST-B: head/tail measurements from a legal within-scan interval drive
  // the partial propagation through their midpoint average.
  StateHistory interval_history_a;
  StateHistory interval_history_b;
  interval_history_a.insertMonotonic(lower);
  interval_history_a.insertMonotonic(upper_a);
  FilterStateSnapshot interval_upper_b = upper_a;
  interval_upper_b.acc_measurement = Eigen::Vector3d(5.0, 0.0, 0.0);
  interval_upper_b.gyro_measurement = Eigen::Vector3d(0.0, 0.0, 0.8);
  interval_history_b.insertMonotonic(lower);
  interval_history_b.insertMonotonic(interval_upper_b);
  ImuDeskewPose interval_pose_a, interval_pose_b;
  if (!dog_prior_map_localization::interpolateImuDeskewPose(
          interval_history_a, 2.005, 0.011, Eigen::Vector3d::Zero(),
          midpoint_model, interval_pose_a, reason, 2.01) ||
      !dog_prior_map_localization::interpolateImuDeskewPose(
          interval_history_b, 2.005, 0.011, Eigen::Vector3d::Zero(),
          midpoint_model, interval_pose_b, reason, 2.01) ||
      !near(interval_pose_a.latest_source_stamp, 2.01) ||
      (interval_pose_a.p - interval_pose_b.p).norm() < 1e-8 ||
      (interval_pose_a.v - interval_pose_b.v).norm() < 1e-5 ||
      (interval_pose_a.R - interval_pose_b.R).norm() < 1e-8 ||
      !near(interval_pose_a.interval_acc_input.x(), 2.0) ||
      !near(interval_pose_a.interval_gyro_input.z(), 0.3))
  {
    std::cerr << "TEST-B legal head/tail interval did not affect partial propagation: " << reason
              << " source=" << interval_pose_a.latest_source_stamp
              << " input_acc=" << interval_pose_a.interval_acc_input.transpose()
              << " input_gyro=" << interval_pose_a.interval_gyro_input.transpose() << "\n";
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
