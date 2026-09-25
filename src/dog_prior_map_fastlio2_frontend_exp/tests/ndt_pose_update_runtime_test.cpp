#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.initial_accel_bias = Eigen::Vector3d(0.02, -0.01, 0.03);
  parameters.pose_position_sigma_m = 0.04;
  parameters.pose_rotation_sigma_rad = 0.025;
  FastLio2IkfomFrontend committed(parameters);
  const Eigen::Vector3d gyro_bias(0.009, -0.016, 0.004);
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  for (int i = 0; i < parameters.static_init_samples; ++i) {
    ImuSample sample;
    sample.stamp_ns = 1000000000ULL + static_cast<uint64_t>(i) * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.02, -0.01, parameters.gravity_mps2 + 0.03);
    sample.angular_velocity = gyro_bias;
    stationary.push_back(sample);
  }
  Pose3d extrinsic;
  extrinsic.position = Eigen::Vector3d(0.12, -0.035, 0.025);
  extrinsic.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.015, Eigen::Vector3d::UnitY()));
  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.position = extrinsic.position;
  initial_map_T_lidar.orientation = extrinsic.orientation;
  std::string failure;
  if (!committed.initializeStatic(stationary, initial_map_T_lidar, extrinsic, &failure)) {
    std::cerr << "FAIL: initialization: " << failure << '\n';
    return 1;
  }

  const uint64_t start_ns = stationary.back().stamp_ns;
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> sequence;
  sequence.push_back(stationary.back());
  const Eigen::Vector3d acceleration(0.18, 0.07, 0.0);
  const Eigen::Vector3d angular_velocity(0.03, -0.02, 0.42);
  for (int i = 1; i <= 100; ++i) {
    ImuSample sample;
    sample.stamp_ns = start_ns + static_cast<uint64_t>(i) * 5000000ULL;
    const double dt = static_cast<double>(sample.stamp_ns - start_ns) * 1e-9;
    const Eigen::Vector3d rv = angular_velocity * dt;
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        rv.norm(), rv.norm() > 0 ? rv.normalized() : Eigen::Vector3d::UnitX()));
    sample.acceleration = rotation.conjugate() *
        (acceleration + Eigen::Vector3d(0, 0, parameters.gravity_mps2)) +
        parameters.initial_accel_bias;
    sample.angular_velocity = angular_velocity + gyro_bias;
    sequence.push_back(sample);
  }
  std::unique_ptr<FastLio2IkfomFrontend> candidate = committed.cloneCandidate();
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> poses;
  const uint64_t scan_end_ns = sequence.back().stamp_ns;
  if (!candidate->predictImuSequence(sequence, scan_end_ns, &poses, &failure)) {
    std::cerr << "FAIL: candidate prediction: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot predicted = candidate->getState();
  Pose3d measurement = predicted.map_T_imu;
  measurement.position += Eigen::Vector3d(-0.08, 0.035, 0.025);
  measurement.orientation = (measurement.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.06, Eigen::Vector3d(0.1, -0.2, 0.97).normalized()))).normalized();

  std::unique_ptr<FastLio2IkfomFrontend> shadow = candidate->cloneCandidate();
  PoseCorrectionDelta delta;
  if (!shadow->applyPoseMeasurement(measurement, &delta, &failure)) {
    std::cerr << "FAIL: generic pose update: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot corrected = shadow->getState();
  const double position_error_before = (predicted.map_T_imu.position - measurement.position).norm();
  const double position_error_after = (corrected.map_T_imu.position - measurement.position).norm();
  const double rotation_error_before = Eigen::AngleAxisd(
      predicted.map_T_imu.orientation.conjugate() * measurement.orientation).angle();
  const double rotation_error_after = Eigen::AngleAxisd(
      corrected.map_T_imu.orientation.conjugate() * measurement.orientation).angle();
  if (!(position_error_after < position_error_before && rotation_error_after < rotation_error_before) ||
      !shadow->postconditionsValid(&failure) ||
      (corrected.T_imu_lidar_translation - extrinsic.position).norm() > 1e-12 ||
      (corrected.T_imu_lidar_rotation - extrinsic.orientation.toRotationMatrix()).norm() > 1e-12) {
    std::cerr << "FAIL: update direction/postconditions/extrinsic: " << failure << '\n';
    return 1;
  }

  if (!committed.commitCandidate(*candidate, &failure) ||
      committed.getState().stamp_ns != scan_end_ns) {
    std::cerr << "FAIL: prediction-only candidate commit: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot after_prediction_only = committed.getState();
  if ((after_prediction_only.map_T_imu.position - predicted.map_T_imu.position).norm() > 1e-12 ||
      (after_prediction_only.velocity - predicted.velocity).norm() > 1e-12) {
    std::cerr << "FAIL: prediction-only commit changed candidate state\n";
    return 1;
  }

  std::cout << "NDT_POSE_UPDATE_RUNTIME_CONTRACT_PASS"
            << " pos_before_m=" << position_error_before
            << " pos_after_m=" << position_error_after
            << " rot_before_rad=" << rotation_error_before
            << " rot_after_rad=" << rotation_error_after
            << " delta_v=" << delta.velocity.norm()
            << " delta_bg=" << delta.gyro_bias.norm()
            << " delta_ba=" << delta.accel_bias.norm()
            << " delta_gravity=" << delta.gravity_tangent.norm() << '\n';
  return 0;
}
