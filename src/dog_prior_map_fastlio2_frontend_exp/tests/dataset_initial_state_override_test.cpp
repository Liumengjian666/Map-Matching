#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {
bool require(bool condition, const char* message) {
  if (!condition) std::cerr << "FAIL: " << message << '\n';
  return condition;
}
}

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.initial_accel_bias.setZero();
  parameters.pose_position_sigma_m = 0.05;
  parameters.pose_rotation_sigma_rad = 0.03;

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> static_imu;
  static_imu.reserve(200);
  for (uint64_t i = 0; i < 200; ++i) {
    ImuSample sample;
    sample.stamp_ns = 1000000000ULL + i * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2);
    static_imu.push_back(sample);
  }

  FastLio2IkfomFrontend frontend(parameters);
  StaticImuCalibration calibration;
  std::string reason;
  if (!require(frontend.calibrateStaticImu(static_imu, &calibration, &reason),
               "static calibration accepted")) return 1;

  Pose3d initial_pose;
  initial_pose.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  initial_pose.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  Pose3d T_imu_lidar;
  T_imu_lidar.position = Eigen::Vector3d(0.08, 0.029, 0.03);
  const Eigen::Vector3d gravity_map(0.0, 0.0, -parameters.gravity_mps2);

  InitialStateOverrides initial_state;
  initial_state.use_initial_velocity = true;
  initial_state.velocity_world_m_s = Eigen::Vector3d(0.35, 2.91, -0.17);
  initial_state.use_initial_biases = true;
  initial_state.gyro_bias_rad_s.setZero();
  initial_state.accel_bias_m_s2.setZero();
  initial_state.use_covariance_overrides = true;
  initial_state.velocity_std_m_s = 0.5;
  initial_state.gyro_bias_std_rad_s = 0.05;
  initial_state.accel_bias_std_m_s2 = 0.5;

  const uint64_t epoch = static_imu.back().stamp_ns + 10000000ULL;
  if (!require(frontend.initializeFromStaticCalibration(
          calibration, initial_pose, T_imu_lidar, gravity_map,
          initial_state, epoch, &reason),
      "dataset state override initializes")) return 1;

  const FilterSnapshot initialized = frontend.getState();
  const auto& P0 = initialized.covariance;
  if (!require(initialized.stamp_ns == epoch, "navigation timestamp is the new epoch") ||
      !require((initialized.map_T_imu.position - initial_pose.position).norm() < 1e-12,
               "official position is preserved") ||
      !require((initialized.velocity - initial_state.velocity_world_m_s).norm() < 1e-12,
               "configured world velocity is injected once at initialization") ||
      !require(initialized.gyro_bias.norm() == 0.0 && initialized.accel_bias.norm() == 0.0,
               "configured zero biases override static calibration biases") ||
      !require(std::abs(P0(12, 12) - 0.25) < 1e-12 &&
                   std::abs(P0(15, 15) - 0.0025) < 1e-12 &&
                   std::abs(P0(18, 18) - 0.25) < 1e-12,
               "velocity and bias covariance blocks equal configured std squared") ||
      !require(std::abs(P0(0, 0) - 1.0) < 1e-12 &&
                   std::abs(P0(3, 3) - 1.0) < 1e-12,
               "pose covariance remains at the existing frozen baseline value") ||
      !require(frontend.postconditionsValid(&reason), "initialized state is valid"))
    return 1;

  // Generate a short non-degenerate motion interval. A subsequent pose-only
  // update must be able to correct v/bg/ba through propagated cross-covariance.
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> sequence;
  sequence.reserve(101);
  for (uint64_t i = 0; i <= 100; ++i) {
    ImuSample sample;
    sample.stamp_ns = epoch + i * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.35, -0.18, parameters.gravity_mps2 + 0.12);
    sample.angular_velocity = Eigen::Vector3d(0.08, -0.04, 0.32);
    sequence.push_back(sample);
  }
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> poses;
  if (!require(frontend.predictImuSequence(sequence, sequence.back().stamp_ns,
                                            &poses, &reason),
               "short motion prediction succeeds")) return 1;
  const FilterSnapshot predicted = frontend.getState();
  Pose3d measurement = predicted.map_T_imu;
  measurement.position += Eigen::Vector3d(0.12, -0.07, 0.03);
  measurement.orientation = (measurement.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.035, Eigen::Vector3d(0.2, -0.1, 0.97).normalized()))).normalized();
  PoseCorrectionDelta delta;
  if (!require(frontend.applyPoseMeasurement(measurement, &delta, &reason),
               "pose measurement update succeeds")) return 1;
  const FilterSnapshot corrected = frontend.getState();
  if (!require(delta.velocity.norm() > 1e-7,
               "pose update changes velocity through cross-covariance") ||
      !require(delta.gyro_bias.norm() > 1e-8,
               "pose update changes gyro bias through cross-covariance") ||
      !require(delta.accel_bias.norm() > 1e-8,
               "pose update changes accelerometer bias through cross-covariance") ||
      !require(corrected.covariance.allFinite(), "posterior covariance remains finite") ||
      !require(frontend.postconditionsValid(&reason), "posterior state is valid"))
    return 1;

  FastLio2IkfomFrontend gravity_frontend(parameters);
  InitialStateOverrides gravity_override = initial_state;
  gravity_override.use_initial_gravity = true;
  gravity_override.gravity_map_m_s2 =
      Eigen::Vector3d(0.76200002, 0.06218908, -9.77915996);
  if (!require(gravity_frontend.initializeFromStaticCalibration(
          calibration, initial_pose, T_imu_lidar, gravity_map,
          gravity_override, epoch, &reason),
      "dataset-specific gravity override initializes")) return 1;
  const FilterSnapshot gravity_initialized = gravity_frontend.getState();
  if (!require((gravity_initialized.gravity - gravity_override.gravity_map_m_s2).norm() < 1e-8,
               "configured dataset gravity is inserted at initialization") ||
      !require(std::abs(gravity_initialized.gravity.norm() - parameters.gravity_mps2) < 1e-8,
               "gravity override preserves the configured magnitude"))
    return 1;

  FastLio2IkfomFrontend default_gravity_frontend(parameters);
  InitialStateOverrides default_gravity = initial_state;
  default_gravity.use_initial_gravity = false;
  if (!require(default_gravity_frontend.initializeFromStaticCalibration(
          calibration, initial_pose, T_imu_lidar, gravity_map,
          default_gravity, epoch, &reason),
      "legacy gravity path initializes when override is disabled")) return 1;
  if (!require((default_gravity_frontend.getState().gravity - gravity_map).norm() < 1e-8,
               "disabled gravity override preserves the supplied legacy gravity"))
    return 1;

  std::cout << "DATASET_INITIAL_STATE_OVERRIDE_PASS"
            << " P_v0=" << P0(12, 12)
            << " P_bg0=" << P0(15, 15)
            << " P_ba0=" << P0(18, 18)
            << " update_dv=" << delta.velocity.norm()
            << " update_dbg=" << delta.gyro_bias.norm()
            << " update_dba=" << delta.accel_bias.norm()
            << " gravity_override_norm=" << gravity_initialized.gravity.norm()
            << " legacy_gravity_error="
            << (default_gravity_frontend.getState().gravity - gravity_map).norm() << '\n';
  return 0;
}
