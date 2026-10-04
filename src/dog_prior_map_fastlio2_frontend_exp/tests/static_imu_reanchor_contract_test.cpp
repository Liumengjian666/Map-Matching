#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"

#include <cmath>
#include <iostream>
#include <string>

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
  parameters.gravity_mps2 = 9.809;
  parameters.initial_accel_bias = Eigen::Vector3d(0.001, -0.002, 0.003);

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  stationary.reserve(200);
  const Eigen::Vector3d measured_acceleration(0.1, -0.2, 9.7);
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.005);
  for (uint64_t index = 0; index < 200; ++index) {
    ImuSample sample;
    sample.stamp_ns = 1000000000ULL + index * 5000000ULL;
    sample.acceleration = measured_acceleration;
    sample.angular_velocity = gyro_bias;
    stationary.push_back(sample);
  }

  FastLio2IkfomFrontend frontend(parameters);
  StaticImuCalibration calibration;
  std::string reason;
  if (!require(frontend.calibrateStaticImu(stationary, &calibration, &reason),
               reason.c_str()))
    return 1;
  const Eigen::Vector3d expected_force =
      measured_acceleration - parameters.initial_accel_bias;
  if (!require(calibration.gate_passed && calibration.sample_count == 200,
               "calibration records the frozen static gate result") ||
      !require(calibration.start_stamp_ns == stationary.front().stamp_ns &&
                   calibration.end_stamp_ns == stationary.back().stamp_ns,
               "calibration retains its own source epoch") ||
      !require((calibration.gyro_bias - gyro_bias).norm() < 1e-12,
               "static mean is the gyro bias estimate") ||
      !require((calibration.accel_bias_prior - parameters.initial_accel_bias).norm() < 1e-12,
               "accelerometer bias remains the configured prior") ||
      !require((calibration.mean_specific_force - expected_force).norm() < 1e-12,
               "specific force is mean acceleration minus configured bias") ||
      !require(calibration.acceleration_std.maxCoeff() == 0.0 &&
                   calibration.gyro_std.maxCoeff() == 0.0,
               "calibration records per-axis sample standard deviations"))
    return 1;

  Pose3d official_map_T_imu;
  official_map_T_imu.position = Eigen::Vector3d(0.3, 0.4, 1.4);
  official_map_T_imu.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));
  Pose3d T_imu_lidar;
  T_imu_lidar.position = Eigen::Vector3d(0.08, 0.029, 0.03);
  T_imu_lidar.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d gravity_map(0.0, 0.0, -parameters.gravity_mps2);
  const uint64_t navigation_epoch = stationary.back().stamp_ns + 61000000000ULL;
  const Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
  if (!require(frontend.initializeFromStaticCalibration(
                   calibration, official_map_T_imu, T_imu_lidar, gravity_map,
                   zero_velocity, navigation_epoch, &reason), reason.c_str()))
    return 1;

  const FilterSnapshot reanchored = frontend.getState();
  const double rotation_error = Eigen::AngleAxisd(
      official_map_T_imu.orientation.conjugate() *
      reanchored.map_T_imu.orientation).angle();
  if (!require(reanchored.stamp_ns == navigation_epoch,
               "navigation timestamp is reset to the new epoch") ||
      !require((reanchored.map_T_imu.position - official_map_T_imu.position).norm() < 1e-12,
               "navigation position is the supplied official anchor") ||
      !require(rotation_error < 1e-12,
               "navigation orientation is not gravity-aligned or propagated") ||
      !require(reanchored.velocity.norm() == 0.0,
               "zero-velocity start assumption is explicit") ||
      !require((reanchored.gyro_bias - gyro_bias).norm() < 1e-12,
               "only gyro bias is transferred from static samples") ||
      !require((reanchored.accel_bias - parameters.initial_accel_bias).norm() < 1e-12,
               "configured accelerometer bias prior is transferred") ||
      !require((reanchored.gravity - gravity_map).norm() < 1e-9,
               "map gravity is supplied independently at the new epoch") ||
      !require(reanchored.covariance.allFinite(),
               "fresh navigation covariance is finite") ||
      !require((reanchored.T_imu_lidar_translation - T_imu_lidar.position).norm() < 1e-12,
               "calibrated LiDAR lever arm is retained") ||
      !require(frontend.postconditionsValid(&reason),
               "reanchored state satisfies frontend postconditions"))
    return 1;

  FastLio2IkfomFrontend invalid_epoch_frontend(parameters);
  if (!require(!invalid_epoch_frontend.initializeFromStaticCalibration(
                   calibration, official_map_T_imu, T_imu_lidar, gravity_map,
                   zero_velocity, calibration.end_stamp_ns, &reason),
               "reanchor rejects an epoch that is not after calibration") ||
      !require(reason == "invalid_or_noncausal_static_calibration",
               "noncausal epoch rejection is explicit"))
    return 1;

  P7ImuVector rotation_samples;
  for (uint64_t index = 0; index <= 10; ++index) {
    ImuSample sample;
    sample.stamp_ns = 2000000000ULL + index * 50000000ULL;
    sample.angular_velocity = Eigen::Vector3d(0.0, 0.0, 0.4);
    rotation_samples.push_back(sample);
  }
  Eigen::Matrix3d relative_rotation;
  if (!require(integrateBodyRelativeRotation(
                   rotation_samples, 2025000000ULL, 2475000000ULL,
                   Eigen::Vector3d::Zero(), &relative_rotation, &reason),
               reason.c_str()))
    return 1;
  const Eigen::Matrix3d expected_relative_rotation =
      Eigen::AngleAxisd(0.4 * 0.45, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (!require((relative_rotation - expected_relative_rotation).norm() < 1e-12,
               "gyro-only relative rotation uses right/body composition and interpolation"))
    return 1;

  std::cout << "STATIC_IMU_REANCHOR_CONTRACT_PASS"
            << " samples=" << calibration.sample_count
            << " static_span_ns="
            << calibration.end_stamp_ns - calibration.start_stamp_ns
            << " navigation_stamp_ns=" << reanchored.stamp_ns << '\n';
  return 0;
}
