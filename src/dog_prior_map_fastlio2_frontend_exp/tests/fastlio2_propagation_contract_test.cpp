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

}  // namespace

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.initial_accel_bias = Eigen::Vector3d(0.0, 0.0, 0.0);
  FastLio2IkfomFrontend frontend(parameters);

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  stationary.reserve(parameters.static_init_samples);
  const Eigen::Vector3d gyro_bias(0.01, -0.02, 0.005);
  for (int index = 0; index < parameters.static_init_samples; ++index) {
    ImuSample sample;
    sample.stamp_ns = static_cast<uint64_t>(index) * 5000000ULL + 1000000ULL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2);
    sample.angular_velocity = gyro_bias;
    stationary.push_back(sample);
  }

  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.position = Eigen::Vector3d(1.5, -0.4, 0.7);
  initial_map_T_lidar.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  Pose3d T_imu_lidar;
  T_imu_lidar.position = Eigen::Vector3d(0.08, 0.029, 0.03);
  T_imu_lidar.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
  std::string failure;
  if (!require(frontend.initializeStatic(stationary, initial_map_T_lidar,
                                         T_imu_lidar, &failure),
               failure.c_str()))
    return 1;

  const FilterSnapshot initialized = frontend.getState();
  if (!require(initialized.stamp_ns == stationary.back().stamp_ns,
               "initial time is exact last static IMU stamp") ||
      !require((initialized.gyro_bias - gyro_bias).norm() < 1e-12,
               "static mean initializes gyro bias") ||
      !require(initialized.velocity.norm() == 0.0,
               "static initialization starts at zero velocity") ||
      !require(std::abs(initialized.gravity.norm() - parameters.gravity_mps2) < 1e-10,
               "gravity remains on fixed-magnitude S2") ||
      !require((initialized.T_imu_lidar_translation - T_imu_lidar.position).norm() == 0.0,
               "fixed calibrated lever arm is exact"))
    return 1;

  // Midpoint propagation uses the pinned get_f/df_dx/df_dw/kf.predict path.
  const Eigen::Vector3d world_acceleration(0.12, -0.04, 0.0);
  const Eigen::Vector3d angular_velocity(0.0, 0.0, 0.3);
  const uint64_t start_ns = initialized.stamp_ns;
  ImuSample previous = stationary.back();
  const Eigen::Matrix3d initial_rotation = initialized.map_T_imu.orientation.toRotationMatrix();
  for (int index = 1; index <= 20; ++index) {
    ImuSample next;
    next.stamp_ns = start_ns + static_cast<uint64_t>(index) * 5000000ULL;
    const double dt = static_cast<double>(next.stamp_ns - start_ns) * 1e-9;
    const Eigen::Matrix3d rotation = initial_rotation *
        Eigen::AngleAxisd(angular_velocity.z() * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d gravity = initialized.gravity;
    const Eigen::Vector3d specific_force =
        rotation.transpose() * (world_acceleration - gravity);
    next.acceleration = specific_force + parameters.initial_accel_bias;
    next.angular_velocity = angular_velocity + gyro_bias;
    if (!frontend.predictInterval(previous, next, &failure)) {
      std::cerr << "FAIL: interval predict: " << failure << '\n';
      return 1;
    }
    previous = next;
  }
  const FilterSnapshot propagated = frontend.getState();
  const double duration = 0.1;
  const Eigen::Vector3d expected_position = initialized.map_T_imu.position +
      initialized.velocity * duration + 0.5 * world_acceleration * duration * duration;
  const Eigen::Vector3d expected_velocity = world_acceleration * duration;
  const double position_error = (propagated.map_T_imu.position - expected_position).norm();
  const double velocity_error = (propagated.velocity - expected_velocity).norm();
  const double rotation_error = Eigen::AngleAxisd(
      initialized.map_T_imu.orientation.conjugate() *
      propagated.map_T_imu.orientation).angle();
  if (!require(propagated.stamp_ns == start_ns + 100000000ULL,
               "scan interval endpoint timestamp is exact") ||
      !require(position_error < 2e-3, "constant-acceleration position integration") ||
      !require(velocity_error < 2e-3, "constant-acceleration velocity integration") ||
      !require(std::abs(rotation_error - angular_velocity.z() * duration) < 1e-3,
               "constant angular velocity integration") ||
      !require(frontend.postconditionsValid(&failure), "state and covariance postconditions"))
    return 1;

  const auto process_noise = frontend.getProcessNoiseCovariance();
  if (!require(std::abs(process_noise(0, 0) - 0.01) < 1e-12,
               "gyro Q block maps sigma squared") ||
      !require(std::abs(process_noise(3, 3) - 4.0) < 1e-12,
               "accelerometer Q block maps sigma squared"))
    return 1;

  std::cout << "FASTLIO2_PROPAGATION_CONTRACT_PASS"
            << " position_error_m=" << position_error
            << " velocity_error_mps=" << velocity_error
            << " rotation_error_rad=" << rotation_error << '\n';
  return 0;
}
