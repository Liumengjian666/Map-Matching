#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {

bool check(bool value, const char* message) {
  if (!value) std::cerr << "FAIL: " << message << "\n";
  return value;
}

Eigen::Vector3d log3(const Eigen::Matrix3d& rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.axis() * angle_axis.angle();
}

std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples(
    const std::vector<std::uint64_t>& stamps, const Eigen::Vector3d& acceleration,
    const Eigen::Vector3d& gyro) {
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> result;
  for (std::uint64_t stamp : stamps) {
    ImuSample sample;
    sample.stamp_ns = stamp;
    sample.acceleration = acceleration;
    sample.angular_velocity = gyro;
    result.push_back(sample);
  }
  return result;
}

}  // namespace

int main() {
  ImuNoiseParameters noise;
  noise.gravity.setZero();
  ImuNoiseParameters white_noise_only = noise;
  white_noise_only.gyro_bias_random_walk = 0.0;
  white_noise_only.accel_bias_random_walk = 0.0;
  const std::vector<std::uint64_t> regular{1000000000ULL, 1100000000ULL,
                                            1200000000ULL, 1300000000ULL,
                                            1400000000ULL};
  const std::vector<std::uint64_t> irregular{1000000000ULL, 1030000000ULL,
                                              1110000000ULL, 1270000000ULL,
                                              1400000000ULL};
  ImuPreintegratedMeasurement static_measurement;
  std::string reason;
  if (!check(preintegrateImu(samples(irregular, Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero()),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                             white_noise_only, &static_measurement, &reason),
             "static preintegration")) return 1;
  if (!check(static_measurement.delta_velocity.norm() < 1e-12 &&
             static_measurement.delta_position.norm() < 1e-12,
             "static delta")) return 2;
  const double duration_s = static_cast<double>(irregular.back() -
      irregular.front()) * 1e-9;
  const double expected_gyro_trace = 3.0 *
      white_noise_only.gyro_noise_density *
      white_noise_only.gyro_noise_density * duration_s;
  if (!check(std::abs(static_measurement.covariance.block<3, 3>(0, 0).trace() -
                     expected_gyro_trace) < 1e-12,
             "continuous-time gyro noise discretization")) return 12;

  const Eigen::Vector3d constant_gyro(0.0, 0.0, 0.5);
  ImuPreintegratedMeasurement rotation_measurement;
  if (!check(preintegrateImu(samples(irregular, Eigen::Vector3d::Zero(),
                                      constant_gyro),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                             noise, &rotation_measurement, &reason),
             "constant angular-rate preintegration")) return 3;
  const Eigen::AngleAxisd rotation_angle(rotation_measurement.delta_rotation);
  if (!check(std::abs(rotation_angle.angle() - 0.2) < 1e-8,
             "constant angular-rate delta")) return 4;
  constexpr double bias_fd_step = 1e-6;
  ImuPreintegratedMeasurement gyro_bias_plus, gyro_bias_minus;
  const Eigen::Vector3d gyro_bias_probe(bias_fd_step, 0.0, 0.0);
  if (!check(preintegrateImu(samples(irregular, Eigen::Vector3d::Zero(),
                                     constant_gyro),
                             irregular.front(), irregular.back(),
                             gyro_bias_probe, Eigen::Vector3d::Zero(), noise,
                             &gyro_bias_plus, &reason) &&
             preintegrateImu(samples(irregular, Eigen::Vector3d::Zero(),
                                     constant_gyro),
                             irregular.front(), irregular.back(),
                             -gyro_bias_probe, Eigen::Vector3d::Zero(), noise,
                             &gyro_bias_minus, &reason),
             "gyro bias finite-difference preintegration")) return 13;
  const Eigen::Vector3d gyro_bias_fd =
      (log3(rotation_measurement.delta_rotation.transpose() *
            gyro_bias_plus.delta_rotation) -
       log3(rotation_measurement.delta_rotation.transpose() *
            gyro_bias_minus.delta_rotation)) /
      (2.0 * bias_fd_step);
  const double gyro_bias_fd_error = (gyro_bias_fd -
      rotation_measurement.jacobian_rotation_gyro_bias.col(0)).norm();
  if (!check(gyro_bias_fd_error < 2e-3,
             "gyro bias Jacobian finite difference")) {
    std::cerr << "gyro_bias_fd=" << gyro_bias_fd.transpose()
              << " analytic="
              << rotation_measurement.jacobian_rotation_gyro_bias.col(0).transpose()
              << " error=" << gyro_bias_fd_error << "\n";
    return 14;
  }

  const Eigen::Vector3d acceleration(0.0, 2.0, 0.0);
  ImuPreintegratedMeasurement acceleration_measurement;
  if (!check(preintegrateImu(samples(irregular, acceleration,
                                      Eigen::Vector3d::Zero()),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                             noise, &acceleration_measurement, &reason),
             "irregular acceleration preintegration")) return 5;
  if (!check(std::abs(acceleration_measurement.delta_velocity.y() - 0.8) < 1e-8 &&
             std::abs(acceleration_measurement.delta_position.y() - 0.16) < 1e-8,
             "irregular acceleration delta")) return 6;
  ImuPreintegratedMeasurement accel_bias_plus, accel_bias_minus;
  const Eigen::Vector3d accel_bias_probe(bias_fd_step, 0.0, 0.0);
  if (!check(preintegrateImu(samples(irregular, acceleration,
                                     Eigen::Vector3d::Zero()),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), accel_bias_probe, noise,
                             &accel_bias_plus, &reason) &&
             preintegrateImu(samples(irregular, acceleration,
                                     Eigen::Vector3d::Zero()),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), -accel_bias_probe, noise,
                             &accel_bias_minus, &reason),
             "accelerometer bias finite-difference preintegration")) return 15;
  const Eigen::Vector3d accel_bias_fd =
      (accel_bias_plus.delta_velocity - accel_bias_minus.delta_velocity) /
      (2.0 * bias_fd_step);
  if (!check((accel_bias_fd -
              acceleration_measurement.jacobian_velocity_accel_bias.col(0)).norm() <
                 1e-8,
             "accelerometer bias Jacobian finite difference")) return 16;

  const Eigen::Vector3d accel_bias(0.3, -0.1, 0.2);
  ImuPreintegratedMeasurement bias_corrected;
  if (!check(preintegrateImu(samples(irregular, accel_bias,
                                      Eigen::Vector3d::Zero()),
                             irregular.front(), irregular.back(),
                             Eigen::Vector3d::Zero(), accel_bias, noise,
                             &bias_corrected, &reason),
             "bias-corrected preintegration")) return 7;
  if (!check(bias_corrected.delta_velocity.norm() < 1e-12,
             "accelerometer bias correction")) return 8;

  WindowState from, to;
  from.stamp_ns = irregular.front();
  to.stamp_ns = irregular.back();
  to.position = acceleration_measurement.delta_position;
  to.velocity = acceleration_measurement.delta_velocity;
  Eigen::Matrix<double, 15, 15> jacobian_from, jacobian_to;
  Vector15d residual;
  if (!check(linearizeImuFactor(from, to, acceleration_measurement, noise,
                                &jacobian_from, &jacobian_to, &residual,
                                &reason),
             "IMU residual Jacobian")) return 9;
  if (!check(residual.norm() < 1e-8 && jacobian_from.allFinite() &&
             jacobian_to.allFinite(), "IMU residual/Jacobian finite difference")) return 10;

  auto bad = samples(regular, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  std::swap(bad[1].stamp_ns, bad[2].stamp_ns);
  ImuPreintegratedMeasurement rejected;
  if (!check(!preintegrateImu(bad, regular.front(), regular.back(),
                              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                              noise, &rejected, &reason),
             "nonmonotonic IMU rejection")) return 11;

  std::cout << "I6F_IMU_PREINTEGRATION_TEST_PASS"
            << " static=1 constant_gyro_angle=" << rotation_angle.angle()
            << " irregular_dv=" << acceleration_measurement.delta_velocity.y()
            << " irregular_dp=" << acceleration_measurement.delta_position.y()
            << " covariance_trace=" << acceleration_measurement.covariance.trace()
            << " bias_jacobian_norm=" << bias_corrected.jacobian_velocity_accel_bias.norm()
            << " gyro_bias_fd_error="
            << gyro_bias_fd_error
            << " accel_bias_fd_error="
            << (accel_bias_fd -
                acceleration_measurement.jacobian_velocity_accel_bias.col(0)).norm()
            << " jacobian_norm=" << jacobian_from.norm() << "\n";
  return 0;
}
