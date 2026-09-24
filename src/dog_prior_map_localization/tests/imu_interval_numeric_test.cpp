#include "dog_prior_map_localization/core/imu_propagation.hpp"

#include <cmath>
#include <iostream>

using dog_prior_map_localization::ImuIntervalInput;
using dog_prior_map_localization::ImuKinematicsConfig;

namespace
{
struct State
{
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
};

struct Result
{
  double orientation_error = 0.0;
  double velocity_error = 0.0;
  double position_error = 0.0;
  double deskew_point_error = 0.0;
};

const Eigen::Vector3d kOmega(0.15, -0.22, 0.31);
const Eigen::Vector3d kWorldAcceleration(0.7, -0.3, 0.2);
const double kGravity = 0.0;
const double kImuDt = 1.0 / 200.0;
const double kScanDuration = 0.1;
const double kPointTime = 0.0475;

Eigen::Matrix3d truthRotation(double t)
{
  const double angle = kOmega.norm() * t;
  return Eigen::AngleAxisd(angle, kOmega.normalized()).toRotationMatrix();
}

Eigen::Vector3d rawAcceleration(double t)
{
  return truthRotation(t).transpose() * kWorldAcceleration;
}

void integrateOne(State &state, double head_t, double tail_t, double dt,
                  bool midpoint_average)
{
  const ImuIntervalInput input = dog_prior_map_localization::makeImuIntervalInput(
      rawAcceleration(head_t), kOmega,
      rawAcceleration(tail_t), kOmega,
      midpoint_average ? dog_prior_map_localization::ImuIntervalInputPolicy::kMidpointAverage :
                         dog_prior_map_localization::ImuIntervalInputPolicy::kHeadSample);
  Eigen::Vector3d acc_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_unbiased = Eigen::Vector3d::Zero();
  ImuKinematicsConfig config;
  config.use_acc_for_position = true;
  config.midpoint_interval_input_enable = midpoint_average;
  config.continuous_gravity_correction_enable = false;
  dog_prior_map_localization::propagateImuKinematics(
      state.p, state.v, state.R, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      input.acc, input.gyro, dt, Eigen::Vector3d(0.0, 0.0, -kGravity),
      config, acc_world, gyro_unbiased);
}

State integrateTo(double target_t, bool midpoint_average)
{
  State state;
  const int full_steps = static_cast<int>(std::floor(target_t / kImuDt + 1e-10));
  for (int i = 0; i < full_steps; ++i)
  {
    const double head_t = static_cast<double>(i) * kImuDt;
    const double tail_t = head_t + kImuDt;
    integrateOne(state, head_t, tail_t, kImuDt, midpoint_average);
  }
  const double integrated_t = static_cast<double>(full_steps) * kImuDt;
  const double partial_dt = target_t - integrated_t;
  if (partial_dt > 1e-12)
  {
    integrateOne(state, integrated_t, integrated_t + kImuDt,
                 partial_dt, midpoint_average);
  }
  return state;
}

Result compare(bool midpoint_average)
{
  const State end = integrateTo(kScanDuration, midpoint_average);
  const Eigen::Matrix3d R_end_truth = truthRotation(kScanDuration);
  const Eigen::Vector3d v_end_truth = kWorldAcceleration * kScanDuration;
  const Eigen::Vector3d p_end_truth = 0.5 * kWorldAcceleration *
                                      kScanDuration * kScanDuration;

  const State point = integrateTo(kPointTime, midpoint_average);
  const Eigen::Vector3d point_lidar(2.0, -0.4, 0.7);
  const Eigen::Vector3d point_deskewed = point.p + point.R * point_lidar;
  const Eigen::Vector3d point_truth =
      0.5 * kWorldAcceleration * kPointTime * kPointTime +
      truthRotation(kPointTime) * point_lidar;

  Result result;
  result.orientation_error = Eigen::AngleAxisd(end.R * R_end_truth.transpose()).angle();
  result.velocity_error = (end.v - v_end_truth).norm();
  result.position_error = (end.p - p_end_truth).norm();
  result.deskew_point_error = (point_deskewed - point_truth).norm();
  return result;
}
}  // namespace

int main()
{
  const Result zoh = compare(false);
  const Result midpoint = compare(true);
  const bool finite = std::isfinite(zoh.orientation_error) &&
      std::isfinite(zoh.velocity_error) && std::isfinite(zoh.position_error) &&
      std::isfinite(zoh.deskew_point_error) &&
      std::isfinite(midpoint.orientation_error) && std::isfinite(midpoint.velocity_error) &&
      std::isfinite(midpoint.position_error) && std::isfinite(midpoint.deskew_point_error);
  const bool pass = finite && midpoint.orientation_error < 1e-10 &&
      midpoint.velocity_error < 5e-4 && midpoint.position_error < 1e-4 &&
      midpoint.deskew_point_error < 1e-3;

  std::cout << "trajectory,imu_hz,lidar_hz,scan_duration_sec,orientation_error_rad,"
               "velocity_error_mps,position_error_m,deskew_point_error_m,result\n";
  std::cout << "R9A_ZOH,200,10,0.1," << zoh.orientation_error << ","
            << zoh.velocity_error << "," << zoh.position_error << ","
            << zoh.deskew_point_error << "," << (finite ? "MEASURED" : "FAIL") << "\n";
  std::cout << "R9B_HEAD_TAIL_MEAN,200,10,0.1," << midpoint.orientation_error << ","
            << midpoint.velocity_error << "," << midpoint.position_error << ","
            << midpoint.deskew_point_error << "," << (pass ? "PASS" : "FAIL") << "\n";
  std::cerr << (pass ? "IMU_INTERVAL_NUMERIC_TEST_PASS\n" :
                       "IMU_INTERVAL_NUMERIC_TEST_FAIL\n");
  return pass ? 0 : 1;
}
