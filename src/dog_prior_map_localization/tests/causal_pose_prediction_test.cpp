#include "dog_prior_map_localization/core/causal_pose_prediction.hpp"

#include <cmath>
#include <iostream>

namespace {
bool near(double actual, double expected, double tolerance = 1e-10) {
  return std::abs(actual - expected) <= tolerance;
}
}

int main() {
  using dog_prior_map_localization::predictMapTLidarWithBodyTwist;
  constexpr double pi = 3.14159265358979323846;
  Eigen::Matrix4d initial = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d predicted;
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

  if (!predictMapTLidarWithBodyTwist(initial, zero, zero, 0.0, 0.02, &predicted) ||
      (predicted - initial).norm() > 1e-12) {
    std::cerr << "exact timestamp prediction failed\n";
    return 1;
  }

  initial.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  if (!predictMapTLidarWithBodyTwist(initial, Eigen::Vector3d(1.0, 0.0, 0.0),
      zero, 0.01, 0.02, &predicted) ||
      !near(predicted(0, 3), 0.0, 1e-10) ||
      !near(predicted(1, 3), 0.01, 1e-10)) {
    std::cerr << "body-frame translation direction failed\n";
    return 1;
  }

  initial.setIdentity();
  constexpr double omega = 2.0;
  constexpr double dt = 0.01;
  if (!predictMapTLidarWithBodyTwist(initial, Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, omega), dt, 0.02, &predicted) ||
      !near(predicted(0, 3), std::sin(omega * dt) / omega, 1e-10) ||
      !near(predicted(1, 3), (1.0 - std::cos(omega * dt)) / omega, 1e-10) ||
      !near(predicted(0, 0), std::cos(omega * dt), 1e-10) ||
      !near(predicted(1, 0), std::sin(omega * dt), 1e-10)) {
    std::cerr << "constant body-twist SE(3) exponential failed\n";
    return 1;
  }

  if (predictMapTLidarWithBodyTwist(initial, zero, zero, 0.020001, 0.02, &predicted)) {
    std::cerr << "stale sample age guard failed\n";
    return 1;
  }
  std::cout << "CAUSAL_BODY_TWIST_PREDICTION_CONTRACT_PASS\n";
  return 0;
}
