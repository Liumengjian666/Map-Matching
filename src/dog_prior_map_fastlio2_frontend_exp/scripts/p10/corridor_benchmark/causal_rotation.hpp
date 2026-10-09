#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include <algorithm>
#include <stdexcept>

namespace corridor_benchmark {
namespace p = dog_prior_map_fastlio2_frontend_exp;
inline void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}
inline Eigen::Matrix3d exp(const Eigen::Vector3d& vector) {
  const double norm = vector.norm();
  return norm < 1e-15 ? Eigen::Matrix3d::Identity() :
      Eigen::AngleAxisd(norm, vector / norm).toRotationMatrix();
}
// Same causal midpoint SO(3) integration contract as the archived V2 helper.
// This input adapter does not run its GICP/bootstrap or estimate gyro bias.
struct RotationTimeline {
  std::vector<uint64_t> knots;
  std::vector<Eigen::Matrix3d> rotations;
  uint64_t maximum_gap_ns = 0, endpoint_hold_ns = 0, maximum_consumed_ns = 0;
  RotationTimeline(const p::P7ImuVector& imu, uint64_t begin, uint64_t end) {
    require(end > begin, "invalid_rotation_interval");
    const auto upper = std::upper_bound(imu.begin(), imu.end(), end,
        [](uint64_t t, const p::ImuSample& s) { return t < s.stamp_ns; });
    require(upper != imu.begin(), "no_causal_IMU");
    auto first = std::upper_bound(imu.begin(), upper, begin,
        [](uint64_t t, const p::ImuSample& s) { return t < s.stamp_ns; });
    require(first != imu.begin(), "missing_leading_IMU");
    --first;
    maximum_consumed_ns = (upper - 1)->stamp_ns;
    endpoint_hold_ns = end - maximum_consumed_ns;
    require(endpoint_hold_ns <= 10000000, "endpoint_hold_exceeds_10ms");
    for (auto it = first + 1; it != upper; ++it)
      maximum_gap_ns = std::max(maximum_gap_ns, it->stamp_ns - (it - 1)->stamp_ns);
    require(maximum_gap_ns <= 20000000, "IMU_gap_exceeds_20ms");
    auto gyro = [&](uint64_t t) -> Eigen::Vector3d {
      auto hi = std::upper_bound(first, upper, t,
          [](uint64_t query, const p::ImuSample& s) { return query < s.stamp_ns; });
      require(hi != first, "IMU_backfill");
      if (hi == upper) return (upper - 1)->angular_velocity;
      auto lo = hi - 1;
      const double alpha = double(t - lo->stamp_ns) / double(hi->stamp_ns - lo->stamp_ns);
      return (1 - alpha) * lo->angular_velocity + alpha * hi->angular_velocity;
    };
    knots.push_back(begin);
    for (auto it = first; it != upper; ++it)
      if (it->stamp_ns > begin && it->stamp_ns < end) knots.push_back(it->stamp_ns);
    knots.push_back(end);
    rotations.push_back(Eigen::Matrix3d::Identity());
    for (std::size_t i = 1; i < knots.size(); ++i) {
      require(knots[i] - knots[i-1] <= 20000000, "integration_gap_exceeds_20ms");
      rotations.push_back(rotations.back() *
          exp(.5 * (gyro(knots[i-1]) + gyro(knots[i])) * (double(knots[i]-knots[i-1])*1e-9)));
    }
  }
  Eigen::Matrix3d at(uint64_t t) const {
    require(t >= knots.front() && t <= knots.back(), "rotation_outside_scan");
    if (t == knots.back()) return rotations.back();
    const std::size_t i = std::upper_bound(knots.begin(), knots.end(), t) - knots.begin() - 1;
    const Eigen::AngleAxisd delta(rotations[i].transpose() * rotations[i+1]);
    const double alpha = double(t-knots[i])/double(knots[i+1]-knots[i]);
    return rotations[i] * exp(delta.axis()*delta.angle()*alpha);
  }
};
inline Eigen::Vector3d rotationDeskew(const Eigen::Vector3d& point,
    const Eigen::Matrix3d& end_R_time, const Eigen::Matrix4d& extrinsic) {
  const Eigen::Matrix3d R = extrinsic.block<3,3>(0,0);
  const Eigen::Vector3d lever = extrinsic.block<3,1>(0,3);
  return R.transpose() * (end_R_time * (R * point + lever) - lever);
}
inline Eigen::Matrix4d predict(const Eigen::Matrix4d& last_lidar,
    const Eigen::Matrix4d& extrinsic, const Eigen::Matrix3d& delta_R_imu,
    const Eigen::Vector3d& velocity_map_imu, double dt) {
  Eigen::Matrix4d imu = last_lidar * extrinsic.inverse();
  imu.block<3,3>(0,0) = imu.block<3,3>(0,0).eval() * delta_R_imu;
  imu.block<3,1>(0,3) += velocity_map_imu * dt;
  return imu * extrinsic;
}
}  // namespace corridor_benchmark
