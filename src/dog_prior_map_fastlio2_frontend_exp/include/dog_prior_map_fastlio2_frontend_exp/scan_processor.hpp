#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct TimedLidarPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double intensity = 0.0;
  uint64_t stamp_ns = 0;
};

struct ScanEndResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t scan_end_ns = 0;
  Pose3d predicted_map_T_imu;
  Pose3d predicted_map_T_lidar;
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> imu_poses;
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloud_end_frame;
};

class ScanEndProcessor {
 public:
  bool process(FastLio2IkfomFrontend* candidate,
               const Pose3d& T_imu_lidar,
               uint64_t scan_start_ns,
               uint64_t scan_end_ns,
               const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
               const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
               ScanEndResult* result,
               std::string* failure_reason) const;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
