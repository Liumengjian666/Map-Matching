#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"

#include <cstddef>
#include <cstdint>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct RegistrationPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};
using RegistrationCloud = std::vector<RegistrationPoint>;

struct CurrentFrameNdtParameters {
  double map_voxel_m = 0.15;
  double target_voxel_m = 0.15;
  double source_voxel_m = 0.25;
  int max_source_points = 1400;
  double min_range_m = 0.5;
  double max_range_m = 80.0;
  int min_effective_points = 50;
  double resolution_m = 0.8;
  double step_size = 0.08;
  double transformation_epsilon = 1.0e-5;
  int maximum_iterations = 80;
};

enum class CurrentFrameNdtStatus {
  SUCCESS,
  INSUFFICIENT_POINTS,
  NOT_CONVERGED,
  ZERO_ITERATION_PASSTHROUGH,
  ITERATION_LIMIT_EXHAUSTED,
  NONFINITE_TERMINAL
};

CurrentFrameNdtStatus classifyNdtTerminal(bool converged, int iterations,
    int maximum_iterations, bool terminal_pose_finite, bool fitness_finite);
const char* currentFrameNdtStatusName(CurrentFrameNdtStatus status);

// Pure source preparation/hash interfaces; no PCL types escape this module.
RegistrationCloud preprocessRegistrationCloud(const RegistrationCloud& raw,
    const CurrentFrameNdtParameters& parameters);
uint64_t registrationCloudHash(const RegistrationCloud& prepared);

struct CurrentFrameNdtResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t stamp_ns = 0;
  CurrentFrameNdtStatus status = CurrentFrameNdtStatus::NOT_CONVERGED;
  bool converged = false;
  bool effective = false;
  int iterations = 0;
  std::size_t source_point_count = 0;
  std::size_t target_point_count = 0;
  uint64_t source_cloud_hash = 0;
  double fitness = std::numeric_limits<double>::quiet_NaN();
  double transformation_probability = std::numeric_limits<double>::quiet_NaN();
  double alignment_ms = std::numeric_limits<double>::quiet_NaN();
  Pose3d initial_map_T_lidar;
  Pose3d raw_map_T_lidar;
};

struct CurrentFrameNdtOverlap {
  std::size_t evaluated_points = 0;
  std::array<double, 4> fraction_within{{0.0, 0.0, 0.0, 0.0}};  // <0.2, <0.3, <0.5, <1.0 m
  double mean_nearest_distance_m = std::numeric_limits<double>::quiet_NaN();
  double median_nearest_distance_m = std::numeric_limits<double>::quiet_NaN();
  double p90_nearest_distance_m = std::numeric_limits<double>::quiet_NaN();
  double p95_nearest_distance_m = std::numeric_limits<double>::quiet_NaN();
};

class CurrentFrameNdtRegistration {
 public:
  explicit CurrentFrameNdtRegistration(const CurrentFrameNdtParameters& parameters);
  ~CurrentFrameNdtRegistration();
  bool loadMap(const std::string& pcd_path, std::string* reason);
  bool ready() const;
  std::size_t targetPointCount() const;
  std::array<float, 3> targetGridLeafSizeMeters() const;
  bool align(uint64_t stamp_ns, const RegistrationCloud& raw_cloud,
      const Pose3d& initial_map_T_lidar, CurrentFrameNdtResult* result,
      std::string* reason);
  // Diagnostic only: applies the frozen source preprocessing and queries the
  // same preprocessed target cloud used to build the runtime NDT target.
  bool evaluateMapOverlap(const RegistrationCloud& raw_cloud,
      const Pose3d& map_T_lidar, CurrentFrameNdtOverlap* result,
      std::string* reason) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
