#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <Eigen/Dense>

#include "dog_prior_map_localization/core/selective_relative_visual_update.hpp"

namespace dog_prior_map_localization
{

struct SkfVisualMeasurement
{
  uint64_t scan_ns = 0;
  uint64_t reference_ns = 0;
  uint64_t current_ns = 0;
  uint64_t depth_max_point_ns = 0;
  uint64_t depth_max_imu_ns = 0;
  uint64_t max_imu_ns = 0;
  Eigen::Matrix4d relative_pose = Eigen::Matrix4d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  uint32_t pnp_inliers = 0;
  double inlier_ratio = 0.0;
  double reprojection_rmse_px = 0.0;
  double grid_occupancy = 0.0;
  double hull_fraction = 0.0;
  std::string covariance_model;
};

class SkfVisualMeasurementCatalog
{
public:
  void load(const std::string& path);
  const SkfVisualMeasurement* find(uint64_t scan_ns) const;
  std::size_t size() const noexcept { return measurements_.size(); }

private:
  std::map<uint64_t, SkfVisualMeasurement> measurements_;
};

}  // namespace dog_prior_map_localization
