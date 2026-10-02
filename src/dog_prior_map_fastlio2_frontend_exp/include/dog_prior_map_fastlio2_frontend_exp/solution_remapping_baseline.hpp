#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <limits>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp {

// Mature LIO-SAM / X-ICP solution-remapping baseline, NOT a new filter.
// Only the measurement mean is remapped. The existing full-pose filter update
// still uses its original covariance, including along the weak directions.
struct SolutionRemappingBaselineResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool valid = false;
  bool lidar_measurement_available = false;
  std::string status = "NOT_COMPUTED";
  int weak_dimension = 0;
  int reliable_dimension = 0;
  double translation_length_scale_m = 0.0;
  reliability::Matrix6d projector = reliability::Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> raw_correction = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> safe_correction = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> removed_correction = Eigen::Matrix<double, 6, 1>::Zero();
  Pose3d remapped_map_T_lidar;
  double projector_symmetry_error = std::numeric_limits<double>::quiet_NaN();
  double projector_idempotence_error = std::numeric_limits<double>::quiet_NaN();
};

// Coordinates: [map-spatial rotation, map translation / L]. L is taken only
// from the current U_obs; the reliable basis is the unchanged P7-C classifier.
SolutionRemappingBaselineResult remapNdtSolutionBaseline(
    const Pose3d& predicted_map_T_lidar,
    const Pose3d& raw_map_T_lidar,
    const reliability::LocalObservability& local,
    const reliability::FixedPhysicalJointSubspace& subspace);

}  // namespace dog_prior_map_fastlio2_frontend_exp
