#pragma once

#include <string>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dog_prior_map_localization/core/estimator_types.hpp"

namespace dog_prior_map_localization
{

using Matrix6d = Eigen::Matrix<double, 6, 6>;

struct SelectiveRelativeVisualUpdateResult
{
  bool accepted = false;
  int dimension = 0;
  std::string status = "NOT_RUN";
  Eigen::VectorXd innovation;
  Eigen::VectorXd correction;
  double normalized_innovation_squared = 0.0;
  double current_covariance_trace = 0.0;
  double pseudo_covariance_trace = 0.0;
  double minimum_input_state_covariance_eigenvalue =
      std::numeric_limits<double>::quiet_NaN();
  double minimum_conditional_covariance_eigenvalue =
      std::numeric_limits<double>::quiet_NaN();
  double minimum_updated_covariance_eigenvalue =
      std::numeric_limits<double>::quiet_NaN();
  double input_state_covariance_asymmetry =
      std::numeric_limits<double>::quiet_NaN();
};

// Fuse a map-frame EKF pose with a map-frame endpoint estimate constructed
// from a historical reference pose and a camera relative pose. The relative
// covariance is a left-tangent [rho, phi] covariance in the reference IMU
// frame. Only DCReg-degenerate axes are fused. A fixed 0.5 covariance
// intersection weight avoids assuming independence between visual depth and
// the LiDAR/IMU state history; the caller is responsible for exact sensor-time
// alignment and OOSM scheduling.
SelectiveRelativeVisualUpdateResult applySelectiveRelativeVisualUpdate(
    FilterStateSnapshot* current,
    const FilterStateSnapshot& reference,
    const Eigen::Matrix4d& T_imu_ref_imu_cur,
    const Matrix6d& relative_covariance,
    bool translation_degenerate,
    const Eigen::Matrix3d& translation_axes_map,
    bool rotation_degenerate,
    const Eigen::Matrix3d& rotation_axes_map,
    bool covariance_stability_enable = false);

}  // namespace dog_prior_map_localization
