#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"

namespace dog_prior_map_fastlio2_frontend_exp {
struct DirectionalCovarianceResult {
  bool valid = false;
  std::string status = "NOT_RUN";
  CoupledMatrix6 jacobian = CoupledMatrix6::Zero();
  CoupledMatrix6 baseline = CoupledMatrix6::Zero();
  CoupledMatrix6 chart_baseline = CoupledMatrix6::Zero();
  CoupledMatrix6 covariance = CoupledMatrix6::Zero();
  Eigen::VectorXd multipliers, weak_variances;
  double epsilon = 0, minimum_eigenvalue = 0, solve_residual = 0;
  double strong_block_difference = 0, total_ms = 0;
};
// Translation-first P9 chart -> actual pose residual at the predicted state.
// R is frozen for one existing iterated measurement update (no filter changes).
DirectionalCovarianceResult buildDirectionalCovariance(
    const CoupledVector6& curvature, const CoupledMatrix6& basis, int weak_dimension,
    const CoupledMatrix6& baseline, const Pose3d& nominal_map_T_lidar, const Pose3d& measured_map_T_lidar,
    const Pose3d& predicted_map_T_imu, const Pose3d& imu_T_lidar);
}  // namespace dog_prior_map_fastlio2_frontend_exp
