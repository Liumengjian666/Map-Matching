#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp::reliability {
namespace {

bool finitePose(const Eigen::Isometry3d& pose) {
  if (!pose.matrix().allFinite()) return false;
  const Eigen::Matrix3d rotation = pose.linear();
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
         std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

double rotationDistance(const Eigen::Matrix3d& lhs,
                        const Eigen::Matrix3d& rhs) {
  const Eigen::Matrix3d relative = lhs.transpose() * rhs;
  const double cosine = std::clamp((relative.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

}  // namespace

LocalObservability analyzeLocalObservability(
    const Eigen::Matrix<double, 6, 6>& curvature, bool ndt_converged,
    bool score_gradient_coordinate_check_passed) {
  LocalObservability result;
  result.ndt_converged = ndt_converged;
  result.score_gradient_coordinate_check_passed = score_gradient_coordinate_check_passed;
  if (!ndt_converged) {
    result.status = "NDT_NOT_CONVERGED";
    return result;
  }
  if (!score_gradient_coordinate_check_passed) {
    result.status = "SCORE_GRADIENT_COORDINATE_CHECK_UNVERIFIED";
    return result;
  }
  if (!curvature.allFinite()) {
    result.status = "NONFINITE_CURVATURE";
    return result;
  }

  const Eigen::Matrix3d rotation_block =
      0.5 * (curvature.block<3, 3>(0, 0) + curvature.block<3, 3>(0, 0).transpose());
  const Eigen::Matrix3d translation_block =
      0.5 * (curvature.block<3, 3>(3, 3) + curvature.block<3, 3>(3, 3).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_eigen(rotation_block);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_eigen(translation_block);
  if (rotation_eigen.info() != Eigen::Success ||
      translation_eigen.info() != Eigen::Success ||
      !rotation_eigen.eigenvalues().allFinite() ||
      !translation_eigen.eigenvalues().allFinite()) {
    result.status = "EIGENSOLVE_FAILED";
    return result;
  }

  result.rotation_block_eigenvalues = rotation_eigen.eigenvalues();
  result.rotation_block_eigenvectors = rotation_eigen.eigenvectors();
  result.translation_block_eigenvalues = translation_eigen.eigenvalues();
  result.translation_block_eigenvectors = translation_eigen.eigenvectors();
  result.valid = true;
  result.status = "VALID_BLOCK_CURVATURE";
  return result;
}

NonlocalTerminalStability analyzeNonlocalTerminalStability(
    const TerminalCapture& nominal, const TerminalCapture& positive,
    const TerminalCapture& negative, std::uint64_t extra_ndt_calls) {
  NonlocalTerminalStability result;
  result.nominal = nominal;
  result.positive = positive;
  result.negative = negative;
  result.extra_ndt_calls = extra_ndt_calls;

  result.geometry_valid = finitePose(nominal.map_T_lidar) &&
                          finitePose(positive.map_T_lidar) &&
                          finitePose(negative.map_T_lidar);
  result.objectives_finite = std::isfinite(nominal.fixed_objective) &&
                             std::isfinite(positive.fixed_objective) &&
                             std::isfinite(negative.fixed_objective);
  if (result.geometry_valid) {
    result.positive_negative_translation_gap_m =
        (positive.map_T_lidar.translation() - negative.map_T_lidar.translation()).norm();
    result.positive_negative_rotation_gap_rad = rotationDistance(
        positive.map_T_lidar.linear(), negative.map_T_lidar.linear());
  }
  if (result.objectives_finite) {
    result.positive_minus_nominal_objective =
        positive.fixed_objective - nominal.fixed_objective;
    result.negative_minus_nominal_objective =
        negative.fixed_objective - nominal.fixed_objective;
    result.positive_minus_negative_objective =
        positive.fixed_objective - negative.fixed_objective;
  }

  if (!result.geometry_valid) result.status = "INVALID_TERMINAL_POSE";
  else if (!result.objectives_finite) result.status = "NONFINITE_FIXED_OBJECTIVE";
  else result.status = "RECORDED_NO_BASIN_CLASSIFICATION";
  return result;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
