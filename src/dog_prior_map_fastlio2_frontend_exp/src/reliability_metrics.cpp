#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/LU>

#include <algorithm>
#include <array>
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

Eigen::Vector3d rotationLogMap(const Eigen::Matrix3d& rotation) {
  if (!rotation.allFinite() ||
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::abs(rotation.determinant() - 1.0) > 1e-6)
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  const double sine_half = quaternion.vec().norm();
  if (!std::isfinite(sine_half))
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  if (sine_half < 1e-12) return 2.0 * quaternion.vec();
  const double angle = 2.0 * std::atan2(sine_half,
                                        std::clamp(quaternion.w(), -1.0, 1.0));
  return (angle / sine_half) * quaternion.vec();
}

}  // namespace

PclCurvatureTransform transformPclScoreHessianToNormalizedMapTangent(
    const Matrix6d& pcl_score_hessian,
    const Eigen::Vector3d& pcl_rx_ry_rz, double translation_scale_m) {
  PclCurvatureTransform result;
  if (!pcl_score_hessian.allFinite() || !pcl_rx_ry_rz.allFinite() ||
      !std::isfinite(translation_scale_m) || translation_scale_m <= 0.0) {
    result.status = "NONFINITE_OR_INVALID_PCL_CURVATURE_INPUT";
    return result;
  }

  const Eigen::AngleAxisd rx(pcl_rx_ry_rz.x(), Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pcl_rx_ry_rz.y(), Eigen::Vector3d::UnitY());
  Matrix3d jacobian;
  jacobian.col(0) = Eigen::Vector3d::UnitX();
  jacobian.col(1) = rx * Eigen::Vector3d::UnitY();
  jacobian.col(2) = rx * ry * Eigen::Vector3d::UnitZ();
  if (!jacobian.allFinite()) {
    result.status = "NONFINITE_EULER_SPATIAL_JACOBIAN";
    return result;
  }
  Eigen::JacobiSVD<Matrix3d> svd(jacobian);
  const Eigen::Vector3d singular = svd.singularValues();
  if (!singular.allFinite() || singular.minCoeff() <= 0.0) {
    result.status = "SINGULAR_EULER_SPATIAL_JACOBIAN";
    return result;
  }
  result.jacobian_condition = singular.maxCoeff() / singular.minCoeff();
  if (!std::isfinite(result.jacobian_condition) ||
      result.jacobian_condition >
          1.0 / std::sqrt(std::numeric_limits<double>::epsilon())) {
    result.status = "ILL_CONDITIONED_EULER_SPATIAL_JACOBIAN";
    return result;
  }
  Eigen::FullPivLU<Matrix3d> decomposition(jacobian);
  if (decomposition.rank() != 3) {
    result.status = "RANK_DEFICIENT_EULER_SPATIAL_JACOBIAN";
    return result;
  }
  const Matrix3d inverse = decomposition.solve(Matrix3d::Identity());
  if (!inverse.allFinite() ||
      (jacobian * inverse - Matrix3d::Identity()).norm() > 1e-10) {
    result.status = "EULER_SPATIAL_JACOBIAN_INVERSION_FAILED";
    return result;
  }
  result.euler_to_map_spatial_jacobian = jacobian;

  constexpr std::array<int, 6> kPclToCanonical{{3, 4, 5, 0, 1, 2}};
  Matrix6d canonical;
  for (int row = 0; row < 6; ++row)
    for (int col = 0; col < 6; ++col)
      canonical(row, col) = pcl_score_hessian(
          kPclToCanonical[static_cast<std::size_t>(row)],
          kPclToCanonical[static_cast<std::size_t>(col)]);
  result.hessian_euler = -0.5 * (canonical + canonical.transpose());

  Matrix6d physical_from_euler = Matrix6d::Identity();
  physical_from_euler.block<3, 3>(0, 0) = inverse;
  result.hessian_physical = physical_from_euler.transpose() *
      result.hessian_euler * physical_from_euler;
  result.hessian_physical =
      0.5 * (result.hessian_physical + result.hessian_physical.transpose());

  Matrix6d scale = Matrix6d::Identity();
  scale.block<3, 3>(3, 3) *= translation_scale_m;
  result.normalized_negative_score_curvature = scale.transpose() *
      result.hessian_physical * scale;
  result.normalized_negative_score_curvature = 0.5 *
      (result.normalized_negative_score_curvature +
       result.normalized_negative_score_curvature.transpose());
  if (!result.hessian_euler.allFinite() || !result.hessian_physical.allFinite() ||
      !result.normalized_negative_score_curvature.allFinite()) {
    result.status = "NONFINITE_TRANSFORMED_CURVATURE";
    return result;
  }
  result.valid = true;
  result.status = "PASS_P6I3_EULER_TO_MAP_TANGENT";
  return result;
}

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
  const double rotation_min = result.rotation_block_eigenvalues.minCoeff();
  const double translation_min = result.translation_block_eigenvalues.minCoeff();
  result.rotation_block_condition = rotation_min > 0.0
      ? result.rotation_block_eigenvalues.maxCoeff() / rotation_min
      : std::numeric_limits<double>::infinity();
  result.translation_block_condition = translation_min > 0.0
      ? result.translation_block_eigenvalues.maxCoeff() / translation_min
      : std::numeric_limits<double>::infinity();
  if (rotation_min <= 0.0 || translation_min <= 0.0) {
    result.status = "NONPOSITIVE_BLOCK_CURVATURE";
    return result;
  }
  result.valid = true;
  result.status = "VALID_BLOCK_CURVATURE";
  return result;
}

NonlocalTerminalStability analyzeNonlocalTerminalStability(
    const TerminalCapture& nominal, const TerminalCapture& positive,
    const TerminalCapture& negative, const Eigen::Isometry3d& imu_T_lidar,
    std::uint64_t extra_ndt_calls) {
  NonlocalTerminalStability result;
  result.nominal = nominal;
  result.positive = positive;
  result.negative = negative;
  result.extra_ndt_calls = extra_ndt_calls;

  result.geometry_valid = finitePose(nominal.map_T_lidar) &&
                          finitePose(positive.map_T_lidar) &&
                          finitePose(negative.map_T_lidar);
  result.imu_lidar_extrinsic_valid = finitePose(imu_T_lidar);
  result.all_converged = nominal.converged && positive.converged &&
                         negative.converged;
  result.objectives_finite = std::isfinite(nominal.fixed_objective) &&
                             std::isfinite(positive.fixed_objective) &&
                             std::isfinite(negative.fixed_objective);
  if (result.geometry_valid && result.imu_lidar_extrinsic_valid) {
    const Eigen::Isometry3d map_T_imu_nominal =
        nominal.map_T_lidar * imu_T_lidar.inverse();
    const Eigen::Isometry3d map_T_imu_positive =
        positive.map_T_lidar * imu_T_lidar.inverse();
    const Eigen::Isometry3d map_T_imu_negative =
        negative.map_T_lidar * imu_T_lidar.inverse();
    result.delta_position_imu_positive = map_T_imu_positive.translation() -
                                         map_T_imu_nominal.translation();
    result.delta_position_imu_negative = map_T_imu_negative.translation() -
                                         map_T_imu_nominal.translation();
    result.delta_rotation_positive = rotationLogMap(
        positive.map_T_lidar.linear() * nominal.map_T_lidar.linear().transpose());
    result.delta_rotation_negative = rotationLogMap(
        negative.map_T_lidar.linear() * nominal.map_T_lidar.linear().transpose());
    result.response_valid = result.all_converged &&
        result.delta_position_imu_positive.allFinite() &&
        result.delta_position_imu_negative.allFinite() &&
        result.delta_rotation_positive.allFinite() &&
        result.delta_rotation_negative.allFinite();
    result.positive_negative_translation_gap_m =
        (positive.map_T_lidar.translation() - negative.map_T_lidar.translation()).norm();
    result.positive_negative_rotation_gap_rad = rotationDistance(
        positive.map_T_lidar.linear(), negative.map_T_lidar.linear());
    result.max_nominal_translation_delta_m = std::max(
        (positive.map_T_lidar.translation() - nominal.map_T_lidar.translation()).norm(),
        (negative.map_T_lidar.translation() - nominal.map_T_lidar.translation()).norm());
    result.max_nominal_rotation_delta_rad = std::max(
        rotationDistance(nominal.map_T_lidar.linear(), positive.map_T_lidar.linear()),
        rotationDistance(nominal.map_T_lidar.linear(), negative.map_T_lidar.linear()));
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
  else if (!result.imu_lidar_extrinsic_valid)
    result.status = "INVALID_IMU_LIDAR_EXTRINSIC";
  else if (!result.objectives_finite) result.status = "NONFINITE_FIXED_OBJECTIVE";
  else if (!result.all_converged) result.status = "NDT_NOT_CONVERGED";
  else if (!result.response_valid) result.status = "NONFINITE_TERMINAL_RESPONSE";
  else if (extra_ndt_calls != 2) result.status = "PROBE_CALL_COUNT_MISMATCH";
  else result.status = "RECORDED_NO_BASIN_CLASSIFICATION";
  return result;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
