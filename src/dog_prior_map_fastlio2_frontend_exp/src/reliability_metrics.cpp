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

bool pseudoInversePSD3(const Eigen::Matrix3d& input,
                       Eigen::Matrix3d* inverse,
                       int* effective_rank) {
  if (!inverse || !effective_rank || !input.allFinite()) return false;
  const Eigen::Matrix3d symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite())
    return false;
  const double scale = std::max(1.0, solver.eigenvalues().cwiseAbs().maxCoeff());
  const double negative_tolerance = 1e-10 * scale;
  if (solver.eigenvalues().minCoeff() < -negative_tolerance) return false;
  const double maximum_positive =
      std::max(0.0, solver.eigenvalues().maxCoeff());
  const double rank_tolerance = std::max(1e-12, 1e-9 * maximum_positive);
  Eigen::Vector3d reciprocal = Eigen::Vector3d::Zero();
  *effective_rank = 0;
  for (int i = 0; i < 3; ++i) {
    const double eigenvalue = std::max(0.0, solver.eigenvalues()(i));
    if (eigenvalue > rank_tolerance) {
      reciprocal(i) = 1.0 / eigenvalue;
      ++*effective_rank;
    }
  }
  *inverse = solver.eigenvectors() * reciprocal.asDiagonal() *
             solver.eigenvectors().transpose();
  *inverse = (0.5 * (*inverse + inverse->transpose())).eval();
  return inverse->allFinite();
}

bool schurDecouple(const Matrix6d& information, Matrix3d* rotation,
                   Matrix3d* translation, Eigen::Vector3d* rotation_values,
                   Eigen::Vector3d* translation_values) {
  if (!rotation || !translation || !rotation_values || !translation_values ||
      !information.allFinite())
    return false;
  const Matrix3d a = 0.5 * (information.block<3, 3>(0, 0) +
                            information.block<3, 3>(0, 0).transpose());
  const Matrix3d b = information.block<3, 3>(0, 3);
  const Matrix3d c = 0.5 * (information.block<3, 3>(3, 3) +
                            information.block<3, 3>(3, 3).transpose());
  Matrix3d a_inverse, c_inverse;
  int rank_a = 0;
  int rank_c = 0;
  if (!pseudoInversePSD3(a, &a_inverse, &rank_a) ||
      !pseudoInversePSD3(c, &c_inverse, &rank_c))
    return false;
  (void)rank_a;
  (void)rank_c;
  *rotation = a - b * c_inverse * b.transpose();
  *translation = c - b.transpose() * a_inverse * b;
  *rotation = (0.5 * (*rotation + rotation->transpose())).eval();
  *translation = (0.5 * (*translation + translation->transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix3d> rotation_solver(*rotation);
  Eigen::SelfAdjointEigenSolver<Matrix3d> translation_solver(*translation);
  if (rotation_solver.info() != Eigen::Success ||
      translation_solver.info() != Eigen::Success ||
      !rotation_solver.eigenvalues().allFinite() ||
      !translation_solver.eigenvalues().allFinite())
    return false;
  const double rotation_scale = std::max(
      1.0, rotation_solver.eigenvalues().cwiseAbs().maxCoeff());
  const double translation_scale = std::max(
      1.0, translation_solver.eigenvalues().cwiseAbs().maxCoeff());
  const double rotation_tolerance = 1e-10 * rotation_scale;
  const double translation_tolerance = 1e-10 * translation_scale;
  if (rotation_solver.eigenvalues().minCoeff() < -rotation_tolerance ||
      translation_solver.eigenvalues().minCoeff() < -translation_tolerance)
    return false;
  *rotation_values = rotation_solver.eigenvalues().cwiseMax(0.0);
  *translation_values = translation_solver.eigenvalues().cwiseMax(0.0);
  *rotation = rotation_solver.eigenvectors() *
      rotation_values->asDiagonal() * rotation_solver.eigenvectors().transpose();
  *translation = translation_solver.eigenvectors() *
      translation_values->asDiagonal() * translation_solver.eigenvectors().transpose();
  *rotation = (0.5 * (*rotation + rotation->transpose())).eval();
  *translation = (0.5 * (*translation + translation->transpose())).eval();
  return rotation->allFinite() && translation->allFinite() &&
      rotation_values->allFinite() && translation_values->allFinite();
}

}  // namespace

Eigen::Matrix<double, 3, 6> geometricPointResidualJacobian(
    const Eigen::Vector3d& rotated_source_map) {
  Eigen::Matrix<double, 3, 6> jacobian =
      Eigen::Matrix<double, 3, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  if (!rotated_source_map.allFinite()) return jacobian;
  Eigen::Matrix3d skew;
  skew << 0.0, -rotated_source_map.z(), rotated_source_map.y(),
          rotated_source_map.z(), 0.0, -rotated_source_map.x(),
          -rotated_source_map.y(), rotated_source_map.x(), 0.0;
  jacobian.block<3, 3>(0, 0) = -skew;
  jacobian.block<3, 3>(0, 3).setIdentity();
  return jacobian;
}

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
      (0.5 * (result.hessian_physical + result.hessian_physical.transpose())).eval();

  Matrix6d scale = Matrix6d::Identity();
  scale.block<3, 3>(3, 3) *= translation_scale_m;
  result.normalized_negative_score_curvature = scale.transpose() *
      result.hessian_physical * scale;
  result.normalized_negative_score_curvature = (0.5 *
      (result.normalized_negative_score_curvature +
       result.normalized_negative_score_curvature.transpose())).eval();
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

LocalObservability analyzeGeometricObservability(
    const std::vector<GeometricObservation>& observations,
    bool ndt_converged, double length_scale_m,
    double covariance_eigenvalue_floor_m2,
    double covariance_relative_floor) {
  LocalObservability result;
  result.geometric_proxy = true;
  result.ndt_converged = ndt_converged;
  result.score_gradient_coordinate_check_passed = false;
  result.status = "GEOMETRIC_UNINITIALIZED";
  result.translation_length_scale_m = length_scale_m;
  if (!ndt_converged) {
    result.status = "NDT_NOT_CONVERGED";
    return result;
  }
  if (!std::isfinite(length_scale_m) || length_scale_m <= 0.0 ||
      !std::isfinite(covariance_eigenvalue_floor_m2) ||
      covariance_eigenvalue_floor_m2 <= 0.0 ||
      !std::isfinite(covariance_relative_floor) ||
      covariance_relative_floor <= 0.0 || covariance_relative_floor >= 1.0) {
    result.status = "INVALID_GEOMETRIC_REGULARIZATION";
    return result;
  }

  Matrix6d information = Matrix6d::Zero();
  double weight_sum = 0.0;
  for (const GeometricObservation& observation : observations) {
    if (!observation.rotated_source_map.allFinite() ||
        !observation.residual_map.allFinite() ||
        !std::isfinite(observation.nonnegative_weight) ||
        observation.nonnegative_weight <= 0.0)
      continue;
    if (!observation.voxel_covariance_map.allFinite()) {
      ++result.rejected_covariance_count;
      continue;
    }
    const Eigen::Matrix3d symmetric_covariance = 0.5 *
        (observation.voxel_covariance_map + observation.voxel_covariance_map.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance_solver(
        symmetric_covariance);
    if (covariance_solver.info() != Eigen::Success ||
        !covariance_solver.eigenvalues().allFinite() ||
        !covariance_solver.eigenvectors().allFinite()) {
      ++result.rejected_covariance_count;
      continue;
    }
    const double largest = covariance_solver.eigenvalues().maxCoeff();
    if (!std::isfinite(largest) || largest <= 0.0) {
      ++result.rejected_covariance_count;
      continue;
    }
    const double covariance_psd_tolerance = 1e-10 * largest;
    if (covariance_solver.eigenvalues().minCoeff() <
        -covariance_psd_tolerance) {
      ++result.rejected_covariance_count;
      continue;
    }
    const double eigenvalue_floor = std::max(
        covariance_eigenvalue_floor_m2, covariance_relative_floor * largest);
    const Eigen::Vector3d regularized_eigenvalues =
        covariance_solver.eigenvalues().cwiseMax(eigenvalue_floor);
    const Eigen::Matrix3d inverse_covariance = covariance_solver.eigenvectors() *
        regularized_eigenvalues.cwiseInverse().asDiagonal() *
        covariance_solver.eigenvectors().transpose();
    if (!inverse_covariance.allFinite()) {
      ++result.rejected_covariance_count;
      continue;
    }

    const Eigen::Matrix<double, 3, 6> jacobian =
        geometricPointResidualJacobian(observation.rotated_source_map);
    if (!jacobian.allFinite()) {
      ++result.rejected_covariance_count;
      continue;
    }
    const Matrix6d contribution = observation.nonnegative_weight *
        jacobian.transpose() * inverse_covariance * jacobian;
    if (!contribution.allFinite()) {
      ++result.rejected_covariance_count;
      continue;
    }
    information += contribution;
    weight_sum += observation.nonnegative_weight;
    ++result.valid_correspondence_count;
  }
  result.effective_weight_sum = weight_sum;
  if (result.valid_correspondence_count == 0) {
    result.map_support_sufficient = false;
    result.map_support_status = "NO_VALID_GEOMETRIC_CORRESPONDENCES";
    result.status = "NO_VALID_GEOMETRIC_CORRESPONDENCES";
    return result;
  }
  constexpr std::uint64_t kMinimumCorrespondences = 30;
  if (result.valid_correspondence_count < kMinimumCorrespondences) {
    result.map_support_sufficient = false;
    result.map_support_status = "MAP_SUPPORT_INSUFFICIENT";
    result.status = "MAP_SUPPORT_INSUFFICIENT";
    return result;
  }
  if (!std::isfinite(weight_sum) || weight_sum <= 0.0) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "INVALID_GEOMETRIC_WEIGHT_SUM";
    return result;
  }

  information /= weight_sum;
  information = (0.5 * (information + information.transpose())).eval();
  Matrix6d scale = Matrix6d::Identity();
  scale.block<3, 3>(3, 3) *= length_scale_m;
  Matrix6d normalized_information = scale.transpose() * information * scale;
  normalized_information = (0.5 *
      (normalized_information + normalized_information.transpose())).eval();
  if (!normalized_information.allFinite()) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "NONFINITE_GEOMETRIC_INFORMATION";
    return result;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> full_solver(normalized_information);
  if (full_solver.info() != Eigen::Success ||
      !full_solver.eigenvalues().allFinite() ||
      !full_solver.eigenvectors().allFinite()) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "GEOMETRIC_INFORMATION_EIGENSOLVE_FAILED";
    return result;
  }
  const double spectral_scale = std::max(
      1.0, full_solver.eigenvalues().cwiseAbs().maxCoeff());
  const double psd_tolerance = 1e-10 * spectral_scale;
  if (full_solver.eigenvalues().minCoeff() < -psd_tolerance) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "GEOMETRIC_INFORMATION_NOT_PSD";
    return result;
  }
  // Remove only roundoff-scale negative eigenvalues; information is built as
  // a sum of J'WJ terms and must remain PSD by construction.
  normalized_information = full_solver.eigenvectors() *
      full_solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
      full_solver.eigenvectors().transpose();
  normalized_information = (0.5 *
      (normalized_information + normalized_information.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6d> corrected_solver(normalized_information);
  if (corrected_solver.info() != Eigen::Success ||
      !corrected_solver.eigenvalues().allFinite() ||
      !corrected_solver.eigenvectors().allFinite()) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "CORRECTED_GEOMETRIC_INFORMATION_EIGENSOLVE_FAILED";
    return result;
  }
  result.normalized_geometric_information = normalized_information;
  result.joint_eigenvalues = corrected_solver.eigenvalues().cwiseMax(0.0);
  result.joint_eigenvectors = corrected_solver.eigenvectors();
  result.schur_decoupling_valid = schurDecouple(
      normalized_information, &result.rotation_schur_information,
      &result.translation_schur_information,
      &result.rotation_schur_eigenvalues,
      &result.translation_schur_eigenvalues);
  if (!result.schur_decoupling_valid) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "GEOMETRIC_SCHUR_DECOUPLING_FAILED";
    return result;
  }

  const Eigen::Matrix3d rotation_block = normalized_information.block<3, 3>(0, 0);
  const Eigen::Matrix3d translation_block = normalized_information.block<3, 3>(3, 3);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(rotation_block);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_solver(translation_block);
  if (rotation_solver.info() != Eigen::Success ||
      translation_solver.info() != Eigen::Success ||
      !rotation_solver.eigenvalues().allFinite() ||
      !translation_solver.eigenvalues().allFinite() ||
      !rotation_solver.eigenvectors().allFinite() ||
      !translation_solver.eigenvectors().allFinite()) {
    result.numerical_failure = true;
    result.map_support_status = "NUMERICAL_FAILURE";
    result.status = "GEOMETRIC_BLOCK_EIGENSOLVE_FAILED";
    return result;
  }
  result.rotation_block_eigenvalues = rotation_solver.eigenvalues().cwiseMax(0.0);
  result.rotation_block_eigenvectors = rotation_solver.eigenvectors();
  result.translation_block_eigenvalues = translation_solver.eigenvalues().cwiseMax(0.0);
  result.translation_block_eigenvectors = translation_solver.eigenvectors();
  const double rotation_min = result.rotation_block_eigenvalues.minCoeff();
  const double translation_min = result.translation_block_eigenvalues.minCoeff();
  result.rotation_block_condition = rotation_min > 0.0
      ? result.rotation_block_eigenvalues.maxCoeff() / rotation_min
      : std::numeric_limits<double>::infinity();
  result.translation_block_condition = translation_min > 0.0
      ? result.translation_block_eigenvalues.maxCoeff() / translation_min
      : std::numeric_limits<double>::infinity();
  result.map_support_sufficient = true;
  result.map_support_status = "MAP_SUPPORT_SUFFICIENT";
  result.valid = true;
  result.status = "VALID_GEOMETRIC_GAUSS_NEWTON_PROXY";
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
