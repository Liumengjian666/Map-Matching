#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::reliability {
namespace {

bool validRotation(const Eigen::Matrix3d& rotation) {
  return rotation.allFinite() &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
      std::abs(rotation.determinant() - 1.0) <= 1e-6;
}

Eigen::Vector3d logRotation(const Eigen::Matrix3d& rotation) {
  if (!validRotation(rotation))
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  const Eigen::Vector3d vector = quaternion.vec();
  const double sine_half = vector.norm();
  if (!std::isfinite(sine_half))
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  if (sine_half < 1e-12) return 2.0 * vector;
  const double angle = 2.0 * std::atan2(sine_half,
                                        std::clamp(quaternion.w(), -1.0, 1.0));
  return (angle / sine_half) * vector;
}

bool validConfig(const DualReliabilityConfig& config) {
  return std::isfinite(config.weak_eigenvalue_ratio) &&
      config.weak_eigenvalue_ratio > 0.0 && config.weak_eigenvalue_ratio < 1.0 &&
      std::isfinite(config.gamma_translation) && config.gamma_translation >= 1.0 &&
      std::isfinite(config.gamma_rotation) && config.gamma_rotation >= 1.0 &&
      std::isfinite(config.ndt_translation_scale_m) && config.ndt_translation_scale_m > 0.0 &&
      std::isfinite(config.alpha_translation_response) &&
      config.alpha_translation_response >= 0.0 &&
      std::isfinite(config.alpha_rotation_response) &&
      config.alpha_rotation_response >= 0.0 &&
      std::isfinite(config.max_nonlocal_translation_variance_m2) &&
      config.max_nonlocal_translation_variance_m2 > 0.0 &&
      std::isfinite(config.max_nonlocal_rotation_variance_rad2) &&
      config.max_nonlocal_rotation_variance_rad2 > 0.0 &&
      std::isfinite(config.terminal_translation_limit_m) &&
      config.terminal_translation_limit_m > 0.0 &&
      std::isfinite(config.terminal_rotation_limit_rad) &&
      config.terminal_rotation_limit_rad > 0.0 &&
      std::isfinite(config.innovation_chi_square_99pct_6d) &&
      config.innovation_chi_square_99pct_6d > 0.0 &&
      std::isfinite(config.absolute_translation_fallback_m) &&
      config.absolute_translation_fallback_m > 0.0 &&
      std::isfinite(config.absolute_rotation_fallback_rad) &&
      config.absolute_rotation_fallback_rad > 0.0 &&
      config.periodic_probe_interval_scans > 0 &&
      std::isfinite(config.probe_prior_sigma) && config.probe_prior_sigma > 0.0 &&
      config.minimum_visual_inliers >= 6 &&
      std::isfinite(config.minimum_visual_inlier_ratio) &&
      config.minimum_visual_inlier_ratio > 0.0 && config.minimum_visual_inlier_ratio <= 1.0 &&
      std::isfinite(config.minimum_visual_grid_occupancy) &&
      config.minimum_visual_grid_occupancy > 0.0 && config.minimum_visual_grid_occupancy <= 1.0 &&
      std::isfinite(config.minimum_visual_hull_fraction) &&
      config.minimum_visual_hull_fraction > 0.0 && config.minimum_visual_hull_fraction <= 1.0 &&
      std::isfinite(config.minimum_visual_depth_fraction) &&
      config.minimum_visual_depth_fraction > 0.0 && config.minimum_visual_depth_fraction <= 1.0 &&
      std::isfinite(config.minimum_visual_parallax_px) &&
      config.minimum_visual_parallax_px > 0.0 &&
      std::isfinite(config.maximum_visual_reprojection_rmse_px) &&
      config.maximum_visual_reprojection_rmse_px > 0.0 &&
      std::isfinite(config.maximum_visual_pair_gap_s) &&
      config.maximum_visual_pair_gap_s > 0.0 &&
      std::isfinite(config.maximum_visual_depth_age_s) &&
      config.maximum_visual_depth_age_s >= 0.0 &&
      std::isfinite(config.visual_innovation_chi_square_99pct_3d) &&
      config.visual_innovation_chi_square_99pct_3d > 0.0 &&
      std::isfinite(config.minimum_visual_information_eigenvalue) &&
      config.minimum_visual_information_eigenvalue > 0.0 &&
      std::isfinite(config.minimum_visual_information_ratio) &&
      config.minimum_visual_information_ratio > 0.0 &&
      config.minimum_visual_information_ratio < 1.0 &&
      std::isfinite(config.maximum_visual_information_condition) &&
      config.maximum_visual_information_condition > 1.0 &&
      std::isfinite(config.relocalization_coast_limit_s) &&
      config.relocalization_coast_limit_s > 0.0 &&
      std::isfinite(config.relocalization_position_sigma_limit_m) &&
      config.relocalization_position_sigma_limit_m > 0.0 &&
      std::isfinite(config.relocalization_rotation_sigma_limit_rad) &&
      config.relocalization_rotation_sigma_limit_rad > 0.0;
}

bool finitePose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite() && validRotation(pose.linear());
}

bool cappedResponseCovariance(const Eigen::Matrix3d& raw,
                              double alpha, double maximum_eigenvalue,
                              Eigen::Matrix3d* output) {
  if (!output || !raw.allFinite() || !std::isfinite(alpha) || alpha < 0.0 ||
      !std::isfinite(maximum_eigenvalue) || maximum_eigenvalue <= 0.0)
    return false;
  const Eigen::Matrix3d symmetric = 0.5 * (raw + raw.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite())
    return false;
  const double scale = std::max(1.0, solver.eigenvalues().cwiseAbs().maxCoeff());
  const double tolerance = 1e-10 * scale;
  if (solver.eigenvalues().minCoeff() < -tolerance) return false;
  Eigen::Vector3d bounded = solver.eigenvalues().cwiseMax(0.0) * alpha;
  for (int index = 0; index < 3; ++index)
    bounded(index) = std::min(bounded(index), maximum_eigenvalue);
  *output = solver.eigenvectors() * bounded.asDiagonal() *
            solver.eigenvectors().transpose();
  *output = 0.5 * (*output + output->transpose());
  return output->allFinite();
}

}  // namespace

Vector6d mapProductInnovation(const Eigen::Isometry3d& predicted,
                              const Eigen::Isometry3d& measured) {
  Vector6d result = Vector6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  if (!finitePose(predicted) || !finitePose(measured)) return result;
  result.head<3>() = logRotation(measured.linear() * predicted.linear().transpose());
  result.tail<3>() = measured.translation() - predicted.translation();
  return result;
}

InnovationStatistic normalizedInnovation(
    const Vector6d& innovation, const Matrix6d& input_covariance) {
  InnovationStatistic result;
  if (!innovation.allFinite() || !input_covariance.allFinite()) {
    result.status = "NONFINITE_INPUT";
    return result;
  }
  const Matrix6d covariance = 0.5 * (input_covariance + input_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    result.status = "COVARIANCE_EIGENSOLVE_FAILED";
    return result;
  }
  const double largest = solver.eigenvalues().maxCoeff();
  if (!std::isfinite(largest) || largest <= 0.0) {
    result.status = "COVARIANCE_NOT_POSITIVE";
    return result;
  }
  const double tolerance = 1e-10 * largest;
  if (solver.eigenvalues().minCoeff() < -tolerance) {
    result.status = "COVARIANCE_INDEFINITE";
    return result;
  }
  double squared = 0.0;
  double unsupported_squared = 0.0;
  const Vector6d coordinates = solver.eigenvectors().transpose() * innovation;
  for (int index = 0; index < 6; ++index) {
    const double eigenvalue = solver.eigenvalues()(index);
    if (eigenvalue > tolerance) {
      squared += coordinates(index) * coordinates(index) / eigenvalue;
    } else {
      unsupported_squared += coordinates(index) * coordinates(index);
    }
  }
  if (unsupported_squared > 1e-12) {
    result.valid = true;
    result.mahalanobis_squared = std::numeric_limits<double>::infinity();
    result.status = "INNOVATION_OUTSIDE_COVARIANCE_SUPPORT";
    return result;
  }
  if (!std::isfinite(squared) || squared < 0.0) {
    result.status = "INVALID_MAHALANOBIS_VALUE";
    return result;
  }
  result.valid = true;
  result.mahalanobis_squared = squared;
  result.status = "VALID";
  return result;
}

ProbeTrigger shouldRunNonlocalProbes(
    std::uint64_t transaction_id, const Vector6d& innovation,
    const Matrix6d& covariance, const DualReliabilityConfig& config) {
  ProbeTrigger result;
  if (!validConfig(config)) {
    result.reason = "INVALID_CONFIGURATION";
    return result;
  }
  result.innovation = normalizedInnovation(innovation, covariance);
  if (!innovation.allFinite()) {
    result.run_probes = true;
    result.reason = "NONFINITE_INNOVATION";
    return result;
  }
  if (result.innovation.valid &&
      result.innovation.mahalanobis_squared > config.innovation_chi_square_99pct_6d) {
    result.run_probes = true;
    result.reason = "HIGH_NORMALIZED_INNOVATION";
    return result;
  }
  if (!result.innovation.valid) {
    if (innovation.tail<3>().norm() > config.absolute_translation_fallback_m ||
        innovation.head<3>().norm() > config.absolute_rotation_fallback_rad) {
      result.run_probes = true;
      result.reason = "ABSOLUTE_INNOVATION_FALLBACK";
      return result;
    }
  }
  if (transaction_id % config.periodic_probe_interval_scans == 0) {
    result.run_probes = true;
    result.reason = "PERIODIC_CHECK";
  } else {
    result.reason = result.innovation.valid ? "BELOW_TRIGGER" :
                                             "INVALID_STATISTIC_NO_ABSOLUTE_TRIGGER";
  }
  return result;
}

LocalRisk assessLocalRisk(const LocalObservability& local,
                          const DualReliabilityConfig& config) {
  LocalRisk result;
  result.status = local.status;
  result.map_support_sufficient = local.map_support_sufficient;
  result.map_support_correspondences = local.valid_correspondence_count;
  result.map_support_effective_weight = local.effective_weight_sum;
  result.translation_length_scale_m = local.translation_length_scale_m > 0.0
      ? local.translation_length_scale_m : config.ndt_translation_scale_m;
  result.schur_decoupling_valid = local.schur_decoupling_valid;
  result.rotation_schur_eigenvalues = local.rotation_schur_eigenvalues;
  result.translation_schur_eigenvalues = local.translation_schur_eigenvalues;
  if (!validConfig(config) || !local.valid ||
      !local.rotation_block_eigenvalues.allFinite() ||
      !local.translation_block_eigenvalues.allFinite() ||
      !local.rotation_block_eigenvectors.allFinite() ||
      !local.translation_block_eigenvectors.allFinite())
    return result;
  Eigen::Vector3d rotation_values = local.rotation_schur_eigenvalues;
  Eigen::Vector3d translation_values = local.translation_schur_eigenvalues;
  if (!local.schur_decoupling_valid || !rotation_values.allFinite() ||
      !translation_values.allFinite()) {
    rotation_values = local.rotation_block_eigenvalues;
    translation_values = local.translation_block_eigenvalues;
  }
  const double rotation_max = rotation_values.maxCoeff();
  const double translation_max = translation_values.maxCoeff();
  const double rotation_min = rotation_values.minCoeff();
  const double translation_min = translation_values.minCoeff();
  const double rotation_tolerance = 1e-12 * std::max(1.0, rotation_max);
  const double translation_tolerance = 1e-12 * std::max(1.0, translation_max);
  if ((!local.geometric_proxy && (rotation_min <= 0.0 || translation_min <= 0.0)) ||
      (local.geometric_proxy &&
       (rotation_min < -rotation_tolerance || translation_min < -translation_tolerance)) ||
      rotation_max <= 0.0 || translation_max <= 0.0) {
    result.status = local.geometric_proxy ? "INVALID_GEOMETRIC_INFORMATION_BLOCK" :
                                          "NONPOSITIVE_BLOCK_CURVATURE";
    return result;
  }
  result.rotation_weak_ratio = rotation_min / rotation_max;
  result.translation_weak_ratio = translation_min / translation_max;
  result.rotation_min_eigenvalue = rotation_min;
  result.translation_min_eigenvalue = translation_min;
  result.rotation_block_condition = rotation_min > 0.0
      ? rotation_max / rotation_min : std::numeric_limits<double>::infinity();
  result.translation_block_condition = translation_min > 0.0
      ? translation_max / translation_min : std::numeric_limits<double>::infinity();
  result.rotation_weak = result.rotation_weak_ratio <= config.weak_eigenvalue_ratio;
  result.translation_weak =
      result.translation_weak_ratio <= config.weak_eigenvalue_ratio;
  const Eigen::Matrix3d rotation_information = local.schur_decoupling_valid
      ? local.rotation_schur_information
      : local.rotation_block_eigenvectors *
            local.rotation_block_eigenvalues.asDiagonal() *
            local.rotation_block_eigenvectors.transpose();
  const Eigen::Matrix3d translation_information = local.schur_decoupling_valid
      ? local.translation_schur_information
      : local.translation_block_eigenvectors *
            local.translation_block_eigenvalues.asDiagonal() *
            local.translation_block_eigenvectors.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(
      rotation_information);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> translation_solver(
      translation_information);
  const Eigen::Matrix3d rotation_vectors = rotation_solver.info() == Eigen::Success
      ? rotation_solver.eigenvectors() : local.rotation_block_eigenvectors;
  const Eigen::Matrix3d translation_vectors = translation_solver.info() == Eigen::Success
      ? translation_solver.eigenvectors() : local.translation_block_eigenvectors;
  result.map_rotation_weak_direction = rotation_vectors.col(0).normalized();
  result.map_translation_weak_direction = translation_vectors.col(0).normalized();

  if (local.normalized_geometric_information.allFinite()) {
    const Matrix6d information = 0.5 * (local.normalized_geometric_information +
                                       local.normalized_geometric_information.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix6d> joint_solver(information);
    if (joint_solver.info() != Eigen::Success ||
        !joint_solver.eigenvalues().allFinite() ||
        !joint_solver.eigenvectors().allFinite()) {
      result.status = "JOINT_INFORMATION_EIGENSOLVE_FAILED";
      return result;
    }
    const double joint_max = joint_solver.eigenvalues().maxCoeff();
    if (!std::isfinite(joint_max) || joint_max < -1e-10) {
      result.status = "INVALID_JOINT_INFORMATION";
      return result;
    }
    if (joint_max <= 1e-12) {
      result.joint_weak_basis.setIdentity();
      result.weak_dimension = 6;
      result.reliable_dimension = 0;
      result.translation_weak = true;
      result.rotation_weak = true;
      result.translation_weak_ratio = 0.0;
      result.rotation_weak_ratio = 0.0;
    } else {
      for (int index = 0; index < 6; ++index) {
        const double ratio = std::max(0.0, joint_solver.eigenvalues()(index)) /
                             joint_max;
        if (ratio <= config.weak_eigenvalue_ratio) {
          result.joint_weak_basis.col(result.weak_dimension++) =
              joint_solver.eigenvectors().col(index);
        } else {
          result.joint_reliable_basis.col(result.reliable_dimension++) =
              joint_solver.eigenvectors().col(index);
        }
      }
    }
  } else {
    // Compatibility for older unit fixtures that provide only the decoupled
    // blocks. Live geometric U_obs always supplies the full joint information.
    if (result.rotation_weak)
      result.joint_weak_basis.block<3, 1>(0, result.weak_dimension++) =
          rotation_vectors.col(0);
    else
      result.joint_reliable_basis.block<3, 1>(0, result.reliable_dimension++) =
          rotation_vectors.col(0);
    if (result.translation_weak)
      result.joint_weak_basis.block<3, 1>(3, result.weak_dimension++) =
          translation_vectors.col(0);
    else
      result.joint_reliable_basis.block<3, 1>(3, result.reliable_dimension++) =
          translation_vectors.col(0);
  }
  if (!result.map_rotation_weak_direction.allFinite() ||
      !result.map_translation_weak_direction.allFinite()) {
    result.status = "INVALID_WEAK_DIRECTION";
    result.translation_weak = false;
    result.rotation_weak = false;
    return result;
  }
  result.valid = true;
  result.status = result.map_support_sufficient
      ? "VALID_JOINT_WEAK_DIRECTIONS_SCHUR_DECOUPLED"
      : "MAP_SUPPORT_INSUFFICIENT";
  return result;
}

MeasurementNoiseResult makePoseMeasurementNoise(
    double position_sigma_m, double rotation_sigma_rad,
    const Eigen::Matrix3d& predicted_map_R_imu, const LocalRisk& local_risk,
    const NonlocalTerminalStability& nonlocal, bool use_uobs,
    bool use_unonlocal,
    const DualReliabilityConfig& config) {
  MeasurementNoiseResult result;
  if (!validConfig(config) || !std::isfinite(position_sigma_m) ||
      !std::isfinite(rotation_sigma_rad) || position_sigma_m <= 0.0 ||
      rotation_sigma_rad <= 0.0 || !validRotation(predicted_map_R_imu)) {
    result.status = "INVALID_NOISE_INPUT";
    return result;
  }
  const double position_variance = position_sigma_m * position_sigma_m;
  const double rotation_variance = rotation_sigma_rad * rotation_sigma_rad;
  if (!std::isfinite(position_variance) || !std::isfinite(rotation_variance)) {
    result.status = "NONFINITE_BASE_VARIANCE";
    return result;
  }
  result.covariance.setZero();
  if (use_uobs && local_risk.valid) {
    const auto localIncrement = [&](bool weak, double ratio, double sigma,
                                    double gamma, const Eigen::Vector3d& direction,
                                    Eigen::Matrix3d* increment) {
      if (!weak) return true;
      if (!std::isfinite(ratio) || ratio < 0.0 ||
          !direction.allFinite() || std::abs(direction.norm() - 1.0) > 1e-8)
        return false;
      const double strength = std::clamp(
          1.0 - ratio / config.weak_eigenvalue_ratio, 0.0, 1.0);
      const double lambda = sigma * sigma * (gamma - 1.0) * strength;
      *increment = lambda * (direction * direction.transpose());
      return increment->allFinite();
    };
    if (!localIncrement(local_risk.translation_weak,
                        local_risk.translation_weak_ratio, position_sigma_m,
                        config.gamma_translation,
                        local_risk.map_translation_weak_direction,
                        &result.local_translation_increment) ||
        !localIncrement(local_risk.rotation_weak,
                        local_risk.rotation_weak_ratio, rotation_sigma_rad,
                        config.gamma_rotation,
                        local_risk.map_rotation_weak_direction,
                        &result.local_rotation_map_increment)) {
      result.status = "INVALID_LOCAL_WEAK_DIRECTION_OR_STRENGTH";
      return result;
    }
  }

  if (use_unonlocal && nonlocal.status != "NOT_PROBED") {
    if (nonlocal.status != "RECORDED_NO_BASIN_CLASSIFICATION" ||
        !nonlocal.response_valid || nonlocal.extra_ndt_calls != 2) {
      result.status = "PROBE_INVALID:" + nonlocal.status;
      return result;
    }
    const Eigen::Matrix3d raw_position = 0.5 *
        (nonlocal.delta_position_imu_positive *
             nonlocal.delta_position_imu_positive.transpose() +
         nonlocal.delta_position_imu_negative *
             nonlocal.delta_position_imu_negative.transpose());
    const Eigen::Matrix3d raw_rotation = 0.5 *
        (nonlocal.delta_rotation_positive * nonlocal.delta_rotation_positive.transpose() +
         nonlocal.delta_rotation_negative * nonlocal.delta_rotation_negative.transpose());
    if (!cappedResponseCovariance(raw_position,
            config.alpha_translation_response,
            config.max_nonlocal_translation_variance_m2,
            &result.nonlocal_translation_increment) ||
        !cappedResponseCovariance(raw_rotation,
            config.alpha_rotation_response,
            config.max_nonlocal_rotation_variance_rad2,
            &result.nonlocal_rotation_map_increment)) {
      result.status = "INVALID_NONLOCAL_RESPONSE_COVARIANCE";
      return result;
    }
  }

  Eigen::Matrix3d position_covariance = position_variance *
      Eigen::Matrix3d::Identity() + result.local_translation_increment +
      result.nonlocal_translation_increment;
  result.rotation_map_covariance = rotation_variance *
      Eigen::Matrix3d::Identity() + result.local_rotation_map_increment +
      result.nonlocal_rotation_map_increment;
  // The pinned MTK SO3 innovation is right/body. Rotate the complete map-frame
  // covariance (including non-axis-aligned response terms) into that basis.
  const Eigen::Matrix3d rotation_body_covariance =
      predicted_map_R_imu.transpose() * result.rotation_map_covariance *
      predicted_map_R_imu;
  result.covariance.block<3, 3>(0, 0) = position_covariance;
  result.covariance.block<3, 3>(3, 3) = rotation_body_covariance;
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(result.covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      solver.eigenvalues().minCoeff() <= 0.0) {
    result.status = "MEASUREMENT_NOISE_NOT_SPD";
    return result;
  }
  result.valid = true;
  result.status = "PASS_SPD_POSITION_ROTATION_ORDER";
  return result;
}

DualReliabilityDecision decideDualReliability(
    const LocalRisk& local, const NonlocalTerminalStability& nonlocal,
    bool nominal_ndt_converged, const Eigen::Matrix3d& predicted_map_R_imu,
    double position_sigma_m, double rotation_sigma_rad,
    bool use_uobs, bool use_unonlocal, const DualReliabilityConfig& config) {
  DualReliabilityDecision result;
  if (!nominal_ndt_converged) {
    result.action = UpdateAction::PREDICTION_ONLY;
    result.reason = "NOMINAL_NDT_NOT_CONVERGED";
    return result;
  }
  const bool local_available = use_uobs && local.valid;
  result.local_risk = local_available &&
      (local.translation_weak || local.rotation_weak);
  const bool probe_requested = use_unonlocal && nonlocal.status != "NOT_PROBED";
  if (probe_requested &&
      (nonlocal.status != "RECORDED_NO_BASIN_CLASSIFICATION" ||
       !nonlocal.geometry_valid || !nonlocal.objectives_finite ||
       !nonlocal.all_converged || !nonlocal.response_valid ||
       nonlocal.extra_ndt_calls != 2)) {
    result.action = UpdateAction::PREDICTION_ONLY;
    result.nonlocal_risk = true;
    result.reason = "PROBE_INVALID:" + nonlocal.status;
    return result;
  }
  result.nonlocal_response_used = probe_requested;
  result.local_curvature_used = local_available;
  result.nonlocal_risk = probe_requested &&
      (nonlocal.max_nominal_translation_delta_m >
           config.terminal_translation_limit_m ||
       nonlocal.max_nominal_rotation_delta_rad >
           config.terminal_rotation_limit_rad);

  if (result.local_risk && result.nonlocal_risk) {
    result.action = UpdateAction::CAUTIOUS_UPDATE;
    result.reason = "LOCAL_AND_NONLOCAL_DIRECTIONAL_RESPONSE";
  } else if (result.nonlocal_risk) {
    result.action = UpdateAction::CAUTIOUS_UPDATE;
    result.reason = "NONLOCAL_DIRECTIONAL_RESPONSE";
  } else if (result.local_risk) {
    result.action = UpdateAction::DIRECTIONAL_UPDATE;
    result.reason = "LOCAL_WEAK_DIRECTION";
  } else if (result.nonlocal_response_used) {
    result.action = UpdateAction::DIRECTIONAL_UPDATE;
    result.reason = "FINITE_PROBE_RESPONSE_COVARIANCE";
  } else {
    result.action = UpdateAction::NORMAL_UPDATE;
    result.reason = local_available ? "NO_RELIABILITY_RISK" :
        (use_unonlocal ? "UOBS_UNAVAILABLE_NONLOCAL_NOT_PROBED" :
                         "UOBS_UNAVAILABLE_UNONLOCAL_DISABLED");
  }
  result.measurement_noise = makePoseMeasurementNoise(
      position_sigma_m, rotation_sigma_rad, predicted_map_R_imu,
      local, nonlocal, local_available, use_unonlocal, config);
  if (!result.measurement_noise.valid) {
    result.action = UpdateAction::PREDICTION_ONLY;
    result.reason = "MEASUREMENT_NOISE_INVALID:" + result.measurement_noise.status;
  }
  return result;
}

const char* toString(UpdateAction action) {
  switch (action) {
    case UpdateAction::NORMAL_UPDATE: return "NORMAL_UPDATE";
    case UpdateAction::DIRECTIONAL_UPDATE: return "DIRECTIONAL_UPDATE";
    case UpdateAction::CAUTIOUS_UPDATE: return "CAUTIOUS_UPDATE";
    case UpdateAction::PREDICTION_ONLY: return "PREDICTION_ONLY";
  }
  return "UNKNOWN";
}

VisualQualityDecision assessVisualQuality(
    const VisualQualityObservation& observation,
    const DualReliabilityConfig& config) {
  VisualQualityDecision result;
  if (!validConfig(config)) {
    result.rejection_reason = "INVALID_QUALITY_CONFIGURATION";
    return result;
  }
  if (!observation.source_valid) {
    result.rejection_reason = "SOURCE_VISUAL_FACTOR_INVALID";
    return result;
  }
  if (!observation.quality_metadata_available) {
    result.rejection_reason = "QUALITY_METADATA_UNAVAILABLE";
    return result;
  }
  if (observation.reference_stamp_ns == 0 ||
      observation.current_stamp_ns <= observation.reference_stamp_ns ||
      observation.depth_stamp_ns == 0 ||
      observation.depth_stamp_ns > observation.reference_stamp_ns) {
    result.rejection_reason = "INVALID_SENSOR_TIMESTAMP_ORDER";
    return result;
  }
  const double pair_gap_s = static_cast<double>(
      observation.current_stamp_ns - observation.reference_stamp_ns) * 1e-9;
  const double depth_age_s = static_cast<double>(
      observation.reference_stamp_ns - observation.depth_stamp_ns) * 1e-9;
  result.timestamp_valid = std::isfinite(pair_gap_s) && std::isfinite(depth_age_s) &&
      pair_gap_s <= config.maximum_visual_pair_gap_s &&
      depth_age_s <= config.maximum_visual_depth_age_s;
  if (!result.timestamp_valid) {
    result.rejection_reason = "STALE_OR_EXCESSIVE_VISUAL_TIME_GAP";
    return result;
  }
  if (observation.pnp_inlier_count < config.minimum_visual_inliers ||
      observation.tracked_count < observation.pnp_inlier_count ||
      !std::isfinite(observation.inlier_ratio) ||
      observation.inlier_ratio < config.minimum_visual_inlier_ratio) {
    result.rejection_reason = "INSUFFICIENT_TRACKED_PNP_INLIERS";
    return result;
  }
  if (!std::isfinite(observation.grid_occupancy) ||
      observation.grid_occupancy < config.minimum_visual_grid_occupancy ||
      !std::isfinite(observation.hull_fraction) ||
      observation.hull_fraction < config.minimum_visual_hull_fraction) {
    result.rejection_reason = "POOR_FEATURE_SPATIAL_DISTRIBUTION";
    return result;
  }
  if (observation.depth_associated_count < observation.pnp_inlier_count ||
      !std::isfinite(observation.depth_fraction) ||
      observation.depth_fraction < config.minimum_visual_depth_fraction) {
    result.rejection_reason = "INSUFFICIENT_VALID_DEPTH_ASSOCIATION";
    return result;
  }
  if (!std::isfinite(observation.median_parallax_px) ||
      observation.median_parallax_px < config.minimum_visual_parallax_px) {
    result.rejection_reason = "INSUFFICIENT_IMAGE_PARALLAX";
    return result;
  }
  if (!std::isfinite(observation.reprojection_rmse_px) ||
      observation.reprojection_rmse_px <= 0.0 ||
      observation.reprojection_rmse_px >
          config.maximum_visual_reprojection_rmse_px) {
    result.rejection_reason = "REPROJECTION_ERROR_OUT_OF_RANGE";
    return result;
  }
  result.prediction_consistent =
      std::isfinite(observation.innovation_chi_square) &&
      observation.innovation_chi_square >= 0.0 &&
      observation.innovation_chi_square <=
          config.visual_innovation_chi_square_99pct_3d;
  if (!result.prediction_consistent) {
    result.rejection_reason = "VISUAL_PREDICTION_INCONSISTENT";
    return result;
  }
  result.passed = true;
  result.rejection_reason = "QUALITY_PASS";
  return result;
}

VisualSubspaceDecision assessVisualWeakSubspaceInformation(
    const LocalRisk& lidar, const Eigen::Vector3d& map_imu_origin_minus_lidar,
    const Eigen::Matrix3d& measurement_covariance,
    const DualReliabilityConfig& config) {
  VisualSubspaceDecision result;
  result.lidar_weak_dimension = lidar.weak_dimension;
  if (!validConfig(config) || !lidar.valid || lidar.weak_dimension <= 0 ||
      lidar.weak_dimension > 6 || !lidar.joint_weak_basis.allFinite() ||
      !map_imu_origin_minus_lidar.allFinite() ||
      !std::isfinite(lidar.translation_length_scale_m) ||
      lidar.translation_length_scale_m <= 0.0 ||
      !measurement_covariance.allFinite()) {
    result.status = lidar.weak_dimension == 0
        ? "NO_LIDAR_WEAK_DIRECTION" : "INVALID_COMPLEMENTARITY_INPUT";
    return result;
  }
  const Eigen::Matrix3d symmetric_covariance =
      0.5 * (measurement_covariance + measurement_covariance.transpose());
  Eigen::LLT<Eigen::Matrix3d> covariance_factor(symmetric_covariance);
  if (covariance_factor.info() != Eigen::Success ||
      covariance_factor.matrixL().toDenseMatrix().diagonal().minCoeff() <= 0.0) {
    result.status = "VISUAL_MEASUREMENT_COVARIANCE_NOT_SPD";
    return result;
  }

  Eigen::Matrix3d skew;
  skew << 0.0, -map_imu_origin_minus_lidar.z(), map_imu_origin_minus_lidar.y(),
          map_imu_origin_minus_lidar.z(), 0.0, -map_imu_origin_minus_lidar.x(),
          -map_imu_origin_minus_lidar.y(), map_imu_origin_minus_lidar.x(), 0.0;
  Eigen::Matrix<double, 3, 6> visual_jacobian;
  // map_T_imu position = map_T_lidar position + rotated lever arm. Under the
  // same map-spatial LiDAR rotation/additive-origin translation chart used by
  // U_obs, d(position_imu)=[-r]x*dphi + d(position_lidar).
  visual_jacobian.block<3, 3>(0, 0) = -skew;
  visual_jacobian.block<3, 3>(0, 3).setIdentity();
  Matrix6d physical_from_normalized = Matrix6d::Identity();
  physical_from_normalized.block<3, 3>(3, 3) *= lidar.translation_length_scale_m;
  const Eigen::Matrix<double, 3, 6> normalized_jacobian =
      visual_jacobian * physical_from_normalized;
  const Eigen::Matrix3d inverse_covariance = covariance_factor.solve(
      Eigen::Matrix3d::Identity());
  result.visual_information = normalized_jacobian.transpose() *
      inverse_covariance * normalized_jacobian;
  result.visual_information = 0.5 *
      (result.visual_information + result.visual_information.transpose());
  if (!result.visual_information.allFinite()) {
    result.status = "NONFINITE_VISUAL_INFORMATION";
    return result;
  }
  const Eigen::MatrixXd weak_basis = lidar.joint_weak_basis.leftCols(
      lidar.weak_dimension);
  const Eigen::MatrixXd projected = weak_basis.transpose() *
      result.visual_information * weak_basis;
  result.projected_weak_information.setZero();
  result.projected_weak_information.topLeftCorner(lidar.weak_dimension,
      lidar.weak_dimension) = 0.5 * (projected + projected.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(projected);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    result.status = "PROJECTED_VISUAL_INFORMATION_EIGENSOLVE_FAILED";
    return result;
  }
  result.valid = true;
  const Eigen::VectorXd values = solver.eigenvalues().cwiseMax(0.0);
  const double maximum = values.maxCoeff();
  if (!std::isfinite(maximum) || maximum <= 0.0) {
    result.status = "NO_INFORMATION_IN_LIDAR_WEAK_SUBSPACE";
    return result;
  }
  const double threshold = std::max(config.minimum_visual_information_eigenvalue,
      maximum * config.minimum_visual_information_ratio);
  const double stable_floor = std::max(threshold,
      maximum / config.maximum_visual_information_condition);
  std::vector<int> selected;
  for (int i = static_cast<int>(values.size()) - 1; i >= 0; --i)
    if (values(i) >= stable_floor) selected.push_back(i);
  if (selected.empty()) {
    result.status = "PROJECTED_VISUAL_INFORMATION_BELOW_STABLE_RANK";
    return result;
  }
  const double minimum_selected = values(selected.back());
  result.effective_condition = maximum / minimum_selected;
  result.effective_rank = static_cast<int>(selected.size());
  Eigen::MatrixXd selected_weak(6, result.effective_rank);
  for (int column = 0; column < result.effective_rank; ++column) {
    const int eigen_index = selected[static_cast<std::size_t>(column)];
    result.projected_eigenvalues(column) = values(eigen_index);
    selected_weak.col(column) = weak_basis * solver.eigenvectors().col(eigen_index);
  }
  result.complementary_weak_basis.leftCols(result.effective_rank) = selected_weak;
  const Eigen::MatrixXd response = normalized_jacobian * selected_weak;
  Eigen::JacobiSVD<Eigen::MatrixXd> response_svd(
      response, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (!response_svd.singularValues().allFinite() ||
      !response_svd.matrixU().allFinite()) {
    result.status = "VISUAL_MEASUREMENT_ROWSPACE_SVD_FAILED";
    return result;
  }
  const Eigen::VectorXd singular = response_svd.singularValues();
  const double singular_max = singular.size() ? singular.maxCoeff() : 0.0;
  const double singular_floor = std::max(1e-10, singular_max * 1e-8);
  for (Eigen::Index i = 0; i < singular.size() && i < 3; ++i)
    if (singular(i) > singular_floor)
      result.measurement_basis.col(result.measurement_rank++) =
          response_svd.matrixU().col(i);
  if (result.measurement_rank == 0) {
    result.status = "VISUAL_WEAK_MODES_HAVE_NO_MEASUREMENT_ROWSPACE";
    result.effective_rank = 0;
    result.complementary_weak_basis.setZero();
    return result;
  }
  result.complementary = true;
  result.status = "VISUAL_COMPLEMENTS_LIDAR_WEAK_SUBSPACE";
  return result;
}

ConditionalRouteDecision routeConditionalCompensation(
    const ConditionalRouteInput& input, const DualReliabilityConfig& config) {
  ConditionalRouteDecision result;
  if (!validConfig(config) || !std::isfinite(input.time_without_external_constraint_s) ||
      !std::isfinite(input.maximum_position_sigma_m) ||
      !std::isfinite(input.maximum_rotation_sigma_rad) ||
      input.time_without_external_constraint_s < 0.0 ||
      input.maximum_position_sigma_m < 0.0 ||
      input.maximum_rotation_sigma_rad < 0.0 || input.lidar_weak_dimension < 0 ||
      input.lidar_reliable_dimension < 0 || input.lidar_weak_dimension > 6 ||
      input.lidar_reliable_dimension > 6 ||
      input.lidar_weak_dimension + input.lidar_reliable_dimension > 6) {
    result.mode = ConditionalFusionMode::RELOCALIZATION_REQUIRED;
    result.relocalization_required = true;
    result.reason = "INVALID_ROUTER_INPUT";
    return result;
  }
  const bool lidar_supported = input.lidar_converged &&
                               input.map_support_sufficient;
  const bool degraded = input.lidar_registration_unstable ||
                        input.lidar_weak_dimension > 0 || !lidar_supported;
  if (lidar_supported && !degraded) {
    result.mode = ConditionalFusionMode::NORMAL_LIDAR;
    result.apply_lidar_measurement = true;
    result.preserve_lidar_reliable_subspace = true;
    result.imu_coasting = false;
    result.reason = "LIDAR_RELIABLE_NO_DEGENERACY";
    return result;
  }
  if (lidar_supported && degraded) {
    result.apply_lidar_measurement = input.lidar_reliable_dimension > 0;
    result.preserve_lidar_reliable_subspace = result.apply_lidar_measurement;
    if (input.vision_trigger_requested && input.vision_observation_available &&
        input.vision_quality_passed && input.vision_complementary &&
        input.vision_complementary_rank > 0) {
      result.mode = ConditionalFusionMode::LIDAR_DEGRADED_VISION_VALID;
      result.apply_visual_measurement = true;
      result.imu_coasting = input.vision_complementary_rank <
                            input.lidar_weak_dimension;
      result.reason = result.imu_coasting
          ? "PARTIAL_VISUAL_COMPLEMENT;REMAINING_WEAK_DIRECTIONS_COAST"
          : "VISUAL_COMPLEMENTS_LIDAR_WEAK_SUBSPACE";
    } else {
      result.mode = ConditionalFusionMode::LIDAR_DEGRADED_VISION_INVALID;
      result.imu_coasting = input.lidar_weak_dimension > 0 ||
                            input.lidar_reliable_dimension == 0;
      if (!input.vision_trigger_requested)
        result.reason = "VISION_NOT_TRIGGERED";
      else if (!input.vision_observation_available)
        result.reason = "NO_VISUAL_OBSERVATION_AVAILABLE";
      else if (!input.vision_quality_passed)
        result.reason = "VISION_QUALITY_REJECTED";
      else
        result.reason = "VISION_HAS_NO_STABLE_INFORMATION_IN_WEAK_SUBSPACE";
    }
  } else {
    result.mode = ConditionalFusionMode::IMU_COASTING;
    result.imu_coasting = true;
    result.reason = !input.lidar_converged ? "LIDAR_REGISTRATION_UNAVAILABLE" :
        (!input.map_support_sufficient ? "MAP_SUPPORT_INSUFFICIENT" :
                                         "NO_EXTERNAL_CONSTRAINT_AVAILABLE");
    if (input.vision_trigger_requested && input.vision_observation_available &&
        (!input.vision_quality_passed || !input.vision_complementary))
      result.reason += ";VISION_REJECTED";
  }
  if (result.imu_coasting &&
      (input.time_without_external_constraint_s >=
           config.relocalization_coast_limit_s ||
       input.maximum_position_sigma_m >=
           config.relocalization_position_sigma_limit_m ||
       input.maximum_rotation_sigma_rad >=
           config.relocalization_rotation_sigma_limit_rad)) {
    result.mode = ConditionalFusionMode::RELOCALIZATION_REQUIRED;
    result.relocalization_required = true;
    result.reason += ";COAST_LIMIT_OR_UNCERTAINTY_EXCEEDED";
  }
  return result;
}

const char* toString(ConditionalFusionMode mode) {
  switch (mode) {
    case ConditionalFusionMode::NORMAL_LIDAR: return "NORMAL_LIDAR";
    case ConditionalFusionMode::LIDAR_DEGRADED_VISION_VALID:
      return "LIDAR_DEGRADED_VISION_VALID";
    case ConditionalFusionMode::LIDAR_DEGRADED_VISION_INVALID:
      return "LIDAR_DEGRADED_VISION_INVALID";
    case ConditionalFusionMode::IMU_COASTING: return "IMU_COASTING";
    case ConditionalFusionMode::RELOCALIZATION_REQUIRED:
      return "RELOCALIZATION_REQUIRED";
  }
  return "UNKNOWN";
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
