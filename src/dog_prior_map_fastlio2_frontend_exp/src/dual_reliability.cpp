#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>

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
      std::isfinite(config.probe_prior_sigma) && config.probe_prior_sigma > 0.0;
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
  if (!validConfig(config) || !local.valid ||
      !local.rotation_block_eigenvalues.allFinite() ||
      !local.translation_block_eigenvalues.allFinite() ||
      !local.rotation_block_eigenvectors.allFinite() ||
      !local.translation_block_eigenvectors.allFinite())
    return result;
  const double rotation_max = local.rotation_block_eigenvalues.maxCoeff();
  const double translation_max = local.translation_block_eigenvalues.maxCoeff();
  const double rotation_min = local.rotation_block_eigenvalues.minCoeff();
  const double translation_min = local.translation_block_eigenvalues.minCoeff();
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
  result.rotation_block_condition = local.rotation_block_condition;
  result.translation_block_condition = local.translation_block_condition;
  result.rotation_weak = result.rotation_weak_ratio <= config.weak_eigenvalue_ratio;
  result.translation_weak =
      result.translation_weak_ratio <= config.weak_eigenvalue_ratio;
  result.map_rotation_weak_direction = local.rotation_block_eigenvectors.col(0).normalized();
  // P6-I3 applies one scalar NDT-resolution scale to all translation axes, so
  // S_t v has the same direction as the normalized-block eigenvector.
  result.map_translation_weak_direction =
      (config.ndt_translation_scale_m *
       local.translation_block_eigenvectors.col(0)).normalized();
  if (!result.map_rotation_weak_direction.allFinite() ||
      !result.map_translation_weak_direction.allFinite()) {
    result.status = "INVALID_WEAK_DIRECTION";
    result.translation_weak = false;
    result.rotation_weak = false;
    return result;
  }
  result.valid = true;
  result.status = "VALID_BLOCK_WEAK_DIRECTIONS";
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

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
