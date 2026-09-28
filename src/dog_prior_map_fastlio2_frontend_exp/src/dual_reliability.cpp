#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

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
      std::isfinite(config.terminal_translation_limit_m) &&
      config.terminal_translation_limit_m > 0.0 &&
      std::isfinite(config.terminal_rotation_limit_rad) &&
      config.terminal_rotation_limit_rad > 0.0 &&
      std::isfinite(config.cautious_noise_inflation) &&
      config.cautious_noise_inflation >= 1.0 &&
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
  if (rotation_min <= 0.0 || translation_min <= 0.0 ||
      rotation_max <= 0.0 || translation_max <= 0.0) {
    result.status = "NONPOSITIVE_BLOCK_CURVATURE";
    return result;
  }
  result.rotation_weak_ratio = rotation_min / rotation_max;
  result.translation_weak_ratio = translation_min / translation_max;
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
    bool apply_directional_downdate, double additional_isotropic_inflation,
    const DualReliabilityConfig& config) {
  MeasurementNoiseResult result;
  if (!validConfig(config) || !std::isfinite(position_sigma_m) ||
      !std::isfinite(rotation_sigma_rad) || position_sigma_m <= 0.0 ||
      rotation_sigma_rad <= 0.0 || !validRotation(predicted_map_R_imu) ||
      !std::isfinite(additional_isotropic_inflation) ||
      additional_isotropic_inflation < 1.0) {
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
  Eigen::Matrix3d position_covariance =
      position_variance * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rotation_body_covariance =
      rotation_variance * Eigen::Matrix3d::Identity();
  if (apply_directional_downdate && local_risk.valid) {
    if (local_risk.translation_weak) {
      const Eigen::Vector3d v = local_risk.map_translation_weak_direction.normalized();
      if (!v.allFinite() || std::abs(v.norm() - 1.0) > 1e-10) {
        result.status = "INVALID_MAP_TRANSLATION_DIRECTION";
        return result;
      }
      position_covariance = position_variance *
          (Eigen::Matrix3d::Identity() +
           (config.gamma_translation - 1.0) * (v * v.transpose()));
    }
    if (local_risk.rotation_weak) {
      const Eigen::Vector3d map_direction =
          local_risk.map_rotation_weak_direction.normalized();
      // MTK SO3 box-plus is right/body multiplication. Convert the P6-I3
      // map-spatial weak direction to body coordinates at the predicted pose.
      const Eigen::Vector3d body_direction =
          predicted_map_R_imu.transpose() * map_direction;
      if (!body_direction.allFinite() ||
          std::abs(body_direction.norm() - 1.0) > 1e-10) {
        result.status = "INVALID_BODY_ROTATION_DIRECTION";
        return result;
      }
      rotation_body_covariance = rotation_variance *
          (Eigen::Matrix3d::Identity() +
           (config.gamma_rotation - 1.0) *
               (body_direction * body_direction.transpose()));
    }
  }
  result.covariance.block<3, 3>(0, 0) = additional_isotropic_inflation *
      position_covariance;
  result.covariance.block<3, 3>(3, 3) = additional_isotropic_inflation *
      rotation_body_covariance;
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
  const bool has_probe_record = nonlocal.status != "UNINITIALIZED" &&
                                nonlocal.status != "NOT_PROBED";
  result.nonlocal_risk = use_unonlocal && has_probe_record &&
      (!nonlocal.geometry_valid || !nonlocal.objectives_finite ||
       !nonlocal.all_converged ||
       nonlocal.max_nominal_translation_delta_m >
           config.terminal_translation_limit_m ||
       nonlocal.max_nominal_rotation_delta_rad >
           config.terminal_rotation_limit_rad);

  if (result.local_risk && result.nonlocal_risk) {
    result.action = UpdateAction::CAUTIOUS_UPDATE;
    result.additional_noise_inflation = config.cautious_noise_inflation;
    result.reason = "LOCAL_AND_TERMINAL_RESPONSE_RISK";
  } else if (result.nonlocal_risk) {
    result.action = UpdateAction::CAUTIOUS_UPDATE;
    result.additional_noise_inflation = config.cautious_noise_inflation;
    result.reason = "TERMINAL_RESPONSE_RISK";
  } else if (result.local_risk) {
    result.action = UpdateAction::DIRECTIONAL_UPDATE;
    result.reason = "LOCAL_WEAK_DIRECTION";
  } else {
    result.action = UpdateAction::NORMAL_UPDATE;
    result.reason = local_available ? "NO_RELIABILITY_RISK" :
                                      "UOBS_UNAVAILABLE_NONLOCAL_NOT_RISKY";
  }
  const bool apply_directional = local_available && result.local_risk;
  result.measurement_noise = makePoseMeasurementNoise(
      position_sigma_m, rotation_sigma_rad, predicted_map_R_imu,
      local, apply_directional, result.additional_noise_inflation, config);
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
