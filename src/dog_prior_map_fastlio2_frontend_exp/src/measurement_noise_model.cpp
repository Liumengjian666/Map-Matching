#include "dog_prior_map_fastlio2_frontend_exp/measurement_noise_model.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

bool finiteAndSymmetric(const Matrix6d& covariance, double tolerance) {
  return covariance.allFinite() &&
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() <= tolerance;
}

}  // namespace

const char* toString(MeasurementNoiseSemantic semantic) {
  switch (semantic) {
    case MeasurementNoiseSemantic::PHYSICAL_LIDAR_PERTURBATION:
      return "PHYSICAL_LIDAR_PERTURBATION";
    case MeasurementNoiseSemantic::EMPIRICAL_POSE_RESIDUAL:
      return "EMPIRICAL_POSE_RESIDUAL";
  }
  return "UNKNOWN";
}

bool covarianceIsPSD(const Matrix6d& covariance, double tolerance,
                     std::string* reason) {
  if (reason) reason->clear();
  if (!std::isfinite(tolerance) || tolerance < 0.0)
    return fail(reason, "invalid_psd_tolerance");
  if (!finiteAndSymmetric(covariance, tolerance))
    return fail(reason, "covariance_not_finite_or_symmetric");
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    return fail(reason, "covariance_eigensolve_failed");
  if (solver.eigenvalues().minCoeff() < -tolerance)
    return fail(reason, "covariance_not_psd");
  return true;
}

PhysicalCovariancePropagation propagatePhysicalLidarCovariance(
    const Matrix6d& A_exact, const Matrix6d& Q_lidar) {
  PhysicalCovariancePropagation result;
  if (!A_exact.allFinite() || !Q_lidar.allFinite()) {
    result.status = "NONFINITE_INPUT";
    return result;
  }
  std::string reason;
  if (!covarianceIsPSD(Q_lidar, 1e-10, &reason)) {
    result.status = "INVALID_PHYSICAL_COVARIANCE:" + reason;
    return result;
  }
  result.residual_covariance = A_exact * Q_lidar * A_exact.transpose();
  result.residual_covariance = (0.5 *
      (result.residual_covariance + result.residual_covariance.transpose())).eval();
  if (!result.residual_covariance.allFinite() ||
      !covarianceIsPSD(result.residual_covariance, 1e-9, &reason)) {
    result.status = "INVALID_PROPAGATED_COVARIANCE:" + reason;
    return result;
  }
  result.valid = true;
  result.status = "PASS_PHYSICAL_LIDAR_TO_POSE_RESIDUAL";
  return result;
}

bool makeEmpiricalPoseResidualModel(
    const Matrix6d& residual_covariance,
    const std::string& coordinate_definition,
    MeasurementNoiseModel* output, std::string* reason) {
  if (reason) reason->clear();
  if (!output) return fail(reason, "null_empirical_noise_output");
  if (coordinate_definition.empty())
    return fail(reason, "missing_empirical_coordinate_definition");
  std::string local_reason;
  if (!covarianceIsPSD(residual_covariance, 1e-9, &local_reason))
    return fail(reason, local_reason.c_str());
  output->semantic = MeasurementNoiseSemantic::EMPIRICAL_POSE_RESIDUAL;
  output->covariance = 0.5 *
      (residual_covariance + residual_covariance.transpose());
  output->coordinate_definition = coordinate_definition;
  output->statistically_calibrated = false;
  return true;
}

bool transformResidualAndCovariance(const Vector6d& residual_source,
                                    const Matrix6d& A_exact,
                                    const Matrix6d& Q_source,
                                    Vector6d* residual_target,
                                    Matrix6d* Q_target,
                                    std::string* reason) {
  if (reason) reason->clear();
  if (!residual_target || !Q_target)
    return fail(reason, "null_transformed_residual_output");
  if (!residual_source.allFinite() || !A_exact.allFinite())
    return fail(reason, "nonfinite_residual_transform_input");
  const PhysicalCovariancePropagation propagated =
      propagatePhysicalLidarCovariance(A_exact, Q_source);
  if (!propagated.valid) return fail(reason, propagated.status.c_str());
  *residual_target = A_exact * residual_source;
  *Q_target = propagated.residual_covariance;
  return residual_target->allFinite() && Q_target->allFinite();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
