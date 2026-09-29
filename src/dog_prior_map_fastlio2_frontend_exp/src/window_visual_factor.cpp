#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {

bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

}  // namespace

bool buildVisualResidual(const WindowState& reference,
                         const WindowState& current,
                         const VisualRelativeMeasurement& measurement,
                         Eigen::Vector3d* residual, std::string* reason) {
  if (reason) reason->clear();
  if (!residual) return fail(reason, "null_visual_residual");
  if (!measurement.valid || reference.stamp_ns != measurement.reference_stamp_ns ||
      current.stamp_ns != measurement.current_stamp_ns ||
      current.stamp_ns <= reference.stamp_ns)
    return fail(reason, "invalid_visual_factor_endpoint");
  if (!measurement.reference_imu_translation.allFinite() ||
      !measurement.covariance.allFinite())
    return fail(reason, "invalid_visual_measurement");
  Eigen::LLT<Eigen::Matrix3d> covariance_factor(measurement.covariance);
  if (covariance_factor.info() != Eigen::Success)
    return fail(reason, "visual_factor_covariance_not_spd");
  *residual = current.position - reference.position -
      reference.rotation * measurement.reference_imu_translation;
  if (!residual->allFinite()) return fail(reason, "nonfinite_visual_residual");
  return true;
}

bool linearizeVisualFactor(
    const WindowState& reference, const WindowState& current,
    const VisualRelativeMeasurement& measurement, Eigen::Vector3d* residual,
    Eigen::Matrix<double, 3, 15>* jacobian_reference,
    Eigen::Matrix<double, 3, 15>* jacobian_current,
    std::string* reason) {
  if (!jacobian_reference || !jacobian_current)
    return fail(reason, "null_visual_jacobian_output");
  if (!buildVisualResidual(reference, current, measurement, residual, reason))
    return false;
  jacobian_reference->setZero();
  jacobian_current->setZero();
  jacobian_reference->block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
  jacobian_current->block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  // This is the requested right-perturbation convention for the window
  // residual chart. The factor's rotation differential is kept explicit so a
  // central-FD audit can detect a convention mismatch instead of hiding it.
  jacobian_reference->block<3, 3>(0, 0) =
      reference.rotation * skew(measurement.reference_imu_translation);
  return jacobian_reference->allFinite() && jacobian_current->allFinite();
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
