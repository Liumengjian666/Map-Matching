#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

#include "dog_prior_map_localization/core/estimator_types.hpp"
#include "dog_prior_map_localization/core/positive_covariance_factor.hpp"

namespace dog_prior_map_localization
{

// Preserve the legacy pose-variance contraction targets while applying them
// as a diagonal congruence, D P D. Unlike replacing selected diagonal entries
// in isolation, this preserves positive semidefiniteness and scales the
// corresponding cross-covariances consistently.
inline bool applyPsdPoseCovarianceDamping(Matrix15d* covariance,
    double factor, double position_floor, double rotation_floor,
    bool covariance_stability_enable = false)
{
  if (!covariance || !covariance->allFinite() || !std::isfinite(factor) ||
      factor <= 0.0 || factor > 1.0 || !std::isfinite(position_floor) ||
      position_floor <= 0.0 || !std::isfinite(rotation_floor) || rotation_floor <= 0.0)
    return false;

  const Matrix15d symmetric = 0.5 * (*covariance + covariance->transpose());
  if (covariance_stability_enable) {
    if (!factorPositiveCovariance(symmetric).valid) return false;
  } else {
    Eigen::SelfAdjointEigenSolver<Matrix15d> input_eigen(symmetric, Eigen::EigenvaluesOnly);
    if (input_eigen.info() != Eigen::Success || !input_eigen.eigenvalues().allFinite() ||
        input_eigen.eigenvalues().minCoeff() < -1e-10) return false;
  }

  Eigen::Matrix<double, 15, 1> scale = Eigen::Matrix<double, 15, 1>::Ones();
  for (int index : {0, 1, 2, 6, 7, 8})
  {
    const double old_variance = symmetric(index, index);
    if (!std::isfinite(old_variance) || old_variance <= 0.0) return false;
    const double floor = index < 3 ? position_floor : rotation_floor;
    const double new_variance = std::max(old_variance * factor, floor);
    scale(index) = std::sqrt(new_variance / old_variance);
  }
  const Matrix15d diagonal_scale = scale.asDiagonal();
  Matrix15d damped = diagonal_scale * symmetric * diagonal_scale;
  if (covariance_stability_enable) {
    const Matrix15ld precise = covariance->cast<long double>();
    const Matrix15ld symmetric_precise = (0.5L*(precise+precise.transpose())).eval();
    const Matrix15ld precise_scale=diagonal_scale.cast<long double>();
    damped=(precise_scale*symmetric_precise*precise_scale).cast<double>();
  }
  damped = (0.5 * (damped + damped.transpose())).eval();
  if (!damped.allFinite()) return false;
  if (covariance_stability_enable) {
    if (!factorPositiveCovariance(damped).valid) return false;
  } else {
    Eigen::SelfAdjointEigenSolver<Matrix15d> output_eigen(damped, Eigen::EigenvaluesOnly);
    if (output_eigen.info() != Eigen::Success || !output_eigen.eigenvalues().allFinite() ||
        output_eigen.eigenvalues().minCoeff() < -1e-10) return false;
  }
  *covariance = damped;
  return true;
}

}  // namespace dog_prior_map_localization
