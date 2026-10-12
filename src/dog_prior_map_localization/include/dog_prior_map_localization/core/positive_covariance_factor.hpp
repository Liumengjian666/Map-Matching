#pragma once

#include <cmath>
#include <Eigen/Cholesky>
#include "dog_prior_map_localization/core/estimator_types.hpp"

namespace dog_prior_map_localization
{
using Matrix15ld = Eigen::Matrix<long double,15,15>;
struct PositiveCovarianceFactor
{
  bool valid = false;
  Matrix15d root = Matrix15d::Zero();
  Matrix15ld precise_root = Matrix15ld::Zero();
  // lambda_min(P) >= 1 / ||L^-1||_F^2 for P=L L^T. This is a
  // conservative positive bound, not the raw unscaled eigensolver result.
  double minimum_eigenvalue_lower_bound = NAN;
};

inline PositiveCovarianceFactor factorPositiveCovariance(const Matrix15d& covariance)
{
  PositiveCovarianceFactor result;
  if (!covariance.allFinite()) return result;
  // Covariance here spans >1e18 in spectral condition. Double eigensolvers
  // and even double LLT can report false signs. Factor the *stored entries*
  // in extended precision; do not change eigenvalues or add diagonal jitter.
  const Matrix15ld precise = covariance.cast<long double>();
  const Matrix15ld symmetric = (0.5L * (precise + precise.transpose())).eval();
  Eigen::LLT<Matrix15ld> factor(symmetric);
  if (factor.info() != Eigen::Success) return result;
  result.precise_root = factor.matrixL();
  result.root = result.precise_root.cast<double>();
  if (!result.precise_root.allFinite() || (result.precise_root.diagonal().array() <= 0.0L).any()) return result;
  const Matrix15ld inverse_root = result.precise_root.triangularView<Eigen::Lower>().solve(Matrix15ld::Identity());
  const long double norm_squared = inverse_root.squaredNorm();
  if (!std::isfinite(norm_squared) || norm_squared <= 0.0) return result;
  result.minimum_eigenvalue_lower_bound = static_cast<double>(1.0L / norm_squared);
  result.valid = std::isfinite(result.minimum_eigenvalue_lower_bound) &&
      result.minimum_eigenvalue_lower_bound > 0.0;
  return result;
}
} // namespace dog_prior_map_localization
