#pragma once

#include "dog_prior_map_localization/core/estimator_types.hpp"

namespace dog_prior_map_localization {
// Positive lower triangular factor; no dense covariance refactorization.
bool validCovarianceRoot(const Matrix15d& root);
bool covarianceRootFromColumns(const Eigen::MatrixXd& columns, Matrix15d* root);
bool propagateCovarianceRoot(const Matrix15d& F, const Eigen::MatrixXd& noise_root,
                             Matrix15d* root);
// Exact factor of the existing diagonal IMU Q, including zero-noise entries.
Matrix15d diagonalImuNoiseRoot(double dt, double acc, double gyro,
                             double acc_bias, double gyro_bias);
bool dampPoseCovarianceRoot(Matrix15d* root, double factor,
                           double position_floor, double rotation_floor);
}  // namespace dog_prior_map_localization
