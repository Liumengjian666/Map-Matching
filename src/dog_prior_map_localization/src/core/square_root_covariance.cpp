#include "dog_prior_map_localization/core/square_root_covariance.hpp"
#include <Eigen/QR>
#include <cmath>

namespace dog_prior_map_localization {
bool validCovarianceRoot(const Matrix15d& root)
{
  return root.allFinite() && (root.diagonal().array() > 0.0).all() &&
      root.triangularView<Eigen::StrictlyUpper>().toDenseMatrix().isZero(0.0);
}

bool covarianceRootFromColumns(const Eigen::MatrixXd& columns, Matrix15d* root)
{
  if (!root || columns.rows() != 15 || columns.cols() < 15 || !columns.allFinite()) return false;
  // A A^T = R^T R for A^T = Q R. Normalize signs without changing the Gram.
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(columns.transpose());
  Matrix15d R = qr.matrixQR().topLeftCorner<15,15>().triangularView<Eigen::Upper>();
  for (int i=0; i<15; ++i) if (R(i,i) < 0.0) R.row(i) *= -1.0;
  const Matrix15d candidate = R.transpose();
  if (!validCovarianceRoot(candidate)) return false;
  *root = candidate;
  return true;
}

bool propagateCovarianceRoot(const Matrix15d& F, const Eigen::MatrixXd& noise_root,
                             Matrix15d* root)
{
  if (!root || !validCovarianceRoot(*root) || !F.allFinite() || noise_root.rows()!=15) return false;
  Eigen::MatrixXd columns(15,15+noise_root.cols());
  columns.leftCols<15>() = F * *root;
  columns.rightCols(noise_root.cols()) = noise_root;
  return covarianceRootFromColumns(columns, root);
}

Matrix15d diagonalImuNoiseRoot(double dt, double acc, double gyro,
                             double acc_bias, double gyro_bias)
{
  Matrix15d root = Matrix15d::Zero();
  root.diagonal().segment<3>(3).setConstant(std::abs(acc)*dt);
  root.diagonal().segment<3>(6).setConstant(std::abs(gyro)*dt);
  root.diagonal().segment<3>(9).setConstant(std::abs(acc_bias)*std::sqrt(dt));
  root.diagonal().segment<3>(12).setConstant(std::abs(gyro_bias)*std::sqrt(dt));
  return root;
}

bool dampPoseCovarianceRoot(Matrix15d* root, double factor,
                           double position_floor, double rotation_floor)
{
  if (!root || !validCovarianceRoot(*root) || !std::isfinite(factor) || factor<=0.0 ||
      factor>1.0 || !std::isfinite(position_floor) || !std::isfinite(rotation_floor) ||
      position_floor<=0.0 || rotation_floor<=0.0) return false;
  Matrix15d candidate = *root;
  for (int i : {0,1,2,6,7,8}) {
    const double variance = root->row(i).squaredNorm();
    if (!std::isfinite(variance) || variance<=0.0) return false;
    const double floor = i<3 ? position_floor : rotation_floor;
    candidate.row(i) *= std::sqrt(std::max(variance*factor,floor)/variance);
  }
  if (!validCovarianceRoot(candidate)) return false;
  *root = candidate;
  return true;
}
}  // namespace dog_prior_map_localization
