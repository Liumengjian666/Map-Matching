#include <cassert>
#include <cmath>

#include <Eigen/Geometry>

#include "dog_prior_map_localization/core/psd_covariance_damping.hpp"
#include "dog_prior_map_localization/core/selective_relative_visual_update.hpp"

using namespace dog_prior_map_localization;

int main()
{
  Matrix15d damped_covariance = Matrix15d::Identity();
  damped_covariance(0, 3) = 0.2;
  damped_covariance(3, 0) = 0.2;
  assert(applyPsdPoseCovarianceDamping(&damped_covariance, 0.85, 1e-4, 1e-5));
  assert(std::abs(damped_covariance(0, 0) - 0.85) < 1e-12);
  assert(std::abs(damped_covariance(6, 6) - 0.85) < 1e-12);
  Eigen::SelfAdjointEigenSolver<Matrix15d> damped_eigen(damped_covariance);
  assert(damped_eigen.info() == Eigen::Success && damped_eigen.eigenvalues().minCoeff() > 0.0);
  Matrix15d invalid_damped_covariance = Matrix15d::Identity();
  invalid_damped_covariance(0, 3) = 1.5;
  invalid_damped_covariance(3, 0) = 1.5;
  assert(!applyPsdPoseCovarianceDamping(&invalid_damped_covariance, 0.85, 1e-4, 1e-5));

  FilterStateSnapshot reference;
  reference.stamp = 1.0;
  reference.P = Matrix15d::Identity();
  FilterStateSnapshot current = reference;
  current.stamp = 1.1;
  current.P = Matrix15d::Identity();

  Eigen::Matrix4d relative = Eigen::Matrix4d::Identity();
  relative(0, 3) = 1.0;
  Matrix6d covariance = Matrix6d::Identity() * 0.01;
  const Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();

  auto no_degeneracy = applySelectiveRelativeVisualUpdate(&current, reference,
      relative, covariance, false, axes, false, axes);
  assert(!no_degeneracy.accepted && no_degeneracy.status == "SKIPPED_NO_LIDAR_DEGENERACY");

  auto translation = applySelectiveRelativeVisualUpdate(&current, reference,
      relative, covariance, true, axes, false, axes);
  assert(translation.accepted && translation.dimension == 1);
  assert(current.p.x() > 0.0 && current.p.x() < 1.0);
  assert(std::abs(current.p.y()) < 1e-12 && std::abs(current.p.z()) < 1e-12);
  assert(current.P.allFinite());
  Eigen::SelfAdjointEigenSolver<Matrix15d> translation_eigen(current.P);
  assert(translation_eigen.info() == Eigen::Success &&
         translation_eigen.eigenvalues().minCoeff() > 0.0);

  FilterStateSnapshot rotation_current = reference;
  relative.setIdentity();
  relative.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Matrix3d rotation_axes;
  rotation_axes.col(0) = Eigen::Vector3d::UnitZ();
  rotation_axes.col(1) = Eigen::Vector3d::UnitX();
  rotation_axes.col(2) = Eigen::Vector3d::UnitY();
  auto rotation = applySelectiveRelativeVisualUpdate(&rotation_current, reference,
      relative, covariance, false, axes, true, rotation_axes);
  assert(rotation.accepted && rotation.dimension == 1);
  assert(rotation_current.R.allFinite());
  assert(std::abs(Eigen::AngleAxisd(rotation_current.R).angle()) > 0.0);
  Eigen::SelfAdjointEigenSolver<Matrix15d> rotation_eigen(rotation_current.P);
  assert(rotation_eigen.info() == Eigen::Success && rotation_eigen.eigenvalues().minCoeff() > 0.0);

  Matrix6d invalid_covariance = Matrix6d::Identity();
  invalid_covariance(0, 0) = -1.0;
  auto invalid = applySelectiveRelativeVisualUpdate(&rotation_current, reference,
      relative, invalid_covariance, false, axes, true, axes);
  assert(!invalid.accepted && invalid.status == "INVALID_RELATIVE_COVARIANCE");

  return 0;
}
