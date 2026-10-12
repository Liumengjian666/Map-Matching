#include "dog_prior_map_localization/core/selective_relative_visual_update.hpp"
#include "dog_prior_map_localization/core/positive_covariance_factor.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace dog_prior_map_localization
{
namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d& value)
{
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(),
            value.z(), 0.0, -value.x(),
            -value.y(), value.x(), 0.0;
  return result;
}

Eigen::Matrix3d expSo3(const Eigen::Vector3d& theta)
{
  const double angle = theta.norm();
  if (angle <= 1e-12) return Eigen::Matrix3d::Identity() + skew(theta);
  return Eigen::AngleAxisd(angle, theta / angle).toRotationMatrix();
}

Eigen::Vector3d logSo3(const Eigen::Matrix3d& rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || !angle_axis.axis().allFinite())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  return angle_axis.axis() * angle_axis.angle();
}

bool properRotation(const Eigen::Matrix3d& rotation)
{
  return rotation.allFinite() &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-5 &&
      std::abs(rotation.determinant() - 1.0) <= 1e-5;
}

bool orthonormalAxes(const Eigen::Matrix3d& axes)
{
  return axes.allFinite() &&
      (axes.transpose() * axes - Eigen::Matrix3d::Identity()).norm() <= 1e-5;
}

bool covariancePositive(const Eigen::MatrixXd& covariance)
{
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
      !covariance.allFinite()) return false;
  const Eigen::MatrixXd symmetric = 0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric, Eigen::EigenvaluesOnly);
  return solver.info() == Eigen::Success && solver.eigenvalues().allFinite() &&
      solver.eigenvalues().minCoeff() > 1e-12;
}

Eigen::Matrix<double, 6, 15> poseJacobian(const Eigen::Matrix3d& rotation)
{
  Eigen::Matrix<double, 6, 15> jacobian = Eigen::Matrix<double, 6, 15>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  // EKF orientation error is right/body; pose covariance is represented in
  // map-left translation/rotation tangent coordinates.
  jacobian.block<3, 3>(3, 6) = rotation;
  return jacobian;
}

Eigen::Matrix<double, 6, 6> adjoint(const Eigen::Matrix3d& rotation,
                                   const Eigen::Vector3d& translation)
{
  Eigen::Matrix<double, 6, 6> result = Eigen::Matrix<double, 6, 6>::Zero();
  result.block<3, 3>(0, 0) = rotation;
  result.block<3, 3>(0, 3) = skew(translation) * rotation;
  result.block<3, 3>(3, 3) = rotation;
  return result;
}
}  // namespace

SelectiveRelativeVisualUpdateResult applySelectiveRelativeVisualUpdate(
    FilterStateSnapshot* current,
    const FilterStateSnapshot& reference,
    const Eigen::Matrix4d& relative,
    const Matrix6d& relative_covariance,
    bool translation_degenerate,
    const Eigen::Matrix3d& translation_axes_map,
    bool rotation_degenerate,
    const Eigen::Matrix3d& rotation_axes_map,
    bool covariance_stability_enable)
{
  SelectiveRelativeVisualUpdateResult result;
  if (!current)
  {
    result.status = "INVALID_CURRENT_STATE";
    return result;
  }
  if (!current->p.allFinite() || !current->v.allFinite() || !properRotation(current->R) ||
      !current->ba.allFinite() || !current->bg.allFinite() || !current->P.allFinite() ||
      !reference.p.allFinite() || !properRotation(reference.R) || !reference.P.allFinite() ||
      !relative.allFinite() || !properRotation(relative.block<3, 3>(0, 0)) ||
      !relative_covariance.allFinite() ||
      !orthonormalAxes(translation_axes_map) || !orthonormalAxes(rotation_axes_map))
  {
    result.status = "INVALID_OR_NONFINITE_INPUT";
    return result;
  }
  const Matrix15d input_covariance_symmetric = 0.5 *
      (current->P + current->P.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix15d> input_covariance_eigen(
      input_covariance_symmetric, Eigen::EigenvaluesOnly);
  if (input_covariance_eigen.info() == Eigen::Success &&
      input_covariance_eigen.eigenvalues().allFinite())
    result.minimum_input_state_covariance_eigenvalue =
        input_covariance_eigen.eigenvalues().minCoeff();
  result.input_state_covariance_asymmetry =
      (current->P - current->P.transpose()).cwiseAbs().maxCoeff();
  PositiveCovarianceFactor current_factor15;
  if (covariance_stability_enable) {
    current_factor15 = factorPositiveCovariance(current->P);
    if (!current_factor15.valid || !factorPositiveCovariance(reference.P).valid) {
      result.status = "NON_PSD_INPUT_STATE_COVARIANCE";
      return result;
    }
  }
  if (!translation_degenerate && !rotation_degenerate)
  {
    result.status = "SKIPPED_NO_LIDAR_DEGENERACY";
    return result;
  }
  if (!covariancePositive(relative_covariance))
  {
    result.status = "INVALID_RELATIVE_COVARIANCE";
    return result;
  }

  std::vector<Eigen::Matrix<double, 1, 15>> rows;
  if (translation_degenerate)
  {
    Eigen::Vector3d axis = translation_axes_map.col(0);
    if (!axis.allFinite() || axis.norm() < 1e-9)
    {
      result.status = "INVALID_TRANSLATION_WEAK_AXIS";
      return result;
    }
    axis.normalize();
    Eigen::Matrix<double, 1, 15> row = Eigen::Matrix<double, 1, 15>::Zero();
    row.block<1, 3>(0, 0) = axis.transpose();
    rows.push_back(row);
  }
  if (rotation_degenerate)
  {
    Eigen::Vector3d axis = rotation_axes_map.col(0);
    if (!axis.allFinite() || axis.norm() < 1e-9)
    {
      result.status = "INVALID_ROTATION_WEAK_AXIS";
      return result;
    }
    axis.normalize();
    Eigen::Matrix<double, 1, 15> row = Eigen::Matrix<double, 1, 15>::Zero();
    row.block<1, 3>(0, 6) = axis.transpose() * current->R;
    rows.push_back(row);
  }

  const Eigen::Matrix3d relative_rotation = relative.block<3, 3>(0, 0);
  const Eigen::Vector3d relative_translation = relative.block<3, 1>(0, 3);
  const Eigen::Vector3d pseudo_position = reference.p + reference.R * relative_translation;
  const Eigen::Matrix3d pseudo_rotation = reference.R * relative_rotation;
  Eigen::VectorXd innovation(static_cast<Eigen::Index>(rows.size()));
  Eigen::Index row_index = 0;
  if (translation_degenerate)
  {
    const Eigen::Vector3d axis = translation_axes_map.col(0).normalized();
    innovation(row_index++) = axis.dot(pseudo_position - current->p);
  }
  if (rotation_degenerate)
  {
    const Eigen::Vector3d axis = rotation_axes_map.col(0).normalized();
    const Eigen::Vector3d rotation_residual = logSo3(pseudo_rotation * current->R.transpose());
    if (!rotation_residual.allFinite())
    {
      result.status = "INVALID_ROTATION_RESIDUAL";
      return result;
    }
    innovation(row_index++) = axis.dot(rotation_residual);
  }

  Eigen::MatrixXd selector(static_cast<Eigen::Index>(rows.size()), 15);
  for (std::size_t i = 0; i < rows.size(); ++i) selector.row(static_cast<Eigen::Index>(i)) = rows[i];
  const Eigen::MatrixXd current_covariance = selector * current->P * selector.transpose();
  if (!covariancePositive(current_covariance))
  {
    result.status = "INVALID_CURRENT_SUBSPACE_COVARIANCE";
    return result;
  }

  const Eigen::Matrix<double, 6, 15> reference_pose_jacobian = poseJacobian(reference.R);
  const Matrix6d reference_pose_covariance = reference_pose_jacobian * reference.P *
      reference_pose_jacobian.transpose();
  const Matrix6d map_adjoint = adjoint(reference.R, reference.p);
  const Matrix6d relative_map_covariance = map_adjoint * relative_covariance * map_adjoint.transpose();
  // Cauchy-Schwarz upper bound for unknown correlation between historical EKF
  // error and LiDAR-depth/PnP error: Cov(a+b) <= 2(Cov(a)+Cov(b)).
  const Matrix6d pseudo_pose_covariance = 2.0 *
      (reference_pose_covariance + relative_map_covariance);
  Eigen::MatrixXd measurement_selector(static_cast<Eigen::Index>(rows.size()), 6);
  measurement_selector.setZero();
  row_index = 0;
  if (translation_degenerate)
  {
    measurement_selector.block<1, 3>(row_index++, 0) =
        translation_axes_map.col(0).normalized().transpose();
  }
  if (rotation_degenerate)
  {
    measurement_selector.block<1, 3>(row_index++, 3) =
        rotation_axes_map.col(0).normalized().transpose();
  }
  const Eigen::MatrixXd pseudo_subspace_covariance = measurement_selector *
      pseudo_pose_covariance * measurement_selector.transpose();
  if (!covariancePositive(pseudo_subspace_covariance))
  {
    result.status = "INVALID_PSEUDO_SUBSPACE_COVARIANCE";
    return result;
  }

  const Eigen::MatrixXd innovation_bound = 2.0 *
      (current_covariance + pseudo_subspace_covariance);
  Eigen::LDLT<Eigen::MatrixXd> bound_factor(innovation_bound);
  if (bound_factor.info() != Eigen::Success || !bound_factor.isPositive())
  {
    result.status = "INVALID_INNOVATION_COVARIANCE";
    return result;
  }
  const double nis = innovation.dot(bound_factor.solve(innovation));
  static constexpr std::array<double, 6> kChiSquare999 =
      {{10.828, 13.816, 16.266, 18.467, 20.515, 22.458}};
  if (!std::isfinite(nis) || nis > kChiSquare999[rows.size() - 1])
  {
    result.status = "RELATIVE_INNOVATION_GATE";
    result.dimension = static_cast<int>(rows.size());
    result.normalized_innovation_squared = nis;
    return result;
  }

  Eigen::LDLT<Eigen::MatrixXd> current_factor(current_covariance);
  Eigen::LDLT<Eigen::MatrixXd> pseudo_factor(pseudo_subspace_covariance);
  if (current_factor.info() != Eigen::Success || !current_factor.isPositive() ||
      pseudo_factor.info() != Eigen::Success || !pseudo_factor.isPositive())
  {
    result.status = "CI_INFORMATION_FACTOR_FAILED";
    return result;
  }
  constexpr double ci_weight = 0.5;
  const Eigen::MatrixXd information = ci_weight * current_factor.solve(
      Eigen::MatrixXd::Identity(rows.size(), rows.size())) +
      (1.0 - ci_weight) * pseudo_factor.solve(
      Eigen::MatrixXd::Identity(rows.size(), rows.size()));
  Eigen::LDLT<Eigen::MatrixXd> information_factor(information);
  if (information_factor.info() != Eigen::Success || !information_factor.isPositive())
  {
    result.status = "CI_INFORMATION_SOLVE_FAILED";
    return result;
  }
  const Eigen::MatrixXd ci_covariance = information_factor.solve(
      Eigen::MatrixXd::Identity(rows.size(), rows.size()));
  const Eigen::VectorXd ci_delta = ci_covariance *
      ((1.0 - ci_weight) * pseudo_factor.solve(innovation));

  Eigen::LDLT<Eigen::MatrixXd> selector_factor(current_covariance);
  const Eigen::Matrix<double, 15, Eigen::Dynamic> conditional_gain =
      current->P * selector.transpose() * selector_factor.solve(
          Eigen::MatrixXd::Identity(rows.size(), rows.size()));
  const Vector15d state_delta = conditional_gain * ci_delta;
  if (!ci_covariance.allFinite() || !state_delta.allFinite())
  {
    result.status = "NONFINITE_CI_RESULT";
    return result;
  }

  Matrix15d conditional_covariance;
  Matrix15d updated_covariance;
  if (covariance_stability_enable) {
    // Joseph/conditional form: replace subtraction of nearly equal large
    // matrices by a sum of positive Gram products. Same conditional update.
    const Matrix15d residual_map = Matrix15d::Identity() - conditional_gain * selector;
    const Matrix15ld residual_root = residual_map.cast<long double>() * current_factor15.precise_root;
    const Matrix15ld conditional_precise = residual_root * residual_root.transpose();
    conditional_covariance = conditional_precise.cast<double>();
    Eigen::LLT<Eigen::MatrixXd> ci_factor(ci_covariance);
    if (ci_factor.info() != Eigen::Success) {
      result.status = "CI_COVARIANCE_FACTOR_FAILED";
      return result;
    }
    const Eigen::Matrix<long double,15,Eigen::Dynamic> ci_root = conditional_gain.cast<long double>() *
        Eigen::MatrixXd(ci_factor.matrixL()).cast<long double>();
    updated_covariance = (conditional_precise + ci_root * ci_root.transpose()).cast<double>();
  } else {
    updated_covariance = current->P + conditional_gain *
        (ci_covariance - current_covariance) * conditional_gain.transpose();
    conditional_covariance = current->P - conditional_gain *
        current_covariance * conditional_gain.transpose();
  }
  updated_covariance = (0.5 * (updated_covariance + updated_covariance.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix15d> conditional_eigen(
      0.5 * (conditional_covariance + conditional_covariance.transpose()),
      Eigen::EigenvaluesOnly);
  if (conditional_eigen.info() == Eigen::Success && conditional_eigen.eigenvalues().allFinite())
    result.minimum_conditional_covariance_eigenvalue = conditional_eigen.eigenvalues().minCoeff();
  Eigen::SelfAdjointEigenSolver<Matrix15d> updated_eigen(updated_covariance, Eigen::EigenvaluesOnly);
  if (updated_eigen.info() == Eigen::Success && updated_eigen.eigenvalues().allFinite())
    result.minimum_updated_covariance_eigenvalue = updated_eigen.eigenvalues().minCoeff();
  const bool invalid_covariance = covariance_stability_enable ?
      !factorPositiveCovariance(updated_covariance).valid :
      (updated_eigen.info() != Eigen::Success || !updated_eigen.eigenvalues().allFinite() ||
       updated_eigen.eigenvalues().minCoeff() < -1e-8);
  if (!updated_covariance.allFinite() || invalid_covariance)
  {
    result.status = "NON_PSD_CI_COVARIANCE";
    return result;
  }

  current->p += state_delta.segment<3>(0);
  current->v += state_delta.segment<3>(3);
  current->R = current->R * expSo3(state_delta.segment<3>(6));
  current->ba += state_delta.segment<3>(9);
  current->bg += state_delta.segment<3>(12);
  current->P = updated_covariance;
  if (!current->p.allFinite() || !current->v.allFinite() || !properRotation(current->R) ||
      !current->ba.allFinite() || !current->bg.allFinite() || !current->P.allFinite())
  {
    result.status = "NONFINITE_UPDATED_STATE";
    return result;
  }

  result.accepted = true;
  result.dimension = static_cast<int>(rows.size());
  result.status = "CI_SELECTIVE_RELATIVE_UPDATE_APPLIED";
  result.innovation = innovation;
  result.correction = state_delta;
  result.normalized_innovation_squared = nis;
  result.current_covariance_trace = current_covariance.trace();
  result.pseudo_covariance_trace = pseudo_subspace_covariance.trace();
  return result;
}

}  // namespace dog_prior_map_localization
