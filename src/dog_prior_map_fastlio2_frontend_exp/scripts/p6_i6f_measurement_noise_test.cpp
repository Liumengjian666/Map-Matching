#include "dog_prior_map_fastlio2_frontend_exp/measurement_noise_model.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <iostream>
#include <random>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {

bool close(double value, double limit) {
  return std::isfinite(value) && value <= limit;
}

Matrix6d makeQ() {
  Matrix6d q = Matrix6d::Zero();
  q.diagonal() << 0.04, 0.03, 0.02, 0.09, 0.07, 0.05;
  q(0, 3) = q(3, 0) = 0.006;
  q(1, 4) = q(4, 1) = -0.004;
  q(2, 5) = q(5, 2) = 0.003;
  return q;
}

Matrix6d makeLeverArmA() {
  Matrix6d a = Matrix6d::Identity();
  Eigen::Matrix3d skew;
  const Eigen::Vector3d lever(0.7, -0.2, 0.4);
  skew << 0.0, -lever.z(), lever.y(),
      lever.z(), 0.0, -lever.x(), -lever.y(), lever.x(), 0.0;
  a.block<3, 3>(3, 0) = -skew;
  return a;
}

double nis(const Vector6d& residual, const Matrix6d& covariance) {
  return residual.dot(covariance.ldlt().solve(residual));
}

}  // namespace

int main() {
  const Matrix6d q = makeQ();
  const Matrix6d a = makeLeverArmA();
  const auto propagated = propagatePhysicalLidarCovariance(a, q);
  if (!propagated.valid ||
      !close((propagated.residual_covariance -
              propagated.residual_covariance.transpose()).norm(), 1e-12))
    return 1;
  std::string reason;
  if (!covarianceIsPSD(propagated.residual_covariance, 1e-9, &reason))
    return 2;
  if (propagated.residual_covariance.block<3, 3>(3, 0).norm() <= 1e-8)
    return 3;

  const Matrix6d identity = Matrix6d::Identity();
  const auto zero_lever = propagatePhysicalLidarCovariance(identity, q);
  if (!zero_lever.valid || (zero_lever.residual_covariance - q).norm() > 1e-12)
    return 4;

  MeasurementNoiseModel empirical;
  if (!makeEmpiricalPoseResidualModel(q, "IKFOM_RIGHT_POSE_RESIDUAL", &empirical,
                                      &reason) || empirical.statistically_calibrated ||
      empirical.semantic != MeasurementNoiseSemantic::EMPIRICAL_POSE_RESIDUAL)
    return 5;

  const auto decision_noise = reliability::makePoseMeasurementNoise(
      0.05, 0.02, Eigen::Matrix3d::Identity(), reliability::LocalRisk{},
      reliability::NonlocalTerminalStability{}, false, false);
  if (!decision_noise.valid ||
      decision_noise.semantic != MeasurementNoiseSemantic::EMPIRICAL_POSE_RESIDUAL ||
      decision_noise.coordinate_definition != "IKFOM_RIGHT_POSE_RESIDUAL" ||
      decision_noise.statistically_calibrated)
    return 9;

  std::mt19937 generator(613u);
  Eigen::LLT<Matrix6d> factor(q);
  Matrix6d sample_cov = Matrix6d::Zero();
  constexpr int sample_count = 200000;
  std::normal_distribution<double> normal(0.0, 1.0);
  for (int sample = 0; sample < sample_count; ++sample) {
    Vector6d standard;
    for (int index = 0; index < 6; ++index) standard(index) = normal(generator);
    const Vector6d x = factor.matrixL() * standard;
    const Vector6d y = a * x;
    sample_cov.noalias() += y * y.transpose();
  }
  sample_cov /= static_cast<double>(sample_count);
  const double monte_carlo_error =
      (sample_cov - propagated.residual_covariance).norm() /
      propagated.residual_covariance.norm();
  if (!close(monte_carlo_error, 0.025)) return 6;

  Vector6d source;
  source << 0.1, -0.2, 0.05, 0.4, -0.1, 0.2;
  Vector6d target;
  Matrix6d target_cov;
  if (!transformResidualAndCovariance(source, a, q, &target, &target_cov,
                                      &reason))
    return 7;
  if (!close(std::abs(nis(source, q) - nis(target, target_cov)), 1e-8))
    return 8;

  std::cout << "R4_NOISE_TEST_PASS"
            << " symmetry_psd=1"
            << " cross_block=" << propagated.residual_covariance.block<3, 3>(3, 0).norm()
            << " monte_carlo_relative_error=" << monte_carlo_error
            << " nis_delta=" << std::abs(nis(source, q) - nis(target, target_cov))
            << " empirical_not_calibrated=1\n";
  return 0;
}
