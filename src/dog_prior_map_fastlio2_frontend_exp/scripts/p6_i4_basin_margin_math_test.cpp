#include "p6_i4_basin_margin_math.hpp"

#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void testOperationalModeBoundaries() {
  using namespace p6_i4;
  const Pose identity = Pose::Identity();
  Pose at_translation = identity;
  at_translation(0, 3) = 0.20;
  require(compareOperationalMode(identity, true, at_translation, true).same_mode,
          "translation threshold must be inclusive");
  at_translation(0, 3) = std::nextafter(0.20, 1.0);
  require(!compareOperationalMode(identity, true, at_translation, true).same_mode,
          "translation above threshold must be different mode");

  Pose at_rotation = identity;
  at_rotation.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(compareOperationalMode(identity, true, at_rotation, true).same_mode,
          "rotation threshold must be inclusive");
  at_rotation.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(2.0001 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(!compareOperationalMode(identity, true, at_rotation, true).same_mode,
          "rotation above threshold must be different mode");
  require(!compareOperationalMode(identity, false, identity, true).same_mode,
          "nonconverged result cannot be same operational mode");
}

void testProductPosePerturbationAndJacobian() {
  using namespace p6_i4;
  Pose nominal = Pose::Identity();
  nominal.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()).toRotationMatrix();
  nominal.block<3, 1>(0, 3) << 4.0, -2.0, 1.0;
  Vector6d delta;
  delta << 0.03, -0.02, 0.04, 0.2, -0.1, 0.05;
  const Pose perturbed = boxplusMapPose(nominal, delta);
  require((perturbed.block<3, 1>(0, 3) - nominal.block<3, 1>(0, 3) -
           delta.tail<3>()).norm() < 1e-14,
          "translation must use independent additive product tangent");
  require((perturbed.block<3, 3>(0, 0) -
           expSO3(delta.head<3>()) * nominal.block<3, 3>(0, 0)).norm() < 1e-14,
          "rotation must use map-frame left product perturbation");

  constexpr int kStateDof = 24;
  constexpr int kPositionIndex = 0;
  constexpr int kRotationIndex = 3;
  const Eigen::MatrixXd jacobian = buildPoseErrorJacobian(
      nominal.block<3, 3>(0, 0), kStateDof, kPositionIndex, kRotationIndex);
  require((jacobian.block<3, 3>(0, kRotationIndex) -
           nominal.block<3, 3>(0, 0)).norm() < 1e-14,
          "right body rotation error must map through nominal R");
  require((jacobian.block<3, 3>(3, kPositionIndex) -
           Eigen::Matrix3d::Identity()).norm() == 0.0,
          "map-frame position error block must be identity");
}

void testCovarianceCrossTermsAndWhitening() {
  using namespace p6_i4;
  Matrix6d state_covariance = Matrix6d::Identity();
  state_covariance(0, 3) = state_covariance(3, 0) = 0.15;
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::MatrixXd jacobian = buildPoseErrorJacobian(rotation, 6, 0, 3);
  const Matrix6d mapped = jacobian * state_covariance * jacobian.transpose();
  require(mapped.block<3, 3>(0, 3).norm() > 0.1,
          "rotation-position covariance cross terms must be preserved");
  const PoseCovarianceSpectrum spectrum = analyzePoseCovariance(mapped);
  require(spectrum.valid && spectrum.effective_rank == 6,
          "positive-definite pose covariance should remain full rank");
  Eigen::VectorXd unit = Eigen::VectorXd::Ones(spectrum.effective_rank);
  unit.normalize();
  const Vector6d delta = whitenedPerturbation(spectrum, unit, 1.7);
  require(std::abs(priorMetricRadius(delta, spectrum) - 1.7) < 1e-10,
          "whitened perturbation radius must equal alpha");

  Matrix6d semidefinite = Matrix6d::Identity();
  semidefinite(5, 5) = -1e-30;
  const PoseCovarianceSpectrum clamped = analyzePoseCovariance(semidefinite);
  require(clamped.valid && clamped.effective_rank == 5,
          "negative roundoff only may be clamped to zero");
  Eigen::VectorXd rank_five_direction = Eigen::VectorXd::Unit(5, 3);
  const Vector6d rank_five_delta = whitenedPerturbation(
      clamped, rank_five_direction, 0.75);
  require(std::abs(priorMetricRadius(rank_five_delta, clamped) - 0.75) < 1e-10,
          "rank-deficient whitening must use only active-support coordinates");
  semidefinite(5, 5) = -1e-4;
  require(!analyzePoseCovariance(semidefinite).valid,
          "materially negative covariance eigenvalue must fail");
}
}  // namespace

int main() {
  try {
    testOperationalModeBoundaries();
    testProductPosePerturbationAndJacobian();
    testCovarianceCrossTermsAndWhitening();
    std::cout << "P6_I4_MATH_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I4_MATH_TEST_FAIL: " << error.what() << '\n';
    return 1;
  }
}
