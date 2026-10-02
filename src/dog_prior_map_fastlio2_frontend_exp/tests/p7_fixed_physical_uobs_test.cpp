#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace r = dog_prior_map_fastlio2_frontend_exp::reliability;
namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
r::LocalObservability fromInformation(const r::Matrix6d& information) {
  r::LocalObservability local;
  local.valid = true;
  local.geometric_proxy = true;
  local.map_support_sufficient = true;
  local.translation_length_scale_m = 0.8;
  local.normalized_geometric_information = information;
  Eigen::SelfAdjointEigenSolver<r::Matrix6d> solver(information);
  require(solver.info() == Eigen::Success, "fixture eigensolve failed");
  local.joint_eigenvalues = solver.eigenvalues();
  local.joint_eigenvectors = solver.eigenvectors();
  return local;
}
r::Matrix6d coupledBasis() {
  r::Matrix6d basis = r::Matrix6d::Identity();
  for (const auto& pair : {std::pair<int, double>{2, 0.4}, {3, -0.7}, {4, 0.6}}) {
    r::Matrix6d rotation = r::Matrix6d::Identity();
    rotation(0, 0) = rotation(pair.first, pair.first) = std::cos(pair.second);
    rotation(pair.first, 0) = std::sin(pair.second);
    rotation(0, pair.first) = -std::sin(pair.second);
    basis = rotation * basis;
  }
  return basis;
}
void checkBases(const r::FixedPhysicalJointSubspace& s, int weak_dimension) {
  require(s.valid && s.weak_dimension == weak_dimension &&
      s.reliable_dimension == 6 - weak_dimension, "wrong joint dimensions");
  const auto w = s.weak_basis.leftCols(s.weak_dimension);
  const auto b = s.reliable_basis.leftCols(s.reliable_dimension);
  require((w.transpose() * w - Eigen::MatrixXd::Identity(weak_dimension, weak_dimension)).norm() < 1e-8,
      "weak basis is not orthonormal");
  require((b.transpose() * b - Eigen::MatrixXd::Identity(6 - weak_dimension, 6 - weak_dimension)).norm() < 1e-8,
      "reliable basis is not orthonormal");
  require((w.transpose() * b).norm() < 1e-8 &&
      (w * w.transpose() + b * b.transpose() - r::Matrix6d::Identity()).norm() < 1e-8,
      "joint bases are not orthogonal complements");
}
void testPhysicalInformationAndSchur() {
  std::vector<r::GeometricObservation> observations;
  r::Matrix6d expected = r::Matrix6d::Zero();
  double weight_sum = 0.0;
  for (int i = 0; i < 64; ++i) {
    r::GeometricObservation o;
    o.rotated_source_map = Eigen::Vector3d(0.4 + 0.2 * (i % 4),
        -0.8 + 0.3 * ((i / 4) % 4), -0.5 + 0.4 * (i / 16));
    o.voxel_covariance_map = Eigen::Vector3d(0.02, 0.04, 0.03).asDiagonal();
    o.nonnegative_weight = 0.5 + 0.02 * i;
    const auto j = r::geometricPointResidualJacobian(o.rotated_source_map);
    expected += o.nonnegative_weight * j.transpose() *
        Eigen::Vector3d(50.0, 25.0, 1.0 / 0.03).asDiagonal() * j;
    weight_sum += o.nonnegative_weight;
    observations.push_back(o);
  }
  expected /= weight_sum;
  const auto local = r::analyzeGeometricObservability(observations, true, 0.8);
  require(local.valid && local.physical_geometric_information.allFinite(), "physical information invalid");
  require((local.physical_geometric_information - expected).norm() < 1e-12 * expected.norm(),
      "weighted physical information changed");
  r::Matrix6d d = r::Matrix6d::Identity(); d.bottomRightCorner<3, 3>() *= 0.8;
  const r::Matrix6d normalized = d.transpose() * expected * d;
  require((local.normalized_geometric_information - normalized).norm() < 1e-12 * normalized.norm(),
      "Hbar is not D-transpose Hphys D");
  require(local.schur_decoupling_valid && local.schur_status == "VALID_DIAGNOSTIC",
      "Schur diagnostics unavailable in rich fixture");
  const Eigen::Matrix3d a = normalized.topLeftCorner<3, 3>();
  const Eigen::Matrix3d b = normalized.topRightCorner<3, 3>();
  const Eigen::Matrix3d c = normalized.bottomRightCorner<3, 3>();
  const Eigen::Matrix3d sr = a - b * c.ldlt().solve(b.transpose());
  const Eigen::Matrix3d st = c - b.transpose() * a.ldlt().solve(b);
  require((local.rotation_schur_information - sr).norm() < 1e-10 &&
      (local.translation_schur_information - st).norm() < 1e-10, "Schur differs from reference solve");
  observations.front().voxel_covariance_map(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = r::analyzeGeometricObservability(observations, true, 0.8);
  require(!invalid.valid && invalid.numerical_failure, "nonfinite covariance did not fail closed");
  observations.front().voxel_covariance_map(0, 0) = 0.02;
  for (int component = 0; component < 3; ++component) {
    auto mixed = observations;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (component == 0) mixed.front().rotated_source_map.x() = nan;
    if (component == 1) mixed.front().residual_map.x() = nan;
    if (component == 2) mixed.front().nonnegative_weight = nan;
    const auto failed = r::analyzeGeometricObservability(mixed, true, 0.8);
    require(!failed.valid && failed.numerical_failure,
        "nonfinite point/residual/weight hidden by valid observations");
  }
  r::GeometricObservation huge;
  huge.voxel_covariance_map = 1e100 * Eigen::Matrix3d::Identity();
  huge.nonnegative_weight = 1e308;
  const auto overflow = r::analyzeGeometricObservability({huge, huge}, true, 0.8);
  require(!overflow.valid && overflow.numerical_failure &&
      overflow.map_support_status == "NUMERICAL_FAILURE",
      "low support hid aggregate weight overflow");
}
void testSpectraAndCoupling() {
  const r::Matrix6d q = coupledBasis();
  Eigen::Matrix<double, 6, 1> values; values << 1, 1.1, 1.2, 1.3, 1.4, 1.5;
  checkBases(r::classifyFixedPhysicalJointSubspace(fromInformation(values.asDiagonal())), 0);
  values << 0.001, 1, 1.1, 1.2, 1.3, 1.4;
  const r::Matrix6d h = q * values.asDiagonal() * q.transpose();
  const auto coupled = r::classifyFixedPhysicalJointSubspace(fromInformation(h));
  checkBases(coupled, 1);
  const Eigen::Matrix<double, 6, 1> w = q.col(0);
  require(w.head<3>().norm() > 0.1 && w.tail<3>().norm() > 0.1 &&
      w.cwiseAbs().maxCoeff() < 0.95, "fixture is not coupled/non-axis-aligned");
  require((coupled.weak_basis.col(0) * coupled.weak_basis.col(0).transpose() -
      w * w.transpose()).norm() < 1e-10, "coupled weak direction was remapped");
  for (double scale : {1e-6, 1.0, 1e6}) {
    const auto scaled = r::classifyFixedPhysicalJointSubspace(fromInformation(scale * h));
    checkBases(scaled, 1);
    require((scaled.weak_basis.col(0) * scaled.weak_basis.col(0).transpose() -
        w * w.transpose()).norm() < 1e-10, "positive global scale changed subspace");
  }
  for (int weak = 0; weak <= 6; ++weak) {
    values.setOnes();
    values.head(weak).setConstant(0.001);
    if (weak == 6) values.setZero();
    const auto s = r::classifyFixedPhysicalJointSubspace(
        fromInformation(q * values.asDiagonal() * q.transpose()));
    checkBases(s, weak);
    if (weak == 6) require(s.status == "NO_NUMERICALLY_RESOLVED_INFORMATION", "zero status");
  }
}
void testNoGroupEqualizationAndFailures() {
  Eigen::Matrix<double, 6, 1> values; values << 1000, 1100, 1200, 1, 1.1, 1.2;
  auto local = fromInformation(values.asDiagonal());
  const auto s = r::classifyFixedPhysicalJointSubspace(local);
  checkBases(s, 3);
  require(s.lambda_max == 1200 && s.weak_threshold == 60 && s.weak_relative_ratio == 0.05,
      "joint magnitudes were equalized or threshold changed");
  require((s.weak_basis.topRows<3>()).norm() < 1e-12, "translation weak modes lost");
  // Deliberately unavailable diagnostics do not veto valid joint information.
  local.schur_decoupling_valid = false;
  local.schur_status = "SCHUR_DIAGNOSTIC_UNAVAILABLE";
  checkBases(r::classifyFixedPhysicalJointSubspace(local), 3);
  values(3) = -0.1;
  const auto indefinite = r::classifyFixedPhysicalJointSubspace(fromInformation(values.asDiagonal()));
  require(!indefinite.valid && indefinite.status == "NEGATIVE_JOINT_INFORMATION_EIGENVALUE" &&
      indefinite.reliable_dimension == 0, "indefinite information repaired");
  values.setOnes(); values(0) = -std::numeric_limits<double>::epsilon();
  const auto roundoff = r::classifyFixedPhysicalJointSubspace(fromInformation(values.asDiagonal()));
  checkBases(roundoff, 1);
  require(roundoff.eigenvalues(0) == 0, "roundoff negative not clamped");
  local = fromInformation(r::Matrix6d::Identity());
  local.joint_eigenvectors.col(0) = local.joint_eigenvectors.col(1);
  const auto bad_basis = r::classifyFixedPhysicalJointSubspace(local);
  require(!bad_basis.valid && bad_basis.reliable_dimension == 0, "nonorthogonal basis accepted");
  local = fromInformation(r::Matrix6d::Identity()); local.map_support_sufficient = false;
  require(!r::classifyFixedPhysicalJointSubspace(local).valid, "unsupported geometry accepted");
  local.map_support_sufficient = true; local.normalized_geometric_information(0, 0) =
      std::numeric_limits<double>::infinity();
  require(!r::classifyFixedPhysicalJointSubspace(local).valid, "nonfinite information accepted");
  require(!r::classifyFixedPhysicalJointSubspace(local, 0.0).valid, "invalid ratio accepted");
}
}  // namespace
int main() {
  try {
    testPhysicalInformationAndSchur(); testSpectraAndCoupling(); testNoGroupEqualizationAndFailures();
    std::cout << "p7_fixed_physical_uobs_test PASS (physical normalization, Schur, joint rank 0-6)\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "p7_fixed_physical_uobs_test FAIL: " << e.what() << '\n';
    return 1;
  }
}
