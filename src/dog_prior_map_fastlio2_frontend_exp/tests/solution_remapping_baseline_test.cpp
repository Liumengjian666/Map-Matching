#include "dog_prior_map_fastlio2_frontend_exp/solution_remapping_baseline.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace r = paper::reliability;
namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
r::Matrix6d coupledBasis() {
  r::Matrix6d q = r::Matrix6d::Identity();
  // Same non-axis, rotation+translation fixture as the P7-C joint basis test.
  for (const auto& pair : {std::pair<int, double>{2, 0.4}, {3, -0.7}, {4, 0.6}}) {
    r::Matrix6d rotation = r::Matrix6d::Identity();
    rotation(0, 0) = rotation(pair.first, pair.first) = std::cos(pair.second);
    rotation(pair.first, 0) = std::sin(pair.second);
    rotation(0, pair.first) = -std::sin(pair.second);
    q = rotation * q;
  }
  return q;
}
r::LocalObservability localWithWeakDimension(int weak) {
  r::LocalObservability local;
  local.valid = local.geometric_proxy = local.map_support_sufficient = true;
  local.translation_length_scale_m = 0.8;
  Eigen::Matrix<double, 6, 1> values = Eigen::Matrix<double, 6, 1>::Ones();
  values.head(weak).setConstant(0.001);
  if (weak == 6) values.setZero();
  const auto q = coupledBasis();
  local.normalized_geometric_information = q * values.asDiagonal() * q.transpose();
  Eigen::SelfAdjointEigenSolver<r::Matrix6d> solver(local.normalized_geometric_information);
  require(solver.info() == Eigen::Success, "fixture eigensolve");
  local.joint_eigenvalues = solver.eigenvalues();
  local.joint_eigenvectors = solver.eigenvectors();
  return local;
}
paper::Pose3d pose(const Eigen::Vector3d& position, const Eigen::Vector3d& phi) {
  paper::Pose3d result;
  result.position = position;
  if (phi.norm() != 0.0)
    result.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(phi.norm(), phi.normalized()));
  return result;
}
void equalPose(const paper::Pose3d& a, const paper::Pose3d& b) {
  require((a.position - b.position).norm() < 1e-12, "pose translation differs");
  require((a.orientation.toRotationMatrix() - b.orientation.toRotationMatrix()).norm() < 1e-12,
      "pose rotation differs");
}
void testAllDimensions() {
  const auto predicted = pose({1.2, -0.9, 0.7}, {0.3, -0.2, 0.4});
  const auto raw = pose({2.4, -0.2, 0.1}, {-0.2, 0.4, 0.7});
  for (int weak = 0; weak <= 6; ++weak) {
    const auto local = localWithWeakDimension(weak);
    const auto subspace = r::classifyFixedPhysicalJointSubspace(local);
    const auto result = paper::remapNdtSolutionBaseline(predicted, raw, local, subspace);
    require(result.valid && result.weak_dimension == weak && result.reliable_dimension == 6 - weak,
        "invalid remapping dimensions");
    const auto v = subspace.reliable_basis.leftCols(6 - weak);
    const auto w = subspace.weak_basis.leftCols(weak);
    require(result.projector_symmetry_error < 1e-10, "projector symmetry");
    require(result.projector_idempotence_error < 1e-10, "projector idempotence");
    require((result.projector * v - v).norm() < 1e-10, "reliable preservation");
    require((result.projector * w).norm() < 1e-10, "weak rejection");
    require((v.transpose() * result.removed_correction).norm() < 1e-10, "removed correction not weak");
    require(result.safe_correction.norm() <= result.raw_correction.norm() + 1e-12, "projection energy");
    require(result.remapped_map_T_lidar.position.allFinite() &&
        result.remapped_map_T_lidar.orientation.coeffs().allFinite() &&
        std::abs(result.remapped_map_T_lidar.orientation.norm() - 1.0) < 1e-12, "finite normalized pose");
    if (weak == 0) equalPose(result.remapped_map_T_lidar, raw);
    if (weak == 6) {
      require(!result.lidar_measurement_available && result.safe_correction.norm() == 0.0,
          "rank zero must be prediction-only, not a zero-residual measurement");
      equalPose(result.remapped_map_T_lidar, predicted);
    } else require(result.lidar_measurement_available, "missing reliable measurement");
  }
}
void testCoupledAndFiniteCorrections() {
  const auto local = localWithWeakDimension(1);
  const auto subspace = r::classifyFixedPhysicalJointSubspace(local);
  const auto weak = subspace.weak_basis.col(0).eval();
  require(weak.head<3>().norm() > 0.1 && weak.tail<3>().norm() > 0.1 &&
      weak.cwiseAbs().maxCoeff() < 0.95, "fixture is not coupled");
  const auto predicted = pose({1.2, -0.9, 0.7}, {0.3, -0.2, 0.4});
  paper::Pose3d raw;
  const Eigen::Vector3d rotation = 0.2 * weak.head<3>();
  raw.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(rotation.norm(), rotation.normalized())) *
      predicted.orientation;
  raw.position = predicted.position + 0.8 * 0.2 * weak.tail<3>();
  const auto rejected = paper::remapNdtSolutionBaseline(predicted, raw, local, subspace);
  require(rejected.valid && rejected.safe_correction.norm() < 1e-12, "coupled weak correction survived");
  equalPose(rejected.remapped_map_T_lidar, predicted);
  const auto zero = paper::remapNdtSolutionBaseline(predicted, predicted, local, subspace);
  require(zero.valid && zero.raw_correction.norm() < 1e-12, "zero correction");
  equalPose(zero.remapped_map_T_lidar, predicted);
  for (int i = 0; i < 30; ++i) {
    const auto p = pose({0.2 * i, -0.1 * i, 0.02 * i}, {0.02 * i, -0.01 * i, 0.005 * i});
    const auto raw_pose = pose({0.15 * i, -0.08 * i, 0.03 * i}, {-0.01 * i, 0.02 * i, 0.04 * i});
    const auto result = paper::remapNdtSolutionBaseline(p, raw_pose, local, subspace);
    require(result.valid && result.remapped_map_T_lidar.position.allFinite() &&
        result.remapped_map_T_lidar.orientation.coeffs().allFinite() &&
        std::abs(result.remapped_map_T_lidar.orientation.norm() - 1.0) < 1e-12, "SE3 finite fixture");
  }
}
void testInvalidInputsFailClosed() {
  auto local = localWithWeakDimension(3);
  auto subspace = r::classifyFixedPhysicalJointSubspace(local);
  const auto reject = [&](const paper::Pose3d& p, const r::LocalObservability& l,
                          const r::FixedPhysicalJointSubspace& s) {
    const auto result = paper::remapNdtSolutionBaseline(p, paper::Pose3d{}, l, s);
    require(!result.valid && !result.lidar_measurement_available, "invalid input admitted");
  };
  local.valid = false; reject({}, local, subspace); local.valid = true;
  subspace.valid = false; reject({}, local, subspace); subspace.valid = true;
  local.translation_length_scale_m = 0.0; reject({}, local, subspace);
  local.translation_length_scale_m = 0.8;
  auto bad_pose = paper::Pose3d{}; bad_pose.position.x() = std::numeric_limits<double>::quiet_NaN();
  reject(bad_pose, local, subspace);
  subspace.reliable_basis.col(0).setZero(); reject({}, local, subspace);
}
}  // namespace
int main() {
  try {
    testAllDimensions(); testCoupledAndFiniteCorrections(); testInvalidInputsFailClosed();
    std::cout << "solution_remapping_baseline_test PASS (projector, coupled modes, rank 0-6, finite SE3)\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "solution_remapping_baseline_test FAIL: " << error.what() << '\n';
    return 1;
  }
}
