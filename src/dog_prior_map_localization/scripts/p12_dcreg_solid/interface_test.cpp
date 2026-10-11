#include "candidate_adapter.hpp"
#include "dcreg.hpp"
#include <iostream>
#include <limits>

int main() {
  const Eigen::Matrix4d prior = dcreg::PoseToMatrix(
      dcreg::PoseFromDegrees(2, -3, 0.7, 12, -17, 61));
  const Eigen::Matrix4d reference = dcreg::PoseToMatrix(
      dcreg::PoseFromDegrees(-8, 5, 2, -20, 7, -100));
  const auto hypothesis = p12::coarsePosition(reference, prior);
  if (!p12::rigid(hypothesis) ||
      (hypothesis.block<3, 3>(0, 0)-prior.block<3, 3>(0, 0)).norm() > 1e-12 ||
      (hypothesis.block<3, 1>(0, 3)-reference.block<3, 1>(0, 3)).norm() > 1e-12 ||
      (dcreg::PoseToMatrix(dcreg::MatrixToPose(prior))-prior).norm() > 1e-12)
    return 1;
  const dcreg::PointT point(1, 2, 3);
  const Eigen::Vector3d mapped = dcreg::TransformPoint(prior, point);
  const Eigen::Vector4d roundtrip = prior.inverse() * mapped.homogeneous();
  if ((roundtrip.head<3>()-Eigen::Vector3d(1, 2, 3)).norm() > 1e-12) return 2;
  auto invalid = prior;
  invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
  try { p12::coarsePosition(reference, invalid); return 3; }
  catch (const std::invalid_argument&) {}
  dcreg::PointCloud source, target;
  for (int axis = 0; axis < 3; ++axis) for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) {
    Eigen::Vector3d world(2+i*.1, 3+j*.1, 4);
    if (axis == 0) world = Eigen::Vector3d(5, 2+i*.1, 3+j*.1);
    if (axis == 1) world = Eigen::Vector3d(2+i*.1, 6, 3+j*.1);
    const Eigen::Vector4d sensor = prior.inverse()*world.homogeneous();
    source.emplace_back(sensor.x(), sensor.y(), sensor.z());
    target.emplace_back(world.x(), world.y(), world.z());
  }
  dcreg::TestCase test_case;
  test_case.has_ground_truth_pose = false;
  auto guess = prior; guess(0, 3) += .05;
  test_case.initial_pose = dcreg::MatrixToPose(guess);
  dcreg::RunOptions options;
  options.parameterization = dcreg::Parameterization::kSO3;
  options.parallel_mode = dcreg::ParallelMode::kSerial;
  options.verbose = false;
  const auto result = dcreg::RunRegistration(source, target, test_case, options);
  if (!result.converged || !p12::rigid(result.transform) || result.has_pose_error ||
      dcreg::CalculatePoseError(prior, result.transform).translation_m > 1e-3) return 4;
  std::cout << "PASS: coordinate roundtrip, coarse provenance, NaN guard, original DCReg three-plane registration\n";
}
