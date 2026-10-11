#include "candidate_adapter.hpp"
#include <iostream>
#include <limits>

int main() {
  Eigen::Matrix4d reference = Eigen::Matrix4d::Identity(), prior = reference;
  reference.block<3, 3>(0, 0) = (Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitY())).toRotationMatrix();
  reference.block<3, 1>(0, 3) = Eigen::Vector3d(7, -8, 2);
  prior.block<3, 3>(0, 0) = Eigen::AngleAxisd(-.8, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto original_reference = reference, original_prior = prior;
  const auto coarse = p12::coarsePosition(reference, prior);
  const auto diagnostic = p12::approximateReferenceStatePose(reference);
  if ((diagnostic-reference).norm() > 1e-12 ||
      (coarse.block<3, 3>(0, 0)-prior.block<3, 3>(0, 0)).norm() > 1e-12 ||
      (coarse.block<3, 1>(0, 3)-reference.block<3, 1>(0, 3)).norm() > 1e-12 ||
      (reference-original_reference).norm() > 0 || (prior-original_prior).norm() > 0) return 1;
  const Eigen::Vector4d sensor(1, 2, 3, 1), mapped = diagnostic * sensor;
  if ((diagnostic.inverse()*mapped-sensor).norm() > 1e-12 ||
      (mapped.head<3>()-(reference.block<3, 3>(0, 0)*sensor.head<3>()+reference.block<3, 1>(0, 3))).norm() > 1e-12)
    return 2;
  for (int kind = 0; kind < 3; ++kind) {
    auto bad = reference;
    if (kind == 0) bad(0, 3) = std::numeric_limits<double>::quiet_NaN();
    if (kind == 1) bad(0, 0) *= 2;
    if (kind == 2) bad(3, 0) = 1;
    try { p12::approximateReferenceStatePose(bad); return 3; }
    catch (const std::invalid_argument&) {}
  }
  std::cout << "PASS: historical coarse mode, full reference orientation, immutable inputs, source-to-map direction, invalid SE3 guards\n";
}
