#pragma once
#include <Eigen/Geometry>
#include <stdexcept>

namespace p12 {
inline bool rigid(const Eigen::Matrix4d& pose) {
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  return pose.allFinite() &&
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-5 &&
      std::abs(rotation.determinant() - 1.0) < 1e-5 &&
      (pose.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
}
// Deliberately NOT map_T_reference -> exact map_T_lidar. Reference translation
// is an IMU-origin coarse location; rotation comes from this query's NDT prior.
inline Eigen::Matrix4d coarsePosition(const Eigen::Matrix4d& map_T_reference,
                                     const Eigen::Matrix4d& ndt_prior) {
  if (!rigid(map_T_reference) || !rigid(ndt_prior))
    throw std::invalid_argument("invalid candidate/prior SE(3)");
  Eigen::Matrix4d hypothesis = ndt_prior;
  hypothesis.block<3, 1>(0, 3) = map_T_reference.block<3, 1>(0, 3);
  return hypothesis;
}
// P12B diagnostic only: keep the reference IMU-state origin AND orientation.
// Not an exact LiDAR pose: no unverified extrinsic or scan-time transport.
inline Eigen::Matrix4d approximateReferenceStatePose(const Eigen::Matrix4d& map_T_reference) {
  if (!rigid(map_T_reference))
    throw std::invalid_argument("invalid approximate reference SE(3)");
  return map_T_reference;
}
}  // namespace p12
