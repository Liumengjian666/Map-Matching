#include "dog_prior_map_fastlio2_frontend_exp/solution_remapping_baseline.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"

#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
bool finitePose(const Pose3d& pose) {
  return pose.position.allFinite() && pose.orientation.coeffs().allFinite() &&
      std::isfinite(pose.orientation.norm()) && pose.orientation.norm() > 0.0;
}
Eigen::Matrix3d expRotation(const Eigen::Vector3d& phi) {
  const double angle = phi.norm();
  if (angle == 0.0) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
}
}  // namespace

SolutionRemappingBaselineResult remapNdtSolutionBaseline(
    const Pose3d& predicted_map_T_lidar, const Pose3d& raw_map_T_lidar,
    const reliability::LocalObservability& local,
    const reliability::FixedPhysicalJointSubspace& subspace) {
  SolutionRemappingBaselineResult result;
  result.weak_dimension = subspace.weak_dimension;
  result.reliable_dimension = subspace.reliable_dimension;
  result.translation_length_scale_m = local.translation_length_scale_m;
  result.remapped_map_T_lidar = predicted_map_T_lidar;
  if (!local.valid || !local.geometric_proxy || !local.map_support_sufficient) {
    result.status = "INVALID_UOBS";
    return result;
  }
  if (!subspace.valid) {
    result.status = "INVALID_JOINT_SUBSPACE";
    return result;
  }
  const double length = result.translation_length_scale_m;
  const int weak = result.weak_dimension, reliable = result.reliable_dimension;
  if (!std::isfinite(length) || length <= 0.0 ||
      length != subspace.translation_length_scale_m || weak < 0 || weak > 6 ||
      reliable < 0 || reliable > 6 || weak + reliable != 6) {
    result.status = "INVALID_SCALE_OR_DIMENSIONS";
    return result;
  }
  if (!finitePose(predicted_map_T_lidar) || !finitePose(raw_map_T_lidar)) {
    result.status = "INVALID_INPUT_POSE";
    return result;
  }
  const auto v = subspace.reliable_basis.leftCols(reliable);
  const auto w = subspace.weak_basis.leftCols(weak);
  if (!v.allFinite() || !w.allFinite() ||
      (v.transpose() * v - Eigen::MatrixXd::Identity(reliable, reliable)).norm() > 1e-8 ||
      (w.transpose() * w - Eigen::MatrixXd::Identity(weak, weak)).norm() > 1e-8 ||
      (w.transpose() * v).norm() > 1e-8) {
    result.status = "INVALID_JOINT_BASES";
    return result;
  }
  const Eigen::Matrix3d predicted_rotation = predicted_map_T_lidar.orientation.normalized().toRotationMatrix();
  const Eigen::Matrix3d raw_rotation = raw_map_T_lidar.orientation.normalized().toRotationMatrix();
  result.raw_correction.head<3>() = so3Log(raw_rotation * predicted_rotation.transpose());
  result.raw_correction.tail<3>() = (raw_map_T_lidar.position - predicted_map_T_lidar.position) / length;
  result.projector = v * v.transpose();
  // LIO-SAM LMOptimization: matX = matP * matX2.
  // X-ICP PointToPlane: project the normal solution, not a new Kalman update.
  result.safe_correction = result.projector * result.raw_correction;
  result.removed_correction = result.raw_correction - result.safe_correction;
  result.projector_symmetry_error = (result.projector.transpose() - result.projector).norm();
  result.projector_idempotence_error = (result.projector * result.projector - result.projector).norm();
  if (!result.raw_correction.allFinite() || !result.safe_correction.allFinite() ||
      !std::isfinite(result.projector_symmetry_error) ||
      !std::isfinite(result.projector_idempotence_error) ||
      result.projector_symmetry_error > 1e-8 || result.projector_idempotence_error > 1e-8) {
    result.status = "INVALID_REMAPPED_CORRECTION";
    return result;
  }
  result.remapped_map_T_lidar.orientation = Eigen::Quaterniond(
      expRotation(result.safe_correction.head<3>()) * predicted_rotation).normalized();
  result.remapped_map_T_lidar.position = predicted_map_T_lidar.position + length * result.safe_correction.tail<3>();
  if (!finitePose(result.remapped_map_T_lidar)) {
    result.status = "NONFINITE_REMAPPED_POSE";
    return result;
  }
  result.valid = true;
  result.lidar_measurement_available = reliable > 0;
  result.status = reliable > 0 ? "REMAPPED_MEASUREMENT_AVAILABLE" : "NO_RELIABLE_LIDAR_CORRECTION";
  return result;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
