#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp::reliability {

using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// P6-I3's PCL Euler score-Hessian conversion, kept explicit so the closed-loop
// U_obs path and its tests share the same coordinate convention. PCL input
// order is [tx,ty,tz,rx,ry,rz]; normalized output order is
// [map-spatial rotation, scaled translation].
struct PclCurvatureTransform {
  bool valid = false;
  std::string status = "UNINITIALIZED";
  Matrix6d hessian_euler = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Matrix6d hessian_physical = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Matrix6d normalized_negative_score_curvature = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Matrix3d euler_to_map_spatial_jacobian = Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double jacobian_condition = std::numeric_limits<double>::infinity();
};

PclCurvatureTransform transformPclScoreHessianToNormalizedMapTangent(
    const Matrix6d& pcl_score_hessian,
    const Eigen::Vector3d& pcl_rx_ry_rz,
    double translation_scale_m);

// BLOCK-curvature summary matching the P6-I3 convention. The input is the
// dimensionless, normalized negative-score curvature Hbar in the ordered
// coordinates [map-spatial rotation, scaled translation]. These are decoupled
// diagonal blocks, not a Schur complement, calibrated covariance, or a
// dataset-specific decision.
struct LocalObservability {
  bool valid = false;
  bool ndt_converged = false;
  bool score_gradient_coordinate_check_passed = false;
  std::string status = "UNINITIALIZED";
  Eigen::Vector3d rotation_block_eigenvalues = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d rotation_block_eigenvectors = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d translation_block_eigenvalues = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d translation_block_eigenvectors = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double rotation_block_condition = std::numeric_limits<double>::infinity();
  double translation_block_condition = std::numeric_limits<double>::infinity();
};

LocalObservability analyzeLocalObservability(
    const Eigen::Matrix<double, 6, 6>& normalized_negative_score_curvature,
    bool ndt_converged, bool score_gradient_coordinate_check_passed);

struct TerminalCapture {
  Eigen::Isometry3d map_T_lidar = Eigen::Isometry3d::Identity();
  double fixed_objective = std::numeric_limits<double>::quiet_NaN();
  bool converged = false;
  int iterations = -1;
};

// A descriptive record of one nominal and a +/- perturbed pair. It records
// terminal stability only; it does not classify basins or claim minima.
struct NonlocalTerminalStability {
  bool geometry_valid = false;
  bool imu_lidar_extrinsic_valid = false;
  bool objectives_finite = false;
  bool all_converged = false;
  std::string status = "UNINITIALIZED";
  TerminalCapture nominal;
  TerminalCapture positive;
  TerminalCapture negative;
  // Position responses are at the IMU origin in map axes, matching the
  // position component consumed by the IKFoM measurement update. The raw
  // terminal captures above remain map_T_lidar. Rotation responses are
  // map-spatial. Outer products describe only this finite +/- probe pair, not
  // calibrated noise.
  Eigen::Vector3d delta_position_imu_positive = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d delta_position_imu_negative = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d delta_rotation_positive = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d delta_rotation_negative = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  bool response_valid = false;
  double positive_negative_translation_gap_m =
      std::numeric_limits<double>::quiet_NaN();
  double positive_negative_rotation_gap_rad =
      std::numeric_limits<double>::quiet_NaN();
  double max_nominal_translation_delta_m =
      std::numeric_limits<double>::quiet_NaN();
  double max_nominal_rotation_delta_rad =
      std::numeric_limits<double>::quiet_NaN();
  double positive_minus_nominal_objective =
      std::numeric_limits<double>::quiet_NaN();
  double negative_minus_nominal_objective =
      std::numeric_limits<double>::quiet_NaN();
  double positive_minus_negative_objective =
      std::numeric_limits<double>::quiet_NaN();
  std::uint64_t extra_ndt_calls = 0;
};

NonlocalTerminalStability analyzeNonlocalTerminalStability(
    const TerminalCapture& nominal, const TerminalCapture& positive,
    const TerminalCapture& negative,
    const Eigen::Isometry3d& imu_T_lidar,
    std::uint64_t extra_ndt_calls);

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
