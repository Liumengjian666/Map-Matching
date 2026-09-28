#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp::reliability {

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
  bool objectives_finite = false;
  std::string status = "UNINITIALIZED";
  TerminalCapture nominal;
  TerminalCapture positive;
  TerminalCapture negative;
  double positive_negative_translation_gap_m =
      std::numeric_limits<double>::quiet_NaN();
  double positive_negative_rotation_gap_rad =
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
    const TerminalCapture& negative, std::uint64_t extra_ndt_calls);

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
