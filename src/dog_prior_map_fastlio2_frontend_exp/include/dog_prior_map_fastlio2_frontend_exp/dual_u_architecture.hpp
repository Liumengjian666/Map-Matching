#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp {

using DualUMatrix6d = Eigen::Matrix<double, 6, 6>;
using DualUVector6d = Eigen::Matrix<double, 6, 1>;

// PCL's parameter order is [tx,ty,tz,rx,ry,rz], with Rx*Ry*Rz angles.
// The reported U_obs chart is [map translation / L, map-spatial rotation]
// about the LiDAR origin. It is a product chart, not an SE(3) left twist.
struct ProductChartPullback {
  bool valid = false;
  std::string status = "UNINITIALIZED";
  DualUMatrix6d dp_deta = DualUMatrix6d::Zero();
  // One 6x6 Hessian for each PCL parameter p_i.
  std::vector<DualUMatrix6d> d2p_deta2;
  double euler_chart_condition = std::numeric_limits<double>::infinity();
  double base_rotation_reconstruction_error = std::numeric_limits<double>::infinity();
};

struct WithinBasinObservability {
  bool valid = false;
  bool locally_convex = false;
  std::string status = "UNINITIALIZED";
  std::string chart = "MAP_TRANSLATION_OVER_L_THEN_MAP_SPATIAL_ROTATION_LIDAR_ORIGIN";
  double length_scale_m = std::numeric_limits<double>::quiet_NaN();
  double objective_per_source = std::numeric_limits<double>::quiet_NaN();
  double score_hessian_relative_asymmetry = std::numeric_limits<double>::infinity();
  double pulled_hessian_relative_asymmetry = std::numeric_limits<double>::infinity();
  double euler_chart_condition = std::numeric_limits<double>::infinity();
  DualUVector6d objective_gradient = DualUVector6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  DualUMatrix6d local_curvature = DualUMatrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  // Ascending eigenvalues and corresponding columns; no weak-ratio threshold
  // or calibrated covariance is implied by this spectrum.
  DualUVector6d curvature_eigenvalues = DualUVector6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  DualUMatrix6d curvature_eigenvectors = DualUMatrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
};

bool buildMapProductChartPullback(const Pose3d& map_T_lidar,
    const Eigen::Vector3d& pcl_euler_xyz, double length_scale_m,
    ProductChartPullback* output, std::string* reason);

bool analyzeWithinBasinObservability(const PclNdtScoreJet& score_jet,
    const Pose3d& map_T_lidar, const Eigen::Vector3d& actual_target_grid_leaf_m,
    double configured_resolution_m, WithinBasinObservability* output,
    std::string* reason);

// Apply eta=[delta_t_map/L, delta_theta_map] using additive map translation
// and a left/map-spatial rotation perturbation, both about the LiDAR origin.
Pose3d applyMapProductChartIncrement(const Pose3d& map_T_lidar,
    const DualUVector6d& eta, double length_scale_m);

// Coordinate transport for the same physical perturbation from a LiDAR-origin
// chart to the IMU-origin chart. r_map points from the IMU origin to the LiDAR
// origin. For this product chart: eta_imu = G * eta_lidar.
DualUMatrix6d lidarOriginToImuOriginTangentMap(
    const Eigen::Vector3d& r_map_imu_to_lidar, double length_scale_m);

struct NdtObjectiveProvenance {
  std::string backend = "PCL_1.10_NDT";
  std::string map_sha256;
  std::uint64_t source_cloud_hash = 0;
  std::uint64_t source_point_count = 0;
  std::uint64_t target_point_count = 0;
  double configured_resolution_m = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d actual_target_grid_leaf_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double step_size = std::numeric_limits<double>::quiet_NaN();
  double transformation_epsilon = std::numeric_limits<double>::quiet_NaN();
  int maximum_iterations = 0;
};

struct BasinSupportEvidence {
  int converged_seed_count = 0;
  // Set by the mature terminal-clustering producer when the selected nominal
  // NDT terminal belongs to this complete-link cluster.
  bool contains_selected_terminal = false;
};

enum class NonlocalStatus {
  INDETERMINATE,
  POSSIBLY_UNREPRESENTED,
  SINGLE_REPRESENTED,
  MULTI_REPRESENTED,
};

const char* nonlocalStatusName(NonlocalStatus status);

struct NonlocalSearchEvidence {
  NdtObjectiveProvenance provenance;
  std::string finite_seed_domain;
  int planned_seed_count = 0;
  int attempted_seed_count = 0;
  int converged_seed_count = 0;
  // Includes both supported and sub-threshold complete-link clusters.
  std::vector<BasinSupportEvidence> represented_clusters;
};

struct NonlocalReliability {
  bool valid_for_objective = false;
  bool finite_seed_domain_exhausted = false;
  // Structurally false for this finite heuristic multi-start search. There is
  // no caller-controlled switch that can claim global completeness.
  bool exact_global_completeness_proven = false;
  bool unrepresented_basin_possible = true;
  bool search_coverage_incomplete = true;
  NonlocalStatus status = NonlocalStatus::INDETERMINATE;
  std::string diagnostic = "UNINITIALIZED";
  int attempted_seed_count = 0;
  int planned_seed_count = 0;
  int converged_seed_count = 0;
  int represented_cluster_count = 0;
  int supported_cluster_count = 0;
  int subthreshold_cluster_count = 0;
  int selected_terminal_cluster_index = -1;
  bool selected_basin_supported = false;
  std::vector<int> cluster_support_converged;
  std::vector<double> cluster_fraction_of_converged;
  std::vector<double> cluster_fraction_of_attempted;
};

NonlocalReliability classifyCandidateConditionedNonlocalEvidence(
    const NdtObjectiveProvenance& selected_objective,
    const NonlocalSearchEvidence& evidence,
    int minimum_cluster_seeds = 5,
    double minimum_fraction_of_converged = 0.02);

struct DualUReliabilityResult {
  std::uint64_t transaction_id = 0;
  std::uint64_t stamp_ns = 0;
  Pose3d selected_raw_ndt_map_T_lidar;
  WithinBasinObservability u_obs;
  NonlocalReliability u_nonlocal;
  std::string status = "SHADOW_UNINITIALIZED";
  // This interface is diagnostic only; it contains no estimator/filter handle.
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
