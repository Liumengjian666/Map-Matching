#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/reliability_metrics.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp::reliability {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class UpdateAction {
  NORMAL_UPDATE,
  DIRECTIONAL_UPDATE,
  CAUTIOUS_UPDATE,
  PREDICTION_ONLY,
};

struct DualReliabilityConfig {
  // Dimensionless block-curvature ratio. A weak direction is used only when
  // the empirical score/gradient coordinate gate made LocalObservability valid.
  double weak_eigenvalue_ratio = 0.05;
  double gamma_translation = 10.0;
  double gamma_rotation = 10.0;
  double ndt_translation_scale_m = 0.8;

  // U_nonlocal's finite +/- probe response enters as an empirical covariance
  // increment. These are fixed first-version engineering gains and hard
  // eigenvalue caps, not probabilistic calibration parameters.
  double alpha_translation_response = 1.0;
  double alpha_rotation_response = 1.0;
  double max_nonlocal_translation_variance_m2 = 0.04;
  double max_nonlocal_rotation_variance_rad2 =
      0.0012184696791468343;  // (2 degrees)^2

  // P6-I5 operational same-terminal tolerance, used as a stability warning,
  // never as a basin/minimum classifier.
  double terminal_translation_limit_m = 0.20;
  double terminal_rotation_limit_rad = 2.0 * 3.14159265358979323846 / 180.0;

  // Trigger two extra prior-conditioned probes after M0 for a large normalized
  // innovation or periodically. Probe amplitude is in prior standard deviations.
  double innovation_chi_square_99pct_6d = 16.812;
  double absolute_translation_fallback_m = 0.5;
  double absolute_rotation_fallback_rad = 10.0 * 3.14159265358979323846 / 180.0;
  std::uint64_t periodic_probe_interval_scans = 25;
  double probe_prior_sigma = 1.0;

  // FULL-mode vision admission is deliberately stricter than feature count.
  // These are fixed engineering gates and are logged; none is selected from GT.
  int minimum_visual_inliers = 20;
  double minimum_visual_inlier_ratio = 0.20;
  double minimum_visual_grid_occupancy = 0.25;
  double minimum_visual_hull_fraction = 0.02;
  double minimum_visual_depth_fraction = 0.15;
  double minimum_visual_parallax_px = 0.25;
  double maximum_visual_reprojection_rmse_px = 2.0;
  double maximum_visual_pair_gap_s = 0.25;
  double maximum_visual_depth_age_s = 0.020;
  double visual_innovation_chi_square_99pct_3d = 11.345;
  double minimum_visual_information_eigenvalue = 1e-8;
  double minimum_visual_information_ratio = 1e-6;
  double maximum_visual_information_condition = 1e8;
  double relocalization_coast_limit_s = 2.0;
  double relocalization_position_sigma_limit_m = 5.0;
  double relocalization_rotation_sigma_limit_rad =
      30.0 * 3.14159265358979323846 / 180.0;
};

struct InnovationStatistic {
  bool valid = false;
  double mahalanobis_squared = 0.0;
  std::string status = "UNINITIALIZED";
};

struct ProbeTrigger {
  bool run_probes = false;
  std::string reason = "NOT_TRIGGERED";
  InnovationStatistic innovation;
};

// Product tangent order is [map-spatial rotation, map translation], matching
// P6-I4 covariance projection and its R_perturbed*R_nominal^T convention.
Vector6d mapProductInnovation(const Eigen::Isometry3d& predicted,
                              const Eigen::Isometry3d& measured);
InnovationStatistic normalizedInnovation(
    const Vector6d& map_product_innovation,
    const Matrix6d& map_product_prediction_covariance);
ProbeTrigger shouldRunNonlocalProbes(
    std::uint64_t transaction_id, const Vector6d& map_product_innovation,
    const Matrix6d& map_product_prediction_covariance,
    const DualReliabilityConfig& config = {});

struct LocalRisk {
  bool valid = false;
  bool translation_weak = false;
  bool rotation_weak = false;
  double translation_weak_ratio = 0.0;
  double rotation_weak_ratio = 0.0;
  double translation_min_eigenvalue = 0.0;
  double rotation_min_eigenvalue = 0.0;
  double translation_block_condition = std::numeric_limits<double>::infinity();
  double rotation_block_condition = std::numeric_limits<double>::infinity();
  Eigen::Vector3d map_translation_weak_direction = Eigen::Vector3d::Zero();
  Eigen::Vector3d map_rotation_weak_direction = Eigen::Vector3d::Zero();
  // Joint LiDAR weak/reliable bases in normalized physical coordinates
  // [map-spatial rotation, translation / ndt_translation_scale_m]. Columns
  // [0, weak_dimension) and [weak_dimension, 6) are active respectively.
  Matrix6d joint_weak_basis = Matrix6d::Zero();
  Matrix6d joint_reliable_basis = Matrix6d::Zero();
  int weak_dimension = 0;
  int reliable_dimension = 0;
  Eigen::Vector3d rotation_schur_eigenvalues = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d translation_schur_eigenvalues = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  bool schur_decoupling_valid = false;
  bool map_support_sufficient = false;
  std::uint64_t map_support_correspondences = 0;
  double map_support_effective_weight = 0.0;
  double translation_length_scale_m = 0.0;
  std::string status = "UNAVAILABLE";
};

LocalRisk assessLocalRisk(const LocalObservability& local,
                          const DualReliabilityConfig& config = {});

struct MeasurementNoiseResult {
  bool valid = false;
  // PoseMeasurement is ordered [position XYZ, SO(3) residual]. The rotation
  // block is in IKFoM's right/body error coordinates, not map-spatial axes.
  Matrix6d covariance = Matrix6d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d local_translation_increment = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d local_rotation_map_increment = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d nonlocal_translation_increment = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d nonlocal_rotation_map_increment = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rotation_map_covariance = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  std::string status = "UNINITIALIZED";
};

MeasurementNoiseResult makePoseMeasurementNoise(
    double position_sigma_m, double rotation_sigma_rad,
    const Eigen::Matrix3d& predicted_map_R_imu,
    const LocalRisk& local_risk,
    const NonlocalTerminalStability& nonlocal,
    bool use_uobs, bool use_unonlocal,
    const DualReliabilityConfig& config = {});

struct DualReliabilityDecision {
  UpdateAction action = UpdateAction::PREDICTION_ONLY;
  bool local_risk = false;
  bool nonlocal_risk = false;
  bool local_curvature_used = false;
  bool nonlocal_response_used = false;
  std::string reason = "UNINITIALIZED";
  MeasurementNoiseResult measurement_noise;
};

DualReliabilityDecision decideDualReliability(
    const LocalRisk& local_risk,
    const NonlocalTerminalStability& nonlocal,
    bool nominal_ndt_converged,
    const Eigen::Matrix3d& predicted_map_R_imu,
    double position_sigma_m, double rotation_sigma_rad,
    bool use_uobs, bool use_unonlocal,
    const DualReliabilityConfig& config = {});

const char* toString(UpdateAction action);

struct VisualQualityObservation {
  bool source_valid = false;
  bool quality_metadata_available = false;
  std::uint64_t reference_stamp_ns = 0;
  std::uint64_t current_stamp_ns = 0;
  std::uint64_t depth_stamp_ns = 0;
  int detected_count = 0;
  int tracked_count = 0;
  int depth_associated_count = 0;
  int pnp_inlier_count = 0;
  double inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  double depth_fraction = std::numeric_limits<double>::quiet_NaN();
  double grid_occupancy = std::numeric_limits<double>::quiet_NaN();
  double hull_fraction = std::numeric_limits<double>::quiet_NaN();
  double median_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double reprojection_rmse_px = std::numeric_limits<double>::quiet_NaN();
  double innovation_chi_square = std::numeric_limits<double>::quiet_NaN();
};

struct VisualQualityDecision {
  bool passed = false;
  bool timestamp_valid = false;
  bool prediction_consistent = false;
  std::string rejection_reason = "UNINITIALIZED";
};

VisualQualityDecision assessVisualQuality(
    const VisualQualityObservation& observation,
    const DualReliabilityConfig& config = {});

struct VisualSubspaceDecision {
  bool valid = false;
  bool complementary = false;
  std::string status = "UNINITIALIZED";
  int lidar_weak_dimension = 0;
  int effective_rank = 0;
  int measurement_rank = 0;
  Eigen::Matrix<double, 6, 1> projected_eigenvalues =
      Eigen::Matrix<double, 6, 1>::Zero();
  Matrix6d visual_information = Matrix6d::Zero();
  Matrix6d projected_weak_information = Matrix6d::Zero();
  Matrix6d complementary_weak_basis = Matrix6d::Zero();
  Eigen::Matrix3d measurement_basis = Eigen::Matrix3d::Zero();
  double effective_condition = std::numeric_limits<double>::infinity();
};

// The position factor is mapped into the same [map-spatial rotation,
// scaled map translation] coordinates as geometric U_obs. The lever-arm
// Jacobian retains rotation/translation coupling instead of comparing
// unrelated body/map axes.
VisualSubspaceDecision assessVisualWeakSubspaceInformation(
    const LocalRisk& lidar, const Eigen::Vector3d& map_imu_origin_minus_lidar,
    const Eigen::Matrix3d& map_position_measurement_covariance,
    const DualReliabilityConfig& config = {});

enum class ConditionalFusionMode {
  NORMAL_LIDAR,
  LIDAR_DEGRADED_VISION_VALID,
  LIDAR_DEGRADED_VISION_INVALID,
  IMU_COASTING,
  RELOCALIZATION_REQUIRED,
};

struct ConditionalRouteInput {
  bool lidar_converged = false;
  bool lidar_registration_unstable = false;
  bool map_support_sufficient = false;
  int lidar_weak_dimension = 0;
  int lidar_reliable_dimension = 0;
  bool vision_trigger_requested = false;
  bool vision_observation_available = false;
  bool vision_quality_passed = false;
  bool vision_complementary = false;
  int vision_complementary_rank = 0;
  double time_without_external_constraint_s = 0.0;
  double maximum_position_sigma_m = 0.0;
  double maximum_rotation_sigma_rad = 0.0;
};

struct ConditionalRouteDecision {
  ConditionalFusionMode mode = ConditionalFusionMode::IMU_COASTING;
  bool apply_lidar_measurement = false;
  bool apply_visual_measurement = false;
  bool preserve_lidar_reliable_subspace = false;
  bool imu_coasting = true;
  bool relocalization_required = false;
  std::string reason = "UNINITIALIZED";
};

ConditionalRouteDecision routeConditionalCompensation(
    const ConditionalRouteInput& input,
    const DualReliabilityConfig& config = {});
const char* toString(ConditionalFusionMode mode);

// Timestamped two-view epipolar factors produced by the offline LK frontend.
// Jacobian columns are [camera-relative translation XYZ (m), left camera
// relative rotation XYZ (rad)]. These are not absolute-pose observations and
// cannot be fused into IKFoM until mapped through the state-history Jacobian.
struct VisualGeometryFactor {
  std::uint64_t reference_stamp_ns = 0;
  std::uint64_t current_stamp_ns = 0;
  std::size_t tracked_count = 0;
  std::size_t inlier_count = 0;
  double robust_residual_rms = std::numeric_limits<double>::quiet_NaN();
  Eigen::VectorXd residuals;
  Eigen::Matrix<double, Eigen::Dynamic, 6> relative_pose_jacobian;
  std::string status = "UNAVAILABLE";
};

// Vision is enabled only when real timestamped factors exist. The first I6C
// release intentionally keeps fusion disabled: a camera-relative factor is
// not itself a metric map-position measurement.
struct VisionAssistInterface {
  bool enabled = false;
  bool fusion_enabled = false;
  bool trigger_requested = false;
  bool observation_available = false;
  std::uint64_t latest_stamp_ns = 0;
  std::vector<VisualGeometryFactor> factors;
  std::string trigger_reason = "NOT_REQUESTED";
  std::string status = "DISABLED_NO_STATE_HISTORY_JACOBIAN";
};

}  // namespace dog_prior_map_fastlio2_frontend_exp::reliability
