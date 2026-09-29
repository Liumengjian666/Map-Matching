#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace reliability = dog_prior_map_fastlio2_frontend_exp::reliability;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

reliability::LocalRisk weakTranslationX() {
  reliability::LocalRisk risk;
  risk.valid = true;
  risk.map_support_sufficient = true;
  risk.weak_dimension = 1;
  risk.reliable_dimension = 5;
  risk.translation_length_scale_m = 0.8;
  risk.joint_weak_basis.col(0)(3) = 1.0;
  return risk;
}

reliability::VisualQualityObservation goodVision() {
  reliability::VisualQualityObservation observation;
  observation.source_valid = true;
  observation.quality_metadata_available = true;
  observation.reference_stamp_ns = 1'000'000'000ULL;
  observation.current_stamp_ns = 1'050'000'000ULL;
  observation.depth_stamp_ns = 995'000'000ULL;
  observation.detected_count = 100;
  observation.tracked_count = 60;
  observation.depth_associated_count = 40;
  observation.pnp_inlier_count = 30;
  observation.inlier_ratio = 0.75;
  observation.depth_fraction = 40.0 / 60.0;
  observation.grid_occupancy = 0.5;
  observation.hull_fraction = 0.3;
  observation.median_parallax_px = 2.0;
  observation.reprojection_rmse_px = 0.5;
  observation.innovation_chi_square = 1.0;
  return observation;
}

void testSchurAwareJointAndVisualCoordinates() {
  std::vector<reliability::GeometricObservation> observations;
  for (int i = 0; i < 32; ++i) {
    reliability::GeometricObservation observation;
    observation.rotated_source_map = Eigen::Vector3d(
        0.2 * i, std::sin(0.4 * i), std::cos(0.31 * i));
    observation.voxel_covariance_map =
        Eigen::Vector3d(0.02, 0.04, 0.08).asDiagonal();
    observation.nonnegative_weight = 1.0 + 0.1 * i;
    observations.push_back(observation);
  }
  const auto local = reliability::analyzeGeometricObservability(
      observations, true, 0.8);
  require(local.valid && local.schur_decoupling_valid &&
              local.map_support_sufficient && local.joint_eigenvalues.allFinite(),
          "geometric U_obs exposes joint spectrum and Schur diagnostics");
  require((local.normalized_geometric_information.block<3, 3>(0, 3)).norm() > 1e-6,
          "test geometry includes rotation-translation coupling");
  require((local.rotation_schur_information -
           local.normalized_geometric_information.block<3, 3>(0, 0)).norm() > 1e-6,
          "rotation Schur complement removes translation nuisance coupling");

  const auto risk = reliability::assessLocalRisk(local);
  require(risk.valid && risk.weak_dimension > 0 &&
              risk.reliable_dimension + risk.weak_dimension == 6,
          "joint eigenbasis separates weak and reliable LiDAR subspaces");

  const Eigen::Matrix3d visual_noise = 0.01 * Eigen::Matrix3d::Identity();
  reliability::LocalRisk translation_x = weakTranslationX();
  const auto complementary = reliability::assessVisualWeakSubspaceInformation(
      translation_x, Eigen::Vector3d(0.1, -0.05, 0.03), visual_noise);
  require(complementary.valid && complementary.complementary &&
              complementary.effective_rank == 1 &&
              complementary.measurement_rank == 1,
          "metric position factor supplies one stable LiDAR-weak direction");

  reliability::LocalRisk rotation_x = translation_x;
  rotation_x.joint_weak_basis.setZero();
  rotation_x.joint_weak_basis(0, 0) = 1.0;
  const auto not_complementary = reliability::assessVisualWeakSubspaceInformation(
      rotation_x, Eigen::Vector3d::Zero(), visual_noise);
  require(not_complementary.valid && !not_complementary.complementary &&
              not_complementary.effective_rank == 0 &&
              not_complementary.status == "NO_INFORMATION_IN_LIDAR_WEAK_SUBSPACE",
          "valid position observation cannot repair unobserved pure rotation");
}

void testQualityAndAblationScenariosAtoE() {
  const reliability::DualReliabilityConfig config;
  auto quality_observation = goodVision();
  const auto quality = reliability::assessVisualQuality(quality_observation, config);
  require(quality.passed && quality.timestamp_valid && quality.prediction_consistent,
          "scenario B visual passes quality, time, and innovation checks");
  auto legacy_quality = quality_observation;
  legacy_quality.quality_metadata_available = false;
  require(reliability::assessVisualQuality(legacy_quality, config).rejection_reason ==
              "QUALITY_METADATA_UNAVAILABLE",
          "legacy feature-count-only input cannot bypass I6E quality metadata gate");
  auto inconsistent = quality_observation;
  inconsistent.innovation_chi_square = 100.0;
  require(reliability::assessVisualQuality(inconsistent, config).rejection_reason ==
              "VISUAL_PREDICTION_INCONSISTENT",
          "visual innovation must agree with predicted state");

  // A: normal LiDAR is retained and vision is not allowed to perturb it.
  reliability::ConditionalRouteInput input;
  input.lidar_converged = true;
  input.map_support_sufficient = true;
  input.lidar_reliable_dimension = 6;
  const auto normal = reliability::routeConditionalCompensation(input, config);
  require(normal.mode == reliability::ConditionalFusionMode::NORMAL_LIDAR &&
              normal.apply_lidar_measurement && !normal.apply_visual_measurement,
          "scenario A routes to normal LiDAR update");

  // B: only a measured visual direction overlapping the LiDAR weak subspace is fused.
  input.lidar_registration_unstable = true;
  input.lidar_weak_dimension = 1;
  input.lidar_reliable_dimension = 5;
  input.vision_trigger_requested = true;
  input.vision_observation_available = true;
  input.vision_quality_passed = true;
  input.vision_complementary = true;
  input.vision_complementary_rank = 1;
  const auto vision_compensated = reliability::routeConditionalCompensation(input, config);
  require(vision_compensated.mode ==
              reliability::ConditionalFusionMode::LIDAR_DEGRADED_VISION_VALID &&
              vision_compensated.apply_lidar_measurement &&
              vision_compensated.apply_visual_measurement &&
              !vision_compensated.imu_coasting,
          "scenario B preserves LiDAR reliable subspace and fuses complementary vision");

  // C: image-level quality passes, but its information has no component in the weak direction.
  input.vision_complementary = false;
  input.vision_complementary_rank = 0;
  const auto noncomplementary = reliability::routeConditionalCompensation(input, config);
  require(noncomplementary.mode ==
              reliability::ConditionalFusionMode::LIDAR_DEGRADED_VISION_INVALID &&
              noncomplementary.apply_lidar_measurement &&
              !noncomplementary.apply_visual_measurement &&
              noncomplementary.imu_coasting,
          "scenario C rejects valid but noncomplementary vision");

  // D: a visual quality failure is rejected, not force-fed to the filter.
  input.vision_quality_passed = false;
  const auto rejected = reliability::routeConditionalCompensation(input, config);
  require(rejected.mode ==
              reliability::ConditionalFusionMode::LIDAR_DEGRADED_VISION_INVALID &&
              !rejected.apply_visual_measurement && rejected.imu_coasting,
          "scenario D rejects low-quality visual observation");
  auto poor = goodVision();
  poor.grid_occupancy = 0.05;
  require(reliability::assessVisualQuality(poor, config).rejection_reason ==
              "POOR_FEATURE_SPATIAL_DISTRIBUTION",
          "feature distribution is an explicit independent gate");

  // E: sensor outage coasts with propagated uncertainty; recovery returns to LiDAR mode.
  reliability::ConditionalRouteInput outage;
  outage.time_without_external_constraint_s = 0.5;
  const auto coasting = reliability::routeConditionalCompensation(outage, config);
  require(coasting.mode == reliability::ConditionalFusionMode::IMU_COASTING &&
              coasting.imu_coasting && !coasting.apply_lidar_measurement &&
              !coasting.apply_visual_measurement,
          "scenario E short outage is IMU-only prediction");
  outage.time_without_external_constraint_s = 2.1;
  outage.maximum_position_sigma_m = 5.1;
  const auto relocation = reliability::routeConditionalCompensation(outage, config);
  require(relocation.mode ==
              reliability::ConditionalFusionMode::RELOCALIZATION_REQUIRED &&
              relocation.relocalization_required,
          "prolonged or uncertain coasting requests relocalization");
  outage.lidar_converged = true;
  outage.map_support_sufficient = true;
  outage.lidar_registration_unstable = false;
  outage.lidar_weak_dimension = 0;
  outage.lidar_reliable_dimension = 6;
  outage.time_without_external_constraint_s = 0.0;
  outage.maximum_position_sigma_m = 0.1;
  const auto recovered = reliability::routeConditionalCompensation(outage, config);
  require(recovered.mode == reliability::ConditionalFusionMode::NORMAL_LIDAR &&
              recovered.apply_lidar_measurement,
          "recovered LiDAR constraint returns to normal route");
}

}  // namespace

int main() {
  try {
    testSchurAwareJointAndVisualCoordinates();
    testQualityAndAblationScenariosAtoE();
    std::cout << "PAPER_P6_I6E_CONDITIONAL_COMPENSATION_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PAPER_P6_I6E_CONDITIONAL_COMPENSATION_TEST_FAIL: "
              << error.what() << '\n';
    return 1;
  }
}
