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
  require(risk.valid &&
              risk.reliable_dimension + risk.weak_dimension == 6,
          "joint eigenbasis has a complete rank partition");
  const Eigen::MatrixXd weak = risk.joint_weak_basis.leftCols(risk.weak_dimension);
  const Eigen::MatrixXd reliable = risk.joint_reliable_basis.leftCols(
      risk.reliable_dimension);
  require((weak.transpose() * weak - Eigen::MatrixXd::Identity(
              risk.weak_dimension, risk.weak_dimension)).norm() < 1e-8 &&
              (reliable.transpose() * reliable - Eigen::MatrixXd::Identity(
                  risk.reliable_dimension, risk.reliable_dimension)).norm() < 1e-8 &&
              (weak.transpose() * reliable).norm() < 1e-8,
          "mapped weak and reliable normalized-coordinate bases are orthogonal");
  const Eigen::Matrix3d a = local.normalized_geometric_information.block<3, 3>(0, 0);
  const Eigen::Matrix3d b = local.normalized_geometric_information.block<3, 3>(0, 3);
  const Eigen::Matrix3d c = local.normalized_geometric_information.block<3, 3>(3, 3);
  require((local.rotation_schur_information -
           (a - b * c.inverse() * b.transpose())).norm() < 1e-8 &&
              (local.translation_schur_information -
               (c - b.transpose() * a.inverse() * b)).norm() < 1e-8,
          "Schur matrices use the prescribed cross-block order");

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

void testSchurScaleBalancedClassification() {
  const auto make_local = [](const reliability::Matrix6d& information) {
    reliability::LocalObservability local;
    local.valid = true;
    local.geometric_proxy = true;
    local.map_support_sufficient = true;
    local.map_support_status = "MAP_SUPPORT_SUFFICIENT";
    local.valid_correspondence_count = 30;
    local.effective_weight_sum = 30.0;
    local.translation_length_scale_m = 0.8;
    local.normalized_geometric_information = information;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rot_block(
        information.block<3, 3>(0, 0));
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> trans_block(
        information.block<3, 3>(3, 3));
    local.rotation_block_eigenvalues = rot_block.eigenvalues();
    local.rotation_block_eigenvectors = rot_block.eigenvectors();
    local.translation_block_eigenvalues = trans_block.eigenvalues();
    local.translation_block_eigenvectors = trans_block.eigenvectors();
    const Eigen::Matrix3d a = information.block<3, 3>(0, 0);
    const Eigen::Matrix3d b = information.block<3, 3>(0, 3);
    const Eigen::Matrix3d c = information.block<3, 3>(3, 3);
    local.rotation_schur_information = a - b * c.inverse() * b.transpose();
    local.translation_schur_information = c - b.transpose() * a.inverse() * b;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rot_schur(
        local.rotation_schur_information);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> trans_schur(
        local.translation_schur_information);
    local.rotation_schur_eigenvalues = rot_schur.eigenvalues().cwiseMax(0.0);
    local.translation_schur_eigenvalues = trans_schur.eigenvalues().cwiseMax(0.0);
    local.schur_decoupling_valid = true;
    return local;
  };

  reliability::Matrix6d first = reliability::Matrix6d::Zero();
  first.block<3, 3>(0, 0) = Eigen::Vector3d(400, 500, 600).asDiagonal();
  first.block<3, 3>(3, 3) = Eigen::Vector3d(4, 5, 6).asDiagonal();
  const auto scale_only = reliability::assessLocalRisk(make_local(first));
  std::cout << "scale_only_weak=" << scale_only.weak_dimension
            << " scale_only_reliable=" << scale_only.reliable_dimension
            << " rotation_ratio=" << scale_only.rotation_weak_ratio
            << " translation_ratio=" << scale_only.translation_weak_ratio
            << '\n';
  require(scale_only.valid && scale_only.weak_dimension == 0 &&
              scale_only.reliable_dimension == 6,
          "100x group scale difference does not erase an otherwise reliable translation block");

  reliability::Matrix6d second = reliability::Matrix6d::Zero();
  second.block<3, 3>(0, 0) = Eigen::Vector3d(400, 500, 600).asDiagonal();
  second.block<3, 3>(3, 3) = Eigen::Vector3d(0.01, 5, 6).asDiagonal();
  const auto weak_translation = reliability::assessLocalRisk(make_local(second));
  std::cout << "true_translation_weak=" << weak_translation.weak_dimension
            << " true_translation_reliable=" << weak_translation.reliable_dimension
            << " rotation_ratio=" << weak_translation.rotation_weak_ratio
            << " translation_ratio=" << weak_translation.translation_weak_ratio
            << '\n';
  require(weak_translation.valid && weak_translation.weak_dimension == 1 &&
              weak_translation.reliable_dimension == 5 &&
              std::abs(weak_translation.joint_weak_basis(3, 0)) > 1.0 - 1e-8,
          "a true within-translation weak direction remains detectable after scale balancing");

  Eigen::Matrix<double, 6, 6> mixing = Eigen::Matrix<double, 6, 6>::Identity();
  constexpr double angle = 0.37;
  for (int axis = 0; axis < 3; ++axis) {
    const int trans_axis = axis + 3;
    mixing(axis, axis) = std::cos(angle);
    mixing(trans_axis, trans_axis) = std::cos(angle);
    mixing(axis, trans_axis) = -std::sin(angle);
    mixing(trans_axis, axis) = std::sin(angle);
  }
  Eigen::Matrix<double, 6, 1> eigenvalues;
  eigenvalues << 0.002, 0.3, 0.5, 0.8, 1.0, 1.2;
  const reliability::Matrix6d cross_information =
      mixing * eigenvalues.asDiagonal() * mixing.transpose();
  const auto cross_local = make_local(cross_information);
  const auto cross_risk = reliability::assessLocalRisk(cross_local);
  const Eigen::Matrix3d a = cross_information.block<3, 3>(0, 0);
  const Eigen::Matrix3d b = cross_information.block<3, 3>(0, 3);
  const Eigen::Matrix3d c = cross_information.block<3, 3>(3, 3);
  const double rotation_schur_error = (cross_local.rotation_schur_information -
      (a - b * c.inverse() * b.transpose())).norm();
  const double translation_schur_error = (cross_local.translation_schur_information -
      (c - b.transpose() * a.inverse() * b)).norm();
  std::cout << "cross_coupled_weak=" << cross_risk.weak_dimension
            << " cross_coupled_reliable=" << cross_risk.reliable_dimension
            << " rotation_schur_formula_error=" << rotation_schur_error
            << " translation_schur_formula_error=" << translation_schur_error
            << '\n';
  require(cross_risk.valid && cross_local.schur_decoupling_valid &&
              rotation_schur_error < 1e-8 && translation_schur_error < 1e-8 &&
              cross_risk.weak_dimension + cross_risk.reliable_dimension == 6,
          "PSD cross-coupled information retains exact Schur order and complete partition");
  const Eigen::MatrixXd cross_weak = cross_risk.joint_weak_basis.leftCols(
      cross_risk.weak_dimension);
  const Eigen::MatrixXd cross_reliable = cross_risk.joint_reliable_basis.leftCols(
      cross_risk.reliable_dimension);
  require((cross_weak.transpose() * cross_weak - Eigen::MatrixXd::Identity(
              cross_risk.weak_dimension, cross_risk.weak_dimension)).norm() < 1e-8 &&
              (cross_reliable.transpose() * cross_reliable - Eigen::MatrixXd::Identity(
                  cross_risk.reliable_dimension, cross_risk.reliable_dimension)).norm() < 1e-8 &&
              (cross_weak.transpose() * cross_reliable).norm() < 1e-8,
          "cross-coupled mapped weak/reliable bases remain orthonormal");
}

void testCoupledWeakDirectionAnnihilation() {
  const Eigen::Matrix3d map_R_imu = Eigen::AngleAxisd(
      0.63, Eigen::Vector3d(0.2, -0.3, 0.9).normalized()).toRotationMatrix();
  const Eigen::Vector3d imu_T_lidar(0.14, -0.06, 0.09);
  const Eigen::Vector3d map_imu_origin_minus_lidar = -(map_R_imu * imu_T_lidar);
  Eigen::Matrix3d lever_skew;
  lever_skew << 0.0, -map_imu_origin_minus_lidar.z(), map_imu_origin_minus_lidar.y(),
      map_imu_origin_minus_lidar.z(), 0.0, -map_imu_origin_minus_lidar.x(),
      -map_imu_origin_minus_lidar.y(), map_imu_origin_minus_lidar.x(), 0.0;
  reliability::Matrix6d transform = reliability::Matrix6d::Zero();
  transform.block<3, 3>(0, 0) = -lever_skew;
  transform.block<3, 3>(0, 3) = 0.8 * Eigen::Matrix3d::Identity();
  transform.block<3, 3>(3, 0) = map_R_imu.transpose();
  require((transform.transpose() * transform - reliability::Matrix6d::Identity()).norm() > 1e-2,
          "test measurement transform is deliberately non-orthogonal");

  Eigen::Matrix<double, 6, 1> weak;
  weak << 0.2, -0.3, 0.4, 0.5, -0.1, 0.2;
  weak.normalize();
  reliability::Matrix6d weak_basis = reliability::Matrix6d::Zero();
  weak_basis.col(0) = weak;
  reliability::Matrix6d measurement_basis;
  int rank = 0;
  std::string failure;
  require(reliability::buildReliableMeasurementBasisFromWeak(
              transform, weak_basis, 1, &measurement_basis, &rank, &failure) &&
              rank == 5,
          "mixed rotation/translation weak direction produces rank-five orthogonal complement");
  const Eigen::MatrixXd reliable = measurement_basis.leftCols(rank);
  const Eigen::MatrixXd mapped_weak = transform * weak_basis.leftCols(1);
  const double leakage = (reliable.transpose() * mapped_weak).norm() /
      (1.0 + mapped_weak.norm());
  const double orthogonality_error = (reliable.transpose() * reliable -
      Eigen::Matrix<double, 5, 5>::Identity()).norm();
  require((reliable.transpose() * reliable - Eigen::MatrixXd::Identity(5, 5)).norm() < 1e-10 &&
              leakage < 1e-9,
          "projected measurement row-space annihilates mapped weak direction");
  Eigen::Matrix<double, 6, 1> residual_one;
  residual_one << 0.4, -0.2, 0.1, 0.05, 0.3, -0.7;
  const Eigen::Matrix<double, 6, 1> residual_two =
      residual_one + mapped_weak.col(0) * 0.1;
  const double discarded_residual_projection_delta =
      (reliable.transpose() * residual_one -
       reliable.transpose() * residual_two).norm();
  require(discarded_residual_projection_delta < 1e-10,
          "adding a discarded weak residual cannot change the projected measurement");
  const Eigen::MatrixXd S = reliable.transpose() *
      (reliability::Matrix6d::Identity() + transform * transform.transpose()) * reliable;
  const Eigen::VectorXd r1 = reliable.transpose() * residual_one;
  const Eigen::VectorXd r2 = reliable.transpose() * residual_two;
  const double nis_difference = std::abs(r1.dot(S.ldlt().solve(r1)) -
                                        r2.dot(S.ldlt().solve(r2)));
  require(nis_difference < 1e-9, "coupled physical weak direction preserves linear NIS");
  constexpr double epsilon = 1e-7;
  const Eigen::Quaterniond nominal_q(map_R_imu);
  const Eigen::Vector3d nominal_p(0.3, -0.4, 0.7);
  const Eigen::Vector3d lidar_p = nominal_p + map_R_imu * imu_T_lidar;
  auto perturbed_residual = [&](double sign) {
    const Eigen::Vector3d phi = sign * epsilon * weak.head<3>();
    const Eigen::Quaterniond dq(Eigen::AngleAxisd(phi.norm(), phi.normalized()));
    const Eigen::Quaterniond measured_q = (dq * nominal_q).normalized();
    const Eigen::Vector3d measured_p = lidar_p + sign * epsilon * 0.8 * weak.tail<3>() -
        measured_q * imu_T_lidar;
    Eigen::Quaterniond error = (nominal_q.conjugate() * measured_q).normalized();
    if (error.w() < 0.0) error.coeffs() *= -1.0;
    const Eigen::AngleAxisd angle(error);
    Eigen::Matrix<double, 6, 1> r;
    r.head<3>() = measured_p - nominal_p;
    r.tail<3>() = angle.angle() * angle.axis();
    return r;
  };
  const Eigen::Matrix<double, 6, 1> fd =
      (perturbed_residual(1.0) - perturbed_residual(-1.0)) / (2.0 * epsilon);
  const double nonlinear_relative_error = (reliable.transpose() * fd).norm() /
      (transform * weak).norm();
  require(nonlinear_relative_error < 1e-5,
          "left-product LiDAR perturbation cancels in actual right-SO3 IMU residual at equal priors");
  std::cout << "coupled_linear_nis_difference=" << nis_difference
            << " nonlinear_fd_epsilon=" << epsilon
            << " nonlinear_fd_relative_error=" << nonlinear_relative_error << '\n';
  std::cout << "single_weak_leakage_relative=" << leakage
            << " single_weak_orthogonality_error=" << orthogonality_error
            << " discarded_residual_projection_delta=" <<
                discarded_residual_projection_delta << '\n';

  Eigen::Matrix<double, 6, 1> second_weak;
  second_weak << -0.1, 0.2, 0.3, -0.2, 0.5, 0.1;
  second_weak -= weak * weak.dot(second_weak);
  second_weak.normalize();
  weak_basis.col(1) = second_weak;
  require(reliability::buildReliableMeasurementBasisFromWeak(
              transform, weak_basis, 2, &measurement_basis, &rank, &failure) &&
              rank == 4,
          "two independent coupled weak directions produce rank-four complement");
  const Eigen::MatrixXd reliable_two = measurement_basis.leftCols(rank);
  const Eigen::MatrixXd mapped_weak_two = transform * weak_basis.leftCols(2);
  const double two_weak_orthogonality_error = (reliable_two.transpose() *
      reliable_two - Eigen::Matrix4d::Identity()).norm();
  const double two_weak_leakage_relative = (reliable_two.transpose() *
      mapped_weak_two).norm() / (1.0 + mapped_weak_two.norm());
  require(two_weak_orthogonality_error < 1e-10 &&
              two_weak_leakage_relative < 1e-9,
          "two-dimensional mixed weak subspace is fully annihilated");
  std::cout << "two_weak_rank=" << rank
            << " two_weak_leakage_relative=" << two_weak_leakage_relative
            << " two_weak_orthogonality_error=" << two_weak_orthogonality_error
            << '\n';
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
    testSchurScaleBalancedClassification();
    testCoupledWeakDirectionAnnihilation();
    testQualityAndAblationScenariosAtoE();
    std::cout << "PAPER_P6_I6E_CONDITIONAL_COMPENSATION_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PAPER_P6_I6E_CONDITIONAL_COMPENSATION_TEST_FAIL: "
              << error.what() << '\n';
    return 1;
  }
}
