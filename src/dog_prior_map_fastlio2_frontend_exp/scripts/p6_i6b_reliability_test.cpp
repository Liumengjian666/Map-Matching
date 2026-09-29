#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace reliability = dog_prior_map_fastlio2_frontend_exp::reliability;

namespace {

void require(bool condition, const char* label) {
  if (!condition) throw std::runtime_error(label);
}

reliability::LocalObservability validLocal() {
  reliability::LocalObservability local;
  local.valid = true;
  local.status = "VALID_BLOCK_CURVATURE";
  local.rotation_block_eigenvalues << 0.01, 0.5, 1.0;
  local.translation_block_eigenvalues << 0.02, 0.4, 0.9;
  local.rotation_block_eigenvectors.setIdentity();
  local.translation_block_eigenvectors.setIdentity();
  return local;
}

void testSo3MapInnovation() {
  const Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
  measured.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
                          .toRotationMatrix();
  measured.translation() = Eigen::Vector3d(1.0, -2.0, 0.5);
  const reliability::Vector6d innovation =
      reliability::mapProductInnovation(predicted, measured);
  require(innovation.allFinite(), "SO3 map innovation finite");
  require((innovation.head<3>() - Eigen::Vector3d(0.0, 0.0, M_PI / 2.0)).norm() < 1e-12,
          "SO3 logarithm is map-spatial");
  require((innovation.tail<3>() - measured.translation()).norm() < 1e-12,
          "position innovation is map additive");
}

void testPclCurvatureCoordinateTransform() {
  reliability::Matrix6d pcl_hessian = reliability::Matrix6d::Zero();
  // PCL order is [translation xyz, Euler rx/ry/rz]. The score is locally
  // concave here, so -sym(H_score) has positive diagonal curvature.
  pcl_hessian.diagonal() << -2.0, -3.0, -4.0, -5.0, -6.0, -7.0;
  const Eigen::Vector3d euler(0.31, -0.22, 0.43);
  const auto transformed = reliability::transformPclScoreHessianToNormalizedMapTangent(
      pcl_hessian, euler, 0.8);
  require(transformed.valid, "P6-I3 Euler-to-map curvature transform valid");
  require(transformed.status == "PASS_P6I3_EULER_TO_MAP_TANGENT",
          "P6-I3 transform status explicit");
  const Eigen::Matrix3d expected_rotation_euler =
      Eigen::Vector3d(5.0, 6.0, 7.0).asDiagonal();
  const Eigen::Matrix3d expected_translation_euler =
      Eigen::Vector3d(2.0, 3.0, 4.0).asDiagonal();
  require((transformed.hessian_euler.block<3, 3>(0, 0) -
           expected_rotation_euler).norm() < 1e-12,
          "PCL rotation columns reordered before canonical Hessian");
  require((transformed.hessian_euler.block<3, 3>(3, 3) -
           expected_translation_euler).norm() < 1e-12,
          "PCL translation columns reordered before canonical Hessian");
  Eigen::Matrix3d inverse = transformed.euler_to_map_spatial_jacobian.inverse();
  const Eigen::Matrix3d expected_rotation = inverse.transpose() *
      Eigen::Vector3d(5.0, 6.0, 7.0).asDiagonal() * inverse;
  require((transformed.hessian_physical.block<3, 3>(0, 0) -
           expected_rotation).norm() < 1e-10,
          "physical rotation curvature uses P6-I3 inverse Euler Jacobian");
  require(std::abs(transformed.normalized_negative_score_curvature(3, 3) - 1.28) < 1e-12,
          "translation curvature receives S^T H S with resolution scale squared");
  const auto singular = reliability::transformPclScoreHessianToNormalizedMapTangent(
      pcl_hessian, Eigen::Vector3d(0.1, M_PI_2, -0.2), 0.8);
  require(!singular.valid, "Euler spatial Jacobian singularity is rejected");
}

void testWeakDirectionAndAdaptiveNoise() {
  const reliability::DualReliabilityConfig config;
  const reliability::LocalRisk local = reliability::assessLocalRisk(validLocal(), config);
  require(local.valid && local.translation_weak && local.rotation_weak,
          "weak blocks detected from valid eigenvalue ratios");
  require((local.map_translation_weak_direction - Eigen::Vector3d::UnitX()).norm() < 1e-12,
          "scaled translation block direction returned in map axes");

  const Eigen::Matrix3d map_R_imu = Eigen::AngleAxisd(
      M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  reliability::NonlocalTerminalStability not_probed;
  not_probed.status = "NOT_PROBED";
  const auto noise = reliability::makePoseMeasurementNoise(
      0.2, 0.1, map_R_imu, local, not_probed, true, false, config);
  require(noise.valid, "adaptive noise valid");
  Eigen::SelfAdjointEigenSolver<reliability::Matrix6d> solver(noise.covariance);
  require(solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() > 0.0,
          "6x6 covariance SPD");
  require(noise.covariance.block<3, 3>(0, 0)(0, 0) >
              noise.covariance.block<3, 3>(0, 0)(1, 1),
          "translation weak map direction gets larger variance");
  const Eigen::Vector3d expected_body = map_R_imu.transpose() * Eigen::Vector3d::UnitX();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> rotation_solver(
      noise.covariance.block<3, 3>(3, 3));
  require(rotation_solver.info() == Eigen::Success,
          "rotation covariance eigensolve");
  require(std::abs(rotation_solver.eigenvectors().col(2).dot(expected_body)) > 1.0 - 1e-12,
          "map weak rotation transformed into body error axes");
  require((noise.rotation_map_covariance -
      (0.1 * 0.1 * Eigen::Matrix3d::Identity() +
       noise.local_rotation_map_increment)).norm() < 1e-12,
      "U_obs contributes map-frame direction noise before basis conversion");
  const Eigen::Matrix3d expected_body_cov = map_R_imu.transpose() *
      noise.rotation_map_covariance * map_R_imu;
  require((noise.covariance.block<3, 3>(3, 3) - expected_body_cov).norm() < 1e-12,
          "full map rotation covariance is converted to right/body basis");
  require(noise.local_translation_increment(0, 0) > 0.0 &&
              noise.local_translation_increment(1, 1) == 0.0,
          "weak-curvature severity produces bounded rank-one translation increment");
}

void testProbeTriggerAndMahalanobis() {
  reliability::Matrix6d covariance = reliability::Matrix6d::Identity();
  reliability::Vector6d innovation = reliability::Vector6d::Zero();
  auto periodic = reliability::shouldRunNonlocalProbes(25, innovation, covariance);
  require(periodic.run_probes && periodic.reason == "PERIODIC_CHECK",
          "periodic probe trigger");
  innovation(3) = std::sqrt(20.0);
  auto abnormal = reliability::shouldRunNonlocalProbes(26, innovation, covariance);
  require(abnormal.run_probes && abnormal.reason == "HIGH_NORMALIZED_INNOVATION",
          "innovation-triggered probe");
  require(abnormal.innovation.valid &&
              std::abs(abnormal.innovation.mahalanobis_squared - 20.0) < 1e-12,
          "Mahalanobis statistic");

  covariance.setZero();
  const auto unsupported = reliability::normalizedInnovation(innovation, covariance);
  require(!unsupported.valid, "zero covariance rejected");
  const auto fallback = reliability::shouldRunNonlocalProbes(27, innovation, covariance);
  require(fallback.run_probes && fallback.reason == "ABSOLUTE_INNOVATION_FALLBACK",
          "absolute innovation fallback when covariance invalid");

  innovation(0) = std::numeric_limits<double>::quiet_NaN();
  const auto nonfinite = reliability::shouldRunNonlocalProbes(28, innovation,
                                                               reliability::Matrix6d::Identity());
  require(nonfinite.run_probes && nonfinite.reason == "NONFINITE_INNOVATION",
          "nonfinite innovation triggers conservative probing");
}

void testTerminalResponseAndDecision() {
  reliability::TerminalCapture nominal, positive, negative;
  nominal.converged = positive.converged = negative.converged = true;
  nominal.fixed_objective = 10.0;
  positive.fixed_objective = 9.0;
  negative.fixed_objective = 8.0;
  positive.map_T_lidar.translation().x() = 0.25;
  negative.map_T_lidar.translation().x() = -0.03;
  positive.map_T_lidar.linear() = Eigen::AngleAxisd(
      3.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto stability = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, Eigen::Isometry3d::Identity(), 2);
  require(stability.geometry_valid && stability.objectives_finite &&
              stability.all_converged && stability.extra_ndt_calls == 2,
          "terminal response fields captured");
  require(std::abs(stability.max_nominal_translation_delta_m - 0.25) < 1e-12,
          "Delta_t is max nominal response");
  require(std::abs(stability.max_nominal_rotation_delta_rad - 3.0 * M_PI / 180.0) < 1e-12,
          "Delta_R is max nominal response");
  require(stability.response_valid &&
              (stability.delta_position_imu_positive - Eigen::Vector3d(0.25, 0.0, 0.0)).norm() < 1e-12 &&
              (stability.delta_position_imu_negative - Eigen::Vector3d(-0.03, 0.0, 0.0)).norm() < 1e-12,
          "signed plus/minus IMU-origin translation responses retained");
  require((stability.delta_rotation_positive -
           Eigen::Vector3d(0.0, 0.0, 3.0 * M_PI / 180.0)).norm() < 1e-12,
          "SO3 log response retained in map-spatial coordinates");
  const auto local = reliability::assessLocalRisk(validLocal());
  const auto decision = reliability::decideDualReliability(
      local, stability, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      true, true);
  require(decision.action == reliability::UpdateAction::CAUTIOUS_UPDATE &&
              decision.nonlocal_risk && decision.measurement_noise.valid &&
              decision.nonlocal_response_used,
          "nonlocal instability selects directional cautious update without candidate switch");
  const Eigen::Matrix3d expected_bp = 0.5 *
      (stability.delta_position_imu_positive *
           stability.delta_position_imu_positive.transpose() +
       stability.delta_position_imu_negative *
           stability.delta_position_imu_negative.transpose());
  require((decision.measurement_noise.nonlocal_translation_increment - expected_bp).norm() < 1e-12,
          "U_nonlocal position covariance uses signed response outer products");
  const double expected_rotation_cap = reliability::DualReliabilityConfig{}
      .max_nonlocal_rotation_variance_rad2;
  require(std::abs(decision.measurement_noise.nonlocal_rotation_map_increment(2, 2) -
                   expected_rotation_cap) < 1e-12,
          "U_nonlocal rotation response obeys configured spectral cap");

  const auto failed = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, Eigen::Isometry3d::Identity(), 2);
  auto nonconverged_nominal = nominal;
  nonconverged_nominal.converged = false;
  const auto failed_pair = reliability::analyzeNonlocalTerminalStability(
      nonconverged_nominal, positive, negative, Eigen::Isometry3d::Identity(), 2);
  require(failed.all_converged && !failed_pair.all_converged &&
              failed_pair.status == "NDT_NOT_CONVERGED" &&
              !failed_pair.response_valid,
          "nonconverged probe remains explicit");
  const auto wrong_call_count = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, Eigen::Isometry3d::Identity(), 1);
  require(wrong_call_count.status == "PROBE_CALL_COUNT_MISMATCH" &&
              wrong_call_count.response_valid,
          "probe response requires exactly two additional NDT calls");
  auto nan_objective_terminal = positive;
  nan_objective_terminal.fixed_objective =
      std::numeric_limits<double>::quiet_NaN();
  const auto nan_objective_pair = reliability::analyzeNonlocalTerminalStability(
      nominal, nan_objective_terminal, negative,
      Eigen::Isometry3d::Identity(), 2);
  require(!nan_objective_pair.objectives_finite &&
              nan_objective_pair.status == "NONFINITE_FIXED_OBJECTIVE",
          "NaN fixed objective invalidates the probe pair");

  // NDT terminals are map_T_lidar, but the adaptive position covariance is
  // consumed by an IKFoM map_T_imu update. A nonzero rotated lever arm makes
  // those translation responses measurably different when the LiDAR rotates.
  Eigen::Isometry3d imu_T_lidar = Eigen::Isometry3d::Identity();
  imu_T_lidar.linear() = Eigen::AngleAxisd(
      0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  imu_T_lidar.translation() = Eigen::Vector3d(0.08, 0.029, 0.03);
  const auto lever_arm_stability = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, imu_T_lidar, 2);
  const Eigen::Vector3d expected_positive_imu_position =
      (positive.map_T_lidar * imu_T_lidar.inverse()).translation() -
      (nominal.map_T_lidar * imu_T_lidar.inverse()).translation();
  const Eigen::Vector3d expected_negative_imu_position =
      (negative.map_T_lidar * imu_T_lidar.inverse()).translation() -
      (nominal.map_T_lidar * imu_T_lidar.inverse()).translation();
  require(lever_arm_stability.imu_lidar_extrinsic_valid &&
              lever_arm_stability.response_valid &&
              (lever_arm_stability.delta_position_imu_positive -
               expected_positive_imu_position).norm() < 1e-12 &&
              (lever_arm_stability.delta_position_imu_negative -
               expected_negative_imu_position).norm() < 1e-12,
          "nonlocal position response is transformed to map_T_imu origin");
  require((lever_arm_stability.delta_position_imu_positive -
           Eigen::Vector3d(0.25, 0.0, 0.0)).norm() > 1e-3,
          "LiDAR rotation lever arm contributes to IMU-origin position response");
  Eigen::Isometry3d invalid_extrinsic = Eigen::Isometry3d::Identity();
  invalid_extrinsic.linear()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_extrinsic_result = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, invalid_extrinsic, 2);
  require(!invalid_extrinsic_result.imu_lidar_extrinsic_valid &&
              !invalid_extrinsic_result.response_valid &&
              invalid_extrinsic_result.status == "INVALID_IMU_LIDAR_EXTRINSIC",
          "invalid LiDAR-IMU transform cannot produce a nonlocal response");
  const auto prediction_only = reliability::decideDualReliability(
      local, stability, false, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      true, true);
  require(prediction_only.action == reliability::UpdateAction::PREDICTION_ONLY,
          "nonconverged nominal rejects measurement update");

  reliability::NonlocalTerminalStability not_probed;
  not_probed.status = "NOT_PROBED";
  const auto no_probe = reliability::decideDualReliability(
      local, not_probed, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      false, true);
  require(no_probe.action == reliability::UpdateAction::NORMAL_UPDATE &&
              !no_probe.nonlocal_response_used &&
              no_probe.measurement_noise.nonlocal_translation_increment.isZero(0.0),
          "NOT_PROBED has zero response contribution and is not called stable");

  auto invalid_probe = stability;
  invalid_probe.status = "NDT_NOT_CONVERGED";
  invalid_probe.all_converged = false;
  const auto invalid_decision = reliability::decideDualReliability(
      local, invalid_probe, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      false, true);
  require(invalid_decision.action == reliability::UpdateAction::PREDICTION_ONLY &&
              invalid_decision.reason.find("PROBE_INVALID:") == 0,
          "nonconverged probes never become a response covariance");
  const auto nan_objective_decision = reliability::decideDualReliability(
      local, nan_objective_pair, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      false, true);
  require(nan_objective_decision.action == reliability::UpdateAction::PREDICTION_ONLY,
          "NaN probe objective cannot form adaptive measurement covariance");
  const auto wrong_count_decision = reliability::decideDualReliability(
      local, wrong_call_count, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      false, true);
  require(wrong_count_decision.action == reliability::UpdateAction::PREDICTION_ONLY,
          "mismatched probe call count invalidates reliability update");
}

void testInvalidInputs() {
  reliability::LocalObservability unavailable;
  unavailable.valid = false;
  unavailable.status = "SCORE_GRADIENT_COORDINATE_CHECK_UNVERIFIED";
  const auto risk = reliability::assessLocalRisk(unavailable);
  require(!risk.valid, "unverified U_obs stays invalid");
  const auto noise = reliability::makePoseMeasurementNoise(
      std::numeric_limits<double>::quiet_NaN(), 0.1, Eigen::Matrix3d::Identity(),
      risk, reliability::NonlocalTerminalStability{}, true, false);
  require(!noise.valid && noise.status == "INVALID_NOISE_INPUT",
          "nonfinite adaptive noise input rejected");
  reliability::NonlocalTerminalStability not_probed;
  not_probed.status = "NOT_PROBED";
  const auto decision = reliability::decideDualReliability(
      risk, not_probed, true, Eigen::Matrix3d::Identity(), 0.2, 0.1, true, true);
  require(decision.action == reliability::UpdateAction::NORMAL_UPDATE &&
              !decision.local_risk && !decision.nonlocal_risk,
          "unavailable U_obs never fabricates a weak direction");
}

void testGeometricObservabilityProxy() {
  const Eigen::Vector3d rotated_source(0.7, -0.4, 1.2);
  const auto analytic = reliability::geometricPointResidualJacobian(rotated_source);
  Eigen::Matrix<double, 3, 6> numeric;
  constexpr double step = 1e-7;
  for (int axis = 0; axis < 6; ++axis) {
    Eigen::Vector3d translation_delta = Eigen::Vector3d::Zero();
    if (axis >= 3) translation_delta(axis % 3) = step;
    const Eigen::Matrix3d plus_rotation =
        Eigen::AngleAxisd(step, Eigen::Vector3d::Unit(axis % 3)).toRotationMatrix();
    const Eigen::Matrix3d minus_rotation =
        Eigen::AngleAxisd(-step, Eigen::Vector3d::Unit(axis % 3)).toRotationMatrix();
    const Eigen::Vector3d plus = (axis < 3 ? plus_rotation * rotated_source :
        rotated_source) + translation_delta;
    const Eigen::Vector3d minus = (axis < 3 ? minus_rotation * rotated_source :
        rotated_source) - translation_delta;
    numeric.col(axis) = (plus - minus) / (2.0 * step);
  }
  require(analytic.allFinite() && (analytic - numeric).norm() < 1e-8,
          "map-spatial left perturbation geometric Jacobian matches central differences");

  std::vector<reliability::GeometricObservation> observations;
  for (int i = 0; i < 12; ++i) {
    reliability::GeometricObservation sample;
    sample.rotated_source_map = Eigen::Vector3d(
        0.2 * i, std::sin(0.4 * i), std::cos(0.31 * i));
    sample.residual_map = Eigen::Vector3d(0.001 * i, -0.002, 0.003);
    sample.voxel_covariance_map = Eigen::Vector3d(0.02, 0.04, 0.08).asDiagonal();
    sample.nonnegative_weight = 1.0 + 0.1 * i;
    observations.push_back(sample);
  }
  const auto local = reliability::analyzeGeometricObservability(
      observations, true, 0.8);
  require(local.valid && local.geometric_proxy &&
              local.status == "MAP_SUPPORT_INSUFFICIENT" &&
              !local.map_support_sufficient,
          "low-count geometric U_obs remains diagnostic but cannot claim map support");
  require(!local.score_gradient_coordinate_check_passed &&
              local.normalized_geometric_information.allFinite() &&
              (local.normalized_geometric_information -
               local.normalized_geometric_information.transpose()).norm() < 1e-12,
          "geometric information is explicitly separate, finite, symmetric");
  Eigen::SelfAdjointEigenSolver<reliability::Matrix6d> psd_solver(
      local.normalized_geometric_information);
  require(psd_solver.info() == Eigen::Success &&
              psd_solver.eigenvalues().minCoeff() >= -1e-10,
          "geometric information proxy is positive semidefinite");
  require(local.valid_correspondence_count == observations.size() &&
              std::abs(local.effective_weight_sum - 18.6) < 1e-12,
          "geometric observability reports effective correspondence weights");

  auto invalid_covariance = observations;
  invalid_covariance.front().voxel_covariance_map.diagonal() << 1.0, 0.1, -0.5;
  const auto rejected_covariance = reliability::analyzeGeometricObservability(
      invalid_covariance, true, 0.8);
  require(rejected_covariance.valid &&
              rejected_covariance.rejected_covariance_count == 1 &&
              rejected_covariance.valid_correspondence_count == observations.size() - 1,
          "materially indefinite target covariance is rejected, not floored into fake geometry");

  // Singular block curvature is valid geometric evidence: a zero eigenvalue
  // denotes an unconstrained direction, not an invalid PCL Hessian.
  reliability::LocalObservability rank_deficient;
  rank_deficient.valid = true;
  rank_deficient.geometric_proxy = true;
  rank_deficient.status = "VALID_GEOMETRIC_GAUSS_NEWTON_PROXY";
  rank_deficient.rotation_block_eigenvalues << 0.0, 0.5, 1.0;
  rank_deficient.translation_block_eigenvalues << 0.0, 0.3, 0.9;
  rank_deficient.rotation_block_eigenvectors.setIdentity();
  rank_deficient.translation_block_eigenvectors.setIdentity();
  const auto risk = reliability::assessLocalRisk(rank_deficient);
  require(risk.valid && risk.rotation_weak && risk.translation_weak &&
              risk.rotation_weak_ratio == 0.0 && risk.translation_weak_ratio == 0.0,
          "PSD geometric block with null direction is usable as directional risk");

  const auto invalid = reliability::analyzeGeometricObservability(
      observations, false, 0.8);
  require(!invalid.valid && invalid.status == "NDT_NOT_CONVERGED",
          "nonconverged NDT does not expose geometric observability");
}

}  // namespace

int main() {
  try {
    testSo3MapInnovation();
    testPclCurvatureCoordinateTransform();
    testWeakDirectionAndAdaptiveNoise();
    testProbeTriggerAndMahalanobis();
    testTerminalResponseAndDecision();
    testInvalidInputs();
    testGeometricObservabilityProxy();
    std::cout << "P6_I6B_DUAL_RELIABILITY_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6B_DUAL_RELIABILITY_TEST_FAIL: " << error.what() << '\n';
    return 1;
  }
}
