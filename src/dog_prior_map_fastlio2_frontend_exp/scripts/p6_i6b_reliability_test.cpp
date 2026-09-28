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

void testWeakDirectionAndAdaptiveNoise() {
  const reliability::DualReliabilityConfig config;
  const reliability::LocalRisk local = reliability::assessLocalRisk(validLocal(), config);
  require(local.valid && local.translation_weak && local.rotation_weak,
          "weak blocks detected from valid eigenvalue ratios");
  require((local.map_translation_weak_direction - Eigen::Vector3d::UnitX()).norm() < 1e-12,
          "scaled translation block direction returned in map axes");

  const Eigen::Matrix3d map_R_imu = Eigen::AngleAxisd(
      M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto noise = reliability::makePoseMeasurementNoise(
      0.2, 0.1, map_R_imu, local, true, 1.0, config);
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

  const auto inflated = reliability::makePoseMeasurementNoise(
      0.2, 0.1, map_R_imu, local, false, 4.0, config);
  require(inflated.valid, "inflated covariance valid");
  reliability::Matrix6d expected_inflated = reliability::Matrix6d::Zero();
  expected_inflated.block<3, 3>(0, 0).diagonal().setConstant(4.0 * 0.2 * 0.2);
  expected_inflated.block<3, 3>(3, 3).diagonal().setConstant(4.0 * 0.1 * 0.1);
  require((inflated.covariance - expected_inflated).norm() < 1e-12,
          "cautious update isotropically inflates baseline noise");
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
      nominal, positive, negative, 2);
  require(stability.geometry_valid && stability.objectives_finite &&
              stability.all_converged && stability.extra_ndt_calls == 2,
          "terminal response fields captured");
  require(std::abs(stability.max_nominal_translation_delta_m - 0.25) < 1e-12,
          "Delta_t is max nominal response");
  require(std::abs(stability.max_nominal_rotation_delta_rad - 3.0 * M_PI / 180.0) < 1e-12,
          "Delta_R is max nominal response");
  const auto local = reliability::assessLocalRisk(validLocal());
  const auto decision = reliability::decideDualReliability(
      local, stability, true, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      true, true);
  require(decision.action == reliability::UpdateAction::CAUTIOUS_UPDATE &&
              decision.nonlocal_risk && decision.measurement_noise.valid,
          "nonlocal instability selects cautious update without candidate switch");

  const auto failed = reliability::analyzeNonlocalTerminalStability(
      nominal, positive, negative, 2);
  auto nonconverged_nominal = nominal;
  nonconverged_nominal.converged = false;
  const auto failed_pair = reliability::analyzeNonlocalTerminalStability(
      nonconverged_nominal, positive, negative, 2);
  require(failed.all_converged && !failed_pair.all_converged &&
              failed_pair.status == "NDT_NOT_CONVERGED",
          "nonconverged probe remains explicit");
  const auto prediction_only = reliability::decideDualReliability(
      local, stability, false, Eigen::Matrix3d::Identity(), 0.2, 0.1,
      true, true);
  require(prediction_only.action == reliability::UpdateAction::PREDICTION_ONLY,
          "nonconverged nominal rejects measurement update");
}

void testInvalidInputs() {
  reliability::LocalObservability unavailable;
  unavailable.valid = false;
  unavailable.status = "SCORE_GRADIENT_COORDINATE_CHECK_UNVERIFIED";
  const auto risk = reliability::assessLocalRisk(unavailable);
  require(!risk.valid, "unverified U_obs stays invalid");
  const auto noise = reliability::makePoseMeasurementNoise(
      std::numeric_limits<double>::quiet_NaN(), 0.1, Eigen::Matrix3d::Identity(),
      risk, true, 1.0);
  require(!noise.valid && noise.status == "INVALID_NOISE_INPUT",
          "nonfinite adaptive noise input rejected");
  const auto decision = reliability::decideDualReliability(
      risk, reliability::NonlocalTerminalStability{}, true,
      Eigen::Matrix3d::Identity(), 0.2, 0.1, true, true);
  require(decision.action == reliability::UpdateAction::NORMAL_UPDATE &&
              !decision.local_risk && !decision.nonlocal_risk,
          "unavailable U_obs never fabricates a weak direction");
}

}  // namespace

int main() {
  try {
    testSo3MapInnovation();
    testWeakDirectionAndAdaptiveNoise();
    testProbeTriggerAndMahalanobis();
    testTerminalResponseAndDecision();
    testInvalidInputs();
    std::cout << "P6_I6B_DUAL_RELIABILITY_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6B_DUAL_RELIABILITY_TEST_FAIL: " << error.what() << '\n';
    return 1;
  }
}
