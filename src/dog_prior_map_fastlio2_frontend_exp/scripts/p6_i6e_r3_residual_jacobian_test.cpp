#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include "p6_i6e_r3_actual_state.hpp"
#include "p6_i6e_r3_math.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;
namespace r3 = p6_i6e_r3;
namespace reliability = dog_prior_map_fastlio2_frontend_exp::reliability;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Eigen::Quaterniond expQuaternion(const Eigen::Vector3d& phi) {
  if (phi.norm() < 1e-15) return Eigen::Quaterniond::Identity();
  return Eigen::Quaterniond(Eigen::AngleAxisd(phi.norm(), phi.normalized()));
}

Eigen::Vector3d logQuaternion(Eigen::Quaterniond quaternion) {
  quaternion.normalize();
  if (quaternion.w() < 0.0) quaternion.coeffs() *= -1.0;
  const Eigen::AngleAxisd angle_axis(quaternion);
  return angle_axis.axis() * angle_axis.angle();
}

double relativeError(const Eigen::VectorXd& actual,
                     const Eigen::VectorXd& expected) {
  return (actual - expected).norm() /
      std::max({actual.norm(), expected.norm(), 1e-12});
}

void testSo3LeftJacobianInverse() {
  const std::vector<Eigen::Vector3d> phis = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.01, 0.0, 0.0),
      Eigen::Vector3d(0.3, -0.2, 0.1),
      Eigen::Vector3d(0.5, 0.4, -0.3),
      Eigen::Vector3d(0.8, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> directions = {
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0.3, -0.7, 0.2).normalized()};
  constexpr double epsilon = 1e-7;
  double maximum_error = 0.0;
  for (const Eigen::Vector3d& phi : phis) {
    Eigen::Matrix3d jacobian;
    std::string reason;
    require(so3LeftJacobianInverse(phi, &jacobian, &reason),
            "SO3 inverse failed:" + reason);
    for (const Eigen::Vector3d& direction : directions) {
      const Eigen::Vector3d fd =
          (logQuaternion(expQuaternion(epsilon * direction) *
                         expQuaternion(phi)) - phi) / epsilon;
      const double error = relativeError(fd, jacobian * direction);
      maximum_error = std::max(maximum_error, error);
      require(error < 1e-5, "SO3 left inverse finite difference mismatch");
    }
  }
  std::cout << "SO3_LEFT_JACOBIAN_INVERSE_PASS max_relative_error="
            << maximum_error << '\n';
}

Pose3d makePose(const Eigen::Vector3d& position,
                const Eigen::Vector3d& rotation_vector) {
  Pose3d pose;
  pose.position = position;
  pose.orientation = expQuaternion(rotation_vector);
  return pose;
}

Pose3d lidarMeasurementToImu(const Pose3d& map_T_lidar,
                             const Pose3d& T_imu_lidar) {
  Pose3d result;
  result.orientation = (map_T_lidar.orientation *
      T_imu_lidar.orientation.conjugate()).normalized();
  result.position = map_T_lidar.position -
      result.orientation * T_imu_lidar.position;
  return result;
}

Pose3d perturbLidar(const Pose3d& pose,
                    const Eigen::Matrix<double, 6, 1>& normalized_delta,
                    double length_scale, double factor) {
  Pose3d result = pose;
  result.orientation = (expQuaternion(factor * normalized_delta.head<3>()) *
                        result.orientation).normalized();
  result.position += factor * length_scale * normalized_delta.tail<3>();
  return result;
}

void testPhysicalResidualJacobian() {
  const Pose3d prior = makePose(Eigen::Vector3d(1.2, -0.4, 0.8),
                                Eigen::Vector3d(0.2, -0.1, 0.15));
  const Pose3d extrinsic = makePose(Eigen::Vector3d(0.31, -0.17, 0.09),
                                    Eigen::Vector3d(0.03, 0.02, -0.04));
  Pose3d nominal = makePose(Eigen::Vector3d(1.8, -0.1, 0.5),
                            Eigen::Vector3d(0.65, -0.25, 0.28));
  constexpr double length_scale = 0.8;
  Eigen::Matrix<double, 6, 6> jacobian;
  std::string reason;
  const Pose3d nominal_imu = lidarMeasurementToImu(nominal, extrinsic);
  require(r3::normalizedLidarToExactResidualJacobianFromImuMeasurement(
              prior, nominal_imu, extrinsic, length_scale, &jacobian, &reason),
          "exact A construction failed:" + reason);
  std::vector<Eigen::Matrix<double, 6, 1>> directions;
  for (int axis = 0; axis < 6; ++axis) {
    Eigen::Matrix<double, 6, 1> direction =
        Eigen::Matrix<double, 6, 1>::Zero();
    direction(axis) = 1.0;
    directions.push_back(direction);
  }
  Eigen::Matrix<double, 6, 1> mixed;
  mixed << 0.3, -0.5, 0.2, 0.7, -0.1, 0.4;
  directions.push_back(mixed.normalized());
  constexpr double epsilon = 1e-7;
  double maximum_error = 0.0;
  for (const auto& direction : directions) {
    const Pose3d plus = perturbLidar(nominal, direction, length_scale, epsilon);
    const Pose3d minus = perturbLidar(nominal, direction, length_scale, -epsilon);
    const Eigen::Matrix<double, 6, 1> fd =
        (r3::poseResidual(prior, lidarMeasurementToImu(plus, extrinsic)) -
         r3::poseResidual(prior, lidarMeasurementToImu(minus, extrinsic))) /
        (2.0 * epsilon);
    const double error = relativeError(fd, jacobian * direction);
    maximum_error = std::max(maximum_error, error);
    require(error < 1e-5, "physical A_exact finite difference mismatch");
  }
  std::cout << "PHYSICAL_RESIDUAL_JACOBIAN_PASS max_relative_error="
            << maximum_error << '\n';
}

void testBasisAndPriorSign() {
  const Pose3d prior = makePose(Eigen::Vector3d(-0.2, 0.5, 1.1),
                                Eigen::Vector3d(0.1, 0.2, -0.15));
  const Pose3d extrinsic = makePose(Eigen::Vector3d(0.25, -0.08, 0.12),
                                    Eigen::Vector3d(-0.02, 0.01, 0.03));
  const Pose3d nominal = makePose(Eigen::Vector3d(0.3, 0.9, 0.7),
                                  Eigen::Vector3d(0.55, -0.35, 0.22));
  constexpr double length_scale = 0.8;
  reliability::Matrix6d weak = reliability::Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> direction;
  direction << 0.35, -0.22, 0.41, 0.62, -0.31, 0.17;
  weak.col(0) = direction.normalized();
  reliability::Matrix6d exact_basis;
  int exact_rank = 0;
  std::string reason;
  const Pose3d measurement = lidarMeasurementToImu(nominal, extrinsic);
  require(r3::makeResidualConsistentMeasurementBasisFromImuMeasurement(
              prior, measurement, extrinsic, length_scale, weak, 1,
              &exact_basis, &exact_rank, &reason),
          "exact basis failed:" + reason);
  require(exact_rank == 5, "exact basis rank mismatch");
  Eigen::Matrix<double, 6, 6> exact_A;
  require(r3::normalizedLidarToExactResidualJacobianFromImuMeasurement(
              prior, measurement, extrinsic, length_scale, &exact_A, &reason),
          "exact A failed:" + reason);
  const double analytic_leakage =
      (exact_basis.leftCols(exact_rank).transpose() * exact_A * weak.leftCols(1)).norm();
  constexpr double epsilon = 1e-7;
  const Pose3d plus = perturbLidar(nominal, weak.col(0), length_scale, epsilon);
  const Pose3d minus = perturbLidar(nominal, weak.col(0), length_scale, -epsilon);
  const Eigen::Matrix<double, 6, 1> fd =
      (r3::poseResidual(prior, lidarMeasurementToImu(plus, extrinsic)) -
       r3::poseResidual(prior, lidarMeasurementToImu(minus, extrinsic))) /
      (2.0 * epsilon);
  const double fd_response =
      (exact_basis.leftCols(exact_rank).transpose() * fd).norm();
  const Eigen::Vector3d lever = -(measurement.orientation * extrinsic.position);
  reliability::Matrix6d legacy_A = reliability::Matrix6d::Zero();
  legacy_A.block<3, 3>(0, 0) = -r3::skewMatrix(lever);
  legacy_A.block<3, 3>(0, 3) = length_scale * Eigen::Matrix3d::Identity();
  legacy_A.block<3, 3>(3, 0) =
      measurement.orientation.toRotationMatrix().transpose();
  reliability::Matrix6d legacy_basis;
  int legacy_rank = 0;
  require(reliability::buildReliableMeasurementBasisFromWeak(
              legacy_A, weak, 1, &legacy_basis, &legacy_rank, &reason),
          "legacy basis construction failed:" + reason);
  const double legacy_analytic_leakage =
      (legacy_basis.leftCols(legacy_rank).transpose() * legacy_A *
       weak.leftCols(1)).norm();
  const double legacy_fd_response =
      (legacy_basis.leftCols(legacy_rank).transpose() * fd).norm();
  require(analytic_leakage < 1e-9 && fd_response < 1e-5,
          "exact reliable basis leaks weak direction");

  const Eigen::Vector3d phi = r3::poseResidual(prior, measurement).tail<3>();
  Eigen::Matrix3d left_inverse;
  require(so3LeftJacobianInverse(phi, &left_inverse, &reason),
          "prior sign Jacobian failed:" + reason);
  const Eigen::Vector3d d(0.3, -0.6, 0.2);
  Pose3d perturbed_prior = prior;
  perturbed_prior.orientation =
      (prior.orientation * expQuaternion(epsilon * d)).normalized();
  const Eigen::Vector3d prior_fd =
      (r3::poseResidual(perturbed_prior, measurement).tail<3>() - phi) /
      epsilon;
  require(relativeError(prior_fd, -left_inverse * d) < 1e-5,
          "prior right perturbation sign mismatch");
  std::cout << "RESIDUAL_CONSISTENT_BASIS_PASS analytic_leakage="
            << analytic_leakage << " fd_response=" << fd_response
            << " legacy_analytic_leakage=" << legacy_analytic_leakage
            << " legacy_fd_response=" << legacy_fd_response << '\n';
}

std::unique_ptr<FastLio2IkfomFrontend> initializedFrontend() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 20;
  std::unique_ptr<FastLio2IkfomFrontend> frontend(
      new FastLio2IkfomFrontend(parameters));
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples;
  for (int index = 0; index < parameters.static_init_samples; ++index) {
    ImuSample sample;
    sample.stamp_ns = 1'000'000'000ULL + index * 5'000'000ULL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2);
    sample.angular_velocity.setZero();
    samples.push_back(sample);
  }
  Pose3d identity;
  std::string reason;
  require(frontend->initializeStatic(samples, identity, identity, &reason),
          "test frontend initialization failed:" + reason);
  return frontend;
}

bool sameSnapshot(const FilterSnapshot& left, const FilterSnapshot& right) {
  return left.stamp_ns == right.stamp_ns &&
      left.map_T_imu.position == right.map_T_imu.position &&
      left.map_T_imu.orientation.coeffs() == right.map_T_imu.orientation.coeffs() &&
      left.covariance == right.covariance;
}

void testZeroAndNearPi() {
  Pose3d prior = makePose(Eigen::Vector3d(0.2, -0.1, 0.4),
                          Eigen::Vector3d(0.2, 0.1, -0.05));
  Pose3d extrinsic = makePose(Eigen::Vector3d(0.3, 0.1, -0.07),
                              Eigen::Vector3d(0.01, -0.02, 0.03));
  Pose3d nominal;
  nominal.orientation = (prior.orientation * extrinsic.orientation).normalized();
  nominal.position = prior.position + prior.orientation * extrinsic.position;
  Eigen::Matrix<double, 6, 6> exact_A;
  std::string reason;
  require(r3::normalizedLidarToExactResidualJacobianFromImuMeasurement(
              prior, lidarMeasurementToImu(nominal, extrinsic), extrinsic,
              0.8, &exact_A, &reason),
          "zero innovation exact A failed:" + reason);
  const Eigen::Vector3d lever = -(prior.orientation * extrinsic.position);
  Eigen::Matrix<double, 6, 6> legacy_A = Eigen::Matrix<double, 6, 6>::Zero();
  legacy_A.block<3, 3>(0, 0) = -r3::skewMatrix(lever);
  legacy_A.block<3, 3>(0, 3) = 0.8 * Eigen::Matrix3d::Identity();
  legacy_A.block<3, 3>(3, 0) = prior.orientation.toRotationMatrix().transpose();
  require((exact_A - legacy_A).norm() < 1e-12,
          "zero innovation exact/legacy A mismatch");

  auto frontend = initializedFrontend();
  const FilterSnapshot before = frontend->getState();
  const Eigen::Matrix<double, 6, 6> identity =
      Eigen::Matrix<double, 6, 6>::Identity();
  ProjectedPoseInnovation legacy_zero, exact_zero;
  require(frontend->evaluateProjectedPoseInnovation(
              before.map_T_imu, 0.01 * identity, identity, 6,
              &legacy_zero, &reason) &&
          frontend->evaluateProjectedPoseInnovationLinearized(
              before.map_T_imu, 0.01 * identity, identity, 6,
              ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL,
              &exact_zero, &reason) &&
          std::abs(legacy_zero.nis - exact_zero.nis) < 1e-12 &&
          std::abs(legacy_zero.innovation_covariance_trace -
                   exact_zero.innovation_covariance_trace) < 1e-12,
          "zero innovation legacy/exact projected systems differ");
  auto legacy_clone = frontend->cloneCandidate();
  auto exact_clone = frontend->cloneCandidate();
  PoseCorrectionDelta legacy_delta, exact_delta;
  require(legacy_clone->applyProjectedPoseMeasurementChecked(
              before.map_T_imu, 0.01 * identity, identity, 6, false,
              chiSquare99Threshold(6), nullptr, &legacy_delta, &reason) &&
          exact_clone->applyProjectedPoseMeasurementLinearizedChecked(
              before.map_T_imu, 0.01 * identity, identity, 6,
              ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL, false,
              chiSquare99Threshold(6), nullptr, &exact_delta, &reason) &&
          sameSnapshot(legacy_clone->getState(), exact_clone->getState()),
          "zero innovation legacy/exact updates differ");
  Pose3d near_pi = before.map_T_imu;
  near_pi.orientation = (near_pi.orientation * expQuaternion(
      Eigen::Vector3d(M_PI - 5e-5, 0.0, 0.0))).normalized();
  ProjectedPoseInnovation diagnostic;
  require(!frontend->evaluateProjectedPoseInnovationLinearized(
              near_pi, 0.01 * identity, identity, 6,
              ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL,
              &diagnostic, &reason) && reason == "ROTATION_RESIDUAL_NEAR_PI" &&
              sameSnapshot(before, frontend->getState()),
          "near-pi exact mode did not reject atomically");
  PoseCorrectionDelta rejected_delta;
  require(!frontend->applyProjectedPoseMeasurementLinearizedChecked(
              near_pi, 0.01 * identity, identity, 6,
              ProjectedPoseLinearizationMode::EXACT_LOG_RESIDUAL, false,
              chiSquare99Threshold(6), &diagnostic, &rejected_delta,
              &reason) && reason == "ROTATION_RESIDUAL_NEAR_PI" &&
              sameSnapshot(before, frontend->getState()),
          "near-pi exact apply did not reject atomically");
  std::cout << "ZERO_AND_NEAR_PI_PASS reason=" << reason << '\n';
}

void testActualFusionLedger() {
  r3::ActualFusionLedger ledger;
  r3::initializeActualFusionLedger(1'000'000'000ULL, &ledger);
  require(ledger.last_external_commit_ns == 1'000'000'000ULL &&
              !ledger.ever_committed_external,
          "actual ledger initialization mismatch");
  require(r3::actualHealthStatus(false, false, false, &ledger) == "IMU_ONLY" &&
              ledger.last_external_commit_ns == 1'000'000'000ULL,
          "non-commit advanced actual ledger");
  r3::recordVisualCommit(1'100'000'000ULL, &ledger);
  require(r3::actualHealthStatus(false, true, false, &ledger) == "VISUAL_ONLY" &&
              ledger.visual_commit_count == 1,
          "visual commit not recorded");
  r3::recordLidarCommit(1'200'000'000ULL, &ledger);
  require(r3::actualHealthStatus(true, false, true, &ledger) ==
              "PARTIAL_DIRECTIONAL_CONSTRAINT" &&
              ledger.lidar_commit_count == 1,
          "partial directional LiDAR status mismatch");
  r3::updateRelocalizationPending(true, 1'300'000'000ULL, 0.1, 2.0,
                                  &ledger);
  r3::recordLidarCommit(1'400'000'000ULL, &ledger);
  require(r3::actualHealthStatus(true, false, false, &ledger) ==
              "RELOCALIZATION_PENDING_VALIDATION" &&
              ledger.relocalization_requested_ns == 1'300'000'000ULL &&
              ledger.verified_recovery_ns == 0,
          "ledger manufactured unverified recovery");
  std::cout << "ACTUAL_FUSION_LEDGER_PASS\n";
}

}  // namespace

int main() {
  try {
    testSo3LeftJacobianInverse();
    testPhysicalResidualJacobian();
    testBasisAndPriorSign();
    testZeroAndNearPi();
    testActualFusionLedger();
    std::cout << "P6_I6E_R3_RESIDUAL_JACOBIAN_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "P6_I6E_R3_RESIDUAL_JACOBIAN_FAIL: " << error.what() << '\n';
    return 1;
  }
}
