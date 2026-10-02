#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {

bool sameFilterSnapshot(const FilterSnapshot& lhs, const FilterSnapshot& rhs) {
  return lhs.stamp_ns == rhs.stamp_ns &&
      lhs.map_T_imu.position == rhs.map_T_imu.position &&
      lhs.map_T_imu.orientation.coeffs() == rhs.map_T_imu.orientation.coeffs() &&
      lhs.velocity == rhs.velocity && lhs.gyro_bias == rhs.gyro_bias &&
      lhs.accel_bias == rhs.accel_bias && lhs.gravity == rhs.gravity &&
      lhs.T_imu_lidar_translation == rhs.T_imu_lidar_translation &&
      lhs.T_imu_lidar_rotation == rhs.T_imu_lidar_rotation &&
      lhs.covariance.rows() == rhs.covariance.rows() &&
      lhs.covariance.cols() == rhs.covariance.cols() &&
      lhs.covariance == rhs.covariance;
}

}  // namespace

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.initial_accel_bias = Eigen::Vector3d(0.02, -0.01, 0.03);
  parameters.pose_position_sigma_m = 0.04;
  parameters.pose_rotation_sigma_rad = 0.025;
  FastLio2IkfomFrontend committed(parameters);
  const Eigen::Vector3d gyro_bias(0.009, -0.016, 0.004);
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  for (int i = 0; i < parameters.static_init_samples; ++i) {
    ImuSample sample;
    sample.stamp_ns = 1000000000ULL + static_cast<uint64_t>(i) * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.02, -0.01, parameters.gravity_mps2 + 0.03);
    sample.angular_velocity = gyro_bias;
    stationary.push_back(sample);
  }
  Pose3d extrinsic;
  extrinsic.position = Eigen::Vector3d(0.12, -0.035, 0.025);
  extrinsic.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.015, Eigen::Vector3d::UnitY()));
  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.position = extrinsic.position;
  initial_map_T_lidar.orientation = extrinsic.orientation;
  std::string failure;
  if (!committed.initializeStatic(stationary, initial_map_T_lidar, extrinsic, &failure)) {
    std::cerr << "FAIL: initialization: " << failure << '\n';
    return 1;
  }

  const uint64_t start_ns = stationary.back().stamp_ns;
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> sequence;
  sequence.push_back(stationary.back());
  const Eigen::Vector3d acceleration(0.18, 0.07, 0.0);
  const Eigen::Vector3d angular_velocity(0.03, -0.02, 0.42);
  for (int i = 1; i <= 100; ++i) {
    ImuSample sample;
    sample.stamp_ns = start_ns + static_cast<uint64_t>(i) * 5000000ULL;
    const double dt = static_cast<double>(sample.stamp_ns - start_ns) * 1e-9;
    const Eigen::Vector3d rv = angular_velocity * dt;
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        rv.norm(), rv.norm() > 0 ? rv.normalized() : Eigen::Vector3d::UnitX()));
    sample.acceleration = rotation.conjugate() *
        (acceleration + Eigen::Vector3d(0, 0, parameters.gravity_mps2)) +
        parameters.initial_accel_bias;
    sample.angular_velocity = angular_velocity + gyro_bias;
    sequence.push_back(sample);
  }
  std::unique_ptr<FastLio2IkfomFrontend> candidate = committed.cloneCandidate();
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> poses;
  const uint64_t scan_end_ns = sequence.back().stamp_ns;
  if (!candidate->predictImuSequence(sequence, scan_end_ns, &poses, &failure)) {
    std::cerr << "FAIL: candidate prediction: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot predicted = candidate->getState();


  Pose3d measurement = predicted.map_T_imu;
  measurement.position += Eigen::Vector3d(-0.08, 0.035, 0.025);
  measurement.orientation = (measurement.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.06, Eigen::Vector3d(0.1, -0.2, 0.97).normalized()))).normalized();

  std::unique_ptr<FastLio2IkfomFrontend> shadow = candidate->cloneCandidate();
  PoseCorrectionDelta delta;
  if (!shadow->applyPoseMeasurement(measurement, &delta, &failure)) {
    std::cerr << "FAIL: generic pose update: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot corrected = shadow->getState();
  const double position_error_before = (predicted.map_T_imu.position - measurement.position).norm();
  const double position_error_after = (corrected.map_T_imu.position - measurement.position).norm();
  const double rotation_error_before = Eigen::AngleAxisd(
      predicted.map_T_imu.orientation.conjugate() * measurement.orientation).angle();
  const double rotation_error_after = Eigen::AngleAxisd(
      corrected.map_T_imu.orientation.conjugate() * measurement.orientation).angle();
  if (!(position_error_after < position_error_before && rotation_error_after < rotation_error_before) ||
      !shadow->postconditionsValid(&failure) ||
      (corrected.T_imu_lidar_translation - extrinsic.position).norm() > 1e-12 ||
      (corrected.T_imu_lidar_rotation - extrinsic.orientation.toRotationMatrix()).norm() > 1e-12) {
    std::cerr << "FAIL: update direction/postconditions/extrinsic: " << failure << '\n';
    return 1;
  }

  std::unique_ptr<FastLio2IkfomFrontend> visual_position_update =
      candidate->cloneCandidate();
  const FilterSnapshot before_visual_position = visual_position_update->getState();
  const Eigen::Vector3d visual_position_measurement =
      before_visual_position.map_T_imu.position + Eigen::Vector3d(0.2, -0.1, 0.05);
  const Eigen::Matrix3d visual_noise = Eigen::Matrix3d::Identity() * 0.01;
  PoseCorrectionDelta visual_delta;
  if (!visual_position_update->applyPositionMeasurement(
          visual_position_measurement, visual_noise, &visual_delta, &failure)) {
    std::cerr << "FAIL: position-only IKFoM measurement: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot after_visual_position = visual_position_update->getState();
  if ((after_visual_position.map_T_imu.position - visual_position_measurement).norm() >=
          (before_visual_position.map_T_imu.position - visual_position_measurement).norm() ||
      visual_delta.position.norm() <= 0.0 ||
      !visual_position_update->postconditionsValid(&failure) ||
      (after_visual_position.covariance -
       after_visual_position.covariance.transpose()).norm() > 1e-8) {
    std::cerr << "FAIL: position-only update correction/covariance: " << failure << '\n';
    return 1;
  }

  std::unique_ptr<FastLio2IkfomFrontend> projected_position_update =
      candidate->cloneCandidate();
  std::unique_ptr<FastLio2IkfomFrontend> projected_position_update_variant =
      candidate->cloneCandidate();
  const FilterSnapshot before_projected_position =
      projected_position_update->getState();
  Eigen::Matrix3d y_only_basis = Eigen::Matrix3d::Zero();
  y_only_basis.col(0) = Eigen::Vector3d::UnitY();
  const Eigen::Matrix3d projected_noise = Eigen::Matrix3d::Identity() * 0.01;
  Eigen::Vector3d projected_measurement = before_projected_position.map_T_imu.position;
  projected_measurement.y() -= 0.2;
  Eigen::Vector3d projected_measurement_variant = projected_measurement;
  projected_measurement_variant.x() += 10.0;
  projected_measurement_variant.z() -= 7.0;
  PoseCorrectionDelta projected_delta;
  if (!projected_position_update->applyProjectedPositionMeasurement(
          projected_measurement, projected_noise, y_only_basis, 1,
          &projected_delta, &failure) ||
      !projected_position_update_variant->applyProjectedPositionMeasurement(
          projected_measurement_variant, projected_noise, y_only_basis, 1,
          nullptr, &failure)) {
    std::cerr << "FAIL: projected position-only IKFoM measurement: "
              << failure << '\n';
    return 1;
  }
  const FilterSnapshot after_projected_position =
      projected_position_update->getState();
  const FilterSnapshot after_projected_position_variant =
      projected_position_update_variant->getState();
  if (projected_delta.position.norm() <= 0.0 ||
      (after_projected_position.map_T_imu.position -
       after_projected_position_variant.map_T_imu.position).norm() > 1e-9 ||
      (after_projected_position.map_T_imu.orientation.coeffs() -
       after_projected_position_variant.map_T_imu.orientation.coeffs()).norm() > 1e-9 ||
      !projected_position_update->postconditionsValid(&failure)) {
    std::cerr << "FAIL: projected update used discarded measurement axes: "
              << failure << '\n';
    return 1;
  }
  std::unique_ptr<FastLio2IkfomFrontend> projected_pose_update =
      candidate->cloneCandidate();
  std::unique_ptr<FastLio2IkfomFrontend> projected_pose_update_variant =
      candidate->cloneCandidate();
  const FilterSnapshot before_projected_pose = projected_pose_update->getState();
  Pose3d projected_pose_measurement = before_projected_pose.map_T_imu;
  projected_pose_measurement.position.y() -= 0.15;
  projected_pose_measurement.position.x() += 2.0;
  projected_pose_measurement.orientation =
      (projected_pose_measurement.orientation * Eigen::Quaterniond(
          Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()))).normalized();
  Pose3d projected_pose_variant = projected_pose_measurement;
  projected_pose_variant.position.x() -= 4.0;
  projected_pose_variant.position.z() += 3.0;
  projected_pose_variant.orientation = before_projected_pose.map_T_imu.orientation;
  Eigen::Matrix<double, 6, 6> pose_basis =
      Eigen::Matrix<double, 6, 6>::Zero();
  pose_basis(1, 0) = 1.0;
  const Eigen::Matrix<double, 6, 6> pose_noise =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  PoseCorrectionDelta projected_pose_delta;
  if (!projected_pose_update->applyProjectedPoseMeasurement(
          projected_pose_measurement, pose_noise, pose_basis, 1,
          &projected_pose_delta, &failure) ||
      !projected_pose_update_variant->applyProjectedPoseMeasurement(
          projected_pose_variant, pose_noise, pose_basis, 1, nullptr, &failure)) {
    std::cerr << "FAIL: projected LiDAR pose update: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot projected_pose_state = projected_pose_update->getState();
  const FilterSnapshot projected_pose_state_variant =
      projected_pose_update_variant->getState();
  // A selected position residual may update orientation through EKF
  // cross-covariance. The invariant is that changing discarded measurement
  // axes cannot change the result, not that every unmeasured state block is
  // frozen.
  if (projected_pose_delta.position.norm() <= 0.0 ||
      (projected_pose_state.map_T_imu.position -
       projected_pose_state_variant.map_T_imu.position).norm() > 1e-9 ||
      (projected_pose_state.map_T_imu.orientation.coeffs() -
       projected_pose_state_variant.map_T_imu.orientation.coeffs()).norm() > 1e-9 ||
      (projected_pose_state.covariance -
       projected_pose_state_variant.covariance).norm() > 1e-9 ||
      !projected_pose_update->postconditionsValid(&failure)) {
    std::cerr << "FAIL: projected pose update used a discarded measurement direction: "
              << failure << '\n';
    return 1;
  }

  const double expected_chi99[] = {
      std::numeric_limits<double>::quiet_NaN(),
      6.635, 9.210, 11.345, 13.277, 15.086, 16.812};
  for (int rank = 1; rank <= 6; ++rank) {
    if (std::abs(chiSquare99Threshold(rank) - expected_chi99[rank]) > 1e-12) {
      std::cerr << "FAIL: rank-specific chi-square threshold: " << rank << '\n';
      return 1;
    }
  }
  if (std::isfinite(chiSquare99Threshold(0)) ||
      std::isfinite(chiSquare99Threshold(7))) {
    std::cerr << "FAIL: invalid rank returned a chi-square threshold\n";
    return 1;
  }

  Eigen::Matrix<double, 6, 6> y_basis = Eigen::Matrix<double, 6, 6>::Zero();
  y_basis(1, 0) = 1.0;
  Pose3d diagnostic_measurement = before_projected_pose.map_T_imu;
  diagnostic_measurement.position.y() += 0.03;
  const FilterSnapshot before_diagnostic = candidate->getState();
  ProjectedPoseInnovation diagnostic;
  if (!candidate->evaluateProjectedPoseInnovation(
          diagnostic_measurement, pose_noise, y_basis, 1, &diagnostic, &failure) ||
      !diagnostic.valid || diagnostic.rank != 1 ||
      diagnostic.status != "OK" || !std::isfinite(diagnostic.nis) ||
      !sameFilterSnapshot(before_diagnostic, candidate->getState())) {
    std::cerr << "FAIL: projected innovation read-only evaluation: " << failure << '\n';
    return 1;
  }

  Pose3d weak_direction_variant = diagnostic_measurement;
  weak_direction_variant.position.x() += 8.0;
  weak_direction_variant.position.z() -= 4.0;
  weak_direction_variant.orientation =
      (weak_direction_variant.orientation * Eigen::Quaterniond(
          Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()))).normalized();
  ProjectedPoseInnovation variant_diagnostic;
  if (!candidate->evaluateProjectedPoseInnovation(
          weak_direction_variant, pose_noise, y_basis, 1,
          &variant_diagnostic, &failure) ||
      std::abs(diagnostic.projected_residual_norm -
               variant_diagnostic.projected_residual_norm) > 1e-9 ||
      std::abs(diagnostic.nis - variant_diagnostic.nis) > 1e-9) {
    std::cerr << "FAIL: selected NIS depends on discarded residual axes: "
              << failure << '\n';
    return 1;
  }

  Pose3d high_innovation = diagnostic_measurement;
  high_innovation.position.y() += 100.0;
  std::unique_ptr<FastLio2IkfomFrontend> gated_candidate = candidate->cloneCandidate();
  const FilterSnapshot before_gate = gated_candidate->getState();
  PoseCorrectionDelta gated_delta;
  gated_delta.position.setConstant(1.0);
  gated_delta.rotation.setConstant(1.0);
  gated_delta.velocity.setConstant(1.0);
  gated_delta.gyro_bias.setConstant(1.0);
  gated_delta.accel_bias.setConstant(1.0);
  gated_delta.gravity_tangent.setConstant(1.0);
  ProjectedPoseInnovation gated_diagnostic;
  if (gated_candidate->applyProjectedPoseMeasurementChecked(
          high_innovation, pose_noise, y_basis, 1, true,
          chiSquare99Threshold(1), &gated_diagnostic, &gated_delta, &failure) ||
      failure != "SELECTED_NIS_REJECTED" ||
      gated_diagnostic.status != "SELECTED_NIS_REJECTED" ||
      !sameFilterSnapshot(before_gate, gated_candidate->getState()) ||
      !gated_delta.position.isOnes() || !gated_delta.rotation.isOnes() ||
      !gated_delta.velocity.isOnes() || !gated_delta.gyro_bias.isOnes() ||
      !gated_delta.accel_bias.isOnes() || !gated_delta.gravity_tangent.isOnes()) {
    std::cerr << "FAIL: selected NIS rejection was not atomic: " << failure << '\n';
    return 1;
  }

  std::unique_ptr<FastLio2IkfomFrontend> ungated_candidate = candidate->cloneCandidate();
  PoseCorrectionDelta ungated_delta;
  if (!ungated_candidate->applyProjectedPoseMeasurement(
          high_innovation, pose_noise, y_basis, 1, &ungated_delta, &failure) ||
      ungated_delta.position.norm() <= 0.0) {
    std::cerr << "FAIL: gate-off legacy projected update: " << failure << '\n';
    return 1;
  }

  Eigen::Matrix<double, 6, 6> adaptive_noise =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  adaptive_noise(0, 0) = 0.09;
  Eigen::Matrix<double, 6, 6> diagonal_basis =
      Eigen::Matrix<double, 6, 6>::Zero();
  diagonal_basis(0, 0) = 1.0;
  ProjectedPoseInnovation fixed_noise_diagnostic;
  ProjectedPoseInnovation adaptive_noise_diagnostic;
  if (!candidate->evaluateProjectedPoseInnovation(
          diagnostic_measurement, pose_noise, diagonal_basis, 1,
          &fixed_noise_diagnostic, &failure) ||
      !candidate->evaluateProjectedPoseInnovation(
          diagnostic_measurement, adaptive_noise, diagonal_basis, 1,
          &adaptive_noise_diagnostic, &failure) ||
      std::abs(fixed_noise_diagnostic.projected_noise_trace - 0.01) > 1e-12 ||
      std::abs(adaptive_noise_diagnostic.projected_noise_trace - 0.09) > 1e-12) {
    std::cerr << "FAIL: projected covariance did not use selected noise: "
              << failure << '\n';
    return 1;
  }

  std::unique_ptr<FastLio2IkfomFrontend> shadow_branches[4];
  const FilterSnapshot before_shadow_branches = candidate->getState();
  const bool shadow_gates[] = {false, false, true, true};
  const Eigen::Matrix<double, 6, 6>* shadow_noises[] = {
      &pose_noise, &adaptive_noise, &pose_noise, &adaptive_noise};
  for (int index = 0; index < 4; ++index) {
    shadow_branches[index] = candidate->cloneCandidate();
    ProjectedPoseInnovation branch_diagnostic;
    const bool applied = shadow_branches[index]->applyProjectedPoseMeasurementChecked(
        diagnostic_measurement, *shadow_noises[index], y_basis, 1,
        shadow_gates[index], chiSquare99Threshold(1), &branch_diagnostic,
        nullptr, &failure);
    if (!applied) {
      std::cerr << "FAIL: cloned counterfactual update: " << failure << '\n';
      return 1;
    }
  }
  if (!sameFilterSnapshot(before_shadow_branches, candidate->getState())) {
    std::cerr << "FAIL: shadow branches mutated the official frontend\n";
    return 1;
  }

  Eigen::Matrix<double, 6, 6> invalid_checked_pose_noise = pose_noise;
  invalid_checked_pose_noise(1, 1) = -1.0;
  Eigen::Matrix<double, 6, 6> nonorthogonal_basis = y_basis;
  nonorthogonal_basis(1, 0) = 2.0;
  const FilterSnapshot before_invalid_pose = candidate->getState();
  ProjectedPoseInnovation invalid_diagnostic;
  if (candidate->applyProjectedPoseMeasurementChecked(
          diagnostic_measurement, invalid_checked_pose_noise, y_basis, 1, false, 0.0,
          &invalid_diagnostic, nullptr, &failure) ||
      !sameFilterSnapshot(before_invalid_pose, candidate->getState()) ||
      candidate->applyProjectedPoseMeasurementChecked(
          diagnostic_measurement, pose_noise, y_basis, 0, false, 0.0,
          &invalid_diagnostic, nullptr, &failure) ||
      !sameFilterSnapshot(before_invalid_pose, candidate->getState()) ||
      candidate->applyProjectedPoseMeasurementChecked(
          diagnostic_measurement, pose_noise, nonorthogonal_basis, 1, false, 0.0,
          &invalid_diagnostic, nullptr, &failure) ||
      !sameFilterSnapshot(before_invalid_pose, candidate->getState())) {
    std::cerr << "FAIL: invalid projected pose input was accepted or mutated state: "
              << failure << '\n';
    return 1;
  }

  Eigen::Matrix3d invalid_visual_noise = visual_noise;
  invalid_visual_noise(0, 0) = -1.0;
  const FilterSnapshot before_rejected = visual_position_update->getState();
  PoseCorrectionDelta rejected_delta;
  rejected_delta.position.setConstant(11.0);
  rejected_delta.rotation.setConstant(12.0);
  rejected_delta.velocity.setConstant(13.0);
  rejected_delta.gyro_bias.setConstant(14.0);
  rejected_delta.accel_bias.setConstant(15.0);
  rejected_delta.gravity_tangent.setConstant(16.0);
  const PoseCorrectionDelta delta_before_rejection = rejected_delta;
  if (visual_position_update->applyPositionMeasurement(
          visual_position_measurement, invalid_visual_noise, &rejected_delta, &failure) ||
      !sameFilterSnapshot(before_rejected, visual_position_update->getState()) ||
      (rejected_delta.position - delta_before_rejection.position).norm() > 1e-12 ||
      (rejected_delta.rotation - delta_before_rejection.rotation).norm() > 1e-12 ||
      (rejected_delta.velocity - delta_before_rejection.velocity).norm() > 1e-12 ||
      (rejected_delta.gyro_bias - delta_before_rejection.gyro_bias).norm() > 1e-12 ||
      (rejected_delta.accel_bias - delta_before_rejection.accel_bias).norm() > 1e-12 ||
      (rejected_delta.gravity_tangent - delta_before_rejection.gravity_tangent).norm() > 1e-12) {
    std::cerr << "FAIL: invalid position update must reject without state/delta mutation: "
              << failure << '\n';
    return 1;
  }

  Eigen::Matrix<double, 6, 6> invalid_pose_noise =
      Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  invalid_pose_noise(2, 2) = -1.0;
  Pose3d invalid_pose_measurement = before_rejected.map_T_imu;
  invalid_pose_measurement.position.x() += 0.4;
  if (visual_position_update->applyPoseMeasurement(
          invalid_pose_measurement, invalid_pose_noise, &rejected_delta, &failure) ||
      !sameFilterSnapshot(before_rejected, visual_position_update->getState()) ||
      (rejected_delta.position - delta_before_rejection.position).norm() > 1e-12) {
    std::cerr << "FAIL: invalid ordinary pose update must reject atomically: "
              << failure << '\n';
    return 1;
  }

  Eigen::Matrix<double, 6, 6> invalid_projected_basis =
      Eigen::Matrix<double, 6, 6>::Zero();
  invalid_projected_basis(0, 0) = 2.0;
  if (visual_position_update->applyProjectedPoseMeasurement(
          invalid_pose_measurement,
          Eigen::Matrix<double, 6, 6>::Identity() * 0.01,
          invalid_projected_basis, 1, &rejected_delta, &failure) ||
      !sameFilterSnapshot(before_rejected, visual_position_update->getState())) {
    std::cerr << "FAIL: invalid projected pose update must reject atomically: "
              << failure << '\n';
    return 1;
  }

  if (!committed.commitCandidate(*candidate, &failure) ||
      committed.getState().stamp_ns != scan_end_ns) {
    std::cerr << "FAIL: prediction-only candidate commit: " << failure << '\n';
    return 1;
  }
  const FilterSnapshot after_prediction_only = committed.getState();
  if ((after_prediction_only.map_T_imu.position - predicted.map_T_imu.position).norm() > 1e-12 ||
      (after_prediction_only.velocity - predicted.velocity).norm() > 1e-12) {
    std::cerr << "FAIL: prediction-only commit changed candidate state\n";
    return 1;
  }

  std::cout << "NDT_POSE_UPDATE_RUNTIME_CONTRACT_PASS"
            << " pos_before_m=" << position_error_before
            << " pos_after_m=" << position_error_after
            << " rot_before_rad=" << rotation_error_before
            << " rot_after_rad=" << rotation_error_after
            << " projected_update_m=" << projected_delta.position.norm()
            << " delta_v=" << delta.velocity.norm()
            << " delta_bg=" << delta.gyro_bias.norm()
            << " delta_ba=" << delta.accel_bias.norm()
            << " delta_gravity=" << delta.gravity_tangent.norm() << '\n';
  return 0;
}
