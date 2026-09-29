#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_experiment.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <iostream>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {

Eigen::Matrix3d exp3(const Eigen::Vector3d& vector) {
  const double angle = vector.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, vector / angle).toRotationMatrix();
}

WindowState truthState(std::uint64_t stamp_ns, double t) {
  WindowState state;
  state.stamp_ns = stamp_ns;
  state.position = Eigen::Vector3d(0.5 * t * t, 0.1 * t, 0.0);
  state.velocity = Eigen::Vector3d(t, 0.1, 0.0);
  return state;
}

std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> makeImu(
    std::uint64_t start_ns, std::uint64_t end_ns) {
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples;
  for (std::uint64_t stamp = start_ns; stamp <= end_ns; stamp += 50000000ULL) {
    ImuSample sample;
    sample.stamp_ns = stamp;
    sample.acceleration = Eigen::Vector3d(1.0, 0.0, 0.0);
    sample.angular_velocity = Eigen::Vector3d::Zero();
    samples.push_back(sample);
  }
  if (samples.back().stamp_ns != end_ns) {
    ImuSample sample = samples.back();
    sample.stamp_ns = end_ns;
    samples.push_back(sample);
  }
  return samples;
}

bool require(bool condition, const char* message) {
  if (!condition) std::cerr << "FAIL: " << message << "\n";
  return condition;
}

}  // namespace

int main() {
  std::string reason;
  int basis_relinearization_count = 0;
  const auto attachFiveDofBasis = [&](LidarWindowMeasurement* measurement) {
    measurement->basis_relinearizer =
        [&basis_relinearization_count](const WindowState& state, Matrix6d* basis,
                                       int* rank, std::string* local_reason) {
          ++basis_relinearization_count;
          if (!basis || !rank || !state.rotation.allFinite()) {
            if (local_reason) *local_reason = "invalid_synthetic_basis_input";
            return false;
          }
          *basis = Matrix6d::Identity();
          *rank = 5;
          return true;
        };
  };
  FixedLagExperimentalController formal_controller;
  WindowState formal_state;
  formal_state.stamp_ns = 1;
  if (!require(!formal_controller.enabled() &&
             !formal_controller.addState(formal_state, &reason),
             "formal FULL isolation")) return 1;
  FixedLagExperimentalController experimental_controller(
      WindowExecutionMode::FULL_FIXED_LAG_EXPERIMENTAL);
  if (!require(experimental_controller.enabled(), "experimental mode enable")) return 2;
  constexpr std::uint64_t start_ns = 1000000000ULL;
  const auto imu = makeImu(start_ns, start_ns + 3000000000ULL);
  ImuNoiseParameters noise;
  noise.gravity.setZero();
  ImuPreintegratedMeasurement one_second;
  if (!require(preintegrateImu(imu, start_ns, start_ns + 1000000000ULL,
                               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               noise, &one_second, &reason),
               "preintegration failed")) return 3;
  if (!require(std::abs(one_second.delta_velocity.x() - 1.0) < 1e-8,
             "constant acceleration delta velocity")) return 4;
  if (!require(std::abs(one_second.delta_position.x() - 0.5) < 1e-8,
             "constant acceleration delta position")) return 5;
  if (!require(one_second.jacobian_rotation_gyro_bias.norm() > 0.0 &&
               one_second.covariance.allFinite(),
               "bias Jacobians and covariance")) return 6;

  WindowState reference = truthState(start_ns, 0.0);
  WindowState current = truthState(start_ns + 1000000000ULL, 1.0);
  VisualRelativeMeasurement visual;
  visual.valid = true;
  visual.observation_id = 9001;
  visual.reference_stamp_ns = reference.stamp_ns;
  visual.current_stamp_ns = current.stamp_ns;
  visual.reference_imu_translation = current.position - reference.position;
  visual.covariance = 0.01 * 0.01 * Eigen::Matrix3d::Identity();
  Eigen::Vector3d visual_residual;
  Eigen::Matrix<double, 3, 15> visual_jacobian_reference, visual_jacobian_current;
  if (!require(linearizeVisualFactor(reference, current, visual, &visual_residual,
                                    &visual_jacobian_reference,
                                    &visual_jacobian_current, &reason),
               "visual factor linearization")) return 7;
  const double visual_fd_translation =
      (visual_jacobian_reference.block<3, 3>(0, 3) +
       Eigen::Matrix3d::Identity()).norm();
  if (!require(visual_fd_translation < 1e-12 &&
               (visual_jacobian_current.block<3, 3>(0, 3) -
                Eigen::Matrix3d::Identity()).norm() < 1.0e-12,
               "visual translation Jacobian")) return 8;
  const Eigen::Vector3d rotation_probe(1e-6, 0.0, 0.0);
  WindowState ref_plus = reference;
  WindowState ref_minus = reference;
  ref_plus.rotation = reference.rotation * exp3(rotation_probe);
  ref_minus.rotation = reference.rotation * exp3(-rotation_probe);
  Eigen::Vector3d residual_plus, residual_minus;
  if (!buildVisualResidual(ref_plus, current, visual, &residual_plus, &reason) ||
      !buildVisualResidual(ref_minus, current, visual, &residual_minus, &reason))
    return 9;
  const Eigen::Vector3d finite_difference = (residual_plus - residual_minus) /
      (2.0e-6);
  if (!require((finite_difference -
                visual_jacobian_reference.block<3, 3>(0, 0).col(0)).norm() < 2e-5,
               "visual rotation central finite difference")) return 10;
  Eigen::Matrix<double, 3, 30> visual_joint_jacobian;
  visual_joint_jacobian << visual_jacobian_reference, visual_jacobian_current;
  const Eigen::Matrix3d visual_information = visual.covariance.ldlt().solve(
      Eigen::Matrix3d::Identity());
  const Eigen::Matrix<double, 30, 30> visual_joint_information =
      visual_joint_jacobian.transpose() * visual_information *
      visual_joint_jacobian;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 30, 30>> visual_solver(
      visual_joint_information);
  const Eigen::Index visual_information_rank =
      (visual_solver.eigenvalues().array() > 1e-8).count();
  if (!require(visual_solver.info() == Eigen::Success &&
               visual_information_rank == 3 &&
               visual_joint_information.block<15, 15>(0, 15).norm() > 1e-8,
               "visual cross-state information rank")) return 29;

  FixedLagOptions options;
  options.maximum_nodes = 6;
  options.maximum_duration_s = 10.0;
  options.maximum_optimizer_iterations = 10;
  FixedLagWindow window(options, noise);
  constexpr int node_count = 9;
  for (int index = 0; index < node_count; ++index) {
    const double time_s = 0.25 * index;
    WindowState state = truthState(start_ns +
        static_cast<std::uint64_t>(time_s * 1e9), time_s);
    state.position += Eigen::Vector3d(0.15, -0.08, 0.04);
    state.velocity += Eigen::Vector3d(-0.1, 0.05, 0.02);
    if (index > 0) state.rotation = exp3(Eigen::Vector3d(0.01, -0.005, 0.002)) *
        state.rotation;
    if (!require(window.addState(state, &reason), "window state insertion")) return 11;
  }
  for (int index = 0; index + 1 < node_count; ++index) {
    const std::uint64_t from_stamp = start_ns +
        static_cast<std::uint64_t>(0.25 * index * 1e9);
    const std::uint64_t to_stamp = start_ns +
        static_cast<std::uint64_t>(0.25 * (index + 1) * 1e9);
    ImuPreintegratedMeasurement preintegrated;
    if (!require(preintegrateImu(imu, from_stamp, to_stamp,
                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                 noise, &preintegrated, &reason),
                 "window preintegration")) return 12;
    if (!require(window.addImuFactor(100 + index, from_stamp, to_stamp,
                                     preintegrated, &reason),
                 "window IMU factor insertion")) return 13;
  }
  for (int index = 1; index < node_count; ++index) {
    const double time_s = 0.25 * index;
    LidarWindowMeasurement lidar;
    lidar.valid = true;
    lidar.observation_id = 1000 + index;
    lidar.stamp_ns = start_ns + static_cast<std::uint64_t>(time_s * 1e9);
    const WindowState truth = truthState(lidar.stamp_ns, time_s);
    lidar.measured_position = truth.position;
    lidar.measured_rotation = truth.rotation;
    lidar.covariance = 0.02 * 0.02 * Eigen::Matrix<double, 6, 6>::Identity();
    lidar.reliable_rank = 5;
    attachFiveDofBasis(&lidar);
    if (!require(window.addLidarFactor(lidar, &reason), "window lidar factor insertion"))
      return 14;
  }
  LidarWindowMeasurement missing_basis;
  missing_basis.valid = true;
  missing_basis.observation_id = 8800;
  missing_basis.stamp_ns = start_ns + 250000000ULL;
  missing_basis.measured_position =
      truthState(missing_basis.stamp_ns, 0.25).position;
  missing_basis.measured_rotation = Eigen::Matrix3d::Identity();
  missing_basis.covariance =
      0.02 * 0.02 * Eigen::Matrix<double, 6, 6>::Identity();
  missing_basis.reliable_rank = 5;
  if (!require(!window.addLidarFactor(missing_basis, &reason) &&
               window.summary().lidar_factor_skipped_count == 1 &&
               reason == "lidar_basis_relinearizer_missing",
               "rank-deficient LiDAR without relinearizer is skipped"))
    return 30;
  for (int index = 0; index + 2 < node_count; index += 2) {
    VisualRelativeMeasurement factor = visual;
    factor.observation_id = 2000 + index;
    factor.reference_stamp_ns = start_ns +
        static_cast<std::uint64_t>(0.25 * index * 1e9);
    factor.current_stamp_ns = start_ns +
        static_cast<std::uint64_t>(0.25 * (index + 2) * 1e9);
    const WindowState reference_truth = truthState(factor.reference_stamp_ns,
                                                   0.25 * index);
    const WindowState current_truth = truthState(factor.current_stamp_ns,
                                                 0.25 * (index + 2));
    factor.reference_imu_translation = current_truth.position -
        reference_truth.position;
    if (!require(window.addVisualFactor(factor, &reason),
                 "window visual factor insertion")) return 15;
  }
  LidarWindowMeasurement duplicate;
  duplicate.valid = true;
  duplicate.observation_id = 1001;
  duplicate.stamp_ns = start_ns + 250000000ULL;
  duplicate.measured_position = truthState(duplicate.stamp_ns, 0.25).position;
  duplicate.measured_rotation = Eigen::Matrix3d::Identity();
  duplicate.covariance = 0.02 * 0.02 * Eigen::Matrix<double, 6, 6>::Identity();
  duplicate.reliable_rank = 5;
  attachFiveDofBasis(&duplicate);
  if (!require(!window.addLidarFactor(duplicate, &reason) &&
               window.summary().duplicate_measurement_count == 1,
               "duplicate observation rejection")) return 16;

  const Eigen::Vector3d initial_error =
      window.latestState()->position - truthState(window.latestState()->stamp_ns,
                                                   0.25 * (node_count - 1)).position;
  if (!require(window.optimize(&reason), "joint optimization")) return 17;
  const Eigen::Vector3d final_error =
      window.latestState()->position - truthState(window.latestState()->stamp_ns,
                                                   0.25 * (node_count - 1)).position;
  const WindowSummary optimized = window.summary();
  if (!require(optimized.optimizer_final_cost < optimized.optimizer_initial_cost &&
               final_error.norm() < initial_error.norm() &&
               optimized.hessian_dimension == node_count * 15 &&
               optimized.hessian_numerical_rank > 0 &&
               basis_relinearization_count > node_count - 1,
               "joint objective and Hessian structure")) return 18;

  if (!require(window.marginalizeIfNeeded(&reason), "Schur marginalization")) return 19;
  const WindowSummary marginalized = window.summary();
  if (!require(marginalized.window_node_count <= 6 &&
               marginalized.marginalization_psd &&
               window.priorInformation().rows() == 6 * 15 &&
               marginalized.retained_prior_cross_information_norm > 1e-10,
               "bounded window and retained cross information")) return 20;
  WindowState feedback;
  if (!require(window.predictionFeedbackSeed(&feedback, &reason) &&
               feedback.stamp_ns == marginalized.latest_state_timestamp &&
               feedback.position.allFinite(), "prediction feedback seed")) return 21;

  // Process the identical synthetic observations with an incrementally
  // marginalized six-node window, then compare it with the nine-node batch
  // solution above. This is an information-retention check, not a claim that
  // marginalization is exactly invariant under nonlinear relinearization.
  FixedLagWindow incremental_window(options, noise);
  for (int index = 0; index < node_count; ++index) {
    const double time_s = 0.25 * index;
    const std::uint64_t stamp = start_ns +
        static_cast<std::uint64_t>(time_s * 1e9);
    WindowState state = truthState(stamp, time_s);
    state.position += Eigen::Vector3d(0.15, -0.08, 0.04);
    state.velocity += Eigen::Vector3d(-0.1, 0.05, 0.02);
    if (index > 0)
      state.rotation = exp3(Eigen::Vector3d(0.01, -0.005, 0.002)) *
          state.rotation;
    if (!require(incremental_window.addState(state, &reason),
                 "incremental window state insertion")) return 22;
    if (index > 0) {
      const std::uint64_t previous_stamp = start_ns +
          static_cast<std::uint64_t>(0.25 * (index - 1) * 1e9);
      ImuPreintegratedMeasurement preintegrated;
      if (!require(preintegrateImu(imu, previous_stamp, stamp,
                                   Eigen::Vector3d::Zero(),
                                   Eigen::Vector3d::Zero(), noise,
                                   &preintegrated, &reason) &&
                   incremental_window.addImuFactor(
                       5000 + index, previous_stamp, stamp, preintegrated,
                       &reason),
                   "incremental IMU factor")) return 23;
      LidarWindowMeasurement lidar;
      lidar.valid = true;
      lidar.observation_id = 6000 + index;
      lidar.stamp_ns = stamp;
      const WindowState truth = truthState(stamp, time_s);
      lidar.measured_position = truth.position;
      lidar.measured_rotation = truth.rotation;
      lidar.covariance =
          0.02 * 0.02 * Eigen::Matrix<double, 6, 6>::Identity();
      lidar.reliable_rank = 5;
      attachFiveDofBasis(&lidar);
      if (!require(incremental_window.addLidarFactor(lidar, &reason),
                   "incremental LiDAR factor")) return 24;
    }
    if (index >= 2 && index % 2 == 0) {
      VisualRelativeMeasurement factor = visual;
      factor.observation_id = 7000 + index;
      factor.reference_stamp_ns = start_ns +
          static_cast<std::uint64_t>(0.25 * (index - 2) * 1e9);
      factor.current_stamp_ns = stamp;
      const WindowState reference_truth = truthState(
          factor.reference_stamp_ns, 0.25 * (index - 2));
      const WindowState current_truth = truthState(stamp, time_s);
      factor.reference_imu_translation = current_truth.position -
          reference_truth.position;
      if (!require(incremental_window.addVisualFactor(factor, &reason),
                   "incremental visual factor")) return 25;
    }
    if (index > 0 && !incremental_window.optimize(&reason)) {
      std::cerr << "FAIL: incremental joint optimization: " << reason << "\n";
      return 26;
    }
    if (index > 0 && !incremental_window.marginalizeIfNeeded(&reason)) {
      std::cerr << "FAIL: incremental Schur marginalization: " << reason << "\n";
      return 26;
    }
  }
  const WindowState* incremental_latest = incremental_window.latestState();
  if (!incremental_latest) return 27;
  const double batch_window_position_delta =
      (incremental_latest->position - feedback.position).norm();
  const double incremental_truth_error =
      (incremental_latest->position -
       truthState(incremental_latest->stamp_ns,
                  0.25 * (node_count - 1)).position).norm();
  if (!require(batch_window_position_delta < 5e-3 &&
               incremental_truth_error < 5e-3,
               "incremental window versus full-batch consistency")) return 28;

  std::cout << "I6F_JOINT_WINDOW_TEST_PASS"
            << " initial_cost=" << optimized.optimizer_initial_cost
            << " final_cost=" << optimized.optimizer_final_cost
            << " initial_latest_error=" << initial_error.norm()
            << " final_latest_error=" << final_error.norm()
            << " hessian_rank=" << optimized.hessian_numerical_rank
            << " marginalized_nodes=" << marginalized.window_node_count
            << " retained_prior_cross_norm="
            << marginalized.retained_prior_cross_information_norm
            << " duplicate_count=" << marginalized.duplicate_measurement_count
            << " basis_relinearizations=" << basis_relinearization_count
            << " visual_information_rank=" << visual_information_rank
            << " batch_window_position_delta=" << batch_window_position_delta
            << " incremental_truth_error=" << incremental_truth_error
            << "\n";
  return 0;
}
