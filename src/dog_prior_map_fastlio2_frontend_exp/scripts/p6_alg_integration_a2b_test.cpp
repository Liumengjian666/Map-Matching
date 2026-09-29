#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/lidar_residual_math.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Eigen::Matrix3d exp3(const Eigen::Vector3d& v) {
  const double angle = v.norm();
  return angle < 1e-12 ? Eigen::Matrix3d::Identity() + lidarSkew(v) :
      Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

Eigen::Matrix<double, 6, 1> residual(
    const WindowState& state, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& position) {
  Eigen::Matrix<double, 6, 1> value;
  value.head<3>() = position - state.position;
  value.tail<3>() = lidarSo3Log(state.rotation.transpose() * rotation);
  return value;
}

void testAnalyticJacobian() {
  WindowState state;
  state.stamp_ns = 1;
  state.rotation = exp3(Eigen::Vector3d(0.15, -0.08, 0.11));
  state.position = Eigen::Vector3d(0.3, -0.4, 0.2);
  const Eigen::Matrix3d measured_rotation =
      exp3(Eigen::Vector3d(-0.12, 0.18, 0.07));
  const Eigen::Vector3d measured_position(1.2, -0.6, 0.9);
  const Eigen::Vector3d t_il(0.23, -0.17, 0.08);
  constexpr double scale = 0.8;
  Matrix6d analytic;
  std::string reason;
  require(normalizedLidarResidualJacobian(
              state, measured_rotation, t_il, scale, &analytic, &reason),
          "analytic Jacobian: " + reason);
  Matrix6d fd = Matrix6d::Zero();
  constexpr double h = 5e-7;
  const Eigen::Vector3d lidar_origin = measured_position + measured_rotation * t_il;
  for (int axis = 0; axis < 6; ++axis) {
    Eigen::Vector3d unit = Eigen::Vector3d::Zero();
    unit(axis % 3) = 1.0;
    Eigen::Matrix3d plus_rotation = measured_rotation;
    Eigen::Matrix3d minus_rotation = measured_rotation;
    Eigen::Vector3d plus_lidar = lidar_origin;
    Eigen::Vector3d minus_lidar = lidar_origin;
    if (axis < 3) {
      plus_rotation = exp3(h * unit) * measured_rotation;
      minus_rotation = exp3(-h * unit) * measured_rotation;
    } else {
      plus_lidar += h * scale * unit;
      minus_lidar -= h * scale * unit;
    }
    const Eigen::Vector3d plus_position = plus_lidar - plus_rotation * t_il;
    const Eigen::Vector3d minus_position = minus_lidar - minus_rotation * t_il;
    fd.col(axis) = (residual(state, plus_rotation, plus_position) -
                    residual(state, minus_rotation, minus_position)) / (2.0 * h);
  }
  require((analytic - fd).norm() < 2e-7,
          "six-axis analytic/FD mismatch: " +
              std::to_string((analytic - fd).norm()));

  Matrix6d weak = Matrix6d::Zero();
  weak.col(0) = Eigen::Matrix<double, 6, 1>::Unit(0);
  weak.col(1) = Eigen::Matrix<double, 6, 1>::Unit(4);
  Matrix6d reliable;
  int rank = 0;
  require(reliability::buildReliableMeasurementBasisFromWeak(
              analytic, weak, 2, &reliable, &rank, &reason),
          "multi-axis reliable basis: " + reason);
  require(rank == 4 &&
              (reliable.leftCols(rank).transpose() * analytic *
               weak.leftCols(2)).norm() < 1e-9,
          "weak-direction leakage");

  WindowState near_zero = state;
  near_zero.rotation = measured_rotation;
  require(normalizedLidarResidualJacobian(
              near_zero, measured_rotation, t_il, scale, &analytic, &reason),
          "near-zero rotation residual");
  WindowState near_pi = state;
  near_pi.rotation = measured_rotation *
      exp3(Eigen::Vector3d(M_PI - 5e-5, 0.0, 0.0));
  require(!normalizedLidarResidualJacobian(
              near_pi, measured_rotation, t_il, scale, &analytic, &reason) &&
              reason == "ROTATION_RESIDUAL_NEAR_PI",
          "near-pi residual rejected explicitly");
}

ImuSample imu(std::uint64_t stamp_ns) {
  ImuSample sample;
  sample.stamp_ns = stamp_ns;
  sample.acceleration = Eigen::Vector3d(0.0, 0.0, 9.809);
  return sample;
}

reliability::LocalRisk riskForAxis(int axis) {
  reliability::LocalRisk risk;
  risk.valid = true;
  risk.map_support_sufficient = true;
  risk.translation_length_scale_m = 0.8;
  risk.weak_dimension = 1;
  risk.reliable_dimension = 5;
  risk.joint_weak_basis.setZero();
  risk.joint_weak_basis.col(0) = Eigen::Matrix<double, 6, 1>::Unit(axis);
  return risk;
}

FrozenLidarEvent lidar(std::uint64_t id, std::uint64_t stamp, int weak_axis) {
  FrozenLidarEvent event;
  event.transaction_id = id;
  event.stamp_ns = stamp;
  event.residual_covariance = 0.01 * Matrix6d::Identity();
  event.local_risk = riskForAxis(weak_axis);
  event.ndt_converged = true;
  event.map_support_valid = true;
  return event;
}

FrozenVisualEvent visual(std::uint64_t ref, std::uint64_t cur) {
  FrozenVisualEvent event;
  event.ref_ns = ref;
  event.cur_ns = cur;
  event.depth_ns = ref;
  event.translation_ref_imu = Eigen::Vector3d(0.01, 0.0, 0.0);
  event.measurement_covariance = 0.01 * Eigen::Matrix3d::Identity();
  event.source_valid = true;
  event.quality.quality_metadata_available = true;
  event.quality.detected_count = 100;
  event.quality.tracked_count = 80;
  event.quality.depth_associated_count = 70;
  event.quality.pnp_inlier_count = 60;
  event.quality.inlier_ratio = 0.75;
  event.quality.depth_fraction = 0.70;
  event.quality.grid_occupancy = 0.6;
  event.quality.hull_fraction = 0.4;
  event.quality.median_parallax_px = 2.0;
  event.quality.reprojection_rmse_px = 0.5;
  return event;
}

FixedLagEventAdapter makeAdapter(const FixedLagOptions& options = {}) {
  ImuNoiseParameters noise;
  noise.gravity = Eigen::Vector3d(0.0, 0.0, -9.809);
  FixedLagAdapterCalibration calibration;
  calibration.reliability_config.maximum_visual_pair_gap_s = 10.0;
  return FixedLagEventAdapter(options, noise, calibration);
}

void initialize(FixedLagEventAdapter* adapter, std::uint64_t stamp) {
  WindowState state;
  state.stamp_ns = stamp;
  std::string reason;
  require(adapter->initialize(state, 100.0 * Matrix15d::Identity(),
                              Vector15d::Zero(), &reason),
          "adapter initialize: " + reason);
}

void testAtomicAndQuality() {
  FixedLagWindow window;
  WindowState initial;
  initial.stamp_ns = 1;
  Matrix15d invalid = Matrix15d::Identity();
  invalid(0, 0) = -1.0;
  std::string reason;
  require(!window.initializeWithPriorAtomic(
              initial, invalid, Vector15d::Zero(), &reason),
          "invalid initial prior rejected");
  require(window.states().empty(), "failed initialization leaves no state");
  require(window.initializeWithPriorAtomic(
              initial, Matrix15d::Identity(), Vector15d::Zero(), &reason),
          "initialization retry succeeds");
  WindowState next = initial;
  next.stamp_ns = 2;
  ImuPreintegratedMeasurement bad;
  bad.start_stamp_ns = 1;
  bad.end_stamp_ns = 2;
  bad.valid = true;
  const WindowSummary before = window.summary();
  require(!window.addStateWithImuFactorAtomic(next, 1, 1, bad, &reason),
          "singular factor covariance rejected atomically");
  const WindowSummary after = window.summary();
  require(window.states().size() == 1 &&
              before.window_revision == after.window_revision &&
              after.imu_factor_count == 0,
          "atomic failure leaves state/factor/revision unchanged");

  reliability::VisualQualityObservation observation;
  FrozenVisualEvent event = visual(10, 20);
  observation = event.quality;
  observation.source_valid = true;
  observation.reference_stamp_ns = 10;
  observation.current_stamp_ns = 20;
  observation.depth_stamp_ns = 10;
  observation.innovation_chi_square =
      std::numeric_limits<double>::quiet_NaN();
  const auto sensor = reliability::assessVisualSensorQuality(observation);
  require(sensor.passed && !sensor.prediction_consistent &&
              sensor.rejection_reason.find("NIS_PENDING") != std::string::npos,
          "sensor quality does not fake prediction NIS");
  require(!reliability::assessVisualQuality(observation).passed,
          "full quality still requires a real prediction NIS");
}

void testAsyncVisualAndDirectionalProjection() {
  FixedLagEventAdapter adapter = makeAdapter();
  initialize(&adapter, 1'000'000'000ULL);
  std::string reason;
  for (std::uint64_t stamp : {1'000'000'000ULL, 1'100'000'000ULL,
                              1'150'000'000ULL, 1'200'000'000ULL})
    require(adapter.appendImu(imu(stamp), &reason), "append IMU: " + reason);
  require(adapter.processLidarEvent(lidar(1, 1'100'000'000ULL, 3), &reason),
          "translation-weak LiDAR: " + reason);
  const auto covariance_diagnostics = adapter.lifecycleDiagnostics();
  require(covariance_diagnostics.last_imu_factor_covariance_regularized &&
              covariance_diagnostics.last_imu_regularization == 1e-9 &&
              covariance_diagnostics.last_imu_factor_min_eigenvalue >
                  covariance_diagnostics.last_imu_physical_min_eigenvalue &&
              covariance_diagnostics.last_imu_information_change_norm > 0.0,
          "factor-only covariance regularization is explicitly audited");
  require(adapter.processVisualReferenceStamp(1'150'000'000ULL, &reason),
          "non-LiDAR visual reference state: " + reason);
  const std::size_t before_nodes = adapter.summary().window_node_count;
  require(adapter.processVisualReferenceStamp(1'150'000'000ULL, &reason) &&
              adapter.summary().window_node_count == before_nodes,
          "same timestamp reference node reused");
  require(adapter.processVisualEvent(
              visual(1'150'000'000ULL, 1'200'000'000ULL), &reason),
          "asynchronous directional visual factor: " + reason);
  require(adapter.summary().visual_factor_count == 1,
          "directional visual factor enters common window");

  FixedLagEventAdapter rotation_only = makeAdapter();
  initialize(&rotation_only, 2'000'000'000ULL);
  for (std::uint64_t stamp : {2'000'000'000ULL, 2'100'000'000ULL,
                              2'200'000'000ULL})
    require(rotation_only.appendImu(imu(stamp), &reason), "rotation IMU");
  require(rotation_only.processLidarEvent(
              lidar(1, 2'100'000'000ULL, 0), &reason),
          "rotation-weak LiDAR");
  require(!rotation_only.processVisualEvent(
              visual(2'100'000'000ULL, 2'200'000'000ULL), &reason) &&
              reason == "NO_TRANSLATIONAL_COMPLEMENT",
          "pure rotation weakness creates no fake translation complement");

  VisualRelativeMeasurement measurement;
  measurement.valid = true;
  measurement.reference_stamp_ns = 1;
  measurement.current_stamp_ns = 2;
  measurement.mode = VisualFactorMode::LIDAR_WEAK_TRANSLATION;
  measurement.selected_rank = 1;
  measurement.measurement_basis.setZero();
  measurement.measurement_basis.col(0) = Eigen::Vector3d::UnitY();
  WindowState reference, current;
  reference.stamp_ns = 1;
  current.stamp_ns = 2;
  current.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::VectorXd selected_residual;
  Eigen::MatrixXd ji, jj, covariance;
  require(linearizeSelectedVisualFactor(
              reference, current, measurement, &selected_residual, &ji, &jj,
              &covariance, &reason) && selected_residual.size() == 1 &&
              std::abs(selected_residual(0) - 2.0) < 1e-12,
          "visual residual/Jacobians/covariance projected by Qw");
}

void testRetryAndMemoryLifecycle() {
  FixedLagEventAdapter retry = makeAdapter();
  initialize(&retry, 1'000'000'000ULL);
  std::string reason;
  require(retry.appendImu(imu(1'000'000'000ULL), &reason), "retry anchor");
  require(!retry.processVisualReferenceStamp(1'050'000'000ULL, &reason),
          "reference waits for causal right IMU");
  require(retry.appendImu(imu(1'100'000'000ULL), &reason), "retry right IMU");
  require(retry.processVisualReferenceStamp(1'050'000'000ULL, &reason),
          "reference retry succeeds after right IMU");

  FixedLagOptions options;
  options.maximum_nodes = 3;
  options.maximum_duration_s = 0.25;
  options.maximum_active_observation_ids = 32;
  FixedLagEventAdapter lifecycle = makeAdapter(options);
  constexpr std::uint64_t start = 3'000'000'000ULL;
  initialize(&lifecycle, start);
  for (int i = 0; i < 10000; ++i) {
    const std::uint64_t stamp = start + static_cast<std::uint64_t>(i) * 1'000'000ULL;
    require(lifecycle.appendImu(imu(stamp), &reason), "10k IMU append");
    if (i > 0 && i % 100 == 0) {
      require(lifecycle.processLidarEvent(
                  lidar(static_cast<std::uint64_t>(i / 100), stamp, 3), &reason),
              "10k lifecycle LiDAR: " + reason);
      require(lifecycle.optimizeCurrentWindow(&reason),
              "10k lifecycle optimize: " + reason);
    }
  }
  const auto diagnostics = lifecycle.lifecycleDiagnostics();
  require(lifecycle.imuSampleCount() < 400 &&
              lifecycle.sourceRecordCount() < 16 &&
              diagnostics.expired_source_records_removed > 0 &&
              diagnostics.peak_imu_buffer_size < 400,
          "10k lifecycle keeps IMU/source history bounded");
  std::cout << "A2B_MEMORY peak_imu_buffer_size="
            << diagnostics.peak_imu_buffer_size
            << " peak_active_source_records="
            << diagnostics.peak_active_source_records
            << " expired_source_records_removed="
            << diagnostics.expired_source_records_removed
            << " final_imu_buffer_size=" << lifecycle.imuSampleCount()
            << " final_active_source_records=" << lifecycle.sourceRecordCount()
            << "\n";

  FixedLagExperimentalController formal(
      WindowExecutionMode::FORMAL_FULL_LEGACY);
  WindowState state;
  state.stamp_ns = 1;
  require(!formal.addState(state, &reason) && !formal.enabled(),
          "formal FULL path remains unchanged and isolated");
}

}  // namespace

int main() {
  try {
    testAnalyticJacobian();
    testAtomicAndQuality();
    testAsyncVisualAndDirectionalProjection();
    testRetryAndMemoryLifecycle();
    std::cout << "P6_ALG_INTEGRATION_A2B_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << "\n";
    return 1;
  }
}
