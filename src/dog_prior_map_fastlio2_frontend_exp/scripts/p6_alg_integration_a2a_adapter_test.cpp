#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"

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

ImuSample imu(std::uint64_t stamp_ns) {
  ImuSample sample;
  sample.stamp_ns = stamp_ns;
  sample.acceleration = Eigen::Vector3d(0.15, 0.0, 9.809);
  sample.angular_velocity = Eigen::Vector3d::Zero();
  return sample;
}

reliability::LocalRisk validRisk() {
  reliability::LocalRisk risk;
  risk.valid = true;
  risk.map_support_sufficient = true;
  risk.map_support_correspondences = 100;
  risk.map_support_effective_weight = 100.0;
  risk.translation_length_scale_m = 0.8;
  risk.weak_dimension = 1;
  risk.reliable_dimension = 5;
  risk.joint_weak_basis.setZero();
  risk.joint_weak_basis.col(0) = Eigen::Matrix<double, 6, 1>::Unit(3);
  risk.status = "SYNTHETIC_VALID_RISK";
  return risk;
}

FrozenLidarEvent lidar(std::uint64_t transaction_id, std::uint64_t stamp_ns) {
  FrozenLidarEvent event;
  event.transaction_id = transaction_id;
  event.stamp_ns = stamp_ns;
  event.map_T_lidar.position = Eigen::Vector3d(0.001 *
      static_cast<double>(stamp_ns / 1000000ULL), 0.0, 0.0);
  event.residual_covariance = 0.01 * Matrix6d::Identity();
  event.local_risk = validRisk();
  event.ndt_converged = true;
  event.map_support_valid = true;
  return event;
}

FrozenVisualEvent visual(std::uint64_t ref_ns, std::uint64_t cur_ns) {
  FrozenVisualEvent event;
  event.ref_ns = ref_ns;
  event.cur_ns = cur_ns;
  event.depth_ns = ref_ns;
  event.translation_ref_imu = Eigen::Vector3d(0.001, 0.0, 0.0);
  event.measurement_covariance = 0.01 * Eigen::Matrix3d::Identity();
  event.source_valid = true;
  event.quality.source_valid = true;
  event.quality.quality_metadata_available = true;
  event.quality.detected_count = 100;
  event.quality.tracked_count = 80;
  event.quality.pnp_inlier_count = 60;
  event.quality.inlier_ratio = 0.75;
  event.quality.depth_associated_count = 70;
  event.quality.depth_fraction = 0.70;
  event.quality.grid_occupancy = 0.60;
  event.quality.hull_fraction = 0.40;
  event.quality.median_parallax_px = 2.0;
  event.quality.reprojection_rmse_px = 0.5;
  return event;
}

void testCausalEventSequence() {
  FixedLagOptions options;
  options.maximum_duration_s = 0.55;
  options.maximum_nodes = 6;
  options.maximum_active_observation_ids = 32;
  options.maximum_optimizer_iterations = 5;
  ImuNoiseParameters noise;
  noise.gravity = Eigen::Vector3d(0.0, 0.0, -9.809);

  FixedLagAdapterCalibration calibration;
  calibration.reliability_config.maximum_visual_pair_gap_s = 10.0;
  FixedLagEventAdapter adapter(options, noise, calibration);
  WindowState initial;
  initial.stamp_ns = 1'000'000'000ULL;
  const Matrix15d information = 100.0 * Matrix15d::Identity();
  const Vector15d gradient = Vector15d::Zero();
  std::string reason;
  require(adapter.initialize(initial, information, gradient, &reason),
          "adapter initialization: " + reason);

  require(adapter.appendImu(imu(1'000'000'000ULL), &reason),
          "causal IMU anchor: " + reason);
  require(adapter.appendImu(imu(1'100'000'000ULL), &reason),
          "first causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(1, 1'100'000'000ULL), &reason),
          "first LiDAR event: " + reason);
  require(adapter.lastEventStatus().observation_id == 2,
          "one IMU interval and one LiDAR observation use unified IDs");
  require(!adapter.processLidarEvent(lidar(1, 1'100'000'000ULL), &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::DUPLICATE_SOURCE,
          "duplicate LiDAR source rejected before new ID");

  require(adapter.appendImu(imu(1'200'000'000ULL), &reason),
          "second causal IMU: " + reason);
  require(adapter.processVisualEvent(visual(1'100'000'000ULL,
                                             1'200'000'000ULL), &reason),
          "first visual event: " + reason);
  require(adapter.lastEventStatus().observation_id == 4,
          "visual factor receives the next global observation ID");
  require(!adapter.processVisualEvent(visual(1'100'000'000ULL,
                                               1'200'000'000ULL), &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::DUPLICATE_SOURCE,
          "duplicate visual pair rejected before new ID");

  require(adapter.appendImu(imu(1'300'000'000ULL), &reason),
          "third causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(2, 1'300'000'000ULL), &reason),
          "second LiDAR event: " + reason);
  require(adapter.processVisualEvent(visual(1'200'000'000ULL,
                                             1'300'000'000ULL), &reason),
          "second visual event: " + reason);

  require(adapter.appendImu(imu(1'400'000'000ULL), &reason),
          "fourth causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(3, 1'400'000'000ULL), &reason),
          "third LiDAR event: " + reason);
  require(!adapter.appendImu(imu(1'400'000'000ULL), &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::DUPLICATE_SOURCE,
          "duplicate IMU timestamp rejected");

  const bool first_optimization = adapter.optimizeCurrentWindow(&reason);
  require(first_optimization,
          "causal sequence optimization: " + reason);
  WindowState optimized;
  require(adapter.latestOptimizedState(&optimized, &reason),
          "optimized feedback is available only after success: " + reason);
  require(optimized.stamp_ns == 1'400'000'000ULL,
          "optimized state timestamp is the latest event timestamp");

  // An event between samples is rejected when its right boundary sample has
  // not arrived yet, then accepted after the sample becomes causal history.
  require(!adapter.processLidarEvent(lidar(4, 1'550'000'000ULL), &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::REJECTED_CAUSALITY,
          "future IMU is never silently consumed");
  require(adapter.appendImu(imu(1'500'000'000ULL), &reason),
          "fifth causal IMU: " + reason);
  require(!adapter.processLidarEvent(lidar(4, 1'550'000'000ULL), &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::REJECTED_CAUSALITY,
          "interpolation still requires an observed right boundary");
  require(adapter.appendImu(imu(1'600'000'000ULL), &reason),
          "sixth causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(4, 1'550'000'000ULL), &reason),
          "between-sample event after causal right sample: " + reason);

  const bool second_optimization = adapter.optimizeCurrentWindow(&reason);
  require(second_optimization,
          "post-interpolation optimization: " + reason);
  const WindowSummary pre_marginalization_summary = adapter.summary();
  require(pre_marginalization_summary.lidar_factor_count > 0 &&
              pre_marginalization_summary.visual_factor_count > 0,
          "all real event families entered the same window");
  require(adapter.appendImu(imu(1'700'000'000ULL), &reason),
          "seventh causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(5, 1'700'000'000ULL), &reason),
          "fourth LiDAR event: " + reason);
  require(adapter.appendImu(imu(1'800'000'000ULL), &reason),
          "eighth causal IMU: " + reason);
  require(adapter.processLidarEvent(lidar(6, 1'800'000'000ULL), &reason),
          "fifth LiDAR event: " + reason);
  const bool third_optimization = adapter.optimizeCurrentWindow(&reason);
  require(third_optimization,
          "marginalization optimization: " + reason);
  FrozenVisualEvent expired_visual = visual(1'100'000'000ULL,
                                            1'800'000'000ULL);
  require(!adapter.processVisualEvent(expired_visual, &reason) &&
              adapter.lastEventStatus().disposition ==
                  AdapterEventDisposition::REJECTED_WINDOW,
          "visual reference outside the active window is rejected");
  const WindowSummary summary = adapter.summary();
  require(summary.window_node_count <= options.maximum_nodes,
          "node limit is enforced after marginalization");
  require(summary.active_observation_id_count <=
              options.maximum_active_observation_ids,
          "active observation ID limit is bounded");
  require(adapter.sourceRecordCount() <= 15 &&
              adapter.lifecycleDiagnostics().expired_source_records_removed > 0,
          "expired source keys are pruned while active keys remain bounded");
}

void testSkipReasonsAndEndpointChecks() {
  FixedLagEventAdapter adapter;
  WindowState initial;
  initial.stamp_ns = 2'000'000'000ULL;
  std::string reason;
  require(adapter.initialize(initial, 10.0 * Matrix15d::Identity(),
                             Vector15d::Zero(), &reason),
          "second adapter initialization");
  FrozenLidarEvent invalid = lidar(10, initial.stamp_ns);
  invalid.ndt_converged = false;
  require(!adapter.processLidarEvent(invalid, &reason) &&
              reason == "ndt_not_converged",
          "NDT non-convergence is a skip reason, not a fake factor");
  require(adapter.summary().lidar_factor_count == 0,
          "skipped NDT event does not enter the objective");
  FrozenVisualEvent bad_visual = visual(initial.stamp_ns,
                                        initial.stamp_ns + 100'000'000ULL);
  bad_visual.quality.grid_occupancy = 0.0;
  require(!adapter.processVisualEvent(bad_visual, &reason) &&
              reason == "POOR_FEATURE_SPATIAL_DISTRIBUTION",
          "visual quality rejection is explicit");
  require(adapter.summary().visual_factor_count == 0,
          "rejected visual quality does not enter the objective");
}

}  // namespace

int main() {
  try {
    testCausalEventSequence();
    testSkipReasonsAndEndpointChecks();
    std::cout << "P6_ALG_INTEGRATION_A2A_ADAPTER_PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << "\n";
    return 1;
  }
}
