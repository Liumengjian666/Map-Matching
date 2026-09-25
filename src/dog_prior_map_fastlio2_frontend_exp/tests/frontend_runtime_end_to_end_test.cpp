#include "dog_prior_map_fastlio2_frontend_exp/frontend_runtime.hpp"

#include <iostream>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {

bool require(bool condition, const std::string& message) {
  if (!condition) std::cerr << "FAIL: " << message << '\n';
  return condition;
}

std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imuRange(
    uint64_t start_ns, uint64_t end_ns) {
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> samples;
  for (uint64_t stamp = start_ns; stamp <= end_ns; stamp += 5000000ULL) {
    ImuSample sample;
    sample.stamp_ns = stamp;
    sample.acceleration = Eigen::Vector3d(0.02, -0.01, 9.809);
    sample.angular_velocity = Eigen::Vector3d(0.003, -0.002, 0.015);
    samples.push_back(sample);
  }
  return samples;
}

std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloudFor(
    uint64_t start_ns, uint64_t end_ns) {
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloud;
  for (int i = 0; i < 24; ++i) {
    TimedLidarPoint point;
    point.stamp_ns = start_ns +
        static_cast<uint64_t>(i % 5) * (end_ns - start_ns) / 4;
    point.position = Eigen::Vector3d(2.0 + 0.1 * i, -0.8 + 0.04 * i,
                                     0.3 + 0.02 * (i % 7));
    point.intensity = static_cast<double>(i);
    cloud.push_back(point);
  }
  return cloud;
}

bool overlapBoundaryContractTest() {
  const auto point = [](uint64_t stamp_ns, int index) {
    TimedLidarPoint lidar_point;
    lidar_point.stamp_ns = stamp_ns;
    lidar_point.position = Eigen::Vector3d(2.0 + 0.1 * index,
                                           -0.8 + 0.04 * index,
                                           0.3 + 0.02 * (index % 7));
    lidar_point.intensity = static_cast<double>(index);
    return lidar_point;
  };

  std::string failure;
  ScanWindowDecision decision = ScanWindowDecision::PROCESS;
  ScanWindowStats window_stats;
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> stale_cloud = {
      point(99000000ULL, 0), point(199000000ULL, 1)};
  if (!require(prepareScanWindow(99000000ULL, 199000000ULL, 200000000ULL,
                                 &stale_cloud, &decision, &window_stats,
                                 &failure),
               "whole stale scan preparation: " + failure) ||
      !require(decision == ScanWindowDecision::SKIP_STALE &&
               stale_cloud.size() == 2 && window_stats.remaining_points == 2 &&
               window_stats.effective_scan_start_ns == 200000000ULL,
               "whole stale scan must be skipped without mutating its cloud"))
    return false;

  RuntimeParameters parameters;
  parameters.static_init_samples = 20;
  parameters.pose_position_sigma_m = 0.03;
  parameters.pose_rotation_sigma_rad = 0.02;
  FrontendRuntime runtime(parameters);
  const uint64_t init_start = 1000000000ULL;
  const auto init_imu = imuRange(init_start, init_start + 95000000ULL);
  Pose3d extrinsic;
  extrinsic.position = Eigen::Vector3d(0.08, 0.015, 0.03);
  extrinsic.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.orientation = extrinsic.orientation;
  if (!require(runtime.initializeStatic(init_imu, initial_map_T_lidar, extrinsic,
                                        &failure),
               "overlap-test static init: " + failure)) return false;

  const uint64_t scan1_start = runtime.committedState().stamp_ns;
  const uint64_t scan1_end = scan1_start + 100000000ULL;
  const uint64_t raw_scan2_start = scan1_end - 35000ULL;
  const uint64_t scan2_end = scan1_end + 100000000ULL;
  const uint64_t scan3_end = scan2_end + 100000000ULL;
  ScanEndResult output;

  if (!require(runtime.beginScan(1, scan1_start, scan1_end,
                                 imuRange(scan1_start, scan1_end),
                                 cloudFor(scan1_start, scan1_end), &output,
                                 &failure),
               "overlap-test scan1 begin: " + failure) ||
      !require(runtime.finishScan(1, RuntimeDisposition::SUCCESS, true,
                                  output.predicted_map_T_lidar, nullptr, &failure),
               "overlap-test scan1 commit: " + failure))
    return false;
  const uint64_t committed_after_scan1 = runtime.committedState().stamp_ns;

  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> runtime_stale = {
      point(committed_after_scan1 - 101000000ULL, 0),
      point(committed_after_scan1 - 1000000ULL, 1)};
  const RuntimeCounters before_stale = runtime.counters();
  if (!require(prepareScanWindow(runtime_stale.front().stamp_ns,
                                 runtime_stale.back().stamp_ns,
                                 committed_after_scan1, &runtime_stale,
                                 &decision, &window_stats, &failure),
               "runtime stale scan preparation: " + failure) ||
      !require(decision == ScanWindowDecision::SKIP_STALE &&
               runtime.counters().last_committed_transaction ==
                   before_stale.last_committed_transaction &&
               runtime.committedState().stamp_ns == committed_after_scan1,
               "stale scan must not create a transaction or advance filter time"))
    return false;

  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> overlap_cloud = {
      point(raw_scan2_start, 0),
      point(scan1_end - 20000ULL, 1),
      point(scan1_end, 2),
      point(scan1_end + 20000000ULL, 3),
      point(scan2_end, 4)};
  if (!require(prepareScanWindow(raw_scan2_start, scan2_end, scan1_end,
                                 &overlap_cloud, &decision, &window_stats,
                                 &failure),
               "partial overlap preparation: " + failure) ||
      !require(decision == ScanWindowDecision::PROCESS &&
               window_stats.effective_scan_start_ns == scan1_end &&
               window_stats.overlap_duration_ns == 35000ULL &&
               window_stats.overlap_points_dropped == 2 &&
               window_stats.remaining_points == 3 && overlap_cloud.size() == 3,
               "35us overlap must drop only points before committed time"))
    return false;

  if (!require(runtime.beginScan(2, window_stats.effective_scan_start_ns,
                                 scan2_end, imuRange(scan1_end, scan2_end),
                                 overlap_cloud, &output, &failure),
               "overlap-test scan2 transaction: " + failure) ||
      !require(output.imu_poses.front().stamp_ns == scan1_end &&
               output.imu_poses.back().stamp_ns == scan2_end &&
               output.scan_end_ns == scan2_end,
               "overlap scan propagation must begin at commit and end exactly"))
    return false;
  if (!require(runtime.finishScan(2, RuntimeDisposition::SUCCESS, true,
                                  output.predicted_map_T_lidar, nullptr, &failure),
               "overlap-test scan2 commit: " + failure) ||
      !require(runtime.committedState().stamp_ns == scan2_end,
               "overlap scan must commit at its physical scan end"))
    return false;

  if (!require(runtime.beginScan(3, scan2_end, scan3_end,
                                 imuRange(scan2_end, scan3_end),
                                 cloudFor(scan2_end, scan3_end), &output,
                                 &failure),
               "overlap-test scan3 continuation: " + failure) ||
      !require(runtime.finishScan(3, RuntimeDisposition::SUCCESS, true,
                                  output.predicted_map_T_lidar, nullptr, &failure),
               "overlap-test scan3 commit: " + failure))
    return false;

  return require(!runtime.fatal() &&
                 runtime.counters().last_committed_transaction == 3 &&
                 runtime.committedState().stamp_ns == scan3_end,
                 "scan after an overlapping scan must continue normally");
}

}  // namespace

int main() {
  if (!overlapBoundaryContractTest()) return 1;
  RuntimeParameters parameters;
  parameters.static_init_samples = 20;
  parameters.pose_position_sigma_m = 0.03;
  parameters.pose_rotation_sigma_rad = 0.02;
  FrontendRuntime runtime(parameters);
  const uint64_t init_start = 1000000000ULL;
  const auto init_imu = imuRange(init_start, init_start + 95000000ULL);
  Pose3d extrinsic;
  extrinsic.position = Eigen::Vector3d(0.08, 0.015, 0.03);
  extrinsic.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitY()));
  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.position = Eigen::Vector3d(1.2, -0.4, 0.0);
  initial_map_T_lidar.orientation = extrinsic.orientation;
  std::string failure;
  if (!require(runtime.initializeStatic(init_imu, initial_map_T_lidar, extrinsic,
                                        &failure),
               "static init: " + failure)) return 1;

  const uint64_t init_end = runtime.committedState().stamp_ns;
  const uint64_t scan_duration = 100000000ULL;
  const uint64_t scan1_end = init_end + scan_duration;
  const uint64_t scan2_end = scan1_end + scan_duration;
  const uint64_t scan3_end = scan2_end + scan_duration;
  ScanEndResult output1, output2, output3;

  auto imu1 = imuRange(init_end, scan1_end);
  if (!require(runtime.beginScan(1, init_end, scan1_end, imu1,
                                 cloudFor(init_end, scan1_end), &output1,
                                 &failure),
               "scan1 candidate: " + failure)) return 1;
  Pose3d success_pose = output1.predicted_map_T_lidar;
  success_pose.position += Eigen::Vector3d(0.07, -0.025, 0.01);
  PoseCorrectionDelta delta;
  if (!require(runtime.finishScan(1, RuntimeDisposition::SUCCESS, true,
                                  success_pose, &delta, &failure),
               "scan1 success commit: " + failure)) return 1;
  const FilterSnapshot scan1_committed = runtime.committedState();
  if (!require(scan1_committed.stamp_ns == scan1_end &&
               runtime.counters().measurement_updates == 1 &&
               runtime.counters().prediction_only_commits == 0,
               "scan1 must commit exactly one measurement update")) return 1;

  auto imu2 = imuRange(scan1_end, scan2_end);
  if (!require(runtime.beginScan(2, scan1_end, scan2_end, imu2,
                                 cloudFor(scan1_end, scan2_end), &output2,
                                 &failure),
               "scan2 candidate: " + failure)) return 1;
  if (!require(runtime.finishScan(2,
                                  RuntimeDisposition::REJECT_INSUFFICIENT_POINTS,
                                  false, Pose3d(), nullptr, &failure),
               "scan2 reject commit: " + failure)) return 1;
  const FilterSnapshot scan2_committed = runtime.committedState();
  if (!require(scan2_committed.stamp_ns == scan2_end &&
               runtime.counters().measurement_updates == 1 &&
               runtime.counters().prediction_only_commits == 1 &&
               (scan2_committed.map_T_imu.position -
                output2.predicted_map_T_imu.position).norm() < 1e-12,
               "scan2 reject must commit prediction and not update pose")) return 1;

  // The scan3 IMU buffer overlaps the prior scan by one sample, as a real
  // sensor ring buffer does. The candidate starts at the scan2 committed
  // watermark, so the prior scan interval is not integrated again.
  auto imu3 = imuRange(scan2_end - 5000000ULL, scan3_end - 1000000ULL);
  if (!require(runtime.beginScan(3, scan2_end, scan3_end, imu3,
                                 cloudFor(scan2_end, scan3_end), &output3,
                                 &failure),
               "scan3 candidate after rejected scan: " + failure)) return 1;
  if (!require(output3.imu_poses.front().stamp_ns == scan2_end &&
               output3.imu_poses.back().stamp_ns == scan3_end &&
               output3.predicted_map_T_imu.position.allFinite(),
               "scan3 must start at scan2 committed time and reach exact scan end")) return 1;
  if (!require(runtime.finishScan(3, RuntimeDisposition::REJECT_NOT_CONVERGED,
                                  false, Pose3d(), nullptr, &failure),
               "scan3 prediction-only commit: " + failure)) return 1;

  const RuntimeCounters counters = runtime.counters();
  if (!require(counters.last_committed_transaction == 3 &&
               counters.measurement_updates == 1 &&
               counters.prediction_only_commits == 2 &&
               runtime.committedState().stamp_ns == scan3_end && !runtime.fatal(),
               "transaction order/counters/final state")) return 1;

  std::cout << "OVERLAP_BOUNDARY_CONTRACT_PASS\n"
            << "FRONTEND_RUNTIME_END_TO_END_PASS"
            << " measurement_updates=" << counters.measurement_updates
            << " prediction_only_commits=" << counters.prediction_only_commits
            << " last_tx=" << counters.last_committed_transaction
            << " scan1_stamp=" << scan1_committed.stamp_ns
            << " scan2_stamp=" << scan2_committed.stamp_ns
            << " scan3_stamp=" << runtime.committedState().stamp_ns << '\n';
  return 0;
}
