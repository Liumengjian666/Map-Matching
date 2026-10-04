#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>

using namespace dog_prior_map_fastlio2_frontend_exp;

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.gravity_mps2 = 9.809;
  parameters.max_static_accel_std_m_s2 = 0.5;
  parameters.max_static_gyro_std_rad_s = 0.05;

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  for (uint64_t index = 0; index < 200; ++index) {
    ImuSample sample;
    sample.stamp_ns = 1000000000ULL + index * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2);
    stationary.push_back(sample);
  }
  FastLio2IkfomFrontend frontend(parameters);
  StaticImuCalibration calibration;
  std::string reason;
  if (!frontend.calibrateStaticImu(stationary, &calibration, &reason)) {
    std::cerr << "FAIL: static calibration: " << reason << '\n';
    return 1;
  }

  Pose3d map_T_imu;
  map_T_imu.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  map_T_imu.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  Pose3d T_imu_lidar;
  T_imu_lidar.position = Eigen::Vector3d(0.08, 0.029, 0.03);
  const uint64_t anchor_ns = 3000000000ULL;
  if (!frontend.initializeFromStaticCalibration(
          calibration, map_T_imu, T_imu_lidar,
          Eigen::Vector3d(0.0, 0.0, -parameters.gravity_mps2),
          Eigen::Vector3d::Zero(), anchor_ns, &reason)) {
    std::cerr << "FAIL: reanchor: " << reason << '\n';
    return 1;
  }

  const uint64_t scan_start_ns = anchor_ns - 9000000ULL;
  const uint64_t scan_end_ns = anchor_ns + 100000000ULL;
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imu;
  for (int step = -3; step <= 20; ++step) {
    ImuSample sample;
    sample.stamp_ns = anchor_ns + static_cast<int64_t>(step) * 5000000LL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2);
    imu.push_back(sample);
  }

  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloud;
  const std::array<uint64_t, 4> point_stamps{{
      scan_start_ns, static_cast<uint64_t>(anchor_ns - 2000000ULL),
      anchor_ns, scan_end_ns}};
  for (const uint64_t stamp : point_stamps) {
    TimedLidarPoint point;
    point.stamp_ns = stamp;
    point.position = Eigen::Vector3d(2.0, -0.5, 1.0);
    cloud.push_back(point);
  }

  ScanWindowStats stats;
  if (!prepareScanWindowWithCausalPreroll(scan_start_ns, scan_end_ns, anchor_ns,
                                           imu, &cloud, &stats, &reason)) {
    std::cerr << "FAIL: preroll window: " << reason << '\n';
    return 1;
  }
  if (stats.remaining_points != 4 || stats.overlap_points_dropped != 0 ||
      stats.effective_scan_start_ns != scan_start_ns ||
      stats.overlap_duration_ns != anchor_ns - scan_start_ns) {
    std::cerr << "FAIL: first scan points were not preserved\n";
    return 1;
  }

  ScanEndProcessor processor;
  auto missing_anchor_imu = imu;
  missing_anchor_imu.erase(std::remove_if(missing_anchor_imu.begin(), missing_anchor_imu.end(),
      [anchor_ns](const ImuSample& sample) { return sample.stamp_ns == anchor_ns; }),
      missing_anchor_imu.end());
  FastLio2IkfomFrontend missing_anchor_frontend(parameters);
  if (!missing_anchor_frontend.initializeFromStaticCalibration(
          calibration, map_T_imu, T_imu_lidar,
          Eigen::Vector3d(0.0, 0.0, -parameters.gravity_mps2),
          Eigen::Vector3d::Zero(), anchor_ns, &reason)) {
    std::cerr << "FAIL: missing-anchor reanchor: " << reason << '\n';
    return 1;
  }
  ScanEndResult missing_anchor_result;
  if (processor.process(&missing_anchor_frontend, T_imu_lidar, scan_start_ns,
                        scan_end_ns, missing_anchor_imu, cloud,
                        &missing_anchor_result, &reason, true) ||
      reason != "causal_preroll_requires_exact_anchor_imu_sample") {
    std::cerr << "FAIL: pre-anchor processing accepted an interpolated anchor sample\n";
    return 1;
  }

  ScanEndResult result;
  if (!processor.process(&frontend, T_imu_lidar, scan_start_ns, scan_end_ns,
                         imu, cloud, &result, &reason, true)) {
    std::cerr << "FAIL: causal preroll scan processing: " << reason << '\n';
    return 1;
  }
  if (result.imu_poses.empty() || result.imu_poses.front().stamp_ns != scan_start_ns ||
      result.imu_poses.back().stamp_ns != scan_end_ns ||
      result.cloud_end_frame.size() != cloud.size() ||
      frontend.getState().stamp_ns != scan_end_ns) {
    std::cerr << "FAIL: complete first scan was not causally deskewed to scan end\n";
    return 1;
  }
  for (std::size_t index = 1; index < result.imu_poses.size(); ++index) {
    if (result.imu_poses[index].stamp_ns <= result.imu_poses[index - 1].stamp_ns) {
      std::cerr << "FAIL: merged IMU pose history is not strictly increasing\n";
      return 1;
    }
  }
  for (const auto& point : result.cloud_end_frame)
    if (!point.position.allFinite()) {
      std::cerr << "FAIL: nonfinite deskewed point\n";
      return 1;
    }

  std::cout << "SCAN_PREROLL_CONTRACT_PASS points=" << result.cloud_end_frame.size()
            << " dropped=" << stats.overlap_points_dropped
            << " anchor_overlap_ns=" << stats.overlap_duration_ns
            << " pose_knots=" << result.imu_poses.size() << '\n';
  return 0;
}
