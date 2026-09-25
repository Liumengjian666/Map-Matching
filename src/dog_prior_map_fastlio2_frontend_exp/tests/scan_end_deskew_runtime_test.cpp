#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {

Eigen::Isometry3d toIsometry(const Pose3d& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.toRotationMatrix();
  transform.translation() = pose.position;
  return transform;
}

Pose3d truthMapTImu(uint64_t stamp, uint64_t start,
                    const Eigen::Vector3d& acceleration,
                    const Eigen::Vector3d& angular_velocity) {
  const double dt = static_cast<double>(stamp - start) * 1e-9;
  Pose3d pose;
  pose.position = 0.5 * acceleration * dt * dt;
  const Eigen::Vector3d rotation_vector = angular_velocity * dt;
  pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(
      rotation_vector.norm(), rotation_vector.norm() > 0.0
                                  ? rotation_vector.normalized()
                                  : Eigen::Vector3d::UnitX()));
  return pose;
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(values.size()))) - 1;
  return values.at(std::min(index, values.size() - 1));
}

}  // namespace

int main() {
  RuntimeParameters parameters;
  parameters.static_init_samples = 200;
  parameters.initial_accel_bias = Eigen::Vector3d(0.025, -0.018, 0.032);
  FastLio2IkfomFrontend committed(parameters);
  const Eigen::Vector3d gyro_bias(0.012, -0.021, 0.006);
  const Eigen::Vector3d accel_bias = parameters.initial_accel_bias;
  const uint64_t start_ns = 1000000000ULL;

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> stationary;
  stationary.reserve(parameters.static_init_samples);
  for (int i = 0; i < parameters.static_init_samples; ++i) {
    ImuSample sample;
    sample.stamp_ns = start_ns -
        static_cast<uint64_t>(parameters.static_init_samples - 1 - i) * 5000000ULL;
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, parameters.gravity_mps2) + accel_bias;
    sample.angular_velocity = gyro_bias;
    stationary.push_back(sample);
  }

  Pose3d T_imu_lidar;
  T_imu_lidar.position = Eigen::Vector3d(0.24, -0.035, 0.06);
  T_imu_lidar.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitY()));
  Pose3d initial_map_T_lidar;
  initial_map_T_lidar.position = T_imu_lidar.position;
  initial_map_T_lidar.orientation = T_imu_lidar.orientation;
  std::string failure;
  if (!committed.initializeStatic(stationary, initial_map_T_lidar, T_imu_lidar,
                                  &failure)) {
    std::cerr << "FAIL: initialization: " << failure << '\n';
    return 1;
  }

  const Eigen::Vector3d acceleration(0.28, -0.11, 0.045);
  const Eigen::Vector3d angular_velocity(0.16, -0.09, 0.72);
  const uint64_t scan_end_ns = start_ns + 100000000ULL;
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imu;
  imu.push_back(stationary.back());
  for (int i = 1; i <= 19; ++i) {
    ImuSample sample;
    sample.stamp_ns = start_ns + static_cast<uint64_t>(i) * 5000000ULL;
    const Pose3d truth = truthMapTImu(sample.stamp_ns, start_ns,
                                     acceleration, angular_velocity);
    sample.acceleration = truth.orientation.conjugate() *
        (acceleration - Eigen::Vector3d(0.0, 0.0, -parameters.gravity_mps2)) +
        accel_bias;
    sample.angular_velocity = angular_velocity + gyro_bias;
    imu.push_back(sample);
  }

  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> landmarks = {
      Eigen::Vector3d(4.0, 1.1, 0.8), Eigen::Vector3d(-1.5, 3.0, 1.6),
      Eigen::Vector3d(2.2, -2.4, 0.15), Eigen::Vector3d(0.7, 1.8, 3.3)};
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> raw_cloud;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> truth_end;
  for (int i = 0; i <= 10; ++i) {
    const uint64_t stamp = start_ns + static_cast<uint64_t>(i) * 10000000ULL;
    const Pose3d truth_imu = truthMapTImu(stamp, start_ns,
                                        acceleration, angular_velocity);
    Pose3d truth_lidar;
    truth_lidar.position = truth_imu.position + truth_imu.orientation * T_imu_lidar.position;
    truth_lidar.orientation = truth_imu.orientation * T_imu_lidar.orientation;
    const Eigen::Isometry3d map_T_lidar = toIsometry(truth_lidar);
    const Eigen::Isometry3d map_T_lidar_end = toIsometry([&]() {
      const Pose3d end_imu = truthMapTImu(scan_end_ns, start_ns,
                                         acceleration, angular_velocity);
      Pose3d end_lidar;
      end_lidar.position = end_imu.position + end_imu.orientation * T_imu_lidar.position;
      end_lidar.orientation = end_imu.orientation * T_imu_lidar.orientation;
      return end_lidar;
    }());
    for (std::size_t landmark = 0; landmark < landmarks.size(); ++landmark) {
      TimedLidarPoint point;
      point.stamp_ns = stamp;
      point.intensity = static_cast<double>(landmark);
      point.position = map_T_lidar.inverse() * landmarks[landmark];
      raw_cloud.push_back(point);
      truth_end.push_back(map_T_lidar_end.inverse() * landmarks[landmark]);
    }
  }

  std::unique_ptr<FastLio2IkfomFrontend> candidate = committed.cloneCandidate();
  ScanEndResult output;
  ScanEndProcessor processor;
  if (!processor.process(candidate.get(), T_imu_lidar, start_ns, scan_end_ns,
                         imu, raw_cloud, &output, &failure)) {
    std::cerr << "FAIL: scan processing: " << failure << '\n';
    return 1;
  }
  if (imu.back().stamp_ns >= scan_end_ns || output.scan_end_ns != scan_end_ns ||
      output.imu_poses.back().stamp_ns != scan_end_ns ||
      output.cloud_end_frame.size() != raw_cloud.size()) {
    std::cerr << "FAIL: causal tail path did not reach exact scan end\n";
    return 1;
  }
  const FilterSnapshot scan_end_state = candidate->getState();
  if (scan_end_state.stamp_ns != scan_end_ns ||
      !scan_end_state.map_T_imu.position.allFinite() ||
      !scan_end_state.map_T_imu.orientation.coeffs().allFinite() ||
      !scan_end_state.velocity.allFinite()) {
    std::cerr << "FAIL: causal tail state is not finite at exact scan end\n";
    return 1;
  }

  std::vector<double> raw_errors;
  std::vector<double> deskewed_errors;
  raw_errors.reserve(raw_cloud.size());
  deskewed_errors.reserve(raw_cloud.size());
  for (std::size_t i = 0; i < raw_cloud.size(); ++i) {
    raw_errors.push_back((raw_cloud[i].position - truth_end[i]).norm());
    deskewed_errors.push_back((output.cloud_end_frame[i].position - truth_end[i]).norm());
  }
  const double raw_mean = std::accumulate(raw_errors.begin(), raw_errors.end(), 0.0) /
                          static_cast<double>(raw_errors.size());
  const double deskew_mean = std::accumulate(deskewed_errors.begin(), deskewed_errors.end(), 0.0) /
                             static_cast<double>(deskewed_errors.size());
  const double raw_p95 = percentile(raw_errors, 0.95);
  const double deskew_p95 = percentile(deskewed_errors, 0.95);
  const double raw_max = *std::max_element(raw_errors.begin(), raw_errors.end());
  const double deskew_max = *std::max_element(deskewed_errors.begin(), deskewed_errors.end());
  if (!(deskew_mean < 0.25 * raw_mean && deskew_p95 < 0.25 * raw_p95 &&
        deskew_max < 0.25 * raw_max)) {
    std::cerr << "FAIL: deskew did not materially reduce point error\n"
              << "raw mean/P95/max=" << raw_mean << '/' << raw_p95 << '/' << raw_max
              << " deskew=" << deskew_mean << '/' << deskew_p95 << '/' << deskew_max << '\n';
    return 1;
  }

  std::cout << "SCAN_END_DESKEW_RUNTIME_CONTRACT_PASS"
            << " points=" << raw_cloud.size()
            << " raw_mean_m=" << raw_mean << " raw_p95_m=" << raw_p95
            << " raw_max_m=" << raw_max << " deskew_mean_m=" << deskew_mean
            << " deskew_p95_m=" << deskew_p95 << " deskew_max_m=" << deskew_max
            << " imu_tail_gap_ns=" << (scan_end_ns - imu.back().stamp_ns)
            << " exact_end_ns=" << scan_end_state.stamp_ns << '\n';
  return 0;
}
