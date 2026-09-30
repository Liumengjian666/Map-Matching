#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/lidar_deskew_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

bool setFailure(std::string* reason, const char* value) {
  if (reason) *reason = value;
  return false;
}

}  // namespace

bool prepareScanWindow(
    uint64_t raw_scan_start_ns, uint64_t scan_end_ns,
    uint64_t committed_ns,
    std::vector<TimedLidarPoint,
                Eigen::aligned_allocator<TimedLidarPoint>>* cloud,
    ScanWindowDecision* decision, ScanWindowStats* stats,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!cloud || !decision || !stats || raw_scan_start_ns == 0 ||
      scan_end_ns <= raw_scan_start_ns || committed_ns == 0 || cloud->empty()) {
    if (failure_reason) *failure_reason = "invalid_scan_window_input";
    return false;
  }

  for (const TimedLidarPoint& point : *cloud) {
    if (point.stamp_ns < raw_scan_start_ns || point.stamp_ns > scan_end_ns) {
      if (failure_reason) *failure_reason = "point_timestamp_outside_raw_scan_window";
      return false;
    }
  }

  ScanWindowStats prepared;
  prepared.effective_scan_start_ns = std::max(raw_scan_start_ns, committed_ns);
  prepared.remaining_points = cloud->size();
  if (scan_end_ns <= committed_ns) {
    *decision = ScanWindowDecision::SKIP_STALE;
    *stats = prepared;
    return true;
  }

  *decision = ScanWindowDecision::PROCESS;
  if (raw_scan_start_ns < committed_ns) {
    prepared.overlap_duration_ns = committed_ns - raw_scan_start_ns;
    const std::size_t retained = static_cast<std::size_t>(std::count_if(
        cloud->begin(), cloud->end(), [committed_ns](const TimedLidarPoint& point) {
          return point.stamp_ns >= committed_ns;
        }));
    if (retained == 0) {
      if (failure_reason) *failure_reason = "partial_overlap_has_no_points_at_or_after_commit";
      return false;
    }
    const auto first_retained = std::remove_if(
        cloud->begin(), cloud->end(), [committed_ns](const TimedLidarPoint& point) {
          return point.stamp_ns < committed_ns;
        });
    prepared.overlap_points_dropped =
        static_cast<std::size_t>(std::distance(first_retained, cloud->end()));
    cloud->erase(first_retained, cloud->end());
    prepared.remaining_points = cloud->size();
  }
  *stats = prepared;
  return true;
}

bool ScanEndProcessor::process(
    FastLio2IkfomFrontend* candidate, const Pose3d& T_imu_lidar,
    uint64_t scan_start_ns, uint64_t scan_end_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    ScanEndResult* result, std::string* failure_reason) const {
  if (failure_reason) failure_reason->clear();
  if (!candidate || !result) return setFailure(failure_reason, "null_candidate_or_output");
  if (!candidate->initialized()) return setFailure(failure_reason, "candidate_not_initialized");
  if (scan_start_ns == 0 || scan_end_ns <= scan_start_ns || imu.size() < 2 || cloud.empty())
    return setFailure(failure_reason, "invalid_scan_or_empty_sensor_data");
  if (!T_imu_lidar.position.allFinite() || !T_imu_lidar.orientation.coeffs().allFinite() ||
      std::abs(T_imu_lidar.orientation.norm() - 1.0) > 1e-6)
    return setFailure(failure_reason, "invalid_imu_lidar_extrinsic");
  for (const TimedLidarPoint& point : cloud) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity) ||
        point.stamp_ns < scan_start_ns || point.stamp_ns > scan_end_ns)
      return setFailure(failure_reason, "point_time_or_payload_outside_scan_interval");
  }

  ScanEndResult pending;
  pending.scan_end_ns = scan_end_ns;
  if (!candidate->predictImuSequence(imu, scan_end_ns, &pending.imu_poses,
                                    failure_reason))
    return false;
  if (pending.imu_poses.size() < 2 || pending.imu_poses.front().stamp_ns > scan_start_ns ||
      pending.imu_poses.back().stamp_ns != scan_end_ns)
    return setFailure(failure_reason, "imu_pose_sequence_does_not_cover_scan");

  const FilterSnapshot predicted = candidate->getState();
  pending.predicted_map_T_imu = predicted.map_T_imu;
  pending.predicted_map_T_lidar.position =
      predicted.map_T_imu.position + predicted.map_T_imu.orientation * T_imu_lidar.position;
  pending.predicted_map_T_lidar.orientation =
      (predicted.map_T_imu.orientation * T_imu_lidar.orientation).normalized();
  if (!deskewCloudToEndFrame(T_imu_lidar, pending.predicted_map_T_lidar,
                            pending.imu_poses, cloud, &pending.cloud_end_frame,
                            failure_reason)) return false;
  *result = std::move(pending);
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
