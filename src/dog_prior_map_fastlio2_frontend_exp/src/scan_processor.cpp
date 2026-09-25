#include "dog_prior_map_fastlio2_frontend_exp/scan_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {

Eigen::Isometry3d toIsometry(const Pose3d& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = pose.orientation.toRotationMatrix();
  transform.translation() = pose.position;
  return transform;
}

Pose3d interpolatePose(const ImuPoseSample& a, const ImuPoseSample& b,
                       uint64_t stamp_ns) {
  Pose3d pose;
  const double alpha = static_cast<double>(stamp_ns - a.stamp_ns) /
                       static_cast<double>(b.stamp_ns - a.stamp_ns);
  pose.position = (1.0 - alpha) * a.position + alpha * b.position;
  pose.orientation = Eigen::Quaterniond(a.rotation).slerp(alpha, Eigen::Quaterniond(b.rotation));
  pose.orientation.normalize();
  return pose;
}

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
  const Eigen::Isometry3d map_T_lidar_end = toIsometry(pending.predicted_map_T_lidar);
  pending.cloud_end_frame.reserve(cloud.size());

  for (const TimedLidarPoint& point : cloud) {
    auto upper = std::lower_bound(
        pending.imu_poses.begin(), pending.imu_poses.end(), point.stamp_ns,
        [](const ImuPoseSample& pose, uint64_t stamp) { return pose.stamp_ns < stamp; });
    Pose3d map_T_imu_at_point;
    if (upper == pending.imu_poses.end()) {
      return setFailure(failure_reason, "point_time_after_imu_pose_sequence");
    } else if (upper->stamp_ns == point.stamp_ns) {
      map_T_imu_at_point.position = upper->position;
      map_T_imu_at_point.orientation = Eigen::Quaterniond(upper->rotation).normalized();
    } else {
      if (upper == pending.imu_poses.begin())
        return setFailure(failure_reason, "point_time_before_imu_pose_sequence");
      const ImuPoseSample& lower = *(upper - 1);
      map_T_imu_at_point = interpolatePose(lower, *upper, point.stamp_ns);
    }

    Pose3d map_T_lidar_at_point;
    map_T_lidar_at_point.position = map_T_imu_at_point.position +
        map_T_imu_at_point.orientation * T_imu_lidar.position;
    map_T_lidar_at_point.orientation =
        (map_T_imu_at_point.orientation * T_imu_lidar.orientation).normalized();
    const Eigen::Vector3d point_end = map_T_lidar_end.inverse() *
        (toIsometry(map_T_lidar_at_point) * point.position);
    if (!point_end.allFinite())
      return setFailure(failure_reason, "nonfinite_deskewed_point");
    TimedLidarPoint deskewed = point;
    deskewed.position = point_end;
    pending.cloud_end_frame.push_back(deskewed);
  }
  *result = std::move(pending);
  return true;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
