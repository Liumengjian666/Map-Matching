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

bool interpolateImuAt(
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& samples,
    uint64_t stamp_ns, ImuSample* output) {
  if (!output || samples.size() < 2 || stamp_ns < samples.front().stamp_ns ||
      stamp_ns > samples.back().stamp_ns) return false;
  const auto upper = std::lower_bound(samples.begin(), samples.end(), stamp_ns,
      [](const ImuSample& sample, uint64_t stamp) { return sample.stamp_ns < stamp; });
  if (upper != samples.end() && upper->stamp_ns == stamp_ns) {
    *output = *upper;
    return true;
  }
  if (upper == samples.begin() || upper == samples.end()) return false;
  const ImuSample& lower = *(upper - 1);
  const uint64_t span_ns = upper->stamp_ns - lower.stamp_ns;
  if (span_ns == 0) return false;
  const double alpha = static_cast<double>(stamp_ns - lower.stamp_ns) /
                       static_cast<double>(span_ns);
  output->stamp_ns = stamp_ns;
  output->acceleration = (1.0 - alpha) * lower.acceleration + alpha * upper->acceleration;
  output->angular_velocity =
      (1.0 - alpha) * lower.angular_velocity + alpha * upper->angular_velocity;
  return output->acceleration.allFinite() && output->angular_velocity.allFinite();
}

Eigen::Matrix3d expRotation(const Eigen::Vector3d& tangent) {
  const double angle = tangent.norm();
  if (angle < 1e-15) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, tangent / angle).toRotationMatrix();
}

bool buildBackwardAnchorHistory(
    const FilterSnapshot& anchor,
    uint64_t scan_start_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>* history,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!history || scan_start_ns == 0 || scan_start_ns >= anchor.stamp_ns ||
      imu.size() < 2 || imu.front().stamp_ns > scan_start_ns ||
      imu.back().stamp_ns < anchor.stamp_ns)
    return setFailure(failure_reason, "causal_preroll_imu_does_not_cover_anchor_interval");
  const auto anchor_sample = std::lower_bound(imu.begin(), imu.end(), anchor.stamp_ns,
      [](const ImuSample& sample, uint64_t stamp) { return sample.stamp_ns < stamp; });
  if (anchor_sample == imu.end() || anchor_sample->stamp_ns != anchor.stamp_ns)
    return setFailure(failure_reason, "causal_preroll_requires_exact_anchor_imu_sample");

  std::vector<uint64_t> knots;
  knots.reserve(imu.size() + 2);
  knots.push_back(scan_start_ns);
  for (const ImuSample& sample : imu)
    if (sample.stamp_ns > scan_start_ns && sample.stamp_ns < anchor.stamp_ns)
      knots.push_back(sample.stamp_ns);
  knots.push_back(anchor.stamp_ns);
  std::sort(knots.begin(), knots.end());
  knots.erase(std::unique(knots.begin(), knots.end()), knots.end());
  if (knots.size() < 2 || knots.front() != scan_start_ns || knots.back() != anchor.stamp_ns)
    return setFailure(failure_reason, "invalid_causal_preroll_knot_sequence");

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> knot_samples;
  knot_samples.reserve(knots.size());
  for (const uint64_t stamp : knots) {
    ImuSample sample;
    if (!interpolateImuAt(imu, stamp, &sample))
      return setFailure(failure_reason, "causal_preroll_missing_imu_interpolation_bracket");
    knot_samples.push_back(sample);
  }

  history->assign(knots.size(), ImuPoseSample());
  ImuPoseSample& end = history->back();
  end.stamp_ns = anchor.stamp_ns;
  end.rotation = anchor.map_T_imu.orientation.toRotationMatrix();
  end.position = anchor.map_T_imu.position;
  end.velocity = anchor.velocity;
  end.world_acceleration = end.rotation *
      (knot_samples.back().acceleration - anchor.accel_bias) + anchor.gravity;
  end.unbiased_gyro = knot_samples.back().angular_velocity - anchor.gyro_bias;

  for (std::size_t index = knots.size() - 1; index > 0; --index) {
    const double dt = static_cast<double>(knots[index] - knots[index - 1]) * 1e-9;
    const ImuSample& lower_imu = knot_samples[index - 1];
    const ImuSample& upper_imu = knot_samples[index];
    const Eigen::Vector3d omega =
        0.5 * (lower_imu.angular_velocity + upper_imu.angular_velocity) - anchor.gyro_bias;
    const Eigen::Vector3d tangent = omega * dt;
    const Eigen::Matrix3d rotation_upper = (*history)[index].rotation;
    const Eigen::Matrix3d rotation_lower = rotation_upper * expRotation(-tangent);
    const Eigen::Matrix3d rotation_mid = rotation_lower * expRotation(0.5 * tangent);
    const Eigen::Vector3d specific_force =
        0.5 * (lower_imu.acceleration + upper_imu.acceleration) - anchor.accel_bias;
    const Eigen::Vector3d acceleration_map = rotation_mid * specific_force + anchor.gravity;

    ImuPoseSample& lower = (*history)[index - 1];
    const ImuPoseSample& upper = (*history)[index];
    lower.stamp_ns = knots[index - 1];
    lower.rotation = rotation_lower;
    lower.velocity = upper.velocity - acceleration_map * dt;
    lower.position = upper.position - upper.velocity * dt + 0.5 * acceleration_map * dt * dt;
    lower.world_acceleration = rotation_lower *
        (lower_imu.acceleration - anchor.accel_bias) + anchor.gravity;
    lower.unbiased_gyro = lower_imu.angular_velocity - anchor.gyro_bias;
    if (!lower.rotation.allFinite() || !lower.position.allFinite() ||
        !lower.velocity.allFinite() || !lower.world_acceleration.allFinite() ||
        !std::isfinite(dt) || dt <= 0.0)
      return setFailure(failure_reason, "nonfinite_causal_preroll_pose");
  }
  return true;
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

bool prepareScanWindowWithCausalPreroll(
    uint64_t raw_scan_start_ns, uint64_t scan_end_ns,
    uint64_t committed_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    std::vector<TimedLidarPoint,
                Eigen::aligned_allocator<TimedLidarPoint>>* cloud,
    ScanWindowStats* stats, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!cloud || !stats || raw_scan_start_ns == 0 || committed_ns == 0 ||
      scan_end_ns <= committed_ns || raw_scan_start_ns >= committed_ns ||
      cloud->empty() || imu.size() < 2)
    return setFailure(failure_reason, "invalid_causal_preroll_scan_window");

  uint64_t previous_stamp = 0;
  for (const ImuSample& sample : imu) {
    if (sample.stamp_ns == 0 || sample.stamp_ns <= previous_stamp ||
        !sample.acceleration.allFinite() || !sample.angular_velocity.allFinite())
      return setFailure(failure_reason, "invalid_causal_preroll_imu_sequence");
    previous_stamp = sample.stamp_ns;
  }
  if (imu.front().stamp_ns > raw_scan_start_ns || imu.back().stamp_ns < committed_ns)
    return setFailure(failure_reason, "causal_preroll_imu_bracket_missing");
  const auto before_anchor = std::upper_bound(imu.begin(), imu.end(), committed_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (before_anchor == imu.begin() || before_anchor == imu.end())
    return setFailure(failure_reason, "causal_preroll_anchor_not_bracketed");

  for (const TimedLidarPoint& point : *cloud) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity) ||
        point.stamp_ns < raw_scan_start_ns || point.stamp_ns > scan_end_ns)
      return setFailure(failure_reason, "point_timestamp_or_payload_outside_preroll_scan");
  }
  ScanWindowStats prepared;
  prepared.effective_scan_start_ns = raw_scan_start_ns;
  prepared.overlap_duration_ns = committed_ns - raw_scan_start_ns;
  prepared.overlap_points_dropped = 0;
  prepared.remaining_points = cloud->size();
  *stats = prepared;
  return true;
}

bool ScanEndProcessor::process(
    FastLio2IkfomFrontend* candidate, const Pose3d& T_imu_lidar,
    uint64_t scan_start_ns, uint64_t scan_end_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    ScanEndResult* result, std::string* failure_reason,
    bool allow_causal_pre_anchor_preroll) const {
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
  const FilterSnapshot anchor = candidate->getState();
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> pre_anchor_poses;
  if (scan_start_ns < anchor.stamp_ns) {
    if (!allow_causal_pre_anchor_preroll)
      return setFailure(failure_reason, "scan_start_precedes_filter_state");
    if (!buildBackwardAnchorHistory(anchor, scan_start_ns, imu, &pre_anchor_poses,
                                    failure_reason)) return false;
  }
  if (!candidate->predictImuSequence(imu, scan_end_ns, &pending.imu_poses,
                                    failure_reason))
    return false;
  if (pending.imu_poses.size() < 2 || pending.imu_poses.back().stamp_ns != scan_end_ns)
    return setFailure(failure_reason, "imu_pose_sequence_does_not_cover_scan");
  if (!pre_anchor_poses.empty()) {
    if (pending.imu_poses.front().stamp_ns != anchor.stamp_ns ||
        pre_anchor_poses.back().stamp_ns != anchor.stamp_ns)
      return setFailure(failure_reason, "causal_preroll_anchor_pose_mismatch");
    pre_anchor_poses.insert(pre_anchor_poses.end(), pending.imu_poses.begin() + 1,
                            pending.imu_poses.end());
    pending.imu_poses = std::move(pre_anchor_poses);
  }
  if (pending.imu_poses.front().stamp_ns > scan_start_ns)
    return setFailure(failure_reason, "imu_pose_sequence_does_not_cover_scan_start");

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
