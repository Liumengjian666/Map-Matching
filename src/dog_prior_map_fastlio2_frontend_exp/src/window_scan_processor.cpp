#include "dog_prior_map_fastlio2_frontend_exp/window_scan_processor.hpp"
#include <cmath>
namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
bool deskewScanWithWindowState(const WindowState& anchor,
    std::uint64_t start, std::uint64_t end,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const ImuNoiseParameters& noise, const Pose3d& extrinsic,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    WindowDeskewResult* output, std::string* reason) {
  auto fail = [&](const char* text) { if (reason) *reason = text; return false; };
  if (reason) reason->clear();
  if (!output || anchor.stamp_ns != start || !start || end <= start || cloud.empty())
    return fail("invalid_window_deskew_interval_or_anchor");
  if (!extrinsic.position.allFinite() || !extrinsic.orientation.coeffs().allFinite() ||
      std::abs(extrinsic.orientation.norm() - 1.0) > 1e-6)
    return fail("invalid_imu_lidar_extrinsic");
  for (const auto& point : cloud) {
    if (!point.stamp_ns) return fail("RAW_POINT_TIME_UNAVAILABLE");
    if (point.stamp_ns < start || point.stamp_ns > end ||
        !point.position.allFinite() || !std::isfinite(point.intensity))
      return fail("point_time_or_payload_outside_scan_interval");
  }
  WindowDeskewResult pending;
  pending.scan_start_ns = start; pending.scan_end_ns = end; pending.anchor_state = anchor;
  ImuPreintegratedMeasurement integrated;
  if (!integrateWindowImuTrajectory(anchor, end, imu, noise, &integrated,
                                   &pending.imu_trajectory, reason)) return false;
  pending.predicted_end_state = propagateWindowState(anchor, integrated, noise.gravity);
  Pose3d lidar_end;
  lidar_end.position = pending.predicted_end_state.position +
      pending.predicted_end_state.rotation * extrinsic.position;
  lidar_end.orientation = (Eigen::Quaterniond(pending.predicted_end_state.rotation) *
      extrinsic.orientation).normalized();
  if (!deskewCloudToEndFrame(extrinsic, lidar_end, pending.imu_trajectory,
                            cloud, &pending.cloud_end_frame, reason)) return false;
  pending.status = "WINDOW_OWNED_SE3_DESKEW";
  *output = std::move(pending);
  return true;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
