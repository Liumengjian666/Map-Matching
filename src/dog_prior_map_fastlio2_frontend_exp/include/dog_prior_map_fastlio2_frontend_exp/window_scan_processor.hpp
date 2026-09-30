#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/measurement_provenance.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/lidar_deskew_geometry.hpp"

namespace dog_prior_map_fastlio2_frontend_exp {
namespace fixed_lag {
struct RawTimedScan {
  std::uint64_t transaction_id = 0;
  std::uint64_t scan_start_ns = 0, scan_end_ns = 0;
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> points;
  LidarCloudProvenance provenance = LidarCloudProvenance::RAW_TIMED_SENSOR;
};
struct WindowDeskewResult {
  std::uint64_t scan_start_ns = 0, scan_end_ns = 0;
  WindowState anchor_state, predicted_end_state;
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> imu_trajectory;
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> cloud_end_frame;
  LidarCloudProvenance provenance = LidarCloudProvenance::WINDOW_OWNED_SE3_DESKEW;
  std::string status = "UNINITIALIZED";
};
bool deskewScanWithWindowState(const WindowState& scan_start_state,
    std::uint64_t scan_start_ns, std::uint64_t scan_end_ns,
    const std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>& imu,
    const ImuNoiseParameters& noise, const Pose3d& T_imu_lidar,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& raw_cloud,
    WindowDeskewResult* output, std::string* reason = nullptr);
}  // namespace fixed_lag
}  // namespace dog_prior_map_fastlio2_frontend_exp
