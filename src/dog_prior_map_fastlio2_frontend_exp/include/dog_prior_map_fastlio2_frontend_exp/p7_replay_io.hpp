#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <Eigen/StdVector>

namespace dog_prior_map_fastlio2_frontend_exp {

struct P7ScanRecord {
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  double time_s = 0.0;
  uint64_t cloud_byte_offset = 0;
  uint64_t cloud_point_count = 0;
  bool expected_source_hash_available = false;
  uint64_t expected_source_hash = 0;
};

// Raw, timestamped source scans for baseline replay. The binary payload is a
// packed sequence of little-endian {float32 x,y,z; uint32 offset_ns} records.
struct P7TimedScanRecord {
  uint64_t transaction_id = 0;
  uint64_t scan_start_ns = 0;
  uint64_t scan_end_ns = 0;
  uint64_t cloud_byte_offset = 0;
  uint64_t cloud_point_count = 0;
};

using P7ImuVector = std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>;
using P7TimedLidarVector = std::vector<TimedLidarPoint,
    Eigen::aligned_allocator<TimedLidarPoint>>;

P7ImuVector readP7Imu(const std::string& path);
// Right/body-frame rotation product satisfying
// R_map_imu(end) = R_map_imu(start) * R_start_to_end.
bool integrateBodyRelativeRotation(
    const P7ImuVector& all, uint64_t start_stamp_ns, uint64_t end_stamp_ns,
    const Eigen::Vector3d& gyro_bias, Eigen::Matrix3d* R_start_to_end,
    std::string* failure_reason);
std::vector<P7ScanRecord> readP7Scans(const std::string& filter_path,
                                    const std::string& asset_path);
std::vector<P7TimedScanRecord> readP7TimedScans(
    const std::string& filter_path, const std::string& raw_scan_index_path);
RegistrationCloud readP7PackedCloud(const std::string& path, const P7ScanRecord& scan);
P7TimedLidarVector readP7PackedTimedCloud(
    const std::string& path, const P7TimedScanRecord& scan);
RuntimeParameters readP7Parameters(const std::string& path,
    Pose3d* initial_map_T_lidar, Pose3d* T_imu_lidar);

Eigen::Isometry3d asIsometry(const Pose3d& pose);
Pose3d fromIsometry(const Eigen::Isometry3d& transform);
Pose3d lidarMeasurementToImu(const Pose3d& map_T_lidar, const Pose3d& T_imu_lidar);
P7ImuVector imuWindow(const P7ImuVector& all, uint64_t state_stamp_ns, uint64_t end_stamp_ns);

}  // namespace dog_prior_map_fastlio2_frontend_exp
