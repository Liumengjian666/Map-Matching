#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"
#include <string>
namespace dog_prior_map_fastlio2_frontend_exp {
// Pure geometry extracted from ScanEndProcessor. No state ownership or IKFoM.
bool deskewCloudToEndFrame(const Pose3d& T_imu_lidar,
    const Pose3d& map_T_lidar_end,
    const std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>& poses,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>* output,
    std::string* reason = nullptr);
}  // namespace dog_prior_map_fastlio2_frontend_exp
