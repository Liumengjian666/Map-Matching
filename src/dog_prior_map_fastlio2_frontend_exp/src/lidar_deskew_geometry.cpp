#include "dog_prior_map_fastlio2_frontend_exp/lidar_deskew_geometry.hpp"
#include <algorithm>
#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {
bool deskewCloudToEndFrame(const Pose3d& extrinsic, const Pose3d& end,
    const std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>>& poses,
    const std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>& cloud,
    std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>>* output,
    std::string* reason) {
  auto fail = [&](const char* text) { if (reason) *reason = text; return false; };
  if (!output || poses.size() < 2 || cloud.empty()) return fail("invalid_deskew_geometry_input");
  auto iso = [](const Pose3d& p) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = p.orientation.toRotationMatrix(); t.translation() = p.position; return t;
  };
  const Eigen::Isometry3d end_inverse = iso(end).inverse();
  std::vector<TimedLidarPoint, Eigen::aligned_allocator<TimedLidarPoint>> pending;
  pending.reserve(cloud.size());
  for (const auto& point : cloud) {
    auto upper = std::lower_bound(poses.begin(), poses.end(), point.stamp_ns,
        [](const ImuPoseSample& p, std::uint64_t stamp) { return p.stamp_ns < stamp; });
    if (upper == poses.end()) return fail("point_time_after_imu_pose_sequence");
    Pose3d at;
    if (upper->stamp_ns == point.stamp_ns) {
      at.position = upper->position;
      at.orientation = Eigen::Quaterniond(upper->rotation).normalized();
    } else {
      if (upper == poses.begin()) return fail("point_time_before_imu_pose_sequence");
      const auto& lower = *(upper - 1);
      const double alpha = static_cast<double>(point.stamp_ns - lower.stamp_ns) /
          static_cast<double>(upper->stamp_ns - lower.stamp_ns);
      at.position = (1.0 - alpha) * lower.position + alpha * upper->position;
      at.orientation = Eigen::Quaterniond(lower.rotation).slerp(alpha, Eigen::Quaterniond(upper->rotation));
      at.orientation.normalize();
    }
    Pose3d lidar;
    lidar.position = at.position + at.orientation * extrinsic.position;
    lidar.orientation = (at.orientation * extrinsic.orientation).normalized();
    TimedLidarPoint transformed = point;
    transformed.position = end_inverse * (iso(lidar) * point.position);
    if (!transformed.position.allFinite()) return fail("nonfinite_deskewed_point");
    pending.push_back(transformed);
  }
  *output = std::move(pending);
  return true;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
