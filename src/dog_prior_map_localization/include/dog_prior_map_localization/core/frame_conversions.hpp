#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dog_prior_map_localization
{

inline Eigen::Isometry3d lidarPoseToImuPose(const Eigen::Isometry3d &T_world_lidar,
                                             const Eigen::Isometry3d &T_imu_lidar)
{
  return T_world_lidar * T_imu_lidar.inverse();
}

inline Eigen::Isometry3d imuPoseToLidarPose(const Eigen::Isometry3d &T_world_imu,
                                            const Eigen::Isometry3d &T_imu_lidar)
{
  return T_world_imu * T_imu_lidar;
}

struct LidarFrameTwist
{
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
};

// nav_msgs/Odometry twist is expressed in child_frame_id. Move the origin
// velocity from IMU to LiDAR, including the rotational lever-arm term, then
// express both vectors in the LiDAR axes.
inline LidarFrameTwist imuTwistToLidarFrame(const Eigen::Vector3d &v_world_imu,
                                            const Eigen::Vector3d &omega_imu,
                                            const Eigen::Matrix3d &R_world_imu,
                                            const Eigen::Isometry3d &T_imu_lidar)
{
  LidarFrameTwist result;
  const Eigen::Matrix3d R_world_lidar = R_world_imu * T_imu_lidar.linear();
  const Eigen::Vector3d v_world_lidar =
      v_world_imu + R_world_imu * omega_imu.cross(T_imu_lidar.translation());
  result.linear = R_world_lidar.transpose() * v_world_lidar;
  result.angular = T_imu_lidar.linear().transpose() * omega_imu;
  return result;
}

}  // namespace dog_prior_map_localization
