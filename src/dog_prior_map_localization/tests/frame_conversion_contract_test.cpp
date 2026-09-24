#include "dog_prior_map_localization/core/frame_conversions.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
Eigen::Isometry3d pose(const Eigen::Vector3d &translation,
                       const Eigen::Vector3d &axis,
                       double angle)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  result.translation() = translation;
  return result;
}

double rotationDistance(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
{
  return Eigen::AngleAxisd(a * b.transpose()).angle();
}
}  // namespace

int main()
{
  using dog_prior_map_localization::imuPoseToLidarPose;
  using dog_prior_map_localization::imuTwistToLidarFrame;
  using dog_prior_map_localization::lidarPoseToImuPose;

  const Eigen::Isometry3d T_world_imu = pose(
      Eigen::Vector3d(3.1, -2.4, 0.7), Eigen::Vector3d(0.3, -0.5, 0.8), 0.73);
  Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pure_translation = Eigen::Isometry3d::Identity();
  pure_translation.translation() = Eigen::Vector3d(0.08, 0.029, 0.03);
  const Eigen::Isometry3d pure_rotation = pose(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 2.0, -1.0), 0.37);
  const Eigen::Isometry3d combined = pose(
      Eigen::Vector3d(0.08, 0.029, 0.03), Eigen::Vector3d(-0.4, 0.6, 0.2), 0.21);
  const Eigen::Isometry3d yaw90 = pose(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), M_PI / 2.0);
  Eigen::Isometry3d lever_arm = Eigen::Isometry3d::Identity();
  lever_arm.translation() = Eigen::Vector3d(0.45, -0.17, 0.23);

  const std::vector<std::pair<std::string, Eigen::Isometry3d>> cases = {
      {"identity", identity},
      {"pure_translation", pure_translation},
      {"pure_rotation", pure_rotation},
      {"combined_rotation_translation", combined},
      {"ninety_degree_yaw", yaw90},
      {"nonzero_lever_arm", lever_arm}};

  std::cout << "contract,translation_error_m,rotation_error_rad,linear_twist_error,angular_twist_error,result\n";
  for (const auto &entry : cases)
  {
    const Eigen::Isometry3d T_world_lidar = imuPoseToLidarPose(T_world_imu, entry.second);
    const Eigen::Isometry3d recovered = lidarPoseToImuPose(T_world_lidar, entry.second);
    const double translation_error = (recovered.translation() - T_world_imu.translation()).norm();
    const double rotation_error = rotationDistance(recovered.linear(), T_world_imu.linear());
    const bool pass = translation_error < 1e-9 && rotation_error < 1e-9;
    std::cout << entry.first << "," << translation_error << "," << rotation_error
              << ",nan,nan," << (pass ? "PASS" : "FAIL") << "\n";
    if (!pass) return 1;
  }

  const Eigen::Vector3d v_world_imu(1.2, -0.4, 0.7);
  const Eigen::Vector3d omega_imu(0.3, -0.2, 0.5);
  const Eigen::Isometry3d T_imu_lidar = combined;
  const auto twist = imuTwistToLidarFrame(v_world_imu, omega_imu,
                                          T_world_imu.linear(), T_imu_lidar);
  const Eigen::Matrix3d R_world_lidar = T_world_imu.linear() * T_imu_lidar.linear();
  const Eigen::Vector3d expected_world_lidar_velocity = v_world_imu +
      T_world_imu.linear() * omega_imu.cross(T_imu_lidar.translation());
  const Eigen::Vector3d expected_linear = R_world_lidar.transpose() * expected_world_lidar_velocity;
  const Eigen::Vector3d expected_angular = T_imu_lidar.linear().transpose() * omega_imu;
  const double linear_error = (twist.linear - expected_linear).norm();
  const double angular_error = (twist.angular - expected_angular).norm();
  const bool twist_pass = linear_error < 1e-12 && angular_error < 1e-12;
  std::cout << "twist_child_lidar," << std::numeric_limits<double>::quiet_NaN() << ","
            << std::numeric_limits<double>::quiet_NaN() << "," << linear_error << ","
            << angular_error << "," << (twist_pass ? "PASS" : "FAIL") << "\n";
  if (!twist_pass) return 1;

  std::cerr << "FRAME_CONVERSION_AND_TWIST_CONTRACT_PASS\n";
  return 0;
}
