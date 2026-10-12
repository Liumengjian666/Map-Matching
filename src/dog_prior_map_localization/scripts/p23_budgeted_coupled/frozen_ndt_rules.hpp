#pragma once
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
// Verbatim production helpers; compare function text in test_contract.py.
namespace p23 {
void finalizeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}
struct FrozenNdtRules {
  bool ndt_step_limit_enable_=true,has_previous_pose_=true;
  double ndt_step_limit_max_translation_=.5,ndt_step_limit_max_rotation_deg_=5.;
  Eigen::Matrix4d previous_pose_=Eigen::Matrix4d::Identity();
  static double rotationAngleDeg(const Eigen::Matrix3d& R) { return std::abs(Eigen::AngleAxisd(R).angle())*180.0/M_PI; }
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDown(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                double voxel_size,
                                                double voxel_z_size,
                                                int max_points) const
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
    if (voxel_size > 0.01)
    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setLeafSize(voxel_size, voxel_size, voxel_z_size);
      voxel.setInputCloud(cloud);
      voxel.filter(*down);
    }
    else
    {
      *down = *cloud;
    }

    if (max_points > 0 && static_cast<int>(down->size()) > max_points)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZ>());
      sampled->reserve(max_points);
      const double step = static_cast<double>(down->size() - 1) / static_cast<double>(std::max(max_points - 1, 1));
      for (int i = 0; i < max_points; ++i)
      {
        sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
      }
      finalizeCloud(sampled);
      return sampled;
    }
    finalizeCloud(down);
    return down;
  }
static uint64_t stableCloudHash(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
  {
    constexpr uint64_t kOffset = 1469598103934665603ULL;
    constexpr uint64_t kPrime = 1099511628211ULL;
    uint64_t hash = kOffset;
    const auto mixByte = [&hash](uint8_t byte) {
      hash ^= static_cast<uint64_t>(byte);
      hash *= kPrime;
    };
    const auto mixU32 = [&mixByte](uint32_t value) {
      for (int shift = 0; shift < 32; shift += 8)
      {
        mixByte(static_cast<uint8_t>((value >> shift) & 0xffU));
      }
    };
    if (!cloud) return hash;
    mixU32(cloud->width);
    mixU32(cloud->height);
    mixU32(cloud->is_dense ? 1U : 0U);
    mixU32(static_cast<uint32_t>(cloud->size()));
    for (const auto &point : cloud->points)
    {
      uint32_t bits = 0;
      std::memcpy(&bits, &point.x, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.y, sizeof(bits));
      mixU32(bits);
      std::memcpy(&bits, &point.z, sizeof(bits));
      mixU32(bits);
    }
    return hash;
  }
bool limitNdtStep(const Eigen::Matrix4d &raw_result,
                    Eigen::Matrix4d &used_result,
                    bool &translation_limited,
                    bool &rotation_limited) const
  {
    translation_limited = false;
    rotation_limited = false;
    if (!ndt_step_limit_enable_ || !has_previous_pose_) return false;

    bool limited = false;
    const Eigen::Vector3d previous_p = previous_pose_.block<3, 1>(0, 3);
    const Eigen::Vector3d raw_p = raw_result.block<3, 1>(0, 3);
    const Eigen::Vector3d dp = raw_p - previous_p;
    const double dist = dp.norm();

    const Eigen::Matrix3d previous_R = previous_pose_.block<3, 3>(0, 0);
    const Eigen::Matrix3d raw_R = raw_result.block<3, 3>(0, 0);
    const double angle_deg = rotationAngleDeg(previous_R.transpose() * raw_R);

    if (ndt_step_limit_max_translation_ > 0.0 && dist > ndt_step_limit_max_translation_)
    {
      used_result.block<3, 1>(0, 3) = previous_p + dp.normalized() * ndt_step_limit_max_translation_;
      limited = true;
      translation_limited = true;
    }
    if (ndt_step_limit_max_rotation_deg_ > 0.0 && angle_deg > ndt_step_limit_max_rotation_deg_)
    {
      const double ratio = ndt_step_limit_max_rotation_deg_ / std::max(angle_deg, 1e-6);
      Eigen::Quaterniond q_prev(previous_R);
      Eigen::Quaterniond q_raw(raw_R);
      used_result.block<3, 3>(0, 0) = q_prev.normalized().slerp(ratio, q_raw.normalized()).toRotationMatrix();
      limited = true;
      rotation_limited = true;
    }
    return limited;
  }
};
}
