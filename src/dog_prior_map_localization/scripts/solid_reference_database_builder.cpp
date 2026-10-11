#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "dog_prior_map_localization/solid_place_recognition.hpp"

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

bool readPose(const std::string &line, double &stamp, Eigen::Matrix4d &pose)
{
  std::istringstream row(line);
  double x = 0.0, y = 0.0, z = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  if (!(row >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) return false;
  const Eigen::Quaterniond orientation(qw, qx, qy, qz);
  if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z) || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
    return false;
  pose.setIdentity();
  pose.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return true;
}

Cloud::Ptr prepareLikeNdtSource(const Cloud::ConstPtr &raw)
{
  Cloud::Ptr finite_range(new Cloud());
  finite_range->reserve(raw->size());
  for (const pcl::PointXYZ &point : raw->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(static_cast<double>(point.x) * point.x +
                                   static_cast<double>(point.y) * point.y +
                                   static_cast<double>(point.z) * point.z);
    if (range < 0.5 || range > 80.0) continue;
    finite_range->push_back(point);
  }
  finite_range->width = static_cast<uint32_t>(finite_range->size());
  finite_range->height = 1;
  finite_range->is_dense = true;

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.25f, 0.25f, 0.25f);
  voxel.setInputCloud(finite_range);
  Cloud::Ptr down(new Cloud());
  voxel.filter(*down);
  if (down->size() > 1400)
  {
    Cloud::Ptr sampled(new Cloud());
    sampled->reserve(1400);
    const double step = static_cast<double>(down->size() - 1) / 1399.0;
    for (int i = 0; i < 1400; ++i)
      sampled->push_back(down->points[static_cast<std::size_t>(std::round(i * step))]);
    down.swap(sampled);
  }
  down->width = static_cast<uint32_t>(down->size());
  down->height = 1;
  down->is_dense = true;
  return down;
}

std::string scanPath(const std::string &directory, std::size_t index)
{
  std::ostringstream name;
  name << directory << "/scans/key_" << std::setw(6) << std::setfill('0') << index << ".pcd";
  return name.str();
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: solid_reference_database_builder <keyframe_dir> <keyframe_poses.txt> "
                 "<database.txt> <build_diagnostics.csv>\n";
    return 2;
  }
  const std::string keyframe_dir = argv[1];
  const std::string poses_path = argv[2];
  const std::string database_path = argv[3];
  const std::string diagnostics_path = argv[4];
  std::ifstream poses(poses_path);
  if (!poses.is_open())
  {
    std::cerr << "cannot open poses: " << poses_path << "\n";
    return 2;
  }
  std::ofstream diagnostics(diagnostics_path, std::ios::out | std::ios::trunc);
  if (!diagnostics.is_open())
  {
    std::cerr << "cannot open diagnostics: " << diagnostics_path << "\n";
    return 2;
  }
  diagnostics << "id,stamp_sec,raw_pcd_points,ndt_source_points,solid_fov_range_points,"
                 "solid_descriptor_points,descriptor_ms,status\n";
  diagnostics << std::setprecision(17);

  dog_prior_map_localization::SolidPlaceRecognitionAdapter adapter;
  std::vector<dog_prior_map_localization::SolidReferenceEntry> entries;
  std::string line;
  std::size_t index = 0;
  std::size_t pose_rows = 0;
  std::size_t pcd_failures = 0;
  std::size_t valid_descriptors = 0;
  while (std::getline(poses, line))
  {
    if (line.empty()) continue;
    ++pose_rows;
    double stamp = 0.0;
    Eigen::Matrix4d map_T_reference;
    if (!readPose(line, stamp, map_T_reference))
    {
      std::cerr << "invalid pose row " << pose_rows << "\n";
      return 3;
    }
    Cloud::Ptr raw(new Cloud());
    const std::string pcd_path = scanPath(keyframe_dir, index);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *raw) != 0)
    {
      std::cerr << "missing/unreadable keyframe PCD " << pcd_path << "\n";
      ++pcd_failures;
      diagnostics << index << ',' << stamp << ",0,0,0,0,0,PCD_LOAD_FAILED\n";
      ++index;
      continue;
    }
    Cloud::Ptr ndt_source = prepareLikeNdtSource(raw);
    const auto descriptor = adapter.describe(*ndt_source);
    diagnostics << index << ',' << stamp << ',' << raw->size() << ',' << ndt_source->size() << ','
                << descriptor.fov_range_points << ',' << descriptor.descriptor_points << ','
                << descriptor.descriptor_ms << ',' << descriptor.status << '\n';
    if (descriptor.valid)
    {
      dog_prior_map_localization::SolidReferenceEntry entry;
      entry.id = index;
      entry.stamp_sec = stamp;
      entry.map_T_reference = map_T_reference;
      entry.descriptor = descriptor.descriptor;
      entries.push_back(std::move(entry));
      ++valid_descriptors;
    }
    ++index;
  }
  diagnostics.flush();
  if (entries.empty())
  {
    std::cerr << "no valid descriptors; see " << diagnostics_path << "\n";
    return 4;
  }
  std::string reason;
  if (!adapter.writeDatabase(database_path, entries, reason))
  {
    std::cerr << "database write failed: " << reason << "\n";
    return 5;
  }
  std::cout << "pose_rows=" << pose_rows << " scanned_pcds=" << index
            << " pcd_failures=" << pcd_failures << " valid_descriptors=" << valid_descriptors
            << " descriptor_dim=" << adapter.descriptorSize()
            << " database_path=" << database_path << "\n";
  return pcd_failures == 0 && valid_descriptors > 0 ? 0 : 6;
}
