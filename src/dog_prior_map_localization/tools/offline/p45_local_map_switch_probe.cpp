#include "dog_prior_map_localization/local_map_tiles.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sys/resource.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud::Ptr prepareSource(const std::string &path)
{
  Cloud::Ptr raw(new Cloud());
  if (pcl::io::loadPCDFile(path, *raw) != 0)
    throw std::runtime_error("cannot read scan PCD: " + path);

  Cloud::Ptr filtered(new Cloud());
  filtered->reserve(raw->size());
  for (const pcl::PointXYZ &point : raw->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  filtered->height = 1;
  filtered->width = static_cast<std::uint32_t>(filtered->size());
  filtered->is_dense = true;

  Cloud::Ptr downsampled(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(0.25f, 0.25f, 0.25f);
  voxel.setInputCloud(filtered);
  voxel.filter(*downsampled);
  if (downsampled->size() > 1400)
  {
    Cloud::Ptr capped(new Cloud());
    capped->reserve(1400);
    const double step = static_cast<double>(downsampled->size() - 1) / 1399.0;
    for (int index = 0; index < 1400; ++index)
      capped->push_back(downsampled->points[static_cast<std::size_t>(std::llround(index * step))]);
    downsampled.swap(capped);
  }
  downsampled->height = 1;
  downsampled->width = static_cast<std::uint32_t>(downsampled->size());
  downsampled->is_dense = true;
  return downsampled;
}

Eigen::Matrix4d poseFromArgs(char **argv, int begin)
{
  const double tx = std::stod(argv[begin + 0]);
  const double ty = std::stod(argv[begin + 1]);
  const double tz = std::stod(argv[begin + 2]);
  Eigen::Quaterniond q(std::stod(argv[begin + 6]), std::stod(argv[begin + 3]),
                       std::stod(argv[begin + 4]), std::stod(argv[begin + 5]));
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-3)
    throw std::runtime_error("test pose quaternion must be finite and normalized");
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3d(tx, ty, tz);
  return pose;
}

std::set<std::string> asSet(const std::vector<std::string> &values)
{
  return std::set<std::string>(values.begin(), values.end());
}

std::size_t newlyLoaded(const std::vector<std::string> &before,
                        const std::vector<std::string> &after)
{
  const std::set<std::string> old_ids = asSet(before);
  const std::set<std::string> new_ids = asSet(after);
  std::size_t added = 0;
  for (const std::string &id : new_ids)
    if (old_ids.count(id) == 0) ++added;
  return added;
}

void writeRow(std::ofstream &out, const char *name, const char *scan_role,
              const Cloud::ConstPtr &source, const Eigen::Matrix4d &pose,
              const dog_prior_map_localization::LocalMapUpdate &update,
              std::size_t new_tile_count, std::uint64_t peak_rss_kib)
{
  out << name << ',' << scan_role << ',' << source->size() << ','
      << std::setprecision(17) << pose(0, 3) << ',' << pose(1, 3) << ',' << pose(2, 3) << ','
      << (update.updated ? 1 : 0) << ',' << update.reason << ','
      << update.loaded_tile_ids.size() << ',' << new_tile_count << ',' << update.target_points << ','
      << update.tile_io_ms << ',' << update.target_merge_ms << ','
      << update.projected_min.x() << ',' << update.projected_min.y() << ',' << update.projected_min.z() << ','
      << update.projected_max.x() << ',' << update.projected_max.y() << ',' << update.projected_max.z() << ','
      << peak_rss_kib << ',';
  for (std::size_t i = 0; i < update.loaded_tile_ids.size(); ++i)
  {
    if (i) out << ';';
    out << update.loaded_tile_ids[i];
  }
  out << '\n';
}

std::uint64_t peakRssKiB()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return 0;
  return static_cast<std::uint64_t>(usage.ru_maxrss);
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 19)
  {
    std::cerr << "usage: p45_local_map_switch_probe MANIFEST FIRST_SCAN REMOTE_SCAN OUTPUT_CSV "
                 "FIRST_TX FIRST_TY FIRST_TZ FIRST_QX FIRST_QY FIRST_QZ FIRST_QW "
                 "REMOTE_TX REMOTE_TY REMOTE_TZ REMOTE_QX REMOTE_QY REMOTE_QZ REMOTE_QW\n";
    return 2;
  }

  try
  {
    dog_prior_map_localization::LocalMapTiles manager;
    std::string error;
    if (!manager.loadManifest(argv[1], &error))
      throw std::runtime_error("manifest load failed: " + error);
    const Cloud::Ptr first_source = prepareSource(argv[2]);
    const Cloud::Ptr remote_source = prepareSource(argv[3]);
    const Eigen::Matrix4d first_pose = poseFromArgs(argv, 5);
    const Eigen::Matrix4d remote_test_pose = poseFromArgs(argv, 12);

    std::ofstream output(argv[4]);
    if (!output) throw std::runtime_error("cannot open output CSV: " + std::string(argv[4]));
    output << "phase,scan_role,source_points,test_pose_tx,test_pose_ty,test_pose_tz,updated,reason,"
              "loaded_tile_count,new_tile_count,target_points,tile_io_ms,target_merge_ms,"
              "projected_x_min,projected_y_min,projected_z_min,projected_x_max,projected_y_max,projected_z_max,"
              "peak_rss_kib,loaded_tile_ids\n";

    const auto first = manager.ensureCoverage(first_source, first_pose, 10.0, 30.0);
    if (!first.ok || !first.updated)
      throw std::runtime_error("first fixture did not establish an initial tile target: " + first.error);
    writeRow(output, "INITIAL_ACTIVE_SET", "P45_FRAME_1_REAL_SOURCE", first_source,
             first_pose, first, first.loaded_tile_ids.size(), peakRssKiB());

    const std::vector<std::string> previous_ids = manager.activeTileIds();
    const auto remote = manager.ensureCoverage(remote_source, remote_test_pose, 10.0, 30.0);
    const std::size_t added = newlyLoaded(previous_ids, remote.loaded_tile_ids);
    if (!remote.ok || !remote.updated || added == 0 ||
        asSet(previous_ids) == asSet(remote.loaded_tile_ids))
      throw std::runtime_error("remote test fixture did not trigger a distinct tile replacement: " + remote.error);
    writeRow(output, "REMOTE_CACHE_SWITCH", "FROZEN_T0_PLUS_150S_REAL_SCAN_TEST_POSE_ONLY",
             remote_source, remote_test_pose, remote, added, peakRssKiB());
    output.close();

    std::cout << "P45 local-map cache switch functional test PASS; NDT align calls=0; GT reads=0\n"
              << "initial_tiles=" << first.loaded_tile_ids.size()
              << " remote_tiles=" << remote.loaded_tile_ids.size()
              << " newly_loaded_tiles=" << added
              << " remote_target_points=" << remote.target_points
              << " remote_tile_io_ms=" << remote.tile_io_ms
              << " remote_target_merge_ms=" << remote.target_merge_ms
              << " probe_peak_rss_kib=" << peakRssKiB() << '\n';
    return 0;
  }
  catch (const std::exception &exception)
  {
    std::cerr << "P45 local-map switch probe failed: " << exception.what() << '\n';
    return 1;
  }
}
