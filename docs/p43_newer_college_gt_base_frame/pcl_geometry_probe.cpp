#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

std::vector<std::string> splitCsv(const std::string &line)
{
  std::stringstream stream(line);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

void finalizeCloud(const Cloud::Ptr &cloud)
{
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::ConstPtr &cloud, float leaf, int max_points)
{
  Cloud::Ptr down(new Cloud());
  if (leaf > 0.01f)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(cloud);
    voxel.filter(*down);
  }
  else
  {
    *down = *cloud;
  }
  if (max_points > 0 && static_cast<int>(down->size()) > max_points)
  {
    Cloud::Ptr sampled(new Cloud());
    sampled->reserve(max_points);
    const double step = static_cast<double>(down->size() - 1) /
                        static_cast<double>(std::max(max_points - 1, 1));
    for (int i = 0; i < max_points; ++i)
      sampled->push_back(down->points[static_cast<std::size_t>(std::round(i * step))]);
    finalizeCloud(sampled);
    return sampled;
  }
  finalizeCloud(down);
  return down;
}

Cloud::Ptr preprocessSource(const Cloud::ConstPtr &input)
{
  Cloud::Ptr filtered(new Cloud());
  filtered->reserve(input->size());
  for (const auto &point : input->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(static_cast<double>(point.x) * point.x +
                                   static_cast<double>(point.y) * point.y +
                                   static_cast<double>(point.z) * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);  // P41 default z limits are unbounded.
  }
  finalizeCloud(filtered);
  return voxelDown(filtered, 0.25f, 1400);
}

Cloud::Ptr preprocessMap(const Cloud::ConstPtr &input)
{
  Cloud::Ptr finite(new Cloud());
  finite->reserve(input->size());
  for (const auto &point : input->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite->push_back(point);
  finalizeCloud(finite);
  const Cloud::Ptr map_cloud = voxelDown(finite, 0.15f, 0);
  return voxelDown(map_cloud, 0.15f, 0);
}

struct Geometry
{
  std::size_t points = 0;
  double median = 0.0;
  double p90 = 0.0;
  double overlap05 = 0.0;
  double overlap2 = 0.0;
  double overlap8 = 0.0;
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
};

Geometry measure(const Cloud::ConstPtr &source, const Eigen::Matrix4d &T,
                 const pcl::KdTreeFLANN<pcl::PointXYZ> &tree, Cloud::Ptr transformed)
{
  Geometry stats;
  std::vector<double> distances;
  distances.reserve(source->size());
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  std::size_t within05 = 0, within2 = 0, within8 = 0;
  transformed->reserve(source->size());
  for (const auto &point : source->points)
  {
    const Eigen::Vector4d input(point.x, point.y, point.z, 1.0);
    const Eigen::Vector4d mapped = T * input;
    const pcl::PointXYZ query(static_cast<float>(mapped.x()), static_cast<float>(mapped.y()),
                              static_cast<float>(mapped.z()));
    transformed->push_back(query);
    stats.minimum = stats.minimum.cwiseMin(mapped.head<3>());
    stats.maximum = stats.maximum.cwiseMax(mapped.head<3>());
    if (tree.nearestKSearch(query, 1, indices, squared_distances) != 1 || squared_distances[0] < 0.0f)
      continue;
    const double distance = std::sqrt(static_cast<double>(squared_distances[0]));
    distances.push_back(distance);
    within05 += distance <= 0.5 ? 1 : 0;
    within2 += distance <= 2.0 ? 1 : 0;
    within8 += distance <= 8.0 ? 1 : 0;
  }
  finalizeCloud(transformed);
  if (distances.empty()) throw std::runtime_error("no valid nearest-neighbor results");
  std::sort(distances.begin(), distances.end());
  const auto percentile = [&distances](double fraction) {
    const std::size_t index = static_cast<std::size_t>(std::floor(fraction * (distances.size() - 1)));
    return distances[index];
  };
  stats.points = distances.size();
  stats.median = percentile(0.50);
  stats.p90 = percentile(0.90);
  stats.overlap05 = static_cast<double>(within05) / distances.size();
  stats.overlap2 = static_cast<double>(within2) / distances.size();
  stats.overlap8 = static_cast<double>(within8) / distances.size();
  return stats;
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: pcl_geometry_probe <dataset_root> <map_pcd> <pose_matrices.csv> <output_dir>\n";
    return 2;
  }
  try
  {
    const std::string dataset_root = argv[1];
    const std::string map_path = argv[2];
    const std::string pose_path = argv[3];
    const std::string output_dir = argv[4];
    Cloud::Ptr raw_map(new Cloud());
    if (pcl::io::loadPCDFile(map_path, *raw_map) != 0) throw std::runtime_error("cannot load map PCD");
    if (raw_map->size() != 10'277'007) throw std::runtime_error("unexpected lossless map point count");
    const Cloud::Ptr target = preprocessMap(raw_map);
    if (target->size() != 3'142'751) throw std::runtime_error("P41 NDT target voxel count mismatch");
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(target);

    std::ifstream poses(pose_path);
    if (!poses) throw std::runtime_error("cannot read pose matrix CSV");
    std::ofstream results(output_dir + "/three_frame_geometry.csv");
    if (!results) throw std::runtime_error("cannot create geometry CSV");
    results << "role,scan_file,scan_ns,hypothesis,raw_points,source_points,target_points,nn_median_m,nn_p90_m,overlap_0p5m,overlap_2m,overlap_8m,min_x,min_y,min_z,max_x,max_y,max_z\n";
    std::string line;
    std::getline(poses, line);  // Header.
    int count = 0;
    while (std::getline(poses, line))
    {
      if (line.empty()) continue;
      const auto fields = splitCsv(line);
      if (fields.size() != 20) throw std::runtime_error("pose matrix row must have 20 CSV fields");
      const std::string role = fields[0];
      const std::string scan_file = fields[1];
      const std::string stamp = fields[2];
      const std::string hypothesis = fields[3];
      Eigen::Matrix4d T;
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          T(r, c) = std::stod(fields[4 + 4 * r + c]);
      if (!T.allFinite() || (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-10 ||
          (T.block<3,3>(0,0).transpose() * T.block<3,3>(0,0) - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
          std::abs(T.block<3,3>(0,0).determinant() - 1.0) > 1e-6)
        throw std::runtime_error("invalid pose transform in CSV");

      const std::string scan_path = dataset_root + "/raw_format/ouster_scan/" + scan_file;
      Cloud::Ptr raw_source(new Cloud());
      if (pcl::io::loadPCDFile(scan_path, *raw_source) != 0) throw std::runtime_error("cannot load scan PCD: " + scan_file);
      const std::size_t raw_count = raw_source->size();
      const Cloud::Ptr source = preprocessSource(raw_source);
      if (source->size() != 1400) throw std::runtime_error("P41 source point cap not reached");
      Cloud::Ptr projected(new Cloud());
      const Geometry stats = measure(source, T, tree, projected);
      const std::string projected_path = output_dir + "/projection_" + role + "_" + hypothesis + ".pcd";
      if (pcl::io::savePCDFileBinary(projected_path, *projected) != 0)
        throw std::runtime_error("cannot save projected source cloud");
      results << std::setprecision(17) << role << ',' << scan_file << ',' << stamp << ',' << hypothesis << ','
              << raw_count << ',' << stats.points << ',' << target->size() << ',' << stats.median << ',' << stats.p90 << ','
              << stats.overlap05 << ',' << stats.overlap2 << ',' << stats.overlap8 << ','
              << stats.minimum.x() << ',' << stats.minimum.y() << ',' << stats.minimum.z() << ','
              << stats.maximum.x() << ',' << stats.maximum.y() << ',' << stats.maximum.z() << '\n';
      std::cout << role << ' ' << hypothesis << " median=" << stats.median
                << " p90=" << stats.p90 << " overlap05=" << stats.overlap05 << '\n';
      ++count;
    }
    if (count != 6) throw std::runtime_error("expected exactly six fixed pose/frame geometry cases");
    results.close();
    std::cout << "P43_GEOMETRY_COMPLETE cases=" << count << " target_points=" << target->size()
              << " ICP_calls=0 NDT_align_calls=0\n";
  }
  catch (const std::exception &error)
  {
    std::cerr << "P43_GEOMETRY_FAILED: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
