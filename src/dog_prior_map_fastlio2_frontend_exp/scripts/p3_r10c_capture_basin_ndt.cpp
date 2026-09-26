#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
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

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

void finalize(const Cloud::Ptr& cloud) {
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf, double z_leaf, int cap) {
  Cloud::Ptr down(new Cloud);
  if (leaf > 0.01) {
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                      static_cast<float>(z_leaf));
    voxel.setInputCloud(input);
    voxel.filter(*down);
  } else {
    *down = *input;
  }
  if (cap > 0 && static_cast<int>(down->size()) > cap) {
    Cloud::Ptr sampled(new Cloud);
    sampled->reserve(static_cast<std::size_t>(cap));
    const double increment = static_cast<double>(down->size() - 1) /
                            static_cast<double>(std::max(cap - 1, 1));
    for (int i = 0; i < cap; ++i) {
      sampled->push_back(down->points[static_cast<std::size_t>(std::llround(i * increment))]);
    }
    finalize(sampled);
    return sampled;
  }
  finalize(down);
  return down;
}

Cloud::Ptr loadTarget(const std::string& path) {
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0) {
    throw std::runtime_error("failed to load map PCD: " + path);
  }
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const auto& point : raw->points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      finite->push_back(point);
    }
  }
  finalize(finite);
  Cloud::Ptr map = voxelDown(finite, 0.15, 0.15, 0);
  return voxelDown(map, 0.15, 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const auto& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 0.25, 1400);
}

std::vector<std::string> splitTabs(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, '\t')) fields.push_back(field);
  return fields;
}

Eigen::Matrix4f poseFromFields(const std::vector<std::string>& f, std::size_t offset) {
  const float x = std::stof(f.at(offset + 0));
  const float y = std::stof(f.at(offset + 1));
  const float z = std::stof(f.at(offset + 2));
  Eigen::Quaternionf q(std::stof(f.at(offset + 6)), std::stof(f.at(offset + 3)),
                      std::stof(f.at(offset + 4)), std::stof(f.at(offset + 5)));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f) {
    throw std::runtime_error("invalid quaternion in manifest");
  }
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
  return pose;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: p3_r10c_capture_basin_ndt MAP.pcd MANIFEST.tsv OUTPUT.csv\n";
    return 2;
  }
  try {
    const Cloud::Ptr target = loadTarget(argv[1]);
    std::ifstream manifest(argv[2]);
    std::ofstream output(argv[3]);
    if (!manifest || !output) throw std::runtime_error("cannot open manifest/output");

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setInputTarget(target);
    ndt.setResolution(0.8);
    ndt.setStepSize(0.08);
    ndt.setTransformationEpsilon(0.001);
    ndt.setMaximumIterations(40);

    output << "frame_stamp\tcrossing_group\talpha\tsource_points\ttarget_points\tconverged\titerations\tfitness\t"
              "final_tx\tfinal_ty\tfinal_tz\tfinal_qx\tfinal_qy\tfinal_qz\tfinal_qw\n";
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(manifest, line)) {
      ++line_number;
      if (line.empty() || line[0] == '#') continue;
      const auto f = splitTabs(line);
      if (f.size() != 11) throw std::runtime_error("manifest line must have 11 tab fields");
      Cloud::Ptr raw(new Cloud);
      if (pcl::io::loadPCDFile<Point>(f.at(3), *raw) != 0) {
        throw std::runtime_error("failed to load source cloud: " + f.at(3));
      }
      const Cloud::Ptr source = preprocessSource(raw);
      if (source->empty()) throw std::runtime_error("preprocessed source cloud is empty");
      ndt.setInputSource(source);
      const Eigen::Matrix4f guess = poseFromFields(f, 4);
      Cloud aligned;
      ndt.align(aligned, guess);
      const Eigen::Matrix4f final = ndt.getFinalTransformation();
      Eigen::Quaternionf q(final.block<3, 3>(0, 0));
      q.normalize();
      output << std::setprecision(12) << f.at(0) << '\t' << f.at(1) << '\t' << f.at(2) << '\t'
             << source->size() << '\t' << target->size() << '\t' << (ndt.hasConverged() ? 1 : 0)
             << '\t' << ndt.getFinalNumIteration() << '\t' << ndt.getFitnessScore() << '\t'
             << final(0, 3) << '\t' << final(1, 3) << '\t' << final(2, 3) << '\t'
             << q.x() << '\t' << q.y() << '\t' << q.z() << '\t' << q.w() << '\n';
      if (!output) throw std::runtime_error("failed writing capture output");
      std::cerr << "capture row " << line_number << ": stamp=" << f.at(0)
                << " alpha=" << f.at(2) << " points=" << source->size()
                << " converged=" << ndt.hasConverged() << " iter=" << ndt.getFinalNumIteration()
                << " fitness=" << ndt.getFitnessScore() << '\n';
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "capture-basin error: " << e.what() << '\n';
    return 1;
  }
}
