#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Eigen::Matrix4d parseMatrix(const char* packed_values) {
  Eigen::Matrix4d result;
  const std::string packed(packed_values);
  std::size_t begin = 0;
  for (int index = 0; index < 16; ++index) {
    const std::size_t end = packed.find(',', begin);
    const std::string token = packed.substr(begin, end == std::string::npos
        ? std::string::npos : end - begin);
    std::size_t parsed = 0;
    const double value = std::stod(token, &parsed);
    require(parsed == token.size() && std::isfinite(value), "invalid_matrix_value");
    result(index / 4, index % 4) = value;
    if (index < 15) require(end != std::string::npos, "matrix_requires_16_values");
    else require(end == std::string::npos, "matrix_has_extra_values");
    begin = end == std::string::npos ? packed.size() : end + 1;
  }
  return result;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                     static_cast<float>(leaf));
  filter.setInputCloud(input);
  filter.filter(*output);
  output->width = static_cast<std::uint32_t>(output->size());
  output->height = 1;
  output->is_dense = true;
  return output;
}

struct Metrics {
  std::array<double, 4> overlap{{0, 0, 0, 0}};
  double mean = std::numeric_limits<double>::quiet_NaN();
  double median = std::numeric_limits<double>::quiet_NaN();
  double p95 = std::numeric_limits<double>::quiet_NaN();
  double mean_squared = std::numeric_limits<double>::quiet_NaN();
};

double quantile(const std::vector<double>& sorted, double q) {
  const double index = q * static_cast<double>(sorted.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(index));
  const auto hi = static_cast<std::size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lo);
  return (1.0 - alpha) * sorted[lo] + alpha * sorted[hi];
}

Metrics overlap(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                const Eigen::Matrix4d& transform) {
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(target);
  const std::array<double, 4> thresholds{{0.2, 0.3, 0.5, 1.0}};
  std::array<std::size_t, 4> counts{{0, 0, 0, 0}};
  std::vector<double> distances;
  distances.reserve(source->size());
  double sum = 0.0;
  double squared_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> squared(1);
  for (const Point& point : source->points) {
    const Eigen::Vector4d local(point.x, point.y, point.z, 1.0);
    const Eigen::Vector4d mapped = transform * local;
    const Point query(static_cast<float>(mapped.x()), static_cast<float>(mapped.y()),
                      static_cast<float>(mapped.z()));
    require(tree.nearestKSearch(query, 1, indices, squared) == 1 &&
            std::isfinite(squared.front()) && squared.front() >= 0.0f,
            "nearest_neighbor_query_failed");
    const double distance = std::sqrt(static_cast<double>(squared.front()));
    distances.push_back(distance);
    sum += distance;
    squared_sum += distance * distance;
    for (std::size_t i = 0; i < thresholds.size(); ++i)
      if (distance < thresholds[i]) ++counts[i];
  }
  std::sort(distances.begin(), distances.end());
  Metrics result;
  for (std::size_t i = 0; i < thresholds.size(); ++i)
    result.overlap[i] = static_cast<double>(counts[i]) / source->size();
  result.mean = sum / source->size();
  result.mean_squared = squared_sum / source->size();
  result.median = quantile(distances, 0.5);
  result.p95 = quantile(distances, 0.95);
  return result;
}

void printMetrics(const char* prefix, const Metrics& metrics) {
  std::cout << prefix << "_OVERLAP_020=" << metrics.overlap[0] << '\n'
            << prefix << "_OVERLAP_030=" << metrics.overlap[1] << '\n'
            << prefix << "_OVERLAP_050=" << metrics.overlap[2] << '\n'
            << prefix << "_OVERLAP_100=" << metrics.overlap[3] << '\n'
            << prefix << "_NN_MEAN_M=" << metrics.mean << '\n'
            << prefix << "_NN_MEDIAN_M=" << metrics.median << '\n'
            << prefix << "_NN_P95_M=" << metrics.p95 << '\n'
            << prefix << "_NN_MSE_M2=" << metrics.mean_squared << '\n';
}

Cloud::Ptr loadMap(const std::string& path, const paper::CurrentFrameNdtParameters& p) {
  Cloud::Ptr raw(new Cloud);
  require(pcl::io::loadPCDFile<Point>(path, *raw) == 0, "cannot_load_raw_map");
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (pcl::isFinite(point)) finite->push_back(point);
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1;
  finite->is_dense = true;
  return voxelDown(voxelDown(finite, p.map_voxel_m), p.target_voxel_m);
}

Eigen::Matrix4d poseDelta(const Eigen::Matrix4d& initial, const Eigen::Matrix4d& terminal) {
  return terminal * initial.inverse();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 7)
      throw std::runtime_error("usage: public_raw_tx666_ndt RAW_MAP SCAN_PCD OUT_INITIAL_PCD "
          "OUT_FINAL_PCD T_WORLD_IMU_16 T_IMU_LIDAR_16");
    const Eigen::Matrix4d T_world_imu = parseMatrix(argv[5]);
    const Eigen::Matrix4d T_imu_lidar = parseMatrix(argv[6]);
    require((T_world_imu.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-12 &&
            (T_imu_lidar.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-12,
            "invalid_homogeneous_bottom_row");
    const Eigen::Matrix4d T_world_lidar = T_world_imu * T_imu_lidar;

    paper::CurrentFrameNdtParameters parameters;
    Cloud::Ptr target = loadMap(argv[1], parameters);
    Cloud::Ptr scan(new Cloud);
    require(pcl::io::loadPCDFile<Point>(argv[2], *scan) == 0, "cannot_load_deskewed_scan");
    require(scan->size() == 29063, "TX666_full_point_count_mismatch");
    scan->is_dense = true;

    paper::RegistrationCloud raw_source;
    raw_source.reserve(scan->size());
    for (const Point& point : scan->points)
      raw_source.push_back({point.x, point.y, point.z});
    const paper::RegistrationCloud prepared =
        paper::preprocessRegistrationCloud(raw_source, parameters);
    require(prepared.size() >= static_cast<std::size_t>(parameters.min_effective_points),
            "insufficient_preprocessed_source_points");
    Cloud::Ptr source(new Cloud);
    source->reserve(prepared.size());
    for (const auto& point : prepared) source->push_back(Point(point.x, point.y, point.z));
    source->width = static_cast<std::uint32_t>(source->size());
    source->height = 1;
    source->is_dense = true;

    pcl::NormalDistributionsTransform<Point, Point> ndt;
    ndt.setResolution(static_cast<float>(parameters.resolution_m));
    ndt.setInputTarget(target);
    ndt.setStepSize(parameters.step_size);
    ndt.setTransformationEpsilon(parameters.transformation_epsilon);
    ndt.setMaximumIterations(parameters.maximum_iterations);
    ndt.setInputSource(source);

    const Metrics initial_metrics = overlap(source, target, T_world_lidar);
    Cloud aligned;
    ndt.align(aligned, T_world_lidar.cast<float>());
    const Eigen::Matrix4d T_world_lidar_terminal =
        ndt.getFinalTransformation().cast<double>();
    const Metrics final_metrics = overlap(source, target, T_world_lidar_terminal);
    const Eigen::Matrix4d delta = poseDelta(T_world_lidar, T_world_lidar_terminal);
    const Eigen::Matrix3d delta_rotation = delta.block<3, 3>(0, 0);
    const double delta_orthogonality =
        (delta_rotation.transpose() * delta_rotation - Eigen::Matrix3d::Identity()).norm();
    const double delta_determinant = delta_rotation.determinant();
    const bool angular_metric_valid = delta_orthogonality < 1e-5 &&
        std::abs(delta_determinant - 1.0) < 1e-5;
    double correction_angle = std::numeric_limits<double>::quiet_NaN();
    if (angular_metric_valid) {
      const double cosine = std::max(-1.0, std::min(1.0,
          (delta_rotation.trace() - 1.0) * 0.5));
      correction_angle = std::acos(cosine);
    }

    Cloud initial_aligned;
    Cloud terminal_aligned;
    pcl::transformPointCloud(*scan, initial_aligned, T_world_lidar.cast<float>());
    pcl::transformPointCloud(*scan, terminal_aligned, T_world_lidar_terminal.cast<float>());
    require(pcl::io::savePCDFileBinary(argv[3], initial_aligned) == 0,
            "cannot_save_initial_aligned_cloud");
    require(pcl::io::savePCDFileBinary(argv[4], terminal_aligned) == 0,
            "cannot_save_terminal_aligned_cloud");

    std::cout << std::setprecision(12)
              << "MAP_RAW_POINT_COUNT=" << target->size() << '\n'
              << "TX666_RAW_SCAN_POINT_COUNT=" << scan->size() << '\n'
              << "NDT_SOURCE_POINT_COUNT=" << source->size() << '\n'
              << "NDT_TARGET_POINT_COUNT=" << target->size() << '\n'
              << "NDT_GRID_ACTUAL_M=" << parameters.resolution_m << '\n'
              << "NDT_STEP_M=" << parameters.step_size << '\n'
              << "NDT_EPSILON=" << parameters.transformation_epsilon << '\n'
              << "NDT_MAX_ITERATIONS=" << parameters.maximum_iterations << '\n'
              << "SVD_USED=NO\nNORMALIZED_MAP_USED=NO\nGT_USED=NO\n"
              << "T_WORLD_IMU=\n" << T_world_imu << '\n'
              << "T_IMU_LIDAR=\n" << T_imu_lidar << '\n'
              << "T_WORLD_LIDAR=\n" << T_world_lidar << '\n'
              << "T_WORLD_LIDAR_ROTATION_ORTHOGONALITY_FROBENIUS="
              << (T_world_lidar.block<3, 3>(0, 0).transpose() *
                  T_world_lidar.block<3, 3>(0, 0) - Eigen::Matrix3d::Identity()).norm() << '\n'
              << "T_WORLD_LIDAR_ROTATION_DETERMINANT="
              << T_world_lidar.block<3, 3>(0, 0).determinant() << '\n';
    printMetrics("INITIAL", initial_metrics);
    std::cout << "INITIAL_FITNESS_NN_MSE_M2=" << initial_metrics.mean_squared << '\n'
              << "PCL_CONVERGED=" << (ndt.hasConverged() ? "YES" : "NO") << '\n'
              << "NDT_ITERATIONS=" << ndt.getFinalNumIteration() << '\n'
              << "NDT_STATUS=" << paper::currentFrameNdtStatusName(
                    paper::classifyNdtTerminal(ndt.hasConverged(), ndt.getFinalNumIteration(),
                        parameters.maximum_iterations, T_world_lidar_terminal.allFinite(),
                        std::isfinite(ndt.getFitnessScore()))) << '\n'
              << "TRANSLATION_CORRECTION_M="
              << (T_world_lidar_terminal.block<3, 1>(0, 3) -
                  T_world_lidar.block<3, 1>(0, 3)).norm() << '\n'
              << "RELATIVE_ROTATION_ORTHOGONALITY_FROBENIUS=" << delta_orthogonality << '\n'
              << "RELATIVE_ROTATION_DETERMINANT=" << delta_determinant << '\n'
              << "ROTATION_CORRECTION_RAD=";
    if (angular_metric_valid) std::cout << correction_angle << '\n';
    else std::cout << "UNDEFINED_NONRIGID_RELATIVE_MATRIX\n";
    std::cout << "FINAL_PCL_FITNESS_M2=" << ndt.getFitnessScore() << '\n';
    printMetrics("FINAL", final_metrics);
    std::cout << "T_WORLD_LIDAR_TERMINAL=\n" << T_world_lidar_terminal << '\n'
              << "INITIAL_ALIGNED_PCD=" << argv[3] << '\n'
              << "FINAL_ALIGNED_PCD=" << argv[4] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PUBLIC_RAW_TX666_NDT_FAILED=" << error.what() << '\n';
    return 2;
  }
}
