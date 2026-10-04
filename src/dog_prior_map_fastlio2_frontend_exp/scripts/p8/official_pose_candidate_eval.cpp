#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace paper = dog_prior_map_fastlio2_frontend_exp;
namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Isometry = Eigen::Isometry3d;

class ScorableNdt : public pcl::NormalDistributionsTransform<Point, Point> {
 public:
  double scoreAtGuess(const Eigen::Matrix4f& guess, Cloud& transformed) {
    // PCL 1.10 recomputes these constants at the start of each
    // computeTransformation(); mirror that exact setup before calling its
    // protected derivative/objective routine directly.
    const double gauss_c1 = 10.0 * (1.0 - this->outlier_ratio_);
    const double gauss_c2 = this->outlier_ratio_ /
        std::pow(static_cast<double>(this->resolution_), 3.0);
    const double gauss_d3 = -std::log(gauss_c2);
    this->gauss_d1_ = -std::log(gauss_c1 + gauss_c2) - gauss_d3;
    this->gauss_d2_ = -2.0 * std::log(
        (-std::log(gauss_c1 * std::exp(-0.5) + gauss_c2) - gauss_d3) /
        this->gauss_d1_);
    pcl::transformPointCloud(*this->input_, transformed, guess);
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> pose;
    pose.matrix() = guess;
    const Eigen::Vector3f translation = pose.translation();
    const Eigen::Vector3f rotation = pose.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> p, gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    p << translation.cast<double>(), rotation.cast<double>();
    this->point_gradient_.setZero();
    this->point_gradient_.block<3, 3>(0, 0).setIdentity();
    this->point_hessian_.setZero();
    return this->computeDerivatives(gradient, hessian, transformed, p, false);
  }
};

struct CloudMetrics {
  std::size_t count = 0;
  double overlap_020 = 0.0;
  double overlap_030 = 0.0;
  double overlap_050 = 0.0;
  double overlap_100 = 0.0;
  double nn_mean = 0.0;
  double nn_median = 0.0;
  double nn_p90 = 0.0;
  double nn_p95 = 0.0;
  double nn_mse = 0.0;
  double bbox_fraction = 0.0;
};

struct Candidate {
  std::string name;
  std::string tag;
  Isometry map_T_lidar = Isometry::Identity();
};

struct RotationProjectionStats {
  double orthogonality_error = 0.0;
  double determinant = 0.0;
  double projection_frobenius = 0.0;
};

struct LowVarianceGyroReference {
  uint64_t start_ns = 0;
  uint64_t end_ns = 0;
  Eigen::Vector3d gyro_reference_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_std = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_std = Eigen::Vector3d::Zero();
};

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void finalize(Cloud::Ptr& cloud) {
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                     static_cast<float>(leaf));
  filter.setInputCloud(input);
  filter.filter(*down);
  finalize(down);
  return down;
}

Cloud::Ptr loadBaselineTarget(const std::string& path,
                              const paper::CurrentFrameNdtParameters& parameters) {
  Cloud::Ptr raw(new Cloud);
  require(pcl::io::loadPCDFile<Point>(path, *raw) == 0, "cannot_load_map_pcd");
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (pcl::isFinite(point)) finite->push_back(point);
  finalize(finite);
  return voxelDown(voxelDown(finite, parameters.map_voxel_m),
                   parameters.target_voxel_m);
}

Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d& input,
                             RotationProjectionStats* stats = nullptr) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(input,
      Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  const Eigen::Matrix3d projected =
      svd.matrixU() * correction * svd.matrixV().transpose();
  if (stats != nullptr) {
    stats->orthogonality_error =
        (input.transpose() * input - Eigen::Matrix3d::Identity()).norm();
    stats->determinant = input.determinant();
    stats->projection_frobenius = (projected - input).norm();
  }
  return projected;
}

Eigen::Matrix3d officialYamlRotationInput() {
  Eigen::Matrix3d rotation;
  rotation << 0.135990, -0.990409, -0.024406,
              0.990705,  0.136027,  0.000140,
              0.003181, -0.024198,  0.999702;
  return rotation;
}

Isometry transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
  Isometry result = Isometry::Identity();
  result.linear() = rotation;
  result.translation() = translation;
  return result;
}

Isometry projectToRigidIsometry(const Isometry& input) {
  Isometry result = input;
  result.linear() = projectToSO3(input.linear());
  return result;
}

Isometry parseMapFrameTransform(const std::string& csv) {
  std::stringstream input(csv);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 4; ++col) {
      require(static_cast<bool>(input >> matrix(row, col)),
              "invalid_map_frame_transform_value");
      if (row != 2 || col != 3) {
        char delimiter = '\0';
        require(static_cast<bool>(input >> delimiter) && delimiter == ',',
                "invalid_map_frame_transform_delimiter");
      }
    }
  }
  input >> std::ws;
  require(input.eof(), "extra_map_frame_transform_values");
  Isometry result = Isometry::Identity();
  result.matrix() = matrix;
  require((result.linear().transpose() * result.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
          std::abs(result.linear().determinant() - 1.0) < 1e-6,
          "map_frame_transform_rotation_not_proper");
  result.linear() = projectToSO3(result.linear());
  return result;
}

Isometry yamlTransform(RotationProjectionStats* stats = nullptr) {
  return transform(projectToSO3(officialYamlRotationInput(), stats),
                   Eigen::Vector3d(1.968147, -6.879292, -0.896125));
}

Eigen::Matrix3d officialImuLidarRotationInput() {
  Eigen::Matrix3d rotation;
  rotation << 0.999212900, -0.000519121,  0.004000000,
              0.000516111,  0.999218492, -0.000939132,
             -0.004000000,  0.000802565,  0.999993652;
  return rotation;
}

Isometry officialTImuLidar(RotationProjectionStats* stats = nullptr) {
  return transform(projectToSO3(officialImuLidarRotationInput(), stats),
                   Eigen::Vector3d(0.080000000, 0.029000000, 0.030000000));
}

LowVarianceGyroReference earliestCausalLowVarianceGyroReference(
    const paper::P7ImuVector& imu, uint64_t reference_stamp_ns) {
  constexpr std::size_t count = 200;
  constexpr double accel_std_limit = 0.5;
  constexpr double gyro_std_limit = 0.05;
  require(imu.size() >= count, "insufficient_imu_for_gyro_reference_window");
  std::vector<Eigen::Vector3d> acc_sum(imu.size() + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gyro_sum(imu.size() + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> acc_sq(imu.size() + 1, Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> gyro_sq(imu.size() + 1, Eigen::Vector3d::Zero());
  for (std::size_t i = 0; i < imu.size(); ++i) {
    acc_sum[i + 1] = acc_sum[i] + imu[i].acceleration;
    gyro_sum[i + 1] = gyro_sum[i] + imu[i].angular_velocity;
    acc_sq[i + 1] = acc_sq[i] + imu[i].acceleration.array().square().matrix();
    gyro_sq[i + 1] = gyro_sq[i] + imu[i].angular_velocity.array().square().matrix();
  }
  for (std::size_t first = 0; first + count <= imu.size(); ++first) {
    const std::size_t last = first + count - 1;
    if (imu[last].stamp_ns > reference_stamp_ns) break;
    const Eigen::Vector3d mean_acc = (acc_sum[first + count] - acc_sum[first]) /
        static_cast<double>(count);
    const Eigen::Vector3d mean_gyro = (gyro_sum[first + count] - gyro_sum[first]) /
        static_cast<double>(count);
    const Eigen::Vector3d acc_variance =
        ((acc_sq[first + count] - acc_sq[first]) -
         static_cast<double>(count) * mean_acc.array().square().matrix()) /
        static_cast<double>(count - 1);
    const Eigen::Vector3d gyro_variance =
        ((gyro_sq[first + count] - gyro_sq[first]) -
         static_cast<double>(count) * mean_gyro.array().square().matrix()) /
        static_cast<double>(count - 1);
    const Eigen::Vector3d accel_std = acc_variance.cwiseMax(0.0).cwiseSqrt();
    const Eigen::Vector3d gyro_std = gyro_variance.cwiseMax(0.0).cwiseSqrt();
    if (accel_std.maxCoeff() <= accel_std_limit &&
        gyro_std.maxCoeff() <= gyro_std_limit) {
      LowVarianceGyroReference result;
      result.start_ns = imu[first].stamp_ns;
      result.end_ns = imu[last].stamp_ns;
      result.gyro_reference_mean = mean_gyro;
      result.accel_std = accel_std;
      result.gyro_std = gyro_std;
      return result;
    }
  }
  throw std::runtime_error("no_causal_200_sample_low_variance_window_before_s67");
}

Eigen::Vector3d interpolatedGyro(const paper::P7ImuVector& imu, uint64_t stamp_ns) {
  const auto after = std::lower_bound(imu.begin(), imu.end(), stamp_ns,
      [](const paper::ImuSample& sample, uint64_t stamp) { return sample.stamp_ns < stamp; });
  if (after == imu.begin()) {
    require(after != imu.end() && after->stamp_ns == stamp_ns,
            "imu_does_not_cover_point_time");
    return after->angular_velocity;
  }
  if (after == imu.end()) {
    require(imu.back().stamp_ns == stamp_ns, "imu_does_not_cover_point_time");
    return imu.back().angular_velocity;
  }
  if (after->stamp_ns == stamp_ns) return after->angular_velocity;
  const auto before = after - 1;
  const double alpha = static_cast<double>(stamp_ns - before->stamp_ns) /
      static_cast<double>(after->stamp_ns - before->stamp_ns);
  return (1.0 - alpha) * before->angular_velocity + alpha * after->angular_velocity;
}

Eigen::Matrix3d integrateGyroForward(const paper::P7ImuVector& imu,
    const Eigen::Vector3d& gyro_reference_mean, uint64_t start_ns, uint64_t end_ns) {
  require(end_ns >= start_ns, "invalid_forward_gyro_interval");
  if (end_ns == start_ns) return Eigen::Matrix3d::Identity();
  std::vector<uint64_t> knots{start_ns};
  auto next = std::upper_bound(imu.begin(), imu.end(), start_ns,
      [](uint64_t stamp, const paper::ImuSample& sample) { return stamp < sample.stamp_ns; });
  for (; next != imu.end() && next->stamp_ns < end_ns; ++next)
    knots.push_back(next->stamp_ns);
  knots.push_back(end_ns);
  Eigen::Matrix3d delta = Eigen::Matrix3d::Identity();
  for (std::size_t i = 1; i < knots.size(); ++i) {
    const uint64_t t0 = knots[i - 1], t1 = knots[i];
    const Eigen::Vector3d w0 = interpolatedGyro(imu, t0) - gyro_reference_mean;
    const Eigen::Vector3d w1 = interpolatedGyro(imu, t1) - gyro_reference_mean;
    const Eigen::Vector3d omega = 0.5 * (w0 + w1);
    const double dt = static_cast<double>(t1 - t0) * 1.0e-9;
    const Eigen::Vector3d rotation_vector = omega * dt;
    const double angle = rotation_vector.norm();
    if (angle > 1.0e-15)
      delta = (delta * Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix()).eval();
  }
  return delta;
}

Eigen::Matrix3d imuRotationFromReferenceToPoint(const paper::P7ImuVector& imu,
    const Eigen::Vector3d& gyro_reference_mean, uint64_t reference_ns,
    uint64_t point_ns) {
  if (point_ns >= reference_ns)
    return integrateGyroForward(imu, gyro_reference_mean, reference_ns, point_ns);
  return integrateGyroForward(imu, gyro_reference_mean, point_ns, reference_ns).transpose();
}

paper::RegistrationCloud rotationallyDeskewToReference(
    const paper::P7TimedLidarVector& timed, const paper::P7ImuVector& imu,
    const LowVarianceGyroReference& gyro_reference, uint64_t reference_ns) {
  const Isometry T_imu_lidar = officialTImuLidar();
  const Eigen::Matrix3d R_imu_lidar = T_imu_lidar.linear();
  const Eigen::Vector3d t_imu_lidar = T_imu_lidar.translation();
  paper::RegistrationCloud cloud;
  cloud.reserve(timed.size());
  for (const auto& point : timed) {
    const Eigen::Matrix3d delta_imu = imuRotationFromReferenceToPoint(
        imu, gyro_reference.gyro_reference_mean, reference_ns, point.stamp_ns);
    const Eigen::Matrix3d R_lidar_ref_lidar_point =
        R_imu_lidar.transpose() * delta_imu * R_imu_lidar;
    const Eigen::Vector3d t_lidar_ref_lidar_point = R_imu_lidar.transpose() *
        (delta_imu - Eigen::Matrix3d::Identity()) * t_imu_lidar;
    const Eigen::Vector3d p_reference = R_lidar_ref_lidar_point * point.position +
        t_lidar_ref_lidar_point;
    cloud.push_back({static_cast<float>(p_reference.x()),
                     static_cast<float>(p_reference.y()),
                     static_cast<float>(p_reference.z())});
  }
  return cloud;
}

paper::Pose3d toPose(const Isometry& isometry) {
  paper::Pose3d pose;
  pose.position = isometry.translation();
  pose.orientation = Eigen::Quaterniond(isometry.linear()).normalized();
  return pose;
}

Isometry toIsometry(const paper::Pose3d& pose) {
  Isometry result = Isometry::Identity();
  result.linear() = pose.orientation.toRotationMatrix();
  result.translation() = pose.position;
  return result;
}

std::vector<Candidate> candidates() {
  const Isometry T_yaml = yamlTransform();
  const Isometry T_imu_lidar = officialTImuLidar();
  return {
      {"A_DIRECT_AS_T_MAP_LIDAR", "A", T_yaml},
      {"B_INVERSE_AS_T_MAP_LIDAR", "B", T_yaml.inverse()},
      {"C_DIRECT_AS_T_MAP_IMU", "C", T_yaml * T_imu_lidar},
      {"D_INVERSE_AS_T_MAP_IMU", "D", T_yaml.inverse() * T_imu_lidar}};
}

CloudMetrics measure(const Cloud::ConstPtr& target, const Cloud::ConstPtr& source,
                     const Isometry& map_T_lidar) {
  require(!target->empty(), "empty_target_cloud");
  require(!source->empty(), "empty_source_cloud");
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(target);
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*target, min_pt, max_pt);
  std::vector<float> distances;
  distances.reserve(source->size());
  std::size_t in020 = 0, in030 = 0, in050 = 0, in100 = 0, inside = 0;
  double sum_sq = 0.0;
  std::vector<int> index(1);
  std::vector<float> squared_distance(1);
  for (const Point& source_point : source->points) {
    const Eigen::Vector3d map_point = map_T_lidar *
        Eigen::Vector3d(source_point.x, source_point.y, source_point.z);
    const Point query(static_cast<float>(map_point.x()),
                      static_cast<float>(map_point.y()),
                      static_cast<float>(map_point.z()));
    if (query.x >= min_pt.x() && query.x <= max_pt.x() &&
        query.y >= min_pt.y() && query.y <= max_pt.y() &&
        query.z >= min_pt.z() && query.z <= max_pt.z()) ++inside;
    require(tree.nearestKSearch(query, 1, index, squared_distance) == 1,
            "nearest_neighbor_search_failed");
    const double distance = std::sqrt(std::max(0.0f, squared_distance[0]));
    distances.push_back(static_cast<float>(distance));
    sum_sq += static_cast<double>(squared_distance[0]);
    in020 += distance < 0.20;
    in030 += distance < 0.30;
    in050 += distance < 0.50;
    in100 += distance < 1.00;
  }
  std::sort(distances.begin(), distances.end());
  const auto quantile = [&distances](double q) {
    const std::size_t i = static_cast<std::size_t>(
        std::floor(q * static_cast<double>(distances.size() - 1)));
    return static_cast<double>(distances[i]);
  };
  const double denominator = static_cast<double>(source->size());
  double sum = 0.0;
  for (const float distance : distances) sum += distance;
  CloudMetrics result;
  result.count = source->size();
  result.overlap_020 = static_cast<double>(in020) / denominator;
  result.overlap_030 = static_cast<double>(in030) / denominator;
  result.overlap_050 = static_cast<double>(in050) / denominator;
  result.overlap_100 = static_cast<double>(in100) / denominator;
  result.nn_mean = sum / denominator;
  result.nn_median = quantile(0.50);
  result.nn_p90 = quantile(0.90);
  result.nn_p95 = quantile(0.95);
  result.nn_mse = sum_sq / denominator;
  result.bbox_fraction = static_cast<double>(inside) / denominator;
  return result;
}

Cloud::Ptr transformedCloud(const Cloud::ConstPtr& source, const Isometry& transform) {
  Cloud::Ptr result(new Cloud);
  result->reserve(source->size());
  for (const Point& point : source->points) {
    const Eigen::Vector3d p = transform * Eigen::Vector3d(point.x, point.y, point.z);
    result->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                         static_cast<float>(p.z()));
  }
  finalize(result);
  return result;
}

Cloud::Ptr makePclCloud(const paper::RegistrationCloud& source) {
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(source.size());
  for (const auto& point : source) cloud->emplace_back(point.x, point.y, point.z);
  finalize(cloud);
  return cloud;
}

void saveCandidateCloud(const std::string& directory, const std::string& tag,
                        const std::string& suffix, const Cloud::ConstPtr& cloud,
                        const Isometry& pose) {
  if (directory.empty()) return;
  const Cloud::Ptr aligned = transformedCloud(cloud, pose);
  const std::string path = directory + "/candidate_" + tag + "_" + suffix + ".pcd";
  require(pcl::io::savePCDFileBinary(path, *aligned) == 0,
          "cannot_write_candidate_cloud:" + path);
}

double initialNdtScore(const Cloud::ConstPtr& target, const Cloud::ConstPtr& source,
                       const Isometry& pose,
                       const paper::CurrentFrameNdtParameters& parameters) {
  ScorableNdt ndt;
  ndt.setResolution(parameters.resolution_m);
  ndt.setInputTarget(target);
  ndt.setStepSize(parameters.step_size);
  ndt.setTransformationEpsilon(parameters.transformation_epsilon);
  ndt.setMaximumIterations(parameters.maximum_iterations);
  ndt.setInputSource(source);
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess.block<3, 3>(0, 0) = pose.linear().cast<float>();
  guess.block<3, 1>(0, 3) = pose.translation().cast<float>();
  Cloud transformed;
  return ndt.scoreAtGuess(guess, transformed);
}

double rotationDistanceRad(const Isometry& a, const Isometry& b) {
  const Eigen::Quaterniond qa(a.linear()), qb(b.linear());
  return Eigen::AngleAxisd(qa.conjugate() * qb).angle();
}

void printMatrix(const Isometry& pose) {
  std::cout << "T_map_lidar=\n" << std::setprecision(12)
            << pose.matrix() << "\n";
  const Eigen::Vector3d ypr = pose.linear().eulerAngles(2, 1, 0);
  std::cout << "xyz_m=" << pose.translation().transpose()
            << " rpy_rad=" << ypr.z() << ',' << ypr.y() << ',' << ypr.x()
            << " rpy_deg=" << ypr.z() * 180.0 / M_PI << ','
            << ypr.y() * 180.0 / M_PI << ',' << ypr.x() * 180.0 / M_PI << '\n';
}

void writeMetrics(std::ostream& output, const CloudMetrics& metrics) {
  output << metrics.overlap_020 << ',' << metrics.overlap_030 << ','
         << metrics.overlap_050 << ',' << metrics.overlap_100 << ','
         << metrics.nn_mean << ',' << metrics.nn_median << ',' << metrics.nn_p90 << ','
         << metrics.nn_p95 << ',' << metrics.nn_mse << ',' << metrics.bbox_fraction;
}

void writePoseMatrix(std::ostream& output, const Isometry& pose) {
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 4; ++col)
      output << ',' << pose.matrix()(row, col);
}

void writeMetricsBeforeBbox(std::ostream& output, const CloudMetrics& metrics) {
  output << metrics.overlap_020 << ',' << metrics.overlap_030 << ','
         << metrics.overlap_050 << ',' << metrics.overlap_100 << ','
         << metrics.nn_mean << ',' << metrics.nn_median << ',' << metrics.nn_p90 << ','
         << metrics.nn_p95 << ',' << metrics.nn_mse;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 7 || argc > 9)
      throw std::runtime_error("usage: p8_official_pose_candidate_eval FILTER_SCANS_CSV "
          "RAW_TIMED_SCAN_INDEX_CSV TIMED_POINTS_BIN IMU_CSV MAP_PCD OUTPUT_CSV "
          "[PCD_OUTPUT_DIR [MAP_T_RAW_3X4_CSV]]");
    const std::string filter_path = argv[1];
    const std::string index_path = argv[2];
    const std::string points_path = argv[3];
    const std::string imu_path = argv[4];
    const std::string map_path = argv[5];
    const std::string output_path = argv[6];
    const std::string pcd_output_dir = argc >= 8 ? argv[7] : "";
    const Isometry map_T_raw = argc == 9 ? parseMapFrameTransform(argv[8]) :
                                           Isometry::Identity();

    std::cerr << "stage=read_scan_index\n";
    const auto scans = paper::readP7TimedScans(filter_path, index_path);
    const auto selected_scan = std::find_if(scans.begin(), scans.end(),
        [](const paper::P7TimedScanRecord& scan) { return scan.transaction_id == 665; });
    require(selected_scan != scans.end(), "missing_S67_transaction_665");
    require(selected_scan->scan_start_ns == 1517157286055073023ULL &&
            selected_scan->scan_end_ns == 1517157286155912472ULL &&
            selected_scan->cloud_point_count == 29067,
            "S67_transaction_does_not_match_frozen_adapter_record");

    // Screening approximation only: map bag-relative s=67 to sensor time by
    // applying the local bag-record/header offset as a unit-rate clock map.
    // This cross-clock mapping is not an official epoch contract.
    constexpr uint64_t assumed_s67_reference_ns = 1517157286063423943ULL;
    require(assumed_s67_reference_ns >= selected_scan->scan_start_ns &&
            assumed_s67_reference_ns <= selected_scan->scan_end_ns,
            "s67_reference_not_inside_transaction_665");
    std::cerr << "stage=read_S67_points point_count=" << selected_scan->cloud_point_count << '\n';
    paper::CurrentFrameNdtParameters parameters;
    const auto timed = paper::readP7PackedTimedCloud(points_path, *selected_scan);
    const auto imu = paper::readP7Imu(imu_path);
    RotationProjectionStats yaml_projection, extrinsic_projection;
    const Isometry T_yaml = yamlTransform(&yaml_projection);
    const Isometry T_imu_lidar = officialTImuLidar(&extrinsic_projection);
    const LowVarianceGyroReference gyro_reference =
        earliestCausalLowVarianceGyroReference(imu, assumed_s67_reference_ns);
    const paper::RegistrationCloud deskewed_source = rotationallyDeskewToReference(
        timed, imu, gyro_reference, assumed_s67_reference_ns);
    paper::RegistrationCloud raw_source = deskewed_source;
    std::size_t finite_points = 0;
    for (const auto& point : raw_source)
      finite_points += std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    const auto prepared = paper::preprocessRegistrationCloud(raw_source, parameters);
    const Cloud::Ptr visual_source = makePclCloud(raw_source);
    Cloud::Ptr source(new Cloud);
    source->reserve(prepared.size());
    for (const auto& point : prepared)
      source->emplace_back(point.x, point.y, point.z);
    finalize(source);

    std::cerr << "stage=load_target map=" << map_path << '\n';
    const Cloud::Ptr target = loadBaselineTarget(map_path, parameters);
    std::cerr << "stage=load_ndt_target\n";
    paper::CurrentFrameNdtRegistration registration(parameters);
    std::string reason;
    require(registration.loadMap(map_path, &reason), "baseline_ndt_map_load:" + reason);
    const auto grid = registration.targetGridLeafSizeMeters();
    std::cerr << "stage=candidates_ready source_prepared=" << prepared.size()
              << " target_points=" << target->size() << '\n';
    std::cout << std::setprecision(12) << "map_T_raw=\n" << map_T_raw.matrix() << '\n';
    std::cout << "official_yaml_rotation_projection orthogonality_error="
        << yaml_projection.orthogonality_error << " determinant=" << yaml_projection.determinant
        << " frobenius_change=" << yaml_projection.projection_frobenius
        << " projected_R=\n" << T_yaml.linear() << '\n'
        << "official_imu_lidar_rotation_projection orthogonality_error="
        << extrinsic_projection.orthogonality_error << " determinant="
        << extrinsic_projection.determinant << " frobenius_change="
        << extrinsic_projection.projection_frobenius << " input_R=\n"
        << officialImuLidarRotationInput() << "\nprojected_T_imu_lidar=\n"
        << T_imu_lidar.matrix() << '\n';
    std::ofstream output(output_path);
    require(static_cast<bool>(output), "cannot_create_output_csv");
    output << std::setprecision(17)
        << "candidate,source_raw_points,source_finite_points,source_prepared_points,target_points,"
           "target_grid_x_m,target_grid_y_m,target_grid_z_m,overlap_020,overlap_030,overlap_050,"
           "overlap_100,nn_mean_m,nn_median_m,nn_p90_m,nn_p95_m,initial_nn_mse_m2,"
           "initial_pcl_ndt_score,initial_pcl_ndt_probability,final_pcl_ndt_score_check,"
           "final_pcl_ndt_probability_check,final_probability_check_delta,bbox_fraction,"
           "ndt_converged,ndt_iterations,ndt_status,ndt_fitness_final_m2,"
           "ndt_probability_final,ndt_alignment_ms,translation_correction_m,rotation_correction_rad,"
           "final_overlap_020,final_overlap_030,final_overlap_050,final_overlap_100,"
           "final_nn_mean_m,final_nn_median_m,final_nn_p90_m,final_nn_p95_m,final_nn_mse_m2,"
           "final_bbox_fraction,final_x,final_y,final_z,final_qx,final_qy,final_qz,final_qw,"
           "evaluated_T_map_lidar_r00,evaluated_T_map_lidar_r01,evaluated_T_map_lidar_r02,"
           "evaluated_T_map_lidar_t0,evaluated_T_map_lidar_r10,evaluated_T_map_lidar_r11,"
           "evaluated_T_map_lidar_r12,evaluated_T_map_lidar_t1,evaluated_T_map_lidar_r20,"
           "evaluated_T_map_lidar_r21,evaluated_T_map_lidar_r22,evaluated_T_map_lidar_t2\n";
    output.flush();
    std::cout << std::setprecision(12)
        << "transaction_id=665 scan_start_ns=" << selected_scan->scan_start_ns
        << " scan_end_ns=" << selected_scan->scan_end_ns
        << " assumed_s67_reference_ns=" << assumed_s67_reference_ns
        << " point_count=" << selected_scan->cloud_point_count << '\n'
        << "prepared_source_points=" << prepared.size() << " finite_points="
        << finite_points << '/' << timed.size() << " target_points=" << target->size()
        << " actual_ndt_target_grid=" << grid[0] << ',' << grid[1] << ',' << grid[2] << "\n"
        << "rotational_deskew=gyro_only_ref=assumed_s67 low_variance_gyro_reference_window_ns="
        << gyro_reference.start_ns << ':' << gyro_reference.end_ns
        << " gyro_reference_mean=" << gyro_reference.gyro_reference_mean.transpose()
        << " accel_std=" << gyro_reference.accel_std.transpose() << " gyro_std="
        << gyro_reference.gyro_std.transpose() << " translational_deskew=NO\n";

    for (const Candidate& candidate : candidates()) {
      std::cerr << "stage=align candidate=" << candidate.name << '\n';
      // Canonicalize once. Initial metrics, the NDT seed, CSV and PCD exports
      // all use this exact effective rigid transform.
      const Isometry map_T_lidar = projectToRigidIsometry(map_T_raw * candidate.map_T_lidar);
      const CloudMetrics initial_metrics = measure(target, source, map_T_lidar);
      saveCandidateCloud(pcd_output_dir, candidate.tag, "aligned", visual_source, map_T_lidar);
      const double initial_ndt_score = initialNdtScore(target, source,
          map_T_lidar, parameters);
      const double initial_ndt_probability = initial_ndt_score /
          static_cast<double>(source->size());
      paper::CurrentFrameNdtResult result;
      require(registration.align(assumed_s67_reference_ns, raw_source,
                  toPose(map_T_lidar), &result, &reason),
              candidate.name + "_ndt_align:" + reason);
      const Isometry refined = toIsometry(result.raw_map_T_lidar);
      const double final_ndt_score_check = initialNdtScore(target, source, refined, parameters);
      const double final_ndt_probability_check = final_ndt_score_check /
          static_cast<double>(source->size());
      const double translation_correction =
          (refined.translation() - map_T_lidar.translation()).norm();
      const double rotation_correction = rotationDistanceRad(map_T_lidar, refined);
      const Cloud::Ptr final_source = transformedCloud(source, refined);
      const CloudMetrics final_metrics = measure(target, final_source, Isometry::Identity());
      saveCandidateCloud(pcd_output_dir, candidate.tag, "refined", visual_source, refined);
      output << candidate.name << ',' << raw_source.size() << ',' << finite_points << ','
          << prepared.size() << ',' << target->size() << ',' << grid[0] << ',' << grid[1]
          << ',' << grid[2] << ',';
      writeMetricsBeforeBbox(output, initial_metrics);
      output << ',' << initial_ndt_score << ',' << initial_ndt_probability << ','
          << final_ndt_score_check << ',' << final_ndt_probability_check << ','
          << (final_ndt_probability_check - result.transformation_probability) << ','
          << initial_metrics.bbox_fraction << ','
          << (result.converged ? 1 : 0) << ',' << result.iterations << ','
          << paper::currentFrameNdtStatusName(result.status) << ',' << result.fitness << ','
          << result.transformation_probability << ',' << result.alignment_ms << ','
          << translation_correction << ',' << rotation_correction << ',';
      writeMetrics(output, final_metrics);
      output << ',' << refined.translation().x() << ',' << refined.translation().y() << ','
          << refined.translation().z() << ',' << result.raw_map_T_lidar.orientation.x() << ','
          << result.raw_map_T_lidar.orientation.y() << ','
          << result.raw_map_T_lidar.orientation.z() << ','
          << result.raw_map_T_lidar.orientation.w();
      writePoseMatrix(output, map_T_lidar);
      output << '\n';
      output.flush();

      std::cout << "\n" << candidate.name << '\n';
      printMatrix(map_T_lidar);
      std::cout << "initial_overlap_020_030_050_100=" << initial_metrics.overlap_020 << ','
          << initial_metrics.overlap_030 << ',' << initial_metrics.overlap_050 << ','
          << initial_metrics.overlap_100 << " nn_mean_median_p90_p95="
          << initial_metrics.nn_mean << ',' << initial_metrics.nn_median << ','
          << initial_metrics.nn_p90 << ',' << initial_metrics.nn_p95
          << " initial_nn_mse_m2=" << initial_metrics.nn_mse
          << " initial_pcl_ndt_score=" << initial_ndt_score
          << " initial_pcl_ndt_probability=" << initial_ndt_probability
          << " final_pcl_ndt_score_check=" << final_ndt_score_check
          << " final_pcl_ndt_probability_check=" << final_ndt_probability_check
          << " final_probability_check_delta="
          << (final_ndt_probability_check - result.transformation_probability)
          << " bbox_fraction=" << initial_metrics.bbox_fraction << '\n'
          << "ndt_converged=" << (result.converged ? "YES" : "NO")
          << " iterations=" << result.iterations << " status="
          << paper::currentFrameNdtStatusName(result.status)
          << " final_pcl_fitness_m2=" << result.fitness
          << " final_probability=" << result.transformation_probability
          << " runtime_ms=" << result.alignment_ms
          << " correction_translation_m=" << translation_correction
          << " correction_rotation_rad=" << rotation_correction << '\n'
          << "refined_T_map_lidar=\n" << refined.matrix() << '\n'
          << "refined_overlap_020_030_050_100=" << final_metrics.overlap_020 << ','
          << final_metrics.overlap_030 << ',' << final_metrics.overlap_050 << ','
          << final_metrics.overlap_100 << " nn_mean_median_p90_p95="
          << final_metrics.nn_mean << ',' << final_metrics.nn_median << ','
          << final_metrics.nn_p90 << ',' << final_metrics.nn_p95
          << " bbox_fraction=" << final_metrics.bbox_fraction << '\n';
    }
    std::cout << "gt_used=false output_csv=" << output_path
              << " pcd_output_dir=" << pcd_output_dir << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ERROR=" << error.what() << '\n';
    return 1;
  }
}
