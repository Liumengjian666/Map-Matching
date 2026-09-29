// Offline P6 runner. Mature FAST-LIO2/IKFoM and P5 PCL-NDT helpers are reused
// verbatim by including their validated analysis translation units under
// renamed entry points; this code is not linked into the ROS runtime.
#define main p4_i2_unused_entry_point
#include "p4_i2_state_contamination_replay.cpp"
#undef main

#define main p5_i1_unused_entry_point
#include "p5_i1_ndt_mode_landscape.cpp"
#undef main

#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"
#include "p6_i4_basin_margin_math.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <dcreg.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace p6_i1 {
using namespace dog_prior_map_fastlio2_frontend_exp;

enum class R2Policy {
  LEGACY_BASE_NO_GATE,
  ADAPTIVE_NO_GATE,
  BASE_SELECTED_NIS,
  ADAPTIVE_SELECTED_NIS
};

const char* r2PolicyName(R2Policy policy) {
  switch (policy) {
    case R2Policy::LEGACY_BASE_NO_GATE: return "LEGACY_BASE_NO_GATE";
    case R2Policy::ADAPTIVE_NO_GATE: return "ADAPTIVE_NO_GATE";
    case R2Policy::BASE_SELECTED_NIS: return "BASE_SELECTED_NIS";
    case R2Policy::ADAPTIVE_SELECTED_NIS: return "ADAPTIVE_SELECTED_NIS";
  }
  return "UNKNOWN_R2_POLICY";
}

R2Policy parseR2Policy(const std::string& value) {
  if (value == "LEGACY_BASE_NO_GATE") return R2Policy::LEGACY_BASE_NO_GATE;
  if (value == "ADAPTIVE_NO_GATE") return R2Policy::ADAPTIVE_NO_GATE;
  if (value == "BASE_SELECTED_NIS") return R2Policy::BASE_SELECTED_NIS;
  if (value == "ADAPTIVE_SELECTED_NIS") return R2Policy::ADAPTIVE_SELECTED_NIS;
  throw std::runtime_error("unsupported_r2_policy:" + value);
}

bool r2PolicyUsesAdaptiveNoise(R2Policy policy) {
  return policy == R2Policy::ADAPTIVE_NO_GATE ||
         policy == R2Policy::ADAPTIVE_SELECTED_NIS;
}

bool r2PolicyUsesNisGate(R2Policy policy) {
  return policy == R2Policy::BASE_SELECTED_NIS ||
         policy == R2Policy::ADAPTIVE_SELECTED_NIS;
}

Eigen::Matrix<double, 6, 6> makeBasePoseNoise(
    const RuntimeParameters& parameters) {
  Eigen::Matrix<double, 6, 6> noise = Eigen::Matrix<double, 6, 6>::Zero();
  noise.block<3, 3>(0, 0) = parameters.pose_position_sigma_m *
      parameters.pose_position_sigma_m * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(3, 3) = parameters.pose_rotation_sigma_rad *
      parameters.pose_rotation_sigma_rad * Eigen::Matrix3d::Identity();
  return noise;
}

struct GeometricSearchDiagnostic {
  std::size_t sampled_source_points = 0;
  std::size_t queries_with_neighbors_r = 0;
  std::size_t queries_with_neighbors_2r = 0;
  std::size_t total_neighbors_r = 0;
  std::size_t total_neighbors_2r = 0;
  bool valid = false;
};

class GeometricNdt : public AuditedNdt {
 public:
  using LeafConstPtr = typename AuditedNdt::TargetGridLeafConstPtr;

  GeometricSearchDiagnostic diagnoseSearch(
      const Cloud& source, const Eigen::Matrix4f& map_T_lidar,
      double resolution) {
    GeometricSearchDiagnostic result;
    if (source.empty() || !map_T_lidar.allFinite() ||
        !std::isfinite(resolution) || resolution <= 0.0)
      return result;
    const Eigen::Matrix3f rotation_float = map_T_lidar.block<3, 3>(0, 0);
    if ((rotation_float.transpose() * rotation_float -
         Eigen::Matrix3f::Identity()).norm() > 1e-3f ||
        std::abs(rotation_float.determinant() - 1.0f) > 1e-3f ||
        (map_T_lidar.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-5f)
      return result;

    const std::size_t sample_count = std::min<std::size_t>(128, source.size());
    result.sampled_source_points = sample_count;
    const double increment = sample_count > 1
        ? static_cast<double>(source.size() - 1) /
              static_cast<double>(sample_count - 1)
        : 0.0;
    const Eigen::Matrix3d rotation = rotation_float.cast<double>();
    const Eigen::Vector3d translation =
        map_T_lidar.block<3, 1>(0, 3).cast<double>();
    std::vector<LeafConstPtr> leaves_r;
    std::vector<float> distances_r;
    std::vector<LeafConstPtr> leaves_2r;
    std::vector<float> distances_2r;
    std::size_t valid_queries = 0;
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const std::size_t index = sample_count > 1
          ? static_cast<std::size_t>(std::llround(sample * increment)) : 0;
      const Point& point = source.points[index];
      if (!pcl::isFinite(point)) continue;
      const Eigen::Vector3d query = rotation *
          Eigen::Vector3d(point.x, point.y, point.z) + translation;
      if (!query.allFinite()) continue;
      const Point query_point(static_cast<float>(query.x()),
                              static_cast<float>(query.y()),
                              static_cast<float>(query.z()));
      leaves_r.clear();
      distances_r.clear();
      this->target_cells_.radiusSearch(query_point, resolution,
                                       leaves_r, distances_r);
      leaves_2r.clear();
      distances_2r.clear();
      this->target_cells_.radiusSearch(query_point, 2.0 * resolution,
                                       leaves_2r, distances_2r);
      ++valid_queries;
      const std::size_t count_r = std::min(leaves_r.size(), distances_r.size());
      const std::size_t count_2r = std::min(leaves_2r.size(), distances_2r.size());
      result.total_neighbors_r += count_r;
      result.total_neighbors_2r += count_2r;
      if (count_r > 0) ++result.queries_with_neighbors_r;
      if (count_2r > 0) ++result.queries_with_neighbors_2r;
    }
    result.valid = valid_queries > 0;
    return result;
  }

  std::vector<reliability::GeometricObservation> geometricObservations(
      const Cloud& source, const Eigen::Matrix4f& map_T_lidar,
      double resolution) {
    std::vector<reliability::GeometricObservation> observations;
    if (!std::isfinite(resolution) || resolution <= 0.0 ||
        !map_T_lidar.allFinite())
      return observations;
    std::vector<LeafConstPtr> leaves;
    std::vector<float> squared_distances;
    const Eigen::Matrix3d rotation = map_T_lidar.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d translation = map_T_lidar.block<3, 1>(0, 3).cast<double>();
    for (const Point& point : source.points) {
      if (!pcl::isFinite(point)) continue;
      const Eigen::Vector3d rotated = rotation *
          Eigen::Vector3d(point.x, point.y, point.z);
      const Eigen::Vector3d query = rotated + translation;
      if (!query.allFinite()) continue;
      const Point query_point(static_cast<float>(query.x()),
                              static_cast<float>(query.y()),
                              static_cast<float>(query.z()));
      this->target_cells_.radiusSearch(query_point, resolution,
                                       leaves, squared_distances);
      const std::size_t count = std::min(leaves.size(), squared_distances.size());
      for (std::size_t index = 0; index < count; ++index) {
        if (!leaves[index] || !std::isfinite(squared_distances[index]) ||
            squared_distances[index] < 0.0f)
          continue;
        reliability::GeometricObservation observation;
        observation.rotated_source_map = rotated;
        observation.residual_map = query - leaves[index]->getMean();
        observation.voxel_covariance_map = leaves[index]->getCov();
        observation.nonnegative_weight = std::exp(
            -0.5 * static_cast<double>(squared_distances[index]) /
            (resolution * resolution));
        observations.push_back(std::move(observation));
      }
    }
    return observations;
  }
};

struct ScanAsset {
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  double time_s = 0.0;
  uint64_t cloud_byte_offset = 0;
  uint64_t cloud_point_count = 0;
  uint64_t expected_source_hash = 0;
  bool expected_source_hash_available = false;
  uint64_t request_cloud_hash = 0;
  Eigen::Matrix4f saved_raw = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f saved_used = Eigen::Matrix4f::Identity();
  double saved_fitness = 0.0;
  int saved_iterations = 0;
  int saved_converged = 0;
  int saved_step_limited = 0;
};

struct VisualMeasurement {
  uint64_t ref_ns = 0;
  uint64_t cur_ns = 0;
  uint64_t depth_stamp_ns = 0;
  uint64_t transaction_ref = 0;
  uint64_t transaction_cur = 0;
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  int inliers = 0;
  int detected_count = 0;
  int tracked_count = 0;
  int depth_associated_count = 0;
  double ratio = 0.0;
  double depth_fraction = std::numeric_limits<double>::quiet_NaN();
  double grid_occupancy = std::numeric_limits<double>::quiet_NaN();
  double hull_fraction = std::numeric_limits<double>::quiet_NaN();
  double median_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double reprojection = 0.0;
  bool source_valid = false;
  bool quality_metadata_available = false;
};

struct ImageTimeEvent {
  uint64_t stamp_ns = 0;
  std::size_t measurement_index = 0;
  bool is_current = false;
};

struct VisualAnchorSnapshot {
  FilterSnapshot state;
};

struct Candidate {
  int seed_index = -1;
  std::string seed_name;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  bool converged = false;
  double objective = -std::numeric_limits<double>::infinity();
  double transformation_probability = std::numeric_limits<double>::quiet_NaN();
  double objective_per_source_point = std::numeric_limits<double>::quiet_NaN();
  double fitness = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  double runtime_ms = 0.0;
  int cluster_id = -1;
  bool objective_gate = false;
  double visual_residual = std::numeric_limits<double>::quiet_NaN();
};

struct DcregSummary {
  bool attempted = false;
  bool converged = false;
  bool degenerate = false;
  std::array<bool, 6> mask = {false, false, false, false, false, false};
  Eigen::Vector3d lambda_rot = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d lambda_trans = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double cond_rot = std::numeric_limits<double>::quiet_NaN();
  double cond_trans = std::numeric_limits<double>::quiet_NaN();
  double runtime_ms = 0.0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
};

std::vector<std::string> split(const std::string& text, char delimiter) {
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, delimiter)) fields.push_back(field);
  return fields;
}

Eigen::Matrix4f parsePose7(const std::vector<std::string>& fields,
                           const std::map<std::string, std::size_t>& columns,
                           const std::string& prefix) {
  auto get = [&](const std::string& key) -> float {
    const auto found = columns.find(prefix + key);
    if (found == columns.end() || found->second >= fields.size())
      throw std::runtime_error("missing scan pose field: " + prefix + key);
    return std::stof(fields[found->second]);
  };
  Eigen::Quaternionf q(get("qw"), get("qx"), get("qy"), get("qz"));
  const Eigen::Vector3f translation(get("x"), get("y"), get("z"));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f || !translation.allFinite())
    throw std::runtime_error("invalid saved pose quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation;
  return pose;
}

std::vector<ScanAsset> readScanAssets(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_scan_asset_csv");
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty_scan_asset_csv");
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  auto value = [&columns](const std::vector<std::string>& row,
                          const std::string& name) -> const std::string& {
    const auto found = columns.find(name);
    if (found == columns.end() || found->second >= row.size())
      throw std::runtime_error("missing scan metadata field: " + name);
    return row[found->second];
  };
  auto optionalValue = [&columns](const std::vector<std::string>& row,
                                  const std::string& name,
                                  const std::string& fallback) -> std::string {
    const auto found = columns.find(name);
    return found == columns.end() || found->second >= row.size()
        ? fallback : row[found->second];
  };
  std::vector<ScanAsset> assets;
  uint64_t previous_tx = 0;
  uint64_t previous_stamp = 0;
  uint64_t expected_cloud_offset = 0;
  double previous_time_s = -std::numeric_limits<double>::infinity();
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto row = split(line, ',');
    ScanAsset asset;
    asset.transaction_id = std::stoull(value(row, "transaction_id"));
    asset.stamp_ns = std::stoull(value(row, "stamp_ns"));
    asset.time_s = std::stod(value(row, "time_s"));
    asset.cloud_byte_offset = std::stoull(value(row, "cloud_byte_offset"));
    asset.cloud_point_count = std::stoull(value(row, "cloud_point_count"));
    asset.expected_source_hash = std::stoull(
        optionalValue(row, "ndt_source_cloud_hash", "0"));
    const std::string source_hash_available = optionalValue(
        row, "ndt_source_cloud_hash_available",
        columns.count("ndt_source_cloud_hash") != 0 ? "1" : "0");
    if (source_hash_available != "0" && source_hash_available != "1")
      throw std::runtime_error("invalid ndt_source_cloud_hash_available flag");
    asset.expected_source_hash_available = source_hash_available == "1";
    asset.request_cloud_hash = std::stoull(
        optionalValue(row, "request_cloud_hash", "0"));
    asset.saved_fitness = std::stod(optionalValue(row, "fitness", "0"));
    asset.saved_iterations = std::stoi(optionalValue(row, "iterations", "0"));
    asset.saved_converged = std::stoi(optionalValue(row, "converged", "0"));
    asset.saved_step_limited = std::stoi(optionalValue(row, "step_limited", "0"));
    if (columns.count("raw_x") != 0)
      asset.saved_raw = parsePose7(row, columns, "raw_");
    if (columns.count("used_x") != 0)
      asset.saved_used = parsePose7(row, columns, "used_");
    if (asset.transaction_id != previous_tx + 1 || asset.stamp_ns <= previous_stamp ||
        asset.cloud_byte_offset != expected_cloud_offset ||
        asset.cloud_point_count == 0 || !std::isfinite(asset.time_s) ||
        asset.time_s <= previous_time_s ||
        !std::isfinite(asset.saved_fitness) || asset.saved_iterations < 0 ||
        (asset.saved_converged != 0 && asset.saved_converged != 1) ||
        (asset.saved_step_limited != 0 && asset.saved_step_limited != 1))
      throw std::runtime_error("invalid scan metadata sequence");
    previous_tx = asset.transaction_id;
    previous_stamp = asset.stamp_ns;
    previous_time_s = asset.time_s;
    expected_cloud_offset += asset.cloud_point_count * 3 * sizeof(float);
    assets.push_back(asset);
  }
  if (assets.empty()) throw std::runtime_error("scan_metadata_csv_contains_no_scans");
  return assets;
}

std::vector<VisualMeasurement> readI6dVisual(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_i6d_metric_visual_csv");
  std::string line;
  if (!std::getline(input, line))
    throw std::runtime_error("empty_i6d_metric_visual_csv");
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  auto get = [&columns](const std::vector<std::string>& fields,
                        const std::string& key) -> const std::string& {
    const auto found = columns.find(key);
    if (found == columns.end() || found->second >= fields.size())
      throw std::runtime_error("missing_i6d_visual_field:" + key);
    return fields[found->second];
  };
  const auto has_column = [&columns](const std::string& name) {
    return columns.find(name) != columns.end();
  };
  std::vector<VisualMeasurement> visual;
  std::uint64_t previous_current = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = split(line, ',');
    VisualMeasurement measurement;
    measurement.transaction_ref = std::stoull(get(fields, "transaction_ref"));
    measurement.transaction_cur = std::stoull(get(fields, "transaction_cur"));
    measurement.ref_ns = std::stoull(get(fields, "ref_ns"));
    measurement.cur_ns = std::stoull(get(fields, "cur_ns"));
    measurement.depth_stamp_ns = std::stoull(get(fields, "depth_stamp_ns"));
    measurement.translation = Eigen::Vector3d(
        std::stod(get(fields, "zx")), std::stod(get(fields, "zy")),
        std::stod(get(fields, "zz")));
    measurement.inliers = std::stoi(get(fields, "inliers"));
    measurement.ratio = std::stod(get(fields, "inlier_ratio"));
    measurement.reprojection = std::stod(get(fields, "reprojection_rmse_px"));
    measurement.source_valid = get(fields, "status") == "VALID_CAUSAL_LIDAR_DEPTH";
    const bool has_quality_metadata =
        has_column("detected") && has_column("tracked") &&
        has_column("depth_associated") && has_column("depth_fraction") &&
        has_column("grid_occupancy") && has_column("hull_fraction") &&
        has_column("median_parallax_px");
    if (has_quality_metadata) {
      measurement.detected_count = std::stoi(get(fields, "detected"));
      measurement.tracked_count = std::stoi(get(fields, "tracked"));
      measurement.depth_associated_count = std::stoi(get(fields, "depth_associated"));
      measurement.depth_fraction = std::stod(get(fields, "depth_fraction"));
      measurement.grid_occupancy = std::stod(get(fields, "grid_occupancy"));
      measurement.hull_fraction = std::stod(get(fields, "hull_fraction"));
      measurement.median_parallax_px = std::stod(get(fields, "median_parallax_px"));
      measurement.quality_metadata_available = true;
    }
    if (!measurement.source_valid ||
        measurement.transaction_cur != measurement.transaction_ref + 1 ||
        measurement.transaction_cur <= previous_current ||
        measurement.depth_stamp_ns == 0 ||
        measurement.depth_stamp_ns > measurement.ref_ns ||
        measurement.ref_ns - measurement.depth_stamp_ns > 20000000ULL ||
        measurement.ref_ns == 0 || measurement.cur_ns <= measurement.ref_ns ||
        measurement.cur_ns - measurement.ref_ns > 250000000ULL ||
        !measurement.translation.allFinite() || measurement.inliers < 0 ||
        !std::isfinite(measurement.ratio) || measurement.ratio <= 0.0 ||
        !std::isfinite(measurement.reprojection) || measurement.reprojection <= 0.0)
      throw std::runtime_error("invalid_or_noncausal_i6d_visual_pair");
    previous_current = measurement.transaction_cur;
    visual.push_back(measurement);
  }
  if (visual.empty()) throw std::runtime_error("no_valid_causal_visual_measurements");
  return visual;
}

Cloud::Ptr loadRawCloudAt(const std::string& path, const ScanAsset& asset) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot_open_packed_xyz_file");
  input.seekg(static_cast<std::streamoff>(asset.cloud_byte_offset));
  if (!input) throw std::runtime_error("cannot_seek_packed_xyz_file");
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(static_cast<std::size_t>(asset.cloud_point_count));
  for (uint64_t i = 0; i < asset.cloud_point_count; ++i) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(xyz)))
      throw std::runtime_error("truncated_packed_xyz_file");
    cloud->push_back(Point(xyz[0], xyz[1], xyz[2]));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

Eigen::Matrix4d poseMatrix(const Eigen::Matrix4f& pose) {
  return pose.cast<double>();
}

double rotationDifferenceDeg(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) {
  const Eigen::Matrix3d delta = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const double cosine = std::max(-1.0, std::min(1.0, (delta.trace() - 1.0) * 0.5));
  return std::acos(cosine) * 180.0 / M_PI;
}

void writePose7(std::ostream& output, const Eigen::Matrix4d& pose) {
  Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  q.normalize();
  output << ',' << pose(0, 3) << ',' << pose(1, 3) << ',' << pose(2, 3)
         << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

Eigen::Matrix4d limitStep(const Eigen::Matrix4d& raw,
                          const Eigen::Matrix4d& previous_used,
                          bool has_previous, bool* limited) {
  *limited = false;
  Eigen::Matrix4d used = raw;
  if (!has_previous) return used;
  constexpr double kMaxTranslation = 0.5;
  constexpr double kMaxRotationDeg = 5.0;
  const Eigen::Vector3d previous_p = previous_used.block<3, 1>(0, 3);
  const Eigen::Vector3d delta_p = raw.block<3, 1>(0, 3) - previous_p;
  if (delta_p.norm() > kMaxTranslation) {
    used.block<3, 1>(0, 3) = previous_p + delta_p.normalized() * kMaxTranslation;
    *limited = true;
  }
  const Eigen::Matrix3d previous_r = previous_used.block<3, 3>(0, 0);
  const Eigen::Matrix3d raw_r = raw.block<3, 3>(0, 0);
  const Eigen::AngleAxisd delta(previous_r.transpose() * raw_r);
  const double angle_deg = std::abs(delta.angle()) * 180.0 / M_PI;
  if (angle_deg > kMaxRotationDeg) {
    const double fraction = kMaxRotationDeg / std::max(angle_deg, 1e-12);
    used.block<3, 3>(0, 0) = Eigen::Quaterniond(previous_r).normalized()
                                  .slerp(fraction, Eigen::Quaterniond(raw_r).normalized())
                                  .toRotationMatrix();
    *limited = true;
  }
  return used;
}

void runBaseline(const p4_i2::Inputs& inputs,
                 const std::vector<ScanAsset>& assets,
                 const std::string& cloud_binary_path,
                 const std::string& map_path,
                 const std::string& params_path,
                 const std::string& output_path,
                 const std::string& trajectory_path,
                 bool strict_profile = false) {
  const char* profile_name = strict_profile ? "STRICT" : "BASELINE";
  const int maximum_iterations = strict_profile ? 80 : 40;
  const double transformation_epsilon = strict_profile ? 1e-5 : 0.001;
  constexpr double ndt_resolution = 0.8;
  constexpr double ndt_step_size = 0.08;
  using namespace dog_prior_map_fastlio2_frontend_exp;
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("frozen_target_point_count_mismatch");
  Pose3d initial_map_T_lidar, T_imu_lidar;
  const RuntimeParameters parameters = p4_i2::readParameters(
      params_path, &initial_map_T_lidar, &T_imu_lidar);
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend frontend(parameters);
  p4_i2::ImuVector initialization_imu(inputs.imu.begin(),
      inputs.imu.begin() + parameters.static_init_samples);
  std::string reason;
  if (!frontend.initializeStatic(initialization_imu, initial_map_T_lidar,
                                 T_imu_lidar, &reason))
    throw std::runtime_error("baseline_static_initialization_failed:" + reason);

  AuditedNdt ndt;
  configureNdt(ndt, target);
  ndt.setResolution(ndt_resolution);
  ndt.setStepSize(ndt_step_size);
  ndt.setTransformationEpsilon(transformation_epsilon);
  ndt.setMaximumIterations(maximum_iterations);
  std::ofstream output(output_path);
  std::ofstream trajectory(trajectory_path);
  if (!output || !trajectory) throw std::runtime_error("cannot_create_baseline_outputs");
  trajectory << std::setprecision(17);
  p4_i2::writeHeader(trajectory);
  output << std::setprecision(17)
         << "transaction_id,stamp_ns,time_s,source_points,target_points,source_hash_expected,source_hash_actual"
         << ",saved_fitness,replayed_fitness,saved_iterations,replayed_iterations,saved_converged,replayed_converged"
         << ",saved_step_limited,replayed_step_limited,predictor_t_difference_m,predictor_r_difference_deg"
         << ",raw_t_difference_m,raw_r_difference_deg,used_t_difference_m,used_r_difference_deg"
         << ",corrected_t_difference_m,corrected_r_difference_deg,ndt_objective,runtime_ms"
         << ",step_total_ms,run_profile,ndt_resolution,ndt_step_size"
         << ",ndt_epsilon,ndt_max_iterations"
         << ",predicted_lidar_x,predicted_lidar_y,predicted_lidar_z,predicted_lidar_qx,predicted_lidar_qy,predicted_lidar_qz,predicted_lidar_qw"
         << ",raw_lidar_x,raw_lidar_y,raw_lidar_z,raw_lidar_qx,raw_lidar_qy,raw_lidar_qz,raw_lidar_qw"
         << ",used_lidar_x,used_lidar_y,used_lidar_z,used_lidar_qx,used_lidar_qy,used_lidar_qz,used_lidar_qw"
         << ",corrected_lidar_x,corrected_lidar_y,corrected_lidar_z,corrected_lidar_qx,corrected_lidar_qy,corrected_lidar_qz,corrected_lidar_qw\n";

  Eigen::Matrix4d previous_used = Eigen::Matrix4d::Identity();
  bool has_previous_used = false;
  double max_translation_delta = 0.0;
  double max_rotation_delta = 0.0;
  for (std::size_t index = 0; index < assets.size(); ++index) {
    const ScanAsset& asset = assets[index];
    const p4_i2::PoseRecord& saved = inputs.scans[index];
    if (saved.transaction_id != asset.transaction_id || saved.stamp_ns != asset.stamp_ns)
      throw std::runtime_error("filter_input_and_cloud_metadata_alignment_mismatch");
    const auto step_start = std::chrono::steady_clock::now();
    const FilterSnapshot start = frontend.getState();
    p4_i2::ImuVector window = p4_i2::imuWindow(inputs.imu, start.stamp_ns, asset.stamp_ns);
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> imu_poses;
    if (!frontend.predictImuSequence(window, asset.stamp_ns, &imu_poses, &reason))
      throw std::runtime_error("baseline_imu_prediction_failed_tx_" +
                               std::to_string(asset.transaction_id) + ":" + reason);
    const FilterSnapshot predicted = frontend.getState();
    const Eigen::Matrix4d predicted_lidar =
        (p4_i2::asIsometry(predicted.map_T_imu) *
         p4_i2::asIsometry(T_imu_lidar)).matrix();

    Cloud::Ptr raw_source = loadRawCloudAt(cloud_binary_path, asset);
    const Cloud::Ptr source = preprocessSource(raw_source);
    const uint64_t source_hash = sourceCloudHash(source);
    if (!asset.expected_source_hash_available ||
        source_hash != asset.expected_source_hash)
      throw std::runtime_error("prepared_source_hash_mismatch_tx_" +
                               std::to_string(asset.transaction_id));
    ndt.setInputSource(source);
    Cloud aligned;
    const auto ndt_start = std::chrono::steady_clock::now();
    ndt.align(aligned, predicted_lidar.cast<float>());
    const double runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - ndt_start).count();
    const bool converged = ndt.hasConverged();
    if (!converged) throw std::runtime_error(std::string(profile_name) + "_ndt_not_converged_tx_" +
                                             std::to_string(asset.transaction_id));
    const Eigen::Matrix4f raw_pose = ndt.getFinalTransformation();
    const Eigen::Matrix4d raw_pose_d = poseMatrix(raw_pose);
    bool step_limited = false;
    const Eigen::Matrix4d used_pose_d = limitStep(raw_pose_d, previous_used,
                                                  has_previous_used, &step_limited);
    previous_used = used_pose_d;
    has_previous_used = true;
    Pose3d used_pose_lidar;
    used_pose_lidar.position = used_pose_d.block<3, 1>(0, 3);
    used_pose_lidar.orientation = Eigen::Quaterniond(used_pose_d.block<3, 3>(0, 0)).normalized();
    const Pose3d measurement_imu = p4_i2::lidarMeasurementToImu(
        used_pose_lidar, T_imu_lidar);
    PoseCorrectionDelta delta;
    if (!frontend.applyPoseMeasurement(measurement_imu, &delta, &reason))
      throw std::runtime_error("baseline_pose_update_failed_tx_" +
                               std::to_string(asset.transaction_id) + ":" + reason);
    const double step_total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - step_start).count();
    const FilterSnapshot corrected = frontend.getState();
    p4_i2::writeRow(trajectory, index, saved, predicted.map_T_imu,
                    predicted, corrected, T_imu_lidar);
    const Eigen::Matrix4d corrected_lidar =
        (p4_i2::asIsometry(corrected.map_T_imu) *
         p4_i2::asIsometry(T_imu_lidar)).matrix();
    const Eigen::Matrix4d saved_predictor =
        p4_i2::asIsometry(saved.saved_predictor_lidar).matrix();
    const Eigen::Matrix4d saved_corrected =
        p4_i2::asIsometry(saved.saved_corrected_lidar).matrix();
    const Eigen::Matrix4d saved_used =
        p4_i2::asIsometry(saved.used_lidar).matrix();
    const Eigen::Matrix4d saved_raw = poseMatrix(asset.saved_raw);
    const double predictor_t = (predicted_lidar.block<3, 1>(0, 3) -
                                saved_predictor.block<3, 1>(0, 3)).norm();
    const double predictor_r = rotationDifferenceDeg(predicted_lidar, saved_predictor);
    const double raw_t = (raw_pose_d.block<3, 1>(0, 3) - saved_raw.block<3, 1>(0, 3)).norm();
    const double raw_r = rotationDifferenceDeg(raw_pose_d, saved_raw);
    const double used_t = (used_pose_d.block<3, 1>(0, 3) - saved_used.block<3, 1>(0, 3)).norm();
    const double used_r = rotationDifferenceDeg(used_pose_d, saved_used);
    const double corrected_t = (corrected_lidar.block<3, 1>(0, 3) -
                                 saved_corrected.block<3, 1>(0, 3)).norm();
    const double corrected_r = rotationDifferenceDeg(corrected_lidar, saved_corrected);
    for (double delta : {predictor_t, raw_t, used_t, corrected_t})
      max_translation_delta = std::max(max_translation_delta, delta);
    for (double delta : {predictor_r, raw_r, used_r, corrected_r})
      max_rotation_delta = std::max(max_rotation_delta, delta);
    const double objective = fixedPclScore(ndt, source, raw_pose);
    output << asset.transaction_id << ',' << asset.stamp_ns << ',' << asset.time_s << ','
           << source->size() << ',' << target->size() << ',' << asset.expected_source_hash << ',' << source_hash << ','
           << asset.saved_fitness << ',' << ndt.getFitnessScore() << ','
           << asset.saved_iterations << ',' << ndt.getFinalNumIteration() << ','
           << asset.saved_converged << ',' << (converged ? 1 : 0) << ','
           << asset.saved_step_limited << ',' << (step_limited ? 1 : 0) << ','
           << predictor_t << ',' << predictor_r << ',' << raw_t << ',' << raw_r << ','
           << used_t << ',' << used_r << ',' << corrected_t << ',' << corrected_r << ','
           << objective << ',' << runtime_ms << ',' << step_total_ms << ','
           << profile_name << ',' << ndt_resolution << ',' << ndt_step_size << ','
           << transformation_epsilon << ',' << maximum_iterations;
    writePose7(output, predicted_lidar);
    writePose7(output, raw_pose_d);
    writePose7(output, used_pose_d);
    writePose7(output, corrected_lidar);
    output << '\n';
    if ((index + 1) % 100 == 0 || index + 1 == assets.size()) {
      output.flush();
      std::cerr << profile_name << "_REPLAY_PROGRESS=" << index + 1 << "/" << assets.size()
                << " raw_dt=" << raw_t << " used_dt=" << used_t
                << " corrected_dt=" << corrected_t << '\n';
    }
  }
  output.close();
  trajectory.close();
  std::cout << std::setprecision(12)
            << profile_name << "_MAX_TRANSLATION_DELTA_VS_FROZEN_BASE_M="
            << max_translation_delta << '\n'
            << profile_name << "_MAX_ROTATION_DELTA_VS_FROZEN_BASE_DEG="
            << max_rotation_delta << '\n';
  if (!strict_profile && !(max_translation_delta < 0.005 && max_rotation_delta < 0.05))
    throw std::runtime_error("BASELINE_REPLAY_GATE_FAIL; prototype modes blocked");
  std::cout << profile_name << "_REPLAY_COMPLETE frames=" << assets.size()
            << " epsilon=" << transformation_epsilon
            << " max_iterations=" << maximum_iterations << '\n';
}

std::vector<VisualMeasurement> readVisual(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_frozen_visual_csv");
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty_frozen_visual_csv");
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  auto get = [&columns](const std::vector<std::string>& fields,
                        const std::string& name) -> const std::string& {
    const auto found = columns.find(name);
    if (found == columns.end() || found->second >= fields.size())
      throw std::runtime_error("missing_visual_field:" + name);
    return fields[found->second];
  };
  std::vector<VisualMeasurement> result;
  uint64_t previous_current = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = split(line, ',');
    VisualMeasurement measurement;
    measurement.transaction_ref = std::stoull(get(fields, "transaction_ref"));
    measurement.transaction_cur = std::stoull(get(fields, "transaction_cur"));
    measurement.ref_ns = std::stoull(get(fields, "timestamp_ref_ns"));
    measurement.cur_ns = std::stoull(get(fields, "timestamp_cur_ns"));
    measurement.translation = Eigen::Vector3d(
        std::stod(get(fields, "zx")), std::stod(get(fields, "zy")),
        std::stod(get(fields, "zz")));
    measurement.inliers = std::stoi(get(fields, "inliers"));
    measurement.ratio = std::stod(get(fields, "ratio"));
    measurement.reprojection = std::stod(get(fields, "reprojection"));
    if (measurement.transaction_cur != measurement.transaction_ref + 1 ||
        measurement.transaction_cur <= previous_current ||
        measurement.ref_ns == 0 || measurement.cur_ns <= measurement.ref_ns ||
        !measurement.translation.allFinite() || measurement.inliers <= 0 ||
        !std::isfinite(measurement.ratio) ||
        !std::isfinite(measurement.reprojection))
      throw std::runtime_error("invalid_frozen_visual_pair");
    previous_current = measurement.transaction_cur;
    result.push_back(measurement);
  }
  if (result.size() != 1802) throw std::runtime_error("expected_1802_frozen_visual_pairs");
  return result;
}

Eigen::Matrix4d poseMatrix(const Pose3d& pose) {
  return p4_i2::asIsometry(pose).matrix();
}

Pose3d poseFromMatrix(const Eigen::Matrix4d& matrix) {
  Pose3d pose;
  pose.position = matrix.block<3, 1>(0, 3);
  pose.orientation = Eigen::Quaterniond(matrix.block<3, 3>(0, 0)).normalized();
  return pose;
}

Eigen::Matrix3d positionMarginalCovariance(const FilterSnapshot& snapshot) {
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  if (snapshot.covariance.rows() != state_ikfom::DOF ||
      snapshot.covariance.cols() != state_ikfom::DOF ||
      !snapshot.covariance.allFinite())
    return Eigen::Matrix3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  return snapshot.covariance.block<3, 3>(position_index, position_index);
}

Eigen::Matrix3d visualAnchorTargetCovariance(
    const FilterSnapshot& anchor,
    const Eigen::Vector3d& reference_frame_translation) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  if (anchor.covariance.rows() != state_ikfom::DOF ||
      anchor.covariance.cols() != state_ikfom::DOF ||
      !anchor.covariance.allFinite())
    return result;
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  Eigen::Matrix<double, 3, state_ikfom::DOF> anchor_jacobian =
      Eigen::Matrix<double, 3, state_ikfom::DOF>::Zero();
  anchor_jacobian.block<3, 3>(0, position_index).setIdentity();
  Eigen::Matrix3d skew;
  skew << 0.0, -reference_frame_translation.z(), reference_frame_translation.y(),
          reference_frame_translation.z(), 0.0, -reference_frame_translation.x(),
          -reference_frame_translation.y(), reference_frame_translation.x(), 0.0;
  // IKFoM SO(3) error is right/body: d(R z)/d(theta_body) = -R[z]x.
  anchor_jacobian.block<3, 3>(0, rotation_index) =
      -anchor.map_T_imu.orientation.toRotationMatrix() * skew;
  result = anchor_jacobian * anchor.covariance * anchor_jacobian.transpose();
  return 0.5 * (result + result.transpose());
}

Eigen::Matrix3d conservativeVisualInnovationNoise(
    const FilterSnapshot& anchor, const FilterSnapshot& current,
    const Eigen::Vector3d& reference_frame_translation, double visual_sigma_m) {
  Eigen::Matrix3d result = Eigen::Matrix3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  if (!std::isfinite(visual_sigma_m) || visual_sigma_m <= 0.0)
    return Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  const Eigen::Matrix3d anchor_target_covariance =
      visualAnchorTargetCovariance(anchor, reference_frame_translation);
  const Eigen::Matrix3d current_position_covariance =
      positionMarginalCovariance(current);
  if (!anchor_target_covariance.allFinite() ||
      !current_position_covariance.allFinite())
    return result;
  // The historical/current cross-covariance is not available in this minimal
  // frontend API. By Cauchy-Schwarz, Cov(x_cur-x_ref) <= 2(P_cur+P_ref).
  // Add current marginal to R (the update already contains one P_cur) and
  // twice the projected anchor marginal, yielding that conservative bound.
  result = visual_sigma_m * visual_sigma_m * Eigen::Matrix3d::Identity() +
      2.0 * anchor_target_covariance + current_position_covariance;
  result = 0.5 * (result + result.transpose());
  result.diagonal().array() += 1e-9;
  return result;
}

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
            vector.z(), 0.0, -vector.x(),
            -vector.y(), vector.x(), 0.0;
  return result;
}

reliability::LocalRisk riskWithNonlocalResponse(
    const reliability::LocalRisk& local,
    const reliability::NonlocalTerminalStability& nonlocal,
    bool nonlocal_risk, const Eigen::Vector3d& map_imu_origin_minus_lidar,
    const reliability::DualReliabilityConfig&) {
  reliability::LocalRisk result = local;
  if (!result.valid || !nonlocal_risk || !nonlocal.response_valid ||
      !map_imu_origin_minus_lidar.allFinite() ||
      !std::isfinite(result.translation_length_scale_m) ||
      result.translation_length_scale_m <= 0.0)
    return result;
  Eigen::MatrixXd columns(6, 6);
  columns.setZero();
  int count = 0;
  for (int i = 0; i < result.weak_dimension; ++i)
    columns.col(count++) = result.joint_weak_basis.col(i);
  const Eigen::Matrix3d lever_skew = skewMatrix(map_imu_origin_minus_lidar);
  const auto append_response = [&](const Eigen::Vector3d& delta_p_imu,
                                   const Eigen::Vector3d& delta_phi,
                                   Eigen::MatrixXd* vectors, int* size) {
    if (*size >= 6 || !delta_p_imu.allFinite() || !delta_phi.allFinite()) return;
    Eigen::Matrix<double, 6, 1> direction;
    direction.head<3>() = delta_phi;
    // Convert IMU-origin displacement back to the LiDAR-origin chart before
    // normalizing translation. This retains the lever-arm coupling.
    direction.tail<3>() = (delta_p_imu + lever_skew * delta_phi) /
                          result.translation_length_scale_m;
    for (int i = 0; i < *size; ++i)
      direction -= vectors->col(i).dot(direction) * vectors->col(i);
    const double norm = direction.norm();
    if (std::isfinite(norm) && norm > 1e-8)
      vectors->col((*size)++) = direction / norm;
  };
  append_response(nonlocal.delta_position_imu_positive,
                  nonlocal.delta_rotation_positive, &columns, &count);
  append_response(nonlocal.delta_position_imu_negative,
                  nonlocal.delta_rotation_negative, &columns, &count);
  result.joint_weak_basis.setZero();
  result.joint_reliable_basis.setZero();
  if (count == 0) return result;
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(columns.leftCols(count));
  const Eigen::Matrix<double, 6, 6> orthogonal =
      qr.householderQ() * Eigen::Matrix<double, 6, 6>::Identity();
  result.weak_dimension = count;
  result.reliable_dimension = 6 - count;
  result.joint_weak_basis.leftCols(count) = orthogonal.leftCols(count);
  if (result.reliable_dimension > 0)
    result.joint_reliable_basis.leftCols(result.reliable_dimension) =
        orthogonal.rightCols(result.reliable_dimension);
  result.translation_weak = true;
  result.status += ";UNONLOCAL_RESPONSE_DIRECTIONS_INCLUDED";
  return result;
}

bool makeReliablePoseMeasurementBasis(
    const reliability::LocalRisk& risk, const Pose3d& map_T_imu,
    const Pose3d& T_imu_lidar, Eigen::Matrix<double, 6, 6>* basis,
    int* rank, std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  if (!basis || !rank || !risk.valid || !map_T_imu.position.allFinite() ||
      !map_T_imu.orientation.coeffs().allFinite() ||
      !T_imu_lidar.position.allFinite() ||
      !T_imu_lidar.orientation.coeffs().allFinite() ||
      !std::isfinite(risk.translation_length_scale_m) ||
      risk.translation_length_scale_m <= 0.0) {
    if (failure_reason) *failure_reason = "INVALID_POSE_BASIS_INPUT";
    return false;
  }
  if (risk.weak_dimension < 0 || risk.weak_dimension > 6 ||
      risk.reliable_dimension != 6 - risk.weak_dimension) {
    if (failure_reason) *failure_reason = "INVALID_LIDAR_WEAK_RELIABLE_DIMENSIONS";
    return false;
  }
  const Eigen::Vector3d map_imu_origin_minus_lidar =
      -(map_T_imu.orientation * T_imu_lidar.position);
  Eigen::Matrix<double, 6, 6> pose_from_normalized_lidar =
      Eigen::Matrix<double, 6, 6>::Zero();
  pose_from_normalized_lidar.block<3, 3>(0, 0) =
      -skewMatrix(map_imu_origin_minus_lidar);
  pose_from_normalized_lidar.block<3, 3>(0, 3) =
      risk.translation_length_scale_m * Eigen::Matrix3d::Identity();
  pose_from_normalized_lidar.block<3, 3>(3, 0) =
      map_T_imu.orientation.toRotationMatrix().transpose();
  return reliability::buildReliableMeasurementBasisFromWeak(
      pose_from_normalized_lidar, risk.joint_weak_basis,
      risk.weak_dimension, basis, rank, failure_reason);
}

double maximumPositionSigma(const FilterSnapshot& snapshot) {
  if (!snapshot.covariance.allFinite())
    return std::numeric_limits<double>::infinity();
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const Eigen::Matrix3d p = 0.5 * (snapshot.covariance.block<3, 3>(
      position_index, position_index) + snapshot.covariance.block<3, 3>(
      position_index, position_index).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> p_solver(p);
  if (p_solver.info() != Eigen::Success || !p_solver.eigenvalues().allFinite())
    return std::numeric_limits<double>::infinity();
  return std::sqrt(std::max(0.0, p_solver.eigenvalues().maxCoeff()));
}

double maximumRotationSigma(const FilterSnapshot& snapshot) {
  if (!snapshot.covariance.allFinite())
    return std::numeric_limits<double>::infinity();
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  const Eigen::Matrix3d r = 0.5 * (snapshot.covariance.block<3, 3>(
      rotation_index, rotation_index) + snapshot.covariance.block<3, 3>(
      rotation_index, rotation_index).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> r_solver(r);
  if (r_solver.info() != Eigen::Success || !r_solver.eigenvalues().allFinite())
    return std::numeric_limits<double>::infinity();
  return std::sqrt(std::max(0.0, r_solver.eigenvalues().maxCoeff()));
}

void predictFrontendTo(FastLio2IkfomFrontend& frontend,
                       const p4_i2::ImuVector& all_imu,
                       uint64_t end_stamp_ns) {
  const FilterSnapshot start = frontend.getState();
  if (end_stamp_ns < start.stamp_ns)
    throw std::runtime_error("image_time_event_would_rewind_filter");
  if (end_stamp_ns == start.stamp_ns) return;
  const auto first_after_start = std::upper_bound(
      all_imu.begin(), all_imu.end(), start.stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  const auto first_after_end = std::upper_bound(
      all_imu.begin(), all_imu.end(), end_stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (first_after_start == all_imu.begin())
    throw std::runtime_error("image_time_prediction_has_no_causal_imu_history");
  std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> poses;
  std::string failure;
  const auto first_causal = first_after_start - 1;
  if (first_after_end - first_causal >= 2) {
    const p4_i2::ImuVector window(first_causal, first_after_end);
    if (!frontend.predictImuSequence(window, end_stamp_ns, &poses, &failure))
      throw std::runtime_error("causal_image_time_prediction_failed:" + failure);
  } else {
    // A camera event can fall between consecutive IMU samples. In that case
    // there is not yet a two-sample causal interval ending before the image;
    // extrapolate only the last already-observed IMU input to the image time.
    const auto causal_history_end = std::upper_bound(
        all_imu.begin(), all_imu.end(), start.stamp_ns,
        [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
    if (causal_history_end - all_imu.begin() < 2)
      throw std::runtime_error("image_time_prediction_lacks_two_past_imu_samples");
    const ImuSample& head = *(causal_history_end - 2);
    const ImuSample& tail = *(causal_history_end - 1);
    if (!frontend.predictHeldInputTo(end_stamp_ns, head, tail, &failure))
      throw std::runtime_error("causal_image_time_held_imu_prediction_failed:" + failure);
  }
  if (frontend.getState().stamp_ns != end_stamp_ns)
    throw std::runtime_error("image_time_prediction_not_exact");
}

Eigen::Matrix4d makeBodyDelta(const Eigen::Vector3d& translation,
                              double yaw_rad) {
  Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
  delta.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  delta.block<3, 1>(0, 3) = translation;
  return delta;
}

std::vector<std::pair<int, Eigen::Matrix4d>> makeSeeds(
    const Eigen::Matrix4d& predicted_map_T_lidar,
    const VisualMeasurement* visual,
    const Eigen::Matrix4d& previous_map_T_imu,
    const Eigen::Matrix4d& T_imu_lidar) {
  constexpr double kResolution = 0.8;
  constexpr double kYaw = 5.0 * M_PI / 180.0;
  const Eigen::Vector3d ex = Eigen::Vector3d::UnitX() * kResolution;
  const Eigen::Vector3d ey = Eigen::Vector3d::UnitY() * kResolution;
  std::vector<std::pair<int, Eigen::Matrix4d>> seeds = {
      {0, predicted_map_T_lidar},
      {1, predicted_map_T_lidar * makeBodyDelta(ex, 0.0)},
      {2, predicted_map_T_lidar * makeBodyDelta(-ex, 0.0)},
      {3, predicted_map_T_lidar * makeBodyDelta(ey, 0.0)},
      {4, predicted_map_T_lidar * makeBodyDelta(-ey, 0.0)},
      {5, predicted_map_T_lidar * makeBodyDelta(Eigen::Vector3d::Zero(), kYaw)},
      {6, predicted_map_T_lidar * makeBodyDelta(Eigen::Vector3d::Zero(), -kYaw)},
  };
  if (visual) {
    const Eigen::Vector3d p_visual = previous_map_T_imu.block<3, 1>(0, 3) +
        previous_map_T_imu.block<3, 3>(0, 0) * visual->translation;
    Eigen::Matrix4d visual_map_T_imu = Eigen::Matrix4d::Identity();
    visual_map_T_imu.block<3, 3>(0, 0) = predicted_map_T_lidar.block<3, 3>(0, 0) *
        T_imu_lidar.block<3, 3>(0, 0).transpose();
    visual_map_T_imu.block<3, 1>(0, 3) = p_visual;
    seeds.emplace_back(7, visual_map_T_imu * T_imu_lidar);
  }
  return seeds;
}

void checkVisualSeedConvention(const Eigen::Matrix4d& T_imu_lidar) {
  Eigen::Matrix4d previous = Eigen::Matrix4d::Identity();
  previous.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      0.31, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()).toRotationMatrix();
  previous.block<3, 1>(0, 3) = Eigen::Vector3d(3.0, -1.0, 2.0);
  const Eigen::Vector3d measured(0.2, -0.1, 0.07);
  Eigen::Matrix4d predicted = Eigen::Matrix4d::Identity();
  predicted.block<3, 3>(0, 0) = Eigen::AngleAxisd(
      -0.17, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  predicted.block<3, 1>(0, 3) = Eigen::Vector3d(9.0, 4.0, -2.0);
  VisualMeasurement visual;
  visual.translation = measured;
  const auto seeds = makeSeeds(predicted, &visual, previous, T_imu_lidar);
  if (seeds.size() != 8 || seeds.back().first != 7)
    throw std::runtime_error("visual_seed_count_contract_failed");
  const Eigen::Matrix4d recovered_map_T_imu = seeds.back().second * T_imu_lidar.inverse();
  const Eigen::Vector3d expected_position = previous.block<3, 1>(0, 3) +
      previous.block<3, 3>(0, 0) * measured;
  const Eigen::Matrix3d expected_rotation =
      predicted.block<3, 3>(0, 0) * T_imu_lidar.block<3, 3>(0, 0).transpose();
  if ((recovered_map_T_imu.block<3, 1>(0, 3) - expected_position).norm() > 1e-12 ||
      (recovered_map_T_imu.block<3, 3>(0, 0) - expected_rotation).norm() > 1e-12)
    throw std::runtime_error("visual_seed_frame_conversion_contract_failed");
  const Eigen::Matrix4d no_visual = makeSeeds(predicted, nullptr, previous,
                                               T_imu_lidar).front().second;
  if ((no_visual - predicted).norm() > 1e-12)
    throw std::runtime_error("S0_predicted_seed_contract_failed");
  std::cout << "P6_VISUAL_SEED_CONVENTION_PASS\n";
}

Candidate runNdtCandidate(AuditedNdt& ndt, const Cloud::Ptr& source,
                          const Eigen::Matrix4d& seed, int seed_index,
                          const std::string& seed_name) {
  Candidate candidate;
  candidate.seed_index = seed_index;
  candidate.seed_name = seed_name;
  ndt.setInputSource(source);
  Cloud aligned;
  const auto started = std::chrono::steady_clock::now();
  ndt.align(aligned, seed.cast<float>());
  candidate.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  candidate.converged = ndt.hasConverged();
  candidate.iterations = ndt.getFinalNumIteration();
  candidate.fitness = ndt.getFitnessScore();
  candidate.transformation_probability = ndt.getTransformationProbability();
  const Eigen::Matrix4f pose = ndt.getFinalTransformation();
  candidate.pose = pose.cast<double>();
  candidate.objective = fixedPclScore(ndt, source, pose);
  if (source->empty()) throw std::runtime_error("ndt_candidate_source_cloud_empty");
  candidate.objective_per_source_point = candidate.objective /
      static_cast<double>(source->size());
  if (candidate.converged) {
    if (!candidate.pose.allFinite() || !std::isfinite(candidate.objective))
      throw std::runtime_error("nonfinite_ndt_candidate");
  }
  return candidate;
}

Eigen::Matrix<double, 6, 6> projectMapProductPoseCovariance(
    const FilterSnapshot& snapshot) {
  if (snapshot.covariance.rows() != state_ikfom::DOF ||
      snapshot.covariance.cols() != state_ikfom::DOF ||
      !snapshot.covariance.allFinite())
    throw std::runtime_error("dual_reliability_prediction_covariance_invalid");
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  const Eigen::MatrixXd jacobian = p6_i4::buildPoseErrorJacobian(
      snapshot.map_T_imu.orientation.toRotationMatrix(), state_ikfom::DOF,
      position_index, rotation_index);
  Eigen::Matrix<double, 6, 6> covariance =
      jacobian * snapshot.covariance * jacobian.transpose();
  return 0.5 * (covariance + covariance.transpose());
}

reliability::TerminalCapture terminalCapture(const Candidate& candidate) {
  reliability::TerminalCapture terminal;
  terminal.map_T_lidar = Eigen::Isometry3d::Identity();
  terminal.map_T_lidar.matrix() = candidate.pose;
  terminal.fixed_objective = candidate.objective;
  terminal.converged = candidate.converged;
  terminal.iterations = candidate.iterations;
  return terminal;
}

Eigen::Matrix4f poseFromPclParameters(const Eigen::Matrix<double, 6, 1>& p) {
  const Eigen::AngleAxisf rx(static_cast<float>(p(3)), Eigen::Vector3f::UnitX());
  const Eigen::AngleAxisf ry(static_cast<float>(p(4)), Eigen::Vector3f::UnitY());
  const Eigen::AngleAxisf rz(static_cast<float>(p(5)), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  // PCL 1.10 NDT::computeTransformation() uses this exact composition; do not
  // substitute pcl::getTransformation(), whose generic Euler helper differs.
  pose.block<3, 3>(0, 0) = (rx * ry * rz).toRotationMatrix();
  pose.block<3, 1>(0, 3) = p.head<3>().cast<float>();
  return pose;
}

Eigen::Vector3d normalNdtEulerXyz(const Eigen::Matrix3f& rotation) {
  const Eigen::Matrix3d r = rotation.cast<double>();
  const double pitch = std::asin(std::clamp(r(0, 2), -1.0, 1.0));
  return Eigen::Vector3d(std::atan2(-r(1, 2), r(2, 2)), pitch,
                         std::atan2(-r(0, 1), r(0, 0)));
}

Eigen::Matrix<double, 6, 1> pclNdtParameters(const Eigen::Matrix4f& pose) {
  Eigen::Matrix<double, 6, 1> parameters;
  parameters.head<3>() = pose.block<3, 1>(0, 3).cast<double>();
  parameters.tail<3>() = normalNdtEulerXyz(pose.block<3, 3>(0, 0));
  return parameters;
}

double scoreAtPclParameters(AuditedNdt& ndt, const Cloud::Ptr& source,
                            const Eigen::Matrix<double, 6, 1>& p) {
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, poseFromPclParameters(p));
  Eigen::Matrix<double, 6, 1> mutable_p = p;
  return ndt.scoreDerivatives(transformed, mutable_p, nullptr, nullptr);
}

bool verifyScoreGradientConvention(
    AuditedNdt& ndt, const Cloud::Ptr& source, const Candidate& candidate,
    std::size_t sample_index, const std::string& mode,
    std::ofstream& output) {
  constexpr double kStep = 1e-4;
  constexpr double kHalfStep = 5e-5;
  constexpr double kQuarterStep = 2.5e-5;
  constexpr double kRelativeTolerance = 0.10;
  constexpr double kMeaningful = 1e-6;
  const char* axis_names[] = {"tx", "ty", "tz", "rx", "ry", "rz"};
  const Eigen::Matrix4f pose = candidate.pose.cast<float>();
  // Extract the normal branch for PCL NDT's internal Rx*Ry*Rz parameterization.
  const Eigen::Matrix<double, 6, 1> p = pclNdtParameters(pose);
  const Eigen::Vector3d euler = p.tail<3>();
  const bool normal_coordinates = euler.allFinite() &&
      euler.cwiseAbs().maxCoeff() < 1.0 && std::abs(std::cos(euler.y())) > 0.5;
  Eigen::Matrix<double, 6, 1> analytic =
      Eigen::Matrix<double, 6, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 6, 6> hessian =
      Eigen::Matrix<double, 6, 6>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Eigen::Matrix<double, 6, 1> mutable_p = p;
  const double center_score = ndt.scoreDerivatives(
      transformed, mutable_p, &analytic, &hessian);
  const bool common_pass = candidate.converged && normal_coordinates &&
      std::isfinite(center_score) && analytic.allFinite() && hessian.allFinite();
  bool sample_pass = common_pass;
  for (int axis = 0; axis < 6; ++axis) {
    const auto finiteDifference = [&](double step) {
      Eigen::Matrix<double, 6, 1> plus = p, minus = p;
      plus(axis) += step;
      minus(axis) -= step;
      return (scoreAtPclParameters(ndt, source, plus) -
              scoreAtPclParameters(ndt, source, minus)) / (2.0 * step);
    };
    const double fd = finiteDifference(kStep);
    const double fd_half = finiteDifference(kHalfStep);
    const double fd_quarter = finiteDifference(kQuarterStep);
    const double denominator = std::max({std::abs(analytic(axis)), std::abs(fd), 1e-8});
    const double half_denominator = std::max(
        {std::abs(analytic(axis)), std::abs(fd_half), 1e-8});
    const double relative_error = std::abs(analytic(axis) - fd) / denominator;
    const double half_relative_error =
        std::abs(analytic(axis) - fd_half) / half_denominator;
    const double quarter_denominator = std::max(
        {std::abs(analytic(axis)), std::abs(fd_quarter), 1e-8});
    const double quarter_relative_error =
        std::abs(analytic(axis) - fd_quarter) / quarter_denominator;
    const bool meaningful = std::isfinite(analytic(axis)) &&
        std::isfinite(fd) && std::isfinite(fd_half) && std::isfinite(fd_quarter) &&
        std::abs(analytic(axis)) > kMeaningful &&
        std::abs(fd) > kMeaningful && std::abs(fd_half) > kMeaningful &&
        std::abs(fd_quarter) > kMeaningful;
    const bool signs_match = meaningful && analytic(axis) * fd > 0.0 &&
        analytic(axis) * fd_half > 0.0 && analytic(axis) * fd_quarter > 0.0;
    const bool axis_pass = common_pass && meaningful && signs_match &&
        relative_error <= kRelativeTolerance &&
        half_relative_error <= kRelativeTolerance &&
        quarter_relative_error <= kRelativeTolerance;
    sample_pass = sample_pass && axis_pass;
    output << mode << ',' << sample_index << ',' << candidate.seed_name << ','
           << (candidate.converged ? 1 : 0) << ','
           << (normal_coordinates ? 1 : 0) << ',' << axis_names[axis] << ','
           << axis << ',' << p(axis) << ',' << center_score << ','
           << analytic(axis) << ',' << fd << ',' << fd_half << ',' << fd_quarter << ','
           << relative_error << ',' << half_relative_error << ',' << quarter_relative_error << ','
           << (signs_match ? 1 : 0) << ',' << (axis_pass ? 1 : 0) << ",0\n";
  }
  return common_pass && sample_pass;
}

std::string matrixField(const Eigen::MatrixXd& matrix) {
  std::ostringstream stream;
  stream << std::setprecision(10);
  for (Eigen::Index row = 0; row < matrix.rows(); ++row)
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (row != 0 || column != 0) stream << ';';
      stream << matrix(row, column);
    }
  return stream.str();
}

std::string poseField(const Eigen::Matrix4d& pose) {
  return matrixField(pose);
}

std::string vectorField(const Eigen::Vector3d& vector) {
  std::ostringstream stream;
  stream << std::setprecision(10) << vector.x() << ';' << vector.y() << ';'
         << vector.z();
  return stream.str();
}

std::string stateSnapshotHash(const FilterSnapshot& snapshot) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto consume = [&](const void* data, std::size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
      hash ^= bytes[index];
      hash *= 1099511628211ULL;
    }
  };
  consume(&snapshot.stamp_ns, sizeof(snapshot.stamp_ns));
  consume(snapshot.map_T_imu.position.data(), 3 * sizeof(double));
  consume(snapshot.map_T_imu.orientation.coeffs().data(), 4 * sizeof(double));
  consume(snapshot.velocity.data(), 3 * sizeof(double));
  consume(snapshot.gyro_bias.data(), 3 * sizeof(double));
  consume(snapshot.accel_bias.data(), 3 * sizeof(double));
  consume(snapshot.gravity.data(), 3 * sizeof(double));
  consume(snapshot.T_imu_lidar_rotation.data(), 9 * sizeof(double));
  consume(snapshot.T_imu_lidar_translation.data(), 3 * sizeof(double));
  for (Eigen::Index index = 0; index < snapshot.covariance.size(); ++index)
    consume(snapshot.covariance.data() + index, sizeof(double));
  std::ostringstream text;
  text << std::hex << std::setw(16) << std::setfill('0') << hash;
  return text.str();
}

bool sameSnapshotExactly(const FilterSnapshot& left,
                         const FilterSnapshot& right) {
  return left.stamp_ns == right.stamp_ns &&
      (left.map_T_imu.position.array() == right.map_T_imu.position.array()).all() &&
      (left.map_T_imu.orientation.coeffs().array() ==
       right.map_T_imu.orientation.coeffs().array()).all() &&
      (left.velocity.array() == right.velocity.array()).all() &&
      (left.gyro_bias.array() == right.gyro_bias.array()).all() &&
      (left.accel_bias.array() == right.accel_bias.array()).all() &&
      (left.gravity.array() == right.gravity.array()).all() &&
      (left.T_imu_lidar_rotation.array() == right.T_imu_lidar_rotation.array()).all() &&
      (left.T_imu_lidar_translation.array() ==
       right.T_imu_lidar_translation.array()).all() &&
      left.covariance.rows() == right.covariance.rows() &&
      left.covariance.cols() == right.covariance.cols() &&
      (left.covariance.array() == right.covariance.array()).all();
}

Eigen::Matrix<double, 6, 1> poseResidual(
    const Pose3d& prior, const Pose3d& measurement) {
  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = measurement.position - prior.position;
  Eigen::Quaterniond error =
      (prior.orientation.conjugate() * measurement.orientation.normalized()).normalized();
  if (error.w() < 0.0) error.coeffs() *= -1.0;
  const Eigen::AngleAxisd angle_axis(error);
  residual.tail<3>() = angle_axis.axis() * angle_axis.angle();
  return residual;
}

Eigen::Matrix<double, 6, 6> normalizedLidarToPoseJacobian(
    const Pose3d& map_T_imu, const Pose3d& T_imu_lidar,
    double length_scale_m) {
  const Eigen::Vector3d map_imu_origin_minus_lidar =
      -(map_T_imu.orientation * T_imu_lidar.position);
  Eigen::Matrix<double, 6, 6> jacobian =
      Eigen::Matrix<double, 6, 6>::Zero();
  jacobian.block<3, 3>(0, 0) = -skewMatrix(map_imu_origin_minus_lidar);
  jacobian.block<3, 3>(0, 3) =
      length_scale_m * Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(3, 0) =
      map_T_imu.orientation.toRotationMatrix().transpose();
  return jacobian;
}

#include "p6_i6e_r2_diagnostics.hpp"

void runDualReliabilityMode(
    const std::string& mode, const p4_i2::Inputs& inputs,
    const std::vector<ScanAsset>& assets, const std::string& cloud_binary_path,
    const std::string& map_path, const std::string& params_path,
    const std::string& trajectory_path, const std::string& reliability_path,
    const std::string& runtime_path,
    uint64_t initialization_stamp_ns = 0,
    const std::string& map_profile = "floor01",
    const std::vector<VisualMeasurement>* visual_measurements = nullptr,
    double visual_sigma_m = 0.05,
    R2Policy r2_policy = R2Policy::LEGACY_BASE_NO_GATE) {
  if (mode != "STRICT_BASELINE" && mode != "UOBS_ONLY" &&
      mode != "UNONLOCAL_ONLY" && mode != "DUAL_RELIABILITY" &&
      mode != "FULL_ALGORITHM_V1")
    throw std::runtime_error("unsupported_dual_reliability_mode:" + mode);
  const bool enable_vision = mode == "FULL_ALGORITHM_V1";
  if (!enable_vision && r2_policy != R2Policy::LEGACY_BASE_NO_GATE)
    throw std::runtime_error("nonlegacy_r2_policy_only_valid_for_full_algorithm");
  if (enable_vision && (!visual_measurements || visual_measurements->empty() ||
      !std::isfinite(visual_sigma_m) || visual_sigma_m <= 0.0))
    throw std::runtime_error("FULL_ALGORITHM_V1_requires_valid_metric_visual_input");
  if (map_profile == "floor01") {
    p5_i1::requireFrozenMapSha256(map_path);
  } else if (map_profile == "corridor01") {
    if (p5_i1::sha256File(map_path) !=
        "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f")
      throw std::runtime_error("corridor01_map_sha256_mismatch");
  } else {
    throw std::runtime_error("unsupported_map_profile:" + map_profile);
  }
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->empty()) throw std::runtime_error("empty_preprocessed_target_map");
  if (map_profile == "floor01" && target->size() != 549606)
    throw std::runtime_error("frozen_target_point_count_mismatch");

  Pose3d initial_map_T_lidar, T_imu_lidar_pose;
  const RuntimeParameters parameters = p4_i2::readParameters(
      params_path, &initial_map_T_lidar, &T_imu_lidar_pose);
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend frontend(parameters);
  p4_i2::ImuVector initialization_imu;
  if (initialization_stamp_ns == 0) {
    initialization_imu.assign(inputs.imu.begin(),
        inputs.imu.begin() + parameters.static_init_samples);
  } else {
    const auto exact_or_after = std::lower_bound(
        inputs.imu.begin(), inputs.imu.end(), initialization_stamp_ns,
        [](const ImuSample& sample, uint64_t stamp) { return sample.stamp_ns < stamp; });
    if (exact_or_after == inputs.imu.begin())
      throw std::runtime_error("initialization_stamp_outside_imu_range");
    auto causal_end = exact_or_after;
    const bool exact_imu_stamp = exact_or_after != inputs.imu.end() &&
        exact_or_after->stamp_ns == initialization_stamp_ns;
    if (exact_imu_stamp) ++causal_end;
    const auto count = static_cast<std::ptrdiff_t>(parameters.static_init_samples);
    if (std::distance(inputs.imu.begin(), causal_end) < count)
      throw std::runtime_error("insufficient_causal_imu_samples_before_initialization_stamp");
    initialization_imu.assign(causal_end - count, causal_end);
    if (initialization_imu.back().stamp_ns > initialization_stamp_ns)
      throw std::runtime_error("static_initialization_uses_future_imu_sample");
    if (initialization_imu.back().stamp_ns < initialization_stamp_ns) {
      std::vector<uint64_t> recent_periods;
      const std::size_t first_period = initialization_imu.size() > 101
          ? initialization_imu.size() - 101 : 1;
      for (std::size_t i = first_period; i < initialization_imu.size(); ++i) {
        if (initialization_imu[i].stamp_ns <= initialization_imu[i - 1].stamp_ns)
          throw std::runtime_error("nonmonotonic_causal_static_imu_window");
        recent_periods.push_back(initialization_imu[i].stamp_ns -
                                 initialization_imu[i - 1].stamp_ns);
      }
      if (recent_periods.empty())
        throw std::runtime_error("insufficient_imu_periods_for_initialization_epoch");
      std::sort(recent_periods.begin(), recent_periods.end());
      const uint64_t median_period_ns = recent_periods[recent_periods.size() / 2];
      const uint64_t held_gap_ns = initialization_stamp_ns - initialization_imu.back().stamp_ns;
      if (median_period_ns == 0 || held_gap_ns > 2 * median_period_ns)
        throw std::runtime_error("initialization_epoch_extrapolation_exceeds_two_median_imu_periods");
    }
  }
  std::string failure;
  if (!frontend.initializeStatic(initialization_imu, initial_map_T_lidar,
                                 T_imu_lidar_pose, &failure))
    throw std::runtime_error("dual_reliability_static_initialization_failed:" + failure);
  if (initialization_stamp_ns != 0 &&
      frontend.getState().stamp_ns < initialization_stamp_ns) {
    const ImuSample& causal_head = initialization_imu[initialization_imu.size() - 2];
    const ImuSample& causal_tail = initialization_imu.back();
    if (!frontend.predictHeldInputTo(initialization_stamp_ns, causal_head,
                                     causal_tail, &failure))
      throw std::runtime_error("causal_initialization_epoch_propagation_failed:" + failure);
  }

  constexpr double kStrictEpsilon = 1e-5;
  constexpr int kStrictMaximumIterations = 80;
  const reliability::DualReliabilityConfig reliability_config;
  const bool use_uobs = mode == "UOBS_ONLY" || mode == "DUAL_RELIABILITY" ||
                        mode == "FULL_ALGORITHM_V1";
  const bool use_unonlocal = mode == "UNONLOCAL_ONLY" ||
      mode == "DUAL_RELIABILITY" || mode == "FULL_ALGORITHM_V1";

  GeometricNdt ndt;
  configureNdt(ndt, target);
  ndt.setResolution(0.8);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(kStrictEpsilon);
  ndt.setMaximumIterations(kStrictMaximumIterations);

  std::ofstream trajectory(trajectory_path), events(reliability_path), runtime(runtime_path);
  std::ofstream visual_updates;
  if (enable_vision) visual_updates.open(reliability_path + ".visual_updates.csv");
  std::ofstream projected_innovation, shadow_counterfactual;
  std::ofstream nonlocal_basis_comparison, linearization_consistency;
  if (enable_vision) {
    projected_innovation.open(reliability_path + ".r2_projected_innovation.csv");
    shadow_counterfactual.open(reliability_path + ".r2_shadow_counterfactual.csv");
    nonlocal_basis_comparison.open(reliability_path + ".r2_nonlocal_basis_comparison.csv");
    linearization_consistency.open(reliability_path + ".r2_linearization_consistency.csv");
  }
  std::ofstream map_support_diagnostic;
  if (enable_vision)
    map_support_diagnostic.open(reliability_path + ".map_support_diagnostic.csv");
  std::ofstream pcl_hessian_status;
  if (use_uobs) pcl_hessian_status.open(reliability_path + ".pcl_score_hessian.txt");
  if (!trajectory || !events || !runtime)
    throw std::runtime_error("cannot_create_dual_reliability_outputs");
  if (enable_vision && !visual_updates)
    throw std::runtime_error("cannot_create_i6d_visual_update_output");
  if (enable_vision && (!projected_innovation || !shadow_counterfactual ||
      !nonlocal_basis_comparison || !linearization_consistency))
    throw std::runtime_error("cannot_create_i6e_r2_diagnostic_outputs");
  if (enable_vision && !map_support_diagnostic)
    throw std::runtime_error("cannot_create_map_support_diagnostic_output");
  if (use_uobs && !pcl_hessian_status)
    throw std::runtime_error("cannot_create_legacy_pcl_status_output");
  if (use_uobs)
    pcl_hessian_status << "PCL_SCORE_HESSIAN=INDETERMINATE\n"
        << "reason=geometric_voxel_information_proxy_is_independent_of_legacy_score_Hessian_gate\n";
  trajectory << std::setprecision(17);
  p4_i2::writeHeader(trajectory);
  events << std::setprecision(17)
      << "mode,transaction_id,stamp_ns,time_s,source_cloud_hash_verified,uobs_valid,uobs_status"
         ",legacy_pcl_score_gradient_gate_passed,legacy_pcl_gate_samples_passed"
         ",uobs_effective_weight_sum,uobs_valid_correspondences,uobs_rejected_voxel_covariances"
         ",uobs_normalized_information_rowmajor,uobs_map_support_sufficient"
         ",uobs_map_support_status,uobs_rotation_schur_eigenvalues"
         ",uobs_translation_schur_eigenvalues,uobs_joint_weak_basis_rowmajor"
         ",uobs_joint_reliable_basis_rowmajor"
         ",visual_valid_factor_count,visual_updates_total,visual_nonzero_updates"
         ",visual_last_update_status,visual_trigger_reason"
         ",translation_weak,translation_weak_ratio,translation_min_eigenvalue"
         ",translation_block_condition,translation_weak_direction_map_xyz"
         ",rotation_weak,rotation_weak_ratio,rotation_min_eigenvalue"
         ",rotation_block_condition,rotation_weak_direction_map_xyz"
         ",innovation_chi2,innovation_status,probe_trigger_reason,probe_triggered"
         ",probe_executed,probe_status,ndt_call_count"
         ",M0_converged,M0_iterations,M0_objective"
         ",M0_pose_map_T_lidar_rowmajor"
         ",Mplus_converged,Mplus_iterations,Mplus_objective"
         ",Mplus_pose_map_T_lidar_rowmajor"
         ",Mminus_converged,Mminus_iterations,Mminus_objective"
         ",Mminus_pose_map_T_lidar_rowmajor"
         ",delta_p_plus_map_T_imu_xyz_m,delta_p_minus_map_T_imu_xyz_m,delta_phi_plus_map_xyz_rad"
         ",delta_phi_minus_map_xyz_rad,Bp_map_T_imu_rowmajor_m2,BR_map_spatial_rowmajor_rad2"
         ",Delta_t_map_T_lidar_m,Delta_R_map_T_lidar_rad,plus_minus_translation_gap_map_T_lidar_m"
         ",plus_minus_rotation_gap_rad,plus_minus_objective_delta"
         ",R_position_rowmajor,R_rotation_map_rowmajor,R_rotation_body_rowmajor"
         ",step_limited,decision,decision_reason,local_curvature_used"
         ",nonlocal_response_used,vision_assist_status,vision_observation_available"
         ",vision_assist_trigger_requested,vision_assist_trigger_reason"
         ",fusion_mode,fusion_reason,lidar_reliable_dimension,lidar_weak_dimension"
         ",visual_trigger_count,visual_observations_available,visual_quality_pass_count"
         ",visual_weak_information_count,visual_applied_count,visual_rejected_count"
         ",visual_rejection_reason,imu_coasting_s,relocalization_required,recovery_stamp_ns"
         ",local_curvature_ms,gradient_gate_ms,extra_probe_ms"
         ",maximum_position_sigma_m,maximum_rotation_sigma_rad"
         ",lidar_update_applied,lidar_update_projected"
         ",route_provisional,lidar_update_attempted,lidar_update_committed"
         ",lidar_nis_rejected,visual_update_in_scan_interval"
         ",external_update_in_scan_interval,last_actual_external_update_ns"
         ",time_since_last_actual_external_update_s,route_actual\n";
  runtime << std::setprecision(17)
      << "mode,transaction_id,prediction_ms,nominal_ndt_ms,local_curvature_ms"
         ",gradient_gate_ms,extra_probe_ms,ikfom_update_ms,total_ms"
         ",ndt_call_count,probe_trigger,decision,vision_assist_status"
         ",vision_assist_trigger_requested,vision_assist_trigger_reason\n";

  if (enable_vision) {
    visual_updates << std::setprecision(17)
        << "transaction_ref,transaction_cur,ref_ns,cur_ns,depth_stamp_ns,status"
        << ",triggered,observation_available,quality_passed,quality_rejection_reason"
        << ",detected,tracked,depth_associated,depth_fraction,grid_occupancy,hull_fraction"
        << ",median_parallax_px,pnp_inliers,inlier_ratio,reprojection_rmse_px"
        << ",innovation_chi_square,visual_information_rowmajor"
        << ",projected_weak_information_rowmajor,projected_eigenvalues"
        << ",visual_effective_rank,visual_measurement_rank,visual_condition"
        << ",visual_subspace_status"
        << ",trigger_reason,measurement_map_xyz,innovation_map_xyz,innovation_norm_m"
        << ",position_correction_norm_m,velocity_correction_norm_m"
        << ",anchor_projected_covariance_rowmajor,current_position_covariance_rowmajor"
        << ",effective_noise_rowmajor,update_ms\n";
    projected_innovation << std::setprecision(17)
        << "transaction_id,time_s,r2_policy,route_provisional,route_actual"
        << ",projected_route,reliable_rank,weak_rank,raw_residual_norm"
        << ",selected_residual_norm,base_R_selected_trace,adaptive_R_selected_trace"
        << ",adaptive_R_minus_base_frobenius,selected_R_trace,selected_R_min_eigenvalue"
        << ",selected_R_max_eigenvalue,selected_S_trace,selected_nis,nis_threshold"
        << ",nis_gate_enabled,update_attempted,update_committed,nis_rejected"
        << ",update_status,actual_delta_position_norm,actual_delta_rotation_norm"
        << ",actual_delta_velocity_norm,prior_position_sigma_max,posterior_position_sigma_max\n";
    shadow_counterfactual << std::setprecision(17)
        << "transaction_id,time_s,policy,prior_state_hash,prior_position_xyz"
        << ",prior_velocity_xyz,reliable_rank,weak_rank,raw_pose_residual_norm"
        << ",selected_residual_norm,selected_noise_trace,selected_noise_min_eigenvalue"
        << ",selected_noise_max_eigenvalue,selected_innovation_trace,selected_nis"
        << ",selected_nis_threshold,nis_gate_passed,update_attempted,update_committed"
        << ",update_failure,delta_position_norm,delta_rotation_norm,delta_velocity_norm"
        << ",delta_gyro_bias_norm,delta_accel_bias_norm,posterior_position_sigma_max"
        << ",official_state_hash_after,official_state_unchanged\n";
    nonlocal_basis_comparison << std::setprecision(17)
        << "transaction_id,time_s,r2_policy,local_status,combined_status"
        << ",local_weak_dimension,combined_weak_dimension,local_reliable_dimension"
        << ",combined_reliable_dimension,additional_removed_dimension"
        << ",local_selected_rank,combined_selected_rank,local_nis,local_threshold"
        << ",combined_nis,combined_threshold,local_raw_residual_norm"
        << ",combined_raw_residual_norm,local_selected_residual_norm"
        << ",combined_selected_residual_norm,local_projected_R_trace"
        << ",combined_projected_R_trace,local_projected_R_min_eigenvalue"
        << ",local_projected_R_max_eigenvalue,combined_projected_R_min_eigenvalue"
        << ",combined_projected_R_max_eigenvalue,local_projected_R_rowmajor"
        << ",combined_projected_R_rowmajor\n";
    linearization_consistency << std::setprecision(17)
        << "transaction_id,time_s,r2_policy,weak_index,weak_direction_normalized"
        << ",nominal_leakage,predicted_leakage,rotation_difference_rad"
        << ",translation_difference_m,fd_selected_weak_response_norm"
        << ",fd_selected_weak_response,status\n";
    map_support_diagnostic << std::setprecision(17)
        << "transaction_id,time_s,source_points,sampled_points,predicted_sampled_points"
        << ",M0_converged,M0_fitness,M0_objective,ndt_converged_without_geometric_support"
        << ",predicted_pose_xyz,nominal_pose_xyz,predicted_to_nominal_translation_m"
        << ",predicted_to_nominal_rotation_rad,uobs_valid_correspondences"
        << ",uobs_rejected_voxel_covariances,nominal_queries_with_neighbors_r"
        << ",nominal_queries_with_neighbors_2r,nominal_total_neighbors_r"
        << ",nominal_total_neighbors_2r,predicted_queries_with_neighbors_r"
        << ",predicted_queries_with_neighbors_2r,predicted_total_neighbors_r"
        << ",predicted_total_neighbors_2r,maximum_position_sigma_m"
        << ",maximum_rotation_sigma_rad,lidar_reliable_dimension,lidar_weak_dimension"
        << ",nominal_search_valid,predicted_search_valid\n";
  }

  Eigen::Matrix4d previous_used = Eigen::Matrix4d::Identity();
  bool has_previous_used = false;
  constexpr bool score_gradient_gate_passed = false;
  constexpr int score_gradient_gate_samples = 0;
  std::vector<VisualMeasurement> active_visual_measurements;
  if (enable_vision) {
    const uint64_t init_stamp = frontend.getState().stamp_ns;
    const uint64_t final_stamp = assets.back().stamp_ns;
    for (const VisualMeasurement& measurement : *visual_measurements) {
      // Factors outside a partial smoke/replay window are not replayable here:
      // their reference image state is either before initialization or beyond
      // the selected final scan. They are excluded, never time-shifted.
      if (measurement.ref_ns < init_stamp || measurement.cur_ns > final_stamp)
        continue;
      if (measurement.depth_stamp_ns > measurement.ref_ns ||
          measurement.cur_ns <= measurement.ref_ns)
        throw std::runtime_error("noncausal_visual_factor_in_replay_window");
      active_visual_measurements.push_back(measurement);
    }
    if (active_visual_measurements.empty())
      throw std::runtime_error("no_visual_factors_inside_replay_window");
  }
  reliability::VisionAssistInterface vision_assist;
  vision_assist.enabled = enable_vision;
  vision_assist.fusion_enabled = enable_vision;
  vision_assist.observation_available = enable_vision &&
      !active_visual_measurements.empty();
  vision_assist.status = enable_vision ? "I6E_DIRECTIONAL_CONDITIONAL_VISION" :
                                         "DISABLED_BY_MODE";
  std::vector<ImageTimeEvent> image_events;
  if (enable_vision) {
    for (std::size_t i = 0; i < active_visual_measurements.size(); ++i) {
      const VisualMeasurement& measurement = active_visual_measurements[i];
      image_events.push_back({measurement.ref_ns, i, false});
      image_events.push_back({measurement.cur_ns, i, true});
    }
    std::stable_sort(image_events.begin(), image_events.end(),
        [](const ImageTimeEvent& left, const ImageTimeEvent& right) {
          if (left.stamp_ns != right.stamp_ns) return left.stamp_ns < right.stamp_ns;
          // At one image timestamp, first consume a just-completed factor,
          // then snapshot that corrected state as the next reference anchor.
          return left.is_current && !right.is_current;
        });
    vision_assist.latest_stamp_ns = image_events.empty() ? 0 :
        image_events.back().stamp_ns;
    visual_updates.flush();
  }
  std::map<uint64_t, VisualAnchorSnapshot> visual_anchors;
  std::size_t image_event_index = 0;
  std::uint64_t visual_updates_total = 0;
  std::uint64_t visual_nonzero_updates = 0;
  std::uint64_t visual_trigger_count = 0;
  std::uint64_t visual_observations_available = 0;
  std::uint64_t visual_quality_pass_count = 0;
  std::uint64_t visual_weak_information_count = 0;
  std::uint64_t visual_rejected_total = 0;
  std::map<std::string, std::uint64_t> visual_rejection_counts;
  std::string visual_last_update_status = enable_vision ? "NO_TRIGGERED_UPDATE" :
                                                         "DISABLED_BY_MODE";
  bool vision_trigger_requested = false;
  std::string vision_trigger_reason = "WAITING_FOR_FIRST_RELIABILITY_EVENT";
  reliability::LocalRisk last_visual_lidar_risk;
  last_visual_lidar_risk.status = "NO_PREVIOUS_LIDAR_OBSERVABILITY";
  bool last_visual_quality_passed = false;
  bool last_visual_complementary = false;
  bool last_visual_applied = false;
  int last_visual_complementary_rank = 0;
  std::uint64_t last_visual_observation_stamp_ns = 0;
  bool visual_update_in_scan_interval = false;
  std::uint64_t last_actual_external_update_ns = frontend.getState().stamp_ns;
  std::string last_visual_rejection_reason = "NO_VISUAL_EVENT";
  std::uint64_t coast_start_ns = 0;
  std::uint64_t recovery_stamp_ns = 0;
  bool relocalization_latched = false;
  auto processImageEvent = [&](const ImageTimeEvent& event) {
    const VisualMeasurement& measurement =
        active_visual_measurements[event.measurement_index];
    if (frontend.getState().stamp_ns < event.stamp_ns)
      predictFrontendTo(frontend, inputs.imu, event.stamp_ns);
    const FilterSnapshot event_state = frontend.getState();
    if (event_state.stamp_ns != event.stamp_ns)
      throw std::runtime_error("visual_event_state_timestamp_mismatch");
    if (!event.is_current) {
      visual_anchors[event.stamp_ns] = VisualAnchorSnapshot{event_state};
      return;
    }
    const auto anchor_it = visual_anchors.find(measurement.ref_ns);
    if (anchor_it == visual_anchors.end())
      throw std::runtime_error("causal_visual_reference_snapshot_missing");
    const FilterSnapshot& anchor = anchor_it->second.state;
    const Eigen::Vector3d target_position = anchor.map_T_imu.position +
        anchor.map_T_imu.orientation * measurement.translation;
    const Eigen::Vector3d innovation = target_position -
        event_state.map_T_imu.position;
    const Eigen::Matrix3d anchor_covariance = visualAnchorTargetCovariance(
        anchor, measurement.translation);
    const Eigen::Matrix3d current_covariance =
        positionMarginalCovariance(event_state);
    if (!anchor_covariance.allFinite() || !current_covariance.allFinite())
      throw std::runtime_error("nonfinite_visual_history_covariance");
    const Eigen::Matrix3d noise = conservativeVisualInnovationNoise(
        anchor, event_state, measurement.translation, visual_sigma_m);
    if (!noise.allFinite())
      throw std::runtime_error("nonfinite_conservative_visual_noise");
    Eigen::Matrix3d sensor_covariance = visual_sigma_m * visual_sigma_m *
        Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d innovation_covariance = current_covariance + noise;
    Eigen::LLT<Eigen::Matrix3d> innovation_factor(
        0.5 * (innovation_covariance + innovation_covariance.transpose()));
    double innovation_chi_square = std::numeric_limits<double>::infinity();
    if (innovation_factor.info() == Eigen::Success &&
        innovation_factor.matrixL().toDenseMatrix().diagonal().minCoeff() > 0.0)
      innovation_chi_square = innovation.dot(innovation_factor.solve(innovation));

    reliability::VisualQualityObservation quality_observation;
    quality_observation.source_valid = measurement.source_valid;
    quality_observation.quality_metadata_available =
        measurement.quality_metadata_available;
    quality_observation.reference_stamp_ns = measurement.ref_ns;
    quality_observation.current_stamp_ns = measurement.cur_ns;
    quality_observation.depth_stamp_ns = measurement.depth_stamp_ns;
    quality_observation.detected_count = measurement.detected_count;
    quality_observation.tracked_count = measurement.tracked_count;
    quality_observation.depth_associated_count =
        measurement.depth_associated_count;
    quality_observation.pnp_inlier_count = measurement.inliers;
    quality_observation.inlier_ratio = measurement.ratio;
    quality_observation.depth_fraction = measurement.depth_fraction;
    quality_observation.grid_occupancy = measurement.grid_occupancy;
    quality_observation.hull_fraction = measurement.hull_fraction;
    quality_observation.median_parallax_px = measurement.median_parallax_px;
    quality_observation.reprojection_rmse_px = measurement.reprojection;
    quality_observation.innovation_chi_square = innovation_chi_square;
    const reliability::VisualQualityDecision quality =
        reliability::assessVisualQuality(quality_observation, reliability_config);
    const Eigen::Vector3d map_imu_origin_minus_lidar =
        -(event_state.map_T_imu.orientation * T_imu_lidar_pose.position);
    reliability::VisualSubspaceDecision visual_subspace;
    if (last_visual_lidar_risk.valid &&
        last_visual_lidar_risk.map_support_sufficient) {
      visual_subspace = reliability::assessVisualWeakSubspaceInformation(
          last_visual_lidar_risk, map_imu_origin_minus_lidar,
          sensor_covariance, reliability_config);
    } else {
      visual_subspace.status = last_visual_lidar_risk.valid
          ? "LIDAR_MAP_SUPPORT_INSUFFICIENT"
          : "LIDAR_WEAK_SUBSPACE_UNAVAILABLE";
    }
    if (vision_trigger_requested) ++visual_trigger_count;
    if (measurement.source_valid) ++visual_observations_available;
    if (quality.passed) ++visual_quality_pass_count;
    if (quality.passed && visual_subspace.valid && visual_subspace.complementary)
      ++visual_weak_information_count;
    last_visual_quality_passed = quality.passed;
    last_visual_complementary = visual_subspace.complementary;
    last_visual_applied = false;
    last_visual_complementary_rank = visual_subspace.measurement_rank;
    last_visual_observation_stamp_ns = measurement.cur_ns;
    last_visual_rejection_reason = quality.passed
        ? (visual_subspace.complementary ? "NONE" : visual_subspace.status)
        : quality.rejection_reason;

    std::string status = "NOT_TRIGGERED";
    std::string rejection = "NOT_TRIGGERED";
    PoseCorrectionDelta delta;
    double update_ms = 0.0;
    bool applied = false;
    if (!vision_trigger_requested) {
      rejection = "NOT_TRIGGERED";
    } else if (!quality.passed) {
      rejection = quality.rejection_reason;
      status = "REJECTED_" + rejection;
    } else if (!visual_subspace.valid || !visual_subspace.complementary ||
               visual_subspace.measurement_rank <= 0) {
      rejection = visual_subspace.status;
      status = "REJECTED_NO_COMPLEMENTARY_WEAK_INFORMATION";
    } else {
      std::string update_failure;
      const auto update_start = std::chrono::steady_clock::now();
      applied = frontend.applyProjectedPositionMeasurement(
          target_position, noise, visual_subspace.measurement_basis,
          visual_subspace.measurement_rank, &delta, &update_failure);
      update_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - update_start).count();
      if (applied) {
        status = "APPLIED_PROJECTED_COMPLEMENTARY_TRANSLATION";
        rejection = "NONE";
        ++visual_updates_total;
        if (delta.position.norm() > 1e-12 || delta.velocity.norm() > 1e-12)
          ++visual_nonzero_updates;
        last_visual_applied = true;
        visual_update_in_scan_interval = true;
        last_actual_external_update_ns = measurement.cur_ns;
      } else {
        rejection = "UPDATE_FAILED_" + update_failure;
        status = "REJECTED_UPDATE_FAILURE";
      }
    }
    if (!applied && rejection != "NOT_TRIGGERED") {
      ++visual_rejection_counts[rejection];
      ++visual_rejected_total;
    }
    visual_last_update_status = status;
    const FilterSnapshot after = frontend.getState();
    visual_updates << measurement.transaction_ref << ','
        << measurement.transaction_cur << ',' << measurement.ref_ns << ','
        << measurement.cur_ns << ',' << measurement.depth_stamp_ns << ','
        << status << ',' << (vision_trigger_requested ? 1 : 0) << ','
        << (measurement.source_valid ? 1 : 0) << ',' << (quality.passed ? 1 : 0)
        << ',' << (quality.passed ? "NONE" : quality.rejection_reason) << ','
        << measurement.detected_count << ',' << measurement.tracked_count << ','
        << measurement.depth_associated_count << ',' << measurement.depth_fraction
        << ',' << measurement.grid_occupancy << ',' << measurement.hull_fraction
        << ',' << measurement.median_parallax_px << ',' << measurement.inliers
        << ',' << measurement.ratio << ',' << measurement.reprojection << ','
        << innovation_chi_square << ','
        << matrixField(visual_subspace.visual_information) << ','
        << matrixField(visual_subspace.projected_weak_information) << ','
        << matrixField(Eigen::MatrixXd(visual_subspace.projected_eigenvalues))
        << ',' << visual_subspace.effective_rank << ','
        << visual_subspace.measurement_rank << ','
        << visual_subspace.effective_condition << ',' << visual_subspace.status << ','
        << vision_trigger_reason << ','
        << vectorField(target_position) << ',' << vectorField(innovation) << ','
        << innovation.norm() << ',' << delta.position.norm() << ','
        << delta.velocity.norm() << ',' << matrixField(anchor_covariance) << ','
        << matrixField(current_covariance) << ',' << matrixField(noise) << ','
        << update_ms << '\n';
    if (!frontend.postconditionsValid(&failure) || after.stamp_ns != event.stamp_ns)
      throw std::runtime_error("causal_visual_update_postcondition_failed:" + failure);
  };
  for (std::size_t index = 0; index < assets.size(); ++index) {
    const ScanAsset& asset = assets[index];
    const p4_i2::PoseRecord& saved = inputs.scans[index];
    visual_update_in_scan_interval = false;
    if (saved.transaction_id != asset.transaction_id || saved.stamp_ns != asset.stamp_ns)
      throw std::runtime_error("filter_input_and_cloud_metadata_alignment_mismatch");
    const auto frame_start = std::chrono::steady_clock::now();
    if (enable_vision) {
      while (image_event_index < image_events.size() &&
             image_events[image_event_index].stamp_ns < asset.stamp_ns) {
        processImageEvent(image_events[image_event_index]);
        ++image_event_index;
      }
    }
    const auto prediction_start = std::chrono::steady_clock::now();
    const FilterSnapshot start = frontend.getState();
    if (enable_vision) {
      predictFrontendTo(frontend, inputs.imu, asset.stamp_ns);
    } else {
      const p4_i2::ImuVector window = p4_i2::imuWindow(
          inputs.imu, start.stamp_ns, asset.stamp_ns);
      std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> imu_poses;
      if (!frontend.predictImuSequence(window, asset.stamp_ns, &imu_poses, &failure))
        throw std::runtime_error("dual_reliability_imu_prediction_failed_tx_" +
            std::to_string(asset.transaction_id) + ":" + failure);
    }
    const double prediction_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - prediction_start).count();
    const FilterSnapshot predicted = frontend.getState();
    const Eigen::Matrix4d predicted_map_T_imu = poseMatrix(predicted.map_T_imu);
    const Eigen::Matrix4d T_imu_lidar = poseMatrix(T_imu_lidar_pose);
    const Eigen::Matrix4d predicted_map_T_lidar = predicted_map_T_imu * T_imu_lidar;

    const Cloud::Ptr raw_source = loadRawCloudAt(cloud_binary_path, asset);
    const Cloud::Ptr source = preprocessSource(raw_source);
    const uint64_t source_hash = sourceCloudHash(source);
    if ((!asset.expected_source_hash_available && map_profile != "corridor01") ||
        (asset.expected_source_hash_available &&
         source_hash != asset.expected_source_hash))
      throw std::runtime_error("prepared_source_hash_mismatch_tx_" +
                               std::to_string(asset.transaction_id));

    const auto nominal_start = std::chrono::steady_clock::now();
    const Candidate nominal = runNdtCandidate(
        ndt, source, predicted_map_T_lidar, 0, "M0_STRICT_PREDICTION");
    const double nominal_ndt_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - nominal_start).count();
    uint64_t ndt_call_count = 1;

    const double gradient_gate_ms = 0.0;

    Eigen::Matrix<double, 6, 6> predicted_map_covariance =
        Eigen::Matrix<double, 6, 6>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    p6_i4::PoseCovarianceSpectrum covariance_spectrum;
    covariance_spectrum.reason = "UNONLOCAL_DISABLED_BY_MODE";
    if (use_unonlocal) {
      try {
        predicted_map_covariance = projectMapProductPoseCovariance(predicted);
        covariance_spectrum = p6_i4::analyzePoseCovariance(predicted_map_covariance);
      } catch (const std::exception&) {
        covariance_spectrum.reason = "POSE_COVARIANCE_PROJECTION_FAILED";
      }
    }

    reliability::Vector6d innovation = reliability::Vector6d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    reliability::ProbeTrigger probe_trigger;
    probe_trigger.reason = "DISABLED_BY_MODE";
    if (use_unonlocal && nominal.converged) {
      const Pose3d nominal_lidar_pose = poseFromMatrix(nominal.pose);
      const Pose3d nominal_imu_pose = p4_i2::lidarMeasurementToImu(
          nominal_lidar_pose, T_imu_lidar_pose);
      innovation = reliability::mapProductInnovation(
          p4_i2::asIsometry(predicted.map_T_imu),
          p4_i2::asIsometry(nominal_imu_pose));
      if (use_unonlocal)
        probe_trigger = reliability::shouldRunNonlocalProbes(
            asset.transaction_id, innovation, predicted_map_covariance,
            reliability_config);
    }

    reliability::NonlocalTerminalStability terminal_stability;
    terminal_stability.status = "NOT_PROBED";
    Candidate positive, negative;
    double extra_probe_ms = 0.0;
    bool probes_executed = false;
    if (nominal.converged && probe_trigger.run_probes) {
      const auto probe_start = std::chrono::steady_clock::now();
      if (covariance_spectrum.valid && covariance_spectrum.effective_rank > 0) {
        const double eigenvalue = covariance_spectrum.eigenvalues(5);
        if (eigenvalue > 0.0 && std::isfinite(eigenvalue)) {
          const Eigen::Matrix<double, 6, 1> delta =
              reliability_config.probe_prior_sigma * std::sqrt(eigenvalue) *
              covariance_spectrum.eigenvectors.col(5);
          const Eigen::Matrix4d positive_map_T_imu = p6_i4::boxplusMapPose(
              predicted_map_T_imu, delta);
          const Eigen::Matrix4d negative_map_T_imu = p6_i4::boxplusMapPose(
              predicted_map_T_imu, -delta);
          positive = runNdtCandidate(ndt, source,
              positive_map_T_imu * T_imu_lidar, 1, "M_PLUS_PRIOR_PRINCIPAL");
          negative = runNdtCandidate(ndt, source,
              negative_map_T_imu * T_imu_lidar, 2, "M_MINUS_PRIOR_PRINCIPAL");
          ndt_call_count += 2;
          probes_executed = true;
          terminal_stability = reliability::analyzeNonlocalTerminalStability(
              terminalCapture(nominal), terminalCapture(positive),
              terminalCapture(negative),
              p4_i2::asIsometry(T_imu_lidar_pose), 2);
        } else {
          terminal_stability.status = "PROBE_PRIOR_VARIANCE_NOT_POSITIVE";
        }
      } else {
        terminal_stability.status = "PROBE_PRIOR_COVARIANCE_INVALID";
      }
      extra_probe_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - probe_start).count();
      if (terminal_stability.status != "RECORDED_NO_BASIN_CLASSIFICATION" &&
          terminal_stability.status != "NDT_NOT_CONVERGED" &&
          terminal_stability.status != "INVALID_TERMINAL_POSE" &&
          terminal_stability.status != "INVALID_IMU_LIDAR_EXTRINSIC" &&
          terminal_stability.status != "NONFINITE_FIXED_OBJECTIVE") {
        terminal_stability.geometry_valid = false;
        terminal_stability.objectives_finite = false;
      }
    }

    const auto curvature_start = std::chrono::steady_clock::now();
    std::vector<reliability::GeometricObservation> geometric_observations;
    if (use_uobs && nominal.converged) {
      geometric_observations = ndt.geometricObservations(
          *source, nominal.pose.cast<float>(), 0.8);
    }
    const double local_curvature_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - curvature_start).count();
    reliability::LocalObservability local_observability;
    if (use_uobs) {
      local_observability = reliability::analyzeGeometricObservability(
          geometric_observations, nominal.converged, 0.8);
    } else {
      local_observability.status = "UOBS_DISABLED_BY_MODE";
    }
    std::string uobs_status = local_observability.status;
    reliability::LocalRisk local_risk;
    if (use_uobs)
      local_risk = reliability::assessLocalRisk(local_observability, reliability_config);
    else
      local_risk.status = "UOBS_DISABLED_BY_MODE";
    if (enable_vision) {
      const GeometricSearchDiagnostic nominal_search = ndt.diagnoseSearch(
          *source, nominal.pose.cast<float>(), 0.8);
      const GeometricSearchDiagnostic predicted_search = ndt.diagnoseSearch(
          *source, predicted_map_T_lidar.cast<float>(), 0.8);
      const Eigen::Matrix4d predicted_pose_double =
          predicted_map_T_lidar.cast<double>();
      const double pose_translation_difference =
          (nominal.pose.block<3, 1>(0, 3) -
           predicted_pose_double.block<3, 1>(0, 3)).norm();
      const double pose_rotation_difference = rotationDifferenceDeg(
          predicted_pose_double, nominal.pose) * M_PI / 180.0;
      map_support_diagnostic << asset.transaction_id << ',' << asset.time_s << ','
          << source->size() << ',' << nominal_search.sampled_source_points << ','
          << predicted_search.sampled_source_points << ','
          << (nominal.converged ? 1 : 0) << ',' << nominal.fitness << ','
          << nominal.objective << ','
          << (nominal.converged && local_observability.valid_correspondence_count == 0
                  ? "NDT_CONVERGED_WITHOUT_GEOMETRIC_SUPPORT" : "") << ','
          << vectorField(predicted_pose_double.block<3, 1>(0, 3)) << ','
          << vectorField(nominal.pose.block<3, 1>(0, 3)) << ','
          << pose_translation_difference << ',' << pose_rotation_difference << ','
          << local_observability.valid_correspondence_count << ','
          << local_observability.rejected_covariance_count << ','
          << nominal_search.queries_with_neighbors_r << ','
          << nominal_search.queries_with_neighbors_2r << ','
          << nominal_search.total_neighbors_r << ','
          << nominal_search.total_neighbors_2r << ','
          << predicted_search.queries_with_neighbors_r << ','
          << predicted_search.queries_with_neighbors_2r << ','
          << predicted_search.total_neighbors_r << ','
          << predicted_search.total_neighbors_2r << ','
          << maximumPositionSigma(predicted) << ','
          << maximumRotationSigma(predicted) << ','
          << local_risk.reliable_dimension << ',' << local_risk.weak_dimension << ','
          << (nominal_search.valid ? 1 : 0) << ','
          << (predicted_search.valid ? 1 : 0) << '\n';
      map_support_diagnostic.flush();
    }

    reliability::DualReliabilityDecision decision;
    if (mode == "STRICT_BASELINE") {
      if (!nominal.converged)
        throw std::runtime_error("strict_single_start_ndt_not_converged_tx_" +
                                 std::to_string(asset.transaction_id));
      decision.action = reliability::UpdateAction::NORMAL_UPDATE;
      decision.reason = "STRICT_BASELINE_ISOTROPIC";
    } else {
      decision = reliability::decideDualReliability(
          local_risk, terminal_stability, nominal.converged,
          predicted.map_T_imu.orientation.toRotationMatrix(),
          parameters.pose_position_sigma_m, parameters.pose_rotation_sigma_rad,
          use_uobs, use_unonlocal, reliability_config);
    }

    const bool nonlocal_translation_risk = terminal_stability.response_valid &&
        std::max(terminal_stability.delta_position_imu_positive.norm(),
                 terminal_stability.delta_position_imu_negative.norm()) >
            reliability_config.terminal_translation_limit_m;
    const bool nonlocal_rotation_risk = terminal_stability.response_valid &&
        std::max(terminal_stability.delta_rotation_positive.norm(),
                 terminal_stability.delta_rotation_negative.norm()) >
            reliability_config.terminal_rotation_limit_rad;
    const Eigen::Vector3d map_imu_origin_minus_lidar =
        -(predicted.map_T_imu.orientation * T_imu_lidar_pose.position);
    reliability::LocalRisk routed_lidar_risk = riskWithNonlocalResponse(
        local_risk, terminal_stability,
        decision.nonlocal_risk || nonlocal_translation_risk || nonlocal_rotation_risk,
        map_imu_origin_minus_lidar, reliability_config);
    reliability::ConditionalRouteDecision conditional_route;
    conditional_route.mode = reliability::ConditionalFusionMode::NORMAL_LIDAR;
    conditional_route.apply_lidar_measurement = nominal.converged;
    conditional_route.preserve_lidar_reliable_subspace = nominal.converged;
    double imu_coasting_duration_s = 0.0;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double maximum_position_sigma_m = nan;
    double maximum_rotation_sigma_rad = nan;
    if (enable_vision) {
      reliability::ConditionalRouteInput route_input;
      route_input.lidar_converged = nominal.converged;
      route_input.lidar_registration_unstable = !nominal.converged ||
          decision.nonlocal_risk || nonlocal_translation_risk ||
          nonlocal_rotation_risk || routed_lidar_risk.weak_dimension > 0;
      route_input.map_support_sufficient = routed_lidar_risk.valid &&
          routed_lidar_risk.map_support_sufficient;
      route_input.lidar_weak_dimension = routed_lidar_risk.weak_dimension;
      route_input.lidar_reliable_dimension = routed_lidar_risk.reliable_dimension;
      const std::uint64_t visual_age_ns = asset.stamp_ns >=
          last_visual_observation_stamp_ns
          ? asset.stamp_ns - last_visual_observation_stamp_ns
          : std::numeric_limits<std::uint64_t>::max();
      const bool recent_visual = last_visual_observation_stamp_ns != 0 &&
          visual_age_ns <= static_cast<std::uint64_t>(
              reliability_config.maximum_visual_pair_gap_s * 1e9);
      route_input.vision_trigger_requested = vision_trigger_requested;
      route_input.vision_observation_available = recent_visual;
      route_input.vision_quality_passed = recent_visual &&
          last_visual_quality_passed && last_visual_applied;
      route_input.vision_complementary = recent_visual &&
          last_visual_complementary && last_visual_applied;
      route_input.vision_complementary_rank = route_input.vision_complementary
          ? last_visual_complementary_rank : 0;
      route_input.time_without_external_constraint_s = coast_start_ns == 0
          ? 0.0 : static_cast<double>(asset.stamp_ns - coast_start_ns) * 1e-9;
      imu_coasting_duration_s = route_input.time_without_external_constraint_s;
      maximum_position_sigma_m = maximumPositionSigma(predicted);
      maximum_rotation_sigma_rad = maximumRotationSigma(predicted);
      route_input.maximum_position_sigma_m = maximum_position_sigma_m;
      route_input.maximum_rotation_sigma_rad = maximum_rotation_sigma_rad;
      conditional_route = reliability::routeConditionalCompensation(
          route_input, reliability_config);
      if (relocalization_latched && !conditional_route.imu_coasting) {
        relocalization_latched = false;
        recovery_stamp_ns = asset.stamp_ns;
      }
      if (conditional_route.relocalization_required)
        relocalization_latched = true;
      if (relocalization_latched && conditional_route.imu_coasting) {
        conditional_route.mode = reliability::ConditionalFusionMode::RELOCALIZATION_REQUIRED;
        conditional_route.relocalization_required = true;
        conditional_route.reason += ";RELOCALIZATION_LATCHED";
      }
      if (conditional_route.imu_coasting) {
        if (coast_start_ns == 0) coast_start_ns = asset.stamp_ns;
      } else if (coast_start_ns != 0) {
        recovery_stamp_ns = asset.stamp_ns;
        coast_start_ns = 0;
      }
      last_visual_lidar_risk = routed_lidar_risk;
      vision_assist.observation_available = recent_visual;
    }

    const std::string route_provisional = reliability::toString(conditional_route.mode);
    const auto base_noise = makeBasePoseNoise(parameters);
    const auto& adaptive_noise = decision.measurement_noise.covariance;
    if (enable_vision) {
      writeR2PreupdateDiagnostics(frontend, asset, r2_policy,
          poseFromMatrix(nominal.pose), T_imu_lidar_pose, local_risk,
          routed_lidar_risk, base_noise, adaptive_noise,
          decision.measurement_noise.valid, nonlocal_basis_comparison,
          linearization_consistency, shadow_counterfactual);
    }
    ProjectedPoseInnovation projected_diagnostic;
    PoseCorrectionDelta actual_projected_delta;
    int selected_rank = 0;
    double base_selected_trace = nan, adaptive_selected_trace = nan;
    double adaptive_base_difference = nan;
    bool lidar_update_attempted = false, lidar_nis_rejected = false;
    std::string projected_status = "NO_PROJECTED_UPDATE";
    double update_ms = 0.0;
    bool step_limited = false;
    bool lidar_update_applied = false;
    bool lidar_update_projected = false;
    const bool allow_lidar_update = enable_vision
        ? conditional_route.apply_lidar_measurement
        : decision.action != reliability::UpdateAction::PREDICTION_ONLY;
    if (allow_lidar_update) {
      const bool projected_lidar_route = enable_vision &&
          routed_lidar_risk.weak_dimension > 0;
      // The legacy limiter scales the full translation/rotation increment.
      // Applying it before row-space projection lets discarded weak-axis
      // components alter the scale of reliable-axis measurements. Keep the
      // complete NDT terminal for a directional update and let its projected
      // EKF covariance control the accepted correction instead.
      const Eigen::Matrix4d used_pose = projected_lidar_route
          ? nominal.pose
          : limitStep(nominal.pose, previous_used, has_previous_used,
                      &step_limited);
      if (!projected_lidar_route) {
        previous_used = used_pose;
        has_previous_used = true;
      }
      const Pose3d used_lidar_pose = poseFromMatrix(used_pose);
      const Pose3d used_imu_measurement = p4_i2::lidarMeasurementToImu(
          used_lidar_pose, T_imu_lidar_pose);
      const auto update_start = std::chrono::steady_clock::now();
      if (mode == "STRICT_BASELINE") {
        lidar_update_attempted = true;
        if (!frontend.applyPoseMeasurement(used_imu_measurement, nullptr, &failure))
          throw std::runtime_error("strict_baseline_pose_update_failed_tx_" +
              std::to_string(asset.transaction_id) + ":" + failure);
        lidar_update_applied = true;
      } else if (enable_vision && routed_lidar_risk.weak_dimension > 0) {
        Eigen::Matrix<double, 6, 6> reliable_basis;
        int reliable_rank = 0;
        std::string reliable_basis_failure;
        const Pose3d nominal_imu_measurement = p4_i2::lidarMeasurementToImu(
            poseFromMatrix(nominal.pose), T_imu_lidar_pose);
        if (!makeReliablePoseMeasurementBasis(routed_lidar_risk,
                nominal_imu_measurement, T_imu_lidar_pose,
                &reliable_basis, &reliable_rank,
                &reliable_basis_failure) || reliable_rank <= 0) {
          update_ms = 0.0;
          conditional_route.apply_lidar_measurement = false;
          conditional_route.imu_coasting = true;
          conditional_route.mode = reliability::ConditionalFusionMode::
              LIDAR_DEGRADED_VISION_INVALID;
          conditional_route.reason += ";RELIABLE_SUBSPACE_BASIS_INVALID:" +
              (reliable_basis_failure.empty() ? "ZERO_RELIABLE_RANK" :
                                                reliable_basis_failure);
          // The NDT terminal was not committed, so keep the step limiter
          // anchored to the propagated state actually held by the filter.
          previous_used = predicted_map_T_lidar;
        } else {
          selected_rank = reliable_rank;
          const Eigen::MatrixXd B = reliable_basis.leftCols(reliable_rank);
          const Eigen::MatrixXd rb = B.transpose() * base_noise * B;
          base_selected_trace = rb.trace();
          if (decision.measurement_noise.valid) {
            const Eigen::MatrixXd ra = B.transpose() * adaptive_noise * B;
            adaptive_selected_trace = ra.trace();
            adaptive_base_difference = (ra - rb).norm();
          }
          lidar_update_attempted = true;
          if (r2PolicyUsesAdaptiveNoise(r2_policy) && !decision.measurement_noise.valid) {
            projected_status = "INVALID_ADAPTIVE_MEASUREMENT_NOISE";
          } else {
            const auto& selected_noise = r2PolicyUsesAdaptiveNoise(r2_policy)
                ? adaptive_noise : base_noise;
            lidar_update_applied = frontend.applyProjectedPoseMeasurementChecked(
                used_imu_measurement, selected_noise, reliable_basis, reliable_rank,
                r2PolicyUsesNisGate(r2_policy), chiSquare99Threshold(reliable_rank),
                &projected_diagnostic, &actual_projected_delta, &failure);
            lidar_update_projected = lidar_update_applied;
            lidar_nis_rejected = failure == "SELECTED_NIS_REJECTED";
            projected_status = lidar_update_applied ? "COMMITTED" : failure;
          }
          if (!lidar_update_applied) {
            previous_used = predicted_map_T_lidar;
            has_previous_used = true;
          }
        }
      } else {
        lidar_update_attempted = true;
        if (!decision.measurement_noise.valid ||
            !frontend.applyPoseMeasurement(used_imu_measurement,
                decision.measurement_noise.covariance, nullptr, &failure))
          throw std::runtime_error("adaptive_pose_update_failed_tx_" +
              std::to_string(asset.transaction_id) + ":" + failure);
        lidar_update_applied = true;
      }
      update_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - update_start).count();
    } else {
      // The filter has still advanced to this scan end. Do not let the next
      // measurement's step limiter compare against a stale pre-rejection NDT.
      previous_used = predicted_map_T_lidar;
      has_previous_used = true;
    }

    const FilterSnapshot corrected = frontend.getState();
    if (lidar_update_applied) last_actual_external_update_ns = asset.stamp_ns;
    const bool external_update_in_scan_interval = lidar_update_applied || visual_update_in_scan_interval;
    const std::string route_actual = lidar_update_applied
        ? reliability::toString(conditional_route.mode)
        : (visual_update_in_scan_interval ? "VISUAL_ONLY_THIS_INTERVAL" : "IMU_ONLY_THIS_INTERVAL");
    const double time_since_actual_external_update =
        static_cast<double>(asset.stamp_ns - last_actual_external_update_ns) * 1e-9;
    if (enable_vision) {
      projected_innovation << asset.transaction_id << ',' << asset.time_s << ',' << r2PolicyName(r2_policy)
          << ',' << route_provisional << ',' << route_actual << ',' << (routed_lidar_risk.weak_dimension > 0)
          << ',' << selected_rank << ',' << routed_lidar_risk.weak_dimension
          << ',' << projected_diagnostic.residual_norm << ',' << projected_diagnostic.projected_residual_norm
          << ',' << base_selected_trace << ',' << adaptive_selected_trace << ',' << adaptive_base_difference
          << ',' << projected_diagnostic.projected_noise_trace << ',' << projected_diagnostic.projected_noise_min_eigenvalue
          << ',' << projected_diagnostic.projected_noise_max_eigenvalue << ',' << projected_diagnostic.innovation_covariance_trace
          << ',' << projected_diagnostic.nis << ',' << chiSquare99Threshold(selected_rank)
          << ',' << r2PolicyUsesNisGate(r2_policy) << ',' << lidar_update_attempted << ',' << lidar_update_applied
          << ',' << lidar_nis_rejected << ',' << projected_status << ',' << actual_projected_delta.position.norm()
          << ',' << actual_projected_delta.rotation.norm() << ',' << actual_projected_delta.velocity.norm()
          << ',' << maximumPositionSigma(predicted) << ',' << maximumPositionSigma(corrected) << '\n';
    }
    if (corrected.stamp_ns != asset.stamp_ns || !frontend.postconditionsValid(&failure))
      throw std::runtime_error("dual_reliability_filter_postcondition_failed_tx_" +
          std::to_string(asset.transaction_id) + ":" + failure);
    if (lidar_update_applied && lidar_update_projected) {
      // Do not let the rejected LiDAR-weak components leak into the next
      // scan's step-limiter reference. Carry forward the pose actually held
      // by the filter after the projected correction.
      previous_used = poseMatrix(corrected.map_T_imu) * T_imu_lidar;
      has_previous_used = true;
    }
    p4_i2::writeRow(trajectory, index, saved, predicted.map_T_imu,
                    predicted, corrected, T_imu_lidar_pose);

    Eigen::Matrix3d raw_bp = Eigen::Matrix3d::Constant(nan);
    Eigen::Matrix3d raw_br = Eigen::Matrix3d::Constant(nan);
    if (terminal_stability.response_valid) {
      raw_bp = 0.5 * (terminal_stability.delta_position_imu_positive *
                          terminal_stability.delta_position_imu_positive.transpose() +
                      terminal_stability.delta_position_imu_negative *
                          terminal_stability.delta_position_imu_negative.transpose());
      raw_br = 0.5 * (terminal_stability.delta_rotation_positive *
                          terminal_stability.delta_rotation_positive.transpose() +
                      terminal_stability.delta_rotation_negative *
                      terminal_stability.delta_rotation_negative.transpose());
    }
    vision_trigger_requested = enable_vision &&
        routed_lidar_risk.valid && routed_lidar_risk.weak_dimension > 0;
    vision_trigger_reason = !enable_vision ? "DISABLED_BY_MODE" :
        (local_risk.valid && local_risk.weak_dimension > 0 &&
         decision.nonlocal_risk ? "UOBS_AND_UNONLOCAL_WEAK_SUBSPACE" :
         (local_risk.valid && local_risk.weak_dimension > 0
             ? "UOBS_JOINT_WEAK_SUBSPACE" :
             (decision.nonlocal_risk ? "UNONLOCAL_TERMINAL_RESPONSE" :
                                       "NO_WEAK_LIDAR_DIRECTION")));
    vision_assist.trigger_requested = vision_trigger_requested;
    vision_assist.trigger_reason = vision_trigger_reason;
    events << mode << ',' << asset.transaction_id << ',' << asset.stamp_ns << ','
           << asset.time_s << ',' << (asset.expected_source_hash_available ? 1 : 0)
           << ',' << (local_observability.valid ? 1 : 0) << ','
           << uobs_status << ','
           << (score_gradient_gate_passed ? 1 : 0) << ','
           << score_gradient_gate_samples << ','
           << local_observability.effective_weight_sum << ','
           << local_observability.valid_correspondence_count << ','
           << local_observability.rejected_covariance_count << ','
           << (local_observability.valid
                   ? matrixField(local_observability.normalized_geometric_information)
                   : "NA") << ','
           << (local_observability.map_support_sufficient ? 1 : 0) << ','
           << local_observability.map_support_status << ','
           << (local_observability.schur_decoupling_valid
                   ? matrixField(Eigen::MatrixXd(
                         local_observability.rotation_schur_eigenvalues)) : "NA") << ','
           << (local_observability.schur_decoupling_valid
                   ? matrixField(Eigen::MatrixXd(
                         local_observability.translation_schur_eigenvalues)) : "NA") << ','
           << (local_risk.valid ? matrixField(local_risk.joint_weak_basis) : "NA") << ','
           << (local_risk.valid ? matrixField(local_risk.joint_reliable_basis) : "NA") << ','
           << (enable_vision ? active_visual_measurements.size() : 0) << ','
           << visual_updates_total << ',' << visual_nonzero_updates << ','
           << visual_last_update_status << ',' << vision_trigger_reason << ','
           << (local_risk.translation_weak ? 1 : 0) << ','
           << local_risk.translation_weak_ratio << ','
           << local_risk.translation_min_eigenvalue << ','
           << local_risk.translation_block_condition << ','
           << (local_risk.valid ? vectorField(local_risk.map_translation_weak_direction) : "NA") << ','
           << (local_risk.rotation_weak ? 1 : 0) << ','
           << local_risk.rotation_weak_ratio << ','
           << local_risk.rotation_min_eigenvalue << ','
           << local_risk.rotation_block_condition << ','
           << (local_risk.valid ? vectorField(local_risk.map_rotation_weak_direction) : "NA") << ','
           << probe_trigger.innovation.mahalanobis_squared << ','
           << probe_trigger.innovation.status << ',' << probe_trigger.reason << ','
           << (probe_trigger.run_probes ? 1 : 0) << ','
           << (probes_executed ? 1 : 0) << ',' << terminal_stability.status << ','
           << ndt_call_count << ','
           << (nominal.converged ? 1 : 0) << ',' << nominal.iterations << ','
           << nominal.objective << ',' << poseField(nominal.pose) << ','
           << (ndt_call_count == 3 ? (positive.converged ? 1 : 0) : -1) << ','
           << (ndt_call_count == 3 ? positive.iterations : -1) << ','
           << (ndt_call_count == 3 ? positive.objective : nan) << ','
           << (ndt_call_count == 3 ? poseField(positive.pose) : "NOT_PROBED") << ','
           << (ndt_call_count == 3 ? (negative.converged ? 1 : 0) : -1) << ','
           << (ndt_call_count == 3 ? negative.iterations : -1) << ','
           << (ndt_call_count == 3 ? negative.objective : nan) << ','
           << (ndt_call_count == 3 ? poseField(negative.pose) : "NOT_PROBED") << ','
           << (terminal_stability.response_valid ? vectorField(terminal_stability.delta_position_imu_positive) : "NA") << ','
           << (terminal_stability.response_valid ? vectorField(terminal_stability.delta_position_imu_negative) : "NA") << ','
           << (terminal_stability.response_valid ? vectorField(terminal_stability.delta_rotation_positive) : "NA") << ','
           << (terminal_stability.response_valid ? vectorField(terminal_stability.delta_rotation_negative) : "NA") << ','
           << (terminal_stability.response_valid ? matrixField(raw_bp) : "NA") << ','
           << (terminal_stability.response_valid ? matrixField(raw_br) : "NA") << ','
           << terminal_stability.max_nominal_translation_delta_m << ','
           << terminal_stability.max_nominal_rotation_delta_rad << ','
           << terminal_stability.positive_negative_translation_gap_m << ','
           << terminal_stability.positive_negative_rotation_gap_rad << ','
           << terminal_stability.positive_minus_negative_objective << ','
           << (decision.measurement_noise.valid
                   ? matrixField(decision.measurement_noise.covariance.block<3, 3>(0, 0)) : "NA") << ','
           << (decision.measurement_noise.valid
                   ? matrixField(decision.measurement_noise.rotation_map_covariance) : "NA") << ','
           << (decision.measurement_noise.valid
                   ? matrixField(decision.measurement_noise.covariance.block<3, 3>(3, 3)) : "NA") << ','
           << (step_limited ? 1 : 0) << ','
           << reliability::toString(decision.action) << ',' << decision.reason << ','
           << (decision.local_curvature_used ? 1 : 0) << ','
           << (decision.nonlocal_response_used ? 1 : 0) << ','
           << vision_assist.status << ','
           << (vision_assist.observation_available ? 1 : 0) << ','
           << (vision_assist.trigger_requested ? 1 : 0) << ','
           << vision_assist.trigger_reason << ','
           << reliability::toString(conditional_route.mode) << ','
           << conditional_route.reason << ','
           << routed_lidar_risk.reliable_dimension << ','
           << routed_lidar_risk.weak_dimension << ','
           << visual_trigger_count << ',' << visual_observations_available << ','
           << visual_quality_pass_count << ',' << visual_weak_information_count << ','
           << visual_updates_total << ',' << visual_rejected_total << ','
           << last_visual_rejection_reason << ',' << imu_coasting_duration_s << ','
           << (conditional_route.relocalization_required ? 1 : 0) << ','
           << recovery_stamp_ns << ','
           << local_curvature_ms << ',' << gradient_gate_ms << ','
           << extra_probe_ms << ',' << maximum_position_sigma_m << ','
           << maximum_rotation_sigma_rad << ',' << (lidar_update_applied ? 1 : 0)
           << ',' << (lidar_update_projected ? 1 : 0)
           << ',' << route_provisional << ',' << lidar_update_attempted << ',' << lidar_update_applied
           << ',' << lidar_nis_rejected << ',' << visual_update_in_scan_interval
           << ',' << external_update_in_scan_interval << ',' << last_actual_external_update_ns
           << ',' << time_since_actual_external_update << ',' << route_actual << '\n';
    const double total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - frame_start).count();
    runtime << mode << ',' << asset.transaction_id << ',' << prediction_ms << ','
            << nominal_ndt_ms << ',' << local_curvature_ms << ','
            << gradient_gate_ms << ',' << extra_probe_ms << ',' << update_ms << ','
            << total_ms << ',' << ndt_call_count << ',' << probe_trigger.reason << ','
            << reliability::toString(decision.action) << ','
            << vision_assist.status << ','
            << (vision_assist.trigger_requested ? 1 : 0) << ','
            << vision_assist.trigger_reason << '\n';
    if ((index + 1) % 100 == 0 || index + 1 == assets.size()) {
      trajectory.flush(); events.flush(); runtime.flush();
      std::cerr << mode << "_PROGRESS=" << index + 1 << "/" << assets.size()
                << " ndt_calls=" << ndt_call_count
                << " decision=" << reliability::toString(decision.action) << '\n';
    }
  }
  if (enable_vision) {
    const uint64_t final_stamp = assets.back().stamp_ns;
    while (image_event_index < image_events.size()) {
      if (image_events[image_event_index].stamp_ns > final_stamp)
        throw std::runtime_error("visual_image_event_after_last_scan");
      processImageEvent(image_events[image_event_index]);
      ++image_event_index;
    }
    visual_updates.flush();
  }
  std::cout << "P6_I6B_MODE_COMPLETE=" << mode << " frames=" << assets.size() << '\n';
}

double normalizedPoseDistance(const Eigen::Matrix4d& left,
                              const Eigen::Matrix4d& right) {
  const double translation = (left.block<3, 1>(0, 3) -
                              right.block<3, 1>(0, 3)).norm();
  const double rotation = rotationDifferenceDeg(left, right);
  return std::max(translation / 0.20, rotation / 2.0);
}

struct CandidateCluster {
  int first_seed = std::numeric_limits<int>::max();
  std::vector<std::size_t> members;
  std::size_t representative = 0;
};

std::vector<CandidateCluster> completeLinkClusters(
    std::vector<Candidate>* candidates) {
  std::vector<std::size_t> converged;
  for (std::size_t i = 0; i < candidates->size(); ++i)
    if ((*candidates)[i].converged) converged.push_back(i);
  std::vector<CandidateCluster> clusters;
  for (const std::size_t i : converged)
    clusters.push_back({(*candidates)[i].seed_index, {i}, i});
  while (clusters.size() > 1) {
    std::stable_sort(clusters.begin(), clusters.end(),
        [](const CandidateCluster& a, const CandidateCluster& b) {
          return a.first_seed < b.first_seed;
        });
    double best = std::numeric_limits<double>::infinity();
    std::size_t best_left = clusters.size(), best_right = clusters.size();
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      for (std::size_t j = i + 1; j < clusters.size(); ++j) {
        double complete_distance = 0.0;
        for (const std::size_t left : clusters[i].members)
          for (const std::size_t right : clusters[j].members)
            complete_distance = std::max(complete_distance,
                normalizedPoseDistance((*candidates)[left].pose,
                                       (*candidates)[right].pose));
        if (complete_distance < best) {
          best = complete_distance;
          best_left = i;
          best_right = j;
        }
      }
    }
    if (best > 1.0 || best_left == clusters.size()) break;
    CandidateCluster merged;
    merged.first_seed = std::min(clusters[best_left].first_seed,
                                 clusters[best_right].first_seed);
    merged.members = clusters[best_left].members;
    merged.members.insert(merged.members.end(),
        clusters[best_right].members.begin(), clusters[best_right].members.end());
    std::sort(merged.members.begin(), merged.members.end(),
        [&](std::size_t a, std::size_t b) {
          return (*candidates)[a].seed_index < (*candidates)[b].seed_index;
        });
    merged.representative = merged.members.front();
    for (const std::size_t member : merged.members) {
      if ((*candidates)[member].objective >
          (*candidates)[merged.representative].objective)
        merged.representative = member;
    }
    clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(best_right));
    clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(best_left));
    clusters.push_back(std::move(merged));
  }
  std::stable_sort(clusters.begin(), clusters.end(),
      [](const CandidateCluster& a, const CandidateCluster& b) {
        return a.first_seed < b.first_seed;
      });
  for (std::size_t id = 0; id < clusters.size(); ++id)
    for (const std::size_t member : clusters[id].members)
      (*candidates)[member].cluster_id = static_cast<int>(id);
  return clusters;
}

std::vector<std::size_t> objectiveGatedRepresentatives(
    const std::vector<Candidate>& candidates,
    const std::vector<CandidateCluster>& clusters,
    double baseline_objective) {
  const double tolerance = 1e-6 * std::max(std::abs(baseline_objective), 1.0);
  std::vector<std::size_t> eligible;
  for (const CandidateCluster& cluster : clusters) {
    const std::size_t rep = cluster.representative;
    if (candidates[rep].objective >= baseline_objective - tolerance)
      eligible.push_back(rep);
  }
  return eligible;
}

Eigen::Vector3d candidateVisualResidual(const Eigen::Matrix4d& map_T_lidar,
                                        const Eigen::Matrix4d& T_imu_lidar,
                                        const Eigen::Matrix4d& previous_map_T_imu,
                                        const Eigen::Vector3d& visual_translation) {
  const Eigen::Matrix4d map_T_imu = map_T_lidar * T_imu_lidar.inverse();
  const Eigen::Matrix4d relative = previous_map_T_imu.inverse() * map_T_imu;
  return relative.block<3, 1>(0, 3) - visual_translation;
}

DcregSummary runDcreg(const Cloud::Ptr& source, const Cloud::Ptr& target,
                      const Eigen::Matrix4d& initial_pose) {
  DcregSummary summary;
  summary.attempted = true;
  dcreg::SolverParameters parameters;
  dcreg::TestCase test_case;
  test_case.params = parameters;
  test_case.has_ground_truth_pose = false;
  dcreg::RunOptions options;
  options.algorithm = dcreg::Algorithm::kDCReg;
  options.parameterization = dcreg::Parameterization::kSE3;
  options.verbose = false;
  dcreg::Se3State initial;
  initial.rotation = initial_pose.block<3, 3>(0, 0);
  initial.translation = initial_pose.block<3, 1>(0, 3);
  dcreg::DegeneracyCharacterization final_characterization;
  const auto started = std::chrono::steady_clock::now();
  const auto result = dcreg::RunParameterizedRegistration(
      *source, *target, test_case, options, initial,
      [](const dcreg::Se3State& state) { return state.Matrix(); },
      [&parameters, &final_characterization](
          const std::vector<dcreg::Correspondence>& correspondences,
          const dcreg::Se3State& state, dcreg::ParallelMode mode) {
        const dcreg::LinearSystem system =
            dcreg::BuildSe3LinearSystem(correspondences, state, mode);
        const auto detection = dcreg::DetectDegeneracy(system.hessian, parameters);
        final_characterization = dcreg::CharacterizeDegeneracy(detection, parameters);
        return system;
      },
      [](dcreg::Se3State* state, const dcreg::Vector6d& delta) {
        *state = state->BoxPlus(delta);
      });
  summary.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  summary.converged = result.converged;
  summary.degenerate = final_characterization.is_degenerate;
  summary.mask = final_characterization.degenerate_mask;
  summary.lambda_rot = final_characterization.lambda_schur_rot;
  summary.lambda_trans = final_characterization.lambda_schur_trans;
  summary.cond_rot = final_characterization.cond_schur_rot;
  summary.cond_trans = final_characterization.cond_schur_trans;
  summary.pose = result.transform;
  if (summary.converged && !summary.pose.allFinite())
    throw std::runtime_error("dcreg_returned_nonfinite_converged_pose");
  return summary;
}

std::string seedName(int index) {
  switch (index) {
    case 0: return "S0_PREDICTED";
    case 1: return "S1_PLUS_X";
    case 2: return "S2_MINUS_X";
    case 3: return "S3_PLUS_Y";
    case 4: return "S4_MINUS_Y";
    case 5: return "S5_PLUS_YAW";
    case 6: return "S6_MINUS_YAW";
    case 7: return "S7_VISUAL_MOTION";
    default: return "UNKNOWN";
  }
}

void writeCandidateRow(std::ostream& output, const std::string& mode,
                       const ScanAsset& asset, const Candidate& candidate,
                       bool selected) {
  output << mode << ',' << asset.transaction_id << ',' << asset.time_s << ','
         << candidate.seed_index << ',' << candidate.seed_name << ','
         << (candidate.converged ? 1 : 0) << ',' << candidate.cluster_id << ','
         << candidate.objective_gate << ',' << candidate.objective << ','
         << candidate.objective_per_source_point << ','
         << candidate.transformation_probability << ',' << candidate.fitness << ','
         << candidate.iterations << ','
         << candidate.runtime_ms << ',' << (selected ? 1 : 0);
  writePose7(output, candidate.pose);
  output << '\n';
}

void writeDcregRow(std::ostream& output, const std::string& mode,
                   const ScanAsset& asset, const DcregSummary& dc,
                   const Eigen::Matrix4d& baseline_pose) {
  output << mode << ',' << asset.transaction_id << ',' << asset.time_s << ','
         << (dc.attempted ? 1 : 0) << ',' << (dc.converged ? 1 : 0) << ','
         << (dc.degenerate ? 1 : 0);
  for (const bool bit : dc.mask) output << ',' << (bit ? 1 : 0);
  output << ',' << dc.lambda_rot(0) << ',' << dc.lambda_rot(1) << ','
         << dc.lambda_rot(2) << ',' << dc.lambda_trans(0) << ','
         << dc.lambda_trans(1) << ',' << dc.lambda_trans(2) << ','
         << dc.cond_rot << ',' << dc.cond_trans << ',' << dc.runtime_ms << ','
         << (dc.converged ? (dc.pose.block<3, 1>(0, 3) -
                             baseline_pose.block<3, 1>(0, 3)).norm() :
                             std::numeric_limits<double>::quiet_NaN()) << ','
         << (dc.converged ? rotationDifferenceDeg(dc.pose, baseline_pose) :
                             std::numeric_limits<double>::quiet_NaN());
  writePose7(output, dc.pose);
  output << ',';
  output << baseline_pose(0, 3) << ',' << baseline_pose(1, 3) << ','
         << baseline_pose(2, 3) << ',';
  const Eigen::Quaterniond baseline_q(baseline_pose.block<3, 3>(0, 0));
  output << baseline_q.x() << ',' << baseline_q.y() << ',' << baseline_q.z()
         << ',' << baseline_q.w();
  output << '\n';
}

struct HorizontalCovarianceProbe {
  bool valid = false;
  std::string failure_reason = "uninitialized";
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d eigenvalues = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d maximum_eigenvector = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  int position_state_index = -1;
};

HorizontalCovarianceProbe horizontalCovarianceProbe(
    const FilterSnapshot& predicted) {
  HorizontalCovarianceProbe result;
  result.position_state_index = MTK::getStartIdx(&state_ikfom::pos);
  if (result.position_state_index < 0 ||
      predicted.covariance.rows() < result.position_state_index + 2 ||
      predicted.covariance.cols() < result.position_state_index + 2) {
    result.failure_reason = "position_covariance_block_out_of_range";
    return result;
  }
  const Eigen::Matrix2d raw = predicted.covariance.block<2, 2>(
      result.position_state_index, result.position_state_index);
  if (!raw.allFinite()) {
    result.failure_reason = "position_covariance_nonfinite";
    return result;
  }
  const double scale = std::max(raw.cwiseAbs().maxCoeff(), 1.0);
  if ((raw - raw.transpose()).cwiseAbs().maxCoeff() > 1e-8 * scale) {
    result.failure_reason = "position_covariance_not_symmetric";
    return result;
  }
  result.covariance = 0.5 * (raw + raw.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(result.covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      !solver.eigenvectors().allFinite()) {
    result.failure_reason = "position_covariance_eigendecomposition_failed";
    return result;
  }
  result.eigenvalues = solver.eigenvalues();
  if (result.eigenvalues(0) < -1e-10 * scale) {
    result.failure_reason = "position_covariance_not_positive_semidefinite";
    return result;
  }
  result.maximum_eigenvector = solver.eigenvectors().col(1);
  const double vector_norm = result.maximum_eigenvector.norm();
  if (!std::isfinite(vector_norm) || vector_norm <= 1e-12) {
    result.failure_reason = "maximum_eigenvector_invalid";
    return result;
  }
  result.maximum_eigenvector /= vector_norm;
  result.valid = true;
  result.failure_reason = "OK";
  return result;
}

void runMode(const std::string& mode, const p4_i2::Inputs& inputs,
             const std::vector<ScanAsset>& assets,
             const std::vector<VisualMeasurement>& visual,
             const std::string& cloud_binary_path, const std::string& map_path,
             const std::string& params_path, const std::string& trajectory_path,
             const std::string& branch_path, const std::string& dcreg_path,
             const std::string& multistart_path, const std::string& candidates_path,
             const std::string& arbitration_path, const std::string& runtime_path,
             const std::string& basin_path = "",
             const std::string& covariance_path = "") {
  const bool use_dcreg = mode == "DCREG_ONLY" || mode == "FULL_ROUTER";
  const bool use_multistart = mode == "MULTISTART_OBJECTIVE" ||
      mode == "MULTISTART_VISUAL" || mode == "FULL_ROUTER" ||
      mode == "GEO7_REFERENCE";
  const bool use_visual_arbitration = mode == "MULTISTART_VISUAL" ||
      mode == "FULL_ROUTER";
  const bool use_covariance_probe = mode == "COV3_OBJECTIVE";
  const bool geometry_reference = mode == "GEO7_REFERENCE";
  if (!(mode == "DCREG_ONLY" || mode == "MULTISTART_OBJECTIVE" ||
        mode == "MULTISTART_VISUAL" || mode == "FULL_ROUTER" ||
        use_covariance_probe || geometry_reference))
    throw std::runtime_error("unsupported_p6_mode:" + mode);
  p5_i1::requireFrozenMapSha256(map_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("frozen_target_point_count_mismatch");
  Pose3d initial_map_T_lidar, T_imu_lidar_pose;
  const RuntimeParameters parameters = p4_i2::readParameters(
      params_path, &initial_map_T_lidar, &T_imu_lidar_pose);
  const Eigen::Matrix4d T_imu_lidar = poseMatrix(T_imu_lidar_pose);
  if (!use_covariance_probe && !geometry_reference)
    checkVisualSeedConvention(T_imu_lidar);
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend frontend(parameters);
  p4_i2::ImuVector initialization_imu(inputs.imu.begin(),
      inputs.imu.begin() + parameters.static_init_samples);
  std::string reason;
  if (!frontend.initializeStatic(initialization_imu, initial_map_T_lidar,
                                 T_imu_lidar_pose, &reason))
    throw std::runtime_error("mode_static_initialization_failed:" + reason);

  std::map<uint64_t, const VisualMeasurement*> visual_by_tx;
  for (const auto& measurement : visual) {
    if (measurement.transaction_cur > assets.size() ||
        measurement.transaction_cur != measurement.transaction_ref + 1)
      throw std::runtime_error("visual_transaction_out_of_scan_range");
    visual_by_tx[measurement.transaction_cur] = &measurement;
  }

  AuditedNdt ndt;
  configureNdt(ndt, target);
  std::ofstream trajectory(trajectory_path), branches(branch_path), dc_events(dcreg_path),
      multistart(multistart_path), candidate_output(candidates_path),
      arbitration(arbitration_path), runtime(runtime_path);
  if (!trajectory || !branches || !dc_events || !multistart ||
      !candidate_output || !arbitration || !runtime)
    throw std::runtime_error("cannot_create_mode_outputs");
  trajectory << std::setprecision(17);
  p4_i2::writeHeader(trajectory);
  branches << "mode,transaction_id,time_s,branch,visual_valid,baseline_objective,selected_objective,"
              "baseline_seed_index,selected_seed_index,selected_nonbaseline,step_limited\n";
  dc_events << "mode,transaction_id,time_s,attempted,converged,degenerate,mask_rx,mask_ry,mask_rz,mask_tx,mask_ty,mask_tz,"
               "lambda_rot_0,lambda_rot_1,lambda_rot_2,lambda_trans_0,lambda_trans_1,lambda_trans_2,"
               "cond_rot,cond_trans,runtime_ms,translation_from_baseline_m,rotation_from_baseline_deg,"
               "dcreg_pose_x,dcreg_pose_y,dcreg_pose_z,dcreg_pose_qx,dcreg_pose_qy,dcreg_pose_qz,dcreg_pose_qw,"
               "baseline_pose_x,baseline_pose_y,baseline_pose_z,baseline_pose_qx,baseline_pose_qy,baseline_pose_qz,baseline_pose_qw\n";
  multistart << "mode,transaction_id,time_s,visual_valid,candidate_count,converged_count,cluster_count,"
                "objective_gated_cluster_count,baseline_objective,best_objective,selected_objective,"
                "baseline_visual_residual,selected_visual_residual,visual_arbitration_used,"
                "selected_seed_index,baseline_choice_changed,translation_from_baseline_m,"
                "rotation_from_baseline_deg,max_inter_cluster_translation_m,"
                "max_inter_cluster_rotation_deg,objective_gap,visual_residual_spread_m,"
                "total_runtime_ms\n";
  candidate_output << "mode,transaction_id,time_s,seed_index,seed_name,converged,cluster_id,"
                     "objective_gate,objective,objective_per_source_point,"
                     "transformation_probability,fitness,iterations,runtime_ms,selected,"
                     "pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw\n";
  arbitration << "mode,transaction_id,time_s,cluster_id,seed_index,objective,visual_residual,"
                 "is_baseline_cluster,selected\n";
  runtime << "mode,transaction_id,time_s,prediction_ms,baseline_ndt_ms,dcreg_ms,"
              "multistart_ms,visual_arbitration_ms,ikfom_update_ms,total_ms\n";
  std::ofstream basin_events, covariance_directions;
  if (use_covariance_probe) {
    if (basin_path.empty() || covariance_path.empty())
      throw std::runtime_error("cov3_output_paths_required");
    basin_events.open(basin_path);
    covariance_directions.open(covariance_path);
    if (!basin_events || !covariance_directions)
      throw std::runtime_error("cannot_create_cov3_diagnostics");
    basin_events << "mode,transaction_id,time_s,covariance_valid,covariance_failure_reason,"
                    "ndt_calls,m0_converged,mplus_converged,mminus_converged,"
                    "m0_objective,mplus_objective,mminus_objective,selected_seed_index,"
                    "selected_objective,m0_cluster,selected_cluster,basin_escape,"
                    "objective_uplift_gb,delta_translation_m,delta_rotation_deg,"
                    "m0_x,m0_y,m0_z,m0_qx,m0_qy,m0_qz,m0_qw,"
                    "selected_x,selected_y,selected_z,selected_qx,selected_qy,selected_qz,selected_qw\n";
    covariance_directions << "mode,transaction_id,time_s,covariance_valid,fallback_to_m0,"
                              "failure_reason,p_xx,p_xy,p_yy,eigenvalue_min,eigenvalue_max,"
                              "eigen_gap,vmax_x,vmax_y,rho_m,plus_x,plus_y,minus_x,minus_y,"
                              "position_state_index\n";
  }

  Eigen::Matrix4d previous_used = Eigen::Matrix4d::Identity();
  bool has_previous_used = false;
  Eigen::Matrix4d previous_accepted_map_T_imu = poseMatrix(frontend.getState().map_T_imu);
  for (std::size_t index = 0; index < assets.size(); ++index) {
    const ScanAsset& asset = assets[index];
    const p4_i2::PoseRecord& saved = inputs.scans[index];
    if (saved.transaction_id != asset.transaction_id || saved.stamp_ns != asset.stamp_ns)
      throw std::runtime_error("filter_input_and_cloud_metadata_alignment_mismatch");
    const auto full_start = std::chrono::steady_clock::now();
    const auto prediction_start = std::chrono::steady_clock::now();
    const FilterSnapshot start = frontend.getState();
    p4_i2::ImuVector window = p4_i2::imuWindow(inputs.imu, start.stamp_ns, asset.stamp_ns);
    std::vector<ImuPoseSample, Eigen::aligned_allocator<ImuPoseSample>> imu_poses;
    if (!frontend.predictImuSequence(window, asset.stamp_ns, &imu_poses, &reason))
      throw std::runtime_error("mode_imu_prediction_failed_tx_" +
                               std::to_string(asset.transaction_id) + ":" + reason);
    const double prediction_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - prediction_start).count();
    const FilterSnapshot predicted = frontend.getState();
    const Eigen::Matrix4d predicted_map_T_lidar =
        (p4_i2::asIsometry(predicted.map_T_imu) *
         p4_i2::asIsometry(T_imu_lidar_pose)).matrix();
    Cloud::Ptr raw_source = loadRawCloudAt(cloud_binary_path, asset);
    const Cloud::Ptr source = preprocessSource(raw_source);
    const uint64_t source_hash = sourceCloudHash(source);
    if (!asset.expected_source_hash_available ||
        source_hash != asset.expected_source_hash)
      throw std::runtime_error("prepared_source_hash_mismatch_tx_" +
                               std::to_string(asset.transaction_id));
    const auto visual_it = visual_by_tx.find(asset.transaction_id);
    const VisualMeasurement* visual_measurement = visual_it == visual_by_tx.end()
        ? nullptr : visual_it->second;
    if (visual_measurement && visual_measurement->cur_ns != asset.stamp_ns &&
        visual_measurement->transaction_cur != asset.transaction_id)
      throw std::runtime_error("visual_pair_transaction_mismatch");

    const auto baseline = runNdtCandidate(ndt, source, predicted_map_T_lidar,
                                          0, seedName(0));
    if (!baseline.converged)
      throw std::runtime_error("baseline_candidate_not_converged_tx_" +
                               std::to_string(asset.transaction_id));
    const double baseline_ndt_ms = baseline.runtime_ms;
    double dcreg_ms = 0.0, multistart_ms = 0.0, visual_arbitration_ms = 0.0;
    DcregSummary dc;
    bool dc_branch = false;
    std::string branch = "BASELINE_NDT";
    Eigen::Matrix4d selected_pose = baseline.pose;
    int selected_seed_index = 0;
    bool visual_arbitration_used = false;
    double selected_objective = baseline.objective;
    double selected_visual_residual = std::numeric_limits<double>::quiet_NaN();
    if (use_dcreg) {
      dc = runDcreg(source, target, predicted_map_T_lidar);
      dcreg_ms = dc.runtime_ms;
      dc_branch = dc.degenerate && dc.converged;
      writeDcregRow(dc_events, mode, asset, dc, baseline.pose);
    }
    if (mode == "DCREG_ONLY") {
      if (dc_branch) {
        branch = "DCREG_DEGENERATE";
        selected_pose = dc.pose;
        selected_seed_index = -2;
        selected_objective = std::numeric_limits<double>::quiet_NaN();
      }
    } else if (mode == "FULL_ROUTER" && dc_branch) {
      branch = "DCREG_DEGENERATE";
      selected_pose = dc.pose;
      selected_seed_index = -2;
      selected_objective = std::numeric_limits<double>::quiet_NaN();
    } else if (use_covariance_probe) {
      constexpr double kProbeRadius = 0.8;
      const HorizontalCovarianceProbe probe = horizontalCovarianceProbe(predicted);
      std::vector<Candidate> candidates;
      candidates.reserve(probe.valid ? 3 : 1);
      candidates.push_back(baseline);
      Eigen::Matrix4d plus_seed = Eigen::Matrix4d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      Eigen::Matrix4d minus_seed = plus_seed;
      if (probe.valid) {
        plus_seed = predicted_map_T_lidar;
        minus_seed = predicted_map_T_lidar;
        plus_seed(0, 3) += kProbeRadius * probe.maximum_eigenvector.x();
        plus_seed(1, 3) += kProbeRadius * probe.maximum_eigenvector.y();
        minus_seed(0, 3) -= kProbeRadius * probe.maximum_eigenvector.x();
        minus_seed(1, 3) -= kProbeRadius * probe.maximum_eigenvector.y();
        candidates.push_back(runNdtCandidate(ndt, source, plus_seed, 1,
                                              "M_PLUS_VMAX"));
        candidates.push_back(runNdtCandidate(ndt, source, minus_seed, 2,
                                              "M_MINUS_VMAX"));
        multistart_ms = candidates[1].runtime_ms + candidates[2].runtime_ms;
      }
      const auto clusters = completeLinkClusters(&candidates);
      std::size_t selected_candidate = 0;
      for (std::size_t candidate_i = 1; candidate_i < candidates.size();
           ++candidate_i) {
        if (candidates[candidate_i].converged &&
            candidates[candidate_i].objective >
                candidates[selected_candidate].objective)
          selected_candidate = candidate_i;
      }
      for (std::size_t candidate_i = 0; candidate_i < candidates.size();
           ++candidate_i) {
        candidates[candidate_i].objective_gate = candidates[candidate_i].converged;
        writeCandidateRow(candidate_output, mode, asset, candidates[candidate_i],
                          candidate_i == selected_candidate);
      }
      const Candidate& best = candidates[selected_candidate];
      selected_pose = best.pose;
      selected_seed_index = best.seed_index;
      selected_objective = best.objective;
      branch = probe.valid ? "COV3_OBJECTIVE" :
                             "COV3_INVALID_COVARIANCE_FALLBACK_M0";
      const double denominator = std::max(
          {std::abs(best.objective), std::abs(baseline.objective), 1e-12});
      const double objective_uplift =
          (best.objective - baseline.objective) / denominator;
      const double delta_translation = (best.pose.block<3, 1>(0, 3) -
          baseline.pose.block<3, 1>(0, 3)).norm();
      const double delta_rotation = rotationDifferenceDeg(baseline.pose, best.pose);
      const double numerical_epsilon =
          1e-6 * std::max(std::abs(baseline.objective), 1.0);
      const bool same_cluster = best.cluster_id == candidates.front().cluster_id;
      const bool basin_escape = !same_cluster &&
          best.objective > baseline.objective + numerical_epsilon;
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const Eigen::Quaterniond m0_q(baseline.pose.block<3, 3>(0, 0));
      const Eigen::Quaterniond selected_q(best.pose.block<3, 3>(0, 0));
      basin_events << mode << ',' << asset.transaction_id << ',' << asset.time_s
                   << ',' << (probe.valid ? 1 : 0) << ',' << probe.failure_reason
                   << ',' << candidates.size() << ','
                   << (candidates[0].converged ? 1 : 0) << ','
                   << (probe.valid ? (candidates[1].converged ? 1 : 0) : -1)
                   << ',' << (probe.valid ? (candidates[2].converged ? 1 : 0) : -1)
                   << ',' << baseline.objective << ','
                   << (probe.valid ? candidates[1].objective : nan) << ','
                   << (probe.valid ? candidates[2].objective : nan) << ','
                   << best.seed_index << ',' << best.objective << ','
                   << candidates.front().cluster_id << ',' << best.cluster_id << ','
                   << (basin_escape ? 1 : 0) << ',' << objective_uplift << ','
                   << delta_translation << ',' << delta_rotation << ','
                   << baseline.pose(0, 3) << ',' << baseline.pose(1, 3) << ','
                   << baseline.pose(2, 3) << ',' << m0_q.x() << ',' << m0_q.y()
                   << ',' << m0_q.z() << ',' << m0_q.w() << ','
                   << best.pose(0, 3) << ',' << best.pose(1, 3) << ','
                   << best.pose(2, 3) << ',' << selected_q.x() << ','
                   << selected_q.y() << ',' << selected_q.z() << ','
                   << selected_q.w() << '\n';
      covariance_directions << mode << ',' << asset.transaction_id << ','
                            << asset.time_s << ',' << (probe.valid ? 1 : 0)
                            << ',' << (probe.valid ? 0 : 1) << ','
                            << probe.failure_reason << ','
                            << probe.covariance(0, 0) << ','
                            << probe.covariance(0, 1) << ','
                            << probe.covariance(1, 1) << ','
                            << probe.eigenvalues(0) << ','
                            << probe.eigenvalues(1) << ','
                            << probe.eigenvalues(1) - probe.eigenvalues(0) << ','
                            << probe.maximum_eigenvector.x() << ','
                            << probe.maximum_eigenvector.y() << ',' << kProbeRadius
                            << ',' << (probe.valid ? plus_seed(0, 3) : nan) << ','
                            << (probe.valid ? plus_seed(1, 3) : nan) << ','
                            << (probe.valid ? minus_seed(0, 3) : nan) << ','
                            << (probe.valid ? minus_seed(1, 3) : nan) << ','
                            << probe.position_state_index << '\n';
      const std::size_t converged_count = static_cast<std::size_t>(
          std::count_if(candidates.begin(), candidates.end(),
              [](const Candidate& candidate) { return candidate.converged; }));
      multistart << mode << ',' << asset.transaction_id << ',' << asset.time_s
                 << ",0," << candidates.size() << ',' << converged_count << ','
                 << clusters.size() << ',' << clusters.size() << ','
                 << baseline.objective << ',' << best.objective << ','
                 << best.objective << ",nan,nan,0," << best.seed_index << ','
                 << (best.seed_index != 0 ? 1 : 0) << ',' << delta_translation << ','
                 << delta_rotation << ",nan,nan," << best.objective -
                     baseline.objective << ",nan," << multistart_ms << '\n';
    } else if (use_multistart && (visual_measurement || geometry_reference)) {
      auto seeds = makeSeeds(predicted_map_T_lidar,
                             geometry_reference ? nullptr : visual_measurement,
                             previous_accepted_map_T_imu, T_imu_lidar);
      std::vector<Candidate> candidates;
      candidates.reserve(seeds.size());
      candidates.push_back(baseline);
      for (std::size_t seed_i = 1; seed_i < seeds.size(); ++seed_i) {
        candidates.push_back(runNdtCandidate(ndt, source, seeds[seed_i].second,
            seeds[seed_i].first, seedName(seeds[seed_i].first)));
        multistart_ms += candidates.back().runtime_ms;
      }
      const auto clusters = completeLinkClusters(&candidates);
      const auto gated = objectiveGatedRepresentatives(candidates, clusters,
                                                        baseline.objective);
      if (gated.empty()) throw std::runtime_error("no_objective_gated_candidate");
      for (Candidate& candidate : candidates) {
        candidate.objective_gate = candidate.converged &&
            candidate.objective >= baseline.objective -
                1e-6 * std::max(std::abs(baseline.objective), 1.0);
      }
      std::size_t selected_candidate = gated.front();
      if (use_visual_arbitration && gated.size() > 1) {
        visual_arbitration_used = true;
        const auto arbitration_start = std::chrono::steady_clock::now();
        for (const std::size_t candidate_index : gated) {
          candidates[candidate_index].visual_residual = candidateVisualResidual(
              candidates[candidate_index].pose, T_imu_lidar,
              previous_accepted_map_T_imu, visual_measurement->translation).norm();
        }
        const auto minimum_residual_candidate = std::min_element(
            gated.begin(), gated.end(), [&](std::size_t a, std::size_t b) {
              return candidates[a].visual_residual < candidates[b].visual_residual;
            });
        const double minimum_visual_residual =
            candidates[*minimum_residual_candidate].visual_residual;
        std::vector<std::size_t> visual_tied;
        for (const std::size_t candidate_index : gated) {
          if (candidates[candidate_index].visual_residual - minimum_visual_residual < 1e-3)
            visual_tied.push_back(candidate_index);
        }
        if (visual_tied.empty()) throw std::runtime_error("empty_visual_tie_set");
        selected_candidate = gated.front();
        selected_candidate = visual_tied.front();
        for (std::size_t i = 1; i < visual_tied.size(); ++i) {
          const std::size_t candidate_index = visual_tied[i];
          const Candidate& candidate = candidates[candidate_index];
          const Candidate& selected = candidates[selected_candidate];
          if (candidate.objective > selected.objective ||
              (candidate.objective == selected.objective &&
               candidate.seed_index < selected.seed_index))
            selected_candidate = candidate_index;
        }
        visual_arbitration_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - arbitration_start).count();
      } else {
        if (use_visual_arbitration && gated.size() <= 1) {
          // No competing transform mode: visual evidence is deliberately inert.
          selected_candidate = 0;
        } else {
          selected_candidate = *std::max_element(gated.begin(), gated.end(),
              [&](std::size_t a, std::size_t b) {
                if (candidates[a].objective != candidates[b].objective)
                  return candidates[a].objective < candidates[b].objective;
                return candidates[a].seed_index > candidates[b].seed_index;
              });
        }
      }
      const Candidate& chosen = candidates[selected_candidate];
      selected_pose = chosen.pose;
      selected_seed_index = chosen.seed_index;
      selected_objective = chosen.objective;
      selected_visual_residual = chosen.visual_residual;
      branch = mode == "MULTISTART_OBJECTIVE" ? "MULTISTART_OBJECTIVE" :
          (geometry_reference ? "GEO7_REFERENCE" :
           (visual_arbitration_used ? "MULTISTART_VISUAL_ARBITRATION" :
                                     "MULTISTART_SINGLE_CLUSTER_BASELINE"));
      for (std::size_t candidate_i = 0; candidate_i < candidates.size(); ++candidate_i)
        writeCandidateRow(candidate_output, mode, asset, candidates[candidate_i],
                          candidate_i == selected_candidate);
      if (use_visual_arbitration) {
        for (const CandidateCluster& cluster : clusters) {
          const std::size_t rep = cluster.representative;
          if (!candidates[rep].objective_gate) continue;
          const bool contains_baseline = std::any_of(cluster.members.begin(),
              cluster.members.end(), [&](std::size_t member) {
                return candidates[member].seed_index == 0;
              });
          arbitration << mode << ',' << asset.transaction_id << ',' << asset.time_s
                      << ',' << candidates[rep].cluster_id << ','
                      << candidates[rep].seed_index << ',' << candidates[rep].objective
                      << ',' << candidateVisualResidual(candidates[rep].pose,
                          T_imu_lidar, previous_accepted_map_T_imu,
                          visual_measurement->translation).norm()
                      << ',' << (contains_baseline ? 1 : 0) << ','
                      << (rep == selected_candidate ? 1 : 0) << '\n';
        }
      }
      std::size_t converged_count = 0;
      double best_objective = -std::numeric_limits<double>::infinity();
      double baseline_residual = visual_measurement ? candidateVisualResidual(
          baseline.pose, T_imu_lidar, previous_accepted_map_T_imu,
          visual_measurement->translation).norm() :
          std::numeric_limits<double>::quiet_NaN();
      for (const Candidate& candidate : candidates) {
        converged_count += candidate.converged ? 1 : 0;
        if (candidate.converged) best_objective = std::max(best_objective,
                                                           candidate.objective);
      }
      double max_inter_t = 0.0, max_inter_r = 0.0;
      std::vector<double> gated_visual_residuals;
      std::vector<double> cluster_objectives;
      std::size_t highest_objective_mode = 0;
      for (std::size_t i = 0; i < clusters.size(); ++i) {
        const std::size_t a = clusters[i].representative;
        cluster_objectives.push_back(candidates[a].objective);
        if (candidates[a].objective >
            candidates[clusters[highest_objective_mode].representative].objective)
          highest_objective_mode = i;
        if (visual_measurement && candidates[a].objective_gate)
          gated_visual_residuals.push_back(candidateVisualResidual(
              candidates[a].pose, T_imu_lidar, previous_accepted_map_T_imu,
              visual_measurement->translation).norm());
      }
      if (!clusters.empty()) {
        const Eigen::Matrix4d& reference_pose =
            candidates[clusters[highest_objective_mode].representative].pose;
        for (std::size_t i = 0; i < clusters.size(); ++i) {
          if (i == highest_objective_mode) continue;
          const Eigen::Matrix4d& pose = candidates[clusters[i].representative].pose;
          max_inter_t = std::max(max_inter_t,
              (pose.block<3, 1>(0, 3) -
               reference_pose.block<3, 1>(0, 3)).norm());
          max_inter_r = std::max(max_inter_r,
              rotationDifferenceDeg(pose, reference_pose));
        }
      }
      std::sort(cluster_objectives.begin(), cluster_objectives.end(),
                std::greater<double>());
      const double objective_gap = cluster_objectives.size() > 1
          ? cluster_objectives[0] - cluster_objectives[1]
          : std::numeric_limits<double>::quiet_NaN();
      double visual_residual_spread = std::numeric_limits<double>::quiet_NaN();
      if (gated_visual_residuals.size() > 1) {
        const auto bounds = std::minmax_element(gated_visual_residuals.begin(),
                                                gated_visual_residuals.end());
        visual_residual_spread = *bounds.second - *bounds.first;
      }
      multistart << mode << ',' << asset.transaction_id << ',' << asset.time_s << ','
                 << (visual_measurement ? 1 : 0) << ','
                 << candidates.size() << ',' << converged_count << ',' << clusters.size()
                 << ',' << gated.size() << ',' << baseline.objective << ','
                 << best_objective << ',' << selected_objective << ','
                 << baseline_residual << ',' << selected_visual_residual << ','
                 << (visual_arbitration_used ? 1 : 0) << ',' << selected_seed_index
                 << ',' << (selected_seed_index != 0 ? 1 : 0) << ','
                 << (selected_pose.block<3, 1>(0, 3) -
                     baseline.pose.block<3, 1>(0, 3)).norm() << ','
                 << rotationDifferenceDeg(selected_pose, baseline.pose) << ','
                 << max_inter_t << ',' << max_inter_r << ',' << objective_gap << ','
                 << visual_residual_spread << ','
                 << multistart_ms << '\n';
    } else if (use_multistart && !visual_measurement && !geometry_reference) {
      branch = "BASELINE_NO_VISUAL";
    }

    bool step_limited = false;
    const Eigen::Matrix4d used_pose = limitStep(selected_pose, previous_used,
                                                has_previous_used, &step_limited);
    previous_used = used_pose;
    has_previous_used = true;
    Pose3d used_pose_lidar = poseFromMatrix(used_pose);
    const Pose3d measurement_imu = p4_i2::lidarMeasurementToImu(
        used_pose_lidar, T_imu_lidar_pose);
    PoseCorrectionDelta update_delta;
    const auto update_start = std::chrono::steady_clock::now();
    if (!frontend.applyPoseMeasurement(measurement_imu, &update_delta, &reason))
      throw std::runtime_error("mode_pose_update_failed_tx_" +
          std::to_string(asset.transaction_id) + ":" + reason);
    const double update_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - update_start).count();
    const FilterSnapshot corrected = frontend.getState();
    if (corrected.stamp_ns != asset.stamp_ns)
      throw std::runtime_error("mode_corrected_timestamp_mismatch");
    p4_i2::writeRow(trajectory, index, saved, predicted.map_T_imu,
                    predicted, corrected, T_imu_lidar_pose);
    previous_accepted_map_T_imu = poseMatrix(corrected.map_T_imu);
    branches << mode << ',' << asset.transaction_id << ',' << asset.time_s << ','
             << branch << ',' << (visual_measurement ? 1 : 0) << ','
             << baseline.objective << ',' << selected_objective << ",0,"
             << selected_seed_index << ',' << (selected_seed_index != 0 ? 1 : 0)
             << ',' << (step_limited ? 1 : 0) << '\n';
    if (!use_dcreg) writeDcregRow(dc_events, mode, asset, dc, baseline.pose);
    runtime << mode << ',' << asset.transaction_id << ',' << asset.time_s << ','
            << prediction_ms << ',' << baseline_ndt_ms << ',' << dcreg_ms << ','
            << multistart_ms << ',' << visual_arbitration_ms << ',' << update_ms << ','
            << std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - full_start).count() << '\n';
    if ((index + 1) % 100 == 0 || index + 1 == assets.size()) {
      trajectory.flush(); branches.flush(); dc_events.flush(); multistart.flush();
      candidate_output.flush(); arbitration.flush(); runtime.flush();
      if (use_covariance_probe) {
        basin_events.flush();
        covariance_directions.flush();
      }
      std::cerr << "P6_MODE_PROGRESS mode=" << mode << " frames=" << index + 1
                << '/' << assets.size() << " branch=" << branch << '\n';
    }
  }
  std::cout << "P6_MODE_COMPLETE mode=" << mode << " frames=" << assets.size() << '\n';
}

}  // namespace p6_i1

int main(int argc, char** argv) {
  try {
    if (((argc >= 11 && argc <= 15) ||
         (argc == 16 && std::string(argv[1]) == "FULL_ALGORITHM_V1")) &&
        (std::string(argv[1]) == "STRICT_BASELINE" ||
         std::string(argv[1]) == "UOBS_ONLY" ||
         std::string(argv[1]) == "UNONLOCAL_ONLY" ||
         std::string(argv[1]) == "DUAL_RELIABILITY" ||
         std::string(argv[1]) == "FULL_ALGORITHM_V1")) {
      const bool full_algorithm = std::string(argv[1]) == "FULL_ALGORITHM_V1";
      if (full_algorithm && argc != 15 && argc != 16)
        throw std::runtime_error("FULL_ALGORITHM_V1_requires_visual_csv_and_replay_arguments");
      p4_i2::Inputs inputs;
      std::string reason;
      if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason))
        throw std::runtime_error("filter_input_load_failed:" + reason);
      const auto assets = p6_i1::readScanAssets(argv[4]);
      if (assets.size() != inputs.scans.size())
        throw std::runtime_error("scan_asset_and_filter_counts_differ");
      std::vector<p6_i1::VisualMeasurement> visual;
      if (full_algorithm) visual = p6_i1::readI6dVisual(argv[11]);
      std::vector<p6_i1::ScanAsset> selected_assets = assets;
      const int frame_limit_arg = full_algorithm ? 12 : 11;
      if ((full_algorithm && argc >= 13) || (!full_algorithm && argc >= 12)) {
        const long long requested = std::stoll(argv[frame_limit_arg]);
        if (requested <= 0 || static_cast<std::size_t>(requested) > assets.size())
          throw std::runtime_error("requested_frame_limit_out_of_range");
        selected_assets.resize(static_cast<std::size_t>(requested));
        inputs.scans.resize(static_cast<std::size_t>(requested));
      }
      uint64_t initialization_stamp_ns = 0;
      std::string map_profile = "floor01";
      if (full_algorithm) {
        if (argc >= 14) initialization_stamp_ns = std::stoull(argv[13]);
        if (argc >= 15) map_profile = argv[14];
      } else {
        if (argc >= 13) initialization_stamp_ns = std::stoull(argv[12]);
        if (argc >= 14) map_profile = argv[13];
      }
      p6_i1::runDualReliabilityMode(argv[1], inputs, selected_assets, argv[5], argv[6],
                                    argv[7], argv[8], argv[9], argv[10],
                                    initialization_stamp_ns, map_profile,
                                    full_algorithm ? &visual : nullptr, 0.05,
                                    full_algorithm && argc == 16 ? p6_i1::parseR2Policy(argv[15])
                                        : p6_i1::R2Policy::LEGACY_BASE_NO_GATE);
      return 0;
    }
    if (argc == 10 && std::string(argv[1]) == "baseline") {
      p4_i2::Inputs inputs;
      std::string reason;
      if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason))
        throw std::runtime_error("filter_input_load_failed:" + reason);
      const auto assets = p6_i1::readScanAssets(argv[4]);
      if (assets.size() != inputs.scans.size())
        throw std::runtime_error("scan_asset_and_filter_counts_differ");
      p6_i1::runBaseline(inputs, assets, argv[5], argv[6], argv[7], argv[8], argv[9]);
      return 0;
    }
    if (argc == 10 && std::string(argv[1]) == "strict_single_start") {
      p4_i2::Inputs inputs;
      std::string reason;
      if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason))
        throw std::runtime_error("filter_input_load_failed:" + reason);
      const auto assets = p6_i1::readScanAssets(argv[4]);
      if (assets.size() != inputs.scans.size())
        throw std::runtime_error("scan_asset_and_filter_counts_differ");
      p6_i1::runBaseline(inputs, assets, argv[5], argv[6], argv[7], argv[8], argv[9], true);
      return 0;
    }
    if ((argc == 16 || argc == 18) && std::string(argv[1]) != "baseline") {
      p4_i2::Inputs inputs;
      std::string reason;
      if (!p4_i2::readInputs(argv[2], argv[3], &inputs, &reason))
        throw std::runtime_error("filter_input_load_failed:" + reason);
      const auto assets = p6_i1::readScanAssets(argv[4]);
      if (assets.size() != inputs.scans.size())
        throw std::runtime_error("scan_asset_and_filter_counts_differ");
      const std::string mode = argv[1];
      if (argc == 18 && mode != "COV3_OBJECTIVE")
        throw std::runtime_error("extended_output_arguments_only_valid_for_COV3_OBJECTIVE");
      const auto visual = (mode == "COV3_OBJECTIVE" || mode == "GEO7_REFERENCE")
          ? std::vector<p6_i1::VisualMeasurement>{} : p6_i1::readVisual(argv[8]);
      p6_i1::runMode(argv[1], inputs, assets, visual, argv[5], argv[6], argv[7],
                     argv[9], argv[10], argv[11], argv[12], argv[13], argv[14],
                     argv[15], argc == 18 ? argv[16] : "",
                     argc == 18 ? argv[17] : "");
      return 0;
    }
    std::cerr << "usage:\n"
              << "  p6_i1_branched_recovery baseline imu.csv filter_scans.csv scans.csv xyz.bin map.pcd params.txt baseline_replay.csv baseline_trajectory.csv\n"
              << "  p6_i1_branched_recovery strict_single_start imu.csv filter_scans.csv scans.csv xyz.bin map.pcd params.txt strict_replay.csv strict_trajectory.csv\n"
              << "  p6_i1_branched_recovery (STRICT_BASELINE|UOBS_ONLY|UNONLOCAL_ONLY|DUAL_RELIABILITY) imu.csv filter_scans.csv scans.csv xyz.bin map.pcd params.txt trajectory.csv reliability.csv runtime.csv [frame_limit [init_stamp_ns [floor01|corridor01]]]\n"
              << "  p6_i1_branched_recovery FULL_ALGORITHM_V1 imu.csv filter_scans.csv scans.csv xyz.bin map.pcd params.txt trajectory.csv reliability.csv runtime.csv visual.csv frame_limit init_stamp_ns floor01|corridor01 [r2_policy]\n"
              << "  p6_i1_branched_recovery MODE imu.csv filter_scans.csv scans.csv xyz.bin map.pcd params.txt visual.csv trajectory.csv branch.csv dcreg.csv multistart.csv candidates.csv arbitration.csv runtime.csv [basin.csv covariance.csv]\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "P6_I1_RUNNER_FAILED: " << error.what() << '\n';
    return 1;
  }
}
