// Offline-only P6-I4 prior-conditioned basin-margin experiment.
// Validated P4-I2 FAST-LIO2/IKFoM and P5/P6 NDT helpers are reused without
// changing the ROS runtime or the frozen P6-I3 implementation.
#define main p4_i2_unused_entry_point
#include "p4_i2_state_contamination_replay.cpp"
#undef main

#define main p5_i1_unused_entry_point
#include "p5_i1_ndt_mode_landscape.cpp"
#undef main

#include "p6_i4_basin_margin_math.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <use-ikfom.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace p6_i4_app {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
using AuditedNdt = ::AuditedNdt;
using p4_i2::FilterSnapshot;
using p4_i2::Inputs;
using p4_i2::Pose3d;
using p4_i2::RuntimeParameters;
using p4_i2::ImuVector;
using dog_prior_map_fastlio2_frontend_exp::FastLio2IkfomFrontend;
using p6_i4::Matrix6d;
using p6_i4::Pose;
using p6_i4::PoseCovarianceSpectrum;
using p6_i4::Vector6d;

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, std::size_t> columns;

  const std::string& get(const std::vector<std::string>& row,
                         const std::string& name) const {
    const auto found = columns.find(name);
    if (found == columns.end() || found->second >= row.size())
      throw std::runtime_error("missing_csv_column:" + name);
    return row[found->second];
  }
};

struct ScanAsset {
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  double time_s = 0.0;
  uint64_t cloud_byte_offset = 0;
  uint64_t cloud_point_count = 0;
  uint64_t expected_source_hash = 0;
  uint64_t request_cloud_hash = 0;
  Pose saved_raw = Pose::Identity();
  Pose saved_used = Pose::Identity();
};

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_csv:" + path);
  CsvTable table;
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty_csv:" + path);
  table.header = splitCsv(line);
  for (std::size_t i = 0; i < table.header.size(); ++i)
    table.columns[table.header[i]] = i;
  while (std::getline(input, line)) {
    if (!line.empty()) table.rows.push_back(splitCsv(line));
  }
  return table;
}

double parseDouble(const CsvTable& table, const std::vector<std::string>& row,
                   const std::string& name) {
  const double value = std::stod(table.get(row, name));
  if (!std::isfinite(value)) throw std::runtime_error("nonfinite_csv_value:" + name);
  return value;
}

uint64_t parseU64(const CsvTable& table, const std::vector<std::string>& row,
                  const std::string& name) {
  return std::stoull(table.get(row, name));
}

std::string number(double value) {
  if (!std::isfinite(value)) return "nan";
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

template <typename Derived>
std::string flatten(const Eigen::MatrixBase<Derived>& matrix) {
  std::ostringstream stream;
  stream << std::setprecision(17);
  bool first = true;
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int column = 0; column < matrix.cols(); ++column) {
      if (!first) stream << ';';
      first = false;
      stream << matrix(row, column);
    }
  }
  return stream.str();
}

Pose poseMatrix(const Pose3d& pose) {
  Pose result = Pose::Identity();
  result.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
  result.block<3, 1>(0, 3) = pose.position;
  return result;
}

Pose poseFromPcl(const Eigen::Matrix4f& pose) { return pose.cast<double>(); }

std::string pose7(const Pose& pose) {
  const Eigen::Quaterniond quaternion(pose.block<3, 3>(0, 0));
  std::ostringstream stream;
  stream << std::setprecision(17) << pose(0, 3) << ';' << pose(1, 3) << ';'
         << pose(2, 3) << ';' << quaternion.x() << ';' << quaternion.y() << ';'
         << quaternion.z() << ';' << quaternion.w();
  return stream.str();
}

double rotationDifferenceDeg(const Pose& a, const Pose& b) {
  const Eigen::Matrix3d delta = a.block<3, 3>(0, 0).transpose() *
                                b.block<3, 3>(0, 0);
  const double cosine = std::clamp((delta.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine) * 180.0 / M_PI;
}

Pose parsePose7(const std::string& value) {
  const std::vector<std::string> fields = [&value]() {
    std::vector<std::string> parts;
    std::stringstream stream(value);
    std::string part;
    while (std::getline(stream, part, ';')) parts.push_back(part);
    return parts;
  }();
  if (fields.size() != 7) throw std::runtime_error("invalid_pose7_field");
  const Eigen::Vector3d position(std::stod(fields[0]), std::stod(fields[1]),
                                 std::stod(fields[2]));
  Eigen::Quaterniond quaternion(std::stod(fields[6]), std::stod(fields[3]),
                                std::stod(fields[4]), std::stod(fields[5]));
  if (!position.allFinite() || !quaternion.coeffs().allFinite() ||
      quaternion.norm() < 1e-12)
    throw std::runtime_error("invalid_pose7_value");
  quaternion.normalize();
  Pose result = Pose::Identity();
  result.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  result.block<3, 1>(0, 3) = position;
  return result;
}

std::vector<uint64_t> readTransactions(const std::string& manifest_path) {
  const CsvTable manifest = readCsv(manifest_path);
  std::vector<uint64_t> transactions;
  for (const auto& row : manifest.rows)
    transactions.push_back(parseU64(manifest, row, "transaction_id"));
  if (transactions.empty()) throw std::runtime_error("empty_frame_manifest");
  return transactions;
}

std::vector<ScanAsset> readScanAssets(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<ScanAsset> assets;
  assets.reserve(table.rows.size());
  uint64_t expected_transaction = 1;
  for (const auto& row : table.rows) {
    ScanAsset asset;
    asset.transaction_id = parseU64(table, row, "transaction_id");
    asset.stamp_ns = parseU64(table, row, "stamp_ns");
    asset.time_s = parseDouble(table, row, "time_s");
    asset.cloud_byte_offset = parseU64(table, row, "cloud_byte_offset");
    asset.cloud_point_count = parseU64(table, row, "cloud_point_count");
    asset.expected_source_hash = parseU64(table, row, "ndt_source_cloud_hash");
    asset.request_cloud_hash = parseU64(table, row, "request_cloud_hash");
    asset.saved_raw = parsePose7(table.get(row, "raw_x") + ";" +
        table.get(row, "raw_y") + ";" + table.get(row, "raw_z") + ";" +
        table.get(row, "raw_qx") + ";" + table.get(row, "raw_qy") + ";" +
        table.get(row, "raw_qz") + ";" + table.get(row, "raw_qw"));
    asset.saved_used = parsePose7(table.get(row, "used_x") + ";" +
        table.get(row, "used_y") + ";" + table.get(row, "used_z") + ";" +
        table.get(row, "used_qx") + ";" + table.get(row, "used_qy") + ";" +
        table.get(row, "used_qz") + ";" + table.get(row, "used_qw"));
    if (asset.transaction_id != expected_transaction || asset.stamp_ns == 0 ||
        asset.cloud_point_count == 0)
      throw std::runtime_error("invalid_scan_asset_sequence");
    ++expected_transaction;
    assets.push_back(asset);
  }
  if (assets.size() != 4127) throw std::runtime_error("scan_metadata_count_not_4127");
  return assets;
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

Pose limitStep(const Pose& raw, const Pose& previous, bool has_previous,
               bool* limited) {
  *limited = false;
  Pose used = raw;
  if (!has_previous) return used;
  constexpr double kMaxTranslation = 0.5;
  constexpr double kMaxRotationDeg = 5.0;
  const Eigen::Vector3d previous_p = previous.block<3, 1>(0, 3);
  const Eigen::Vector3d delta_p = raw.block<3, 1>(0, 3) - previous_p;
  if (delta_p.norm() > kMaxTranslation) {
    used.block<3, 1>(0, 3) = previous_p + delta_p.normalized() * kMaxTranslation;
    *limited = true;
  }
  const Eigen::Matrix3d previous_r = previous.block<3, 3>(0, 0);
  const Eigen::Matrix3d raw_r = raw.block<3, 3>(0, 0);
  const Eigen::AngleAxisd delta(previous_r.transpose() * raw_r);
  const double angle_deg = std::abs(delta.angle()) * 180.0 / M_PI;
  if (angle_deg > kMaxRotationDeg) {
    const double fraction = kMaxRotationDeg / std::max(angle_deg, 1e-12);
    used.block<3, 3>(0, 0) = Eigen::Quaterniond(previous_r).normalized()
        .slerp(fraction, Eigen::Quaterniond(raw_r).normalized()).toRotationMatrix();
    *limited = true;
  }
  return used;
}

Pose3d statePose3d(const FilterSnapshot& snapshot) {
  Pose3d pose;
  pose.position = snapshot.map_T_imu.position;
  pose.orientation = snapshot.map_T_imu.orientation.normalized();
  return pose;
}

state_ikfom reconstructIkfomState(const FilterSnapshot& snapshot) {
  state_ikfom state;
  state.pos = vect3(snapshot.map_T_imu.position);
  state.rot = SO3(snapshot.map_T_imu.orientation.toRotationMatrix());
  state.offset_R_L_I = SO3(snapshot.T_imu_lidar_rotation);
  state.offset_T_L_I = vect3(snapshot.T_imu_lidar_translation);
  state.vel = vect3(snapshot.velocity);
  state.bg = vect3(snapshot.gyro_bias);
  state.ba = vect3(snapshot.accel_bias);
  state.grav = S2(snapshot.gravity);
  return state;
}

Eigen::MatrixXd poseJacobianForState(const FilterSnapshot& snapshot) {
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  return p6_i4::buildPoseErrorJacobian(
      snapshot.map_T_imu.orientation.toRotationMatrix(), state_ikfom::DOF,
      position_index, rotation_index);
}

struct CovarianceCapture {
  Eigen::MatrixXd jacobian;
  Matrix6d pose_covariance = Matrix6d::Zero();
  PoseCovarianceSpectrum spectrum;
};

CovarianceCapture capturePoseCovariance(const FilterSnapshot& snapshot) {
  if (snapshot.covariance.rows() != state_ikfom::DOF ||
      snapshot.covariance.cols() != state_ikfom::DOF ||
      !snapshot.covariance.allFinite())
    throw std::runtime_error("prediction_state_covariance_shape_or_finite_failure");
  CovarianceCapture capture;
  capture.jacobian = poseJacobianForState(snapshot);
  capture.pose_covariance = capture.jacobian * snapshot.covariance *
                            capture.jacobian.transpose();
  capture.pose_covariance = 0.5 *
      (capture.pose_covariance + capture.pose_covariance.transpose());
  capture.spectrum = p6_i4::analyzePoseCovariance(capture.pose_covariance);
  return capture;
}

struct FdColumnResult {
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  int state_index = -1;
  std::string component;
  double epsilon = 1e-7;
  double max_abs_error = std::numeric_limits<double>::quiet_NaN();
  double column_norm_error = std::numeric_limits<double>::quiet_NaN();
  bool pass = false;
};

struct PredictionContext {
  std::string frame_id;
  uint64_t transaction_id = 0;
  uint64_t stamp_ns = 0;
  double time_s = 0.0;
  Pose T_minus_map_T_imu = Pose::Identity();
  Pose nominal_map_T_lidar = Pose::Identity();
  Pose T_imu_lidar = Pose::Identity();
  PoseCovarianceSpectrum covariance;
};

struct RegistrationResult {
  bool converged = false;
  Pose terminal_pose = Pose::Identity();
  double objective = std::numeric_limits<double>::quiet_NaN();
  double fitness = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  double runtime_ms = 0.0;
};

struct RayBoundaryResult {
  std::string ray_type;
  int ray_id = -1;
  int sign = 0;
  bool boundary_found = false;
  bool censored = false;
  double alpha_same = 0.0;
  double alpha_diff = std::numeric_limits<double>::quiet_NaN();
  double margin_alpha = 3.0;
  RegistrationResult terminal_at_boundary;
  p6_i4::ModeComparison terminal_comparison;
};

std::vector<FdColumnResult> validatePoseJacobian(
    const FilterSnapshot& snapshot, uint64_t transaction_id,
    const CovarianceCapture& capture) {
  constexpr double kEpsilon = 1e-7;
  const int position_index = MTK::getStartIdx(&state_ikfom::pos);
  const int rotation_index = MTK::getStartIdx(&state_ikfom::rot);
  const state_ikfom nominal = reconstructIkfomState(snapshot);
  const Eigen::Matrix3d nominal_rotation = snapshot.map_T_imu.orientation.toRotationMatrix();
  const Eigen::Vector3d nominal_position = snapshot.map_T_imu.position;
  std::vector<FdColumnResult> output;
  for (int component = 0; component < 6; ++component) {
    const int state_index = component < 3 ? position_index + component :
                                            rotation_index + component - 3;
    Eigen::Matrix<double, state_ikfom::DOF, 1> tangent =
        Eigen::Matrix<double, state_ikfom::DOF, 1>::Zero();
    tangent(state_index) = kEpsilon;
    state_ikfom perturbed = nominal;
    perturbed.boxplus(tangent);
    const Eigen::Matrix3d perturbed_rotation = perturbed.rot.toRotationMatrix();
    const Eigen::Vector3d perturbed_position = perturbed.pos;
    Vector6d finite_difference;
    finite_difference.head<3>() = p6_i4::logSO3(
        perturbed_rotation * nominal_rotation.transpose()) / kEpsilon;
    finite_difference.tail<3>() =
        (perturbed_position - nominal_position) / kEpsilon;
    const Vector6d analytic = capture.jacobian.col(state_index);
    FdColumnResult result;
    result.transaction_id = transaction_id;
    result.stamp_ns = snapshot.stamp_ns;
    result.state_index = state_index;
    result.component = component < 3 ? "POSITION" : "ROTATION";
    result.max_abs_error = (finite_difference - analytic).cwiseAbs().maxCoeff();
    result.column_norm_error = (finite_difference - analytic).norm();
    result.pass = std::isfinite(result.max_abs_error) && result.max_abs_error <= 1e-5;
    output.push_back(result);
  }
  return output;
}

void writePoseCovarianceContext(
    std::ostream& output, uint64_t transaction_id, double time_s,
    const FilterSnapshot& snapshot, const Pose& nominal_ndt_lidar,
    const CovarianceCapture& covariance) {
  output << transaction_id << ',' << snapshot.stamp_ns << ',' << number(time_s)
         << ',' << pose7(poseMatrix(snapshot.map_T_imu))
         << ',' << flatten(snapshot.covariance)
         << ',' << flatten(covariance.jacobian)
         << ',' << flatten(covariance.pose_covariance)
         << ',' << flatten(covariance.spectrum.eigenvalues)
         << ',' << covariance.spectrum.effective_rank
         << ',' << covariance.spectrum.valid
         << ',' << covariance.spectrum.reason
         << ',' << pose7(nominal_ndt_lidar)
         << ',' << snapshot.velocity.transpose().format(Eigen::IOFormat(
                Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << ',' << snapshot.gyro_bias.transpose().format(Eigen::IOFormat(
                Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << ',' << snapshot.accel_bias.transpose().format(Eigen::IOFormat(
                Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << ',' << snapshot.gravity.transpose().format(Eigen::IOFormat(
                Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << ',' << snapshot.T_imu_lidar_translation.transpose().format(Eigen::IOFormat(
                Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << ',' << Eigen::Quaterniond(snapshot.T_imu_lidar_rotation).coeffs().transpose().format(
                Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ";", ";"))
         << '\n';
}

Eigen::VectorXd parseSemicolonVector(const std::string& text, int expected) {
  Eigen::VectorXd values(expected);
  std::stringstream stream(text);
  std::string field;
  int index = 0;
  while (std::getline(stream, field, ';')) {
    if (index >= expected) throw std::runtime_error("too_many_semicolon_values");
    values(index++) = std::stod(field);
  }
  if (index != expected || !values.allFinite())
    throw std::runtime_error("invalid_semicolon_vector");
  return values;
}

Eigen::Matrix<double, 6, 6> parseMatrix6(const std::string& text) {
  const Eigen::VectorXd values = parseSemicolonVector(text, 36);
  Eigen::Matrix<double, 6, 6> result;
  for (int row = 0; row < 6; ++row)
    for (int column = 0; column < 6; ++column)
      result(row, column) = values(row * 6 + column);
  return result;
}

std::vector<PredictionContext> readPredictionContexts(
    const std::string& path, const std::string& broad_manifest_path) {
  const CsvTable table = readCsv(path);
  const CsvTable manifest = readCsv(broad_manifest_path);
  std::map<uint64_t, std::string> frame_ids;
  for (const auto& row : manifest.rows)
    frame_ids.emplace(parseU64(manifest, row, "transaction_id"),
                      manifest.get(row, "frame_id"));
  std::vector<PredictionContext> contexts;
  contexts.reserve(table.rows.size());
  for (const auto& row : table.rows) {
    PredictionContext context;
    context.transaction_id = parseU64(table, row, "transaction_id");
    context.stamp_ns = parseU64(table, row, "stamp_ns");
    context.time_s = parseDouble(table, row, "time_s");
    const auto frame = frame_ids.find(context.transaction_id);
    if (frame == frame_ids.end()) throw std::runtime_error("context_not_in_broad_cohort");
    context.frame_id = frame->second;
    context.T_minus_map_T_imu = parsePose7(table.get(row, "T_minus_map_T_imu_xyz_q_xyzw"));
    context.nominal_map_T_lidar = parsePose7(table.get(row, "M0_map_T_lidar_xyz_q_xyzw"));
    const Eigen::VectorXd ext_t = parseSemicolonVector(
        table.get(row, "T_imu_lidar_translation"), 3);
    const Eigen::VectorXd ext_q = parseSemicolonVector(
        table.get(row, "T_imu_lidar_rotation_xyzw"), 4);
    Eigen::Quaterniond quaternion(ext_q(3), ext_q(0), ext_q(1), ext_q(2));
    if (quaternion.norm() < 1e-12) throw std::runtime_error("invalid_extrinsic_rotation");
    quaternion.normalize();
    context.T_imu_lidar = Pose::Identity();
    context.T_imu_lidar.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    context.T_imu_lidar.block<3, 1>(0, 3) = ext_t;
    context.covariance = p6_i4::analyzePoseCovariance(
        parseMatrix6(table.get(row, "P_pose_map_row_major")));
    if (!context.covariance.valid ||
        table.get(row, "covariance_valid") != "1")
      throw std::runtime_error("invalid_pose_covariance_context_tx_" +
                               std::to_string(context.transaction_id));
    contexts.push_back(std::move(context));
  }
  if (contexts.size() != 88) throw std::runtime_error("prediction_context_count_not_88");
  return contexts;
}

struct ExtraDirection {
  std::string direction_id;
  Eigen::Matrix<double, 6, 1> vector = Eigen::Matrix<double, 6, 1>::Zero();
};

std::vector<ExtraDirection> readExtraDirections(const std::string& path) {
  const CsvTable table = readCsv(path);
  if (table.rows.size() != 32) throw std::runtime_error("extra_direction_count_not_32");
  std::vector<ExtraDirection> directions;
  directions.reserve(32);
  for (const auto& row : table.rows) {
    ExtraDirection direction;
    direction.direction_id = table.get(row, "direction_id");
    for (int i = 0; i < 6; ++i)
      direction.vector(i) = parseDouble(table, row, "u" + std::to_string(i));
    if (std::abs(direction.vector.norm() - 1.0) > 1e-12)
      throw std::runtime_error("reference_direction_not_unit");
    directions.push_back(std::move(direction));
  }
  return directions;
}

std::set<uint64_t> readDenseTransactions(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::set<uint64_t> result;
  for (const auto& row : table.rows)
    if (!result.insert(parseU64(table, row, "transaction_id")).second)
      throw std::runtime_error("duplicate_dense_transaction");
  if (result.size() != 24) throw std::runtime_error("dense_frame_count_not_24");
  return result;
}

Eigen::VectorXd normalizedTruncatedDirection(const ExtraDirection& direction, int rank) {
  if (rank <= 0 || rank > 6) throw std::runtime_error("unsupported_covariance_rank");
  Eigen::VectorXd result = direction.vector.head(rank);
  if (result.norm() <= 1e-12)
    throw std::runtime_error("truncated_reference_direction_is_zero");
  result.normalize();
  return result;
}

struct ProbeCsvWriter {
  std::ofstream& output;
  const PredictionContext& context;
  const Pose& nominal;

  void operator()(const std::string& ray_type, int ray_id, int sign,
                  double alpha, const Vector6d& delta, const Pose& seed,
                  const RegistrationResult& result,
                  const p6_i4::ModeComparison& comparison) const {
    output << context.frame_id << ',' << context.transaction_id << ','
        << number(context.time_s) << ',' << ray_type << ',' << ray_id << ','
        << sign << ',' << number(alpha) << ',' << number(
            p6_i4::priorMetricRadius(delta, context.covariance)) << ','
        << number(delta(0)) << ',' << number(delta(1)) << ',' << number(delta(2))
        << ',' << number(delta(3)) << ',' << number(delta(4)) << ','
        << number(delta(5)) << ',' << pose7(seed) << ',' << result.converged << ','
        << pose7(result.terminal_pose) << ',' << number(result.objective) << ','
        << number(result.fitness) << ',' << result.iterations << ','
        << comparison.same_mode << ','
        << number(comparison.translation_separation_m) << ','
        << number(comparison.rotation_separation_deg) << ','
        << number(result.runtime_ms) << '\n';
  }
};

class ProbeEngine {
 public:
  ProbeEngine(const PredictionContext& context, const Cloud::Ptr& target,
              const Cloud::Ptr& source, std::ofstream& probe_output)
      : context_(context), source_(source), logger_{probe_output, context,
          context.nominal_map_T_lidar} {
    ::configureNdt(ndt_, target);
    ndt_.setInputSource(source_);
  }

  void clearCache() { cache_.clear(); }
  uint64_t callCount() const { return call_count_; }

  std::pair<RegistrationResult, p6_i4::ModeComparison> run(
      const std::string& ray_type, int ray_id, int sign, double alpha,
      const Eigen::VectorXd& signed_unit_direction) {
    const Vector6d delta = p6_i4::whitenedPerturbation(
        context_.covariance, signed_unit_direction, alpha);
    const int64_t alpha_key = static_cast<int64_t>(std::llround(alpha * 1e6));
    const auto key = std::make_tuple(context_.transaction_id, ray_id, sign, alpha_key);
    const auto cached = cache_.find(key);
    if (cached != cache_.end()) {
      const auto comparison = p6_i4::compareOperationalMode(
          context_.nominal_map_T_lidar, true, cached->second.terminal_pose,
          cached->second.converged);
      return {cached->second, comparison};
    }

    const Pose seed_imu = p6_i4::boxplusMapPose(context_.T_minus_map_T_imu, delta);
    const Pose seed_lidar = seed_imu * context_.T_imu_lidar;
    Cloud aligned;
    const auto begin = std::chrono::steady_clock::now();
    ndt_.align(aligned, seed_lidar.cast<float>());
    ++call_count_;
    RegistrationResult result;
    result.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    result.converged = ndt_.hasConverged();
    result.terminal_pose = poseFromPcl(ndt_.getFinalTransformation());
    result.fitness = ndt_.getFitnessScore();
    result.iterations = ndt_.getFinalNumIteration();
    result.objective = ::fixedPclScore(ndt_, source_,
                                       result.terminal_pose.cast<float>());
    const auto comparison = p6_i4::compareOperationalMode(
        context_.nominal_map_T_lidar, true, result.terminal_pose,
        result.converged);
    logger_(ray_type, ray_id, sign, alpha, delta, seed_lidar, result, comparison);
    cache_.emplace(key, result);
    return {result, comparison};
  }

 private:
  const PredictionContext& context_;
  Cloud::Ptr source_;
  AuditedNdt ndt_;
  ProbeCsvWriter logger_;
  std::map<std::tuple<uint64_t, int, int, int64_t>, RegistrationResult> cache_;
  uint64_t call_count_ = 0;
};

RayBoundaryResult searchFirstBoundaryOnRay(
    ProbeEngine& engine, const std::string& ray_type, int ray_id, int sign,
    const Eigen::VectorXd& unit_direction) {
  RayBoundaryResult boundary;
  boundary.ray_type = ray_type;
  boundary.ray_id = ray_id;
  boundary.sign = sign;
  double previous_same = 0.0;
  for (int step = 1; step <= 12; ++step) {
    const double alpha = 0.25 * static_cast<double>(step);
    const auto probe = engine.run(ray_type, ray_id, sign, alpha, unit_direction);
    if (probe.second.same_mode) {
      previous_same = alpha;
      boundary.terminal_at_boundary = probe.first;
      boundary.terminal_comparison = probe.second;
      continue;
    }
    double alpha_same = previous_same;
    double alpha_diff = alpha;
    RegistrationResult terminal_diff = probe.first;
    p6_i4::ModeComparison terminal_comparison = probe.second;
    while (alpha_diff - alpha_same > 0.01) {
      const double midpoint = 0.5 * (alpha_same + alpha_diff);
      const auto mid = engine.run(ray_type, ray_id, sign, midpoint, unit_direction);
      if (mid.second.same_mode) {
        alpha_same = midpoint;
      } else {
        alpha_diff = midpoint;
        terminal_diff = mid.first;
        terminal_comparison = mid.second;
      }
    }
    boundary.boundary_found = true;
    boundary.alpha_same = alpha_same;
    boundary.alpha_diff = alpha_diff;
    boundary.margin_alpha = alpha_diff;
    boundary.terminal_at_boundary = terminal_diff;
    boundary.terminal_comparison = terminal_comparison;
    return boundary;
  }
  boundary.censored = true;
  boundary.alpha_same = 3.0;
  boundary.margin_alpha = 3.0;
  return boundary;
}

Eigen::VectorXd principalDirection(const PoseCovarianceSpectrum& covariance, int axis) {
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(covariance.effective_rank);
  direction(axis) = 1.0;
  return direction;
}

std::vector<RayBoundaryResult> estimatePrincipalRays(
    ProbeEngine& engine, const PredictionContext& context,
    const std::string& ray_type, int ray_id_offset = 0) {
  std::vector<RayBoundaryResult> result;
  for (int axis = 0; axis < context.covariance.effective_rank; ++axis) {
    const Eigen::VectorXd direction = principalDirection(context.covariance, axis);
    for (int sign : {-1, 1})
      result.push_back(searchFirstBoundaryOnRay(engine, ray_type,
          ray_id_offset + axis, sign, direction * static_cast<double>(sign)));
  }
  return result;
}

void writeRayBoundary(std::ostream& output, const PredictionContext& context,
                      const RayBoundaryResult& boundary) {
  output << context.frame_id << ',' << context.transaction_id << ','
      << number(context.time_s) << ',' << boundary.ray_type << ','
      << boundary.ray_id << ',' << boundary.sign << ','
      << boundary.boundary_found << ',' << boundary.censored << ','
      << number(boundary.alpha_same) << ',' << number(boundary.alpha_diff) << ','
      << number(boundary.margin_alpha) << ','
      << pose7(boundary.terminal_at_boundary.terminal_pose) << ','
      << boundary.terminal_at_boundary.converged << ','
      << number(boundary.terminal_at_boundary.objective) << ','
      << number(boundary.terminal_at_boundary.fitness) << ','
      << boundary.terminal_at_boundary.iterations << ','
      << number(boundary.terminal_at_boundary.runtime_ms) << ','
      << number(boundary.terminal_comparison.translation_separation_m) << ','
      << number(boundary.terminal_comparison.rotation_separation_deg) << '\n';
}

RayBoundaryResult* firstFiniteBoundary(std::vector<RayBoundaryResult>& rays) {
  RayBoundaryResult* best = nullptr;
  for (RayBoundaryResult& ray : rays)
    if (ray.boundary_found && (!best || ray.margin_alpha < best->margin_alpha))
      best = &ray;
  return best;
}

double minimumMargin(std::vector<RayBoundaryResult>& rays, bool* censored,
                     RayBoundaryResult** best) {
  *best = firstFiniteBoundary(rays);
  *censored = (*best == nullptr);
  return *best ? (*best)->margin_alpha : 3.0;
}

Eigen::Vector3d parsePosePosition(const Pose& pose) {
  return pose.block<3, 1>(0, 3);
}

void runBaselineCapture(const std::string& imu_path,
                        const std::string& filter_scans_path,
                        const std::string& scans_path,
                        const std::string& packed_xyz_path,
                        const std::string& map_path,
                        const std::string& params_path,
                        const std::string& broad_manifest_path,
                        const std::string& output_dir) {
  p5_i1::requireFrozenMapSha256(map_path);
  Inputs inputs;
  std::string input_error;
  if (!p4_i2::readInputs(imu_path, filter_scans_path, &inputs, &input_error))
    throw std::runtime_error("cannot_read_frozen_filter_inputs:" + input_error);
  const std::vector<ScanAsset> assets = readScanAssets(scans_path);
  const std::vector<uint64_t> broad_transactions = readTransactions(broad_manifest_path);
  const std::set<uint64_t> capture_transactions(broad_transactions.begin(),
                                                 broad_transactions.end());
  if (assets.size() != 4127 || inputs.scans.size() != 4127 ||
      inputs.imu.size() != 83342 || capture_transactions.size() != 88)
    throw std::runtime_error("frozen_input_or_88_frame_cohort_count_mismatch");

  Pose3d initial_map_T_lidar;
  Pose3d T_imu_lidar;
  const RuntimeParameters parameters = p4_i2::readParameters(
      params_path, &initial_map_T_lidar, &T_imu_lidar);
  if (inputs.imu.size() < static_cast<std::size_t>(parameters.static_init_samples))
    throw std::runtime_error("not_enough_static_initialization_imu_samples");
  FastLio2IkfomFrontend frontend(parameters);
  ImuVector initialization_imu(inputs.imu.begin(),
      inputs.imu.begin() + parameters.static_init_samples);
  std::string reason;
  if (!frontend.initializeStatic(initialization_imu, initial_map_T_lidar,
                                 T_imu_lidar, &reason))
    throw std::runtime_error("baseline_static_initialization_failed:" + reason);

  const Cloud::Ptr target = ::loadTarget(map_path);
  if (target->size() != 549606)
    throw std::runtime_error("frozen_target_point_count_mismatch");
  AuditedNdt ndt;
  ::configureNdt(ndt, target);
  std::ofstream replay(output_dir + "/baseline_replay.csv");
  std::ofstream trajectory(output_dir + "/trajectory_BASELINE.csv");
  std::ofstream contexts(output_dir + "/prediction_contexts.csv");
  std::ofstream fd(output_dir + "/pose_covariance_fd_validation.csv");
  std::ofstream timings(output_dir + "/runtime_breakdown.csv");
  if (!replay || !trajectory || !contexts || !fd || !timings)
    throw std::runtime_error("cannot_create_baseline_capture_outputs");
  trajectory << std::setprecision(17);
  p4_i2::writeHeader(trajectory);
  replay << std::setprecision(17)
      << "transaction_id,stamp_ns,time_s,source_hash_expected,source_hash_actual"
      << ",predictor_lidar_pose_xyz_q_xyzw,raw_lidar_pose_xyz_q_xyzw"
      << ",used_lidar_pose_xyz_q_xyzw,corrected_imu_pose_xyz_q_xyzw"
      << ",fitness,iterations,converged,step_limited,ndt_runtime_ms,step_total_ms\n";
  contexts << std::setprecision(17)
      << "transaction_id,stamp_ns,time_s,T_minus_map_T_imu_xyz_q_xyzw"
      << ",P_state_minus_row_major,P_pose_jacobian_row_major"
      << ",P_pose_map_row_major,cov_eigenvalues,effective_cov_rank"
      << ",covariance_valid,covariance_status,M0_map_T_lidar_xyz_q_xyzw"
      << ",velocity_map,gyro_bias,accel_bias,gravity_map,T_imu_lidar_translation"
      << ",T_imu_lidar_rotation_xyzw\n";
  fd << "transaction_id,stamp_ns,state_index,component,epsilon"
        ",max_abs_column_error,l2_column_error,pass\n";
  timings << "stage,transaction_id,runtime_ms,ndt_call_count\n";

  Eigen::Matrix4d previous_used = Eigen::Matrix4d::Identity();
  bool has_previous_used = false;
  std::set<uint64_t> fd_transactions;
  for (int i = 0; i < 10; ++i) {
    const std::size_t index = static_cast<std::size_t>(std::llround(
        static_cast<double>(i) * static_cast<double>(broad_transactions.size() - 1) / 9.0));
    fd_transactions.insert(broad_transactions[index]);
  }
  int fd_states_checked = 0;
  double max_raw_translation_delta = 0.0;
  double max_raw_rotation_delta = 0.0;
  double max_used_translation_delta = 0.0;
  double max_used_rotation_delta = 0.0;
  double max_rotation_delta = 0.0;
  uint64_t ndt_calls = 0;
  const auto replay_start = std::chrono::steady_clock::now();

  for (std::size_t index = 0; index < assets.size(); ++index) {
    const ScanAsset& asset = assets[index];
    const p4_i2::PoseRecord& saved = inputs.scans[index];
    if (saved.transaction_id != asset.transaction_id ||
        saved.stamp_ns != asset.stamp_ns)
      throw std::runtime_error("filter_input_and_cloud_metadata_alignment_mismatch");
    const auto step_start = std::chrono::steady_clock::now();
    const FilterSnapshot start = frontend.getState();
    const ImuVector window = p4_i2::imuWindow(inputs.imu, start.stamp_ns,
                                               asset.stamp_ns);
    std::vector<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample,
        Eigen::aligned_allocator<dog_prior_map_fastlio2_frontend_exp::ImuPoseSample>>
        imu_poses;
    if (!frontend.predictImuSequence(window, asset.stamp_ns, &imu_poses, &reason))
      throw std::runtime_error("baseline_imu_prediction_failed_tx_" +
          std::to_string(asset.transaction_id) + ":" + reason);
    const FilterSnapshot predicted = frontend.getState();
    const Pose predictor_lidar = (
        p4_i2::asIsometry(predicted.map_T_imu) *
        p4_i2::asIsometry(T_imu_lidar)).matrix();

    const Cloud::Ptr raw_source = loadRawCloudAt(packed_xyz_path, asset);
    const Cloud::Ptr source = ::preprocessSource(raw_source);
    const uint64_t actual_hash = ::sourceCloudHash(source);
    if (actual_hash != asset.expected_source_hash)
      throw std::runtime_error("prepared_source_hash_mismatch_tx_" +
                               std::to_string(asset.transaction_id));
    ndt.setInputSource(source);
    Cloud aligned;
    const auto ndt_start = std::chrono::steady_clock::now();
    ndt.align(aligned, predictor_lidar.cast<float>());
    const double ndt_runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - ndt_start).count();
    ++ndt_calls;
    if (!ndt.hasConverged())
      throw std::runtime_error("baseline_ndt_not_converged_tx_" +
                               std::to_string(asset.transaction_id));
    const Eigen::Matrix4f raw_pose_f = ndt.getFinalTransformation();
    const Pose raw_pose = poseFromPcl(raw_pose_f);
    bool step_limited = false;
    const Pose used_pose = limitStep(raw_pose, previous_used,
        has_previous_used, &step_limited);
    previous_used = used_pose;
    has_previous_used = true;
    Pose3d used_pose_lidar;
    used_pose_lidar.position = used_pose.block<3, 1>(0, 3);
    used_pose_lidar.orientation = Eigen::Quaterniond(
        used_pose.block<3, 3>(0, 0)).normalized();
    const Pose3d measurement_imu = p4_i2::lidarMeasurementToImu(
        used_pose_lidar, T_imu_lidar);
    dog_prior_map_fastlio2_frontend_exp::PoseCorrectionDelta update_delta;
    if (!frontend.applyPoseMeasurement(measurement_imu, &update_delta, &reason))
      throw std::runtime_error("baseline_pose_update_failed_tx_" +
          std::to_string(asset.transaction_id) + ":" + reason);
    const double step_total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - step_start).count();
    const FilterSnapshot corrected = frontend.getState();
    p4_i2::writeRow(trajectory, index, saved, predicted.map_T_imu,
                    predicted, corrected, T_imu_lidar);
    const double fitness = ndt.getFitnessScore();
    const int iterations = ndt.getFinalNumIteration();
    const bool converged = ndt.hasConverged();
    replay << asset.transaction_id << ',' << asset.stamp_ns << ','
        << number(asset.time_s) << ',' << asset.expected_source_hash << ','
        << actual_hash << ',' << pose7(predictor_lidar) << ',' << pose7(raw_pose)
        << ',' << pose7(used_pose) << ',' << pose7(poseMatrix(corrected.map_T_imu))
        << ',' << number(fitness) << ',' << iterations << ',' << converged << ','
        << step_limited << ',' << number(ndt_runtime_ms) << ','
        << number(step_total_ms) << '\n';

    if (capture_transactions.count(asset.transaction_id)) {
      const CovarianceCapture covariance = capturePoseCovariance(predicted);
      writePoseCovarianceContext(contexts, asset.transaction_id, asset.time_s,
                                  predicted, raw_pose, covariance);
      if (fd_transactions.count(asset.transaction_id)) {
        ++fd_states_checked;
        for (const FdColumnResult& column : validatePoseJacobian(
                 predicted, asset.transaction_id, covariance)) {
          if (!column.pass)
            throw std::runtime_error("pose_covariance_jacobian_fd_failed_tx_" +
                                     std::to_string(asset.transaction_id));
          fd << column.transaction_id << ',' << column.stamp_ns << ','
             << column.state_index << ',' << column.component << ','
             << number(column.epsilon) << ',' << number(column.max_abs_error)
             << ',' << number(column.column_norm_error) << ',' << column.pass << '\n';
        }
      }
    }
    timings << "BASELINE," << asset.transaction_id << ','
            << number(ndt_runtime_ms) << ",1\n";
    max_raw_translation_delta = std::max(max_raw_translation_delta,
        p6_i4::translationSeparation(raw_pose, asset.saved_raw));
    max_raw_rotation_delta = std::max(max_raw_rotation_delta,
        rotationDifferenceDeg(raw_pose, asset.saved_raw));
    max_used_translation_delta = std::max(max_used_translation_delta,
        p6_i4::translationSeparation(used_pose, asset.saved_used));
    max_used_rotation_delta = std::max(max_used_rotation_delta,
        rotationDifferenceDeg(used_pose, asset.saved_used));
    if (asset.transaction_id % 250 == 0)
      std::cout << "P6_I4_BASELINE_PROGRESS tx=" << asset.transaction_id
                << "/" << assets.size() << std::endl;
  }
  const double replay_total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - replay_start).count();
  if (fd_states_checked != 10)
    throw std::runtime_error("pose_covariance_fd_state_count_not_10");
  timings << "BASELINE_TOTAL,0," << number(replay_total_ms) << ',' << ndt_calls << '\n';
  std::cout << "P6_I4_BASELINE_CAPTURE_COMPLETE scans=" << assets.size()
            << " fd_states=" << fd_states_checked
            << " ndt_calls=" << ndt_calls
            << " replay_ms=" << replay_total_ms
            << " max_raw_translation_delta_vs_saved=" << max_raw_translation_delta
            << " max_raw_rotation_delta_deg_vs_saved=" << max_raw_rotation_delta
            << " max_used_translation_delta_vs_saved=" << max_used_translation_delta
            << " max_used_rotation_delta_deg_vs_saved=" << max_used_rotation_delta
            << std::endl;
}

uint64_t probeCallCount() {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (line.rfind("VmHWM:", 0) == 0) {
      std::stringstream parser(line.substr(6));
      uint64_t kib = 0;
      parser >> kib;
      return kib * 1024;
    }
  }
  return 0;
}

struct MarginSummaryRow {
  PredictionContext context;
  double m_principal = 3.0;
  bool principal_censored = true;
  int principal_ray_id = -1;
  int principal_sign = 0;
  double principal_low = 3.0;
  double principal_high = 3.0;
  std::string principal_direction;
  double m_dense = std::numeric_limits<double>::quiet_NaN();
  bool dense_censored = true;
  int dense_ray_id = -1;
  int dense_sign = 0;
  double dense_low = std::numeric_limits<double>::quiet_NaN();
  double dense_high = std::numeric_limits<double>::quiet_NaN();
  std::string dense_direction;
  std::map<double, double> retention;
};

void writeMarginSummary(const MarginSummaryRow& row, std::ostream& principal,
                        std::ostream* dense) {
  principal << row.context.frame_id << ',' << row.context.transaction_id << ','
      << number(row.context.time_s) << ',' << row.context.covariance.effective_rank
      << ',' << flatten(row.context.covariance.eigenvalues) << ','
      << pose7(row.context.T_minus_map_T_imu) << ','
      << pose7(row.context.nominal_map_T_lidar) << ','
      << number(row.m_principal) << ',' << row.principal_censored << ','
      << row.principal_direction << ',' << row.principal_ray_id << ','
      << row.principal_sign << ',' << number(row.principal_low) << ','
      << number(row.principal_high) << '\n';
  if (dense) {
    *dense << row.context.frame_id << ',' << row.context.transaction_id << ','
        << number(row.context.time_s) << ',' << row.context.covariance.effective_rank
        << ',' << flatten(row.context.covariance.eigenvalues) << ','
        << pose7(row.context.nominal_map_T_lidar) << ','
        << number(row.m_principal) << ',' << row.principal_censored << ','
        << number(row.m_dense) << ',' << row.dense_censored << ','
        << row.dense_direction << ',' << row.dense_ray_id << ','
        << row.dense_sign << ',' << number(row.dense_low) << ','
        << number(row.dense_high) << ',';
    for (double alpha : {0.5, 1.0, 2.0, 3.0}) {
      if (alpha != 0.5) *dense << ',';
      const auto found = row.retention.find(alpha);
      *dense << (found == row.retention.end() ? "" : number(found->second));
    }
    *dense << '\n';
  }
}

void runBasinSearch(const std::string& scans_path,
                    const std::string& packed_xyz_path,
                    const std::string& map_path,
                    const std::string& broad_manifest_path,
                    const std::string& dense_manifest_path,
                    const std::string& contexts_path,
                    const std::string& directions_path,
                    const std::string& output_dir) {
  p5_i1::requireFrozenMapSha256(map_path);
  const auto assets = readScanAssets(scans_path);
  const auto contexts = readPredictionContexts(contexts_path, broad_manifest_path);
  const auto dense_transactions = readDenseTransactions(dense_manifest_path);
  const auto directions = readExtraDirections(directions_path);
  const Cloud::Ptr target = ::loadTarget(map_path);
  if (target->size() != 549606)
    throw std::runtime_error("frozen_target_point_count_mismatch");
  std::map<uint64_t, const ScanAsset*> asset_by_tx;
  for (const ScanAsset& asset : assets) asset_by_tx.emplace(asset.transaction_id, &asset);

  std::ofstream principal(output_dir + "/principal_margin.csv");
  std::ofstream dense(output_dir + "/dense_reference_margin.csv");
  std::ofstream probes(output_dir + "/ray_probe_results.csv");
  std::ofstream retention(output_dir + "/basin_retention.csv");
  std::ofstream repeat(output_dir + "/repeatability_audit.csv");
  std::ofstream runtime(output_dir + "/runtime_breakdown.csv", std::ios::app);
  if (!principal || !dense || !probes || !retention || !repeat || !runtime)
    throw std::runtime_error("cannot_create_margin_outputs");
  principal << std::setprecision(17)
      << "frame_id,transaction_id,time_s,effective_cov_rank,cov_eigenvalues"
         ",T_minus_map_T_imu_xyz_q_xyzw,M0_map_T_lidar_xyz_q_xyzw,m_principal"
         ",principal_censored,principal_boundary_direction,principal_boundary_ray"
         ",principal_sign,principal_boundary_interval_low,principal_boundary_interval_high\n";
  dense << std::setprecision(17)
      << "frame_id,transaction_id,time_s,effective_cov_rank,cov_eigenvalues"
         ",M0_map_T_lidar_xyz_q_xyzw,m_principal,principal_censored,m_dense"
         ",dense_censored,dense_boundary_direction,dense_boundary_ray,dense_sign"
         ",dense_boundary_interval_low,dense_boundary_interval_high,S_0_5,S_1,S_2,S_3\n";
  probes << std::setprecision(17)
      << "frame_id,transaction_id,time_s,ray_type,ray_id,sign,alpha,prior_metric_radius"
         ",delta_phi_x,delta_phi_y,delta_phi_z,delta_p_x,delta_p_y,delta_p_z"
         ",seed_map_T_lidar_xyz_q_xyzw,converged,terminal_map_T_lidar_xyz_q_xyzw"
         ",terminal_objective,terminal_fitness,terminal_iterations,same_as_nominal"
         ",terminal_translation_separation_m,terminal_rotation_separation_deg,runtime_ms\n";
  retention << std::setprecision(17)
      << "frame_id,transaction_id,time_s,alpha,same_mode_count,total_probe_count"
         ",empirical_directional_retention\n";
  repeat << std::setprecision(17)
      << "frame_id,transaction_id,time_s,first_boundary_ray,repeat_boundary_ray"
         ",first_sign,repeat_sign,m_principal_first,m_principal_repeat"
         ",absolute_margin_delta,pass,first_censored,repeat_censored\n";
  runtime << std::setprecision(17);

  std::vector<MarginSummaryRow> summaries;
  summaries.reserve(contexts.size());
  uint64_t total_margin_calls = 0;
  for (const PredictionContext& context : contexts) {
    const auto asset_it = asset_by_tx.find(context.transaction_id);
    if (asset_it == asset_by_tx.end()) throw std::runtime_error("context_scan_asset_missing");
    const ScanAsset& asset = *asset_it->second;
    const auto frame_start = std::chrono::steady_clock::now();
    const Cloud::Ptr raw_source = loadRawCloudAt(packed_xyz_path, asset);
    const Cloud::Ptr source = ::preprocessSource(raw_source);
    if (::sourceCloudHash(source) != asset.expected_source_hash)
      throw std::runtime_error("prepared_source_hash_mismatch_probe_tx_" +
                               std::to_string(context.transaction_id));
    ProbeEngine engine(context, target, source, probes);
    MarginSummaryRow summary;
    summary.context = context;
    auto principal_rays = estimatePrincipalRays(engine, context, "PRINCIPAL");
    RayBoundaryResult* principal_best = nullptr;
    summary.m_principal = minimumMargin(principal_rays,
        &summary.principal_censored, &principal_best);
    if (principal_best) {
      summary.principal_ray_id = principal_best->ray_id;
      summary.principal_sign = principal_best->sign;
      summary.principal_low = principal_best->alpha_same;
      summary.principal_high = principal_best->alpha_diff;
      summary.principal_direction = "EIGEN_" +
          std::to_string(principal_best->ray_id + 1);
    } else {
      summary.principal_direction = "CENSORED_HIGH";
    }

    if (dense_transactions.count(context.transaction_id)) {
      std::vector<RayBoundaryResult> all_dense_rays = principal_rays;
      std::vector<Eigen::VectorXd> unique_extra_directions;
      unique_extra_directions.reserve(directions.size());
      for (std::size_t index = 0; index < directions.size(); ++index) {
        Eigen::VectorXd direction = normalizedTruncatedDirection(
            directions[index], context.covariance.effective_rank);
        bool duplicate = false;
        for (int axis = 0; axis < context.covariance.effective_rank; ++axis) {
          Eigen::VectorXd principal_direction = principalDirection(context.covariance, axis);
          if (std::abs(direction.dot(principal_direction)) > 1.0 - 1e-12)
            duplicate = true;
        }
        for (const Eigen::VectorXd& prior : unique_extra_directions)
          if (std::abs(direction.dot(prior)) > 1.0 - 1e-12) duplicate = true;
        if (duplicate) continue;
        unique_extra_directions.push_back(direction);
        const int ray_id = 100 + static_cast<int>(index);
        for (int sign : {-1, 1}) {
          auto boundary = searchFirstBoundaryOnRay(engine, "EXTRA_MARGIN",
              ray_id, sign, direction * static_cast<double>(sign));
          all_dense_rays.push_back(std::move(boundary));
        }
      }
      if (unique_extra_directions.size() != 32)
        throw std::runtime_error("extra_reference_direction_dedup_removed_sample");
      RayBoundaryResult* dense_best = nullptr;
      summary.m_dense = minimumMargin(all_dense_rays,
          &summary.dense_censored, &dense_best);
      if (dense_best) {
        summary.dense_ray_id = dense_best->ray_id;
        summary.dense_sign = dense_best->sign;
        summary.dense_low = dense_best->alpha_same;
        summary.dense_high = dense_best->alpha_diff;
        summary.dense_direction = dense_best->ray_type + "_" +
            std::to_string(dense_best->ray_id);
      } else {
        summary.dense_direction = "CENSORED_HIGH";
      }
      for (double alpha : {0.5, 1.0, 2.0, 3.0}) {
        int same_count = 0;
        for (std::size_t index = 0; index < unique_extra_directions.size(); ++index) {
          const int ray_id = 100 + static_cast<int>(index);
          for (int sign : {-1, 1}) {
            const auto probe = engine.run("EXTRA_RETENTION", ray_id, sign, alpha,
                unique_extra_directions[index] * static_cast<double>(sign));
            same_count += probe.second.same_mode ? 1 : 0;
          }
        }
        const double score = static_cast<double>(same_count) / 64.0;
        summary.retention.emplace(alpha, score);
        retention << context.frame_id << ',' << context.transaction_id << ','
            << number(context.time_s) << ',' << number(alpha) << ','
            << same_count << ",64," << number(score) << '\n';
      }
    }
    writeMarginSummary(summary, principal,
        dense_transactions.count(context.transaction_id) ? &dense : nullptr);
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - frame_start).count();
    const uint64_t frame_calls = engine.callCount();
    total_margin_calls += frame_calls;
    runtime << "PRINCIPAL_AND_DENSE," << context.transaction_id << ','
            << number(elapsed_ms) << ',' << frame_calls << '\n';
    summaries.push_back(std::move(summary));
    if (contexts.size() % 20 == 0 || summaries.size() % 10 == 0 ||
        summaries.size() == contexts.size())
      std::cout << "P6_I4_MARGIN_PROGRESS frames=" << summaries.size()
                << '/' << contexts.size() << " dense_frames="
                << dense_transactions.size() << std::endl;
  }

  std::vector<std::size_t> repeat_indices;
  for (int i = 0; i < 10; ++i)
    repeat_indices.push_back(static_cast<std::size_t>(std::llround(
        static_cast<double>(i) * static_cast<double>(contexts.size() - 1) / 9.0)));
  std::set<std::size_t> unique_repeat_indices(repeat_indices.begin(), repeat_indices.end());
  if (unique_repeat_indices.size() != 10)
    throw std::runtime_error("repeatability_selection_deduplication_error");
  uint64_t repeat_calls = 0;
  for (std::size_t index : repeat_indices) {
    const PredictionContext& context = contexts[index];
    const auto asset_it = asset_by_tx.find(context.transaction_id);
    const Cloud::Ptr raw_source = loadRawCloudAt(packed_xyz_path, *asset_it->second);
    const Cloud::Ptr source = ::preprocessSource(raw_source);
    ProbeEngine engine(context, target, source, probes);
    auto repeated = estimatePrincipalRays(engine, context,
                                          "REPEATABILITY", 300);
    repeat_calls += engine.callCount();
    bool repeat_censored = false;
    RayBoundaryResult* repeat_best = nullptr;
    const double repeat_margin = minimumMargin(repeated, &repeat_censored, &repeat_best);
    const MarginSummaryRow& first = summaries[index];
    const bool same_ray = first.principal_censored == repeat_censored &&
        (first.principal_censored ||
         (first.principal_ray_id == repeat_best->ray_id - 300 &&
          first.principal_sign == repeat_best->sign));
    const double difference = std::abs(first.m_principal - repeat_margin);
    const bool pass = same_ray && difference <= 0.01;
    repeat << context.frame_id << ',' << context.transaction_id << ','
        << number(context.time_s) << ',' << first.principal_ray_id << ','
        << (repeat_best ? std::to_string(repeat_best->ray_id - 300) : "CENSORED") << ','
        << first.principal_sign << ',' << (repeat_best ? repeat_best->sign : 0) << ','
        << number(first.m_principal) << ',' << number(repeat_margin) << ','
        << number(difference) << ',' << pass << ',' << first.principal_censored
        << ',' << repeat_censored << '\n';
    runtime << "REPEATABILITY," << context.transaction_id << ','
            << number(0.0) << ',' << engine.callCount() << "\n";
  }
  std::cout << "P6_I4_MARGIN_SEARCH_COMPLETE frames=" << contexts.size()
            << " dense_frames=" << dense_transactions.size()
            << " repeatability_frames=" << unique_repeat_indices.size()
            << " principal_dense_probe_calls=" << total_margin_calls
            << " baseline_nominal_calls=" << contexts.size()
            << " repeat_calls=" << repeat_calls
            << " peak_rss_bytes=" << probeCallCount() << std::endl;
}

}  // namespace p6_i4_app

int main(int argc, char** argv) {
  try {
    if (argc == 10 && std::string(argv[1]) == "baseline") {
      p6_i4_app::runBaselineCapture(argv[2], argv[3], argv[4], argv[5],
          argv[6], argv[7], argv[8], argv[9]);
      return 0;
    }
    if (argc == 10 && std::string(argv[1]) == "search") {
      p6_i4_app::runBasinSearch(argv[2], argv[3], argv[4], argv[5], argv[6],
          argv[7], argv[8], argv[9]);
      return 0;
    }
    std::cerr << "usage: p6_i4_basin_margin baseline IMU.csv FILTER_SCANS.csv "
        "SCANS.csv XYZ.bin MAP.pcd PARAMS.txt BROAD_MANIFEST.csv OUT_DIR\n"
        "       p6_i4_basin_margin search SCANS.csv XYZ.bin MAP.pcd "
        "BROAD.csv DENSE.csv CONTEXTS.csv DIRECTIONS.csv OUT_DIR\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "P6_I4_ERROR: " << error.what() << '\n';
    return 1;
  }
}
