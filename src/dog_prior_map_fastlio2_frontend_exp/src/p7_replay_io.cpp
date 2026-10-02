#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
std::vector<std::string> splitCsv(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::stringstream input(line);
  std::vector<std::string> fields;
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  return fields;
}

uint64_t unsignedField(const std::string& value) {
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
    throw std::runtime_error("invalid_unsigned_input_field");
  return std::stoull(value);
}

double finiteField(const std::string& value) {
  std::size_t end = 0;
  const double parsed = std::stod(value, &end);
  if (end != value.size() || !std::isfinite(parsed))
    throw std::runtime_error("invalid_finite_input_field");
  return parsed;
}

std::vector<std::pair<uint64_t, uint64_t>> readFilterScans(const std::string& path) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line)) throw std::runtime_error("cannot_read_filter_scans");
  const auto header = splitCsv(line);
  if (header.size() < 2 || header[0] != "transaction_id" || header[1] != "stamp_ns")
    throw std::runtime_error("invalid_filter_scan_header");
  std::vector<std::pair<uint64_t, uint64_t>> scans;
  uint64_t previous_stamp = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 2 && fields.size() != 23)
      throw std::runtime_error("invalid_filter_scan_field_count");
    // All additional historical fields are ignored, not parsed into state.
    const uint64_t tx = unsignedField(fields[0]);
    const uint64_t stamp = unsignedField(fields[1]);
    if (tx != scans.size() + 1 || stamp <= previous_stamp)
      throw std::runtime_error("invalid_filter_scan_sequence");
    previous_stamp = stamp;
    scans.emplace_back(tx, stamp);
  }
  if (scans.empty()) throw std::runtime_error("empty_filter_scans");
  return scans;
}
}  // namespace

P7ImuVector readP7Imu(const std::string& path) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) ||
      splitCsv(line) != std::vector<std::string>{"stamp_ns", "ax", "ay", "az", "gx", "gy", "gz"})
    throw std::runtime_error("cannot_read_imu_header");
  P7ImuVector samples;
  uint64_t previous_stamp = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != 7) throw std::runtime_error("invalid_imu_field_count");
    ImuSample sample;
    sample.stamp_ns = unsignedField(fields[0]);
    sample.acceleration = Eigen::Vector3d(finiteField(fields[1]), finiteField(fields[2]), finiteField(fields[3]));
    sample.angular_velocity = Eigen::Vector3d(finiteField(fields[4]), finiteField(fields[5]), finiteField(fields[6]));
    if (sample.stamp_ns <= previous_stamp) throw std::runtime_error("imu_not_strictly_monotonic");
    previous_stamp = sample.stamp_ns;
    samples.push_back(sample);
  }
  if (samples.empty()) throw std::runtime_error("empty_imu_input");
  return samples;
}

std::vector<P7ScanRecord> readP7Scans(const std::string& filter_path, const std::string& asset_path) {
  const auto filter = readFilterScans(filter_path);
  std::ifstream input(asset_path);
  std::string line;
  if (!input || !std::getline(input, line)) throw std::runtime_error("cannot_read_scan_assets");
  const auto header = splitCsv(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i)
    if (!columns.emplace(header[i], i).second) throw std::runtime_error("duplicate_scan_header");
  auto required = [&columns](const std::vector<std::string>& fields, const std::string& key) -> const std::string& {
    const auto found = columns.find(key);
    if (found == columns.end() || found->second >= fields.size())
      throw std::runtime_error("missing_scan_asset_field:" + key);
    return fields[found->second];
  };
  std::vector<P7ScanRecord> scans;
  uint64_t expected_offset = 0, previous_stamp = 0;
  double previous_time = -std::numeric_limits<double>::infinity();
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    P7ScanRecord scan;
    scan.transaction_id = unsignedField(required(fields, "transaction_id"));
    scan.stamp_ns = unsignedField(required(fields, "stamp_ns"));
    scan.time_s = finiteField(required(fields, "time_s"));
    scan.cloud_byte_offset = unsignedField(required(fields, "cloud_byte_offset"));
    scan.cloud_point_count = unsignedField(required(fields, "cloud_point_count"));
    if (columns.count("ndt_source_cloud_hash"))
      scan.expected_source_hash = unsignedField(required(fields, "ndt_source_cloud_hash"));
    scan.expected_source_hash_available = columns.count("ndt_source_cloud_hash") != 0;
    if (columns.count("ndt_source_cloud_hash_available")) {
      const auto& flag = required(fields, "ndt_source_cloud_hash_available");
      if (flag != "0" && flag != "1") throw std::runtime_error("invalid_source_hash_availability");
      scan.expected_source_hash_available = flag == "1";
      if (scan.expected_source_hash_available && !columns.count("ndt_source_cloud_hash"))
        throw std::runtime_error("missing_available_source_hash");
    }
    if (scan.transaction_id != scans.size() + 1 || scan.stamp_ns <= previous_stamp ||
        scan.time_s <= previous_time || scan.cloud_byte_offset != expected_offset ||
        scan.cloud_point_count == 0 || scan.cloud_point_count >
            (std::numeric_limits<uint64_t>::max() - expected_offset) / 12)
      throw std::runtime_error("invalid_scan_asset_sequence");
    if (scans.size() >= filter.size() || filter[scans.size()].first != scan.transaction_id ||
        filter[scans.size()].second != scan.stamp_ns)
      throw std::runtime_error("scan_filter_asset_alignment_mismatch");
    previous_stamp = scan.stamp_ns;
    previous_time = scan.time_s;
    expected_offset += 12 * scan.cloud_point_count;
    scans.push_back(scan);
  }
  if (scans.size() != filter.size()) throw std::runtime_error("scan_filter_asset_count_mismatch");
  return scans;
}

RegistrationCloud readP7PackedCloud(const std::string& path, const P7ScanRecord& scan) {
  static_assert(sizeof(float) == 4, "packed XYZ requires float32");
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("cannot_open_packed_xyz");
  const std::streamoff end = input.tellg();
  if (end < 0 || scan.cloud_byte_offset > static_cast<uint64_t>(end) ||
      scan.cloud_point_count > (static_cast<uint64_t>(end) - scan.cloud_byte_offset) / 12 ||
      scan.cloud_point_count > std::numeric_limits<std::size_t>::max())
    throw std::runtime_error("truncated_packed_xyz");
  input.seekg(static_cast<std::streamoff>(scan.cloud_byte_offset));
  RegistrationCloud cloud;
  cloud.reserve(static_cast<std::size_t>(scan.cloud_point_count));
  for (uint64_t i = 0; i < scan.cloud_point_count; ++i) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(xyz)))
      throw std::runtime_error("truncated_packed_xyz");
    cloud.push_back({xyz[0], xyz[1], xyz[2]});
  }
  return cloud;
}

RuntimeParameters readP7Parameters(const std::string& path, Pose3d* initial, Pose3d* extrinsic) {
  if (!initial || !extrinsic) throw std::runtime_error("null_parameter_pose_output");
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot_open_parameter_file");
  std::vector<double> values;
  for (int field = 0; field < 27; ++field) {
    double value;
    if (!(input >> value)) throw std::runtime_error("expected_27_runtime_parameters");
    values.push_back(value);
  }
  input >> std::ws;
  if (input.bad() || input.peek() != std::char_traits<char>::eof())
    throw std::runtime_error("expected_27_runtime_parameters");
  for (double scalar : values)
    if (!std::isfinite(scalar)) throw std::runtime_error("nonfinite_runtime_parameter");
  std::size_t i = 0;
  RuntimeParameters parameters;
  if (values[0] < 2 || values[0] > std::numeric_limits<int>::max() || std::floor(values[0]) != values[0])
    throw std::runtime_error("invalid_static_sample_count");
  parameters.static_init_samples = static_cast<int>(values[i++]);
  parameters.gravity_mps2 = values[i++];
  parameters.initial_accel_bias = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
  parameters.gyro_noise_std_rad_s = values[i++];
  parameters.accel_noise_std_m_s2 = values[i++];
  parameters.gyro_bias_rw_std_rad_s2 = values[i++];
  parameters.accel_bias_rw_std_m_s3 = values[i++];
  parameters.pose_position_sigma_m = values[i++];
  parameters.pose_rotation_sigma_rad = values[i++];
  parameters.max_static_accel_std_m_s2 = values[i++];
  parameters.max_static_gyro_std_rad_s = values[i++];
  auto readPose = [&values, &i](Pose3d* pose) {
    pose->position = Eigen::Vector3d(values[i], values[i + 1], values[i + 2]); i += 3;
    pose->orientation = Eigen::Quaterniond(values[i + 3], values[i], values[i + 1], values[i + 2]); i += 4;
    if (pose->orientation.norm() < 1e-12) throw std::runtime_error("invalid_parameter_quaternion");
    pose->orientation.normalize();
  };
  readPose(initial); readPose(extrinsic);
  return parameters;
}

Eigen::Isometry3d asIsometry(const Pose3d& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.orientation.toRotationMatrix(); result.translation() = pose.position;
  return result;
}
Pose3d fromIsometry(const Eigen::Isometry3d& transform) {
  Pose3d pose;
  pose.position = transform.translation();
  pose.orientation = Eigen::Quaterniond(transform.linear()).normalized();
  return pose;
}
Pose3d lidarMeasurementToImu(const Pose3d& lidar, const Pose3d& extrinsic) {
  return fromIsometry(asIsometry(lidar) * asIsometry(extrinsic).inverse());
}
P7ImuVector imuWindow(const P7ImuVector& all, uint64_t state_stamp_ns, uint64_t end_stamp_ns) {
  if (end_stamp_ns <= state_stamp_ns) throw std::runtime_error("invalid_imu_interval");
  const auto after_state = std::upper_bound(all.begin(), all.end(), state_stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (after_state == all.begin()) throw std::runtime_error("no_causal_imu_before_state");
  const auto first = after_state - 1;
  const auto after_end = std::upper_bound(all.begin(), all.end(), end_stamp_ns,
      [](uint64_t stamp, const ImuSample& sample) { return stamp < sample.stamp_ns; });
  if (after_end - first < 2) throw std::runtime_error("fewer_than_two_scan_imu_samples");
  return P7ImuVector(first, after_end);
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
