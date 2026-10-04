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

bool integrateBodyRelativeRotation(
    const P7ImuVector& all, uint64_t start_stamp_ns, uint64_t end_stamp_ns,
    const Eigen::Vector3d& gyro_bias, Eigen::Matrix3d* R_start_to_end,
    std::string* failure_reason) {
  if (failure_reason) failure_reason->clear();
  const auto fail = [failure_reason](const char* message) {
    if (failure_reason) *failure_reason = message;
    return false;
  };
  if (!R_start_to_end || all.size() < 2 || start_stamp_ns == 0 ||
      end_stamp_ns <= start_stamp_ns || !gyro_bias.allFinite())
    return fail("invalid_relative_rotation_input");
  if (start_stamp_ns < all.front().stamp_ns || end_stamp_ns > all.back().stamp_ns)
    return fail("relative_rotation_epoch_outside_imu_coverage");

  const auto sampleAt = [&all](uint64_t stamp, ImuSample* output) {
    if (!output) return false;
    const auto upper = std::lower_bound(all.begin(), all.end(), stamp,
        [](const ImuSample& sample, uint64_t time) { return sample.stamp_ns < time; });
    if (upper != all.end() && upper->stamp_ns == stamp) {
      *output = *upper;
      return true;
    }
    if (upper == all.begin() || upper == all.end()) return false;
    const ImuSample& lower = *(upper - 1);
    if (upper->stamp_ns <= lower.stamp_ns) return false;
    const double alpha = static_cast<double>(stamp - lower.stamp_ns) /
        static_cast<double>(upper->stamp_ns - lower.stamp_ns);
    output->stamp_ns = stamp;
    output->acceleration = (1.0 - alpha) * lower.acceleration + alpha * upper->acceleration;
    output->angular_velocity =
        (1.0 - alpha) * lower.angular_velocity + alpha * upper->angular_velocity;
    return output->acceleration.allFinite() && output->angular_velocity.allFinite();
  };

  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> knots;
  ImuSample start;
  ImuSample end;
  if (!sampleAt(start_stamp_ns, &start) || !sampleAt(end_stamp_ns, &end))
    return fail("relative_rotation_missing_epoch_bracket");
  knots.push_back(start);
  auto next = std::upper_bound(all.begin(), all.end(), start_stamp_ns,
      [](uint64_t time, const ImuSample& sample) { return time < sample.stamp_ns; });
  for (; next != all.end() && next->stamp_ns < end_stamp_ns; ++next)
    knots.push_back(*next);
  knots.push_back(end);

  Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
  for (std::size_t index = 0; index + 1 < knots.size(); ++index) {
    const ImuSample& head = knots[index];
    const ImuSample& tail = knots[index + 1];
    if (tail.stamp_ns <= head.stamp_ns || !head.angular_velocity.allFinite() ||
        !tail.angular_velocity.allFinite())
      return fail("invalid_relative_rotation_imu_interval");
    const double dt = static_cast<double>(tail.stamp_ns - head.stamp_ns) * 1e-9;
    const Eigen::Vector3d omega =
        0.5 * (head.angular_velocity + tail.angular_velocity) - gyro_bias;
    const double angle = omega.norm() * dt;
    if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(angle))
      return fail("nonfinite_relative_rotation_increment");
    if (angle > 0.0)
      result *= Eigen::AngleAxisd(angle, omega.normalized()).toRotationMatrix();
  }
  if (!result.allFinite() || std::abs(result.determinant() - 1.0) > 1e-8 ||
      (result.transpose() * result - Eigen::Matrix3d::Identity()).norm() > 1e-8)
    return fail("invalid_integrated_relative_rotation");
  *R_start_to_end = result;
  return true;
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

std::vector<P7TimedScanRecord> readP7TimedScans(
    const std::string& filter_path, const std::string& raw_scan_index_path) {
  const auto filter = readFilterScans(filter_path);
  std::ifstream input(raw_scan_index_path);
  std::string line;
  if (!input || !std::getline(input, line))
    throw std::runtime_error("cannot_read_raw_timed_scan_index");
  const auto header = splitCsv(line);
  std::map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index)
    if (!columns.emplace(header[index], index).second)
      throw std::runtime_error("duplicate_raw_timed_scan_index_header");
  auto requiredColumn = [&columns](const char* primary, const char* alternate = nullptr) {
    auto found = columns.find(primary);
    if (found == columns.end() && alternate) found = columns.find(alternate);
    if (found == columns.end())
      throw std::runtime_error(std::string("missing_raw_timed_scan_index_field:") + primary);
    return found->second;
  };
  const std::size_t transaction_column = requiredColumn("transaction_id");
  const std::size_t start_column = requiredColumn("scan_start_ns");
  const std::size_t end_column = requiredColumn("scan_end_ns");
  const std::size_t offset_column = requiredColumn("cloud_byte_offset", "byte_offset");
  const std::size_t count_column = requiredColumn("cloud_point_count", "point_count");

  std::vector<P7TimedScanRecord> scans;
  uint64_t expected_offset = 0;
  uint64_t previous_end = 0;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != header.size())
      throw std::runtime_error("invalid_raw_timed_scan_index_field_count");
    P7TimedScanRecord scan;
    scan.transaction_id = unsignedField(fields[transaction_column]);
    scan.scan_start_ns = unsignedField(fields[start_column]);
    scan.scan_end_ns = unsignedField(fields[end_column]);
    scan.cloud_byte_offset = unsignedField(fields[offset_column]);
    scan.cloud_point_count = unsignedField(fields[count_column]);
    if (scan.transaction_id != scans.size() + 1 || scan.scan_start_ns == 0 ||
        scan.scan_end_ns <= scan.scan_start_ns || scan.scan_end_ns <= previous_end ||
        scan.cloud_byte_offset != expected_offset || scan.cloud_point_count == 0 ||
        scan.cloud_point_count > (std::numeric_limits<uint64_t>::max() - expected_offset) / 16)
      throw std::runtime_error("invalid_raw_timed_scan_index_sequence");
    if (scans.size() >= filter.size() || filter[scans.size()].first != scan.transaction_id ||
        filter[scans.size()].second != scan.scan_end_ns)
      throw std::runtime_error("raw_timed_scan_filter_alignment_mismatch");
    expected_offset += 16 * scan.cloud_point_count;
    previous_end = scan.scan_end_ns;
    scans.push_back(scan);
  }
  if (scans.size() != filter.size())
    throw std::runtime_error("raw_timed_scan_filter_count_mismatch");
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

P7TimedLidarVector readP7PackedTimedCloud(
    const std::string& path, const P7TimedScanRecord& scan) {
  static_assert(sizeof(float) == 4 && sizeof(uint32_t) == 4,
                "timed point records require 32-bit float and uint32");
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("cannot_open_packed_timed_lidar");
  const std::streamoff end = input.tellg();
  if (end < 0 || scan.cloud_byte_offset > static_cast<uint64_t>(end) ||
      scan.cloud_point_count > (static_cast<uint64_t>(end) - scan.cloud_byte_offset) / 16 ||
      scan.cloud_point_count > std::numeric_limits<std::size_t>::max())
    throw std::runtime_error("truncated_packed_timed_lidar");
  input.seekg(static_cast<std::streamoff>(scan.cloud_byte_offset));
  P7TimedLidarVector cloud;
  cloud.reserve(static_cast<std::size_t>(scan.cloud_point_count));
  const uint64_t maximum_offset_ns = scan.scan_end_ns - scan.scan_start_ns;
  for (uint64_t i = 0; i < scan.cloud_point_count; ++i) {
    uint8_t record[16];
    input.read(reinterpret_cast<char*>(record), sizeof(record));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(record)))
      throw std::runtime_error("truncated_packed_timed_lidar_record");
    float xyz[3];
    uint32_t offset_ns = 0;
    std::memcpy(xyz, record, sizeof(xyz));
    std::memcpy(&offset_ns, record + sizeof(xyz), sizeof(offset_ns));
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]) ||
        offset_ns > maximum_offset_ns || offset_ns >
            std::numeric_limits<uint64_t>::max() - scan.scan_start_ns)
      throw std::runtime_error("invalid_packed_timed_lidar_payload");
    TimedLidarPoint point;
    point.position = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
    point.stamp_ns = scan.scan_start_ns + offset_ns;
    cloud.push_back(point);
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
