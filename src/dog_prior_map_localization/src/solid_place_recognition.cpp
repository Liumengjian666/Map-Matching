#include "dog_prior_map_localization/solid_place_recognition.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

namespace dog_prior_map_localization
{
namespace
{
using Clock = std::chrono::steady_clock;

bool validRigidPose(const Eigen::Matrix4d &pose)
{
  if (!pose.allFinite()) return false;
  if ((pose.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-8) return false;
  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <= 1e-5 &&
         std::abs(rotation.determinant() - 1.0) <= 1e-5;
}

double elapsedMs(const Clock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

void finalizeCloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

}  // namespace

constexpr const char *SolidPlaceRecognitionAdapter::kDatabaseMagic;
constexpr const char *SolidPlaceRecognitionAdapter::kUpstreamCommit;

bool SolidPlaceRecognitionAdapter::finiteDescriptor(const Eigen::VectorXd &descriptor,
                                                    int expected_size) const
{
  if (descriptor.size() != expected_size || !descriptor.allFinite()) return false;
  if ((descriptor.array() < 0.0).any()) return false;
  const double range_norm = descriptor.head(module_.NUM_RANGE).norm();
  const double total_norm = descriptor.norm();
  if (!std::isfinite(range_norm) || !std::isfinite(total_norm) || range_norm <= 1e-12)
    return false;
  const double range_sum = descriptor.head(module_.NUM_RANGE).sum();
  const double angle_sum = descriptor.tail(module_.NUM_ANGLE).sum();
  return total_norm > 1e-12 && std::isfinite(range_sum) && std::isfinite(angle_sum) &&
         std::abs(range_sum - angle_sum) <= 1e-9 * std::max(1.0, range_sum);
}

SolidDescriptorResult SolidPlaceRecognitionAdapter::describe(
    const pcl::PointCloud<pcl::PointXYZ> &input) const
{
  const Clock::time_point start = Clock::now();
  SolidDescriptorResult result;
  result.input_points = input.size();

  pcl::PointCloud<pcl::PointXYZ> sensor_compatible;
  sensor_compatible.reserve(input.size());
  for (const pcl::PointXYZ &point : input.points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(static_cast<double>(point.x) * point.x +
                                   static_cast<double>(point.y) * point.y +
                                   static_cast<double>(point.z) * point.z);
    if (!(range > module_.MIN_DISTANCE && range < module_.MAX_DISTANCE)) continue;
    const double elevation_deg = std::atan2(static_cast<double>(point.z),
        std::hypot(static_cast<double>(point.x), static_cast<double>(point.y))) * 180.0 / M_PI;
    if (elevation_deg < module_.FOV_d || elevation_deg > module_.FOV_u) continue;
    sensor_compatible.push_back(point);
  }
  finalizeCloud(sensor_compatible);
  result.fov_range_points = sensor_compatible.size();
  if (sensor_compatible.size() < 50)
  {
    result.status = "TOO_FEW_SENSOR_COMPATIBLE_POINTS";
    result.descriptor_ms = elapsedMs(start);
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
  module_.down_sampling(sensor_compatible, downsampled);
  result.descriptor_points = downsampled->size();
  if (downsampled->size() < 50)
  {
    result.status = "TOO_FEW_POINTS_AFTER_UPSTREAM_VOXEL";
    result.descriptor_ms = elapsedMs(start);
    return result;
  }

  // SOLiD's upstream makeSolid normalizes the height histogram by
  // (max-min). Refuse degenerate histograms before calling it; this keeps the
  // published descriptor math unchanged while avoiding NaN/Inf propagation.
  Eigen::VectorXi height_counts = Eigen::VectorXi::Zero(module_.NUM_HEIGHT);
  const double gap_angle = 360.0 / module_.NUM_ANGLE;
  const double gap_range = static_cast<double>(module_.MAX_DISTANCE) / module_.NUM_RANGE;
  const double gap_height = (module_.FOV_u - module_.FOV_d) / module_.NUM_HEIGHT;
  for (pcl::PointXYZ point : downsampled->points)
  {
    const RAH bin = module_.pt2rah(point, static_cast<float>(gap_angle),
                                   static_cast<float>(gap_range),
                                   static_cast<float>(gap_height));
    if (bin.idx_height < 0 || bin.idx_height >= module_.NUM_HEIGHT)
    {
      result.status = "HEIGHT_BIN_OUT_OF_RANGE";
      result.descriptor_ms = elapsedMs(start);
      return result;
    }
    ++height_counts[bin.idx_height];
  }
  if (height_counts.maxCoeff() == height_counts.minCoeff())
  {
    result.status = "ZERO_HEIGHT_NORMALIZER_RANGE";
    result.descriptor_ms = elapsedMs(start);
    return result;
  }

  result.descriptor = module_.makeSolid(*downsampled);
  result.descriptor_ms = elapsedMs(start);
  if (!finiteDescriptor(result.descriptor, descriptorSize()))
  {
    result.descriptor.resize(0);
    result.status = "NONFINITE_OR_ZERO_DESCRIPTOR";
    return result;
  }
  result.valid = true;
  result.status = "OK";
  return result;
}

SolidQueryResult SolidPlaceRecognitionAdapter::query(
    const pcl::PointCloud<pcl::PointXYZ> &input, std::size_t top_k) const
{
  SolidQueryResult result;
  result.database_size = entries_.size();
  result.descriptor = describe(input);
  if (!result.descriptor.valid)
  {
    result.status = result.descriptor.status;
    return result;
  }
  if (entries_.empty())
  {
    result.status = "EMPTY_DATABASE";
    return result;
  }

  const Clock::time_point search_start = Clock::now();
  std::vector<SolidRankedCandidate> ranked;
  ranked.reserve(entries_.size());
  for (const SolidReferenceEntry &entry : entries_)
  {
    const double score = module_.loop_detection(result.descriptor.descriptor, entry.descriptor);
    if (!std::isfinite(score)) continue;
    SolidRankedCandidate candidate;
    candidate.id = entry.id;
    candidate.similarity = score;
    candidate.stamp_sec = entry.stamp_sec;
    candidate.map_T_reference = entry.map_T_reference;
    ranked.push_back(candidate);
  }
  result.scored_candidates = ranked.size();
  std::sort(ranked.begin(), ranked.end(), [](const SolidRankedCandidate &lhs,
                                              const SolidRankedCandidate &rhs)
  {
    if (lhs.similarity != rhs.similarity) return lhs.similarity > rhs.similarity;
    return lhs.id < rhs.id;
  });
  const std::size_t count = std::min(top_k, ranked.size());
  result.top.assign(ranked.begin(), ranked.begin() + count);
  result.search_ms = elapsedMs(search_start);
  result.status = result.top.empty() ? "NO_VALID_DATABASE_SCORES" : "OK";
  return result;
}

bool SolidPlaceRecognitionAdapter::writeDatabase(
    const std::string &path,
    const std::vector<SolidReferenceEntry> &entries,
    std::string &reason) const
{
  std::set<std::size_t> ids;
  if (entries.empty()) { reason = "empty_reference_entries"; return false; }
  for (const SolidReferenceEntry &entry : entries)
  {
    if (!std::isfinite(entry.stamp_sec) || !validRigidPose(entry.map_T_reference) ||
        !finiteDescriptor(entry.descriptor, descriptorSize()) || !ids.insert(entry.id).second)
    {
      reason = "invalid_reference_entry";
      return false;
    }
  }
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output.is_open())
  {
    reason = "cannot_open_output";
    return false;
  }
  output << kDatabaseMagic << ' ' << kUpstreamCommit << ' '
         << module_.FOV_u << ' ' << module_.FOV_d << ' '
         << module_.MAX_DISTANCE << ' ' << module_.NUM_ANGLE << ' '
         << module_.NUM_RANGE << ' ' << module_.NUM_HEIGHT << ' '
         << module_.MIN_DISTANCE << '\n';
  output << std::setprecision(17);
  for (const SolidReferenceEntry &entry : entries)
  {
    if (!std::isfinite(entry.stamp_sec) || !validRigidPose(entry.map_T_reference) ||
        !finiteDescriptor(entry.descriptor, descriptorSize()))
    {
      reason = "invalid_reference_entry";
      return false;
    }
    Eigen::Quaterniond orientation(entry.map_T_reference.block<3, 3>(0, 0));
    orientation.normalize();
    output << entry.id << ' ' << entry.stamp_sec << ' '
           << entry.map_T_reference(0, 3) << ' ' << entry.map_T_reference(1, 3) << ' '
           << entry.map_T_reference(2, 3) << ' '
           << orientation.x() << ' ' << orientation.y() << ' '
           << orientation.z() << ' ' << orientation.w();
    for (Eigen::Index i = 0; i < entry.descriptor.size(); ++i)
      output << ' ' << entry.descriptor[i];
    output << '\n';
  }
  output.flush();
  if (!output.good())
  {
    reason = "write_failed";
    return false;
  }
  reason = "ok";
  return true;
}

bool SolidPlaceRecognitionAdapter::loadDatabase(const std::string &path,
                                                std::string &reason)
{
  std::ifstream input(path);
  if (!input.is_open())
  {
    reason = "cannot_open_database";
    return false;
  }
  std::string header;
  if (!std::getline(input, header))
  {
    reason = "missing_database_header";
    return false;
  }
  std::istringstream header_stream(header);
  std::string magic;
  std::string upstream_commit;
  double fov_up = 0.0;
  double fov_down = 0.0;
  int max_distance = 0, num_angle = 0, num_range = 0, num_height = 0, min_distance = 0;
  if (!(header_stream >> magic >> upstream_commit >> fov_up >> fov_down >> max_distance >>
        num_angle >> num_range >> num_height >> min_distance))
  {
    reason = "malformed_database_header";
    return false;
  }
  std::string header_extra;
  if (header_stream >> header_extra)
  {
    reason = "extra_database_header_field";
    return false;
  }
  if (!std::isfinite(fov_up) || !std::isfinite(fov_down))
  {
    reason = "nonfinite_database_profile";
    return false;
  }
  if (magic != kDatabaseMagic || upstream_commit != kUpstreamCommit ||
      std::abs(fov_up - module_.FOV_u) > 1e-9 ||
      std::abs(fov_down - module_.FOV_d) > 1e-9 ||
      max_distance != module_.MAX_DISTANCE || num_angle != module_.NUM_ANGLE ||
      num_range != module_.NUM_RANGE || num_height != module_.NUM_HEIGHT ||
      min_distance != module_.MIN_DISTANCE)
  {
    reason = "database_contract_mismatch";
    return false;
  }

  std::vector<SolidReferenceEntry> loaded;
  std::set<std::size_t> ids;
  std::string line;
  std::size_t line_number = 1;
  while (std::getline(input, line))
  {
    ++line_number;
    if (line.empty()) continue;
    std::istringstream row(line);
    SolidReferenceEntry entry;
    double x = 0.0, y = 0.0, z = 0.0;
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (!(row >> entry.id >> entry.stamp_sec >> x >> y >> z >> qx >> qy >> qz >> qw))
    {
      reason = "malformed_database_row_" + std::to_string(line_number);
      return false;
    }
    entry.descriptor.resize(descriptorSize());
    for (int i = 0; i < descriptorSize(); ++i)
    {
      if (!(row >> entry.descriptor[i]))
      {
        reason = "short_descriptor_row_" + std::to_string(line_number);
        return false;
      }
    }
    std::string extra;
    if (row >> extra)
    {
      reason = "extra_database_field_" + std::to_string(line_number);
      return false;
    }
    const Eigen::Quaterniond orientation(qw, qx, qy, qz);
    if (!std::isfinite(entry.stamp_sec) || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(z) || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9 ||
        !finiteDescriptor(entry.descriptor, descriptorSize()) || !ids.insert(entry.id).second)
    {
      reason = "invalid_database_entry_" + std::to_string(line_number);
      return false;
    }
    entry.map_T_reference.setIdentity();
    entry.map_T_reference.block<3, 3>(0, 0) = orientation.normalized().toRotationMatrix();
    entry.map_T_reference.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    if (!validRigidPose(entry.map_T_reference))
    {
      reason = "invalid_database_pose_" + std::to_string(line_number);
      return false;
    }
    loaded.push_back(std::move(entry));
  }
  if (loaded.empty())
  {
    reason = "empty_database";
    return false;
  }
  entries_.swap(loaded);
  reason = "ok";
  return true;
}

}  // namespace dog_prior_map_localization
