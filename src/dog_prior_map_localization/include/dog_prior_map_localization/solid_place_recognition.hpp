#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <solid_module.h>

namespace dog_prior_map_localization
{

struct SolidReferenceEntry
{
  // Reference mapping-state pose in map coordinates, not a query NDT pose.
  std::size_t id = 0;
  double stamp_sec = 0.0;
  Eigen::Matrix4d map_T_reference = Eigen::Matrix4d::Identity();
  Eigen::VectorXd descriptor;
};

struct SolidRankedCandidate
{
  std::size_t id = 0;
  double similarity = 0.0;
  double stamp_sec = 0.0;
  Eigen::Matrix4d map_T_reference = Eigen::Matrix4d::Identity();
};

struct SolidDescriptorResult
{
  bool valid = false;
  std::string status = "NOT_RUN";
  std::size_t input_points = 0;
  std::size_t fov_range_points = 0;
  std::size_t descriptor_points = 0;
  Eigen::VectorXd descriptor;
  double descriptor_ms = 0.0;
};

struct SolidQueryResult
{
  SolidDescriptorResult descriptor;
  std::string status = "NOT_RUN";
  std::size_t database_size = 0;
  std::size_t scored_candidates = 0;
  double search_ms = 0.0;
  std::vector<SolidRankedCandidate> top;
};

// Thin sensor/database adapter around the upstream SOLiD C++ implementation.
// It contains no replacement descriptor math and is diagnostic-only in ROS.
class SolidPlaceRecognitionAdapter
{
public:
  static constexpr const char *kDatabaseMagic = "P11_SOLID_DB_V1";
  static constexpr const char *kUpstreamCommit = "9a4f2895e5211f69fbbd6d4732577e1a67e9b882";

  SolidDescriptorResult describe(const pcl::PointCloud<pcl::PointXYZ> &input) const;
  SolidQueryResult query(const pcl::PointCloud<pcl::PointXYZ> &input,
                         std::size_t top_k) const;

  bool loadDatabase(const std::string &path, std::string &reason);
  bool writeDatabase(const std::string &path,
                     const std::vector<SolidReferenceEntry> &entries,
                     std::string &reason) const;

  std::size_t databaseSize() const { return entries_.size(); }
  int descriptorSize() const { return module_.NUM_RANGE + module_.NUM_ANGLE; }
  double fovUpDeg() const { return module_.FOV_u; }
  double fovDownDeg() const { return module_.FOV_d; }
  int maxDistanceM() const { return module_.MAX_DISTANCE; }

private:
  bool finiteDescriptor(const Eigen::VectorXd &descriptor, int expected_size) const;

  // Upstream methods lack const qualifiers but keep no query-time state.
  mutable SOLiDModule module_;
  std::vector<SolidReferenceEntry> entries_;
};

}  // namespace dog_prior_map_localization
