#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <unistd.h>
#include <vector>

#include "dog_prior_map_localization/solid_place_recognition.hpp"

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud makeStructuredMid360Cloud()
{
  Cloud cloud;
  const double fov_down = -7.0 * M_PI / 180.0;
  const double fov_span = 59.0 * M_PI / 180.0;
  for (int angle_deg = 0; angle_deg < 360; angle_deg += 12)
  {
    const double angle = angle_deg * M_PI / 180.0;
    for (int height = 0; height < 32; ++height)
    {
      const int repeats = 1 + height % 3;
      for (int repeat = 0; repeat < repeats; ++repeat)
      {
        const double elevation = fov_down + (height + 0.5) * fov_span / 32.0 +
                                 repeat * 0.15 * fov_span / 32.0;
        const double range = 5.0 + (angle_deg / 12 % 7) * 2.2 + repeat * 0.07;
        pcl::PointXYZ point;
        point.x = static_cast<float>(range * std::cos(elevation) * std::cos(angle));
        point.y = static_cast<float>(range * std::cos(elevation) * std::sin(angle));
        point.z = static_cast<float>(range * std::sin(elevation));
        cloud.push_back(point);
      }
    }
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

Cloud makeUniformHeightHistogramCloud()
{
  Cloud cloud;
  const double fov_down = -7.0 * M_PI / 180.0;
  const double fov_span = 59.0 * M_PI / 180.0;
  for (int angle_deg = 0; angle_deg < 360; angle_deg += 10)
  {
    const double angle = angle_deg * M_PI / 180.0;
    for (int height = 0; height < 32; ++height)
    {
      const double elevation = fov_down + (height + 0.5) * fov_span / 32.0;
      const double range = 36.0;
      pcl::PointXYZ point;
      point.x = static_cast<float>(range * std::cos(elevation) * std::cos(angle));
      point.y = static_cast<float>(range * std::cos(elevation) * std::sin(angle));
      point.z = static_cast<float>(range * std::sin(elevation));
      cloud.push_back(point);
    }
  }
  cloud.width = static_cast<uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}
}  // namespace

int main()
{
  dog_prior_map_localization::SolidPlaceRecognitionAdapter adapter;
  if (std::abs(adapter.fovUpDeg() - 52.0) > 1e-9 ||
      std::abs(adapter.fovDownDeg() + 7.0) > 1e-9 || adapter.maxDistanceM() != 40)
  {
    std::cerr << "MID-360 sensor profile mismatch\n";
    return 1;
  }

  const Cloud valid_cloud = makeStructuredMid360Cloud();
  const auto descriptor = adapter.describe(valid_cloud);
  if (!descriptor.valid || descriptor.status != "OK" || descriptor.descriptor.size() != 100 ||
      !descriptor.descriptor.allFinite() || descriptor.descriptor.norm() <= 1e-12)
  {
    std::cerr << "valid synthetic descriptor failed: " << descriptor.status
              << " points=" << descriptor.descriptor_points << "\n";
    return 2;
  }

  dog_prior_map_localization::SolidReferenceEntry entry;
  entry.id = 9;
  entry.stamp_sec = 123.5;
  entry.map_T_reference.setIdentity();
  entry.map_T_reference(0, 3) = 1.25;
  entry.descriptor = descriptor.descriptor;
  const std::string path = "/tmp/p11_solid_contract_" + std::to_string(getpid()) + ".db";
  std::string reason;
  if (!adapter.writeDatabase(path, std::vector<dog_prior_map_localization::SolidReferenceEntry>{entry}, reason))
  {
    std::cerr << "database write failed: " << reason << "\n";
    return 3;
  }
  dog_prior_map_localization::SolidPlaceRecognitionAdapter loaded;
  if (!loaded.loadDatabase(path, reason))
  {
    std::remove(path.c_str());
    std::cerr << "database load failed: " << reason << "\n";
    return 4;
  }
  std::remove(path.c_str());
  const auto result = loaded.query(valid_cloud, 1);
  if (result.status != "OK" || result.scored_candidates != 1 || result.top.size() != 1 ||
      result.top.front().id != 9 || std::abs(result.top.front().similarity - 1.0) > 1e-12 ||
      std::abs(result.top.front().map_T_reference(0, 3) - 1.25) > 1e-12)
  {
    std::cerr << "self retrieval/database round trip failed: " << result.status << "\n";
    return 5;
  }

  const auto degenerate = loaded.describe(makeUniformHeightHistogramCloud());
  if (degenerate.valid || degenerate.status != "ZERO_HEIGHT_NORMALIZER_RANGE")
  {
    std::cerr << "degenerate height histogram not guarded: " << degenerate.status << "\n";
    return 6;
  }

  Cloud invalid_cloud;
  pcl::PointXYZ nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = 0.0f;
  nan_point.z = 0.0f;
  invalid_cloud.push_back(nan_point);
  pcl::PointXYZ outside_fov;
  outside_fov.x = 5.0f;
  outside_fov.y = 0.0f;
  outside_fov.z = 10.0f;  // 63.4 degrees, outside the MID-360 profile.
  invalid_cloud.push_back(outside_fov);
  const auto invalid = loaded.describe(invalid_cloud);
  if (invalid.valid || invalid.status != "TOO_FEW_SENSOR_COMPATIBLE_POINTS" ||
      invalid.fov_range_points != 0)
  {
    std::cerr << "invalid point/FOV guard failed: " << invalid.status << "\n";
    return 7;
  }
  auto negative = entry;
  negative.descriptor[0] = -1.0;
  if (loaded.writeDatabase(path, {negative}, reason) ||
      reason != "invalid_reference_entry") return 8;
  if (loaded.writeDatabase(path, {entry, entry}, reason)) return 9;
  // A malformed profile must not replace an already valid loaded database.
  {
    std::ofstream malformed(path);
    malformed << "P11_SOLID_DB_V1 invalid_upstream 52 -7 40 60 40 32 3\n";
  }
  if (loaded.loadDatabase(path, reason) || loaded.databaseSize() != 1 ||
      reason != "database_contract_mismatch") return 10;
  std::remove(path.c_str());
  std::cout << "solid_adapter_contract_test=PASS valid_points=" << descriptor.descriptor_points
            << " descriptor_dim=" << descriptor.descriptor.size()
            << " self_similarity=" << result.top.front().similarity
            << " degenerate_guard=" << degenerate.status << "\n";
  return 0;
}
