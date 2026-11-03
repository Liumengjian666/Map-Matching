#pragma once

#include <pcl/point_cloud.h>
#include <cstdint>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>

namespace p63r14 {

constexpr double kBlockSizeMeters = 0.5;
constexpr uint64_t kSplitSeed = 0x5036335231340001ULL;

struct BlockKey {
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator<(const BlockKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct BlockCounts {
  uint64_t registration_points = 0;
  uint64_t validation_points = 0;
};

struct SplitCounts {
  uint64_t registration_points = 0;
  uint64_t validation_points = 0;
  uint64_t registration_blocks = 0;
  uint64_t validation_blocks = 0;
};

inline uint64_t splitmix64(uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

inline uint64_t rotate_left(uint64_t value, unsigned int amount) {
  return (value << amount) | (value >> (64 - amount));
}

inline BlockKey block_for(float x, float y, float z) {
  const double limit = static_cast<double>(std::numeric_limits<int64_t>::max()) * kBlockSizeMeters;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      std::fabs(static_cast<double>(x)) > limit ||
      std::fabs(static_cast<double>(y)) > limit ||
      std::fabs(static_cast<double>(z)) > limit) {
    throw std::runtime_error("non-finite or out-of-range source coordinate");
  }
  return {static_cast<int64_t>(std::floor(static_cast<double>(x) / kBlockSizeMeters)),
          static_cast<int64_t>(std::floor(static_cast<double>(y) / kBlockSizeMeters)),
          static_cast<int64_t>(std::floor(static_cast<double>(z) / kBlockSizeMeters))};
}

inline uint64_t block_hash(const BlockKey& key) {
  const uint64_t x = splitmix64(static_cast<uint64_t>(key.x));
  const uint64_t y = splitmix64(static_cast<uint64_t>(key.y));
  const uint64_t z = splitmix64(static_cast<uint64_t>(key.z));
  return splitmix64(kSplitSeed ^ x ^ rotate_left(y, 21) ^ rotate_left(z, 42));
}

template <typename PointT>
struct SplitClouds {
  using Cloud = pcl::PointCloud<PointT>;
  typename Cloud::Ptr registration{new Cloud};
  typename Cloud::Ptr validation{new Cloud};
  std::map<BlockKey, BlockCounts> blocks;
  SplitCounts counts;
};

template <typename PointT>
SplitClouds<PointT> split_cloud(const pcl::PointCloud<PointT>& source) {
  SplitClouds<PointT> result;
  result.registration->header = source.header;
  result.validation->header = source.header;
  result.registration->is_dense = source.is_dense;
  result.validation->is_dense = source.is_dense;
  result.registration->points.reserve(source.size() / 2);
  result.validation->points.reserve(source.size() - source.size() / 2);

  for (const auto& point : source.points) {
    const BlockKey key = block_for(point.x, point.y, point.z);
    BlockCounts& counts = result.blocks[key];
    if ((block_hash(key) & 1ULL) == 0ULL) {
      result.registration->points.push_back(point);
      ++counts.registration_points;
      ++result.counts.registration_points;
    } else {
      result.validation->points.push_back(point);
      ++counts.validation_points;
      ++result.counts.validation_points;
    }
  }

  for (const auto& entry : result.blocks) {
    if (entry.second.registration_points > 0 && entry.second.validation_points > 0) {
      throw std::runtime_error("one spatial block assigned to both subsets");
    }
    result.counts.registration_blocks += entry.second.registration_points > 0;
    result.counts.validation_blocks += entry.second.validation_points > 0;
  }
  if (result.counts.registration_points + result.counts.validation_points != source.size()) {
    throw std::runtime_error("split point count does not equal source point count");
  }
  result.registration->width = static_cast<uint32_t>(result.registration->size());
  result.registration->height = 1;
  result.validation->width = static_cast<uint32_t>(result.validation->size());
  result.validation->height = 1;
  return result;
}

}  // namespace p63r14
