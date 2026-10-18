#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <numeric>
#include <random>
#include <sstream>

namespace screen {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using FrozenNdt = dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<pcl::PointXYZ>;
inline void finish(Cloud::Ptr cloud) {
  cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;
}
// Exact production voxelDown/preprocess operations, without a runtime node.
inline Cloud::Ptr voxel(Cloud::Ptr cloud, float leaf, int cap) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(leaf, leaf, leaf); grid.setInputCloud(cloud); grid.filter(*out);
  if (cap > 0 && static_cast<int>(out->size()) > cap) {
    Cloud::Ptr sampled(new Cloud); sampled->reserve(cap);
    const double step = double(out->size() - 1) / std::max(cap - 1, 1);
    for (int i = 0; i < cap; ++i) sampled->push_back(out->points[static_cast<size_t>(std::round(i * step))]);
    out = sampled;
  }
  finish(out); return out;
}
inline Cloud::Ptr finite(Cloud::Ptr raw, bool source) {
  Cloud::Ptr out(new Cloud);
  for (const auto& p : *raw) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double r = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    if (source && (r < .5 || r > 80.)) continue;
    out->push_back(p);
  }
  finish(out); return out;
}
inline Cloud::Ptr source(Cloud::Ptr raw) { return voxel(finite(raw, true), .25f, 1400); }
inline Cloud::Ptr target(Cloud::Ptr raw) { return voxel(voxel(finite(raw, false), .15f, 0), .15f, 0); }
inline Cloud::Ptr crop(Cloud::Ptr full) {
  Cloud::Ptr out(new Cloud);
  for (const auto& p : *full) if (p.x >= 0 && std::abs(p.y) <= p.x) out->push_back(p);
  finish(out); return out;
}
inline Cloud::Ptr matched(Cloud::Ptr full, size_t count, unsigned index) {
  if (count > full->size()) throw std::runtime_error("matched count exceeds full source");
  std::vector<size_t> ids(full->size()); std::iota(ids.begin(), ids.end(), 0);
  std::mt19937 rng(6042026U + index); std::shuffle(ids.begin(), ids.end(), rng);
  ids.resize(count); std::sort(ids.begin(), ids.end());
  Cloud::Ptr out(new Cloud);
  for (auto id : ids) out->push_back(full->points[id]);
  finish(out); return out;
}
inline std::string hash(const Cloud::Ptr& cloud) {
  uint64_t value = 1469598103934665603ULL;
  auto byte = [&](uint8_t x) { value ^= x; value *= 1099511628211ULL; };
  auto u32 = [&](uint32_t x) { for (int s=0; s<32; s+=8) byte((x>>s)&255); };
  u32(cloud->width); u32(cloud->height); u32(cloud->is_dense ? 1 : 0); u32(cloud->size());
  for (const auto& p : *cloud) for (float x : {p.x,p.y,p.z}) {
    uint32_t bits; std::memcpy(&bits, &x, sizeof(bits)); u32(bits);
  }
  std::ostringstream text; text << std::hex << std::setw(16) << std::setfill('0') << value; return text.str();
}
class Ndt : public FrozenNdt {
 public:
  void configureTarget(const Cloud::Ptr& target) {
    // Preserve frozen R2 loadMap order. PCL1.10 retains 1m cells here because
    // setResolution() only rebuilds when input_ has already been assigned.
    setInputTarget(target);
    setResolution(.8f); setStepSize(.08); setTransformationEpsilon(.001); setMaximumIterations(40);
    if (getOulierRatio() != .55) throw std::runtime_error("unexpected native outlier default");
  }
  size_t supported(const Cloud::Ptr& source, const Eigen::Matrix4f& pose) {
    Cloud transformed; pcl::transformPointCloud(*source, transformed, pose);
    std::vector<TargetGrid::LeafConstPtr> leaves; std::vector<float> distances;
    size_t count = 0;
    for (const auto& p : transformed) {
      leaves.clear(); distances.clear();
      this->target_cells_.radiusSearch(p, this->resolution_, leaves, distances);
      if (!leaves.empty()) ++count;
    }
    return count;
  }
};
}  // namespace screen
