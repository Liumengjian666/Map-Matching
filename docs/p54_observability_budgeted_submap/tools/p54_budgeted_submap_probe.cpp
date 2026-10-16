#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"

#include <Eigen/Eigenvalues>
#include <openssl/evp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

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
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<Point>;
using Clock = std::chrono::steady_clock;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
constexpr double kDescriptorLeafM = 0.8;
constexpr double kSupportRadiusM = 2.0;
constexpr double kSupportRadiusSq = kSupportRadiusM * kSupportRadiusM;
constexpr double kInformationEpsilon = 1e-4;

struct CsvTable
{
  std::vector<std::string> header;
  std::map<std::string, std::size_t> column;
  std::vector<std::vector<std::string>> rows;

  const std::string &get(const std::vector<std::string> &row, const std::string &key) const
  {
    const auto it = column.find(key);
    if (it == column.end() || it->second >= row.size())
      throw std::runtime_error("missing CSV column: " + key);
    return row[it->second];
  }
};

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> out;
  std::stringstream in(line);
  std::string value;
  while (std::getline(in, value, ','))
  {
    if (!value.empty() && value.back() == '\r') value.pop_back();
    out.push_back(value);
  }
  return out;
}

CsvTable readCsv(const std::string &path)
{
  std::ifstream in(path.c_str());
  if (!in) throw std::runtime_error("cannot open CSV: " + path);
  CsvTable t;
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty CSV: " + path);
  t.header = splitCsv(line);
  for (std::size_t i = 0; i < t.header.size(); ++i) t.column.emplace(t.header[i], i);
  while (std::getline(in, line))
  {
    if (line.empty()) continue;
    auto row = splitCsv(line);
    if (row.size() != t.header.size()) throw std::runtime_error("malformed CSV row: " + path);
    t.rows.push_back(std::move(row));
  }
  return t;
}

void requireNewFile(const std::string &path)
{
  std::ifstream existing(path.c_str(), std::ios::binary);
  if (existing.good()) throw std::runtime_error("refusing to overwrite existing output: " + path);
}

std::ofstream newOutput(const std::string &path)
{
  requireNewFile(path);
  std::ofstream out(path.c_str(), std::ios::out | std::ios::binary);
  if (!out) throw std::runtime_error("cannot create output: " + path);
  return out;
}

std::string sha256File(const std::string &path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input) throw std::runtime_error("cannot open SHA input: " + path);
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx) throw std::runtime_error("SHA context allocation failed");
  bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1;
  std::array<char, 1 << 20> buffer{};
  while (ok && input.good())
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto n = input.gcount();
    if (n > 0 && EVP_DigestUpdate(ctx, buffer.data(), static_cast<std::size_t>(n)) != 1) ok = false;
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  if (ok) ok = EVP_DigestFinal_ex(ctx, digest, &length) == 1 && length == 32;
  EVP_MD_CTX_free(ctx);
  if (!ok) throw std::runtime_error("SHA256 failed: " + path);
  static const char hex[] = "0123456789abcdef";
  std::string out;
  for (unsigned int i = 0; i < length; ++i)
  {
    out.push_back(hex[digest[i] >> 4]);
    out.push_back(hex[digest[i] & 15]);
  }
  return out;
}

uint64_t stableCloudHash(const Cloud::ConstPtr &cloud)
{
  constexpr uint64_t offset = 1469598103934665603ULL;
  constexpr uint64_t prime = 1099511628211ULL;
  uint64_t hash = offset;
  auto byte = [&](uint8_t b) { hash ^= b; hash *= prime; };
  auto u32 = [&](uint32_t v) { for (int s = 0; s < 32; s += 8) byte(static_cast<uint8_t>((v >> s) & 255U)); };
  u32(cloud->width); u32(cloud->height); u32(cloud->is_dense ? 1U : 0U);
  u32(static_cast<uint32_t>(cloud->size()));
  for (const auto &p : cloud->points)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &p.x, 4); u32(bits);
    std::memcpy(&bits, &p.y, 4); u32(bits);
    std::memcpy(&bits, &p.z, 4); u32(bits);
  }
  return hash;
}

std::string hex64(uint64_t v)
{
  std::ostringstream out;
  out << std::hex << std::setw(16) << std::setfill('0') << v;
  return out.str();
}

std::string sha256Cloud(const Cloud::ConstPtr &cloud)
{
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx || EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("could not initialize target SHA");
  for (const auto &p : cloud->points)
  {
    const float xyz[3] = {p.x, p.y, p.z};
    for (float v : xyz)
    {
      uint32_t bits = 0;
      std::memcpy(&bits, &v, sizeof(bits));
      const uint8_t le[4] = {static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
                             static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)};
      if (EVP_DigestUpdate(ctx, le, sizeof(le)) != 1)
      {
        EVP_MD_CTX_free(ctx);
        throw std::runtime_error("target SHA update failed");
      }
    }
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  const bool ok = EVP_DigestFinal_ex(ctx, digest, &length) == 1 && length == 32;
  EVP_MD_CTX_free(ctx);
  if (!ok) throw std::runtime_error("target SHA finalization failed");
  static const char hex[] = "0123456789abcdef";
  std::string out;
  for (unsigned int i = 0; i < length; ++i)
  {
    out.push_back(hex[digest[i] >> 4]);
    out.push_back(hex[digest[i] & 15]);
  }
  return out;
}

std::vector<std::string> splitSemicolon(const std::string &value)
{
  std::vector<std::string> out;
  std::stringstream in(value);
  std::string id;
  while (std::getline(in, id, ';')) if (!id.empty()) out.push_back(id);
  return out;
}

Eigen::Matrix4f poseFrom(const CsvTable &table, const std::vector<std::string> &row,
                         const std::string &prefix)
{
  const Eigen::Vector3f p(static_cast<float>(std::stod(table.get(row, prefix + "_tx"))),
                          static_cast<float>(std::stod(table.get(row, prefix + "_ty"))),
                          static_cast<float>(std::stod(table.get(row, prefix + "_tz"))));
  Eigen::Quaternionf q(static_cast<float>(std::stod(table.get(row, prefix + "_qw"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qx"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qy"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qz"))));
  if (!p.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-8f)
    throw std::runtime_error("invalid frozen pose in sample manifest");
  q.normalize();
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T.block<3, 1>(0, 3) = p;
  return T;
}

struct Tile
{
  std::string path, sha256;
  std::size_t points = 0;
};

std::map<std::string, Tile> readTileCatalog(const CsvTable &table)
{
  std::map<std::string, Tile> out;
  for (const auto &row : table.rows)
  {
    Tile t;
    t.path = table.get(row, "tile_path");
    t.sha256 = table.get(row, "sha256");
    t.points = static_cast<std::size_t>(std::stoull(table.get(row, "point_count")));
    const std::string id = table.get(row, "tile_id");
    if (!out.emplace(id, t).second) throw std::runtime_error("duplicate Tile ID: " + id);
  }
  return out;
}

struct VoxelKey
{
  int x, y, z;
  bool operator==(const VoxelKey &other) const
  { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelHash
{
  std::size_t operator()(const VoxelKey &k) const
  {
    uint64_t h = static_cast<uint32_t>(k.x) * 0x9e3779b185ebca87ULL;
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.y)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<uint64_t>(static_cast<uint32_t>(k.z)) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return static_cast<std::size_t>(h);
  }
};

struct Accumulator
{
  uint32_t count = 0;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
};

struct TileDescriptor
{
  Cloud::Ptr occupied{new Cloud()};
  Cloud::Ptr surfels{new Cloud()};
  std::vector<Eigen::Vector3f> normals;
  std::vector<float> planarity;
  std::unique_ptr<pcl::KdTreeFLANN<Point>> occupied_tree;
  std::unique_ptr<pcl::KdTreeFLANN<Point>> surfel_tree;
  std::size_t point_count = 0;
  std::size_t payload_bytes = 0;
};

TileDescriptor makeDescriptor(const std::string &id, const Tile &tile)
{
  if (sha256File(tile.path) != tile.sha256)
    throw std::runtime_error("P45 Tile SHA mismatch: " + id);
  Cloud::Ptr cloud(new Cloud());
  if (pcl::io::loadPCDFile(tile.path, *cloud) != 0 || cloud->size() != tile.points)
    throw std::runtime_error("P45 Tile load/count mismatch: " + id);

  std::unordered_map<VoxelKey, Accumulator, VoxelHash> voxels;
  voxels.reserve(cloud->size() / 3 + 1);
  for (const auto &p : cloud->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    VoxelKey key{static_cast<int>(std::floor(p.x / kDescriptorLeafM)),
                 static_cast<int>(std::floor(p.y / kDescriptorLeafM)),
                 static_cast<int>(std::floor(p.z / kDescriptorLeafM))};
    auto &a = voxels[key];
    const Eigen::Vector3d x(p.x, p.y, p.z);
    ++a.count;
    a.sum += x;
    a.second.noalias() += x * x.transpose();
  }

  TileDescriptor d;
  d.point_count = cloud->size();
  for (const auto &entry : voxels)
  {
    const Accumulator &a = entry.second;
    if (a.count < 3) continue;
    const Eigen::Vector3d mean = a.sum / static_cast<double>(a.count);
    Point center;
    center.x = static_cast<float>(mean.x());
    center.y = static_cast<float>(mean.y());
    center.z = static_cast<float>(mean.z());
    d.occupied->push_back(center);
    if (a.count < 6) continue;
    Eigen::Matrix3d cov = a.second / static_cast<double>(a.count) - mean * mean.transpose();
    cov = 0.5 * (cov + cov.transpose()).eval();
    if (!cov.allFinite()) continue;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
        eig.eigenvalues()[0] < -1e-8 || eig.eigenvalues()[1] <= 1e-6 || eig.eigenvalues()[2] <= 1e-6)
      continue;
    const double weight = std::max(0.0, std::min(1.0,
        1.0 - eig.eigenvalues()[0] / eig.eigenvalues()[1]));
    if (weight <= 1e-4) continue;
    d.surfels->push_back(center);
    d.normals.push_back(eig.eigenvectors().col(0).cast<float>().normalized());
    d.planarity.push_back(static_cast<float>(weight));
  }
  d.occupied->width = static_cast<uint32_t>(d.occupied->size());
  d.occupied->height = 1;
  d.occupied->is_dense = true;
  d.surfels->width = static_cast<uint32_t>(d.surfels->size());
  d.surfels->height = 1;
  d.surfels->is_dense = true;
  d.occupied_tree.reset(new pcl::KdTreeFLANN<Point>());
  if (!d.occupied->empty()) d.occupied_tree->setInputCloud(d.occupied);
  d.surfel_tree.reset(new pcl::KdTreeFLANN<Point>());
  if (!d.surfels->empty()) d.surfel_tree->setInputCloud(d.surfels);
  d.payload_bytes = d.occupied->size() * 3 * sizeof(float) +
      d.surfels->size() * (3 * sizeof(float) + 3 * sizeof(float) + sizeof(float));
  return d;
}

struct Match
{
  float occupancy_dist2 = std::numeric_limits<float>::infinity();
  float surfel_dist2 = std::numeric_limits<float>::infinity();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float weight = 0.0f;
};

struct TileFrameData
{
  std::string id;
  std::size_t point_count = 0;
  double mean_occupancy_distance = std::numeric_limits<double>::infinity();
  std::vector<Match> matches;
};

struct Information
{
  Matrix6d H = Matrix6d::Zero();
  Eigen::Matrix<double, 6, 1> eigen = Eigen::Matrix<double, 6, 1>::Zero();
  double utility = 0.0;
  bool valid = false;
};

Information informationFor(const std::vector<std::size_t> &selected,
                           const std::vector<TileFrameData> &tiles,
                           const std::vector<Eigen::Vector3f> &relative_points,
                           std::size_t source_count)
{
  Information info;
  if (source_count == 0) return info;
  for (std::size_t i = 0; i < source_count; ++i)
  {
    float best = static_cast<float>(kSupportRadiusSq);
    Eigen::Vector3f n = Eigen::Vector3f::Zero();
    float weight = 0.0f;
    bool found = false;
    for (const std::size_t ti : selected)
    {
      const Match &m = tiles[ti].matches[i];
      if (m.surfel_dist2 <= best && m.weight > 0.0f)
      {
        best = m.surfel_dist2;
        n = m.normal;
        weight = m.weight;
        found = true;
      }
    }
    if (!found) continue;
    const Eigen::Vector3d nd = n.cast<double>();
    const Eigen::Vector3d r = relative_points[i].cast<double>();
    Eigen::Matrix<double, 1, 6> J;
    J.template head<3>() = (0.8 * nd).transpose();
    J.template tail<3>() = r.cross(nd).transpose();
    info.H.noalias() += static_cast<double>(weight) * J.transpose() * J /
        static_cast<double>(source_count);
  }
  info.H = 0.5 * (info.H + info.H.transpose()).eval();
  if (!info.H.allFinite()) return info;
  Eigen::SelfAdjointEigenSolver<Matrix6d> eig(info.H);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return info;
  info.eigen = eig.eigenvalues();
  if (info.eigen[0] < -1e-8) return info;
  const double lo = std::max(0.0, info.eigen[0]);
  const double hi = std::max(0.0, info.eigen[5]);
  double logdet = 0.0;
  for (int k = 0; k < 6; ++k)
    logdet += std::log1p(std::max(0.0, info.eigen[k]) / kInformationEpsilon);
  info.utility = 0.5 * logdet / 6.0 + 0.5 * lo / (hi + kInformationEpsilon);
  info.valid = std::isfinite(info.utility);
  return info;
}

std::size_t coverageCount(const std::vector<std::size_t> &selected,
                          const std::vector<TileFrameData> &tiles,
                          std::size_t source_count)
{
  std::size_t covered = 0;
  for (std::size_t i = 0; i < source_count; ++i)
  {
    float best = std::numeric_limits<float>::infinity();
    for (const auto ti : selected) best = std::min(best, tiles[ti].matches[i].occupancy_dist2);
    if (best <= kSupportRadiusSq) ++covered;
  }
  return covered;
}

struct Selection
{
  std::vector<std::size_t> indices;
  std::size_t points = 0;
  std::size_t covered = 0;
  std::size_t fallback_count = 0;
  Information information;
  std::string status;
};

Selection selectTiles(const std::vector<TileFrameData> &tiles,
                      const std::vector<Eigen::Vector3f> &relative_points,
                      std::size_t source_count, std::size_t budget,
                      bool information_aware, std::size_t full_target_points)
{
  Selection out;
  if (full_target_points <= budget)
  {
    for (std::size_t i = 0; i < tiles.size(); ++i) out.indices.push_back(i);
    out.points = full_target_points;
    out.covered = coverageCount(out.indices, tiles, source_count);
    out.information = informationFor(out.indices, tiles, relative_points, source_count);
    out.status = out.covered * 100 >= source_count * 95 ? "FULL_TARGET_WITHIN_BUDGET" : "FULL_TARGET_COVERAGE_BELOW_95";
    return out;
  }

  std::vector<bool> chosen(tiles.size(), false);
  auto canAdd = [&](std::size_t i) {
    return !chosen[i] && tiles[i].point_count <= budget - out.points;
  };
  const std::size_t target_coverage = static_cast<std::size_t>(std::ceil(0.95 * source_count));
  while (out.covered < target_coverage)
  {
    std::size_t best = tiles.size();
    double best_score = -std::numeric_limits<double>::infinity();
    double best_cov_eff = -1.0;
    Information current_info;
    if (information_aware)
      current_info = informationFor(out.indices, tiles, relative_points, source_count);
    for (std::size_t i = 0; i < tiles.size(); ++i)
    {
      if (!canAdd(i)) continue;
      auto trial = out.indices;
      trial.push_back(i);
      const std::size_t trial_coverage = coverageCount(trial, tiles, source_count);
      if (trial_coverage <= out.covered) continue;
      const double cost = static_cast<double>(tiles[i].point_count);
      const double cov_eff = static_cast<double>(trial_coverage - out.covered) / cost;
      double score = cov_eff;
      if (information_aware)
      {
        const Information trial_info = informationFor(trial, tiles, relative_points, source_count);
        const double gain = trial_info.valid && current_info.valid ?
            trial_info.utility - current_info.utility : 0.0;
        score = gain > 0.0 ? gain / cost : -1.0;
        if (best == tiles.size() && score <= 0.0) { /* selected below by deterministic coverage fallback */ }
      }
      const double dist = tiles[i].mean_occupancy_distance;
      bool take = false;
      if (best == tiles.size()) take = true;
      else if (information_aware)
      {
        if (score > best_score + 1e-15) take = true;
        else if (std::abs(score - best_score) <= 1e-15 && cov_eff > best_cov_eff + 1e-15) take = true;
        else if (std::abs(score - best_score) <= 1e-15 && std::abs(cov_eff - best_cov_eff) <= 1e-15 &&
                 tiles[i].id < tiles[best].id) take = true;
      }
      else
      {
        if (cov_eff > best_score + 1e-15) take = true;
        else if (std::abs(cov_eff - best_score) <= 1e-15 && dist < tiles[best].mean_occupancy_distance) take = true;
        else if (std::abs(cov_eff - best_score) <= 1e-15 && dist == tiles[best].mean_occupancy_distance &&
                 tiles[i].id < tiles[best].id) take = true;
      }
      if (take)
      {
        best = i;
        best_score = information_aware ? score : cov_eff;
        best_cov_eff = cov_eff;
      }
    }
    if (best == tiles.size())
    {
      out.status = "COVERAGE_PROXY_INFEASIBLE_BUDGET";
      break;
    }
    if (information_aware && best_score <= 0.0) ++out.fallback_count;
    chosen[best] = true;
    out.indices.push_back(best);
    out.points += tiles[best].point_count;
    out.covered = coverageCount(out.indices, tiles, source_count);
  }

  if (out.covered >= target_coverage)
  {
    while (true)
    {
      std::size_t best = tiles.size();
      double best_score = -std::numeric_limits<double>::infinity();
      Information current_info;
      if (information_aware) current_info = informationFor(out.indices, tiles, relative_points, source_count);
      for (std::size_t i = 0; i < tiles.size(); ++i)
      {
        if (!canAdd(i)) continue;
        double score = -std::numeric_limits<double>::infinity();
        if (information_aware)
        {
          auto trial = out.indices;
          trial.push_back(i);
          const Information trial_info = informationFor(trial, tiles, relative_points, source_count);
          const double gain = trial_info.valid && current_info.valid ? trial_info.utility - current_info.utility : 0.0;
          if (gain > 1e-12) score = gain / static_cast<double>(tiles[i].point_count);
        }
        else
        {
          score = 1.0 / (1.0 + tiles[i].mean_occupancy_distance) /
              static_cast<double>(tiles[i].point_count);
        }
        if (score > best_score + 1e-18 ||
            (std::abs(score - best_score) <= 1e-18 && best != tiles.size() && tiles[i].id < tiles[best].id))
        {
          best = i;
          best_score = score;
        }
      }
      if (best == tiles.size()) break;
      chosen[best] = true;
      out.indices.push_back(best);
      out.points += tiles[best].point_count;
      out.covered = coverageCount(out.indices, tiles, source_count);
    }
  }
  std::sort(out.indices.begin(), out.indices.end(), [&](std::size_t a, std::size_t b) { return tiles[a].id < tiles[b].id; });
  out.information = informationFor(out.indices, tiles, relative_points, source_count);
  if (out.covered >= target_coverage) out.status = "PROXY_COVERAGE_AT_LEAST_95";
  if (out.points > budget) throw std::runtime_error("internal selector exceeded point budget");
  return out;
}

std::string joinTileIds(const std::vector<std::size_t> &indices,
                        const std::vector<TileFrameData> &tiles)
{
  std::ostringstream out;
  bool first = true;
  for (const auto i : indices)
  {
    if (!first) out << ';';
    first = false;
    out << tiles[i].id;
  }
  return out.str();
}

Cloud::Ptr loadTarget(const std::vector<std::string> &ids,
                      const std::map<std::string, Tile> &catalog,
                      std::size_t expected_points)
{
  if (ids.empty()) throw std::runtime_error("empty selected Tile list");
  Cloud::Ptr target(new Cloud());
  std::size_t sum = 0;
  for (const auto &id : ids)
  {
    const auto it = catalog.find(id);
    if (it == catalog.end()) throw std::runtime_error("selected Tile absent from catalog: " + id);
    if (sha256File(it->second.path) != it->second.sha256)
      throw std::runtime_error("P45 Tile SHA mismatch during Target build: " + id);
    Cloud tile;
    if (pcl::io::loadPCDFile(it->second.path, tile) != 0 || tile.size() != it->second.points)
      throw std::runtime_error("P45 Tile load/count mismatch during Target build: " + id);
    sum += tile.size();
    *target += tile;
  }
  target->width = static_cast<uint32_t>(target->size());
  target->height = 1;
  target->is_dense = true;
  if (target->size() != expected_points || sum != expected_points)
    throw std::runtime_error("selected target point count mismatch");
  return target;
}

struct Coverage
{
  double median = std::numeric_limits<double>::quiet_NaN();
  double p90 = std::numeric_limits<double>::quiet_NaN();
  double overlap_0p5 = 0.0, overlap_2 = 0.0, overlap_8 = 0.0;
  std::size_t support_points_2m = 0;
};

Coverage exactCoverage(const Cloud::ConstPtr &target, const Cloud::ConstPtr &source,
                       const Eigen::Matrix4f &pose)
{
  Coverage c;
  if (!target || target->empty() || !source || source->empty()) return c;
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(target);
  std::vector<float> distances;
  distances.reserve(source->size());
  std::size_t a = 0, b = 0, d = 0;
  std::vector<int> index(1);
  std::vector<float> sqdist(1);
  for (const auto &p : source->points)
  {
    Point q;
    q.x = pose(0,0)*p.x + pose(0,1)*p.y + pose(0,2)*p.z + pose(0,3);
    q.y = pose(1,0)*p.x + pose(1,1)*p.y + pose(1,2)*p.z + pose(1,3);
    q.z = pose(2,0)*p.x + pose(2,1)*p.y + pose(2,2)*p.z + pose(2,3);
    if (tree.nearestKSearch(q, 1, index, sqdist) > 0)
    {
      const double r = std::sqrt(std::max(0.0f, sqdist[0]));
      distances.push_back(static_cast<float>(r));
      if (r <= 0.5) ++a;
      if (r <= 2.0) ++b;
      if (r <= 8.0) ++d;
    }
    else distances.push_back(std::numeric_limits<float>::infinity());
  }
  std::sort(distances.begin(), distances.end());
  const std::size_t n = distances.size();
  c.median = distances[static_cast<std::size_t>(0.5 * (n - 1))];
  c.p90 = distances[static_cast<std::size_t>(0.9 * (n - 1))];
  c.overlap_0p5 = static_cast<double>(a) / n;
  c.overlap_2 = static_cast<double>(b) / n;
  c.overlap_8 = static_cast<double>(d) / n;
  c.support_points_2m = b;
  return c;
}

std::vector<std::string> activeIds(const CsvTable &samples, const std::vector<std::string> &row)
{
  auto ids = splitSemicolon(samples.get(row, "active_tile_ids"));
  if (ids.empty() || !std::is_sorted(ids.begin(), ids.end()) ||
      std::adjacent_find(ids.begin(), ids.end()) != ids.end())
    throw std::runtime_error("P46 active Tile IDs are empty, duplicated or unsorted");
  return ids;
}

std::vector<TileFrameData> buildFrameTileQueries(
    const std::vector<std::string> &ids, const std::map<std::string, Tile> &catalog,
    const std::unordered_map<std::string, TileDescriptor> &descriptors,
    const Cloud::ConstPtr &source, const Eigen::Matrix4f &guess,
    std::vector<Eigen::Vector3f> *relative_points)
{
  Cloud projected;
  pcl::transformPointCloud(*source, projected, guess);
  const Eigen::Matrix3f R = guess.block<3,3>(0,0);
  relative_points->clear();
  relative_points->reserve(source->size());
  for (const auto &p : source->points) relative_points->push_back(R * Eigen::Vector3f(p.x,p.y,p.z));

  std::vector<TileFrameData> result;
  result.reserve(ids.size());
  std::vector<int> index(1);
  std::vector<float> sqdist(1);
  for (const auto &id : ids)
  {
    const auto cat = catalog.find(id);
    const auto desc = descriptors.find(id);
    if (cat == catalog.end() || desc == descriptors.end()) throw std::runtime_error("missing active Tile descriptor: " + id);
    TileFrameData td;
    td.id = id;
    td.point_count = cat->second.points;
    td.matches.resize(source->size());
    double distance_sum = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i)
    {
      Match &m = td.matches[i];
      if (desc->second.occupied_tree && !desc->second.occupied->empty() &&
          desc->second.occupied_tree->nearestKSearch(projected.points[i], 1, index, sqdist) > 0)
      {
        m.occupancy_dist2 = sqdist[0];
        distance_sum += std::sqrt(std::max(0.0f, sqdist[0]));
      }
      if (desc->second.surfel_tree && !desc->second.surfels->empty() &&
          desc->second.surfel_tree->nearestKSearch(projected.points[i], 1, index, sqdist) > 0)
      {
        m.surfel_dist2 = sqdist[0];
        const std::size_t j = static_cast<std::size_t>(index[0]);
        m.normal = desc->second.normals.at(j);
        m.weight = desc->second.planarity.at(j);
      }
    }
    td.mean_occupancy_distance = source->empty() ? std::numeric_limits<double>::infinity() :
        distance_sum / static_cast<double>(source->size());
    result.push_back(std::move(td));
  }
  return result;
}

std::string eigenText(const Information &info)
{
  std::ostringstream out;
  out << std::setprecision(10);
  for (int i = 0; i < 6; ++i) { if (i) out << ';'; out << info.eigen[i]; }
  return out.str();
}

std::string csvQuote(const std::string &s)
{
  if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
  std::string out = "\"";
  for (char c : s) { if (c == '"') out += "\"\""; else out += c; }
  out += '"';
  return out;
}

int selectMode(int argc, char **argv)
{
  if (argc != 6) throw std::runtime_error("usage: p54_probe select SAMPLE_MANIFEST TILE_CATALOG SOURCE_DIR OUTPUT_DIR");
  const std::string sample_path = argv[2], catalog_path = argv[3], source_dir = argv[4], out_dir = argv[5];
  const CsvTable samples = readCsv(sample_path);
  const CsvTable catalog_csv = readCsv(catalog_path);
  const auto catalog = readTileCatalog(catalog_csv);
  if (samples.rows.size() != 154) throw std::runtime_error("P54 requires exactly the frozen P47 154 rows");
  std::set<std::string> active_union;
  for (const auto &r : samples.rows) for (const auto &id : activeIds(samples, r)) active_union.insert(id);

  const auto desc_start = Clock::now();
  std::unordered_map<std::string, TileDescriptor> descriptors;
  std::size_t descriptor_payload_bytes = 0;
  for (const auto &id : active_union)
  {
    const auto it = catalog.find(id);
    if (it == catalog.end()) throw std::runtime_error("active P46 Tile absent from P47 catalog: " + id);
    auto d = makeDescriptor(id, it->second);
    descriptor_payload_bytes += d.payload_bytes;
    descriptors.emplace(id, std::move(d));
  }
  const double descriptor_ms = std::chrono::duration<double, std::milli>(Clock::now() - desc_start).count();
  const std::string selection_path = out_dir + "/selection_manifest.csv";
  const std::string info_path = out_dir + "/information_metrics.csv";
  auto selection_out = newOutput(selection_path);
  auto info_out = newOutput(info_path);
  selection_out << "frame_index,scan_ns,budget_points,method,active_tile_count,active_points,selected_tile_count,selected_points,missing_tile_count,active_tile_ids,selected_tile_ids,dropped_tile_ids,descriptor_coverage,mean_tile_distance_m,info_lambda_min,info_lambda_max,info_ratio,info_utility,selection_ms,descriptor_query_ms,info_coverage_fallback_count,status,source_hash,source_file_sha256\n";
  info_out << "frame_index,scan_ns,method,budget_points,source_points,active_points,selected_points,active_tile_count,selected_tile_count,coverage_proxy,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,lambda_min_max_ratio,trace,utility,descriptor_payload_bytes,descriptor_ms,selection_ms,selection_status\n";

  std::size_t processed = 0;
  for (const auto &row : samples.rows)
  {
    const std::string frame = samples.get(row, "frame_index");
    const std::string scan_ns = samples.get(row, "scan_ns");
    const std::string source_path = source_dir + "/source_frame_" + frame + ".pcd";
    if (sha256File(samples.get(row, "pcd_path")) != samples.get(row, "pcd_sha256"))
      throw std::runtime_error("raw scan PCD SHA mismatch at frame " + frame);
    Cloud::Ptr source(new Cloud());
    if (pcl::io::loadPCDFile(source_path, *source) != 0 || source->size() != 1400)
      throw std::runtime_error("frozen Source missing or wrong count at frame " + frame);
    if (hex64(stableCloudHash(source)) != samples.get(row, "expected_source_hash"))
      throw std::runtime_error("frozen P47 Source stable hash mismatch at frame " + frame);
    const auto ids = activeIds(samples, row);
    std::size_t full_points = 0;
    for (const auto &id : ids) full_points += catalog.at(id).points;
    if (full_points != std::stoull(samples.get(row, "target_points")))
      throw std::runtime_error("P46 active Target point count mismatch at frame " + frame);
    const Eigen::Matrix4f guess = poseFrom(samples, row, "initial");
    std::vector<Eigen::Vector3f> relative_points;
    const auto query_start = Clock::now();
    auto frame_tiles = buildFrameTileQueries(ids, catalog, descriptors, source, guess, &relative_points);
    const double query_ms = std::chrono::duration<double, std::milli>(Clock::now() - query_start).count();

    for (const std::size_t budget : {1200000U, 1500000U})
    {
      for (const bool info_method : {false, true})
      {
        const auto select_start = Clock::now();
        Selection selected = selectTiles(frame_tiles, relative_points, source->size(), budget, info_method, full_points);
        const double select_ms = std::chrono::duration<double, std::milli>(Clock::now() - select_start).count();
        std::set<std::size_t> selected_set(selected.indices.begin(), selected.indices.end());
        std::vector<std::size_t> dropped_idx;
        for (std::size_t i = 0; i < frame_tiles.size(); ++i) if (!selected_set.count(i)) dropped_idx.push_back(i);
        const std::string active_text = [&]() { std::ostringstream x; for (std::size_t i=0;i<ids.size();++i){if(i)x<<';';x<<ids[i];} return x.str(); }();
        const std::string selected_text = joinTileIds(selected.indices, frame_tiles);
        const std::string dropped_text = joinTileIds(dropped_idx, frame_tiles);
        const double coverage = static_cast<double>(selected.covered) / source->size();
        const double ratio = selected.information.eigen[5] > 0.0 ?
            std::max(0.0, selected.information.eigen[0]) / selected.information.eigen[5] : 0.0;
        const double trace = selected.information.H.trace();
        const std::string method = info_method ? "INFORMATION_AWARE" : "DISTANCE_BASED";
        double selected_mean_distance = 0.0;
        for (const auto ti : selected.indices) selected_mean_distance += frame_tiles[ti].mean_occupancy_distance;
        if (!selected.indices.empty()) selected_mean_distance /= static_cast<double>(selected.indices.size());
        const std::string source_sha = sha256File(source_path);
        selection_out << frame << ',' << scan_ns << ',' << budget << ',' << method << ',' << ids.size() << ','
            << full_points << ',' << selected.indices.size() << ',' << selected.points << ',' << dropped_idx.size() << ','
            << csvQuote(active_text) << ',' << csvQuote(selected_text) << ',' << csvQuote(dropped_text) << ','
            << std::setprecision(10) << coverage << ',' << selected_mean_distance << ','
            << selected.information.eigen[0] << ','
            << selected.information.eigen[5] << ',' << ratio << ',' << selected.information.utility << ','
            << select_ms << ',' << query_ms << ',' << selected.fallback_count << ',' << selected.status << ','
            << samples.get(row, "expected_source_hash") << ',' << source_sha << '\n';
        info_out << frame << ',' << scan_ns << ',' << method << ',' << budget << ',' << source->size() << ','
            << full_points << ',' << selected.points << ',' << ids.size() << ',' << selected.indices.size() << ','
            << std::setprecision(10) << coverage;
        for (int k = 0; k < 6; ++k) info_out << ',' << selected.information.eigen[k];
        info_out << ',' << ratio << ',' << trace << ',' << selected.information.utility << ','
            << descriptor_payload_bytes << ',' << descriptor_ms << ',' << select_ms << ',' << selected.status << '\n';
      }
    }
    ++processed;
    if (processed % 10 == 0) std::cout << "P54_SELECT_PROGRESS frames=" << processed << "/" << samples.rows.size() << '\n';
  }
  std::cout << "P54_SELECTION_COMPLETE frames=" << processed << " descriptors=" << descriptors.size()
            << " descriptor_payload_bytes=" << descriptor_payload_bytes << " descriptor_ms=" << descriptor_ms << '\n';
  return 0;
}

struct SelectionRow
{
  std::string frame, stamp, method, active_ids, selected_ids, status, source_hash;
  std::size_t budget = 0, active_points = 0, selected_points = 0;
  double proxy_coverage = 0.0;
};

std::vector<SelectionRow> readSelections(const CsvTable &table)
{
  std::vector<SelectionRow> out;
  out.reserve(table.rows.size());
  for (const auto &r : table.rows)
  {
    SelectionRow s;
    s.frame = table.get(r, "frame_index"); s.stamp = table.get(r, "scan_ns");
    s.method = table.get(r, "method"); s.active_ids = table.get(r, "active_tile_ids");
    s.selected_ids = table.get(r, "selected_tile_ids"); s.status = table.get(r, "status");
    s.source_hash = table.get(r, "source_hash");
    s.budget = std::stoull(table.get(r, "budget_points"));
    s.active_points = std::stoull(table.get(r, "active_points"));
    s.selected_points = std::stoull(table.get(r, "selected_points"));
    s.proxy_coverage = std::stod(table.get(r, "descriptor_coverage"));
    out.push_back(std::move(s));
  }
  return out;
}

double poseTranslationDiff(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{ return (a.block<3,1>(0,3) - b.block<3,1>(0,3)).cast<double>().norm(); }

double poseRotationDiffDeg(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{
  const Eigen::Quaterniond qa(a.block<3,3>(0,0).cast<double>());
  const Eigen::Quaterniond qb(b.block<3,3>(0,0).cast<double>());
  return qa.normalized().angularDistance(qb.normalized()) * 180.0 / std::acos(-1.0);
}

bool validRigid(const Eigen::Matrix4f &T)
{
  if (!T.allFinite() || (T.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-4f) return false;
  const Eigen::Matrix3f R = T.block<3,3>(0,0);
  return (R.transpose()*R - Eigen::Matrix3f::Identity()).norm() <= 1e-3f &&
      std::abs(R.determinant()-1.0f) <= 1e-3f;
}

int alignMode(int argc, char **argv)
{
  if (argc != 8) throw std::runtime_error("usage: p54_probe align SAMPLE_MANIFEST TILE_CATALOG SOURCE_DIR SELECTION_CSV P46_GEOMETRY_CSV OUTPUT_DIR");
  const std::string sample_path = argv[2], catalog_path = argv[3], source_dir = argv[4];
  const std::string selection_path = argv[5], geometry_path = argv[6], out_dir = argv[7];
  const CsvTable samples = readCsv(sample_path), selections_csv = readCsv(selection_path),
      catalog_csv = readCsv(catalog_path), geometry = readCsv(geometry_path);
  if (samples.rows.size() != 154 || selections_csv.rows.size() != 616)
    throw std::runtime_error("align stage requires exactly 154 frozen samples x 4 selection rows");
  const auto catalog = readTileCatalog(catalog_csv);
  const auto selections = readSelections(selections_csv);
  auto out = newOutput(out_dir + "/align_results.csv");
  auto coverage_out = newOutput(out_dir + "/coverage_comparison.csv");
  auto runtime_out = newOutput(out_dir + "/runtime_comparison.csv");
  out << "frame_index,scan_ns,budget_points,method,active_points,selected_points,active_tile_count,selected_tile_count,missing_tile_count,selection_status,proxy_coverage,exact_initial_median_m,exact_initial_p90_m,exact_initial_overlap_0p5m,exact_initial_overlap_2m,exact_initial_overlap_8m,support_source_points_2m,exact_coverage_pass,align_called,candidate_valid,converged,fitness,iterations,align_ms,initial_tx,initial_ty,initial_tz,initial_qx,initial_qy,initial_qz,initial_qw,raw_tx,raw_ty,raw_tz,raw_qx,raw_qy,raw_qz,raw_qw,raw_vs_p46_translation_m,raw_vs_p46_rotation_deg,target_sha256,source_sha256,status\n";
  coverage_out << "frame_index,scan_ns,budget_points,method,baseline_p46_initial_overlap_2m,descriptor_coverage,exact_selected_overlap_0p5m,exact_selected_overlap_2m,exact_selected_overlap_8m,exact_selected_median_m,exact_selected_p90_m,support_source_points_2m,missing_tile_count,missing_tile_ids,selected_points,coverage_gate_status\n";
  runtime_out << "frame_index,scan_ns,budget_points,method,selected_points,target_load_merge_ms,exact_coverage_ms,ndt_align_ms,align_called,converged,iterations,status\n";

  std::unordered_map<std::string, std::size_t> sample_idx;
  for (std::size_t i = 0; i < samples.rows.size(); ++i) sample_idx.emplace(samples.get(samples.rows[i], "frame_index"), i);
  std::map<std::string, double> baseline_coverage;
  for (const auto &r : geometry.rows)
    baseline_coverage.emplace(geometry.get(r, "scan_ns"), std::stod(geometry.get(r, "initial_overlap_2m")));
  std::size_t completed = 0;
  for (const auto &s : selections)
  {
    const auto sit = sample_idx.find(s.frame);
    if (sit == sample_idx.end()) throw std::runtime_error("selection frame not in frozen P47 sample set");
    const auto &row = samples.rows[sit->second];
    if (s.stamp != samples.get(row, "scan_ns") || s.source_hash != samples.get(row, "expected_source_hash"))
      throw std::runtime_error("selection/source/timestamp mismatch at frame " + s.frame);
    const auto base_cov = baseline_coverage.find(s.stamp);
    if (base_cov == baseline_coverage.end()) throw std::runtime_error("P46 initial coverage row missing for scan " + s.stamp);
    const auto active = splitSemicolon(s.active_ids), selected = splitSemicolon(s.selected_ids);
    if (selected.empty() || !std::is_sorted(selected.begin(), selected.end()) ||
        std::adjacent_find(selected.begin(), selected.end()) != selected.end())
      throw std::runtime_error("selected Tile list is empty/unsorted/duplicated at frame " + s.frame);
    for (const auto &id : selected)
      if (!std::binary_search(active.begin(), active.end(), id))
        throw std::runtime_error("selector used Tile outside P46 active set: " + id);
    if (s.selected_points > s.budget) throw std::runtime_error("selection exceeds point budget");

    const std::string source_path = source_dir + "/source_frame_" + s.frame + ".pcd";
    Cloud::Ptr source(new Cloud());
    if (pcl::io::loadPCDFile(source_path, *source) != 0 || source->size() != 1400 ||
        hex64(stableCloudHash(source)) != s.source_hash)
      throw std::runtime_error("frozen Source parity failed at frame " + s.frame);
    const Eigen::Matrix4f initial = poseFrom(samples, row, "initial");
    const Eigen::Matrix4f p46_nominal = poseFrom(samples, row, "nominal");

    const auto load_start = Clock::now();
    Cloud::Ptr target = loadTarget(selected, catalog, s.selected_points);
    const std::string target_sha = sha256Cloud(target);
    const double load_ms = std::chrono::duration<double, std::milli>(Clock::now() - load_start).count();
    const auto coverage_start = Clock::now();
    const Coverage cov = exactCoverage(target, source, initial);
    const double coverage_ms = std::chrono::duration<double, std::milli>(Clock::now() - coverage_start).count();
    const bool coverage_pass = cov.overlap_2 + 1e-12 >= 0.95;
    bool align_called = false, candidate_valid = false, converged = false;
    double fitness = std::numeric_limits<double>::quiet_NaN(), align_ms = 0.0;
    int iterations = 0;
    Eigen::Matrix4f raw = Eigen::Matrix4f::Constant(std::numeric_limits<float>::quiet_NaN());
    std::string status = coverage_pass ? "PENDING_NDT" : "COVERAGE_INFEASIBLE_EXACT_NO_ALIGN";
    if (coverage_pass)
    {
      Ndt ndt;
      ndt.setResolution(0.8f);
      ndt.setStepSize(0.08);
      ndt.setTransformationEpsilon(0.001);
      ndt.setMaximumIterations(40);
      ndt.setOulierRatio(0.55);
      ndt.setInputTarget(target);
      ndt.setInputSource(source);
      Cloud aligned;
      const auto align_start = Clock::now();
      ndt.align(aligned, initial);
      align_ms = std::chrono::duration<double, std::milli>(Clock::now() - align_start).count();
      align_called = true;
      converged = ndt.hasConverged();
      fitness = ndt.getFitnessScore();
      iterations = ndt.getFinalNumIteration();
      raw = ndt.getFinalTransformation();
      candidate_valid = validRigid(raw);
      status = converged && candidate_valid ? "NDT_CONVERGED" :
          (candidate_valid ? "NDT_NOT_CONVERGED" : "NDT_FAILED_OR_NONRIGID");
    }
    const double p46_dt = raw.allFinite() ? poseTranslationDiff(raw, p46_nominal) : std::numeric_limits<double>::quiet_NaN();
    const double p46_dr = raw.allFinite() ? poseRotationDiffDeg(raw, p46_nominal) : std::numeric_limits<double>::quiet_NaN();
    Eigen::Quaternionf qi(initial.block<3,3>(0,0)), qr;
    if (raw.allFinite()) { qr = Eigen::Quaternionf(raw.block<3,3>(0,0)); qr.normalize(); }
    qi.normalize();
    if (raw.allFinite()) qr.normalize();
    const std::string method = s.method;
    const std::size_t missing = active.size() - selected.size();
    std::vector<std::string> dropped;
    for (const auto &id : active) if (!std::binary_search(selected.begin(), selected.end(), id)) dropped.push_back(id);
    std::ostringstream dropped_text;
    for (std::size_t i=0;i<dropped.size();++i){if(i)dropped_text<<';';dropped_text<<dropped[i];}

    out << s.frame << ',' << s.stamp << ',' << s.budget << ',' << method << ',' << s.active_points << ','
        << target->size() << ',' << active.size() << ',' << selected.size() << ',' << missing << ',' << s.status << ','
        << std::setprecision(10) << s.proxy_coverage << ',' << cov.median << ',' << cov.p90 << ','
        << cov.overlap_0p5 << ',' << cov.overlap_2 << ',' << cov.overlap_8 << ',' << cov.support_points_2m << ','
        << (coverage_pass ? 1 : 0) << ',' << (align_called ? 1 : 0) << ',' << (candidate_valid ? 1 : 0) << ',' << (converged ? 1 : 0) << ','
        << fitness << ',' << iterations << ',' << align_ms << ',' << initial(0,3) << ',' << initial(1,3) << ',' << initial(2,3) << ','
        << qi.x() << ',' << qi.y() << ',' << qi.z() << ',' << qi.w() << ',';
    if (raw.allFinite())
      out << raw(0,3) << ',' << raw(1,3) << ',' << raw(2,3) << ',' << qr.x() << ',' << qr.y() << ',' << qr.z() << ',' << qr.w();
    else out << "nan,nan,nan,nan,nan,nan,nan";
    out << ',' << p46_dt << ',' << p46_dr << ',' << target_sha << ',' << s.source_hash << ',' << status << '\n';

    coverage_out << s.frame << ',' << s.stamp << ',' << s.budget << ',' << method << ',' << base_cov->second << ',' << s.proxy_coverage << ','
        << cov.overlap_0p5 << ',' << cov.overlap_2 << ',' << cov.overlap_8 << ',' << cov.median << ',' << cov.p90 << ','
        << cov.support_points_2m << ',' << missing << ',' << csvQuote(dropped_text.str()) << ',' << target->size() << ','
        << (coverage_pass ? "PASS" : "FAIL") << '\n';
    runtime_out << s.frame << ',' << s.stamp << ',' << s.budget << ',' << method << ',' << target->size() << ','
        << load_ms << ',' << coverage_ms << ',' << align_ms << ',' << (align_called ? 1 : 0) << ','
        << (converged ? 1 : 0) << ',' << iterations << ',' << status << '\n';
    ++completed;
    if (completed % 20 == 0) std::cout << "P54_ALIGN_PROGRESS cases=" << completed << "/" << selections.size() << '\n';
  }
  std::cout << "P54_ALIGN_COMPLETE cases=" << completed << " actual_aligns are recorded in align_results.csv\n";
  return 0;
}

int selfTest()
{
  // Verify that a PSD geometric Gram matrix stays PSD and that a hard point budget is respected.
  Matrix6d H = Matrix6d::Zero();
  const Eigen::Vector3d n1(1,0,0), n2(0,1,0), n3(0,0,1);
  for (const auto &n : {n1,n2,n3})
  {
    Eigen::Matrix<double,1,6> J;
    J << 0.8*n.x(), 0.8*n.y(), 0.8*n.z(), 0.0, 0.0, 0.0;
    H += J.transpose()*J;
  }
  Eigen::SelfAdjointEigenSolver<Matrix6d> eig(H);
  if (eig.info()!=Eigen::Success || eig.eigenvalues().minCoeff() < -1e-10) return 2;
  std::vector<TileFrameData> tiles(3);
  for (std::size_t i=0;i<3;++i) { tiles[i].id="t"+std::to_string(i); tiles[i].point_count=600; tiles[i].matches.resize(100); }
  for (std::size_t i=0;i<100;++i)
  {
    tiles[0].matches[i].occupancy_dist2 = i<50 ? 0.0f : 25.0f;
    tiles[1].matches[i].occupancy_dist2 = i>=50 ? 0.0f : 25.0f;
    tiles[2].matches[i].occupancy_dist2 = 25.0f;
  }
  const std::vector<Eigen::Vector3f> relative(100, Eigen::Vector3f::Zero());
  const Selection s = selectTiles(tiles, relative, 100, 1200, false, 1800);
  if (s.points>1200 || s.covered!=100 || s.indices.size()!=2) return 3;
  const Selection impossible = selectTiles(tiles, relative, 100, 600, false, 1800);
  if (impossible.status!="COVERAGE_PROXY_INFEASIBLE_BUDGET" || impossible.points>600)
  {
    std::cerr << "P54_SELF_TEST_FAIL status=" << impossible.status << " points=" << impossible.points
              << " coverage=" << impossible.covered << " selected=" << impossible.indices.size() << '\n';
    return 4;
  }
  std::cout << "P54_SELF_TEST_PASS PSD=1 budget=1 coverage_gate=1\n";
  return 0;
}

}  // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc == 2 && std::string(argv[1]) == "--self-test") return selfTest();
    if (argc < 2) throw std::runtime_error("mode required: select or align");
    const std::string mode(argv[1]);
    if (mode == "select") return selectMode(argc, argv);
    if (mode == "align") return alignMode(argc, argv);
    throw std::runtime_error("unknown mode: " + mode);
  }
  catch (const std::exception &e)
  {
    std::cerr << "P54_ERROR: " << e.what() << '\n';
    return 1;
  }
}
