#include "dog_prior_map_localization/local_map_tiles.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dog_prior_map_localization
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

double elapsedMs(const SteadyClock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

bool sha256File(const std::string &path, std::string *digest, std::string *error)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input)
  {
    if (error) *error = "cannot open tile for checksum: " + path;
    return false;
  }
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx)
  {
    if (error) *error = "cannot allocate SHA256 context";
    return false;
  }
  bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1;
  std::array<char, 1024 * 1024> buffer{};
  while (ok && input.good())
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(ctx, buffer.data(), static_cast<std::size_t>(count)) != 1)
      ok = false;
  }
  unsigned char bytes[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  if (ok) ok = EVP_DigestFinal_ex(ctx, bytes, &length) == 1 && length == 32;
  EVP_MD_CTX_free(ctx);
  if (!ok)
  {
    if (error) *error = "failed to compute SHA256 for tile: " + path;
    return false;
  }
  static const char hex[] = "0123456789abcdef";
  digest->clear();
  digest->reserve(64);
  for (unsigned int i = 0; i < length; ++i)
  {
    digest->push_back(hex[bytes[i] >> 4]);
    digest->push_back(hex[bytes[i] & 0x0f]);
  }
  return true;
}

bool validIsometry(const Eigen::Matrix4d &T)
{
  if (!T.allFinite()) return false;
  if ((T.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1e-9) return false;
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  return (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() <= 1e-3 &&
      std::abs(R.determinant() - 1.0) <= 1e-3;
}
}  // namespace

bool LocalMapTiles::loadManifest(const std::string &path, std::string *error)
{
  try
  {
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    const int schema = root.get<int>("schema_version");
    const double tile_size = root.get<double>("tile_size_m");
    const double origin_x = root.get<double>("grid_origin_x");
    const double origin_y = root.get<double>("grid_origin_y");
    const auto preprocessing = root.get_child("preprocessing");
    const auto leaf = preprocessing.get_child("leaf_xyz_m");
    std::vector<double> leaf_sizes;
    for (const auto &item : leaf) leaf_sizes.push_back(std::stod(item.second.data()));
    if (schema != 1 || !std::isfinite(tile_size) || std::abs(tile_size - 20.0) > 1e-9 ||
        !std::isfinite(origin_x) || !std::isfinite(origin_y))
      throw std::runtime_error("P45 requires schema 1 and 20 m XY tiles with a finite grid origin");
    if (preprocessing.get<std::string>("filter") != "PCL VoxelGrid XYZ" ||
        preprocessing.get<int>("passes") != 2 || leaf_sizes.size() != 3 ||
        std::any_of(leaf_sizes.begin(), leaf_sizes.end(),
                    [](double value) { return !std::isfinite(value) || std::abs(value - 0.15) > 1e-9; }))
      throw std::runtime_error("tile manifest preprocessing differs from the frozen P44 global Target");

    std::map<std::pair<int, int>, Tile> by_grid;
    std::map<std::string, Tile> by_id;
    std::size_t declared_total_points = 0;
    const std::size_t declared_global_points = root.get<std::size_t>("global_target_point_count");
    const auto tiles_node = root.get_child("tiles");
    for (const auto &entry : tiles_node)
    {
      const boost::property_tree::ptree &node = entry.second;
      Tile tile;
      tile.id = node.get<std::string>("tile_id");
      tile.path = node.get<std::string>("tile_path");
      tile.sha256 = node.get<std::string>("sha256");
      tile.tile_x = node.get<int>("tile_x");
      tile.tile_y = node.get<int>("tile_y");
      tile.point_count = node.get<std::size_t>("point_count");
      const double x_min = node.get<double>("x_min");
      const double x_max = node.get<double>("x_max");
      const double y_min = node.get<double>("y_min");
      const double y_max = node.get<double>("y_max");
      const double z_min = node.get<double>("z_min");
      const double z_max = node.get<double>("z_max");
      if (tile.id.empty() || tile.path.empty() || tile.sha256.size() != 64 || tile.point_count == 0)
        throw std::runtime_error("manifest contains an incomplete tile entry");
      if (!std::isfinite(x_min) || !std::isfinite(x_max) || x_min > x_max ||
          !std::isfinite(y_min) || !std::isfinite(y_max) || y_min > y_max ||
          !std::isfinite(z_min) || !std::isfinite(z_max) || z_min > z_max ||
          x_min < origin_x + tile.tile_x * tile_size - 1e-3 ||
          x_max > origin_x + (tile.tile_x + 1) * tile_size + 1e-3 ||
          y_min < origin_y + tile.tile_y * tile_size - 1e-3 ||
          y_max > origin_y + (tile.tile_y + 1) * tile_size + 1e-3 ||
          tile.point_count > std::numeric_limits<std::size_t>::max() - declared_total_points)
        throw std::runtime_error("manifest contains invalid tile bounds or point-count overflow");
      if (!std::all_of(tile.sha256.begin(), tile.sha256.end(),
                       [](unsigned char value) { return std::isxdigit(value) != 0; }))
        throw std::runtime_error("manifest contains malformed tile SHA256");
      declared_total_points += tile.point_count;
      if (!by_grid.emplace(std::make_pair(tile.tile_x, tile.tile_y), tile).second ||
          !by_id.emplace(tile.id, tile).second)
        throw std::runtime_error("manifest contains duplicate tile ID or grid location");
    }
    if (by_id.empty() || declared_total_points != declared_global_points)
      throw std::runtime_error("tile manifest point total does not match the preprocessed global target");

    tile_size_m_ = tile_size;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    tiles_by_grid_.swap(by_grid);
    tiles_.swap(by_id);
    active_tile_ids_set_.clear();
    active_tile_ids_.clear();
    target_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    target_->height = 1;
    target_->width = 0;
    target_->is_dense = true;
    return true;
  }
  catch (const std::exception &exception)
  {
    if (error) *error = exception.what();
    return false;
  }
}

bool LocalMapTiles::selectTiles(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
    const Eigen::Matrix4d &map_T_lidar,
    double margin_m,
    std::set<std::string> *selected,
    Eigen::Vector3d *projected_min,
    Eigen::Vector3d *projected_max,
    std::string *error) const
{
  if (!source || source->empty() || !validIsometry(map_T_lidar) ||
      !std::isfinite(margin_m) || margin_m < 0.0)
  {
    if (error) *error = "invalid source, pose, or tile-selection margin";
    return false;
  }
  if (tiles_by_grid_.empty() || tile_size_m_ <= 0.0)
  {
    if (error) *error = "tile manifest is not loaded";
    return false;
  }

  *projected_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  *projected_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  selected->clear();
  const Eigen::Matrix3d R = map_T_lidar.block<3, 3>(0, 0);
  const Eigen::Vector3d t = map_T_lidar.block<3, 1>(0, 3);
  for (const pcl::PointXYZ &point : source->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const Eigen::Vector3d p_map = R * Eigen::Vector3d(point.x, point.y, point.z) + t;
    if (!p_map.allFinite()) continue;
    *projected_min = projected_min->cwiseMin(p_map);
    *projected_max = projected_max->cwiseMax(p_map);
    const int min_x = static_cast<int>(std::floor((p_map.x() - margin_m - origin_x_) / tile_size_m_));
    const int max_x = static_cast<int>(std::floor((p_map.x() + margin_m - origin_x_) / tile_size_m_));
    const int min_y = static_cast<int>(std::floor((p_map.y() - margin_m - origin_y_) / tile_size_m_));
    const int max_y = static_cast<int>(std::floor((p_map.y() + margin_m - origin_y_) / tile_size_m_));
    for (int tile_x = min_x; tile_x <= max_x; ++tile_x)
      for (int tile_y = min_y; tile_y <= max_y; ++tile_y)
      {
        const auto found = tiles_by_grid_.find(std::make_pair(tile_x, tile_y));
        if (found != tiles_by_grid_.end()) selected->insert(found->second.id);
      }
  }
  if (selected->empty())
  {
    if (error) *error = "predicted source footprint intersects no tile in the manifest";
    return false;
  }
  return true;
}

LocalMapUpdate LocalMapTiles::ensureCoverage(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
    const Eigen::Matrix4d &map_T_lidar,
    double safety_margin_m,
    double hysteresis_margin_m)
{
  LocalMapUpdate result;
  std::set<std::string> required;
  if (!selectTiles(source, map_T_lidar, safety_margin_m, &required,
                   &result.projected_min, &result.projected_max, &result.error))
    return result;

  if (!active_tile_ids_set_.empty() &&
      std::includes(active_tile_ids_set_.begin(), active_tile_ids_set_.end(),
                    required.begin(), required.end()))
  {
    result.ok = true;
    result.updated = false;
    result.reason = "active_cache_covers_required_tiles";
    result.loaded_tile_ids = active_tile_ids_;
    result.target = target_;
    result.target_points = target_ ? target_->size() : 0;
    return result;
  }

  std::set<std::string> expanded;
  const double expanded_margin = std::max(safety_margin_m, hysteresis_margin_m);
  if (!selectTiles(source, map_T_lidar, expanded_margin, &expanded,
                   &result.projected_min, &result.projected_max, &result.error))
    return result;

  std::size_t reserve_points = 0;
  for (const std::string &tile_id : expanded)
  {
    const auto found = tiles_.find(tile_id);
    if (found == tiles_.end() || found->second.point_count >
        std::numeric_limits<std::size_t>::max() - reserve_points)
    {
      result.error = "tile metadata lookup or point-count overflow";
      return result;
    }
    reserve_points += found->second.point_count;
  }

  const bool had_active_target = !active_tile_ids_set_.empty();
  const auto target_build_start = SteadyClock::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr assembled(new pcl::PointCloud<pcl::PointXYZ>());
  assembled->reserve(reserve_points);
  for (const std::string &tile_id : expanded)
  {
    const Tile &tile = tiles_.at(tile_id);
    const auto tile_io_start = SteadyClock::now();
    std::string actual_hash;
    if (!sha256File(tile.path, &actual_hash, &result.error)) return result;
    if (actual_hash != tile.sha256)
    {
      result.error = "tile SHA256 mismatch: " + tile_id;
      return result;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(tile.path, cloud) != 0)
    {
      result.error = "failed to load tile PCD: " + tile.path;
      return result;
    }
    if (cloud.size() != tile.point_count)
    {
      result.error = "tile point-count mismatch: " + tile_id;
      return result;
    }
    result.tile_io_ms += elapsedMs(tile_io_start);
    const auto append_start = SteadyClock::now();
    *assembled += cloud;
    result.target_merge_ms += elapsedMs(append_start);
    result.loaded_tile_ids.push_back(tile_id);
  }

  assembled->height = 1;
  assembled->width = static_cast<std::uint32_t>(assembled->size());
  assembled->is_dense = true;
  if (assembled->empty())
  {
    result.error = "selected tiles produced an empty local target";
    return result;
  }
  result.target_merge_ms += elapsedMs(target_build_start) - result.tile_io_ms - result.target_merge_ms;
  active_tile_ids_set_.swap(expanded);
  active_tile_ids_ = result.loaded_tile_ids;
  target_ = assembled;
  result.ok = true;
  result.updated = true;
  result.reason = had_active_target ? "required_tiles_outside_cache" : "initial_tile_load";
  result.target = target_;
  result.target_points = target_->size();
  return result;
}

}  // namespace dog_prior_map_localization
