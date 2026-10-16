#include <openssl/evp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <sys/stat.h>
#include <utility>
#include <vector>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using TileKey = std::pair<int, int>;

struct Bounds
{
  Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  void add(const pcl::PointXYZ &point)
  {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    min = min.cwiseMin(p);
    max = max.cwiseMax(p);
  }
};

struct Fingerprint
{
  std::uint64_t xor_hash = 0;
  std::uint64_t sum_hash = 0;
  std::uint64_t sum_hash_squared = 0;
  std::uint64_t count = 0;
  void add(const pcl::PointXYZ &point)
  {
    std::uint32_t words[3];
    std::memcpy(&words[0], &point.x, sizeof(float));
    std::memcpy(&words[1], &point.y, sizeof(float));
    std::memcpy(&words[2], &point.z, sizeof(float));
    std::uint64_t hash = 1469598103934665603ULL;
    for (std::uint32_t word : words)
      for (int byte = 0; byte < 4; ++byte)
      {
        hash ^= static_cast<std::uint8_t>((word >> (byte * 8)) & 0xffU);
        hash *= 1099511628211ULL;
      }
    xor_hash ^= hash;
    sum_hash += hash;
    sum_hash_squared += hash * hash;
    ++count;
  }
  bool operator==(const Fingerprint &other) const
  {
    return xor_hash == other.xor_hash && sum_hash == other.sum_hash &&
        sum_hash_squared == other.sum_hash_squared && count == other.count;
  }
};

std::string sha256File(const std::string &path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input) throw std::runtime_error("cannot read for SHA256: " + path);
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx) throw std::runtime_error("cannot allocate SHA256 context");
  bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1;
  std::array<char, 1024 * 1024> buffer{};
  while (ok && input.good())
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (input.gcount() > 0 && EVP_DigestUpdate(ctx, buffer.data(),
        static_cast<std::size_t>(input.gcount())) != 1) ok = false;
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  if (ok) ok = EVP_DigestFinal_ex(ctx, digest, &length) == 1 && length == 32;
  EVP_MD_CTX_free(ctx);
  if (!ok) throw std::runtime_error("SHA256 failed: " + path);
  static const char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(64);
  for (unsigned int i = 0; i < length; ++i)
  {
    result.push_back(hex[digest[i] >> 4]);
    result.push_back(hex[digest[i] & 0x0f]);
  }
  return result;
}

Cloud::Ptr voxelDown(const Cloud::ConstPtr &input, float leaf_size)
{
  Cloud::Ptr output(new Cloud());
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.setInputCloud(input);
  voxel.filter(*output);
  output->height = 1;
  output->width = static_cast<std::uint32_t>(output->size());
  output->is_dense = true;
  return output;
}

std::string tileId(int x, int y)
{
  std::ostringstream out;
  out << "tile_x" << (x < 0 ? "m" : "p") << std::setw(5) << std::setfill('0') << std::abs(x)
      << "_y" << (y < 0 ? "m" : "p") << std::setw(5) << std::setfill('0') << std::abs(y);
  return out.str();
}

std::uint64_t peakRssKiB()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return 0;
  return static_cast<std::uint64_t>(usage.ru_maxrss);
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: build_ndt_map_tiles INPUT_XYZ_PCD OUTPUT_DIR TILE_SIZE_M EXPECTED_TARGET_POINTS\n";
    return 2;
  }
  try
  {
    const std::string input_path(argv[1]);
    const std::string output_dir(argv[2]);
    const double tile_size = std::stod(argv[3]);
    const std::size_t expected_target_points = static_cast<std::size_t>(std::stoull(argv[4]));
    if (!std::isfinite(tile_size) || tile_size <= 0.0)
      throw std::runtime_error("tile size must be positive and finite");
    struct stat output_stat{};
    if (stat(output_dir.c_str(), &output_stat) != 0 || !S_ISDIR(output_stat.st_mode))
      throw std::runtime_error("output directory must already exist: " + output_dir);

    struct stat input_stat{};
    if (stat(input_path.c_str(), &input_stat) != 0 || !S_ISREG(input_stat.st_mode))
      throw std::runtime_error("input map does not exist: " + input_path);
    const std::string input_sha = sha256File(input_path);

    Cloud::Ptr raw(new Cloud());
    if (pcl::io::loadPCDFile(input_path, *raw) != 0)
      throw std::runtime_error("failed to read input PCD");
    const std::size_t raw_count = raw->size();
    Cloud::Ptr finite(new Cloud());
    finite->reserve(raw->size());
    for (const pcl::PointXYZ &point : raw->points)
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        finite->push_back(point);
    finite->height = 1;
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->is_dense = true;
    const std::size_t finite_count = finite->size();
    raw.reset();

    // This exactly mirrors the two sequential 0.15 m XYZ-only PCL VoxelGrid
    // passes used by P44: map/voxel_size, then ndt_target_voxel_size.
    Cloud::Ptr map_filtered = voxelDown(finite, 0.15f);
    finite.reset();
    Cloud::Ptr target = voxelDown(map_filtered, 0.15f);
    map_filtered.reset();
    if (target->size() != expected_target_points)
    {
      std::ostringstream message;
      message << "global preprocessed target count mismatch: expected " << expected_target_points
              << ", got " << target->size() << "; refusing to tile";
      throw std::runtime_error(message.str());
    }

    Bounds target_bounds;
    Fingerprint target_fp;
    for (const pcl::PointXYZ &point : target->points)
    {
      target_bounds.add(point);
      target_fp.add(point);
    }
    const double origin_x = std::floor(target_bounds.min.x() / tile_size) * tile_size;
    const double origin_y = std::floor(target_bounds.min.y() / tile_size) * tile_size;
    std::map<TileKey, Cloud::Ptr> tile_clouds;
    std::map<TileKey, Bounds> tile_bounds;
    for (const pcl::PointXYZ &point : target->points)
    {
      const int tile_x = static_cast<int>(std::floor((point.x - origin_x) / tile_size));
      const int tile_y = static_cast<int>(std::floor((point.y - origin_y) / tile_size));
      const TileKey key(tile_x, tile_y);
      auto &cloud = tile_clouds[key];
      if (!cloud) cloud.reset(new Cloud());
      cloud->push_back(point);
      tile_bounds[key].add(point);
    }

    const std::string manifest_path = output_dir + "/tile_manifest.json";
    struct stat manifest_stat{};
    if (stat(manifest_path.c_str(), &manifest_stat) == 0)
      throw std::runtime_error("refusing to overwrite existing tile manifest: " + manifest_path);
    std::vector<std::string> tile_paths;
    tile_paths.reserve(tile_clouds.size());
    std::ostringstream manifest;
    manifest << std::setprecision(17)
        << "{\n  \"schema_version\": 1,\n"
        << "  \"input_path\": \"" << input_path << "\",\n"
        << "  \"input_sha256\": \"" << input_sha << "\",\n"
        << "  \"input_point_count\": " << raw_count << ",\n"
        << "  \"finite_point_count\": " << finite_count << ",\n"
        << "  \"preprocessing\": {\"filter\": \"PCL VoxelGrid XYZ\", \"passes\": 2, \"leaf_xyz_m\": [0.15, 0.15, 0.15]},\n"
        << "  \"global_target_point_count\": " << target->size() << ",\n"
        << "  \"global_target_point_multiset_fingerprint\": {\"count\": " << target_fp.count
        << ", \"xor_fnv64\": " << target_fp.xor_hash << ", \"sum_fnv64\": " << target_fp.sum_hash
        << ", \"sum_squared_fnv64\": " << target_fp.sum_hash_squared << "},\n"
        << "  \"tile_size_m\": " << tile_size << ",\n"
        << "  \"grid_origin_x\": " << origin_x << ",\n"
        << "  \"grid_origin_y\": " << origin_y << ",\n"
        << "  \"target_bounds\": {\"x_min\": " << target_bounds.min.x()
        << ", \"x_max\": " << target_bounds.max.x() << ", \"y_min\": " << target_bounds.min.y()
        << ", \"y_max\": " << target_bounds.max.y() << ", \"z_min\": " << target_bounds.min.z()
        << ", \"z_max\": " << target_bounds.max.z() << "},\n  \"tiles\": [\n";

    std::size_t tile_index = 0;
    for (auto &entry : tile_clouds)
    {
      Cloud::Ptr &tile = entry.second;
      tile->height = 1;
      tile->width = static_cast<std::uint32_t>(tile->size());
      tile->is_dense = true;
      const std::string id = tileId(entry.first.first, entry.first.second);
      const std::string file_path = output_dir + "/" + id + ".pcd";
      struct stat tile_stat{};
      if (stat(file_path.c_str(), &tile_stat) == 0)
        throw std::runtime_error("refusing to overwrite existing tile: " + file_path);
      if (pcl::io::savePCDFileBinary(file_path, *tile) != 0)
        throw std::runtime_error("failed to save tile: " + file_path);
      tile_paths.push_back(file_path);
      const std::string tile_sha = sha256File(file_path);
      const Bounds &bounds = tile_bounds.at(entry.first);
      if (tile_index++) manifest << ",\n";
      manifest << "    {\"tile_id\": \"" << id << "\", \"tile_path\": \"" << file_path
          << "\", \"tile_x\": " << entry.first.first << ", \"tile_y\": " << entry.first.second
          << ", \"x_min\": " << bounds.min.x() << ", \"x_max\": " << bounds.max.x()
          << ", \"y_min\": " << bounds.min.y() << ", \"y_max\": " << bounds.max.y()
          << ", \"z_min\": " << bounds.min.z() << ", \"z_max\": " << bounds.max.z()
          << ", \"point_count\": " << tile->size() << ", \"sha256\": \"" << tile_sha << "\"}";
    }
    manifest << "\n  ]\n}\n";
    std::ofstream manifest_file(manifest_path.c_str(), std::ios::out | std::ios::binary);
    if (!manifest_file) throw std::runtime_error("cannot create manifest: " + manifest_path);
    manifest_file << manifest.str();
    manifest_file.close();
    if (!manifest_file) throw std::runtime_error("failed writing manifest: " + manifest_path);

    // Re-open each on-disk tile and compare an order-independent fingerprint,
    // catching omissions or point-coordinate changes across the actual files.
    Fingerprint disk_fp;
    std::size_t disk_points = 0;
    for (const std::string &path : tile_paths)
    {
      Cloud tile;
      if (pcl::io::loadPCDFile(path, tile) != 0)
        throw std::runtime_error("failed to re-read saved tile: " + path);
      disk_points += tile.size();
      for (const pcl::PointXYZ &point : tile.points) disk_fp.add(point);
    }
    if (disk_points != target->size() || !(disk_fp == target_fp))
      throw std::runtime_error("saved tile XYZ multiset differs from preprocessed global target");

    std::cout << "input_sha256=" << input_sha << "\n"
              << "raw_input_points=" << raw_count << " finite_input_points=" << finite_count
              << " global_target_points=" << target->size() << "\n"
              << "tile_count=" << tile_clouds.size() << " tile_size_m=" << tile_size << "\n"
              << "grid_origin=" << origin_x << "," << origin_y << "\n"
              << "target_bbox=" << target_bounds.min.transpose() << " .. "
              << target_bounds.max.transpose() << "\n"
              << "xyz_multiset_verified=" << (disk_fp == target_fp ? "true" : "false")
              << " peak_rss_kib=" << peakRssKiB() << "\n"
              << "manifest=" << manifest_path << "\n";
  }
  catch (const std::exception &error)
  {
    std::cerr << "build_ndt_map_tiles failed: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
