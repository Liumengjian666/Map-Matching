#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace dog_prior_map_localization
{

struct LocalMapUpdate
{
  bool ok = false;
  bool updated = false;
  std::string reason;
  std::string error;
  std::vector<std::string> loaded_tile_ids;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target;
  std::size_t target_points = 0;
  double tile_io_ms = 0.0;
  double target_merge_ms = 0.0;
  Eigen::Vector3d projected_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d projected_max = Eigen::Vector3d::Zero();
};

// Metadata-only tile index and one active merged tile target. It never loads
// the global map or retains per-tile point clouds after assembling a target.
class LocalMapTiles
{
public:
  bool loadManifest(const std::string &path, std::string *error);

  LocalMapUpdate ensureCoverage(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
                                const Eigen::Matrix4d &map_T_lidar,
                                double safety_margin_m,
                                double hysteresis_margin_m);

  const pcl::PointCloud<pcl::PointXYZ>::Ptr &target() const { return target_; }
  const std::vector<std::string> &activeTileIds() const { return active_tile_ids_; }
  double tileSize() const { return tile_size_m_; }
  Eigen::Vector2d gridOrigin() const { return Eigen::Vector2d(origin_x_, origin_y_); }
  std::size_t tileCount() const { return tiles_.size(); }

private:
  struct Tile
  {
    std::string id;
    std::string path;
    std::string sha256;
    int tile_x = 0;
    int tile_y = 0;
    std::size_t point_count = 0;
  };

  bool selectTiles(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
                   const Eigen::Matrix4d &map_T_lidar,
                   double margin_m,
                   std::set<std::string> *selected,
                   Eigen::Vector3d *projected_min,
                   Eigen::Vector3d *projected_max,
                   std::string *error) const;

  double tile_size_m_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::map<std::pair<int, int>, Tile> tiles_by_grid_;
  std::map<std::string, Tile> tiles_;
  std::set<std::string> active_tile_ids_set_;
  std::vector<std::string> active_tile_ids_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_;
};

}  // namespace dog_prior_map_localization
