#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

void DogPriorMapEkfNode::initializeCorridorSequenceLocalizer()
{
  corridor_sequence_localizer_.configure(corridor_sequence_enable_, corridor_sequence_bin_size_,
                                         corridor_sequence_length_, corridor_sequence_max_hypotheses_,
                                         corridor_sequence_search_radius_, corridor_axis_min_, corridor_axis_max_);
  if (corridor_sequence_enable_ && map_cloud_ && !map_cloud_->empty())
  {
    corridor_sequence_localizer_.buildMap(map_cloud_);
    ROS_INFO("[DogPriorMap C++] corridor sequence localizer: enabled=%d ready=%d binsize=%.2f hypotheses=%d",
             corridor_sequence_enable_, corridor_sequence_localizer_.ready(), corridor_sequence_bin_size_,
             corridor_sequence_max_hypotheses_);
  }
}

// 加载先验地图、执行体素降采样，并建立后续最近邻匹配使用的 KD 树。
void DogPriorMapEkfNode::loadPriorMap()
{
  const std::string pcd_path = getParam<std::string>(
      "map/pcd_fallback_path",
      "/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd");
  const double map_voxel = getParam<double>("map/voxel_size", 0.30);

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw = loadPcdXyzOnly(pcd_path);
  if (!raw || raw->empty())
  {
    ROS_FATAL("[DogPriorMap C++] failed to read prior map PCD: %s", pcd_path.c_str());
    throw std::runtime_error("failed to load prior map pcd");
  }

  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
  if (map_voxel > 0.01)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(map_voxel, map_voxel, map_voxel);
    voxel.setInputCloud(raw);
    voxel.filter(*map_cloud_);
  }
  else
  {
    *map_cloud_ = *raw;
  }
  map_cloud_->width = static_cast<uint32_t>(map_cloud_->points.size());
  map_cloud_->height = 1;
  map_cloud_->is_dense = true;

  map_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  map_kdtree_->setInputCloud(map_cloud_);
  ROS_INFO("[DogPriorMap C++] prior map loaded: raw=%zu, voxel=%zu, path=%s",
           raw->size(), map_cloud_->size(), pcd_path.c_str());
}

// 兼容读取仅含 XYZ 或同时含强度等字段的 PCD，并只保留有限 XYZ 坐标。
pcl::PointCloud<pcl::PointXYZ>::Ptr DogPriorMapEkfNode::loadPcdXyzOnly(const std::string &pcd_path)
{
  // ------------------------- 轻量PCD读取器 -------------------------
  // 这里故意不用 pcl::io::loadPCDFile，因为当前电脑上PCL_IO会链接到有问题的libusb。
  // 我们只需要先验地图的 x/y/z 三个float字段，所以自己解析PCD头和二进制点数据即可。
  std::ifstream in(pcd_path, std::ios::binary);
  if (!in.is_open())
  {
    ROS_ERROR("[DogPriorMap C++] cannot open PCD file: %s", pcd_path.c_str());
    return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  }

  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<int> counts;
  std::string data_type;
  size_t point_num = 0;
  std::string line;

  while (std::getline(in, line))
  {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    if (key == "FIELDS")
    {
      std::string field;
      while (iss >> field) fields.push_back(field);
    }
    else if (key == "SIZE")
    {
      int v = 0;
      while (iss >> v) sizes.push_back(v);
    }
    else if (key == "COUNT")
    {
      int v = 0;
      while (iss >> v) counts.push_back(v);
    }
    else if (key == "POINTS")
    {
      iss >> point_num;
    }
    else if (key == "DATA")
    {
      iss >> data_type;
      break;
    }
  }

  if (counts.empty()) counts.assign(fields.size(), 1);
  if (fields.empty() || sizes.empty() || point_num == 0)
  {
    ROS_ERROR("[DogPriorMap C++] PCD header parse failed: %s", pcd_path.c_str());
    return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  }

  int point_step = 0;
  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  for (size_t i = 0; i < fields.size(); ++i)
  {
    if (fields[i] == "x") x_offset = point_step;
    if (fields[i] == "y") y_offset = point_step;
    if (fields[i] == "z") z_offset = point_step;
    const int count = i < counts.size() ? counts[i] : 1;
    const int size = i < sizes.size() ? sizes[i] : 4;
    point_step += size * count;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(point_num);

  if (x_offset < 0 || y_offset < 0 || z_offset < 0)
  {
    ROS_ERROR("[DogPriorMap C++] PCD missing x/y/z fields: %s", pcd_path.c_str());
    return cloud;
  }

  if (data_type == "binary")
  {
    std::vector<char> buffer(static_cast<size_t>(point_step));
    for (size_t i = 0; i < point_num && in.good(); ++i)
    {
      in.read(buffer.data(), point_step);
      if (in.gcount() != point_step) break;
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;
      std::memcpy(&x, buffer.data() + x_offset, sizeof(float));
      std::memcpy(&y, buffer.data() + y_offset, sizeof(float));
      std::memcpy(&z, buffer.data() + z_offset, sizeof(float));
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      {
        cloud->push_back(pcl::PointXYZ(x, y, z));
      }
    }
  }
  else if (data_type == "ascii")
  {
    for (size_t i = 0; i < point_num && std::getline(in, line); ++i)
    {
      std::istringstream iss(line);
      std::vector<float> values;
      float v = 0.0f;
      while (iss >> v) values.push_back(v);
      int x_idx = -1;
      int y_idx = -1;
      int z_idx = -1;
      int value_idx = 0;
      for (size_t f = 0; f < fields.size(); ++f)
      {
        if (fields[f] == "x") x_idx = value_idx;
        if (fields[f] == "y") y_idx = value_idx;
        if (fields[f] == "z") z_idx = value_idx;
        value_idx += f < counts.size() ? counts[f] : 1;
      }
      if (x_idx >= 0 && y_idx >= 0 && z_idx >= 0 &&
          static_cast<size_t>(std::max({x_idx, y_idx, z_idx})) < values.size())
      {
        cloud->push_back(pcl::PointXYZ(values[x_idx], values[y_idx], values[z_idx]));
      }
    }
  }
  else
  {
    ROS_ERROR("[DogPriorMap C++] unsupported PCD DATA type: %s", data_type.c_str());
  }

  return cloud;
}

}  // namespace dog_prior_map_localization
