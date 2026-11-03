#include "spatial_split.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/property_tree/json_parser.hpp>
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Tree = boost::property_tree::ptree;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::string xyz_sha(const Cloud& cloud) {
  std::vector<float> xyz;
  xyz.reserve(cloud.size() * 3);
  for (const auto& point : cloud.points) {
    xyz.push_back(point.x);
    xyz.push_back(point.y);
    xyz.push_back(point.z);
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(xyz.data()), xyz.size() * sizeof(float), digest);
  std::ostringstream output;
  for (unsigned char byte : digest) output << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return output.str();
}

Cloud::Ptr prepare(const Tree& config, const std::string& expected_points,
                   const std::string& expected_sha) {
  pcl::PCLPointCloud2 blob;
  require(pcl::io::loadPCDFile(config.get<std::string>("path"), blob) == 0, "PCD load failed");
  const size_t point_count = blob.width * blob.height;
  require(point_count == config.get<size_t>("points"), "raw point count differs from frozen input");

  std::vector<uint32_t> offsets;
  for (const auto* name : {"x", "y", "z"}) {
    const auto field = std::find_if(blob.fields.begin(), blob.fields.end(), [name](const auto& candidate) {
      return candidate.name == name;
    });
    require(field != blob.fields.end() && field->datatype == pcl::PCLPointField::FLOAT32 && field->count == 1,
            "raw XYZ field format differs from frozen input");
    offsets.push_back(field->offset);
  }

  sensor_msgs::PointCloud2 message;
  const uint64_t stamp = config.get<uint64_t>("stamp_ns");
  message.header.stamp.sec = stamp / 1000000000ULL;
  message.header.stamp.nsec = stamp % 1000000000ULL;
  message.header.frame_id = config.get<std::string>("frame_id");
  require(message.header.frame_id == "depth_camera_link", "source frame differs from R10 contract");
  message.width = static_cast<uint32_t>(point_count);
  message.height = 1;
  message.point_step = 16;
  message.row_step = static_cast<uint32_t>(point_count * 16);
  message.is_bigendian = false;
  message.is_dense = true;
  message.data.resize(point_count * 16, 0);
  for (size_t k = 0; k < 4; ++k) {
    sensor_msgs::PointField field;
    field.name = std::vector<std::string>{"x", "y", "z", "intensity"}[k];
    field.offset = static_cast<uint32_t>(k * 4);
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    message.fields.push_back(field);
  }

  Cloud::Ptr direct(new Cloud);
  direct->resize(point_count);
  direct->is_dense = true;
  for (size_t i = 0; i < point_count; ++i) {
    const auto* input = blob.data.data() + (i / blob.width) * blob.row_step + (i % blob.width) * blob.point_step;
    for (int axis = 0; axis < 3; ++axis) {
      std::memcpy(message.data.data() + i * 16 + axis * 4, input + offsets[axis], 4);
    }
    float xyz[3];
    std::memcpy(xyz, message.data.data() + i * 16, sizeof(xyz));
    require(std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]), "non-finite frozen XYZ");
    Point& point = direct->points[i];
    point.x = xyz[0]; point.y = xyz[1]; point.z = xyz[2]; point.intensity = 0.0f;
  }

  Cloud::Ptr converted(new Cloud);
  pcl::fromROSMsg(message, *converted);
  const std::string raw_sha = xyz_sha(*converted);
  require(raw_sha == config.get<std::string>("xyz_sha256"), "raw source XYZ SHA differs from R10");

  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setInputCloud(converted);
  Cloud::Ptr source(new Cloud);
  voxel.filter(*source);
  Cloud::Ptr direct_voxel(new Cloud);
  voxel.setInputCloud(direct);
  voxel.filter(*direct_voxel);
  require(xyz_sha(*source) == xyz_sha(*direct_voxel), "independent ROS/direct voxel paths disagree");
  require(source->size() == std::stoull(expected_points), "post-voxel source count differs from R10");
  require(xyz_sha(*source) == expected_sha, "post-voxel source XYZ SHA differs from R10");
  return source;
}

int main(int argc, char** argv) {
  try {
    require(argc == 4, "usage: partition_preflight INPUT_FREEZE_LOCAL.json R10_SOURCES.csv OUTPUT.csv");
    Tree freeze;
    boost::property_tree::read_json(argv[1], freeze);
    std::ifstream expected(argv[2]);
    require(expected.good(), "cannot open frozen R10 source identity CSV");
    std::string line;
    require(static_cast<bool>(std::getline(expected, line)), "missing R10 source identity header");

    std::ofstream output(argv[3]);
    require(output.good(), "cannot open output CSV");
    output << "raw_ordinal,stamp_ns,source_points,registration_points,validation_points,registration_blocks,validation_blocks,registration_ratio,validation_ratio,source_xyz_sha256,registration_xyz_sha256,validation_xyz_sha256\n";

    size_t count = 0;
    uint64_t total_registration = 0, total_validation = 0;
    for (const auto& entry : freeze.get_child("frames")) {
      require(static_cast<bool>(std::getline(expected, line)), "R10 source identity CSV ended early");
      std::stringstream row(line);
      std::string raw, stamp, source_points, source_hash;
      require(static_cast<bool>(std::getline(row, raw, ',')) && static_cast<bool>(std::getline(row, stamp, ',')) &&
                  static_cast<bool>(std::getline(row, source_points, ',')) && static_cast<bool>(std::getline(row, source_hash, ',')),
              "malformed R10 source identity row");
      const auto& frame = entry.second;
      const int ordinal = frame.get<int>("raw_ordinal");
      const uint64_t stamp_ns = frame.get<uint64_t>("stamp_ns");
      require(std::stoi(raw) == ordinal && std::stoull(stamp) == stamp_ns,
              "source timestamp/order differs from frozen R10 source list");
      const Cloud::Ptr source = prepare(frame, source_points, source_hash);
      const auto split = p63r14::split_cloud(*source);
      require(split.counts.registration_points > 0 && split.counts.validation_points > 0,
              "fixed spatial partition produced an empty subset");
      total_registration += split.counts.registration_points;
      total_validation += split.counts.validation_points;
      output << ordinal << ',' << stamp_ns << ',' << source->size() << ','
             << split.counts.registration_points << ',' << split.counts.validation_points << ','
             << split.counts.registration_blocks << ',' << split.counts.validation_blocks << ','
             << std::setprecision(12)
             << static_cast<double>(split.counts.registration_points) / source->size() << ','
             << static_cast<double>(split.counts.validation_points) / source->size() << ','
             << xyz_sha(*source) << ',' << xyz_sha(*split.registration) << ',' << xyz_sha(*split.validation) << '\n';
      ++count;
    }
    require(count == freeze.get<size_t>("evaluation_count"), "evaluation frame count differs from R10 contract");
    require(!std::getline(expected, line), "R10 source identity CSV contains extra frames");
    output.flush();
    std::cout << "PASS fixed spatial split for " << count << " R10 sources; registration="
              << total_registration << " validation=" << total_validation << " align=0\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PARTITION_PREFLIGHT_BLOCKED " << error.what() << '\n';
    return 2;
  }
}
