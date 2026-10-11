#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace p12 {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Row = std::map<std::string, std::string>;
inline std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) fields.push_back(value);
  return fields;
}
inline std::vector<Row> readSamples(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("missing samples.csv");
  std::string line;
  std::getline(stream, line);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  const auto keys = split(line);
  std::vector<Row> rows;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const auto values = split(line);
    if (values.size() != keys.size()) throw std::runtime_error("bad sample row");
    Row row;
    for (std::size_t i = 0; i < keys.size(); ++i) row[keys[i]] = values[i];
    rows.push_back(row);
  }
  return rows;
}
inline Eigen::Matrix4d pose(const Row& row, const std::string& prefix) {
  Eigen::Quaterniond q(std::stod(row.at(prefix+"_qw")), std::stod(row.at(prefix+"_qx")),
                       std::stod(row.at(prefix+"_qy")), std::stod(row.at(prefix+"_qz")));
  if (!q.coeffs().allFinite() || q.norm() < 1e-8) throw std::runtime_error("bad pose quaternion");
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  for (int i = 0; i < 3; ++i)
    result(i, 3) = std::stod(row.at(prefix+"_t"+std::string(1, "xyz"[i])));
  return result;
}
inline Cloud::Ptr voxel(Cloud::Ptr input, float leaf, std::size_t cap) {
  Cloud::Ptr output(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*output);
  if (cap && output->size() > cap) {
    Cloud::Ptr limited(new Cloud);
    const double stride = static_cast<double>(output->size()-1)/std::max<std::size_t>(1, cap-1);
    for (std::size_t i = 0; i < cap; ++i) limited->push_back(output->points[std::llround(i*stride)]);
    output = limited;
  }
  output->width = output->size(); output->height = 1; output->is_dense = true;
  return output;
}
inline Cloud::Ptr loadFinite(const std::string& path, bool range_filter) {
  Cloud raw;
  if (pcl::io::loadPCDFile(path, raw) < 0) throw std::runtime_error("bad PCD: "+path);
  Cloud::Ptr output(new Cloud);
  for (const auto& p : raw) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const double range = std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);  // Same float arithmetic as NDT.
    if (range_filter && (range < .5 || range > 80)) continue;
    output->push_back(p);
  }
  return output;
}
inline std::uint64_t hash(const Cloud& cloud) {
  std::uint64_t value = 1469598103934665603ULL;
  auto mix = [&](std::uint32_t bits) {
    for (int i = 0; i < 4; ++i) { value ^= (bits >> (8*i)) & 255U; value *= 1099511628211ULL; }
  };
  mix(cloud.width); mix(cloud.height); mix(cloud.is_dense); mix(cloud.size());
  for (const auto& p : cloud) for (float f : {p.x, p.y, p.z}) {
    std::uint32_t bits; std::memcpy(&bits, &f, sizeof(bits)); mix(bits);
  }
  return value;
}
inline void poseHeader(std::ostream& out, const std::string& prefix) {
  for (const char* suffix : {"tx", "ty", "tz", "qx", "qy", "qz", "qw"}) out << ',' << prefix << '_' << suffix;
}
inline void writePose(std::ostream& out, const Eigen::Matrix4d& pose) {
  const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
  for (double value : {pose(0, 3), pose(1, 3), pose(2, 3), q.x(), q.y(), q.z(), q.w()}) out << ',' << value;
}
}  // namespace p12
