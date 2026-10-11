#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <inlier_core/config.hpp>
#include <inlier_core/encoder.hpp>
#include <inlier_core/token.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <sys/resource.h>

namespace p14 {
using Clock = std::chrono::steady_clock;
using Points = inlier::Encoder::PointMatrix;
inline double ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
inline long peakRssKiB() {
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) throw std::runtime_error("getrusage failed");
  return usage.ru_maxrss;
}
inline long rssKiB() {
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("VmRSS:", 0) == 0) return std::stol(line.substr(6));
  }
  throw std::runtime_error("VmRSS unavailable");
}
inline std::ofstream output(const std::string& path) {
  std::ofstream f(path, std::ios::binary | std::ios::out);
  if (!f) throw std::runtime_error("Cannot create " + path);
  f.exceptions(std::ios::badbit | std::ios::failbit);
  f << std::setprecision(17);
  return f;
}
inline Points load(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) < 0) throw std::runtime_error("PCD read failed: " + path);
  Points points(cloud.size(), 3);
  for (size_t i = 0; i < cloud.size(); ++i)
    points.row(i) << cloud[i].x, cloud[i].y, cloud[i].z;
  return points;
}
// Compatibility adapter only: same first occupied voxel / original-order
// semantics as author inlier/eval/encode.py::voxel_downsample, not a centroid.
inline Points preprocess(const Points& raw) {
  std::set<std::tuple<int64_t, int64_t, int64_t>> occupied;
  std::vector<Eigen::Index> keep;
  Points input = raw.cast<float>().cast<double>();
  for (Eigen::Index i = 0; i < raw.rows(); ++i) {
    if (!input.row(i).allFinite() || input.row(i).isZero(0)) continue;
    int64_t k[3];
    for (int j = 0; j < 3; ++j) {
      double q = std::floor(input(i, j) / 0.5);
      if (std::abs(q) >= 9e18) throw std::runtime_error("Voxel key out of range");
      k[j] = static_cast<int64_t>(q);
    }
    if (occupied.emplace(k[0], k[1], k[2]).second) keep.push_back(i);
  }
  Points result(keep.size(), 3);
  for (size_t i = 0; i < keep.size(); ++i) result.row(i) = input.row(keep[i]);
  return result;
}
inline void writePoints(std::ostream& out, const Points& points) {
  for (Eigen::Index i = 0; i < points.rows(); ++i)
    for (int j = 0; j < 3; ++j)
      out.write(reinterpret_cast<const char*>(&points(i, j)), sizeof(double));
}
inline Points readPoints(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f || f.tellg() <= 0 || static_cast<size_t>(f.tellg()) % 24 != 0)
    throw std::runtime_error("Invalid Nx3 float64 fixture");
  const size_t n = static_cast<size_t>(f.tellg()) / 24;
  f.seekg(0);
  Points p(n, 3);
  for (size_t i = 0; i < n; ++i)
    for (int j = 0; j < 3; ++j) f.read(reinterpret_cast<char*>(&p(i, j)), 8);
  if (!f || !p.allFinite()) throw std::runtime_error("Invalid fixture points");
  return p;
}
inline inlier::BEAMScoreConfig beamConfig() {
  inlier::BEAMScoreConfig c;
  c.min_shared_bins = 4;  // official YAML differs from library struct defaults
  return c;
}
inline inlier::ShortlistConfig mintConfig() {
  inlier::ShortlistConfig c;
  c.topk_pct = 0.10;
  return c;
}
inline inlier::VerifyConfig verifyConfig() {
  inlier::VerifyConfig c;
  c.inlier_dist_thresh = 1.0;
  c.min_correspondences = 32;
  c.min_ransac_inliers = 16;
  c.min_keypoint_inliers = 8;
  // Fixed seed is already the author C++ default and author unit-test policy.
  return c;
}
inline bool rigid(const Eigen::Matrix4d& t) {
  const Eigen::Matrix3d r = t.topLeftCorner<3, 3>();
  return t.allFinite() && (r.transpose()*r - Eigen::Matrix3d::Identity()).norm() < 1e-8
      && std::abs(r.determinant() - 1) < 1e-8
      && (t.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8;
}
inline int activeRows(const inlier::Tokens& tokens) {
  std::set<int> heights;
  for (auto t : tokens.token_id) heights.insert(inlier::UnpackToken(t, 20, 7, 60).hb);
  return static_cast<int>(heights.size());
}
inline void validate(const inlier::Keypoints& kp, const inlier::Tokens& tokens) {
  if (!kp.p.allFinite() || !rigid(kp.T_ground) || kp.p.rows() != static_cast<Eigen::Index>(tokens.token_id.size()))
    throw std::runtime_error("Invalid encoded keypoints / ground transform / lengths");
  for (auto t : tokens.token_id)
    if (t >= 10*20*7*60) throw std::runtime_error("Token outside author grid");
}
}  // namespace p14
