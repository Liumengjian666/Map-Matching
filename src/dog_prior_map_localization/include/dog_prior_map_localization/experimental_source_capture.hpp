#pragma once
// Opt-in, bounded research IO. Never changes a cloud or computes a pose.
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <openssl/sha.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dog_prior_map_localization {
class ExperimentalSourceCapture {
 public:
  void configure(const std::string& directory, const std::vector<std::string>& requests) {
    if (directory.empty() || requests.empty() || requests.size() > 8)
      throw std::runtime_error("SOURCE_CAPTURE_INVALID_REQUESTS");
    directory_ = directory;
    for (const auto& request : requests) {
      const auto split = request.find(':');
      const auto left = request.substr(0, split), right = request.substr(split + 1);
      if (split == std::string::npos || left.empty() || right.empty() ||
          left.find_first_not_of("0123456789") != std::string::npos ||
          right.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("SOURCE_CAPTURE_INVALID_ID");
      const uint64_t frame = std::stoull(left), ns = std::stoull(right);
      if (!frame || !ns || !requests_.emplace(ns, frame).second)
        throw std::runtime_error("SOURCE_CAPTURE_DUPLICATE_ID");
    }
  }
  bool requested(uint64_t ns) const { return requests_.count(ns) != 0; }
  void save(uint64_t frame, uint64_t ns, const pcl::PointCloud<pcl::PointXYZ>& source,
            uint64_t hash) {
    if (!requested(ns)) return;
    if (requests_.at(ns) != frame || !saved_.insert(frame).second)
      throw std::runtime_error("SOURCE_CAPTURE_FRAME_ID_MISMATCH_OR_DUPLICATE");
    const uint32_t endian = 1;
    if (*reinterpret_cast<const uint8_t*>(&endian) != 1 || sizeof(float) != 4)
      throw std::runtime_error("SOURCE_CAPTURE_UNSUPPORTED_ENCODING");
    std::vector<float> xyz;
    xyz.reserve(source.size() * 3);
    for (const auto& point : source) {
      xyz.push_back(point.x); xyz.push_back(point.y); xyz.push_back(point.z);
    }
    const auto bytes = reinterpret_cast<const unsigned char*>(xyz.data());
    const std::size_t size = xyz.size() * sizeof(float);
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(bytes, size, digest);
    std::ostringstream sha;
    for (auto byte : digest) sha << std::hex << std::setw(2) << std::setfill('0') << int(byte);
    std::ostringstream metadata;
    metadata << "{\"frame_index\":" << frame << ",\"scan_ns\":\"" << ns
        << "\",\"points\":" << source.size() << ",\"width\":" << source.width
        << ",\"height\":" << source.height << ",\"is_dense\":" << (source.is_dense ? "true" : "false")
        << ",\"stable_hash\":\"" << std::hex << std::setw(16) << std::setfill('0') << hash
        << "\",\"xyz_sha256\":\"" << sha.str() << "\",\"xyz_bytes\":" << std::dec << size
        << ",\"encoding\":\"ordered_float32_xyz_little_endian\"}\n";
    const auto prefix = directory_ + "/frame_" + std::to_string(frame);
    exclusiveWrite(prefix + ".xyz", bytes, size);
    const std::string text = metadata.str();
    exclusiveWrite(prefix + ".source.json", reinterpret_cast<const unsigned char*>(text.data()), text.size());
  }
  void savePoses(uint64_t frame, const std::vector<Eigen::Matrix4d>& poses) const {
    if (!saved_.count(frame)) return;
    // previous_used, initial_guess, imu_rotation_prior, raw_ndt, final_used.
    if (poses.size() != 5) throw std::runtime_error("SOURCE_CAPTURE_POSE_LAYOUT");
    std::vector<double> values;
    for (const auto& pose : poses) {
      if (!pose.allFinite()) throw std::runtime_error("SOURCE_CAPTURE_NONFINITE_POSE");
      for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) values.push_back(pose(i,j));
    }
    exclusiveWrite(directory_ + "/frame_" + std::to_string(frame) + ".poses.f64",
        reinterpret_cast<const unsigned char*>(values.data()), values.size() * sizeof(double));
  }
 private:
  static void exclusiveWrite(const std::string& path, const unsigned char* data, std::size_t size) {
    const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("SOURCE_CAPTURE_OPEN_FAILED: " + path + ": " + std::strerror(errno));
    std::size_t done = 0;
    while (done < size) {
      const auto count = ::write(fd, data + done, size - done);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) { ::close(fd); throw std::runtime_error("SOURCE_CAPTURE_WRITE_FAILED: " + path); }
      done += static_cast<std::size_t>(count);
    }
    if (::close(fd) != 0) throw std::runtime_error("SOURCE_CAPTURE_CLOSE_FAILED: " + path);
  }
  std::string directory_;
  std::map<uint64_t,uint64_t> requests_;
  std::set<uint64_t> saved_;
};
}  // namespace dog_prior_map_localization
