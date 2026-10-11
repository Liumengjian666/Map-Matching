#pragma once
#include <Eigen/Core>
#include <cstdint>
#include <map>
#include <string>
namespace dog_prior_map_localization {
// Owns no filter, map or optimizer. Accepts only precomputed causal P4 rows.
class VisualAidedInitialGuess {
 public:
  void load(const std::string& path);
  bool apply(uint64_t previous_ns, uint64_t current_ns,
      const Eigen::Matrix4d& previous_used, Eigen::Matrix4d& guess, std::string& reason) const;
 private:
  struct Row {
    uint64_t previous_ns, ref_image_ns, cur_image_ns, point_ns, imu_ns;
    bool valid;
    Eigen::Vector3d displacement;
  };
  std::map<uint64_t, Row> rows_;
};
}
