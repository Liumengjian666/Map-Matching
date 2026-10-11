#include "dog_prior_map_localization/visual_aided_initial_guess.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>
namespace dog_prior_map_localization {
namespace {
std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields; std::stringstream stream(line); std::string value;
  while (std::getline(stream, value, ',')) fields.push_back(value);
  return fields;
}
uint64_t integer(const std::string& value) {
  size_t consumed = 0;
  if (value.empty() || value.front() == '-') throw std::runtime_error("invalid visual time");
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size()) throw std::runtime_error("noninteger visual time");
  return parsed;
}
}
void VisualAidedInitialGuess::load(const std::string& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("visual input unavailable: " + path);
  std::string line; std::getline(file, line); const auto names = split(line);
  std::map<std::string, size_t> columns;
  for (size_t i = 0; i < names.size(); ++i)
    if (!columns.emplace(names[i], i).second) throw std::runtime_error("duplicate visual column");
  std::map<uint64_t, Row> loaded;
  while (std::getline(file, line)) {
    const auto fields = split(line);
    auto at = [&](const std::string& name) -> const std::string& { return fields.at(columns.at(name)); };
    const uint64_t current = integer(at("scan_ns"));
    Row r{integer(at("prev_scan_ns")), integer(at("ref_image_ns")), integer(at("cur_image_ns")),
      integer(at("depth_max_point_ns")), integer(at("max_imu_ns")), integer(at("valid")) == 1,
      Eigen::Vector3d(std::stod(at("dx")), std::stod(at("dy")), std::stod(at("dz")))};
    if (r.valid && (!r.displacement.allFinite() || r.previous_ns >= current ||
        r.ref_image_ns > r.previous_ns || r.cur_image_ns > current ||
        r.ref_image_ns >= r.cur_image_ns || r.point_ns > r.ref_image_ns ||
        r.ref_image_ns-r.point_ns > 150000000 ||
        r.imu_ns > current || r.imu_ns == 0 || r.point_ns == 0 ||
        r.previous_ns-r.ref_image_ns > 50000000 || current-r.cur_image_ns > 50000000 ||
        r.cur_image_ns-r.ref_image_ns > 250000000 || current-r.previous_ns > 250000000))
      throw std::runtime_error("noncausal/invalid visual row");
    if (current == 0 || !loaded.emplace(current, r).second) throw std::runtime_error("duplicate/zero scan stamp");
  }
  rows_.swap(loaded);
}
bool VisualAidedInitialGuess::apply(uint64_t previous, uint64_t current,
    const Eigen::Matrix4d& previous_used, Eigen::Matrix4d& guess, std::string& reason) const {
  const auto found = rows_.find(current);
  if (found == rows_.end()) { reason = "NO_VISUAL_ROW"; return false; }
  const Row& r = found->second;
  if (!r.valid) { reason = "INVALID_VISUAL_MEASUREMENT"; return false; }
  if (previous != r.previous_ns) { reason = "PREVIOUS_SCAN_MISMATCH"; return false; }
  if (!previous_used.allFinite() || !guess.allFinite()) { reason = "NONFINITE_BASE_POSE"; return false; }
  const Eigen::Vector3d p = previous_used.block<3,1>(0,3) + previous_used.block<3,3>(0,0)*r.displacement;
  if (!p.allFinite()) { reason = "NONFINITE_VISUAL_GUESS"; return false; }
  // Rotation and downstream step limiting are deliberately unchanged.
  guess.block<3,1>(0,3) = p; reason = "P4_VISUAL_TRANSLATION_SHORT_TIME_BRIDGE"; return true;
}
}
