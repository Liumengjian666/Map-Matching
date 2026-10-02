#include "dog_prior_map_fastlio2_frontend_exp/current_frame_ndt.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

using namespace dog_prior_map_fastlio2_frontend_exp;
namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}
bool identical(const RegistrationCloud& a, const RegistrationCloud& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (std::memcmp(&a[i].x, &b[i].x, sizeof(float)) ||
        std::memcmp(&a[i].y, &b[i].y, sizeof(float)) ||
        std::memcmp(&a[i].z, &b[i].z, sizeof(float))) return false;
  }
  return true;
}

void testPreprocess() {
  RegistrationCloud raw;
  for (int i = 0; i < 4000; ++i)
    raw.push_back({0.6f + 0.31f * (i % 50), 0.31f * ((i / 50) % 50), 0.31f * (i / 2500)});
  raw.push_back({std::numeric_limits<float>::quiet_NaN(), 0, 0});
  raw.push_back({0, std::numeric_limits<float>::infinity(), 0});
  raw.push_back({0.1f, 0, 0});
  raw.push_back({81, 0, 0});
  CurrentFrameNdtParameters parameters;
  const auto a = preprocessRegistrationCloud(raw, parameters);
  const auto b = preprocessRegistrationCloud(raw, parameters);
  require(a.size() == 1400 && identical(a, b), "deterministic source cap failed");
  require(registrationCloudHash(a) == registrationCloudHash(b), "source hash is nondeterministic");
  parameters.max_source_points = 10000;
  const auto uncapped = preprocessRegistrationCloud(raw, parameters);
  require(uncapped.size() == 4000, "finite/range filtering failed");
  const double increment = double(uncapped.size() - 1) / 1399.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const auto index = static_cast<std::size_t>(std::llround(i * increment));
    require(identical({a[i]}, {uncapped[index]}), "evenly spaced source sequence changed");
  }
  require(registrationCloudHash({{1.0f, -2.0f, 0.5f}}) == 18233715357354374355ULL,
          "formal FNV bits/metadata contract changed");
}

void testClassification() {
  using S = CurrentFrameNdtStatus;
  require(classifyNdtTerminal(false, 1, 80, true, true) == S::NOT_CONVERGED, "not converged");
  require(classifyNdtTerminal(true, 0, 80, true, true) == S::ZERO_ITERATION_PASSTHROUGH, "zero iteration");
  for (int iteration : {1, 79})
    require(classifyNdtTerminal(true, iteration, 80, true, true) == S::SUCCESS, "usable iterations");
  for (int iteration : {80, 81})
    require(classifyNdtTerminal(true, iteration, 80, true, true) == S::ITERATION_LIMIT_EXHAUSTED, "iteration limit");
  require(classifyNdtTerminal(true, 1, 80, false, true) == S::NONFINITE_TERMINAL, "nonfinite pose");
  require(classifyNdtTerminal(true, 1, 80, true, false) == S::NONFINITE_TERMINAL, "nonfinite fitness");
}

void testActualNdt() {
  char directory[] = "/tmp/p7_ndt_fixture_XXXXXX";
  require(mkdtemp(directory) != nullptr, "cannot create fixture directory");
  const std::string path = std::string(directory) + "/map.pcd";
  pcl::PointCloud<pcl::PointXYZ> map;
  for (int i = 0; i < 7; ++i) for (int j = 0; j < 5; ++j) for (int k = 0; k < 4; ++k)
    for (int a = -3; a <= 3; ++a) for (int b = -3; b <= 3; ++b) for (int c = -3; c <= 3; ++c) {
      map.push_back(pcl::PointXYZ(3.0f + 1.13f * i + 0.11f * a + 0.04f * std::sin(j + k),
          -2.0f + 1.01f * j + 0.10f * b + 0.07f * std::sin(i),
          -0.4f + 0.94f * k + 0.09f * c + 0.05f * std::cos(i + j)));
    }
  map.width = map.size(); map.height = 1; map.is_dense = true;
  require(pcl::io::savePCDFileBinary(path, map) == 0, "cannot save fixture map");
  Pose3d truth;
  truth.position = Eigen::Vector3d(0.65, -0.45, 0.28);
  truth.orientation = Eigen::AngleAxisd(0.09, Eigen::Vector3d(0.2, -0.3, 1.0).normalized());
  RegistrationCloud source;
  for (const auto& point : map) {
    const Eigen::Vector3d p = truth.orientation.conjugate() *
        (Eigen::Vector3d(point.x, point.y, point.z) - truth.position);
    source.push_back({float(p.x()), float(p.y()), float(p.z())});
  }
  CurrentFrameNdtRegistration ndt{CurrentFrameNdtParameters()};
  std::string reason;
  CurrentFrameNdtResult result;
  require(!ndt.align(1, source, truth, &result, &reason), "unloaded map accepted");
  require(ndt.loadMap(path, &reason) && ndt.ready() && ndt.targetPointCount() > 0, "map loading failed");
  require(ndt.align(1, {{1, 0, 0}}, truth, &result, &reason) &&
      result.status == CurrentFrameNdtStatus::INSUFFICIENT_POINTS && !result.effective &&
      result.iterations == 0 && result.alignment_ms == 0.0, "insufficient points not ordinary rejection");
  Pose3d seed = truth;
  seed.position += Eigen::Vector3d(0.10, -0.08, 0.06);
  seed.orientation = seed.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(0.015, Eigen::Vector3d::UnitY()));
  require(ndt.align(2, source, seed, &result, &reason), "actual NDT alignment failed");
  require(result.raw_map_T_lidar.position.allFinite() && result.raw_map_T_lidar.orientation.coeffs().allFinite(), "nonfinite actual NDT");
  require(result.effective && result.iterations > 0 && result.iterations < 80, "fixture did not effectively converge");
  const uint64_t hash = result.source_cloud_hash;
  const std::size_t count = result.source_point_count;
  require(ndt.align(3, source, seed, &result, &reason) && result.source_cloud_hash == hash &&
      result.source_point_count == count, "actual NDT source nondeterminism");
  std::cout << "actual PCL NDT PASS iterations=" << result.iterations << " count=" << count << '\n';
  std::remove(path.c_str());
  rmdir(directory);
}
}  // namespace

int main() {
  try {
    testPreprocess(); testClassification(); testActualNdt();
    std::cout << "current_frame_ndt_test PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "current_frame_ndt_test FAIL: " << error.what() << '\n';
    return 1;
  }
}
