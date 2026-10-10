#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

#include <pcl/point_types.h>

int main() {
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;
  using Ndt = dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<Point>;

  Cloud::Ptr target(new Cloud);
  uint32_t state = 0x13579bdfU;
  auto next_unit = [&state]() {
    state = 1664525U * state + 1013904223U;
    return static_cast<float>(state) / static_cast<float>(UINT32_MAX);
  };
  for (int i = 0; i < 12000; ++i)
    target->push_back(Point(6.0f * next_unit() - 3.0f,
                            6.0f * next_unit() - 3.0f,
                            3.0f * next_unit() - 1.5f));
  target->width = static_cast<uint32_t>(target->size());
  target->height = 1;
  target->is_dense = true;

  Cloud::Ptr source(new Cloud(*target));
  Ndt ndt;
  ndt.setResolution(0.8f);
  ndt.setInputTarget(target);
  ndt.setInputSource(source);

  const Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
  const double value = ndt.dynamicScore(source, identity);
  const auto jet = ndt.nativeJet(source, identity,
      dog_prior_map_fastlio2_frontend_exp::CoupledVector6::Zero());
  if (!jet.valid || !std::isfinite(value) || value <= 0.0 ||
      std::abs(jet.score_sum - value) >
          1e-12 * std::max(1.0, std::abs(jet.score_sum))) {
    std::cerr << "PCL native derivative/value objective parity FAIL: jet_valid="
              << jet.valid << " value=" << value << " jet_score="
              << jet.score_sum << "\n";
    return 1;
  }
  std::cout << "PCL native derivative/value objective parity PASS: "
            << jet.score_sum << "\n";
  return 0;
}
