#pragma once
#include "cloud_contract.hpp"

namespace screen {
inline void requireCorrectedGrid(const Ndt& ndt) {
  // This getter reads target_cells_.getLeafSize(), not resolution_.
  for (float leaf : ndt.targetGridLeafSizeMeters())
    if (leaf != .8f) throw std::runtime_error("actual Target Gaussian grid is not float32(0.8)m");
}
class CorrectedNdt : public Ndt {
 public:
  void configureTarget(const Cloud::Ptr& target) {
    // The ONLY optimizer configuration change from R4: set resolution first.
    setResolution(.8f);
    Ndt::configureTarget(target);
    requireCorrectedGrid(*this);
  }
};
} // namespace screen
