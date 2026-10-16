#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"

#include <cmath>
#include <iostream>
#include <limits>

int main() {
  using dog_prior_map_fastlio2_frontend_exp::evaluateWeakCoupledInnovation;
  using dog_prior_map_fastlio2_frontend_exp::weakCoupledInnovationTriggered;

  Eigen::Matrix4f prediction = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f nominal = prediction;
  nominal(0, 3) = 0.1201f;
  const auto translation = evaluateWeakCoupledInnovation(nominal, prediction);
  if (!translation.valid || !translation.triggered ||
      std::abs(translation.translation_m - 0.1201) > 1e-5) {
    std::cerr << "translation trigger contract failed\n";
    return 1;
  }

  nominal = prediction;
  nominal.block<3, 3>(0, 0) = Eigen::AngleAxisf(
      static_cast<float>(3.01 * std::acos(-1.0) / 180.0), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const auto rotation = evaluateWeakCoupledInnovation(nominal, prediction);
  if (!rotation.valid || !rotation.triggered || std::abs(rotation.rotation_deg - 3.01) > 1e-3) {
    std::cerr << "rotation trigger contract failed\n";
    return 1;
  }

  if (weakCoupledInnovationTriggered(0.12, 3.0) ||
      !weakCoupledInnovationTriggered(std::nextafter(0.12, 1.0), 3.0) ||
      !weakCoupledInnovationTriggered(0.12, std::nextafter(3.0, 4.0))) {
    std::cerr << "strict threshold comparison contract failed\n";
    return 1;
  }

  nominal = prediction;
  nominal(0, 3) = std::numeric_limits<float>::quiet_NaN();
  if (evaluateWeakCoupledInnovation(nominal, prediction).valid) {
    std::cerr << "non-finite pose accepted by trigger helper\n";
    return 1;
  }
  return 0;
}
