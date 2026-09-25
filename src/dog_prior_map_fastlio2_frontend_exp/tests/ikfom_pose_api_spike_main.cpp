#include "dog_prior_map_fastlio2_frontend_exp/ikfom_pose_api_spike.hpp"

#include <iostream>
#include <string>

int main() {
  std::string failure;
  if (!dog_prior_map_fastlio2_frontend_exp::runIkfomPoseApiSpike(&failure)) {
    std::cerr << "IKFOM_POSE_API_SPIKE_FAIL: " << failure << '\n';
    return 1;
  }
  std::cout << "IKFOM_POSE_API_SPIKE_PASS\n";
  return 0;
}
