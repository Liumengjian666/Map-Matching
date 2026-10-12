#include "dog_prior_map_localization/core/ekf_translation_prior.hpp"
#include <cassert>
#include <iostream>

int main() {
  using namespace dog_prior_map_localization;
  Eigen::Matrix4d original = Eigen::Matrix4d::Identity();
  original.block<3,3>(0,0) = Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  original.block<3,1>(0,3) = Eigen::Vector3d(1,2,3);
  Eigen::Matrix4d predicted = Eigen::Matrix4d::Identity();
  predicted.block<3,1>(0,3) = Eigen::Vector3d(4,5,6);
  int checks = 0;
  auto test = [&](bool enable, bool visual, bool previous, bool imu, bool provenance,
      bool frame, uint64_t sample, const Eigen::Matrix4d& pose, const std::string& expected) {
    Eigen::Matrix4d guess = original;
    const auto result = applyEkfTranslationPrior(enable, true, visual, previous, imu,
        provenance, frame, sample, 100000000, pose, &guess);
    if (result != expected ||
        (guess.block<3,3>(0,0)-original.block<3,3>(0,0)).norm() != 0 ||
        (expected != "EKF_TRANSLATION_APPLIED" && (guess-original).norm() != 0) ||
        (expected == "EKF_TRANSLATION_APPLIED" &&
         (guess.block<3,1>(0,3)-pose.block<3,1>(0,3)).norm() != 0))
      throw std::runtime_error("translation-only selection contract failed: " + expected);
    ++checks;
  };
  test(false,false,true,true,true,true,99000000,predicted,"DISABLED");
  test(true,true,true,true,true,true,99000000,predicted,"VISUAL_APPLIED");
  test(true,false,true,true,true,true,99000000,predicted,"EKF_TRANSLATION_APPLIED");
  test(true,false,true,true,true,true,100000001,predicted,"FUTURE_SAMPLE_REJECTED");
  test(true,false,true,true,true,true,79999999,predicted,"STALE_EKF_SAMPLE");
  auto invalid = predicted; invalid(0,3) = std::numeric_limits<double>::quiet_NaN();
  test(true,false,true,true,true,true,99000000,invalid,"NONFINITE_PREDICTION");
  test(true,false,true,true,true,false,99000000,predicted,"INVALID_FRAME");
  test(true,false,false,true,true,true,99000000,predicted,"NDT_HISTORY_FALLBACK");
  test(true,false,true,true,true,true,0,predicted,"NO_CAUSAL_EKF_SAMPLE");
  test(true,false,true,false,true,true,99000000,predicted,"NDT_HISTORY_FALLBACK");
  test(true,false,true,true,false,true,99000000,predicted,"UNVERIFIED_EKF_PROVENANCE");
  test(true,false,true,true,true,true,80000000,predicted,"EKF_TRANSLATION_APPLIED");
  if (!orderedEkfSample(90,95) || orderedEkfSample(95,90) || orderedEkfSample(95,95))
    throw std::runtime_error("arrival-order ambiguity accepted");
  std::cout << "EKF_TRANSLATION_SELECTION_PASS checks=" << checks
            << " rotation unchanged; ambiguous stream rejected\n";
}
