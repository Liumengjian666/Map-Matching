#include <cassert>
#include <iostream>
#include "dog_prior_map_localization/core/ekf_revision_admission.hpp"
int main() {
  using dog_prior_map_localization::correctedHistoryAdmission;
  assert(correctedHistoryAdmission(true,true,100,100,false)=="REPLACED_EXACT_LATEST");
  assert(correctedHistoryAdmission(true,true,99,100,false)=="LATER_HISTORY_EXISTS");
  assert(correctedHistoryAdmission(true,true,101,100,false)=="EXACT_STAMP_MISMATCH_CORRECTED_LATER");
  assert(correctedHistoryAdmission(false,true,100,100,false)=="INVALID_SOURCE_OR_FRAME");
  assert(correctedHistoryAdmission(true,false,100,100,false)=="INVALID_POSE_OR_TWIST");
  assert(correctedHistoryAdmission(true,true,100,0,false)=="NO_LATEST_HIGH_RATE_SAMPLE");
  assert(correctedHistoryAdmission(true,true,100,100,true)=="DUPLICATE_CORRECTED_SAMPLE");
  std::cout << "EXACT_LATEST_CORRECTED_HISTORY_CONTRACT_PASS\n";
}
