#include "dog_prior_map_fastlio2_frontend_exp/mature_measurement_admission.hpp"

#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {

bool MatureMeasurementAdmission::distanceGatePass(double distance_m) {
  return std::isfinite(distance_m) && distance_m >= 0.0 &&
      distance_m <= kInitialToResultDistanceToleranceM;
}

MatureMeasurementAdmissionResult MatureMeasurementAdmission::observe(
    bool ndt_success, double distance_m, const ProjectedPoseInnovation& innovation) {
  MatureMeasurementAdmissionResult result;
  result.initial_to_result_distance_m = distance_m;
  result.nis_threshold = chiSquare99Threshold(6);
  result.distance_gate_pass = ndt_success && distanceGatePass(distance_m);
  if (state_ == AdmissionTrackingState::LOST) {
    result.reason = "TRACKING_ALREADY_LOST";
  } else if (!ndt_success) {
    result.reason = "NDT_TERMINAL_REJECT";
  } else if (!result.distance_gate_pass) {
    result.reason = "AUTOWARE_INITIAL_TO_RESULT_DISTANCE_REJECT";
  } else {
    result.nis = innovation.nis;
    result.nis_valid = innovation.valid && innovation.rank == 6 &&
        std::isfinite(innovation.nis) && innovation.nis >= 0.0;
    result.nis_gate_pass = result.nis_valid && result.nis <= result.nis_threshold;
    if (!result.nis_valid) result.reason = "NIS_EVALUATION_INVALID";
    else if (!result.nis_gate_pass) result.reason = "MAHALANOBIS_NIS_REJECT";
    else {
      result.accepted = true;
      result.reason = "ACCEPTED";
    }
  }
  if (state_ != AdmissionTrackingState::LOST) {
    consecutive_rejections_ = result.accepted ? 0 : consecutive_rejections_ + 1;
    if (consecutive_rejections_ >= kMaxConsecutiveRejections)
      state_ = AdmissionTrackingState::LOST;
  }
  result.consecutive_rejections = consecutive_rejections_;
  result.tracking_state = state_;
  return result;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp
