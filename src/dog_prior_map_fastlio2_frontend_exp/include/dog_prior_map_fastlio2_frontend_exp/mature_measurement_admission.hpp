#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"

#include <limits>
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp {

enum class AdmissionTrackingState { TRACKING, LOST };

struct MatureMeasurementAdmissionResult {
  bool accepted = false;
  std::string reason = "NOT_EVALUATED";
  double initial_to_result_distance_m = std::numeric_limits<double>::quiet_NaN();
  bool distance_gate_pass = false;
  double nis = std::numeric_limits<double>::quiet_NaN();
  double nis_threshold = std::numeric_limits<double>::quiet_NaN();
  bool nis_valid = false;
  bool nis_gate_pass = false;
  int consecutive_rejections = 0;
  AdmissionTrackingState tracking_state = AdmissionTrackingState::TRACKING;
};

// Fixed P7-E adaptation: upstream Autoware uses these values for diagnostics;
// this baseline explicitly makes them hard admission / stop policies.
class MatureMeasurementAdmission {
 public:
  static constexpr double kInitialToResultDistanceToleranceM = 3.0;
  static constexpr int kMaxConsecutiveRejections = 5;
  static bool distanceGatePass(double distance_m);

  // Innovation MUST come from the existing full-rank EXACT_LOG_RESIDUAL frontend
  // evaluation, after the terminal and distance gates. No NIS math is duplicated.
  MatureMeasurementAdmissionResult observe(
      bool ndt_success, double distance_m, const ProjectedPoseInnovation& innovation);

 private:
  int consecutive_rejections_ = 0;
  AdmissionTrackingState state_ = AdmissionTrackingState::TRACKING;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp
