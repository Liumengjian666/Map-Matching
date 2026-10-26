#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace dog_prior_map_localization {
enum class ObservationIdentityPolicy { Legacy, StrictFailClosed };
inline ObservationIdentityPolicy observationIdentityPolicy(const std::string &name) {
  if (name == "LEGACY") return ObservationIdentityPolicy::Legacy;
  if (name == "STRICT_FAIL_CLOSED") return ObservationIdentityPolicy::StrictFailClosed;
  throw std::runtime_error("unknown observation identity policy: " + name);
}

// Existing limiter arithmetic, extracted unchanged for an independent test.
inline bool limitObservationStep(const Eigen::Matrix4d &raw,
    const Eigen::Matrix4d &previous, bool enabled, bool have_previous,
    double max_translation, double max_rotation_deg, Eigen::Matrix4d &used,
    bool &translation_limited, bool &rotation_limited) {
  translation_limited = rotation_limited = false;
  if (!enabled || !have_previous) return false;
  const Eigen::Vector3d p = previous.block<3,1>(0,3);
  const Eigen::Vector3d dp = raw.block<3,1>(0,3) - p;
  const Eigen::Matrix3d a = previous.block<3,3>(0,0), b = raw.block<3,3>(0,0);
  const double angle = std::abs(Eigen::AngleAxisd(a.transpose()*b).angle())*180.0/M_PI;
  if (max_translation > 0.0 && dp.norm() > max_translation) {
    used.block<3,1>(0,3) = p + dp.normalized()*max_translation;
    translation_limited = true;
  }
  if (max_rotation_deg > 0.0 && angle > max_rotation_deg) {
    Eigen::Quaterniond qa(a), qb(b);
    used.block<3,3>(0,0) = qa.normalized().slerp(
        max_rotation_deg/std::max(angle,1e-6), qb.normalized()).toRotationMatrix();
    rotation_limited = true;
  }
  return translation_limited || rotation_limited;
}

struct ObservationIdentityDecision {
  bool publish = false;
  const char *reason = "ORIGINAL_ADMISSION_REJECTED";
  Eigen::Matrix4d pose = Eigen::Matrix4d::Constant(NAN);
};
inline ObservationIdentityDecision decideObservationIdentity(
    ObservationIdentityPolicy policy, bool admitted, bool converged,
    const Eigen::Matrix4d &certified, const Eigen::Matrix4d &proposed,
    bool limiter_changed) {
  ObservationIdentityDecision decision;
  if (!admitted || !converged || !certified.allFinite() || !proposed.allFinite())
    return decision;
  if (policy == ObservationIdentityPolicy::StrictFailClosed &&
      (limiter_changed || (certified.array()!=proposed.array()).any())) {
    decision.reason = "ALTERED_OBSERVATION_NOT_CERTIFIED";
    return decision;
  }
  decision.publish = true;
  decision.reason = policy == ObservationIdentityPolicy::Legacy ?
      "LEGACY" : "CERTIFIED_TERMINAL_UNCHANGED";
  decision.pose = policy == ObservationIdentityPolicy::Legacy ? proposed : certified;
  return decision;
}
} // namespace dog_prior_map_localization
