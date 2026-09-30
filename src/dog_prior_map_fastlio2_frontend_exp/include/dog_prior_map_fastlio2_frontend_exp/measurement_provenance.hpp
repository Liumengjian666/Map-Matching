#pragma once
#include <string>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
enum class LidarCloudProvenance {
  RAW_TIMED_SENSOR, SENSOR_LOCAL_ROTATION_ONLY, WINDOW_OWNED_SE3_DESKEW,
  LEGACY_STATE_DERIVED_SE3_DESKEW
};
enum class VisualMeasurementProvenance {
  RAW_SENSOR_LOCAL_DEPTH, WINDOW_OWNED_DEPTH, LEGACY_STATE_DERIVED_DEPTH, UNKNOWN
};
inline const char* toString(LidarCloudProvenance p) {
  switch (p) {
    case LidarCloudProvenance::RAW_TIMED_SENSOR: return "RAW_TIMED_SENSOR";
    case LidarCloudProvenance::SENSOR_LOCAL_ROTATION_ONLY: return "SENSOR_LOCAL_ROTATION_ONLY";
    case LidarCloudProvenance::WINDOW_OWNED_SE3_DESKEW: return "WINDOW_OWNED_SE3_DESKEW";
    default: return "LEGACY_STATE_DERIVED_SE3_DESKEW";
  }
}
inline const char* toString(VisualMeasurementProvenance p) {
  switch (p) {
    case VisualMeasurementProvenance::RAW_SENSOR_LOCAL_DEPTH: return "RAW_SENSOR_LOCAL_DEPTH";
    case VisualMeasurementProvenance::WINDOW_OWNED_DEPTH: return "WINDOW_OWNED_DEPTH";
    case VisualMeasurementProvenance::LEGACY_STATE_DERIVED_DEPTH: return "LEGACY_STATE_DERIVED_DEPTH";
    default: return "UNKNOWN";
  }
}
inline bool formalVisualInputAllowed(VisualMeasurementProvenance p) {
  return p == VisualMeasurementProvenance::RAW_SENSOR_LOCAL_DEPTH ||
      p == VisualMeasurementProvenance::WINDOW_OWNED_DEPTH;
}
inline bool rawDeskewInputAllowed(LidarCloudProvenance p, std::string* reason) {
  if (p == LidarCloudProvenance::RAW_TIMED_SENSOR) return true;
  if (reason) *reason = "NOT_ELIGIBLE_FOR_WINDOW_OWNED_DESKEW:" + std::string(toString(p));
  return false;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
