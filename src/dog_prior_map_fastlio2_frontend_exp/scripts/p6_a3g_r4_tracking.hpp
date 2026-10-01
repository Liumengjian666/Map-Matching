#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace p6_tracking {

enum class Health { GOOD, DEGRADED, LOST, RECOVERING };
inline const char* name(Health value) {
  switch (value) {
    case Health::GOOD: return "TRACKING_GOOD";
    case Health::DEGRADED: return "TRACKING_DEGRADED";
    case Health::LOST: return "TRACKING_LOST";
    case Health::RECOVERING: return "TRACKING_RECOVERING";
  }
  return "TRACKING_INVALID";
}

struct Config {
  bool enabled = false;
  unsigned recovery_after_failures = 2;
  unsigned maximum_alternative_seeds = 2;
  double maximum_extrapolation_s = 0.5;
  double maximum_anchor_age_s = 2.0;
};

inline Config readConfig(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("tracking_config_open_failed");
  std::map<std::string,std::string> values;
  std::string line;
  while (std::getline(input,line)) {
    if (line.empty() || line.front()=='#') continue;
    const auto split=line.find('=');
    if (split==std::string::npos || !values.emplace(line.substr(0,split),line.substr(split+1)).second)
      throw std::runtime_error("tracking_config_invalid");
  }
  if (values.size()!=5 || values.count("enabled")==0 ||
      values.count("recovery_after_failures")==0 || values.count("maximum_alternative_seeds")==0 ||
      values.count("maximum_extrapolation_s")==0 || values.count("maximum_anchor_age_s")==0)
    throw std::runtime_error("tracking_config_fields_invalid");
  Config result;
  if (values.at("enabled")!="true" && values.at("enabled")!="false")
    throw std::runtime_error("tracking_config_enabled_invalid");
  result.enabled=values.at("enabled")=="true";
  const auto unsigned_value=[&](const char* key) {
    const auto& value=values.at(key);
    if (value.empty() || value.find_first_not_of("0123456789")!=std::string::npos)
      throw std::runtime_error("tracking_config_unsigned_invalid");
    return std::stoul(value);
  };
  const auto failures=unsigned_value("recovery_after_failures");
  const auto seeds=unsigned_value("maximum_alternative_seeds");
  if (failures<1 || failures>100 || seeds<1 || seeds>2)
    throw std::runtime_error("tracking_config_budget_invalid");
  result.recovery_after_failures=failures;
  result.maximum_alternative_seeds=seeds;
  for (const char* key : {"maximum_extrapolation_s","maximum_anchor_age_s"}) {
    std::size_t consumed=0;
    const double value=std::stod(values.at(key),&consumed);
    if (consumed!=values.at(key).size() || !std::isfinite(value) || value<=0)
      throw std::runtime_error("tracking_config_time_invalid");
    if (std::string(key)=="maximum_extrapolation_s") result.maximum_extrapolation_s=value;
    else result.maximum_anchor_age_s=value;
  }
  if (result.maximum_anchor_age_s<result.maximum_extrapolation_s)
    throw std::runtime_error("tracking_config_horizon_invalid");
  return result;
}

// A zero-iteration terminal supplies no independent registration update.
// Its hasConverged flag alone must never reset tracking health.
inline bool effectiveRegistration(bool converged, int iterations, double objective,
                                  const Eigen::Matrix4d& pose) {
  return converged && iterations>0 && std::isfinite(objective) && pose.allFinite();
}

struct Seed {
  std::string label;
  Eigen::Matrix4d map_T_imu=Eigen::Matrix4d::Identity();
};

class Tracker {
 public:
  explicit Tracker(const Config& config) : config_(config) {}
  Health health() const { return health_; }
  unsigned consecutiveFailures() const { return failures_; }
  std::uint64_t lastReliableStamp() const { return last_.stamp; }

  // Called once per terminal, after evaluating its nominal candidate.
  bool needsRecovery(bool nominal_admissible) {
    if (!config_.enabled || nominal_admissible) return false;
    health_=failures_+1>=config_.recovery_after_failures ? Health::RECOVERING : Health::DEGRADED;
    return health_==Health::RECOVERING;
  }

  // Only a successfully optimized, committed measurement advances anchors.
  void finish(std::uint64_t stamp, bool committed, const Eigen::Matrix4d& measured_map_T_imu,
              const Eigen::Vector3d& optimized_velocity) {
    if (!config_.enabled) return;
    if (!committed) {
      ++failures_;
      health_=failures_>=config_.recovery_after_failures ? Health::LOST : Health::DEGRADED;
      return;
    }
    if (!stamp || !measured_map_T_imu.allFinite() || !optimized_velocity.allFinite() ||
        (last_.stamp && stamp<=last_.stamp))
      throw std::runtime_error("tracking_reliable_anchor_invalid");
    previous_=last_;
    last_={stamp,measured_map_T_imu,optimized_velocity};
    failures_=0; health_=Health::GOOD;
  }

  std::vector<Seed> seeds(std::uint64_t stamp, const Eigen::Matrix4d& nominal_map_T_imu) const {
    std::vector<Seed> output;
    if (!config_.enabled || !last_.stamp || stamp<=last_.stamp || !nominal_map_T_imu.allFinite())
      return output;
    const double age=static_cast<double>(stamp-last_.stamp)*1e-9;
    if (age>config_.maximum_anchor_age_s) return output;
    const double horizon=std::min(age,config_.maximum_extrapolation_s);
    if (previous_.stamp && last_.stamp>previous_.stamp) {
      const double dt=static_cast<double>(last_.stamp-previous_.stamp)*1e-9;
      Seed seed; seed.label="RELIABLE_POSE_CONSTANT_VELOCITY";
      seed.map_T_imu=last_.pose;
      seed.map_T_imu.block<3,1>(0,3)+=horizon/dt*
          (last_.pose.block<3,1>(0,3)-previous_.pose.block<3,1>(0,3));
      const Eigen::AngleAxisd delta(previous_.pose.block<3,3>(0,0).transpose()*
                                    last_.pose.block<3,3>(0,0));
      seed.map_T_imu.block<3,3>(0,0)=last_.pose.block<3,3>(0,0)*
          Eigen::AngleAxisd(delta.angle()*horizon/dt,delta.axis()).toRotationMatrix();
      output.push_back(seed);
    }
    Seed seed; seed.label="IMU_ORIENTATION_RELIABLE_TRANSLATION";
    seed.map_T_imu=nominal_map_T_imu;
    seed.map_T_imu.block<3,1>(0,3)=last_.pose.block<3,1>(0,3)+horizon*last_.velocity;
    output.push_back(seed);
    if (output.size()>config_.maximum_alternative_seeds) output.resize(config_.maximum_alternative_seeds);
    return output;
  }

 private:
  struct Anchor {
    std::uint64_t stamp=0;
    Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
    Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
  };
  Config config_;
  Health health_=Health::GOOD;
  unsigned failures_=0;
  Anchor previous_,last_;
};

}  // namespace p6_tracking
