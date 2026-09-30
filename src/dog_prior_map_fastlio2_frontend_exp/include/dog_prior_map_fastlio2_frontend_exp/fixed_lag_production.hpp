#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <limits>
#include <tuple>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

enum class ProducerEventType { LIDAR_SCAN, VISUAL_CURRENT, VISUAL_REFERENCE };

struct ProducerEvent {
  std::uint64_t stamp_ns = 0;
  ProducerEventType type = ProducerEventType::LIDAR_SCAN;
  std::size_t source_index = 0;
};

inline const char* toString(ProducerEventType type) {
  switch (type) {
    case ProducerEventType::LIDAR_SCAN: return "LIDAR_SCAN";
    case ProducerEventType::VISUAL_CURRENT: return "VISUAL_CURRENT";
    default: return "VISUAL_REFERENCE";
  }
}

inline void sortProducerEvents(std::vector<ProducerEvent>* events) {
  std::stable_sort(events->begin(), events->end(),
      [](const ProducerEvent& a, const ProducerEvent& b) {
        return std::tie(a.stamp_ns, a.type, a.source_index) <
               std::tie(b.stamp_ns, b.type, b.source_index);
      });
}

inline ImuNoiseParameters makeWindowImuNoise(
    const RuntimeParameters& parameters, const Eigen::Vector3d& gravity) {
  ImuNoiseParameters noise;
  noise.gyro_noise_density = parameters.gyro_noise_std_rad_s;
  noise.accel_noise_density = parameters.accel_noise_std_m_s2;
  noise.gyro_bias_random_walk = parameters.gyro_bias_rw_std_rad_s2;
  noise.accel_bias_random_walk = parameters.accel_bias_rw_std_m_s3;
  noise.gravity = gravity;
  return noise;
}

struct SelectedLidarNis {
  bool valid = false;
  bool accepted = false;
  int rank = 0;
  double nis = std::numeric_limits<double>::quiet_NaN();
  double threshold = std::numeric_limits<double>::quiet_NaN();
  std::string status = "SELECTED_NIS_UNAVAILABLE";
};

// The caller obtains P15 before admitting the LiDAR measurement. Covariance,
// residual and Jacobian all use the actual window factor's residual chart.
inline SelectedLidarNis evaluateSelectedLidarNis(
    const WindowState& state, const LidarWindowMeasurement& measurement,
    const WindowMarginalCovariance& prior, double threshold) {
  SelectedLidarNis result;
  result.threshold = threshold;
  if (!prior.valid || !prior.covariance15.allFinite() ||
      !std::isfinite(threshold) || threshold <= 0.0) return result;
  Eigen::VectorXd residual;
  Eigen::MatrixXd jacobian, noise;
  if (!linearizeLidarFactor(state, measurement, &residual, &jacobian,
                            &noise, nullptr)) return result;
  result.rank = static_cast<int>(residual.size());
  Eigen::MatrixXd innovation =
      jacobian * prior.covariance15 * jacobian.transpose() + noise;
  innovation = 0.5 * (innovation + innovation.transpose()).eval();
  Eigen::LLT<Eigen::MatrixXd> factor(innovation);
  if (!innovation.allFinite() || factor.info() != Eigen::Success) return result;
  result.nis = residual.dot(factor.solve(residual));
  if (!std::isfinite(result.nis) || result.nis < 0.0) return result;
  result.valid = true;
  result.accepted = result.nis <= threshold;
  result.status = result.accepted ? "SELECTED_NIS_ACCEPTED" : "SELECTED_NIS_REJECTED";
  return result;
}

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
