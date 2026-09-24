#pragma once

#include <Eigen/Dense>

namespace dog_prior_map_localization
{

struct ImuKinematicsConfig
{
  bool use_acc_for_position = false;
  double velocity_damping = 0.98;
  // When enabled, callers form each nominal interval input from the raw head
  // and tail samples. The legacy/default profile leaves this disabled.
  bool midpoint_interval_input_enable = false;
  bool continuous_gravity_correction_enable = true;
  double gravity_correction_expected_acc_norm = 1.0;
  double gravity_correction_gain = 0.01;
  double gravity_correction_max_angle = 0.5 * 3.14159265358979323846 / 180.0;
  double gravity_correction_acc_tolerance = 1.5;
  double gravity_correction_gyro_max = 0.8;
};

struct ImuIntervalInput
{
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

enum class ImuIntervalInputPolicy
{
  kHeadSample,
  kTailSample,
  kMidpointAverage
};

// Construct the input associated with [head_stamp, tail_stamp]. The stored
// ImuSample values themselves always remain raw samples at their own stamps.
ImuIntervalInput makeImuIntervalInput(const Eigen::Vector3d &head_acc,
                                      const Eigen::Vector3d &head_gyro,
                                      const Eigen::Vector3d &tail_acc,
                                      const Eigen::Vector3d &tail_gyro,
                                      ImuIntervalInputPolicy policy);

// The single nominal-kinematics propagation used both by the EKF and by
// causal within-scan point-time reconstruction. Covariance propagation stays
// in the EKF caller and is not repeated for per-point samples.
void propagateImuKinematics(Eigen::Vector3d &p,
                            Eigen::Vector3d &v,
                            Eigen::Matrix3d &R,
                            const Eigen::Vector3d &ba,
                            const Eigen::Vector3d &bg,
                            const Eigen::Vector3d &acc_measurement,
                            const Eigen::Vector3d &gyro_measurement,
                            double dt,
                            const Eigen::Vector3d &gravity_world,
                            const ImuKinematicsConfig &config,
                            Eigen::Vector3d &acc_world,
                            Eigen::Vector3d &gyro_unbiased);

}  // namespace dog_prior_map_localization
