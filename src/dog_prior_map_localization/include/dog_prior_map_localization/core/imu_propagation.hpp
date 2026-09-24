#pragma once

#include <Eigen/Dense>

namespace dog_prior_map_localization
{

struct ImuKinematicsConfig
{
  bool use_acc_for_position = false;
  double velocity_damping = 0.98;
  bool continuous_gravity_correction_enable = true;
  double gravity_correction_expected_acc_norm = 1.0;
  double gravity_correction_gain = 0.01;
  double gravity_correction_max_angle = 0.5 * 3.14159265358979323846 / 180.0;
  double gravity_correction_acc_tolerance = 1.5;
  double gravity_correction_gyro_max = 0.8;
};

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
