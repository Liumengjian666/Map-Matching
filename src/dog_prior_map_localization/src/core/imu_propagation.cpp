#include "dog_prior_map_localization/core/imu_propagation.hpp"

#include "dog_prior_map_localization/core/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace dog_prior_map_localization
{

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
                            Eigen::Vector3d &gyro_unbiased)
{
  const Eigen::Vector3d acc = acc_measurement - ba;
  const Eigen::Vector3d gyr = gyro_measurement - bg;
  const double gyro_norm = gyr.norm();
  const Eigen::Matrix3d dR = Eigen::AngleAxisd(
      gyro_norm * dt,
      gyro_norm > 1e-12 ? gyr.normalized() : Eigen::Vector3d::UnitX()).toRotationMatrix();
  R = R * dR;

  if (config.continuous_gravity_correction_enable)
  {
    const double acc_norm = acc_measurement.norm();
    const double gyro_measurement_norm = gyro_measurement.norm();
    if (acc_norm > 1e-3 &&
        std::abs(acc_norm - config.gravity_correction_expected_acc_norm) <=
            config.gravity_correction_acc_tolerance &&
        gyro_measurement_norm <= config.gravity_correction_gyro_max)
    {
      const Eigen::Vector3d measured_up_world = (R * acc_measurement.normalized()).normalized();
      const Eigen::Vector3d expected_up_world = Eigen::Vector3d::UnitZ();
      Eigen::Quaterniond q_full;
      q_full.setFromTwoVectors(measured_up_world, expected_up_world);
      Eigen::AngleAxisd aa(q_full);
      Eigen::Vector3d rotvec = aa.axis() * aa.angle();
      rotvec = limitVector(rotvec * config.gravity_correction_gain,
                           config.gravity_correction_max_angle);
      if (rotvec.allFinite() && rotvec.norm() > 1e-12)
      {
        R = Eigen::AngleAxisd(rotvec.norm(), rotvec.normalized()).toRotationMatrix() * R;
      }
    }
  }

  acc_world = R * acc + gravity_world;
  gyro_unbiased = gyr;
  if (config.use_acc_for_position)
  {
    p = p + v * dt + 0.5 * acc_world * dt * dt;
    v = v + acc_world * dt;
  }
  else
  {
    p = p + v * dt;
    const double damping = std::max(0.0, std::min(1.0, config.velocity_damping));
    v *= std::pow(damping, dt * 200.0);
  }
}

}  // namespace dog_prior_map_localization
