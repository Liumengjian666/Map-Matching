#pragma once

#include <Eigen/Core>

#include <string>

namespace dog_prior_map_fastlio2_frontend_exp {

Eigen::Matrix3d skew3(const Eigen::Vector3d& v);
Eigen::Vector3d so3Log(const Eigen::Matrix3d& rotation);
bool so3LeftJacobianInverse(const Eigen::Vector3d& phi,
                            Eigen::Matrix3d* result,
                            std::string* reason = nullptr);

// Rows: [map position residual, right/body SO(3) residual].
// Columns: [map-spatial rotation, map translation / length scale].
bool normalizedRegistrationToPoseResidualJacobian(
    const Eigen::Matrix3d& predicted_map_R_imu,
    const Eigen::Matrix3d& measured_map_R_imu,
    const Eigen::Vector3d& imu_T_lidar_translation,
    double translation_length_scale_m,
    Eigen::Matrix<double, 6, 6>* jacobian,
    std::string* reason = nullptr);

}  // namespace dog_prior_map_fastlio2_frontend_exp
