#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

Eigen::Matrix3d expRotation(const Eigen::Vector3d& phi) {
  if (phi.norm() == 0.0) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(phi.norm(), phi.normalized()).toRotationMatrix();
}

// Independent residual oracle: perturb the LiDAR pose in map coordinates,
// convert its lever arm to the IMU origin, then take a body/right residual.
Eigen::Matrix<double, 6, 1> residual(
    const Eigen::Matrix3d& predicted, const Eigen::Matrix3d& measured,
    const Eigen::Vector3d& lever, double length,
    const Eigen::Matrix<double, 6, 1>& delta) {
  const Eigen::Matrix3d perturbed = expRotation(delta.head<3>()) * measured;
  Eigen::Matrix<double, 6, 1> value;
  value.head<3>() = Eigen::Vector3d(1.2, -0.7, 0.4) +
      length * delta.tail<3>() - perturbed * lever;
  const Eigen::AngleAxisd body_error(predicted.transpose() * perturbed);
  value.tail<3>() = body_error.angle() * body_error.axis();
  return value;
}

void testSo3() {
  require(so3Log(Eigen::Matrix3d::Identity()).norm() == 0.0,
          "identity log is not zero");
  for (const Eigen::Vector3d phi : {Eigen::Vector3d(1e-9, -2e-9, 3e-9),
                                  Eigen::Vector3d(0.3, -0.2, 0.1)}) {
    require((so3Log(expRotation(phi)) - phi).norm() < 1e-12,
            "Exp/Log round trip failed");
    Eigen::Matrix3d inverse;
    std::string reason;
    require(so3LeftJacobianInverse(phi, &inverse, &reason) && inverse.allFinite(),
            "left inverse is not finite");
    constexpr double epsilon = 1e-7;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3d step = epsilon * Eigen::Vector3d::Unit(axis);
      const Eigen::Vector3d fd =
          (so3Log(expRotation(step) * expRotation(phi)) -
           so3Log(expRotation(-step) * expRotation(phi))) / (2.0 * epsilon);
      require((fd - inverse.col(axis)).norm() < 1e-8,
              "left inverse finite difference failed");
    }
  }
  Eigen::Matrix3d inverse;
  std::string reason;
  require(!so3LeftJacobianInverse(Eigen::Vector3d::Zero(), nullptr, &reason) &&
              reason == "NULL_SO3_JACOBIAN_OUTPUT", "null output not rejected");
  require(!so3LeftJacobianInverse(Eigen::Vector3d(M_PI, 0, 0), &inverse, &reason) &&
              reason == "ROTATION_RESIDUAL_NEAR_PI", "near-pi not rejected");
  require(!so3LeftJacobianInverse(Eigen::Vector3d::Constant(
              std::numeric_limits<double>::quiet_NaN()), &inverse, &reason),
          "nonfinite residual not rejected");
}

double testJacobian(const Eigen::Matrix3d& predicted,
                    const Eigen::Matrix3d& measured,
                    const Eigen::Vector3d& lever) {
  constexpr double length = 0.8;
  Eigen::Matrix<double, 6, 6> analytic;
  std::string reason;
  require(normalizedRegistrationToPoseResidualJacobian(
              predicted, measured, lever, length, &analytic, &reason),
          "analytic Jacobian failed");
  constexpr double epsilon = 1e-7;
  Eigen::Matrix<double, 6, 6> fd;
  for (int column = 0; column < 6; ++column) {
    const Eigen::Matrix<double, 6, 1> step =
        epsilon * Eigen::Matrix<double, 6, 1>::Unit(column);
    fd.col(column) = (residual(predicted, measured, lever, length, step) -
                      residual(predicted, measured, lever, length, -step)) /
        (2.0 * epsilon);
  }
  const double error = (fd - analytic).cwiseAbs().maxCoeff();
  require(error < 1e-8, "registration Jacobian finite difference failed");
  require(analytic.block<3, 3>(3, 3).norm() == 0.0,
          "translation changed rotation residual");
  return error;
}

}  // namespace

int main() {
  try {
    testSo3();
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 6, 6> jacobian;
    std::string reason;
    require(normalizedRegistrationToPoseResidualJacobian(
                identity, identity, Eigen::Vector3d::Zero(), 0.8,
                &jacobian, &reason), "identity Jacobian failed");
    require(jacobian.block<3, 3>(0, 0).norm() == 0.0 &&
                (jacobian.block<3, 3>(0, 3) - 0.8 * identity).norm() == 0.0 &&
                (jacobian.block<3, 3>(3, 0) - identity).norm() == 0.0,
            "identity Jacobian blocks are wrong");
    double error = testJacobian(identity, identity, Eigen::Vector3d::Zero());
    error = std::max(error, testJacobian(identity, identity,
                                        Eigen::Vector3d(0.31, -0.17, 0.09)));
    error = std::max(error, testJacobian(expRotation(Eigen::Vector3d(0.2, -0.1, 0.15)),
        expRotation(Eigen::Vector3d(0.65, -0.25, 0.28)),
        Eigen::Vector3d(0.31, -0.17, 0.09)));
    require(!normalizedRegistrationToPoseResidualJacobian(
                identity, identity, Eigen::Vector3d::Zero(), 0.0,
                &jacobian, &reason), "invalid length scale not rejected");
    std::cout << "registration_geometry PASS max_fd_error=" << error << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "registration_geometry FAIL: " << error.what() << '\n';
    return 1;
  }
}
