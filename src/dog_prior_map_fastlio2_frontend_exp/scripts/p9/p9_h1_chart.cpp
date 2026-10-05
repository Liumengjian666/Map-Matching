// Math-only extraction of the P9-R1 float pose carrier and map product chart.
// No PCL dependency, optimizer, map, or trajectory reader.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

extern "C" void p9_h1_displacement(const double* xyz_qxyzw,
                                   const double* matrix16, double* output) {
  Eigen::Matrix4f from = Eigen::Matrix4f::Identity(), to;
  Eigen::Quaternionf q(static_cast<float>(xyz_qxyzw[6]),
                       static_cast<float>(xyz_qxyzw[3]),
                       static_cast<float>(xyz_qxyzw[4]),
                       static_cast<float>(xyz_qxyzw[5]));
  q.normalize();
  from.block<3, 3>(0, 0) = q.toRotationMatrix();
  for (int i = 0; i < 3; ++i) from(i, 3) = static_cast<float>(xyz_qxyzw[i]);
  for (int i = 0; i < 16; ++i) to(i / 4, i % 4) = static_cast<float>(matrix16[i]);
  // Same operations as p9_ndt_energy_contract.cpp::mapChartDisplacement().
  Eigen::Matrix<double, 6, 1> eta;
  eta.head<3>() = (to.block<3, 1>(0, 3) - from.block<3, 1>(0, 3)).cast<double>() / 0.8;
  const Eigen::Quaterniond q_from(from.block<3, 3>(0, 0).cast<double>());
  const Eigen::Quaterniond q_to(to.block<3, 3>(0, 0).cast<double>());
  Eigen::Quaterniond delta = q_to * q_from.conjugate();
  delta.normalize();
  if (delta.w() < 0.0) delta.coeffs() *= -1.0;
  const double vector_norm = delta.vec().norm();
  const double angle = 2.0 * std::atan2(vector_norm, delta.w());
  eta.tail<3>().setZero();
  if (vector_norm >= 1e-12) eta.tail<3>() = angle * delta.vec() / vector_norm;
  for (int i = 0; i < 6; ++i) output[i] = eta[i];
}
