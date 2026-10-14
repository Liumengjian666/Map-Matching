#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <stdexcept>
namespace p32 {
struct Imu {
  uint64_t ns=0;
  Eigen::Vector3d gyro=Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_delta=Eigen::Quaterniond::Identity();
};
// Same trapezoidal SO(3) accumulation and slerp as the existing sensor adapter.
inline void integrate(std::vector<Imu>& samples) {
  for (size_t i=0;i<samples.size();++i) {
    if (!samples[i].gyro.allFinite()) throw std::runtime_error("NONFINITE_IMU");
    if (!i) continue;
    if (samples[i].ns<=samples[i-1].ns || samples[i].ns-samples[i-1].ns>20000000)
      throw std::runtime_error("IMU_GAP_OR_ORDER");
    const double dt=double(samples[i].ns-samples[i-1].ns)*1e-9;
    const Eigen::Vector3d avg=.5*(samples[i].gyro+samples[i-1].gyro);
    const double angle=avg.norm()*dt;
    const Eigen::Quaterniond dq=angle>1e-12
      ? Eigen::Quaterniond(Eigen::AngleAxisd(angle,avg.normalized())) : Eigen::Quaterniond::Identity();
    samples[i].q_delta=(samples[i-1].q_delta*dq).normalized();
  }
}
inline Eigen::Quaterniond orientation(const std::vector<Imu>& samples,uint64_t ns) {
  auto hi=std::lower_bound(samples.begin(),samples.end(),ns,[](const Imu& s,uint64_t n){return s.ns<n;});
  if (hi!=samples.end()&&hi->ns==ns) return hi->q_delta;
  if (hi==samples.begin()||hi==samples.end()) throw std::runtime_error("NO_IMU_EXTRAPOLATION");
  const auto& a=*(hi-1);const auto& b=*hi;
  const double u=double(ns-a.ns)/double(b.ns-a.ns);
  return a.q_delta.slerp(u,b.q_delta).normalized();
}
inline Eigen::Vector3d rotatePoint(const Eigen::Matrix3d& R_i_l,const Eigen::Vector3d& t_i_l,
    const Eigen::Matrix3d& R_i0_ii,const Eigen::Vector3d& p) {
  const Eigen::Matrix3d R_l0_li_rot=R_i_l.transpose()*R_i0_ii*R_i_l;
  const Eigen::Vector3d t_l0_li_rot=R_i_l.transpose()*(R_i0_ii*t_i_l-t_i_l);
  return R_l0_li_rot*p+t_l0_li_rot;
}
}
