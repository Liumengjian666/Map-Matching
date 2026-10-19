#include "dog_prior_map_localization/core/imu_deskew.hpp"
#include "dog_prior_map_localization/core/frame_conversions.hpp"
#include <cassert>
#include <iostream>
using namespace dog_prior_map_localization;
int main() {
  // Constant-velocity counterexample to applying an end measurement at head.
  const double head=1.0, measurement=1.008, tail=1.01;
  const double v=4.42, observed=v*(measurement-head);
  const double old_final=observed+v*(tail-head);
  const double corrected_final=observed+v*(tail-measurement);
  assert(std::abs(old_final-v*(tail-head)-v*(measurement-head)) < 1e-14);
  assert(std::abs(corrected_final-v*(tail-head)) < 1e-14);
  FilterStateSnapshot a,b;
  a.stamp=head; a.v=Eigen::Vector3d(4.42,0,0);
  a.acc_measurement=Eigen::Vector3d(.3,-.2,9.81);
  a.gyro_measurement=Eigen::Vector3d(.1,.2,-.3);
  b=a; b.stamp=tail; b.acc_measurement=Eigen::Vector3d(2,3,4);
  b.gyro_measurement=Eigen::Vector3d(4,5,6); // Future tail must NOT affect query.
  StateHistory history; history.insertMonotonic(a); history.insertMonotonic(b);
  ImuKinematicsConfig cfg; cfg.use_acc_for_position=true;
  cfg.midpoint_interval_input_enable=true; cfg.continuous_gravity_correction_enable=false;
  const Eigen::Vector3d gravity(0,0,-9.81);
  ImuDeskewPose predicted; std::string reason;
  assert(interpolateImuDeskewPose(history,measurement,.02,gravity,cfg,predicted,reason,measurement));
  auto p=a.p, vel=a.v; auto r=a.R; Eigen::Vector3d aw,gw;
  propagateImuKinematics(p,vel,r,a.ba,a.bg,a.acc_measurement,a.gyro_measurement,
                        measurement-head,gravity,cfg,aw,gw);
  assert((p-predicted.p).norm()<1e-14 && (r-predicted.R).norm()<1e-14);
  assert(predicted.latest_source_stamp==head);
  Eigen::Isometry3d il=Eigen::Isometry3d::Identity(), mi=il;
  il.translation()=Eigen::Vector3d(.049258,-.0125,.026946);
  il.linear()=Eigen::AngleAxisd(.03,Eigen::Vector3d::UnitY()).toRotationMatrix();
  mi.translation()=p; mi.linear()=r;
  assert((lidarPoseToImuPose(imuPoseToLidarPose(mi,il),il).matrix()-mi.matrix()).norm()<1e-12);
  std::cout << "PASS: early-update counterexample; causal head fragment; frame round trip\n";
}
