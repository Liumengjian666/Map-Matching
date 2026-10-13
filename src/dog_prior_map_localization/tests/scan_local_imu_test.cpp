#include "dog_prior_map_localization/core/imu_deskew.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_localization;

namespace {
void require(bool ok, const std::string &why) {
  if (!ok) throw std::runtime_error(why);
}
ScanLocalImuTrajectory build(const FilterStateSnapshot &anchor,
                            const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro,
                            const Eigen::Vector3d &gravity, double dt=0.005,
                            bool corrupt_inputs=true) {
  StateHistory history; history.insertMonotonic(anchor);
  // Deliberately corrupt every later EKF pose. Scan-local must ignore them.
  auto corrupt=anchor; corrupt.stamp=0.1; corrupt.p.setConstant(900);
  corrupt.R=Eigen::AngleAxisd(1.5,Eigen::Vector3d::UnitX()).toRotationMatrix();
  if (corrupt_inputs) history.insertMonotonic(corrupt);
  std::deque<ImuSample> imu;
  const int count=static_cast<int>(std::round(0.1/dt));
  for(int i=0;i<=count+1;++i) imu.push_back({i*dt,acc,gyro});
  // Future sample values must never enter scan_end or point-time predictions.
  if (corrupt_inputs) {
    imu.back().acc.setConstant(1e20); imu.back().gyro.setConstant(1e20);
  }
  ImuKinematicsConfig config; config.use_acc_for_position=true;
  config.continuous_gravity_correction_enable=false;
  ScanLocalImuTrajectory t; std::string reason;
  require(buildScanLocalImuTrajectory(history,imu,0,0.1,0.02,gravity,config,t,reason),reason);
  require(t.max_imu_source_stamp<=0.1,"future IMU consumed");
  return t;
}
ImuDeskewPose at(const ScanLocalImuTrajectory &t,double stamp) {
  ImuDeskewPose p; std::string reason;
  require(scanLocalImuPoseAt(t,stamp,p,reason),reason); return p;
}
Eigen::Vector3d deskew(const ScanLocalImuTrajectory &t,double stamp) {
  Eigen::Isometry3d E=Eigen::Isometry3d::Identity(); E.translation()<<.1,-.02,.03;
  Eigen::Vector3d p; std::string reason;
  require(transformPointToReferenceLidar(at(t,stamp),at(t,0),E,{2,1,3},p,reason),reason);
  return p;
}
}

int main() {
  try {
    FilterStateSnapshot s; s.stamp=0; const Eigen::Vector3d g(0,0,-9.80665);
    auto stationary=build(s,-g,Eigen::Vector3d::Zero(),g);
    require((deskew(stationary,.083)-Eigen::Vector3d(2,1,3)).norm()<1e-12,"stationary");
    s.v<<2,-1,.5;
    auto translation=build(s,-g,Eigen::Vector3d::Zero(),g);
    require((at(translation,.083).p-s.v*.083).norm()<1e-12,"constant velocity");
    require((deskew(translation,.083)-Eigen::Vector3d(2,1,3)-s.v*.083).norm()<1e-12,"translation compensation");
    s.v.setZero();
    auto rotation=build(s,-g,{0,0,1.2},g);
    require((at(rotation,.083).R-Eigen::AngleAxisd(1.2*.083,Eigen::Vector3d::UnitZ()).toRotationMatrix()).norm()<1e-12,"constant angular rate");
    s.v<<1,.2,0;
    auto combined=build(s,{.7,-.2,9.80665},{0,0,1.2},g);
    auto precise=build(s,{.7,-.2,9.80665},{0,0,1.2},g,0.0001);
    require((at(combined,.1).p-at(precise,.1).p).norm()<3e-5,"combined p reference");
    require((at(combined,.1).v-at(precise,.1).v).norm()<3e-4,"combined v reference");
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
    T.linear()=Eigen::AngleAxisd(.6,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix(); T.translation()<<4,-2,7;
    auto transformed=s; transformed.R=T.linear()*s.R;
    transformed.p=T*s.p; transformed.v=T.linear()*s.v;
    auto common=build(transformed,{.7,-.2,9.80665},{0,0,1.2},T.linear()*g);
    require((deskew(common,.083)-deskew(combined,.083)).norm()<1e-12,"common left SE3 invariance");
    const auto early=at(combined,.083);
    auto later=at(combined,.095); (void)later;
    require((early.p-at(combined,.083).p).norm()==0 &&
            (early.R-at(combined,.083).R).norm()==0,"query order");
    auto unchanged=build(s,{.7,-.2,9.80665},{0,0,1.2},g,.005,false);
    require((deskew(unchanged,.083)-deskew(combined,.083)).norm()==0,
            "future IMU or later EKF state contaminated trajectory");
    ImuDeskewPose p; std::string reason;
    require(!scanLocalImuPoseAt(combined,.100001,p,reason),"post-end extrapolation");
    require(!scanLocalImuPoseAt(combined,-.001,p,reason),"pre-start extrapolation");
    // An image clone inside the first IMU interval: causal partial anchor,
    // never consume the tail to obtain scan_start.
    StateHistory history; s.stamp=.002; history.insertMonotonic(s);
    std::deque<ImuSample> imu{{0,-g,Eigen::Vector3d::Zero()},{.005,-g,Eigen::Vector3d::Zero()},
                              {.01,-g,Eigen::Vector3d::Zero()},{.015,-g,Eigen::Vector3d::Zero()}};
    ImuKinematicsConfig cfg; cfg.use_acc_for_position=true; cfg.continuous_gravity_correction_enable=false;
    ScanLocalImuTrajectory partial;
    require(buildScanLocalImuTrajectory(history,imu,.003,.012,.02,g,cfg,partial,reason),reason);
    require((at(partial,.003).p-s.p-s.v*.001).norm()<1e-12,"partial anchor");
    require(partial.nodes.front().latest_source_stamp==0,"future anchor IMU");
    auto bad=imu; std::swap(bad[1],bad[2]);
    require(!buildScanLocalImuTrajectory(history,bad,.003,.012,.02,g,cfg,partial,reason) &&
            reason=="nonmonotonic_raw_imu_history","out-of-order raw IMU accepted");
    bad=imu; bad[2].stamp=bad[1].stamp;
    require(!buildScanLocalImuTrajectory(history,bad,.003,.012,.02,g,cfg,partial,reason) &&
            reason=="nonmonotonic_raw_imu_history","duplicate raw IMU accepted");
    std::cout<<"SCAN_LOCAL_IMU_MATH_PASS: stationary, velocity, angular rate, combined reference, common-left, causal horizon, immutable anchor/query\n";
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
