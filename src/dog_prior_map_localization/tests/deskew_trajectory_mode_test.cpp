#include "dog_prior_map_localization/core/deskew_trajectory_mode.hpp"
#include "dog_prior_map_localization/core/imu_deskew.hpp"
#include <iostream>

using namespace dog_prior_map_localization;
void require(bool ok, const char *why) { if (!ok) throw std::runtime_error(why); }

int main() {
  try {
    using Mode = DeskewTrajectoryMode;
    require(deskewTrajectoryMode(false,false)==Mode::LEGACY_HISTORY,"default mode");
    require(deskewTrajectoryMode(false,true)==Mode::HISTORY_NOMINAL_CONSISTENT,"D mode");
    require(deskewTrajectoryMode(true,false)==Mode::SCAN_LOCAL,"C/E mode");
    bool rejected=false;
    try { deskewTrajectoryMode(true,true); } catch (const std::runtime_error &) { rejected=true; }
    require(rejected,"mutual exclusion");
    ImuKinematicsConfig global; global.use_acc_for_position=true;
    global.midpoint_interval_input_enable=true; global.continuous_gravity_correction_enable=true;
    auto d=deskewTrajectoryConfig(global,Mode::HISTORY_NOMINAL_CONSISTENT);
    auto legacy=deskewTrajectoryConfig(global,Mode::LEGACY_HISTORY);
    require(!d.midpoint_interval_input_enable && !d.continuous_gravity_correction_enable,"local ZOH flags");
    require(global.midpoint_interval_input_enable && global.continuous_gravity_correction_enable,"global mutated");
    require(legacy.midpoint_interval_input_enable && legacy.continuous_gravity_correction_enable,"legacy changed");
    require(d.gravity_correction_gain==global.gravity_correction_gain &&
            d.velocity_damping==global.velocity_damping && d.use_acc_for_position==global.use_acc_for_position,"physical config changed");
    StateHistory history; FilterStateSnapshot s; s.stamp=0; s.v.x()=2;
    const Eigen::Vector3d g(0,0,-9.80665); s.acc_measurement=-g;
    history.insertMonotonic(s);
    auto corrected=s; corrected.stamp=.005; corrected.p.x()=7;
    corrected.acc_measurement<<1e6,2e6,3e6; history.insertMonotonic(corrected);
    ImuDeskewPose p; std::string why;
    require(interpolateImuDeskewPose(history,.004,.02,g,d,p,why,.004),why.c_str());
    require((p.p-Eigen::Vector3d(.008,0,0)).norm()<1e-12 && p.latest_source_stamp==0,"D head-only partial");
    require(interpolateImuDeskewPose(history,.004,.02,g,d,p,why,.005),why.c_str());
    require((p.p-Eigen::Vector3d(.008,0,0)).norm()<1e-12 && p.latest_source_stamp==0,"D must ignore admitted tail");
    ImuDeskewPose old_pose;
    require(interpolateImuDeskewPose(history,.004,.02,g,legacy,old_pose,why,.005),why.c_str());
    require(old_pose.latest_source_stamp==.005 && (p.p-old_pose.p).norm()>1,"legacy midpoint was lost");
    require(interpolateImuDeskewPose(history,.005,.02,g,d,p,why,.005),why.c_str());
    require(p.p.x()==7 && why=="exact_state_sample","D must retain historical corrections");
    std::deque<ImuSample> raw{{0,-g,Eigen::Vector3d::Zero()},
                             {.005,-g,Eigen::Vector3d::Zero()}};
    ScanLocalImuTrajectory before,after;
    auto old_c=global; old_c.continuous_gravity_correction_enable=false;
    auto c=deskewTrajectoryConfig(global,Mode::SCAN_LOCAL);
    require(buildScanLocalImuTrajectory(history,raw,0,.005,.02,g,old_c,before,why),why.c_str());
    require(buildScanLocalImuTrajectory(history,raw,0,.005,.02,g,c,after,why),why.c_str());
    require(scanLocalImuPoseAt(after,.005,p,why),why.c_str());
    require(std::abs(p.p.x()-.01)<1e-12,"C must ignore later historical correction");
    require(before.nodes.size()==after.nodes.size(),"C node count changed");
    for (std::size_t i=0;i<before.nodes.size();++i)
      require((before.nodes[i].p-after.nodes[i].p).norm()==0 &&
              (before.nodes[i].R-after.nodes[i].R).norm()==0,"P30 C path changed");
    require(history.samples()[1].p.x()==7,"historical state mutated");
    std::cout<<"DESKEW_TRAJECTORY_MODE_PASS: defaults, exclusivity, local flags, history queries, causal head, unchanged C and global state\n";
    return 0;
  } catch (const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
