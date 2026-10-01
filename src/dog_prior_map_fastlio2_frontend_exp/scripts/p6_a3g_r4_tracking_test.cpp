#include "p6_a3g_r4_tracking.hpp"
#include <iostream>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  using namespace p6_tracking;
  Config config; config.enabled=true;
  Tracker tracker(config);
  Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();
  require(!effectiveRegistration(true,0,0,pose),"zero-iteration convergence must not reset tracking");
  require(!effectiveRegistration(true,80,1,pose),"exhausted iteration budget is not convergence");
  require(effectiveRegistration(true,1,0,pose),"stationary real iteration remains valid");
  require(!tracker.needsRecovery(false),"first failure must degrade");
  tracker.finish(1,false,pose);
  require(tracker.health()==Health::DEGRADED && tracker.lastReliableStamp()==0,"rejected anchor lifecycle");
  require(tracker.needsRecovery(false) && tracker.health()==Health::RECOVERING,"second failure triggers recovery");
  tracker.finish(2,false,pose);
  require(tracker.health()==Health::LOST && tracker.seeds(3,pose).empty(),"no fabricated reliable anchor");
  tracker.finish(1'000'000'000,true,pose);
  pose(0,3)=0.1;
  pose.block<3,3>(0,0)=Eigen::AngleAxisd(0.01,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  tracker.finish(1'100'000'000,true,pose);
  const auto seeds=tracker.seeds(1'200'000'000,Eigen::Matrix4d::Identity());
  require(seeds.size()==2 && (seeds[0].map_T_imu-pose).norm()==0,"last-observation seed must not inherit prediction drift");
  require(std::abs(seeds[1].map_T_imu(0,3)-0.2)<1e-14,"constant velocity frame/time contract");
  require(std::abs(Eigen::AngleAxisd(seeds[1].map_T_imu.block<3,3>(0,0)).angle()-0.02)<1e-14,"rotation extrapolation");
  require(tracker.seeds(3'200'000'000,pose).empty(),"stale local anchor must not become global search");
  tracker.finish(1'200'000'000,false,pose);
  require(tracker.lastReliableStamp()==1'100'000'000,"failure must preserve reliable anchor");
  tracker.reinitialized(1'300'000'000,pose);
  require(tracker.health()==Health::GOOD && tracker.cooldownFrames()==2 &&
      tracker.seeds(1'400'000'000,pose).size()==1,"reset must clear cross-epoch motion history");
  require(!tracker.needsRecovery(false),"cooldown blocks reset trigger only");
  tracker.finish(1'400'000'000,false,pose);
  require(!tracker.needsRecovery(false),"second cooldown frame still matches without reset");
  tracker.finish(1'500'000'000,true,pose);
  require(tracker.cooldownFrames()==0 && tracker.health()==Health::GOOD,
      "ordinary successful commits must remain enabled during cooldown");
  Tracker disabled(Config{});
  require(!disabled.needsRecovery(false) && disabled.seeds(1,pose).empty(),"legacy default preserved");
  std::cout<<"A3G_R4_TRACKING_HEALTH_PASS\n";
}
