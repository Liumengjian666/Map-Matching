#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_production.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/window_scan_processor.hpp"
#include <iostream>
#include <stdexcept>
using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
static void check(bool ok,const std::string& text) {if(!ok)throw std::runtime_error(text);}
static FrozenVisualEvent visual() {
  FrozenVisualEvent v;
  v.ref_ns=v.depth_ns=1100000000; v.cur_ns=1150000000;
  v.translation_ref_imu=Eigen::Vector3d(.2,.01,0);
  v.source_valid=true; v.measurement_covariance=.0025*Eigen::Matrix3d::Identity();
  auto& q=v.quality; q.quality_metadata_available=true;
  q.detected_count=100;q.tracked_count=80;q.depth_associated_count=70;q.pnp_inlier_count=60;
  q.inlier_ratio=.75;q.depth_fraction=.7;q.grid_occupancy=.6;q.hull_fraction=.4;
  q.median_parallax_px=2;q.reprojection_rmse_px=.5;
  return v;
}
int main() {
  std::string reason;
  check(rawDeskewInputAllowed(LidarCloudProvenance::RAW_TIMED_SENSOR,&reason),"raw gate");
  for(auto p:{LidarCloudProvenance::LEGACY_STATE_DERIVED_SE3_DESKEW,
              LidarCloudProvenance::WINDOW_OWNED_SE3_DESKEW,LidarCloudProvenance::SENSOR_LOCAL_ROTATION_ONLY})
    check(!rawDeskewInputAllowed(p,&reason),"already deskewed/rotation-only must not be deskewed as raw");
  FixedLagAdapterCalibration calibration; calibration.allow_compatibility_visual_inputs=false;
  FixedLagOptions options; options.maximum_optimizer_iterations=12;
  FixedLagEventAdapter adapter(options,{},calibration);
  WindowState initial; initial.stamp_ns=1000000000;
  check(adapter.initialize(initial,Matrix15d::Identity(),Vector15d::Zero()),"initialize");
  std::vector<ImuSample,Eigen::aligned_allocator<ImuSample>> imu;
  for(std::uint64_t t=1000000000;t<=1200000000;t+=5000000) {
    ImuSample sample; sample.stamp_ns=t; sample.acceleration=Eigen::Vector3d(0,0,9.809);
    imu.push_back(sample);check(adapter.appendImu(sample),"append");
  }
  WindowState start_snapshot;
  check(adapter.prepareStateAt(1100000000,&start_snapshot),"scan start node");
  FrozenLidarEvent risk;
  risk.transaction_id=1; risk.stamp_ns=start_snapshot.stamp_ns;
  risk.ndt_converged=risk.map_support_valid=true;
  risk.local_risk.valid=risk.local_risk.map_support_sufficient=true;
  risk.local_risk.translation_length_scale_m=.8;
  risk.local_risk.weak_dimension=1; risk.local_risk.reliable_dimension=5;
  risk.local_risk.joint_weak_basis.col(0)=Vector6d::Unit(3);
  risk.residual_covariance=.01*Matrix6d::Identity();
  check(adapter.processLidarEvent(risk,&reason),reason);
  check(adapter.optimizeCurrentWindow(&reason),reason);
  auto event=visual();
  const auto revision=adapter.summary().window_revision;
  for(auto p:{VisualMeasurementProvenance::LEGACY_STATE_DERIVED_DEPTH,VisualMeasurementProvenance::UNKNOWN}) {
    event.provenance=p;
    check(!adapter.processVisualEvent(event,&reason) && reason=="VISUAL_PROVENANCE_REJECTED_NOT_FORMAL_INPUT",reason);
    check(adapter.summary().window_revision==revision && adapter.summary().visual_factor_count==0,"rejected visual changed graph");
  }
  event.provenance=VisualMeasurementProvenance::WINDOW_OWNED_DEPTH;
  check(adapter.processVisualEvent(event,&reason),reason);
  check(adapter.optimizeCurrentWindow(&reason),reason);
  WindowState optimized_start, end;
  check(adapter.activeStateAt(1100000000,&optimized_start,&reason),reason);
  const double start_change=localDifference(optimized_start,start_snapshot).norm();
  check(start_change>1e-5,"visual did not update scan-start state");
  check(adapter.prepareStateAt(1200000000,&end,&reason),reason);
  std::vector<TimedLidarPoint,Eigen::aligned_allocator<TimedLidarPoint>> cloud(2);
  cloud[0].position=Eigen::Vector3d(1,2,3); cloud[0].stamp_ns=1100000000;
  cloud[1]=cloud[0]; cloud[1].stamp_ns=1200000000;
  WindowDeskewResult current,stale;
  check(deskewScanWithWindowState(optimized_start,1100000000,1200000000,imu,{},Pose3d(),cloud,&current,&reason),reason);
  check(deskewScanWithWindowState(start_snapshot,1100000000,1200000000,imu,{},Pose3d(),cloud,&stale,&reason),reason);
  check((current.cloud_end_frame[0].position-stale.cloud_end_frame[0].position).norm()>1e-5,
        "cached scan-start pose would escape test");
  std::vector<ProducerEvent> events={{10,ProducerEventType::VISUAL_REFERENCE,0},
      {10,ProducerEventType::LIDAR_SCAN_END,0},{10,ProducerEventType::LIDAR_SCAN_START,0},
      {9,ProducerEventType::VISUAL_CURRENT,1},{10,ProducerEventType::VISUAL_CURRENT,0}};
  sortProducerEvents(&events);
  check(events[0].stamp_ns==9 && events[1].type==ProducerEventType::LIDAR_SCAN_START &&
      events[2].type==ProducerEventType::LIDAR_SCAN_END && events[3].type==ProducerEventType::VISUAL_CURRENT,
      "explicit tie comparator");
  FixedLagOptions tiny; tiny.maximum_nodes=2;
  FixedLagEventAdapter expired(tiny);
  check(expired.initialize(initial,Matrix15d::Identity(),Vector15d::Zero()),"expiry initialize");
  for(const auto& sample:imu) check(expired.appendImu(sample),"expiry imu");
  for(auto stamp:{1100000000ULL,1150000000ULL,1200000000ULL}) {
    check(expired.prepareStateAt(stamp,&end,&reason),reason);
    check(expired.optimizeCurrentWindow(&reason),reason);
  }
  check(!expired.activeStateAt(1100000000,&end,&reason) &&
      reason=="WINDOW_SCAN_START_NOT_IN_ACTIVE_WINDOW","marginalized scan start reused");
  check(formalVisualInputAllowed(VisualMeasurementProvenance::RAW_SENSOR_LOCAL_DEPTH),"raw visual");
  // Nonzero rotation + projected, frozen basis: analytic state derivative
  // must retain the existing outer basis relinearization contract.
  double lidar_error=0;
  for(double angle:{.4,2.8,3.1413}) for(int rank:{3,6}) {
    WindowState s=initial; s.rotation=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()).toRotationMatrix();
    LidarWindowMeasurement m; m.valid=true;m.stamp_ns=s.stamp_ns;m.reliable_rank=rank;
    m.measured_rotation=s.rotation*Eigen::AngleAxisd(angle,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    m.measured_position=Eigen::Vector3d(.1,-.2,.3);m.covariance=.01*Matrix6d::Identity();
    m.measurement_basis.block<3,3>(0,0)=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitY()).toRotationMatrix();
    const auto basis=m.measurement_basis;
    m.basis_relinearizer=[basis,rank](const WindowState&,Matrix6d* q,int* r,std::string*) {*q=basis;*r=rank;return true;};
    Eigen::VectorXd r,rf;Eigen::MatrixXd a,af,c,cf;
    check(linearizeLidarFactor(s,m,&r,&a,&c,&reason),reason);
    check(linearizeLidarFactorFiniteDifferenceReference(s,m,&rf,&af,&cf,&reason),reason);
    lidar_error=std::max(lidar_error,(a-af).cwiseAbs().maxCoeff());
    check((r-rf).norm()==0 && (c-cf).norm()==0,"LiDAR residual/covariance changed");
  }
  check(lidar_error<2e-7,"LiDAR analytic Jacobian");
  std::cout<<"PASS provenance gates scan_start_reoptimized_delta="<<start_change
      <<" lidar_analytic_FD_max="<<lidar_error<<'\n';
}
