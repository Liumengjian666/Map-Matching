#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_production.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/lidar_residual_math.hpp"
#include <Eigen/LU>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;

namespace {
void require(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
ImuSample imu(std::uint64_t stamp) {
  ImuSample sample;
  sample.stamp_ns = stamp;
  sample.acceleration = Eigen::Vector3d(0, 0, 9.809);
  return sample;
}
FrozenLidarEvent lidar(std::uint64_t id, std::uint64_t stamp) {
  FrozenLidarEvent e;
  e.transaction_id = id;
  e.stamp_ns = stamp;
  e.ndt_converged = e.map_support_valid = true;
  e.local_risk.valid = e.local_risk.map_support_sufficient = true;
  e.local_risk.translation_length_scale_m = 0.8;
  e.local_risk.weak_dimension = 1;
  e.local_risk.reliable_dimension = 5;
  e.local_risk.joint_weak_basis.col(0) = Vector6d::Unit(3);
  e.residual_covariance = 0.01 * Matrix6d::Identity();
  return e;
}
FrozenVisualEvent visual(std::uint64_t ref, std::uint64_t cur) {
  FrozenVisualEvent e;
  e.ref_ns = e.depth_ns = ref;
  e.cur_ns = cur;
  e.source_valid = true;
  e.measurement_covariance = 0.01 * Eigen::Matrix3d::Identity();
  auto& q = e.quality;
  q.quality_metadata_available = true;
  q.detected_count = 100; q.tracked_count = 80;
  q.depth_associated_count = 70; q.pnp_inlier_count = 60;
  q.inlier_ratio = 0.75; q.depth_fraction = 0.7;
  q.grid_occupancy = 0.6; q.hull_fraction = 0.4;
  q.median_parallax_px = 2; q.reprojection_rmse_px = 0.5;
  return e;
}
void covarianceTest() {
  FixedLagWindow window;
  WindowState s;
  s.stamp_ns = 1;
  s.rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  Matrix15d information = Matrix15d::Identity();
  information.diagonal() = Vector15d::LinSpaced(15, 2.0, 16.0);
  information(0,3) = information(3,0) = 0.25;
  require(window.initializeWithPriorAtomic(s, information, Vector15d::Zero()), "init");
  WindowState next = s; next.stamp_ns = 2;
  ImuPreintegratedMeasurement m;
  m.valid = true; m.start_stamp_ns = 1; m.end_stamp_ns = 2;
  m.dt_s = 1e-9; m.covariance = 0.01 * Matrix15d::Identity();
  require(window.addStateWithImuFactorAtomic(next, 1, 1, m), "IMU extension");
  WindowMarginalCovariance p;
  std::string reason;
  require(window.latestMarginalCovariance(&p, &reason), "marginal: " + reason);
  Eigen::MatrixXd h; Eigen::VectorXd g; double cost;
  require(window.linearizedSystem(&h, &g, &cost), "linearize");
  // Test-only independent full inverse reference.
  const Matrix15d direct = h.inverse().bottomRightCorner<15,15>();
  require((p.covariance15-direct).norm() < 1e-9, "latest marginal against full inverse");
  Eigen::Matrix<double,6,15> G = Eigen::Matrix<double,6,15>::Zero();
  G.block<3,3>(0,0) = s.rotation; G.block<3,3>(3,3).setIdentity();
  require((p.map_pose_covariance6-G*direct*G.transpose()).norm() < 1e-10,
          "right body -> map left covariance cross blocks");
  for (int axis=0; axis<3; ++axis) {
    const Eigen::Vector3d v = 1e-6 * Eigen::Vector3d::Unit(axis);
    const Eigen::Matrix3d plus = s.rotation * Eigen::AngleAxisd(v.norm(),v.normalized()).toRotationMatrix();
    const Eigen::Matrix3d minus = s.rotation * Eigen::AngleAxisd(-v.norm(),v.normalized()).toRotationMatrix();
    const Eigen::Vector3d fd = (lidarSo3Log(plus*s.rotation.transpose())-
                               lidarSo3Log(minus*s.rotation.transpose()))/2e-6;
    require((fd-s.rotation.col(axis)).norm()<1e-9, "map-left FD");
  }
  FixedLagWindow singular;
  require(singular.addState(s), "singular state");
  require(!singular.latestMarginalCovariance(&p) && !p.valid &&
          p.status=="WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE", "no covariance damping fallback");
  std::cout << "COVARIANCE_DIRECT_INVERSE_AND_MAP_LEFT_FD_PASS\n";
}
void adapterTest() {
  FixedLagOptions options;
  options.maximum_duration_s=10;
  options.maximum_optimizer_iterations=12;
  FixedLagEventAdapter adapter(options);
  WindowState start; start.stamp_ns=1'000'000'000;
  require(adapter.initialize(start, Matrix15d::Identity(), Vector15d::Zero()),"adapter init");
  for (auto t : {1'000'000'000ULL,1'100'000'000ULL,1'200'000'000ULL})
    require(adapter.appendImu(imu(t)),"imu");
  WindowState predicted;
  require(adapter.prepareStateAt(1'100'000'000,&predicted),"public prediction");
  require(adapter.summary().lidar_factor_count==0 && adapter.summary().imu_factor_count==1,
          "prepare only adds state/IMU");
  FrozenLidarEvent e=lidar(1,1'100'000'000);
  e.map_T_lidar.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(0.4,Eigen::Vector3d::UnitZ()));
  Matrix6d stale;
  require(normalizedLidarResidualJacobian(predicted,e.map_T_lidar.orientation.toRotationMatrix(),
          Eigen::Vector3d::Zero(),0.8,&stale),"stale reference");
  require(adapter.processLidarEvent(e),"lidar admission");
  std::string reason;
  require(adapter.optimizeCurrentWindow(&reason),"optimization: "+reason);
  WindowState optimized;
  require(adapter.prepareStateAt(e.stamp_ns,&optimized),"read optimized risk state");
  require((optimized.rotation-predicted.rotation).norm()>0.1,"risk state changed visibly");
  require(adapter.processVisualEvent(visual(e.stamp_ns,1'200'000'000),&reason),"visual: "+reason);
  const auto& admitted=adapter.lastVisualAdmission();
  Matrix6d current;
  require(normalizedLidarResidualJacobian(optimized,e.map_T_lidar.orientation.toRotationMatrix(),
          Eigen::Vector3d::Zero(),0.8,&current),"current reference");
  require((current-stale).norm()>0.1 &&
          (admitted.admission_exact_jacobian-current).norm()<1e-12,"admission recomputes actual A");
  require((current.topRows(3)-stale.topRows(3)).norm()<1e-12,
          "Wp is state independent under this frozen measurement model");
  const Eigen::Matrix3d projector=admitted.measurement_basis.leftCols(admitted.selected_rank)*
      admitted.measurement_basis.leftCols(admitted.selected_rank).transpose();
  require((projector-Eigen::Vector3d::UnitX()*Eigen::Vector3d::UnitX().transpose()).norm()<1e-10,
          "frozen Qw matches recomputed translation subspace");
  FrozenLidarEvent regressed=lidar(2,1'050'000'000);
  regressed.ndt_converged=false;
  require(!adapter.processLidarEvent(regressed,&reason) && reason=="lidar_timestamp_regression",
          "regression including non-converged rejected");
  regressed.ndt_converged=true; regressed.map_support_valid=false;
  require(!adapter.processLidarEvent(regressed,&reason) && reason=="lidar_timestamp_regression",
          "regression including invalid map support rejected");
  FrozenLidarEvent rejected=lidar(2,1'200'000'000);
  rejected.measurement_commit_allowed=false;
  const auto before=adapter.summary().lidar_factor_count;
  require(!adapter.processLidarEvent(rejected,&reason) && reason=="LIDAR_MEASUREMENT_REJECTED" &&
          adapter.summary().lidar_factor_count==before,"rejected LiDAR never enters factor and watermark unchanged by regression");
  require(adapter.prepareStateAt(rejected.stamp_ns,&predicted),"premeasurement prediction");
  WindowMarginalCovariance prior;
  const bool covariance_available=adapter.latestMarginalCovariance(&prior,&reason);
  require(covariance_available,"premeasurement covariance: "+reason);
  LidarWindowMeasurement measurement;
  require(adapter.previewLidarMeasurement(rejected,&measurement),"measurement preview");
  measurement.measured_position.y()=100;
  const auto nis=evaluateSelectedLidarNis(predicted,measurement,prior,16.812);
  require(nis.valid && !nis.accepted && nis.rank==5,"selected premeasurement NIS rejects conflict");
  require(adapter.appendImu(imu(1'300'000'000)),"post rejection IMU");
  require(adapter.processVisualReferenceStamp(1'200'000'000),"post rejection reference");
  require(adapter.processVisualEvent(visual(1'200'000'000,1'300'000'000),&reason),
          "relative visual fallback after rejected LiDAR: "+reason);
  require(adapter.lastVisualAdmission().mode==VisualFactorMode::FULL_TRANSLATION &&
          adapter.lastVisualAdmission().selected_rank==3 &&
          adapter.lastVisualAdmission().trigger_status=="LIDAR_MEASUREMENT_REJECTED" &&
          adapter.summary().lidar_factor_count==before,
          "rejected LiDAR is not presented as committed global correction");
  std::cout << "PUBLIC_PREDICTION_NIS_REJECT_CAUSALITY_AND_ADMISSION_RECOMPUTE_PASS\n";
}
void expiredRiskTest() {
  FixedLagOptions options; options.maximum_nodes=2; options.maximum_duration_s=10;
  FixedLagEventAdapter adapter(options);
  WindowState start; start.stamp_ns=1'000'000'000;
  require(adapter.initialize(start,Matrix15d::Identity(),Vector15d::Zero()),"expiry init");
  for(auto t:{1'000'000'000ULL,1'100'000'000ULL,1'200'000'000ULL,1'300'000'000ULL})
    require(adapter.appendImu(imu(t)),"expiry IMU");
  require(adapter.processLidarEvent(lidar(1,1'100'000'000)),"expiry risk");
  WindowState s;
  require(adapter.prepareStateAt(1'200'000'000,&s)&&adapter.optimizeCurrentWindow(),"expiry advance 1");
  require(adapter.prepareStateAt(1'300'000'000,&s)&&adapter.optimizeCurrentWindow(),"expiry advance 2");
  std::string reason;
  require(!adapter.processVisualEvent(visual(1'200'000'000,1'300'000'000),&reason)&&
          reason=="LIDAR_RISK_STATE_NOT_IN_ACTIVE_WINDOW"&&adapter.summary().visual_factor_count==0,
          "marginalized risk state cannot route from stale Jacobian");
  std::cout<<"MARGINALIZED_RISK_DIRECTION_REJECT_PASS\n";
}
void schedulerNoiseTest() {
  std::vector<ProducerEvent> events={{20,ProducerEventType::VISUAL_REFERENCE,1},
      {20,ProducerEventType::VISUAL_CURRENT,0},{10,ProducerEventType::VISUAL_REFERENCE,0},
      {20,ProducerEventType::LIDAR_SCAN,0}};
  sortProducerEvents(&events);
  require(events[0].stamp_ns==10 && events[1].type==ProducerEventType::LIDAR_SCAN &&
          events[2].type==ProducerEventType::VISUAL_CURRENT &&
          events[3].type==ProducerEventType::VISUAL_REFERENCE,"sensor scheduler tie order");
  RuntimeParameters p;
  const auto n=makeWindowImuNoise(p,Eigen::Vector3d(0,0,-9.7));
  require(n.gyro_noise_density==p.gyro_noise_std_rad_s &&
          n.accel_noise_density==p.accel_noise_std_m_s2 &&
          n.gyro_bias_random_walk==p.gyro_bias_rw_std_rad_s2 &&
          n.accel_bias_random_walk==p.accel_bias_rw_std_m_s3 && n.gravity.z()==-9.7,
          "real RuntimeParameters noise mapping");
  std::cout << "SCHEDULER_AND_RUNTIME_NOISE_PASS\n";
}
}
int main() {
  try { covarianceTest(); adapterTest(); expiredRiskTest(); schedulerNoiseTest(); return 0; }
  catch(const std::exception& e) { std::cerr<<"A2C_TEST_FAIL: "<<e.what()<<'\n'; return 1; }
}
