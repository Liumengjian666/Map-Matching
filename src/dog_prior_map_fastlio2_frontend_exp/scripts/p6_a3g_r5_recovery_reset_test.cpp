#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_production.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp;
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
void require(bool value, const std::string& reason) {
  if (!value) throw std::runtime_error(reason);
}

void windowResetLifecycle() {
  FixedLagOptions options;
  options.marginalization_backend = MarginalizationBackend::SQUARE_ROOT_QR;
  options.marginal_covariance_backend = MarginalCovarianceBackend::SQUARE_ROOT_QR;
  FixedLagWindow window(options);
  WindowState a; a.stamp_ns=1'000'000'000;
  WindowState b=a; b.stamp_ns+=100'000'000; b.position.x()=.1;
  std::string reason;
  require(window.initializeWithPriorAtomic(a,100*Matrix15d::Identity(),
      Vector15d::Constant(.01),&reason),reason);
  ImuPreintegratedMeasurement imu; imu.valid=true;
  imu.start_stamp_ns=a.stamp_ns; imu.end_stamp_ns=b.stamp_ns; imu.dt_s=.1;
  imu.covariance=Matrix15d::Identity();
  require(window.addStateWithImuFactorAtomic(b,1,a.stamp_ns,imu,&reason),reason);
  LidarWindowMeasurement lidar; lidar.observation_id=2; lidar.stamp_ns=b.stamp_ns;
  lidar.measured_position=b.position; lidar.valid=true;
  require(window.addLidarFactor(lidar,&reason),reason);
  VisualRelativeMeasurement visual; visual.observation_id=3;
  visual.reference_stamp_ns=a.stamp_ns; visual.current_stamp_ns=b.stamp_ns;
  visual.valid=true;
  require(window.addVisualFactor(visual,&reason),reason);
  WindowState recovered=b; recovered.position=Eigen::Vector3d(2,3,4);
  recovered.rotation=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto old_h=window.priorInformation();
  Matrix15d invalid=Matrix15d::Identity(); invalid(0,0)=-1;
  require(!window.resetFromValidatedRecovery(recovered,invalid,4,&reason) &&
      window.states().size()==2 && (window.priorInformation()-old_h).norm()==0,
      "invalid recovery prior must be atomic");
  require(window.resetFromValidatedRecovery(recovered,20*Matrix15d::Identity(),4,&reason),reason);
  const auto s=window.summary();
  require(s.window_node_count==1 && s.imu_factor_count==0 && s.lidar_factor_count==0 &&
      s.visual_factor_count==0 && s.active_observation_id_count==0 &&
      s.retired_observation_id_watermark==4,"reset must discard every old factor and retire IDs");
  require(localDifference(*window.latestState(),recovered).norm()==0 &&
      window.priorGradient().norm()==0 && window.priorSquareRootRows().a.cols()==15 &&
      window.priorSquareRootRows().a.allFinite() && window.priorSquareRootRows().b.allFinite(),
      "recovered state is the zero-gradient square-root reference");
  require(s.optimizer_status=="NOT_RUN" && !s.prediction_feedback_ready &&
      s.optimized_revision==0,"old optimized revision must not survive reset");
  require(!window.addLidarFactor(lidar,&reason) && reason=="retired_measurement_id",
      "old observation ID must never re-enter");
  lidar.observation_id=5; lidar.measured_position=recovered.position;
  lidar.measured_rotation=recovered.rotation;
  require(window.addLidarFactor(lidar,&reason),reason);
  require(window.optimize(&reason),reason);
  std::cout<<"R5_WINDOW_ATOMIC_RESET_PASS old_factors=0 retired_watermark=4\n";
}

void adapterResetContract() {
  FixedLagOptions options;
  options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  options.marginal_covariance_backend=MarginalCovarianceBackend::SQUARE_ROOT_QR;
  FixedLagAdapterCalibration calibration;
  calibration.T_imu_lidar.position=Eigen::Vector3d(.1,.2,.3);
  calibration.T_imu_lidar.orientation=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitX());
  FixedLagEventAdapter adapter(options,{},calibration);
  WindowState initial; initial.stamp_ns=1'000'000'000;
  initial.rotation=Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY()).toRotationMatrix();
  initial.velocity=Eigen::Vector3d(1,2,3);
  initial.gyro_bias=Eigen::Vector3d(.01,.02,.03);
  initial.accel_bias=Eigen::Vector3d(.1,.2,.3);
  std::string reason;
  require(adapter.initialize(initial,100*Matrix15d::Identity(),Vector15d::Zero(),&reason),reason);
  for (int i=0;i<=10;++i) {
    ImuSample sample; sample.stamp_ns=initial.stamp_ns+i*10'000'000;
    sample.acceleration=Eigen::Vector3d(0,0,9.81);
    require(adapter.appendImu(sample,&reason),reason);
  }
  WindowState propagated;
  require(adapter.prepareStateAt(initial.stamp_ns+100'000'000,&propagated,&reason),reason);
  RecoveryReinitializationRequest request;
  auto& event=request.registration;
  event.transaction_id=10; event.stamp_ns=propagated.stamp_ns;
  event.ndt_converged=true; event.map_support_valid=true;
  event.measurement_commit_allowed=false; // Stale normal NIS rejection.
  event.map_T_lidar.position=Eigen::Vector3d(100,3,4);
  event.map_T_lidar.orientation=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ());
  event.local_risk.valid=true; event.local_risk.reliable_dimension=6;
  event.local_risk.map_support_sufficient=true;
  event.local_risk.map_support_correspondences=100;
  event.local_risk.joint_reliable_basis.setIdentity();
  event.local_risk.translation_length_scale_m=1;
  event.residual_covariance=.04*Matrix6d::Identity();
  event.residual_covariance(0,4)=event.residual_covariance(4,0)=.005;
  request.iterations=15; request.objective=1; request.fitness=.5;
  request.measurement_covariance_available=true;
  require(adapter.latestMarginalCovariance(&request.premeasurement_covariance,&reason),reason);
  const auto old_id=adapter.nextObservationId();
  const auto old_prior=adapter.debugWindowForDiagnostics()->priorInformation();
  LidarWindowMeasurement preview;
  require(adapter.previewLidarMeasurement(event,&preview,&reason),reason);
  const auto nis=evaluateSelectedLidarNis(propagated,preview,request.premeasurement_covariance,
                                        16.812);
  require(nis.valid && !nis.accepted,"normal selected NIS must still reject stale large innovation");
  const auto check_rejected=[&](const RecoveryReinitializationRequest& bad) {
    require(!adapter.resetFromValidatedRecovery(bad,&reason),"invalid recovery must fail");
    const auto* window=adapter.debugWindowForDiagnostics();
    require(window->states().size()==2 && localDifference(*window->latestState(),propagated).norm()==0 &&
        (window->priorInformation()-old_prior).norm()==0 && adapter.nextObservationId()==old_id,
        "failed validation must preserve Window and global identity");
  };
  auto bad=request; bad.iterations=0; check_rejected(bad);
  bad=request; bad.initial_map_T_lidar=event.map_T_lidar; check_rejected(bad);
  bad=request; bad.registration.local_risk.map_support_correspondences=29; check_rejected(bad);
  bad=request; bad.registration.local_risk.reliable_dimension=0; check_rejected(bad);
  bad=request; bad.premeasurement_covariance.covariance15(6,6)=-1; check_rejected(bad);
  bad=request; bad.measurement_covariance_available=false; check_rejected(bad);
  require(adapter.resetFromValidatedRecovery(request,&reason),reason);
  const auto* window=adapter.debugWindowForDiagnostics();
  const auto& recovered=*window->latestState();
  const Eigen::Matrix3d recovered_rotation=(event.map_T_lidar.orientation*
      calibration.T_imu_lidar.orientation.conjugate()).toRotationMatrix();
  require((recovered.rotation-recovered_rotation).norm()<1e-14 &&
      (recovered.position-(event.map_T_lidar.position-recovered_rotation*
          calibration.T_imu_lidar.position)).norm()<1e-14,"correct LiDAR-to-IMU recovery pose");
  require((recovered.velocity-propagated.velocity).norm()==0 &&
      (recovered.gyro_bias-propagated.gyro_bias).norm()==0 &&
      (recovered.accel_bias-propagated.accel_bias).norm()==0,"v/bg/ba must be inherited exactly");
  const Matrix15d actual_covariance=window->priorInformation().llt().solve(Matrix15d::Identity());
  Matrix6d chart=Matrix6d::Zero(); chart.block<3,3>(0,3)=recovered_rotation.transpose()*propagated.rotation;
  chart.block<3,3>(3,0).setIdentity();
  require((actual_covariance.topLeftCorner<6,6>()-chart*event.residual_covariance*chart.transpose()).norm()<1e-13 &&
      (actual_covariance.bottomRightCorner<9,9>()-request.premeasurement_covariance.covariance15.bottomRightCorner<9,9>()).norm()<1e-13 &&
      actual_covariance.topRightCorner<6,9>().norm()==0 && window->priorGradient().norm()==0,
      "recovery block prior must remove stale pose/non-pose correlations");
  require(adapter.nextObservationId()==old_id+1 && window->summary().retired_observation_id_watermark==old_id,
      "global observation identity must survive reset");
  require(!adapter.resetFromValidatedRecovery(request,&reason) &&
      !adapter.processLidarEvent(event,&reason),"same source cannot be re-admitted after reset");
  require(adapter.optimizeCurrentWindow(&reason),reason);
  std::cout<<"R5_ADAPTER_RESET_PASS stale_NIS="<<nis.nis
           <<" normal_NIS=REJECT reset=SUCCESS nonpose=EXACT gradient=ZERO\n";
}

int main() {
  windowResetLifecycle();
  adapterResetContract();
}
