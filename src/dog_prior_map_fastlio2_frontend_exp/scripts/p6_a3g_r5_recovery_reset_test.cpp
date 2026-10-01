#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_event_adapter.hpp"
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

int main() {
  windowResetLifecycle();
}
