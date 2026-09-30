#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <stdexcept>
namespace a2d_test {
using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
inline void check(bool ok,const std::string& text) { if (!ok) throw std::runtime_error(text); }
inline void append(FixedLagWindow* window, int index) {
  WindowState state;
  state.stamp_ns = 1000000000ULL + 25000000ULL*index;
  state.rotation = Eigen::AngleAxisd(.01*index,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  state.position = Eigen::Vector3d(.025*index,.001*index,0);
  state.velocity = Eigen::Vector3d(1,.04,0);
  state.gyro_bias = Eigen::Vector3d(.003,-.002,.001);
  state.accel_bias = Eigen::Vector3d(.01,-.02,.03);
  std::string reason;
  if (index == 0) {
    check(window->initializeWithPriorAtomic(state,100*Matrix15d::Identity(),
        Vector15d::LinSpaced(15,-.03,.04),&reason),reason);
  } else {
    const WindowState previous=*window->latestState();
    ImuPreintegratedMeasurement m;
    m.start_stamp_ns=previous.stamp_ns; m.end_stamp_ns=state.stamp_ns; m.dt_s=.025;
    m.delta_rotation=previous.rotation.transpose()*state.rotation;
    m.linearization_gyro_bias=previous.gyro_bias; m.linearization_accel_bias=previous.accel_bias;
    m.jacobian_rotation_gyro_bias=-.025*Eigen::Matrix3d::Identity();
    m.jacobian_velocity_accel_bias=-.025*Eigen::Matrix3d::Identity();
    m.jacobian_position_accel_bias=-.0003125*Eigen::Matrix3d::Identity();
    m.covariance=.001*Matrix15d::Identity(); m.valid=true;
    check(window->addStateWithImuFactorAtomic(state,3*index-2,previous.stamp_ns,m,&reason),reason);
    VisualRelativeMeasurement v;
    v.observation_id=3*index-1; v.reference_stamp_ns=previous.stamp_ns;
    v.current_stamp_ns=state.stamp_ns;
    v.reference_imu_translation=previous.rotation.transpose()*(state.position-previous.position)+Eigen::Vector3d(.001,-.002,.003);
    v.covariance=.01*Eigen::Matrix3d::Identity(); v.selected_rank=2;
    v.mode=VisualFactorMode::LIDAR_WEAK_TRANSLATION; v.valid=true;
    check(window->addVisualFactor(v,&reason),reason);
  }
  LidarWindowMeasurement lidar;
  lidar.observation_id=index ? 3*index : 1000000; // initial lidar not added: IDs are globally monotonic.
  lidar.stamp_ns=state.stamp_ns; lidar.measured_position=state.position+Eigen::Vector3d(.002,-.003,.001);
  lidar.measured_rotation=state.rotation*Eigen::AngleAxisd(.003,Eigen::Vector3d::UnitY()).toRotationMatrix();
  lidar.covariance=.02*dog_prior_map_fastlio2_frontend_exp::Matrix6d::Identity(); lidar.valid=true;
  if(index) check(window->addLidarFactor(lidar,&reason),reason);
}
inline FixedLagWindow make(int nodes,WindowSolverBackend backend) {
  FixedLagOptions options; options.maximum_duration_s=100; options.maximum_nodes=48;
  options.solver_backend=backend; options.maximum_optimizer_iterations=4;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow window(options,noise);
  for(int i=0;i<nodes;++i) append(&window,i);
  return window;
}
}
