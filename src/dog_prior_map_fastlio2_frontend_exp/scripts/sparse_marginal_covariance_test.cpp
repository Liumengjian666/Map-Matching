#include "p6_a2d_synthetic_window.hpp"
#include <chrono>
#include <iostream>
using namespace a2d_test;
using dog_prior_map_fastlio2_frontend_exp::Matrix6d;
void compare(FixedLagWindow& window, const char* label) {
  WindowMarginalCovariance sparse, dense;
  std::string reason;
  const auto before=window.summary();
  check(window.latestMarginalCovariance(&sparse,&reason),reason);
  check(window.summary().dense_marginal_reference_requests==
      before.dense_marginal_reference_requests,"production invoked dense oracle");
  check(window.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason),reason);
  const double p15=(sparse.covariance15-dense.covariance15).norm()/dense.covariance15.norm();
  const double p6=(sparse.map_pose_covariance6-dense.map_pose_covariance6).norm()/dense.map_pose_covariance6.norm();
  check(p15<=1e-9 && p6<=1e-9,"sparse/dense covariance mismatch");
  check(sparse.normalized_backward_error<=1e-10,"backward error");
  std::cout<<label<<",p15="<<p15<<",p6="<<p6<<",backward="
      <<sparse.normalized_backward_error<<'\n';
}
int main() { try {
  auto joint=make(1,WindowSolverBackend::BLOCK_SPARSE);
  compare(joint,"initial_nonzero_prior_gradient");
  auto imu_only=make(1,WindowSolverBackend::BLOCK_SPARSE);
  WindowState next=*imu_only.latestState(); next.stamp_ns+=25000000;
  ImuPreintegratedMeasurement m; m.start_stamp_ns=next.stamp_ns-25000000;
  m.end_stamp_ns=next.stamp_ns; m.dt_s=.025; m.valid=true;
  m.covariance=.001*Matrix15d::Identity();
  m.linearization_gyro_bias=next.gyro_bias; m.linearization_accel_bias=next.accel_bias;
  check(imu_only.addStateWithImuFactorAtomic(next,1,m.start_stamp_ns,m),"imu-only admission");
  compare(imu_only,"imu_only");
  LidarWindowMeasurement directional; directional.observation_id=2;
  directional.stamp_ns=next.stamp_ns; directional.valid=true; directional.reliable_rank=2;
  directional.basis_relinearizer=[](const WindowState&,Matrix6d* basis,int* rank,std::string*) {
    *basis=Matrix6d::Identity(); *rank=2; return true;
  };
  check(imu_only.addLidarFactor(directional),"directional lidar admission");
  compare(imu_only,"directional_lidar");
  for(int i=1;i<6;++i) append(&joint,i);
  compare(joint,"joint_imu_visual_cross_state_lidar");
  FixedLagOptions options; options.maximum_nodes=3; options.maximum_duration_s=100;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow repeated(options,noise);
  for(int i=0;i<12;++i) {
    append(&repeated,i);
    std::string reason;
    check(repeated.marginalizeIfNeeded(&reason),reason);
    compare(repeated,i<3?"before_schur":"repeated_schur");
  }
  FixedLagWindow singular;
  WindowState state; state.stamp_ns=1;
  check(singular.addState(state),"singular add");
  WindowMarginalCovariance unavailable; std::string reason;
  check(joint.latestMarginalCovariance(&unavailable,&reason),"valid output before failure");
  check(!singular.latestMarginalCovariance(&unavailable,&reason) &&
      !unavailable.valid && unavailable.status=="WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE",
      "singular must not be regularized");
  WindowLinearSystem deficient; deficient.gradient=Vector15d::Zero();
  Matrix15d h=Matrix15d::Identity(); h.topLeftCorner<2,2>().setOnes();
  deficient.add(0,0,h);
  Eigen::MatrixXd columns; double backward=0;
  check(!solveLatestMarginalColumnsSparse(deficient,&columns,&backward,&reason),
      "positive-diagonal rank-deficient H accepted");
  h(1,1)+=1e-13; deficient.upper_blocks.clear(); deficient.add(0,0,h);
  check(!solveLatestMarginalColumnsSparse(deficient,&columns,&backward,&reason) &&
      reason=="NUMERICALLY_SINGULAR_HESSIAN","tiny balanced pivot accepted");
  std::cout<<"nodes,sparse_marginal_ms,dense_marginal_ms,p15_relative_error\n";
  for(int n:{8,16,24,32,48}) {
    auto window=make(n,WindowSolverBackend::BLOCK_SPARSE);
    WindowMarginalCovariance sparse,dense;
    const auto a=std::chrono::steady_clock::now();
    check(window.latestMarginalCovariance(&sparse,&reason),reason);
    const auto b=std::chrono::steady_clock::now();
    check(window.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason),reason);
    const auto c=std::chrono::steady_clock::now();
    const double error=(sparse.covariance15-dense.covariance15).norm()/dense.covariance15.norm();
    check(error<=1e-9,"scaling parity");
    std::cout<<n<<','<<std::chrono::duration<double,std::milli>(b-a).count()<<','
        <<std::chrono::duration<double,std::milli>(c-b).count()<<','<<error<<'\n';
  }
  std::cout<<"SPARSE_MARGINAL_COVARIANCE_PASS\n";
  return 0;
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
