#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_production.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <iostream>
#include <memory>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using dog_prior_map_fastlio2_frontend_exp::Matrix6d;
void require(bool yes,const char* text) { if (!yes) throw std::runtime_error(text); }
double relative(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
  return (a-b).norm()/std::max(b.norm(),1e-30);
}
void rowTests() {
  std::string reason; WindowMarginalCovariance result;
  const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.27,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  Eigen::Matrix<double,6,15> map=Eigen::Matrix<double,6,15>::Zero();
  map.block<3,3>(0,0)=rotation; map.block<3,3>(3,3).setIdentity();
  Eigen::MatrixXd a=Eigen::MatrixXd::Random(60,30);
  a.topRows(30)+=3*Eigen::MatrixXd::Identity(30,30);
  // A deliberate nontrivial column permutation; PSD alone cannot detect a
  // wrong latest-state selector permutation.
  a.col(0)*=0.2; a.col(29)*=10;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
  require(qr.colsPermutation().indices()(0)!=0,"fixture must force permutation");
  const Eigen::MatrixXd h=a.transpose()*a;
  const Matrix15d expected=h.llt().solve(Eigen::MatrixXd::Identity(30,30)).bottomRightCorner(15,15);
  require(solveSquareRootMarginalCovariance(a,rotation,&result,&reason),"permuted row covariance");
  require(relative(result.covariance15,expected)<1e-12,"permuted selector P15");
  require(relative(result.map_pose_covariance6,map*expected*map.transpose())<1e-12,"map row covariance");
  require(result.normalized_backward_error<1e-12,"triangular residual");
  std::cout<<"permutation_P15_relative="<<relative(result.covariance15,expected)<<'\n';

  const Eigen::MatrixXd c=Eigen::MatrixXd::Random(35,35);
  const Eigen::MatrixXd covariance=c*c.transpose()+2*Eigen::MatrixXd::Identity(35,35);
  const Eigen::MatrixXd jacobian=Eigen::MatrixXd::Random(35,30);
  SquareRootRows whitened;
  require(whitenSquareRootRows(jacobian,Eigen::VectorXd::Random(35),covariance,&whitened,&reason),"correlated covariance");
  require(solveSquareRootMarginalCovariance(whitened.a,rotation,&result,&reason),"correlated whitened covariance");
  const Eigen::MatrixXd direct_h=jacobian.transpose()*covariance.llt().solve(jacobian);
  const Matrix15d direct=direct_h.llt().solve(Eigen::MatrixXd::Identity(30,30)).bottomRightCorner(15,15);
  require(relative(result.covariance15,direct)<1e-11,"L inverse whitening orientation");
  std::cout<<"correlated_P15_relative="<<relative(result.covariance15,direct)<<'\n';

  // Known covariance: x0 prior precision=1, relative x1-x0 precision=1e18,
  // weak x1 precision=1e-4. Exact P11=1/(1e-4+1/(1+1e-18)).
  a=Eigen::MatrixXd::Zero(45,30);
  a.topLeftCorner(15,15).setIdentity();
  a.block(15,0,15,15)=-1e9*Matrix15d::Identity();
  a.block(15,15,15,15)=1e9*Matrix15d::Identity();
  a.bottomRightCorner(15,15)=0.01*Matrix15d::Identity();
  require(solveSquareRootMarginalCovariance(a,rotation,&result,&reason),"extreme scale QR");
  const Matrix15d known=Matrix15d::Identity()/(1e-4+1/(1+1e-18));
  require(relative(result.covariance15,known)<1e-6,"extreme scale known covariance");
  const Eigen::MatrixXd normal=a.transpose()*a;
  Eigen::LLT<Eigen::MatrixXd> legacy(normal);
  std::cout<<"extreme_P15_known_relative="<<relative(result.covariance15,known)
      <<" legacy_normal_LLT="<<(legacy.info()==Eigen::Success)<<'\n';
  a.col(29).setZero();
  require(!solveSquareRootMarginalCovariance(a,rotation,&result,&reason)&&
      result.detail=="SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT","rank deficient fail closed");
}

WindowState stateAt(unsigned i) {
  WindowState x; x.stamp_ns=1000000000ULL+i*100000000ULL;
  x.rotation=Eigen::AngleAxisd(0.1+0.001*i,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  x.position=Eigen::Vector3d(0.01*i,0.02,-0.03); x.velocity=Eigen::Vector3d(0.1,0,0);
  return x;
}
ImuPreintegratedMeasurement imuBetween(const WindowState& from,const WindowState& to) {
  ImuPreintegratedMeasurement m; m.start_stamp_ns=from.stamp_ns; m.end_stamp_ns=to.stamp_ns;
  m.dt_s=(to.stamp_ns-from.stamp_ns)*1e-9; m.delta_rotation=from.rotation.transpose()*to.rotation;
  m.delta_position=from.rotation.transpose()*(to.position-from.position-from.velocity*m.dt_s);
  m.delta_velocity=from.rotation.transpose()*(to.velocity-from.velocity);
  Matrix15d mix=Matrix15d::Identity(); mix(1,4)=0.1; mix(6,10)=0.2;
  m.covariance=0.04*mix*mix.transpose(); m.valid=true; return m;
}
FixedLagWindow graph(bool shadow,std::shared_ptr<int> calls) {
  FixedLagOptions options; options.maximum_nodes=3; options.maximum_duration_s=100;
  options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  options.marginal_covariance_backend=MarginalCovarianceBackend::SQUARE_ROOT_QR;
  options.capture_covariance_shadow=shadow;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow window(options,noise); std::string reason;
  require(window.initializeWithPriorAtomic(stateAt(0),3*Matrix15d::Identity(),
      Vector15d::Constant(0.01),&reason),"initial prior");
  // Query the initial window, before any raw factor, then IMU-only.
  WindowMarginalCovariance root,dense;
  require(window.latestMarginalCovariance(&root,&reason)&&
      window.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason)&&
      relative(root.covariance15,dense.covariance15)<1e-12,"initial covariance equivalence");
  for (unsigned i=1;i<=2;++i)
    require(window.addStateWithImuFactorAtomic(stateAt(i),i,stateAt(i-1).stamp_ns,
        imuBetween(stateAt(i-1),stateAt(i)),&reason),"IMU graph");
  require(window.latestMarginalCovariance(&root,&reason)&&
      window.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason)&&
      relative(root.covariance15,dense.covariance15)<1e-10,"IMU-only covariance equivalence");
  LidarWindowMeasurement lidar; lidar.observation_id=3; lidar.stamp_ns=stateAt(2).stamp_ns;
  lidar.measured_rotation=stateAt(2).rotation; lidar.measured_position=stateAt(2).position;
  lidar.covariance=0.03*Matrix6d::Identity(); lidar.reliable_rank=5; lidar.valid=true;
  lidar.basis_relinearizer=[calls](const WindowState& state,Matrix6d* basis,int* rank,std::string*) {
    ++*calls; *basis=Matrix6d::Identity(); basis->block<3,3>(0,0)=state.rotation;
    *rank=5; return true;
  };
  require(window.addLidarFactor(lidar,&reason),"rank5 factor");
  Eigen::MatrixXd j,cov; Eigen::VectorXd r;
  require(linearizeLidarFactor(stateAt(2),lidar,&r,&j,&cov,&reason)&&j.rows()==5,"LiDAR still rank5");
  VisualRelativeMeasurement visual; visual.observation_id=4;
  visual.reference_stamp_ns=stateAt(0).stamp_ns; visual.current_stamp_ns=stateAt(2).stamp_ns;
  visual.reference_imu_translation=stateAt(0).rotation.transpose()*(stateAt(2).position-stateAt(0).position);
  visual.covariance=0.02*Eigen::Matrix3d::Identity(); visual.selected_rank=2; visual.valid=true;
  require(window.addVisualFactor(visual,&reason),"directional cross-state visual");
  return window;
}
void windowTests() {
  std::string reason;
  auto off_calls=std::make_shared<int>(0),on_calls=std::make_shared<int>(0);
  auto off=graph(false,off_calls),on=graph(true,on_calls);
  LidarIterationSnapshot invalid;
  require(off.buildLidarIterationSnapshot(&invalid,&reason),"snapshot fixture");
  invalid.projections.front().reliable_rank=-1;
  SquareRootRows invalid_rows;
  require(!off.allFactorsSquareRootRowsWithLidarSnapshot(invalid,&invalid_rows,&reason)&&
      reason=="square_root_snapshot_invalid_or_duplicate","invalid snapshot rejected before row allocation");
  for (unsigned iteration=0;iteration<10;++iteration) {
    const auto states=off.states(); const auto summary=off.summary();
    const auto h=off.priorInformation(),g=Eigen::MatrixXd(off.priorGradient());
    const auto prior=off.priorSquareRootRows();
    const int n_off=*off_calls,n_on=*on_calls;
    WindowMarginalCovariance a,b,dense;
    require(off.latestMarginalCovariance(&a,&reason)&&on.latestMarginalCovariance(&b,&reason),"joint QR covariance");
    require(*off_calls-n_off==1&&*on_calls-n_on==1,"one callback per request including shadow");
    require((a.covariance15-b.covariance15).norm()==0&&
        (a.map_pose_covariance6-b.map_pose_covariance6).norm()==0,"diagnostics covariance parity exact");
    LidarWindowMeasurement next_lidar;
    next_lidar.observation_id=99; next_lidar.stamp_ns=off.latestState()->stamp_ns;
    next_lidar.measured_position=off.latestState()->position+Eigen::Vector3d(0.1,0,0);
    next_lidar.measured_rotation=off.latestState()->rotation;
    next_lidar.covariance=0.03*Matrix6d::Identity(); next_lidar.valid=true;
    const auto na=evaluateSelectedLidarNis(*off.latestState(),next_lidar,a,15.086);
    const auto nb=evaluateSelectedLidarNis(*on.latestState(),next_lidar,b,15.086);
    require(na.valid&&nb.valid&&na.nis==nb.nis&&na.accepted==nb.accepted,"diagnostics NIS parity");
    dog_prior_map_fastlio2_frontend_exp::reliability::DualReliabilityConfig config;
    const dog_prior_map_fastlio2_frontend_exp::Vector6d innovation=
        dog_prior_map_fastlio2_frontend_exp::Vector6d::Constant(0.1);
    require(dog_prior_map_fastlio2_frontend_exp::reliability::shouldRunNonlocalProbes(
        52,innovation,a.map_pose_covariance6,config).run_probes==
        dog_prior_map_fastlio2_frontend_exp::reliability::shouldRunNonlocalProbes(
        52,innovation,b.map_pose_covariance6,config).run_probes,"diagnostics probe parity");
    require(b.legacy_shadow_requested&&b.legacy_shadow_valid&&
        relative(a.covariance15,b.legacy_covariance15)<1e-10,"shared snapshot shadow equivalence");
    require(off.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason)&&
        relative(a.covariance15,dense.covariance15)<1e-9&&
        relative(a.map_pose_covariance6,dense.map_pose_covariance6)<1e-9,"joint/repeated Schur dense equivalence");
    const auto after=off.summary();
    require(after.window_revision==summary.window_revision&&after.optimized_revision==summary.optimized_revision&&
        after.imu_factor_count==summary.imu_factor_count&&after.lidar_factor_count==summary.lidar_factor_count&&
        after.visual_factor_count==summary.visual_factor_count&&
        after.active_observation_id_count==summary.active_observation_id_count&&
        after.retired_observation_id_watermark==summary.retired_observation_id_watermark,"query lifecycle read-only");
    require((off.priorInformation()-h).norm()==0&&(off.priorGradient()-g).norm()==0&&
        (off.priorSquareRootRows().a-prior.a).norm()==0&&(off.priorSquareRootRows().b-prior.b).norm()==0,"query prior read-only");
    for (std::size_t k=0;k<states.size();++k)
      require(localDifference(states[k],off.states()[k]).norm()==0&&states[k].stamp_ns==off.states()[k].stamp_ns,"query states unchanged");
    require(off.optimize(&reason)&&on.optimize(&reason),"nonzero chart optimization");
    if (iteration==0) {
      require(localDifference(off.states().front(),states.front()).head<3>().norm()>1e-5,
          "current prior rotation chart must be nonzero");
      require(off.latestMarginalCovariance(&a,&reason)&&
          off.latestMarginalCovarianceDenseReferenceForTest(&dense,&reason)&&
          relative(a.covariance15,dense.covariance15)<1e-9&&
          relative(a.map_pose_covariance6,dense.map_pose_covariance6)<1e-9,
          "nonzero prior rotation/gradient chart covariance before marginal reanchoring");
    }
    const unsigned next=iteration+3;
    for (auto* w : {&off,&on}) {
      auto from=*w->latestState(),to=stateAt(next);
      require(w->addStateWithImuFactorAtomic(to,5+iteration,from.stamp_ns,imuBetween(from,to),&reason)&&
          w->marginalizeIfNeeded(&reason),"repeated QR marginalization");
    }
    // Once the original unary factor has been retired no callback remains.
    if (iteration==2) break;
  }
  // Deficient query cannot mutate an estimator or silently use another backend.
  FixedLagOptions options; options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  options.marginal_covariance_backend=MarginalCovarianceBackend::SQUARE_ROOT_QR;
  FixedLagWindow bad(options); WindowMarginalCovariance p;
  require(bad.initializeWithPriorAtomic(stateAt(0),Matrix15d::Identity(),Vector15d::Zero(),&reason)&&
      bad.addState(stateAt(1),&reason),"unconstrained latest node");
  const auto before=bad.summary();
  require(!bad.latestMarginalCovariance(&p,&reason)&&p.detail=="SQUARE_ROOT_COVARIANCE_RANK_DEFICIENT"&&
      !p.legacy_shadow_requested,"rank failure without fallback");
  require(bad.summary().window_revision==before.window_revision&&bad.states().size()==2,"rank failure read-only");
  std::cout<<"joint_repeated_Schur_readonly_shadow_parity_PASS\n";
}
int main() {
  try { rowTests(); windowTests(); std::cout<<"A3F_R1_COVARIANCE_TEST_PASS\n"; return 0; }
  catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
