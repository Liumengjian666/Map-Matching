#include "dog_prior_map_fastlio2_frontend_exp/window_square_root.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <limits>
#include <memory>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag;
using dog_prior_map_fastlio2_frontend_exp::Matrix6d;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void rowTests() {
  std::string reason;
  SquareRootRows rows;
  const Eigen::MatrixXd c = Eigen::MatrixXd::Random(7,7);
  const Eigen::MatrixXd covariance = (c*c.transpose()+Eigen::MatrixXd::Identity(7,7)).eval();
  const Eigen::MatrixXd j = Eigen::MatrixXd::Random(7,20);
  const Eigen::VectorXd r = Eigen::VectorXd::Random(7);
  require(whitenSquareRootRows(j,r,covariance,&rows,&reason), "correlated whitening");
  require((rows.a.transpose()*rows.a-j.transpose()*covariance.llt().solve(j)).norm()<1e-12,
          "whitening H orientation");
  require((rows.a.transpose()*rows.b-j.transpose()*covariance.llt().solve(r)).norm()<1e-12,
          "whitening g orientation");
  Matrix15d q = Matrix15d::Random();
  Matrix15d h = (q*q.transpose()+Matrix15d::Identity()).eval();
  Vector15d g = Vector15d::Random();
  require(factorInitialSquareRootPrior(h,g,&rows,&reason),"correlated initial prior");
  require((rows.a.transpose()*rows.a-h).norm()<1e-12 &&
          (rows.a.transpose()*rows.b-g).norm()<1e-12,"correlated initial H/g");
  const Vector15d d = Vector15d::Random();
  require(std::abs((rows.a*d+rows.b).squaredNorm()-d.dot(h*d)-2*g.dot(d)-rows.b.squaredNorm())<1e-12,
          "prior additive constant only");
  Eigen::Matrix2d bad;
  bad << 1e-12,5e-9,0,1e-12;
  require(!whitenSquareRootRows(Eigen::Matrix2d::Identity(),Eigen::Vector2d::Zero(),bad,&rows,&reason),
          "tiny scale asymmetric covariance rejected");

  // Exact oracle: huge eliminated component plus one retained unit row.
  // Normal-equation subtraction loses the unit; QR works on the rows directly.
  SquareRootRows stack;
  stack.a=Eigen::MatrixXd::Zero(17,16); stack.b=Eigen::VectorXd::Zero(17);
  stack.a.topLeftCorner(15,15).setIdentity();
  stack.a(0,15)=1e8; stack.a(15,15)=1; stack.b(15)=0.3; stack.b(16)=0.7;
  SquareRootQrDiagnostics qr;
  require(eliminateSquareRootOldest(stack,15,&rows,&qr,&reason),"extreme scale rows");
  require(std::abs(rows.a.squaredNorm()-1)<1e-14 &&
          std::abs((rows.a.transpose()*rows.b)(0)-0.3)<1e-14,"extreme scale known H/g preserved");
  require(qr.rows_after_compression==1 && std::abs(qr.discarded_constant_squared-0.49)<1e-14,
          "row compression drops only constant row");
  const Eigen::MatrixXd normal=stack.a.transpose()*stack.a;
  const double legacy=normal(15,15)-normal(0,15)*normal(0,15);
  std::cout << "extreme_scale_legacy_H=" << legacy << " QR_H=" << rows.a.squaredNorm() << '\n';
  const double threshold=std::numeric_limits<double>::epsilon()*17;
  stack.a(14,14)=2*threshold;
  require(eliminateSquareRootOldest(stack,15,&rows,&qr,&reason),"rank above standard threshold");
  stack.a(14,14)=0.5*threshold;
  require(!eliminateSquareRootOldest(stack,15,&rows,&qr,&reason) &&
          reason=="square_root_marginalized_block_rank_deficient","rank below standard threshold");
  stack.a(14,14)=1; stack.a.col(15).setZero();
  require(eliminateSquareRootOldest(stack,15,&rows,&qr,&reason) && rows.a.rows()==0,
          "zero retained row-space compressed without invented information");
  stack.a=Eigen::MatrixXd::Zero(17,17); stack.b=Eigen::VectorXd::Zero(17);
  stack.a.topLeftCorner(15,15).setIdentity(); stack.a(15,15)=1;
  stack.a(16,16)=8*std::numeric_limits<double>::epsilon();
  require(eliminateSquareRootOldest(stack,15,&rows,&qr,&reason)&&qr.compression_rank==2,
          "retained compression positive mode above standard threshold");
  stack.a(16,16)=0.5*std::numeric_limits<double>::epsilon();
  require(eliminateSquareRootOldest(stack,15,&rows,&qr,&reason)&&qr.compression_rank==1,
          "retained compression numerical mode below standard threshold");
}

WindowState stateAt(unsigned index) {
  WindowState x;
  x.stamp_ns=1000000000ULL+index*100000000ULL;
  x.rotation=Eigen::AngleAxisd(0.1+0.001*index,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
  x.position=Eigen::Vector3d(0.01*index,0.02,-0.03);
  x.velocity=Eigen::Vector3d(0.1,0,0);
  return x;
}

ImuPreintegratedMeasurement imuBetween(const WindowState& from,const WindowState& to) {
  ImuPreintegratedMeasurement m;
  m.start_stamp_ns=from.stamp_ns; m.end_stamp_ns=to.stamp_ns;
  m.dt_s=(to.stamp_ns-from.stamp_ns)*1e-9;
  m.delta_rotation=from.rotation.transpose()*to.rotation;
  m.delta_position=from.rotation.transpose()*(to.position-from.position-from.velocity*m.dt_s);
  m.delta_velocity=from.rotation.transpose()*(to.velocity-from.velocity);
  m.covariance=0.04*Matrix15d::Identity(); m.valid=true;
  return m;
}

LidarWindowMeasurement rankFive(const WindowState& x,std::uint64_t id) {
  LidarWindowMeasurement m;
  m.observation_id=id; m.stamp_ns=x.stamp_ns; m.measured_position=x.position;
  m.measured_rotation=x.rotation; m.covariance=0.03*Matrix6d::Identity();
  m.reliable_rank=5; m.valid=true;
  m.basis_relinearizer=[](const WindowState&,Matrix6d* b,int* rank,std::string*) {
    *b=Matrix6d::Identity(); *rank=5; return true;
  };
  return m;
}

FixedLagWindow graph(bool capture,std::shared_ptr<int> calls=nullptr,bool health=false) {
  FixedLagOptions options;
  options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  options.maximum_nodes=2; options.maximum_duration_s=100;
  options.capture_marginalization_diagnostics=capture;
  options.capture_marginalization_health=health;
  options.capture_optimizer_trace=health;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow window(options,noise); std::string reason;
  auto x0=stateAt(0),x1=stateAt(1),x2=stateAt(2);
  Vector15d g=Vector15d::Constant(0.01);
  require(window.initializeWithPriorAtomic(x0,3*Matrix15d::Identity(),g,&reason),"graph initial");
  require(window.addStateWithImuFactorAtomic(x1,1,x0.stamp_ns,imuBetween(x0,x1),&reason),"graph imu01");
  require(window.addStateWithImuFactorAtomic(x2,2,x1.stamp_ns,imuBetween(x1,x2),&reason),"graph imu12");
  auto lidar=rankFive(x0,3); lidar.measured_position.x()+=0.13;
  if (calls) lidar.basis_relinearizer=[calls](const WindowState&,Matrix6d* basis,int* rank,std::string*) {
    ++*calls; *basis=Matrix6d::Identity(); *rank=5; return true;
  };
  require(window.addLidarFactor(lidar,&reason),"touching rank5 lidar");
  lidar=rankFive(x2,4); lidar.measured_position.y()-=0.12;
  require(window.addLidarFactor(lidar,&reason),"retained rank5 lidar");
  VisualRelativeMeasurement visual;
  visual.observation_id=5; visual.reference_stamp_ns=x0.stamp_ns; visual.current_stamp_ns=x1.stamp_ns;
  visual.reference_imu_translation=x0.rotation.transpose()*(x1.position-x0.position)+Eigen::Vector3d(0.01,-0.02,0.03);
  visual.covariance=0.02*Eigen::Matrix3d::Identity(); visual.valid=true;
  require(window.addVisualFactor(visual,&reason),"touching cross-state visual");
  visual.observation_id=6; visual.reference_stamp_ns=x1.stamp_ns; visual.current_stamp_ns=x2.stamp_ns;
  require(window.addVisualFactor(visual,&reason),"retained cross-state visual");
  return window;
}

void windowTests() {
  std::string reason;
  auto off_calls=std::make_shared<int>(0),on_calls=std::make_shared<int>(0);
  auto off=graph(false,off_calls),on=graph(true,on_calls);
  Eigen::MatrixXd full_h; Eigen::VectorXd full_g; double cost;
  require(off.linearizedSystem(&full_h,&full_g,&cost,&reason),"joint graph assembly");
  Eigen::MatrixXd mr=full_h.topRightCorner(15,30);
  Eigen::LDLT<Eigen::MatrixXd> solve(full_h.topLeftCorner(15,15));
  Eigen::MatrixXd expected=full_h.bottomRightCorner(30,30)-mr.transpose()*solve.solve(mr);
  Eigen::VectorXd expected_g=full_g.tail(30)-mr.transpose()*solve.solve(full_g.head(15));
  const int before_off=*off_calls,before_on=*on_calls;
  require(off.marginalizeIfNeeded(&reason)&&on.marginalizeIfNeeded(&reason),"joint graph QR marginalization");
  require(*off_calls-before_off==1&&*on_calls-before_on==1,"shadow does not reinvoke shared callback");
  Eigen::MatrixXd h; Eigen::VectorXd g;
  require(off.linearizedSystem(&h,&g,&cost,&reason),"retained graph assembly");
  const double he=(h-expected).norm()/expected.norm(),ge=(g-expected_g).norm()/expected_g.norm();
  require(he<1e-10&&ge<1e-10,"QR prior plus retained-only raw factors conserves H/g");
  require(off.summary().imu_factor_count==1&&off.summary().lidar_factor_count==1&&off.summary().visual_factor_count==1,
          "retained factors remain active exactly once");
  require((off.priorInformation()-on.priorInformation()).norm()==0&&
          (off.priorGradient()-on.priorGradient()).norm()==0&&off.states().size()==on.states().size(),"diagnostics parity prior");
  require(off.optimize(&reason)&&on.optimize(&reason),"diagnostics parity optimize");
  for (std::size_t i=0;i<off.states().size();++i)
    require(localDifference(off.states()[i],on.states()[i]).norm()==0,"diagnostics parity optimized state");
  std::cout << "joint_information_H_relative=" << he << " g_relative=" << ge << '\n';

  FixedLagOptions options; options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  FixedLagWindow chart(options);
  Vector15d nonzero=Vector15d::Constant(0.05);
  require(chart.initializeWithPriorAtomic(stateAt(0),Matrix15d::Identity(),nonzero,&reason)&&chart.optimize(&reason),"nonzero prior chart");
  SquareRootRows chart_rows;
  require(chart.linearizeSquareRootPriorRows(&chart_rows,&reason)&&chart.linearizedSystem(&h,&g,&cost,&reason),"chart comparison");
  require((chart_rows.a.transpose()*chart_rows.a-h).norm()<1e-10&&
          (chart_rows.a.transpose()*chart_rows.b-g).norm()<1e-10,"nonzero chart C H C and C(Hd+g)");
  require(std::abs(chart_rows.b.squaredNorm()-cost-chart.priorSquareRootRows().b.squaredNorm())<1e-12,"constant objective offset");
  WindowState identity=stateAt(0); identity.rotation.setIdentity();
  Eigen::VectorXd residual; Eigen::MatrixXd jacobian,covariance;
  auto lidar=rankFive(identity,10);
  require(linearizeLidarFactor(identity,lidar,&residual,&jacobian,&covariance,&reason)&&jacobian.rows()==5&&jacobian.col(2).norm()==0,
          "rank5 weak rotation direction not given LiDAR information");
  options.maximum_nodes=1;
  FixedLagWindow deficient(options);
  require(deficient.addState(identity,&reason)&&deficient.addState(stateAt(1),&reason)&&
          deficient.addLidarFactor(lidar,&reason),"deficient window fixture");
  const auto before=deficient.summary();
  require(!deficient.marginalizeIfNeeded(&reason)&&reason=="square_root_marginalized_block_rank_deficient",
          "window rank deficiency fails closed");
  require(deficient.states().size()==2&&deficient.states().front().stamp_ns==identity.stamp_ns&&
          deficient.summary().active_observation_id_count==before.active_observation_id_count&&
          deficient.summary().lidar_factor_count==before.lidar_factor_count&&
          deficient.priorInformation().size()==0&&deficient.priorSquareRootRows().a.size()==0,
          "rank failure leaves states/prior/factors/IDs untouched");
}

void lifecycleTest() {
  FixedLagOptions options; options.maximum_nodes=2; options.maximum_duration_s=100;
  options.marginalization_backend=MarginalizationBackend::SQUARE_ROOT_QR;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow window(options,noise); std::string reason;
  require(window.initializeWithPriorAtomic(stateAt(0),Matrix15d::Identity(),Vector15d::Zero(),&reason),"lifecycle init");
  std::size_t max_rows=0,max_ids=0;
  for (unsigned i=1;i<=1002;++i) {
    auto from=stateAt(i-1),to=stateAt(i);
    require(window.addStateWithImuFactorAtomic(to,2*i-1,from.stamp_ns,imuBetween(from,to),&reason),"lifecycle insert");
    require(window.addLidarFactor(rankFive(to,2*i),&reason),"lifecycle rank5");
    require(window.marginalizeIfNeeded(&reason),"lifecycle marginalization");
    const auto& root=window.priorSquareRootRows();
    require(root.a.rows()<=root.a.cols()&&root.a.cols()==15*static_cast<Eigen::Index>(window.states().size()),"bounded prior rows/columns");
    require((window.priorInformation()-root.a.transpose()*root.a).norm()<1e-10&&
            (window.priorGradient()-root.a.transpose()*root.b).norm()<1e-10,"authoritative cache equality");
    max_rows=std::max(max_rows,static_cast<std::size_t>(root.a.rows()));
    max_ids=std::max(max_ids,window.summary().active_observation_id_count);
  }
  require(max_ids<=4,"active observation memory bounded");
  require(!window.addLidarFactor(rankFive(window.states().front(),2),&reason),"historical observation cannot reenter");
  // Same enforcement legitimately removes several states.
  for(unsigned i=1003;i<=1006;++i) {
    auto from=stateAt(i-1),to=stateAt(i);
    require(window.addStateWithImuFactorAtomic(to,2*i-1,from.stamp_ns,imuBetween(from,to),&reason),"multi-removal insert");
  }
  require(window.marginalizeIfNeeded(&reason)&&window.states().size()==2,"multi-removal lifecycle");
  std::cout << "lifecycle_eliminations=1005 max_rows=" << max_rows << " max_active_ids=" << max_ids << '\n';
}

int main() {
  try {
    std::string reason;
    SquareRootRows initial;
    Matrix15d h = 3 * Matrix15d::Identity();
    Vector15d g = Vector15d::Constant(0.25);
    require(factorInitialSquareRootPrior(h, g, &initial, &reason), "initial LLT");
    require((initial.a.transpose() * initial.a - h).norm() < 1e-13, "initial H equality");
    require((initial.a.transpose() * initial.b - g).norm() < 1e-13, "initial g equality");
    require(!factorInitialSquareRootPrior(Matrix15d::Zero(), g, &initial, &reason) &&
        reason == "square_root_initial_prior_factorization_failed", "initial fail closed");
    SquareRootRows stacked;
    stacked.a = Eigen::MatrixXd::Random(65, 45);
    stacked.b = Eigen::VectorXd::Random(65);
    SquareRootRows result;
    SquareRootQrDiagnostics diagnostic;
    require(eliminateSquareRootOldest(stacked, 15, &result, &diagnostic, &reason), "QR elimination");
    const Eigen::MatrixXd full_h = stacked.a.transpose() * stacked.a;
    const Eigen::VectorXd full_g = stacked.a.transpose() * stacked.b;
    const Eigen::MatrixXd mm = full_h.topLeftCorner(15,15), mr = full_h.topRightCorner(15,30);
    const Eigen::MatrixXd expected_h = full_h.bottomRightCorner(30,30) - mr.transpose() * mm.ldlt().solve(mr);
    const Eigen::VectorXd expected_g = full_g.tail(30) - mr.transpose() * mm.ldlt().solve(full_g.head(15));
    std::cout << "linear_H_error=" << (result.a.transpose()*result.a-expected_h).norm()
              << " linear_g_error=" << (result.a.transpose()*result.b-expected_g).norm() << '\n';
    require((result.a.transpose()*result.a-expected_h).norm() < 1e-11, "linear H conservation");
    require((result.a.transpose()*result.b-expected_g).norm() < 1e-11, "linear g conservation");
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> eliminated(stacked.a.leftCols(15));
    double constant=0;
    for(int trial=0;trial<3;++trial) {
      Eigen::VectorXd retained=Eigen::VectorXd::Random(30);
      Eigen::VectorXd oldest=eliminated.solve(-stacked.a.rightCols(30)*retained-stacked.b);
      const double before=(stacked.a.leftCols(15)*oldest+stacked.a.rightCols(30)*retained+stacked.b).squaredNorm();
      const double after=(result.a*retained+result.b).squaredNorm();
      if(trial==0) constant=before-after;
      require(std::abs(before-after-constant)<1e-10,"minimized stacked objective equals retained objective plus constant");
    }
    stacked.a.col(0).setZero();
    require(!eliminateSquareRootOldest(stacked, 15, &result, &diagnostic, &reason) &&
        reason == "square_root_marginalized_block_rank_deficient", "rank fail closed");
    rowTests();
    windowTests();
    lifecycleTest();
    std::cout << "A3E_R2_SQUARE_ROOT_ROWS_TEST_PASS\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
