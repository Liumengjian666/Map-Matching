#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace p=dog_prior_map_fastlio2_frontend_exp;
void require(bool ok,const char* why) {if(!ok) throw std::runtime_error(why);}
int main() {try {
  Eigen::Matrix4f nominal=Eigen::Matrix4f::Identity(),prediction=nominal;
  prediction(0,3)=-.2f;
  p::CoupledAnchorState anchor;anchor.valid=true;anchor.origin_stamp_ns=1000000000;
  anchor.propagated_stamp_ns=1100000000;anchor.prediction(0,3)=.12;
  p::CoupledNdtBackend backend;int jets=0,values=0,aligns=0;
  auto energy=[](const p::CoupledVector6& x) {
    p::CoupledVector6 d;d<<.1,.12,3,4,5,6;
    return .5*x.dot(d.cwiseProduct(x))+.5*x(0)*x(0)*x(2)+.01*x(2);
  };
  backend.jet=[&](const Eigen::Matrix4f&,const p::CoupledVector6& x) {
    ++jets;p::CoupledNativeJet out;out.valid=true;out.score_sum=100-energy(x);
    p::CoupledVector6 d;d<<.1,.12,3,4,5,6;
    out.score_gradient=-d.cwiseProduct(x);out.score_gradient(0)-=x(0)*x(2);
    out.score_gradient(2)-=.5*x(0)*x(0)+.01;
    out.score_hessian=-d.asDiagonal().toDenseMatrix();
    out.score_hessian(0,0)-=x(2);out.score_hessian(0,2)=out.score_hessian(2,0)=-x(0);
    return out;
  };
  backend.score=[&](const Eigen::Matrix4f& pose) {
    ++values;p::CoupledVector6 x;x.head<3>()=pose.block<3,1>(0,3).cast<double>();
    x.tail<3>()=pose.block<3,3>(0,0).cast<double>().eulerAngles(0,1,2);
    return 100-energy(x);
  };
  backend.refine=[&](const Eigen::Matrix4f&) {++aligns;return p::CoupledRefinement{};};
  p::WeakCoupledConfig config;
  const auto coupled=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(coupled.recommended && coupled.status=="LOCAL_REGULARIZED_REFINEMENT","coupled recommendation");
  require(coupled.jet_calls==jets && jets==2 && values==coupled.value_calls && values<=3 && aligns==0,"real call budgets");
  require(coupled.displaced_cross_norm>1e-4 && coupled.nominal_cross_norm<1e-10,"real displaced nonzero coupling");
  require(coupled.strong_selected && coupled.strong_eta.norm()>1e-4 && coupled.coupled_score>coupled.weak_score,"strong correction actual energy");
  const auto W=coupled.eigenvectors.leftCols(coupled.weak_dimension);
  require((W.transpose()*coupled.strong_eta).norm()<1e-10,"strong preserves weak coordinates");
  config.coupled=false;jets=values=0;
  const auto weak=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(weak.recommended && jets==1 && weak.strong_eta.norm()==0 && (weak.weak_eta-coupled.weak_eta).norm()<1e-12,"same weak ablation");
  require((weak.candidate-coupled.candidate).norm()>1e-4,"coupled changes actual candidate");
  anchor.valid=false;jets=values=0;
  auto skipped=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(!skipped.attempted && jets==0 && values==0,"missing anchor zero work");
  anchor.valid=true;
  skipped=p::runWeakCoupledRefinement(nominal,nominal,1100000000,1,true,anchor,backend,config);
  require(!skipped.triggered && jets==0 && values==0,"untriggered zero work");
  anchor.prediction(0,3)=10;config.coupled=true;jets=values=0;
  const auto capped=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(.8*capped.candidate_eta.head<3>().norm()<=.15+1e-10 && capped.candidate_eta.tail<3>().norm()<=2*std::acos(-1.)/180+1e-10,"total physical bounds");
  require((capped.eigenvectors.leftCols(capped.weak_dimension).transpose()*capped.strong_eta).norm()<1e-10,"total bound preserves u");
  const auto real_score=backend.score;const auto real_jet=backend.jet;
  anchor.prediction(0,3)=.12;int half_values=0;double displaced_x=0;
  backend.score=[&](const Eigen::Matrix4f&) {++half_values;return half_values==1 ? 50.:100.;};
  backend.jet=[&](const Eigen::Matrix4f& pose,const p::CoupledVector6& x) {
    if(x.head<3>().norm()>1e-8) displaced_x=x(0);return real_jet(pose,x);
  };
  const auto half=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(half.half_step && half.recommended && half.value_calls<=3 && half.jet_calls==2,"half then displaced jet budget");
  require(std::abs(displaced_x-half.weak_pose(0,3))<1e-8,"second jet at ACTUAL half weak point");
  backend.jet=real_jet;int bad_strong_values=0;
  backend.score=[&](const Eigen::Matrix4f& pose) {++bad_strong_values;return bad_strong_values==1 ? real_score(pose):50.;};
  const auto bad_strong=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(bad_strong.recommended && !bad_strong.strong_selected && (bad_strong.candidate-bad_strong.weak_pose).norm()==0,"strong energy rise retains legal weak point");
  config.near_quality_fraction=.2;jets=values=0;
  require(!p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config).attempted && jets==0,"quality contract frozen");
  config.near_quality_fraction=.05;
  backend.score=[](const Eigen::Matrix4f&) {return std::numeric_limits<double>::quiet_NaN();};
  const auto nonfinite=p::runWeakCoupledRefinement(nominal,prediction,1100000000,1,true,anchor,backend,config);
  require(!nonfinite.recommended && nonfinite.value_calls==2 && nonfinite.jet_calls==1,"nonfinite weak score cannot be rescued without comparison");
  // Neither static initialization nor a full NDT convergence claim is used.
  std::cout<<"weak/coupled, displaced jet, chart, caps, guards, budget PASS\n";
  return 0;
}catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}}
