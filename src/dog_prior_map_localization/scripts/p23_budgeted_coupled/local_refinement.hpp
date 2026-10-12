#pragma once
// Bounded experiment policy; all derivatives/chart/linear solves are old R2 APIs.
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_local_math.hpp"
#include <Eigen/Eigenvalues>
#include <chrono>
#include <string>
namespace p23 {
namespace f = dog_prior_map_fastlio2_frontend_exp;
using Clock = std::chrono::steady_clock;
inline double ms(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
struct Proposal {
  f::CoupledVector6 eigenvalues=f::CoupledVector6::Zero();
  f::CoupledMatrix6 Q=f::CoupledMatrix6::Identity();
  Eigen::VectorXd u, predictor, correction, strong_gradient;
  Eigen::Matrix4f weak=Eigen::Matrix4f::Identity(),coupled=Eigen::Matrix4f::Identity();
  double scores[3]={NAN,NAN,NAN},alpha=0,selected_u=0;
  double symmetry=0,weak_ms=0,eigen_ms=0,coupled_ms=0,solve_ms=0;
  double predicted_score=NAN,coupled_score=NAN,condition=0,damping=0,residual=0;
  int weak_dimension=0;
  bool valid=false,fallback=false,capped=false;
  std::string status="INVALID_NOMINAL_JET";
};
inline Proposal propose(const Eigen::Matrix4f& base,std::size_t count,const f::CoupledNdtBackend& backend) {
  Proposal out;out.weak=out.coupled=base;
  const auto weak_start=Clock::now();
  auto model=f::coupledNominalJet(base,count,backend,.8);
  if (!model.valid) { out.weak_ms=ms(weak_start);return out; }
  out.scores[0]=model.score;
  out.symmetry=(model.H-model.H.transpose()).norm();
  out.status="INVALID_HESSIAN_SYMMETRY";
  if (out.symmetry>1e-10*std::max(1.0,model.H.norm())) {out.weak_ms=ms(weak_start);return out;}
  const auto eigen_start=Clock::now();
  Eigen::SelfAdjointEigenSolver<f::CoupledMatrix6> eig(model.H);
  out.eigen_ms=ms(eigen_start);
  out.status="INVALID_EIGENSYSTEM";
  if (eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite() || !eig.eigenvectors().allFinite()) {
    out.weak_ms=ms(weak_start);return out;
  }
  out.eigenvalues=eig.eigenvalues();out.Q=eig.eigenvectors();out.status="NOMINAL_NONCONVEX";
  if (out.eigenvalues.minCoeff()<=0) {out.weak_ms=ms(weak_start);return out;}
  out.weak_dimension=out.eigenvalues(1)/out.eigenvalues(0)>=2.0?1:2;
  out.valid=true;const auto w=out.Q.col(0);
  const double tn=.8*w.head<3>().norm(),rn=w.tail<3>().norm();
  out.alpha=.999*std::min(tn>1e-12?.20/tn:1e12,rn>1e-12?2*M_PI/180/rn:1e12);
  out.u=Eigen::VectorXd::Zero(out.weak_dimension);
  const Eigen::Matrix4f plus=f::coupledPoseAtEta(base,w*out.alpha,.8);
  const Eigen::Matrix4f minus=f::coupledPoseAtEta(base,-w*out.alpha,.8);
  out.scores[1]=backend.score(plus);out.scores[2]=backend.score(minus);
  // Strict raw-score maximization; finite nominal wins ties, then plus, minus.
  double best=out.scores[0];
  for (int i=1;i<=2;++i) if (std::isfinite(out.scores[i]) && out.scores[i]>best) {
    best=out.scores[i];out.selected_u=(i==1?out.alpha:-out.alpha);
  }
  out.u(0)=out.selected_u;
  out.weak=f::coupledPoseAtEta(base,w*out.selected_u,.8);out.coupled=out.weak;
  out.weak_ms=ms(weak_start);out.status="ZERO_RETAINED";
  if (out.selected_u==0) return out;
  const auto coupled_start=Clock::now();
  const int k=out.weak_dimension,n=6-k;
  const auto W=out.Q.leftCols(k),S=out.Q.rightCols(n);
  auto initial_joint=f::coupledJointJet(base,f::CoupledVector6::Zero(),out.Q,count,backend,.8);
  initial_joint.gradient.setZero(); // Predictor ONLY; no nominal gradient double-count.
  auto solve_start=Clock::now();
  const auto pred=f::coupledStrongStep(initial_joint,k,out.u,.10);
  out.solve_ms+=ms(solve_start);out.status="COUPLED_FALLBACK:"+pred.status;
  if (!pred.valid) {out.fallback=true;out.coupled_ms=ms(coupled_start);return out;}
  out.predictor=pred.delta;
  const f::CoupledVector6 eta=W*out.u+S*out.predictor;
  const auto predicted=f::coupledPoseAtEta(base,eta,.8);
  out.predicted_score=backend.score(predicted);
  const auto shifted=f::coupledJointJet(base,eta,out.Q,count,backend,.8);
  out.strong_gradient=shifted.gradient.tail(n);
  solve_start=Clock::now();
  const auto residual=f::coupledStrongStep(shifted,k,Eigen::VectorXd::Zero(k),.10);
  out.solve_ms+=ms(solve_start);out.status="COUPLED_FALLBACK:"+residual.status;
  if (!residual.valid) {out.fallback=true;out.coupled_ms=ms(coupled_start);return out;}
  Eigen::VectorXd v=out.predictor+residual.delta;
  out.capped=pred.capped||residual.capped||v.norm()>.10;
  if (v.norm()>.10) v*=.10/v.norm();
  out.correction=v-out.predictor;
  out.coupled=f::coupledPoseAtEta(base,W*out.u+S*v,.8);
  out.condition=residual.condition;out.damping=residual.damping;out.residual=residual.residual;
  if (!out.coupled.allFinite()) {out.coupled=out.weak;out.fallback=true;out.status="COUPLED_FALLBACK:NONFINITE";}
  else out.status="COUPLED_ONE_RESIDUAL_STEP";
  out.coupled_score=backend.score(out.coupled);
  out.coupled_ms=ms(coupled_start);return out;
}
}  // namespace p23
