#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>

namespace dog_prior_map_fastlio2_frontend_exp {
void settleWeakRefinementAnchor(CoupledAnchorState* anchor, uint64_t stamp_ns,
    const Eigen::Matrix4d& corrected_lidar, bool ordinary_stable_frame) {
  // A bounded local correction is not consumption of a nonlocal Pending event.
  // Existing settle refuses to overwrite a valid anchor or to seed on an
  // unstable/triggered frame. Propagation and the 2 s lifetime remain unchanged.
  settleCoupledAnchor(anchor,stamp_ns,corrected_lidar,ordinary_stable_frame,false);
}
namespace {
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point start) {
  return std::chrono::duration<double,std::milli>(Clock::now()-start).count();
}
bool rigid(const Eigen::Matrix4f& T) {
  const Eigen::Matrix3d R=T.block<3,3>(0,0).cast<double>();
  return T.allFinite() && (T.row(3)-Eigen::RowVector4f(0,0,0,1)).norm()<1e-6 &&
      (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-4 && std::abs(R.determinant()-1)<1e-4;
}
double angle(const Eigen::Matrix4f& a,const Eigen::Matrix4f& b) {
  return Eigen::Quaterniond(a.block<3,3>(0,0).cast<double>()).normalized().angularDistance(
      Eigen::Quaterniond(b.block<3,3>(0,0).cast<double>()).normalized())*180/std::acos(-1.0);
}
bool validConfig(const WeakCoupledConfig& c) {
  const double x[]={c.translation_limit_m,c.rotation_limit_deg,c.strong_step_cap,
      c.near_quality_fraction,c.minimum_rho,c.objective_margin,c.maximum_condition,c.solve_residual_limit};
  for(double v:x) if(!std::isfinite(v) || v<=0) return false;
  return c.translation_limit_m<=.15 && c.rotation_limit_deg<=2 && c.strong_step_cap<=.10 &&
      c.near_quality_fraction==.05 && c.minimum_rho==1e-4;
}
bool within(const CoupledVector6& eta,const WeakCoupledConfig& c) {
  return eta.allFinite() && .8*eta.head<3>().norm()<=c.translation_limit_m+1e-12 &&
      eta.tail<3>().norm()<=c.rotation_limit_deg*std::acos(-1.0)/180+1e-12;
}
// Convex ball intersection along the strong-only segment: fixed u is preserved.
double strongFraction(const CoupledVector6& weak,const CoupledVector6& strong,
    const WeakCoupledConfig& c) {
  if(within(weak+strong,c)) return 1;
  double lo=0,hi=1;
  for(int i=0;i<48;++i) {
    const double mid=(lo+hi)/2;
    if(within(weak+mid*strong,c)) lo=mid;else hi=mid;
  }
  return lo;
}
}  // namespace

WeakCoupledResult runWeakCoupledRefinement(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction,uint64_t stamp,std::size_t source_count,
    bool effective,const CoupledAnchorState& anchor,const CoupledNdtBackend& backend,
    const WeakCoupledConfig& config) {
  const auto start=Clock::now();const auto cpu=std::clock();
  WeakCoupledResult out;out.nominal=out.weak_pose=out.coupled_pose=out.candidate=nominal;
  out.prediction=prediction;out.anchor_prediction=anchor.prediction;
  auto finish=[&](const std::string& status) {
    out.status=status;out.total_ms=ms(start);out.cpu_ms=1000.*(std::clock()-cpu)/CLOCKS_PER_SEC;
    return out;
  };
  if(!validConfig(config) || !effective || !rigid(nominal) || !rigid(prediction) || !stamp || !source_count)
    return finish("INVALID_OR_INEFFECTIVE_NOMINAL");
  out.innovation_translation_m=(nominal.block<3,1>(0,3).cast<double>()-prediction.block<3,1>(0,3).cast<double>()).norm();
  out.innovation_rotation_deg=angle(nominal,prediction);
  out.triggered=out.innovation_translation_m>.12 || out.innovation_rotation_deg>3;
  out.anchor_valid=anchor.valid && !anchor.frozen && anchor.origin_stamp_ns &&
      anchor.propagated_stamp_ns==stamp && stamp>=anchor.origin_stamp_ns &&
      (stamp-anchor.origin_stamp_ns)/1e9<=kCoupledAnchorLifetimeS;
  out.anchor_age_s=anchor.origin_stamp_ns && stamp>=anchor.origin_stamp_ns ? (stamp-anchor.origin_stamp_ns)/1e9 : 0;
  if(!out.triggered) return finish("UNTRIGGERED_NOMINAL");
  if(!out.anchor_valid || !coupledAnchorChart(anchor.prediction,nominal.cast<double>(),&out.anchor_eta))
    return finish("ANCHOR_UNAVAILABLE_NOMINAL");
  if(!backend.jet || !backend.score) return finish("INVALID_BACKEND");
  out.attempted=true;
  CoupledNdtBackend counted;
  counted.jet=[&](const Eigen::Matrix4f& pose,const CoupledVector6& native) {
    ++out.jet_calls;const auto begin=Clock::now();
    CoupledNativeJet result;
    try {result=backend.jet(pose,native);}catch(...) {}
    out.jet_ms+=ms(begin);return result;
  };
  counted.score=[&](const Eigen::Matrix4f& pose) {
    ++out.value_calls;const auto begin=Clock::now();double score=std::numeric_limits<double>::quiet_NaN();
    try {score=backend.score(pose);}catch(...) {}
    out.value_ms+=ms(begin);return score;
  };
  const double count=static_cast<double>(source_count);
  const auto model=coupledNominalJet(nominal,count,counted,.8);
  if(!model.valid || model.score<=0) return finish("NOMINAL_JET_INVALID");
  out.nominal_score=model.score;
  Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig(model.H);
  if(eig.info()!=Eigen::Success || !eig.eigenvalues().allFinite()) return finish("NOMINAL_EIGEN_INVALID");
  out.eigenvalues=eig.eigenvalues();out.eigenvectors=eig.eigenvectors();
  // Same positive local-curvature contract and 1D/2D split as R2/R5.
  if(out.eigenvalues.minCoeff()<=0) return finish("NOMINAL_NONCONVEX");
  out.weak_dimension=out.eigenvalues(1)/out.eigenvalues(0)>=2 ? 1:2;
  const int k=out.weak_dimension,n=6-k;
  const Eigen::MatrixXd W=out.eigenvectors.leftCols(k),S=out.eigenvectors.rightCols(n);
  const CoupledMatrix6 H=out.eigenvectors.transpose()*model.H*out.eigenvectors;
  const CoupledVector6 g=out.eigenvectors.transpose()*model.gradient;
  out.nominal_cross_norm=H.topRightCorner(k,n).norm();out.nominal_strong_gradient=g.tail(n).norm();
  out.u_anchor=W.transpose()*out.anchor_eta;
  out.rho=std::max(config.minimum_rho,out.eigenvalues.head(k).mean());
  out.nominal_objective=-model.score/count+.5*out.rho*out.u_anchor.squaredNorm();
  const auto solve_start=Clock::now();
  const Eigen::MatrixXd Hvv=H.bottomRightCorner(n,n),Hvu=H.bottomLeftCorner(n,k);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> strong_eig(Hvv);
  if(strong_eig.info()!=Eigen::Success || strong_eig.eigenvalues().minCoeff()<=0)
    return finish("SCHUR_HVV_NOT_SPD");
  out.nominal_strong_condition=strong_eig.eigenvalues().maxCoeff()/strong_eig.eigenvalues().minCoeff();
  if(!std::isfinite(out.nominal_strong_condition) || out.nominal_strong_condition>config.maximum_condition)
    return finish("SCHUR_HVV_ILL_CONDITIONED");
  Eigen::LLT<Eigen::MatrixXd> strong_factor(Hvv);
  if(strong_factor.info()!=Eigen::Success) return finish("SCHUR_HVV_NOT_SPD");
  const Eigen::MatrixXd solved_cross=strong_factor.solve(Hvu);
  const Eigen::VectorXd solved_gradient=strong_factor.solve(g.tail(n));
  if(!solved_cross.allFinite() || !solved_gradient.allFinite() ||
      (Hvv*solved_cross-Hvu).norm()/std::max(1e-12,Hvu.norm())>config.solve_residual_limit ||
      (Hvv*solved_gradient-g.tail(n)).norm()/std::max(1e-12,g.tail(n).norm())>config.solve_residual_limit)
    return finish("SCHUR_HVV_SOLVE_FAILED");
  Eigen::MatrixXd A=H.topLeftCorner(k,k)-H.topRightCorner(k,n)*solved_cross;
  A+=out.rho*Eigen::MatrixXd::Identity(k,k);A=.5*(A+A.transpose()).eval();
  const Eigen::VectorXd rhs=-g.head(k)+H.topRightCorner(k,n)*solved_gradient+out.rho*out.u_anchor;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> schur_eig(A);
  if(schur_eig.info()!=Eigen::Success || !solved_cross.allFinite() || !solved_gradient.allFinite() || !rhs.allFinite())
    return finish("SCHUR_NONFINITE");
  const double floor=1e-4*std::max(1.0,A.diagonal().cwiseAbs().maxCoeff());
  out.weak_damping=std::max(0.0,floor-schur_eig.eigenvalues().minCoeff());
  A+=out.weak_damping*Eigen::MatrixXd::Identity(k,k);
  const double minimum=schur_eig.eigenvalues().minCoeff()+out.weak_damping;
  out.schur_condition=(schur_eig.eigenvalues().maxCoeff()+out.weak_damping)/minimum;
  if(!std::isfinite(out.schur_condition) || minimum<=0 || out.schur_condition>config.maximum_condition)
    return finish("SCHUR_ILL_CONDITIONED");
  Eigen::LLT<Eigen::MatrixXd> factor(A);
  if(factor.info()!=Eigen::Success) return finish("SCHUR_NOT_SPD");
  out.delta_u=factor.solve(rhs);
  out.weak_solve_residual=(A*out.delta_u-rhs).norm()/std::max(1e-12,rhs.norm());
  if(!out.delta_u.allFinite() || !std::isfinite(out.weak_solve_residual) || out.weak_solve_residual>config.solve_residual_limit)
    return finish("WEAK_SOLVE_FAILED");
  out.weak_solve_valid=true;out.weak_eta=W*out.delta_u;
  double scale=1;
  if(.8*out.weak_eta.head<3>().norm()>config.translation_limit_m)
    scale=std::min(scale,config.translation_limit_m/(.8*out.weak_eta.head<3>().norm()));
  if(out.weak_eta.tail<3>().norm()>config.rotation_limit_deg*std::acos(-1.0)/180)
    scale=std::min(scale,config.rotation_limit_deg*std::acos(-1.0)/(180*out.weak_eta.tail<3>().norm()));
  out.delta_u*=scale;out.weak_eta*=scale;out.solve_ms+=ms(solve_start);
  if(out.weak_eta.norm()<1e-9) return finish("NEGLIGIBLE_WEAK_STEP");
  out.weak_pose=coupledPoseAtEta(nominal,out.weak_eta,.8);
  out.full_weak_score=out.weak_score=counted.score(out.weak_pose);
  auto objective=[&](double score,const CoupledVector6& eta) {
    return -score/count+.5*out.rho*(W.transpose()*eta-out.u_anchor).squaredNorm();
  };
  auto quality=[&](double score,const CoupledVector6& eta) {
    return std::isfinite(score) && score>0 && within(eta,config) &&
        -score/count<=-model.score/count+config.near_quality_fraction*std::max(1.0,std::abs(model.score/count)) &&
        objective(score,eta)<out.nominal_objective-config.objective_margin*std::max(1.0,std::abs(out.nominal_objective));
  };
  out.weak_quality_valid=quality(out.weak_score,out.weak_eta);
  // Select the same real, legal weak point in both arms BEFORE conditional
  // correction. A full-point jet cannot certify a strong step at a half point.
  if(!out.weak_quality_valid) {
    out.weak_eta*=.5;out.delta_u*=.5;out.half_step=true;
    out.weak_pose=coupledPoseAtEta(nominal,out.weak_eta,.8);
    out.weak_score=counted.score(out.weak_pose);
    out.weak_quality_valid=quality(out.weak_score,out.weak_eta);
  }
  if(!out.weak_quality_valid) return finish("WEAK_OBJECTIVE_OR_NDT_QUALITY_REJECTED");
  CoupledVector6 chosen_eta=out.weak_eta;double chosen_score=out.weak_score;
  if(config.coupled) {
    // Actual displaced joint jet, not reuse of the diagonal nominal model.
    const auto displaced=coupledJointJet(nominal,out.weak_eta,out.eigenvectors,count,counted,.8);
    out.displaced_score=displaced.score;
    out.displaced_score_gap=std::isfinite(displaced.score) ? std::abs(displaced.score-out.weak_score):0;
    if(displaced.valid && out.displaced_score_gap<=1e-9*std::max(1.0,std::abs(out.weak_score))) {
      out.displaced_cross_norm=displaced.H.topRightCorner(k,n).norm();
      out.displaced_strong_gradient=displaced.gradient.tail(n).norm();
      const auto begin=Clock::now();
      const auto strong=coupledStrongStep(displaced,k,Eigen::VectorXd::Zero(k),config.strong_step_cap);
      out.strong_status=strong.status;out.strong_damping=strong.damping;out.strong_condition=strong.condition;
      out.strong_solve_residual=strong.residual;out.strong_solve_valid=strong.valid;
      if(strong.valid) {
        out.delta_v=strong.delta;out.strong_eta=S*out.delta_v;
        const double fraction=strongFraction(out.weak_eta,out.strong_eta,config);
        out.delta_v*=fraction;out.strong_eta*=fraction;
        out.coupled_pose=coupledPoseAtEta(nominal,out.weak_eta+out.strong_eta,.8);
        out.solve_ms+=ms(begin);
        if(out.strong_eta.norm()>1e-9) {
          out.coupled_score=counted.score(out.coupled_pose);
          if(quality(out.coupled_score,out.weak_eta+out.strong_eta) &&
              out.coupled_score>chosen_score+config.objective_margin*count*std::max(1.0,std::abs(chosen_score/count))) {
            chosen_eta=out.weak_eta+out.strong_eta;chosen_score=out.coupled_score;out.strong_selected=true;
          }
        } else out.strong_status="STRONG_SUPPRESSED_BY_TOTAL_BOUND";
      } else out.solve_ms+=ms(begin);
    } else out.strong_status=displaced.valid ? "DISPLACED_SCORE_OBJECTIVE_MISMATCH":"DISPLACED_JET_INVALID";
  }
  out.candidate_eta=chosen_eta;out.candidate=coupledPoseAtEta(nominal,chosen_eta,.8);
  out.candidate_score=chosen_score;
  out.candidate_objective=std::isfinite(chosen_score) ? objective(chosen_score,chosen_eta):out.nominal_objective;
  if(!rigid(out.candidate) || !within(chosen_eta,config) ||
      (out.candidate.block<3,1>(0,3).cast<double>()-nominal.block<3,1>(0,3).cast<double>()).norm()>config.translation_limit_m+1e-6 ||
      angle(out.candidate,nominal)>config.rotation_limit_deg+1e-4) return finish("FINAL_POSE_BOUND_OR_RIGIDITY_FAILED");
  out.recommended=quality(chosen_score,chosen_eta);
  return finish(out.recommended ? "LOCAL_REGULARIZED_REFINEMENT":"LOCAL_OBJECTIVE_OR_NDT_QUALITY_REJECTED");
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
