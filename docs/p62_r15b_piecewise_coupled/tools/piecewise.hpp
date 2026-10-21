#pragma once

#include "coupled.hpp"

namespace coupled {

struct AssociationDelta {
  size_t changed_points=0;
  size_t total_points=0;
  size_t added_leaf_memberships=0;
  size_t removed_leaf_memberships=0;
  size_t nominal_leaf_memberships=0;
  size_t dynamic_leaf_memberships=0;
  double changed_point_fraction=NAN;
  double changed_membership_fraction=NAN;
};

inline AssociationDelta compareAssociations(const Signature& nominal,const Signature& other) {
  require(nominal.size()==other.size(),"association point count mismatch");
  AssociationDelta d;d.total_points=nominal.size();
  for(size_t i=0;i<nominal.size();++i){
    const auto& a=nominal[i];const auto& b=other[i];
    d.nominal_leaf_memberships+=a.size();d.dynamic_leaf_memberships+=b.size();
    if(a!=b)++d.changed_points;
    size_t ia=0,ib=0;
    while(ia<a.size()&&ib<b.size()){
      if(a[ia]==b[ib]){++ia;++ib;}
      else if(a[ia]<b[ib]){++d.removed_leaf_memberships;++ia;}
      else{++d.added_leaf_memberships;++ib;}
    }
    d.removed_leaf_memberships+=a.size()-ia;
    d.added_leaf_memberships+=b.size()-ib;
  }
  d.changed_point_fraction=d.total_points?double(d.changed_points)/d.total_points:NAN;
  const size_t denominator=std::max(d.nominal_leaf_memberships,size_t(1));
  d.changed_membership_fraction=double(d.added_leaf_memberships+d.removed_leaf_memberships)/denominator;
  return d;
}

inline std::string associationDigest(const Signature& signature) {
  uint64_t h=1469598103934665603ULL;
  auto add=[&](uint64_t v){for(int b=0;b<64;b+=8){h=(h^uint8_t(v>>b))*1099511628211ULL;}};
  for(const auto& ids:signature){add(ids.size());for(uintptr_t id:ids)add(uint64_t(id));}
  std::ostringstream out;out<<std::hex<<std::setw(16)<<std::setfill('0')<<h;return out.str();
}

struct BoundaryStencilSummary {
  size_t sample_count=0;
  size_t changed_sample_count=0;
  size_t changed_point_memberships=0;
  size_t total_point_memberships=0;
  size_t added_leaf_memberships=0;
  size_t removed_leaf_memberships=0;
  size_t nominal_leaf_memberships_per_sample=0;
  size_t max_changed_points_in_sample=0;
};

struct PiecewiseDerivativeResult {
  bool ok=false;
  std::string status="NOT_RUN";
  Vector6 g_z=Vector6::Constant(NAN);
  Matrix6 h_raw=Matrix6::Constant(NAN);
  Matrix6 h_model=Matrix6::Constant(NAN);
  double f_fixed_zero=NAN;
  double f_dynamic_zero=NAN;
  double fixed_dynamic_parity_abs=NAN;
  double pcl_derivative_score_parity_abs=NAN;
  double gradient_check_max_rel=NAN;
  double hessian_antisymmetry_rel=NAN;
  double hessian_check_max_rel=NAN;
  double hessian_step_spread_max=NAN;
  size_t nominal_points=0;
  size_t nominal_association_count=0;
  size_t associated_points=0;
  std::string nominal_association_digest;
  int objective_evaluations=0;
  int pcl_association_query_batches=0;
  BoundaryStencilSummary stencil_boundary;
};

class NominalAssociationBranch {
 public:
  NominalAssociationBranch(ExactPclObjective& ndt,Pose t0)
      : ndt_(ndt),t0_(std::move(t0)),r0_(t0_.block<3,3>(0,0)),
        nominal_(ndt_.associations(t0_)) {
    require(rigid(t0_)&&ndt_.trainingCloud()&&!ndt_.trainingCloud()->empty(),"invalid nominal association branch");
    const Cloud transformed=ndt_.transformed(t0_);
    require(transformed.size()==ndt_.trainingCloud()->size(),"nominal float transform shape mismatch");
    x_.reserve(transformed.size());y0_.reserve(transformed.size());
    for(size_t i=0;i<transformed.size();++i){
      const auto& x=ndt_.trainingCloud()->points[i];
      const auto& y=transformed.points[i];
      x_.emplace_back(double(x.x),double(x.y),double(x.z));
      // This float-transformed PCL point is the exact branch baseline. At z=0
      // the branch objective therefore uses precisely the same residual basis.
      y0_.emplace_back(double(y.x),double(y.y),double(y.z));
    }
    require(nominal_.leaves.size()==x_.size(),"nominal Gaussian association shape mismatch");
  }

  const Pose& nominalPose()const{return t0_;}
  const ExactPclObjective::Associations& nominalAssociations()const{return nominal_;}
  size_t pointCount()const{return x_.size();}

  std::vector<Eigen::Vector3d> continuousPoints(const Vector6& z)const{
    require(z.allFinite(),"nonfinite local branch coordinate");
    const Pose delta=cause::expSE3(scaleMatrix()*z);
    const Eigen::Matrix3d rd=delta.block<3,3>(0,0);
    const Eigen::Vector3d td=delta.block<3,1>(0,3);
    std::vector<Eigen::Vector3d> points;points.reserve(x_.size());
    for(size_t i=0;i<x_.size();++i){
      const Eigen::Vector3d displacement=r0_*(rd*x_[i]+td-x_[i]);
      const Eigen::Vector3d y=y0_[i]+displacement;
      if(!y.allFinite())return {};
      points.push_back(y);
    }
    return points;
  }

  double fixedObjective(const Vector6& z)const{
    const auto points=continuousPoints(z);
    if(points.size()!=x_.size())return NAN;
    const double score=ndt_.scoreFixed(points,nominal_.leaves);
    return std::isfinite(score)?-score/double(x_.size()):NAN;
  }

  double dynamicObjective(const Pose& pose)const{return ndt_.objective(pose);}

  AssociationDelta dynamicAssociationDelta(const Pose& pose)const{
    const auto associations=ndt_.associations(pose);
    return compareAssociations(nominal_.canonical_ids,associations.canonical_ids);
  }

  PiecewiseDerivativeResult differentiate()const{
    PiecewiseDerivativeResult out;
    out.nominal_points=x_.size();out.nominal_association_count=nominal_.association_count;
    out.nominal_association_digest=associationDigest(nominal_.canonical_ids);
    for(const auto& v:nominal_.canonical_ids)if(!v.empty())++out.associated_points;
    out.f_fixed_zero=fixedObjective(Vector6::Zero());++out.objective_evaluations;
    out.f_dynamic_zero=ndt_.objective(t0_);
    const double pcl_score=ndt_.pclDerivativeScore(t0_);
    out.pcl_derivative_score_parity_abs=std::abs(pcl_score-ndt_.directScore(t0_));
    out.fixed_dynamic_parity_abs=std::abs(out.f_fixed_zero-out.f_dynamic_zero);
    if(!std::isfinite(out.f_fixed_zero)||!std::isfinite(out.f_dynamic_zero)||
       !std::isfinite(out.pcl_derivative_score_parity_abs)||
       out.fixed_dynamic_parity_abs>1e-12||out.pcl_derivative_score_parity_abs>1e-9){
      out.status="NOMINAL_FIXED_DYNAMIC_SCORE_PARITY_FAILED";return out;
    }

    std::string failure;
    out.g_z=gradientAt(Vector6::Zero(),out.objective_evaluations,failure);
    if(!failure.empty()||!out.g_z.allFinite()){
      out.status=failure.empty()?"NONFINITE_FIXED_BRANCH_GRADIENT":failure;return out;
    }
    const auto directions=checkDirections();
    double gradient_error=0.;
    for(const auto& v:directions){
      const double h=kDirectionalStep;
      const double fp=fixedObjective(h*v),fm=fixedObjective(-h*v);out.objective_evaluations+=2;
      const double fd=(fp-fm)/(2*h),model=out.g_z.dot(v);
      if(!std::isfinite(fd)||!std::isfinite(model)){out.status="NONFINITE_FIXED_BRANCH_DIRECTIONAL_CHECK";return out;}
      if(std::abs(fd)>1e-8||std::abs(model)>1e-8)gradient_error=std::max(gradient_error,relativeError(fd,model));
    }
    out.gradient_check_max_rel=gradient_error;
    if(gradient_error>2e-2){out.status="FIXED_BRANCH_DIRECTIONAL_DERIVATIVE_FAILED";return out;}

    for(int i=0;i<6;++i){
      Vector6 zp=Vector6::Zero(),zm=Vector6::Zero();zp[i]=kHessianStep;zm[i]=-kHessianStep;
      const Vector6 gp=gradientAt(zp,out.objective_evaluations,failure);
      if(!failure.empty()){out.status=failure;return out;}
      const Vector6 gm=gradientAt(zm,out.objective_evaluations,failure);
      if(!failure.empty()){out.status=failure;return out;}
      out.h_raw.col(i)=(gp-gm)/(2*kHessianStep);
    }
    if(!out.h_raw.allFinite()){out.status="NONFINITE_FIXED_BRANCH_HESSIAN";return out;}
    out.hessian_antisymmetry_rel=(out.h_raw-out.h_raw.transpose()).norm()/std::max(1.,out.h_raw.norm());
    if(!std::isfinite(out.hessian_antisymmetry_rel)||out.hessian_antisymmetry_rel>.05){out.status="FIXED_BRANCH_HESSIAN_ASYMMETRY_FAILED";return out;}

    const std::array<double,3> steps{{4e-3,8e-3,16e-3}};
    double max_relative=0.,max_spread=1.;
    for(const auto& v:directions){
      std::array<double,3> scalar{};
      for(size_t si=0;si<steps.size();++si){
        const double h=steps[si];
        const double fp=fixedObjective(h*v),fm=fixedObjective(-h*v);out.objective_evaluations+=2;
        scalar[si]=(fp-2*out.f_fixed_zero+fm)/(h*h);
        if(!std::isfinite(scalar[si])){out.status="NONFINITE_FIXED_BRANCH_SCALAR_HESSIAN";return out;}
        if(si==1){const double predicted=v.dot(out.h_raw*v);max_relative=std::max(max_relative,relativeError(scalar[si],predicted));}
      }
      for(size_t si=1;si<steps.size();++si){
        const double lo=std::min(std::abs(scalar[si-1]),std::abs(scalar[si]));
        const double hi=std::max(std::abs(scalar[si-1]),std::abs(scalar[si]));
        const double ratio=lo<=1e-8?(hi<=1e-8?1.:std::numeric_limits<double>::infinity()):hi/lo;
        max_spread=std::max(max_spread,ratio);
      }
    }
    out.hessian_check_max_rel=max_relative;out.hessian_step_spread_max=max_spread;
    if(max_relative>1e-1||max_spread>2.){out.status="FIXED_BRANCH_SCALAR_HESSIAN_CHECK_FAILED";return out;}
    out.h_model=.5*(out.h_raw+out.h_raw.transpose()).eval();
    if(!out.h_model.allFinite()){out.status="NONFINITE_SYMMETRIZED_FIXED_BRANCH_HESSIAN";return out;}
    out.status="FIXED_ASSOCIATION_BRANCH_DERIVATIVES_PASS";out.ok=true;return out;
  }

  BoundaryStencilSummary boundaryStencilSummary()const{
    BoundaryStencilSummary summary;
    const auto directions=checkDirections();std::vector<Vector6> samples;
    for(int i=0;i<6;++i){Vector6 d=Vector6::Zero();d[i]=kGradientStep;samples.push_back(d);samples.push_back(-d);}
    for(const auto& v:directions){samples.push_back(kDirectionalStep*v);samples.push_back(-kDirectionalStep*v);}
    for(int i=0;i<6;++i){
      Vector6 outer=Vector6::Zero();outer[i]=kHessianStep;
      for(double sign:{-1.,1.})for(int j=0;j<6;++j){Vector6 inner=Vector6::Zero();inner[j]=kGradientStep;
        samples.push_back(sign*outer+inner);samples.push_back(sign*outer-inner);}
    }
    for(double h:{4e-3,8e-3,16e-3})for(const auto& v:directions){samples.push_back(h*v);samples.push_back(-h*v);}
    summary.nominal_leaf_memberships_per_sample=nominal_.association_count;
    for(const auto& z:samples){
      const Pose pose=poseAt(t0_,z);const auto assoc=ndt_.associations(pose);
      const auto delta=compareAssociations(nominal_.canonical_ids,assoc.canonical_ids);
      ++summary.sample_count;
      if(delta.changed_points){++summary.changed_sample_count;summary.max_changed_points_in_sample=std::max(summary.max_changed_points_in_sample,delta.changed_points);}
      summary.changed_point_memberships+=delta.changed_points;
      summary.total_point_memberships+=delta.total_points;
      summary.added_leaf_memberships+=delta.added_leaf_memberships;
      summary.removed_leaf_memberships+=delta.removed_leaf_memberships;
    }
    return summary;
  }

 private:
  Vector6 gradientAt(const Vector6& z,int& evaluations,std::string& failure)const{
    Vector6 g;
    for(int i=0;i<6;++i){Vector6 zp=z,zm=z;zp[i]+=kGradientStep;zm[i]-=kGradientStep;
      const double fp=fixedObjective(zp),fm=fixedObjective(zm);evaluations+=2;
      if(!std::isfinite(fp)||!std::isfinite(fm)){failure="NONFINITE_FIXED_BRANCH_GRADIENT_STENCIL";return Vector6::Constant(NAN);}
      g[i]=(fp-fm)/(2*kGradientStep);
    }
    failure.clear();return g;
  }
  static std::vector<Vector6> checkDirections(){
    std::vector<Vector6> d;for(int i=0;i<6;++i){Vector6 v=Vector6::Zero();v[i]=1.;d.push_back(v);}
    for(const auto& ij:std::array<std::pair<int,int>,4>{{{0,3},{1,4},{2,5},{0,1}}}){
      Vector6 v=Vector6::Zero();v[ij.first]=v[ij.second]=1./std::sqrt(2.);d.push_back(v);}
    return d;
  }
  ExactPclObjective& ndt_;
  Pose t0_;
  Eigen::Matrix3d r0_;
  ExactPclObjective::Associations nominal_;
  std::vector<Eigen::Vector3d> x_,y0_;
};

} // namespace coupled
