#pragma once
#include "../../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp"
#include <functional>

namespace shadow {
using Runner=std::function<Candidate(const Cloud::Ptr&,const Pose&,int,const char*)>;
struct Recovery {
  std::string direction="NOT_REQUESTED";bool invalid=false;int seeds=0,extra=0,weak_count=-1;
  size_t supported=0;double g_ms=0,extra_ms=0;
  Vector6 eigenvalues=Vector6::Constant(NAN),weak_vector=Vector6::Constant(NAN);
};
inline Recovery recover(NativeNdt& ndt,const Split& split,pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                        const Pose& initial,std::vector<Candidate>& candidates,Budget& budget,const Runner& run){
  Recovery r;
  if(budget.used>=20){r.direction="BUDGET_EXHAUSTED";return r;}
  const auto start=Clock::now();
  const Matrix6 g=ndt.geometry(split.training,candidates[0].pose,r.supported);
  Eigen::SelfAdjointEigenSolver<Matrix6> eig;if(g.allFinite())eig.compute(g);
  if(!g.allFinite()||eig.info()!=Eigen::Success){r.direction="INVALID_G";r.invalid=true;}
  else{
    r.eigenvalues=eig.eigenvalues();const double low=r.eigenvalues[0],high=r.eigenvalues[5];
    if(high<=1e-12)r.direction="NO_INFORMATION";
    else if(low< -1e-8*high){r.direction="INVALID_G_SPECTRUM";r.invalid=true;}
    else{
      r.weak_count=(r.eigenvalues.array()/high<=.01).count();
      if(low/high>.01)r.direction="NO_WEAK_DIRECTION";
      else{r.direction="WEAK_DIRECTION";r.weak_vector=eig.eigenvectors().col(0);}
    }
  }
  r.g_ms=milliseconds(start);
  if(r.direction!="WEAK_DIRECTION")return r;
  const Pose center=candidates[0].pose;std::vector<Pose> seeds;
  for(double sign:{1.,-1.}){
    const Pose guess=seed(center,r.weak_vector,sign);bool same=false;
    for(const auto& old:seeds)if(translation(guess,old)<1e-10&&rotation(guess,old)<1e-8)same=true;
    if(same)continue;seeds.push_back(guess);++r.seeds;
    if(!budget.consume()){r.direction="BUDGET_EXHAUSTED";break;}
    ++r.extra;auto candidate=run(split.training,guess,int(candidates.size()),"RECOVERY");
    r.extra_ms+=candidate.align_ms;validate(candidate,split,tree,initial);candidates.push_back(candidate);
    if(!candidate.finite){r.invalid=true;r.direction="INVALID_RECOVERY_CANDIDATE";break;}
  }
  return r;
}
inline double selectedMargin(const std::vector<Candidate>& candidates,int selected){
  const auto ids=pool(candidates);if(selected<0||ids.size()<2)return NAN;
  double result=std::numeric_limits<double>::infinity();
  for(int k:ids)if(k!=selected)result=std::min(result,paired(candidates[selected].loss,candidates[k].loss));
  return result;
}
} // namespace shadow
