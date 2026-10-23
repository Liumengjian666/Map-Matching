#pragma once
#include "../../p62_r20_quality_gated_recovery/tools/policy.hpp"
namespace r21 {
using namespace r20;
// A nominal comparator need not be converged or pass overlap>=.50, but its
// native pose and every compared metric must be finite and valid.
inline bool comparable(const Candidate& c,const Mask& mask){
  return safeNative(c)&&c.loss_valid&&mask.valid(c.loss)&&std::isfinite(c.overlap)&&c.overlap>=0&&c.overlap<=1;
}
inline bool dominates(const Candidate& candidate,const Candidate& nominal,const Mask& mask){
  return r17::quality(candidate,mask)&&comparable(nominal,mask)
    &&candidate.score>nominal.score&&candidate.overlap>nominal.overlap
    &&lossMean(candidate,mask)<lossMean(nominal,mask)
    &&!duplicate(candidate.pose,nominal.pose);
}
inline bool rankedBefore(const Candidate& a,const Candidate& b,const Pose& tp){
  if(a.score!=b.score)return a.score>b.score;
  const double da=distanceToPrediction(a,tp),db=distanceToPrediction(b,tp);
  return da!=db?da<db:a.id<b.id;
}
inline int chooseRelative(const std::vector<Candidate>& cs,const Mask& mask,const Pose& tp){
  int best=-1;
  for(size_t i=1;i<cs.size();++i)if(dominates(cs[i],cs[0],mask)&&
      (best<0||rankedBefore(cs[i],cs[best],tp)))best=int(i);
  return best;
}
struct Pending {
  bool active=false;int frame=-1,id=-1;int64_t ns=0;Pose pose=Pose::Identity();
  void establish(int f,int64_t stamp,const Candidate& c){
    require(!active&&rigid(c.pose),"invalid pending establishment");active=true;frame=f;ns=stamp;id=c.id;pose=c.pose;
  }
  Pose seed(int next_frame,int64_t next_stamp)const{
    require(active&&next_frame==frame+1&&next_stamp>ns,"pending must be next-frame causal");
    // One shadow pose only: R14B history-size-one rule gives zero-order hold.
    // No rejected/pending correction is inserted into the official history.
    Predictor shadow(ns,pose);return shadow.predict(next_stamp);
  }
  void clear(){active=false;}
};
inline bool canSearch(int used){require(used>=0&&used<=12,"invalid budget");return used+2<=12;}
inline bool canConfirm(int used){require(used>=0&&used<=12,"invalid budget");return used+1<=12;}
} // namespace r21
