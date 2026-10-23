#pragma once
#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"

namespace r17 {
using namespace pilot;
enum class Policy {Baseline, Relaxed, Consensus};
inline const char* name(Policy p){return p==Policy::Baseline?"BASELINE_5DEG":p==Policy::Relaxed?"RELAXED_6DEG":"EVENT_CONSENSUS";}
inline bool quality(const Candidate& c,const Mask& mask){
  return safeNative(c)&&c.converged&&c.loss_valid&&mask.valid(c.loss)&&std::isfinite(c.overlap)&&c.overlap>=.50;
}
inline bool base(const Candidate& c,const Mask& mask,const Pose& tp,double max_rotation){
  return rigid(tp)&&quality(c,mask)&&translation(c.pose,tp)<=.8&&rotation(c.pose,tp)<=max_rotation;
}
inline bool needsConfirmation(const Candidate& full,const Mask& mask,const Pose& tp){
  if(!rigid(tp)||!quality(full,mask)||translation(full.pose,tp)>.8)return false;
  const double r=rotation(full.pose,tp);return r>5.&&r<=6.;
}
inline bool confirmed(const Candidate& full,const Candidate& split,const Mask& mask){
  return quality(full,mask)&&quality(split,mask)&&translation(full.pose,split.pose)<=.10&&rotation(full.pose,split.pose)<=1.;
}
inline void advance(Predictor& predictor,int64_t ns,const Candidate& full,bool accepted){
  require(ns>predictor.history.back().ns,"noncausal policy update");
  if(accepted){require(rigid(full.pose),"accepted pose invalid");predictor.history.push_back({ns,full.pose});
    if(predictor.history.size()>2)predictor.history.erase(predictor.history.begin());predictor.hold=false;
  }else predictor.hold=true;
}
} // namespace r17
