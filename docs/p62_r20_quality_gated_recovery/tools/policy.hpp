#pragma once
#include "../../p62_r17_consensus_admission/tools/policy.hpp"

namespace r20 {
using namespace pilot;
constexpr double HELDOUT_LIMIT=.30;
constexpr int EXTRA_LIMIT=12;
inline double lossMean(const Candidate& c,const Mask& mask){
  return c.loss_valid&&mask.valid(c.loss)?coverage::mean(c.loss,mask):NAN;
}
inline bool accepted(const Candidate& c,const Mask& mask){
  return r17::quality(c,mask)&&lossMean(c,mask)<=HELDOUT_LIMIT;
}
// Map-axis yaw about the sensor origin: left-multiply R only, preserve p.
inline Pose yawSeed(const Pose& tp,int sign){
  require(rigid(tp)&&(sign==1||sign==-1),"invalid yaw seed input");
  Pose seed=tp;
  seed.block<3,3>(0,0)=Eigen::AngleAxisd(sign*5.*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix()*tp.block<3,3>(0,0);
  require(rigid(seed)&&translation(seed,tp)==0&&std::abs(rotation(seed,tp)-5.)<1e-6,"invalid yaw seed");
  return seed;
}
// Exact score tie -> fixed dimensionless TP distance -> candidate ID.
inline double distanceToPrediction(const Candidate& c,const Pose& tp){
  return std::hypot(translation(c.pose,tp)/.8,rotation(c.pose,tp)/5.);
}
inline int choose(const std::vector<Candidate>& candidates,const Mask& mask,const Pose& tp){
  int best=-1;
  for(size_t i=0;i<candidates.size();++i){
    const auto& c=candidates[i];if(!accepted(c,mask))continue;
    if(best<0){best=int(i);continue;}const auto& b=candidates[best];
    const double d=distanceToPrediction(c,tp),db=distanceToPrediction(b,tp);
    if(c.score>b.score||(c.score==b.score&&(d<db||(d==db&&c.id<b.id))))best=int(i);
  }
  return best;
}
inline bool maySearch(bool multistart,bool event,int used){
  require(used>=0&&used<=EXTRA_LIMIT&&used%2==0,"invalid trajectory budget");
  return multistart&&event&&used+2<=EXTRA_LIMIT;
}
// Diagnostic only; selection never deletes a raw candidate.
inline int distinctCount(const std::vector<Candidate>& candidates){
  std::vector<int> reps;
  for(size_t i=0;i<candidates.size();++i){
    if(!safeNative(candidates[i]))continue;bool same=false;
    for(int j:reps)if(duplicate(candidates[i].pose,candidates[j].pose)){same=true;break;}
    if(!same)reps.push_back(int(i));
  }
  return int(reps.size());
}
} // namespace r20
