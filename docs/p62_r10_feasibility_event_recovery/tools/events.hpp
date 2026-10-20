#pragma once
#include "../../p62_r9_coverage_aware_recovery_v2/tools/coverage.hpp"

namespace feasibility {
using namespace coverage;
inline bool safeNative(const Candidate& c){return c.finite&&rigid(c.pose)&&std::isfinite(c.score)&&std::isfinite(c.fitness);}
inline bool admissible(const Candidate& c,const Mask& m,const Pose& prediction){
  return safeNative(c)&&c.converged&&rigid(prediction)&&bounded(c.pose,prediction)&&c.loss_valid&&m.valid(c.loss)&&std::isfinite(c.overlap)&&c.overlap>=.50;
}
struct Event {
  bool old_available=false,old_trigger=false,detected=false,permitted=false;
  std::vector<std::string> reasons;std::string permission="NOT_REQUESTED";
  std::string reasonText()const{std::string s;for(const auto& r:reasons){if(!s.empty())s+='|';s+=r;}return s.empty()?"NONE":s;}
};
inline Event detect(const Candidate& c,const Mask& m,const Pose& prediction,int budget_used){
  Event e;const bool pose_ok=rigid(c.pose),prediction_ok=rigid(prediction);
  e.old_available=m.evaluable()&&c.loss_valid&&m.valid(c.loss);
  e.old_trigger=e.old_available&&(!c.converged||c.iterations>=40||coverage::median(c.loss,m)>.25);
  if(!c.pose.allFinite())e.reasons.push_back("NONFINITE_POSE");else if(!pose_ok)e.reasons.push_back("INVALID_SE3");
  if(!std::isfinite(c.score)||!std::isfinite(c.fitness))e.reasons.push_back("NONFINITE_SCORE");
  if(!c.finite)e.reasons.push_back("INVALID_NATIVE_RESULT");
  if(!prediction_ok)e.reasons.push_back("INVALID_PREDICTION");
  if(!c.converged)e.reasons.push_back("OPTIMIZER_NOT_CONVERGED");
  if(c.iterations>=40)e.reasons.push_back("ITERATION_LIMIT");
  if(pose_ok&&prediction_ok){if(translation(c.pose,prediction)>.8)e.reasons.push_back("TRANSLATION_LIMIT");if(rotation(c.pose,prediction)>5.)e.reasons.push_back("ROTATION_LIMIT");}
  if(!m.training_ok)e.reasons.push_back("TRAINING_INSUFFICIENT");
  if(m.active.size()<4)e.reasons.push_back("COVERAGE_INSUFFICIENT");
  if(!e.old_available||!std::isfinite(c.overlap))e.reasons.push_back("VALIDATION_UNAVAILABLE");
  else{if(c.overlap<.50)e.reasons.push_back("GEOMETRIC_SUPPORT_INSUFFICIENT");if(coverage::median(c.loss,m)>.25)e.reasons.push_back("HELDOUT_MEDIAN_HIGH");}
  // Every failed admission must remain visible, even if native metadata is inconsistent.
  if(!admissible(c,m,prediction))e.reasons.push_back("NOMINAL_ADMISSION_REJECTED");
  e.detected=!e.reasons.empty();
  if(!e.detected)return e;
  if(!safeNative(c))e.permission="BLOCKED_UNSAFE_NATIVE";
  else if(!prediction_ok)e.permission="BLOCKED_INVALID_PREDICTION";
  else if(!m.evaluable()||!e.old_available||!std::isfinite(c.overlap))e.permission="BLOCKED_VALIDATION";
  else if(budget_used>=20)e.permission="BUDGET_EXHAUSTED";
  else{e.permitted=true;e.permission="PERMITTED";}
  return e;
}
struct Decision {int selected=-1;bool observation_valid=false,recovery_success=false,prediction_fallback=true;std::string status="NO_ACCEPTABLE_MEASUREMENT";};
inline Decision decide(const std::vector<Candidate>& cs,const Mask& m,const Pose& prediction,const Selection& comparison){
  require(!cs.empty(),"nominal slot required");Decision d;d.prediction_fallback=rigid(prediction);const auto ids=pool(cs);const int j=comparison.robust;
  bool supported=j>0&&size_t(j)<cs.size()&&ids.size()>=2&&std::find(ids.begin(),ids.end(),j)!=ids.end()&&admissible(cs[j],m,prediction);
  if(supported)for(int k:ids)if(k!=j&&!(coverage::paired(cs[j].loss,cs[k].loss,m)>.02))supported=false;
  if(supported){d.selected=j;d.recovery_success=true;d.status="ACCEPT_REPLACEMENT";}
  else if(admissible(cs[0],m,prediction)){d.selected=0;d.status="ACCEPT_NOMINAL";}
  if(d.selected>=0){d.observation_valid=true;d.prediction_fallback=false;}
  return d;
}
} // namespace feasibility
