#pragma once
#include "../../p62_r21_relative_temporal_admission/tools/policy.hpp"
namespace r22 {
using namespace r21;
enum class Policy { Native, Gate, Multi, Relative };
inline const char* name(Policy p){
 switch(p){case Policy::Native:return "BASELINE_NATIVE_NDT";case Policy::Gate:return "QUALITY_GATE_ONLY";case Policy::Multi:return "QUALITY_TRIGGERED_MULTISTART";default:return "SINGLE_FRAME_RELATIVE_ADMISSION";}
}
inline bool searchPolicy(Policy p){return p==Policy::Multi||p==Policy::Relative;}
inline int select(Policy p,const std::vector<Candidate>& cs,const Mask& mask,const Pose& tp){
 require(!cs.empty(),"empty candidate collection");
 if(p==Policy::Native)return r17::quality(cs[0],mask)?0:-1;
 int chosen=r20::choose(cs,mask,tp);
 return chosen<0&&p==Policy::Relative?r21::chooseRelative(cs,mask,tp):chosen;
}
}
