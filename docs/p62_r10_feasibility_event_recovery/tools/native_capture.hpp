#pragma once
#include "events.hpp"

namespace feasibility {
inline bool fitnessSafe(const Candidate& c){return rigid(c.pose)&&std::isfinite(c.score);}
template<class Fitness> inline void finishFitness(Candidate& c,const Fitness& evaluate){
  c.finite=false;c.fitness=NAN;
  if(!fitnessSafe(c))return;
  c.fitness=evaluate();c.finite=std::isfinite(c.fitness);
}
// The PCL optimizer and its arguments are unchanged. Capture returned pose
// before the auxiliary getFitnessScore NN pass, which the old wrapper did first.
template<class Capture> inline Candidate capturedAlign(NativeNdt& ndt,const Cloud::Ptr& source,const Pose& initial,int id,const Capture& capture){
  require(rigid(initial)&&!source->empty(),"invalid native input");ndt.checkGrid();ndt.setInputSource(source);Cloud aligned;
  const auto start=Clock::now();ndt.align(aligned,initial.cast<float>());
  Candidate c;c.id=id;c.initial=initial;c.align_ms=milliseconds(start);c.pose=ndt.getFinalTransformation().cast<double>();
  c.converged=ndt.hasConverged();c.iterations=ndt.getFinalNumIteration();c.score=ndt.getTransformationProbability();
  capture(c); // must flush before any auxiliary fitness NN or heldout validation
  finishFitness(c,[&](){return ndt.getFitnessScore();});ndt.checkGrid();return c;
}
} // namespace feasibility
