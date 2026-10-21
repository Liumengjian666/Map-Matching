#include "common.hpp"
#include <iostream>
using namespace pilot;
int main(){try{
  Split source;source.training->resize(400);for(auto& b:source.blocks)b->resize(10);Mask m(source);
  Candidate c;c.finite=c.converged=c.loss_valid=true;c.score=1;c.fitness=.1;c.overlap=.8;c.loss.fill(.1);
  Pose identity=Pose::Identity();Predictor p(1000000000,identity);
  require((p.predict(1100000000)-identity).norm()==0,"history1 must hold");
  Vector6 xi;xi<<.1,.02,0,0,0,.02;c.pose=cause::expSE3(xi);
  p.completed(1100000000,c,m,identity);
  require((p.predict(1300000000)-cause::expSE3(3*xi)).norm()<1e-10,"body CV actual dt");
  const Pose before=p.history.back().pose;Candidate rejected=c;rejected.converged=false;
  p.completed(1200000000,rejected,m,before);
  require((p.predict(1300000000)-before).norm()==0&&p.hold,"rejection disables extrapolation");
  c.pose=before;p.completed(1300000000,c,m,before);require(!p.hold,"new admitted full resumes CV");
  bool caught=false;try{p.predict(1300000000);}catch(...){caught=true;}require(caught,"cannot predict past/completed time");
  caught=false;try{Pose bad=Pose::Constant(NAN);Predictor q(0,bad);}catch(...){caught=true;}require(caught,"invalid anchor rejected");
  // Rejected map observations cannot silently become full-history states.
  Candidate far=c;far.pose(0,3)+=1.;const auto last=p.history.back().ns;
  p.completed(1400000000,far,m,before);require(p.history.back().ns==last&&p.hold,"far full rejected");
  auto grid=training_grid::grid(before);require(grid.size()==45,"frozen grid reused");
  for(const auto& s:grid)require(translation(s.native_pose,before)<=.60&&rotation(s.native_pose,before)<=12.,"grid bounds");
  std::cout<<"PILOT_UNIT_PASS hold/body-CV/time/rejection/no-future/frozen45\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
