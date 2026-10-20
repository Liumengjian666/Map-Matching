#include "events.hpp"
#include <iostream>
using namespace feasibility;
bool has(const Event& e,const char* text){return std::find(e.reasons.begin(),e.reasons.end(),text)!=e.reasons.end();}
Candidate good(){Candidate c;c.finite=c.converged=c.loss_valid=c.eligible=true;c.score=1;c.fitness=.1;c.overlap=1;c.iterations=2;c.loss.fill(.1);return c;}
int main(){try{
  Split s;s.training->resize(300);for(auto& b:s.blocks)b->resize(10);Mask m(s);Pose p=Pose::Identity();Candidate c=good();
  auto e=detect(c,m,p,0);require(!e.detected&&!e.permitted,"ordinary no recovery");
  auto d=decide({c},m,p,Selection());require(d.selected==0&&d.observation_valid&&!d.recovery_success&&!d.prediction_fallback,"legal nominal retained");
  c.pose(0,3)=.801;e=detect(c,m,p,0);require(e.detected&&e.permitted&&!e.old_trigger&&has(e,"TRANSLATION_LIMIT"),"translation admission event");
  d=decide({c},m,p,Selection());require(d.selected<0&&!d.observation_valid&&d.prediction_fallback&&d.status=="NO_ACCEPTABLE_MEASUREMENT","rejected nominal not resurrected");
  c=good();c.pose.block<3,3>(0,0)=Eigen::AngleAxisd(5.01*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  e=detect(c,m,p,0);require(e.detected&&e.permitted&&!e.old_trigger&&has(e,"ROTATION_LIMIT"),"rotation admission event");
  c=good();c.overlap=.49;e=detect(c,m,p,0);require(e.detected&&e.permitted&&has(e,"GEOMETRIC_SUPPORT_INSUFFICIENT"),"support event");
  c=good();c.converged=false;e=detect(c,m,p,0);require(e.old_trigger&&e.detected&&e.permitted,"old nonconvergence trigger retained");
  c=good();c.iterations=40;require(detect(c,m,p,0).old_trigger,"old maxiter trigger");c=good();c.loss.fill(.251);require(detect(c,m,p,0).old_trigger,"old median trigger");
  c=good();c.pose(0,0)=NAN;e=detect(c,m,p,0);require(e.detected&&!e.permitted&&has(e,"NONFINITE_POSE"),"nonfinite pose blocks G");
  c=good();c.score=NAN;e=detect(c,m,p,0);require(e.detected&&!e.permitted&&has(e,"NONFINITE_SCORE"),"nonfinite scores block G");
  c=good();c.pose(0,0)=2;e=detect(c,m,p,0);require(e.detected&&!e.permitted&&has(e,"INVALID_SE3"),"invalid finite SE3 blocks G");
  c=good();c.loss[0]=NAN;e=detect(c,m,p,0);require(e.detected&&!e.permitted&&has(e,"VALIDATION_UNAVAILABLE"),"missing evidence blocks recovery");
  Split sparse;sparse.training->resize(300);for(int b=0;b<3;++b)sparse.blocks[b]->resize(10);Mask few(sparse);e=detect(good(),few,p,0);require(e.detected&&!e.permitted&&has(e,"COVERAGE_INSUFFICIENT"),"coverage event not permission");
  c=good();c.pose(0,3)=.801;e=detect(c,m,p,20);require(e.detected&&!e.permitted&&e.permission=="BUDGET_EXHAUSTED","detection survives exhausted budget");
  std::vector<Candidate> cs{c,good()};cs[0].eligible=false;cs[1].id=1;cs[1].pose(0,3)=.3;cs[1].loss.fill(.01);deduplicate(cs);
  auto selection=coverage::select(cs,m);d=decide(cs,m,p,selection);require(d.selected==1&&d.recovery_success&&d.observation_valid,"supported eligible alternative");
  cs[1].loss.fill(.099);selection=coverage::select(cs,m);d=decide(cs,m,p,selection);require(!d.observation_valid&&d.prediction_fallback,"eligible but unsupported alternative not forced");
  cs[1].pose(0,3)=.9;cs[1].loss.fill(0);selection.robust=1;d=decide(cs,m,p,selection);require(!d.observation_valid,"even injected selector cannot accept invalid alternative");
  cs={good()};cs[0].eligible=false;cs[0].finite=false;cs.push_back(good());cs[1].id=1;selection.robust=1;d=decide(cs,m,p,selection);require(!d.observation_valid,"singleton survivor cannot vacuously recover");
  cs={good(),good()};cs[1].id=1;cs[1].pose(0,3)=.3;deduplicate(cs);selection=coverage::select(cs,m);d=decide(cs,m,p,selection);require(d.selected==0&&!d.recovery_success,"unsupported alternative keeps legal nominal");
  std::vector<Candidate> rejected;auto bad=good();bad.converged=false;rejected.push_back(bad);bad=good();bad.pose.block<3,3>(0,0)=Eigen::AngleAxisd(.1,Eigen::Vector3d::UnitZ()).toRotationMatrix();rejected.push_back(bad);
  bad=good();bad.overlap=.49;rejected.push_back(bad);bad=good();bad.score=NAN;rejected.push_back(bad);bad=good();bad.loss_valid=false;rejected.push_back(bad);bad=good();bad.pose(0,0)=NAN;rejected.push_back(bad);
  for(const auto& r:rejected){d=decide({r},m,p,Selection());require(!d.observation_valid&&d.selected<0&&d.prediction_fallback,"each admission failure rejects final nominal");}
  d=decide({good()},few,p,Selection());require(!d.observation_valid,"coverage cannot become final evidence");
  Pose bad_prediction=p;bad_prediction(0,0)=NAN;d=decide({good()},m,bad_prediction,Selection());require(!d.observation_valid&&!d.prediction_fallback,"invalid prediction cannot be legal fallback");
  cs={good(),good()};cs[0].pose(0,3)=.801;cs[0].eligible=false;cs[1].pose(0,3)=.799;cs[1].id=1;cs[1].loss.fill(0);deduplicate(cs);selection.robust=1;
  require(cs[1].duplicate_of==0&&!decide(cs,m,p,selection).observation_valid,"deduplicated alternative cannot be resurrected");
  cs={good(),good()};cs[0].pose(0,3)=.801;cs[0].eligible=false;cs[0].loss.fill(.02);cs[1].pose(0,3)=.3;cs[1].id=1;cs[1].loss.fill(0);deduplicate(cs);
  require(coverage::paired(cs[1].loss,cs[0].loss,m)==.02,"exact evidence boundary fixture");selection=coverage::select(cs,m);require(selection.robust<0&&!decide(cs,m,p,selection).observation_valid,"D equal .02 not strict acceptance");
  for(int n:{4,5}){Split partial;partial.training->resize(300);for(int b=0;b<n;++b)partial.blocks[b]->resize(10);Mask subset(partial);auto x=good();x.pose(0,3)=.81;x.eligible=false;auto y=good();y.id=1;y.pose(0,3)=.3;y.loss.fill(0);cs={x,y};
    selection=coverage::select(cs,subset);require(decide(cs,subset,p,selection).recovery_success,"state transition works with B4/B5");}
  Budget budget;for(int i=0;i<20;++i)require(budget.consume(),"shared token");require(!budget.consume(),"never over budget");
  std::cout<<"R10_EVENT_STATE_TESTS_PASS new_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
