#include "../../p62_r10_feasibility_event_recovery/tools/events.hpp"
#include "se3.hpp"
#include <iostream>
using namespace feasibility;
int main(){try{
  Split split;split.training->resize(300);for(auto& block:split.blocks)block->resize(10);Mask mask(split);
  Pose p=Pose::Identity();Candidate nominal;nominal.finite=true;nominal.converged=true;
  nominal.score=1;nominal.fitness=.01;nominal.loss_valid=true;nominal.loss.fill(.2);nominal.overlap=.8;nominal.eligible=true;
  require(!detect(nominal,mask,p,0).detected,"ordinary event");
  Candidate rejected=nominal;rejected.pose.block<3,3>(0,0)=Eigen::AngleAxisd(6*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();rejected.eligible=false;
  auto event=detect(rejected,mask,p,0);require(event.detected&&event.permitted&&!event.old_trigger,"new bounds event");
  std::vector<Candidate> weak{rejected},proposed{rejected};Candidate alternative=nominal;alternative.id=1;alternative.pose(0,3)=.2;alternative.loss.fill(.1);
  proposed.push_back(alternative);require(weak.size()==1&&weak[0].pose==rejected.pose,"arms must not share candidate vectors");
  deduplicate(proposed);auto pick=coverage::select(proposed,mask);auto decision=decide(proposed,mask,p,pick);
  require(decision.observation_valid&&decision.recovery_success&&decision.selected==1,"supported replacement");
  decision=decide(weak,mask,p,coverage::select(weak,mask));require(!decision.observation_valid&&decision.prediction_fallback&&decision.status=="NO_ACCEPTABLE_MEASUREMENT","weak arm cannot borrow proposed candidate");
  alternative=nominal;alternative.id=1;alternative.pose(0,3)=.2;alternative.loss.fill(0);nominal.loss.fill(.02);
  proposed={nominal,alternative};deduplicate(proposed);pick=coverage::select(proposed,mask);
  require(coverage::paired(alternative.loss,nominal.loss,mask)==.02&&pick.robust==-1,"strict evidence gate");
  nominal.pose(0,3)=.9;nominal.eligible=false;proposed={nominal};decision=decide(proposed,mask,p,Selection{});
  require(!decision.observation_valid&&decision.selected==-1,"rejected nominal cannot return");
  nominal.pose(0,0)=NAN;auto bad=detect(nominal,mask,p,0);
  require(bad.detected&&!bad.permitted&&bad.permission=="BLOCKED_UNSAFE_NATIVE","NaN blocked before generator");
  Budget b;for(int i=0;i<20;++i)require(b.consume(),"budget consumption");require(!b.consume()&&b.used==20,"budget exhaustion");
  require(detect(rejected,mask,p,20).permission=="BUDGET_EXHAUSTED","event budget permission");
  std::cout<<"R11_DECISION_TESTS_PASS new_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
