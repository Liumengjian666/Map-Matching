#include "policy.hpp"
#include <iostream>
using namespace r21;
int main(){
  Split s;for(int i=0;i<320;++i)s.training->push_back(pcl::PointXYZ());
  for(auto& b:s.blocks)for(int i=0;i<10;++i)b->push_back(pcl::PointXYZ());Mask mask(s);
  Pose tp=Pose::Identity();Candidate n;n.id=0;n.pose=tp;n.finite=n.converged=n.loss_valid=true;n.score=1;n.fitness=.1;n.overlap=.6;n.loss.fill(.5);
  Candidate c=n;c.id=1;c.pose(0,3)=.2;c.score=1.1;c.overlap=.7;c.loss.fill(.4);
  require(dominates(c,n,mask)&&!accepted(c,mask),"relative-only case");
  Candidate x=c;x.score=n.score;require(!dominates(x,n,mask),"strict score");
  x=c;x.overlap=n.overlap;require(!dominates(x,n,mask),"strict overlap");
  x=c;x.loss=n.loss;require(!dominates(x,n,mask),"strict loss");
  x=c;x.pose=n.pose;require(!dominates(x,n,mask),"physical duplicate");
  x=c;x.converged=false;require(!dominates(x,n,mask),"nonconverged relative");
  Candidate bad=n;bad.loss[0]=NAN;require(!dominates(c,bad,mask),"invalid comparator");
  bad=n;bad.converged=false;require(dominates(c,bad,mask),"finite nonconverged comparator allowed");
  Candidate d=c;d.id=2;d.pose(0,3)=.3;require(chooseRelative({n,d,c},mask,tp)==2,"relative tie distance");
  d.pose=c.pose;require(chooseRelative({n,d,c},mask,tp)==2,"relative tie ID");
  Pending pending;Predictor trusted(100,tp);const auto old=trusted.history;
  pending.establish(1,200,c);require(trusted.history.size()==old.size()&&trusted.history.back().ns==100,"pending leaked into trusted history");
  r17::advance(trusted,200,n,false);require(trusted.history.back().ns==100&&trusted.hold,"creation must fallback");
  require((pending.seed(2,300)-c.pose).norm()==0,"single-pose shadow seed");
  Candidate confirmation=c;confirmation.pose(0,3)=.25;confirmation.id=3;
  require(dominates(confirmation,n,mask),"second-frame relative confirmation");
  r17::advance(trusted,300,confirmation,true);pending.clear();
  require(trusted.history.size()==2&&trusted.history.back().ns==300&&trusted.history[0].ns==100&&!pending.active,"confirmation must append current pose only");
  require(canSearch(10)&&!canSearch(11)&&canConfirm(11)&&!canConfirm(12),"search/confirmation share budget");
  n.loss.fill(.1);require(accepted(n,mask),"nominal absolute priority");
  std::cout<<"R21_POLICY_TEST_PASS: strict Pareto, nominal comparability, distinctness, ranking, causal pending isolation, one-frame expiry, shared budget\n";
}
