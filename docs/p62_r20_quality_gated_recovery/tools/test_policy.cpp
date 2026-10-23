#include "policy.hpp"
#include <iostream>
using namespace r20;
int main(){
  Split s;for(int i=0;i<320;++i)s.training->push_back(pcl::PointXYZ());
  for(auto& b:s.blocks)for(int i=0;i<10;++i)b->push_back(pcl::PointXYZ());
  Mask mask(s);Pose tp=Pose::Identity();tp(0,3)=2.;tp(1,3)=3.;
  tp.block<3,3>(0,0)=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto plus=yawSeed(tp,1),minus=yawSeed(tp,-1);
  require(translation(plus,tp)==0&&translation(minus,tp)==0,"seed translation changed");
  require((plus.block<3,3>(0,0)-Eigen::AngleAxisd(5*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix()*tp.block<3,3>(0,0)).norm()<1e-12,"map yaw order");
  Candidate c;c.id=0;c.pose=tp;c.score=1;c.fitness=.1;c.finite=c.converged=c.loss_valid=true;c.overlap=.8;c.loss.fill(.29);
  require(accepted(c,mask),"valid candidate rejected");c.loss.fill(.31);require(!accepted(c,mask),"quality loss failed open");
  c.loss.fill(0);c.loss[0]=.30*8; // invalid block is rejected, not averaged away
  require(!accepted(c,mask),"invalid block passed");c.loss.fill(.1);
  c.converged=false;require(!accepted(c,mask),"nonconvergence passed");c.converged=true;
  c.pose(0,0)=NAN;require(!accepted(c,mask),"nonfinite pose passed");c.pose=tp;
  Candidate d=c;d.id=1;d.score=2;d.pose=plus;
  require(choose({c,d},mask,tp)==1,"native maximum not selected");
  d.loss.fill(.31);require(choose({c,d},mask,tp)==0,"bad high-score candidate selected");
  d.loss.fill(.1);d.score=c.score;require(choose({d,c},mask,tp)==1,"TP distance tie break");
  d.pose=c.pose;require(choose({d,c},mask,tp)==1,"ID tie break");
  c.loss.fill(.31);d.loss.fill(.31);require(choose({c,d},mask,tp)==-1,"empty legal pool did not fallback");
  require(!maySearch(false,true,0)&&!maySearch(true,false,0)&&maySearch(true,true,10)&&!maySearch(true,true,12),"budget guard");
  c.pose=Pose::Identity();Predictor a(100,c.pose),b(100,c.pose);c.pose(0,3)=1;
  r17::advance(a,200,c,true);require(std::abs(a.predict(300)(0,3)-2)<1e-12,"causal velocity");
  r17::advance(a,300,c,false);require(a.history.back().ns==200&&std::abs(a.predict(400)(0,3)-1)<1e-12,"rejected pose leaked");
  require(b.history.size()==1&&b.predict(400)(0,3)==0,"cross-policy state leaked");
  std::cout<<"R20_POLICY_TEST_PASS: yaw, quality, ranking, no-candidate, budget, causal fallback, isolation\n";
}
