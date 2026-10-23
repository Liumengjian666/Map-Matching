#include "policy.hpp"
#include <iostream>
using namespace r17;
int main(){
  Split s;for(int i=0;i<320;++i)s.training->push_back(pcl::PointXYZ());
  for(auto& block:s.blocks)for(int i=0;i<10;++i)block->push_back(pcl::PointXYZ());
  Mask mask(s);require(mask.evaluable(),"test mask");
  Pose tp=Pose::Identity();Candidate full;full.finite=full.converged=full.loss_valid=true;full.score=1;full.fitness=1;full.overlap=.8;
  full.loss.fill(.1);full.pose=Pose::Identity();full.pose.block<3,3>(0,0)=Eigen::AngleAxisd(5.5*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(!base(full,mask,tp,5.)&&base(full,mask,tp,6.)&&needsConfirmation(full,mask,tp),"5/6 degree event");
  Candidate split=full;split.pose.block<3,1>(0,3).x()=.04;
  require(confirmed(full,split,mask),"near pair");
  split.pose.block<3,1>(0,3).x()=.11;require(!confirmed(full,split,mask),"far pair");
  full.pose=Pose::Identity();full.pose.block<3,3>(0,0)=Eigen::AngleAxisd(6.01*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(!needsConfirmation(full,mask,tp)&&!base(full,mask,tp,6.),"outside event ceiling");
  full.pose=Pose::Identity();full.pose.block<3,3>(0,0)=Eigen::AngleAxisd(5*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  require(base(full,mask,tp,5.)&&!needsConfirmation(full,mask,tp),"5 degree boundary");
  Predictor p(100,tp);Pose p1=tp;p1(0,3)=1;full.pose=p1;advance(p,200,full,true);
  require(translation(p.predict(300),p1)>0.9,"constant velocity prediction");
  advance(p,300,full,false);require(translation(p.predict(400),p1)<1e-12,"rejection hold");
  std::cout<<"R17_POLICY_TEST_PASS"<<std::endl;
}
