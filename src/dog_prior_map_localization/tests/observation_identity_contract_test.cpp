#include "dog_prior_map_localization/observation_identity.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace dog_prior_map_localization;
using Pose=Eigen::Matrix4d;
bool same(const Pose&a,const Pose&b){return (a.array()==b.array()).all();}
int main(int argc,char**argv){
 const auto strict=observationIdentityPolicy("STRICT_FAIL_CLOSED");
 const auto legacy=observationIdentityPolicy("LEGACY");
 Pose previous=Pose::Identity();int tests=0;
 auto check=[&](const char* name,Pose raw,bool admitted,bool converged,bool have_previous,bool expected){
  Pose proposed=raw;bool t=false,r=false;const Pose history_before=previous;
  const bool changed=limitObservationStep(raw,previous,true,have_previous,.5,5.,proposed,t,r);
  const auto decision=decideObservationIdentity(strict,admitted,converged,raw,proposed,changed);
  if(decision.publish!=expected || !same(previous,history_before) ||
      (decision.publish && !same(decision.pose,raw)))throw std::runtime_error(name);
  if(admitted&&converged){const auto old=decideObservationIdentity(legacy,true,true,raw,proposed,changed);
   if(!old.publish||!same(old.pose,proposed))throw std::runtime_error("LEGACY changed");}
  std::cout<<name<<",PASS,"<<decision.publish<<','<<decision.reason<<'\n';++tests;
 };
 Pose raw=Pose::Identity();raw(0,3)=.2;check("UNCHANGED",raw,true,true,true,true);
 raw(0,3)=.8;check("TRANSLATION_ONLY",raw,true,true,true,false);
 raw=Pose::Identity();raw.block<3,3>(0,0)=Eigen::AngleAxisd(8*M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();check("ROTATION_ONLY",raw,true,true,true,false);
 raw(0,3)=.8;check("BOTH",raw,true,true,true,false);
 check("QUALITY_REJECTED",Pose::Identity(),false,true,true,false);
 check("NOT_CONVERGED",Pose::Identity(),true,false,true,false);
 check("NO_PREVIOUS_OBSERVATION",raw,true,true,false,true);
 const auto refused=decideObservationIdentity(strict,true,true,raw,Pose::Identity(),true);
 if(refused.publish||!same(previous,Pose::Identity()))return 2;
 std::cout<<"REJECT_HISTORY_UNCHANGED,PASS\n";++tests;
 if(argc==4){
  Pose selected;std::ifstream prev(argv[1]),cand(argv[2]);
  for(int i=0;i<4;++i)for(int j=0;j<4;++j){if(!(prev>>previous(i,j))||!(cand>>selected(i,j)))return 3;}
  Pose proposed=selected;bool t=false,r=false;bool changed=limitObservationStep(selected,previous,true,true,.5,5.,proposed,t,r);
  auto old=decideObservationIdentity(legacy,true,true,selected,proposed,changed);
  auto closed=decideObservationIdentity(strict,true,true,selected,proposed,changed);
  if(!old.publish||closed.publish||!changed)return 4;
  std::ofstream out(argv[3]);out<<std::setprecision(17);
  out<<"LEGACY_publish "<<old.publish<<" STRICT_publish "<<closed.publish<<" delta_m "<<(selected.block<3,1>(0,3)-proposed.block<3,1>(0,3)).norm()<<'\n';
  for(const Pose &pose:{selected,proposed,previous})out<<pose<<'\n';
 }
 std::cout<<"OBSERVATION_IDENTITY_CONTRACT_PASS tests="<<tests<<" native_align_calls=0\n";
}
