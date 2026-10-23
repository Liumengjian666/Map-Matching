#include "dispatch.hpp"
#include <iostream>
using namespace r22;
int main(){
 Split s;for(int i=0;i<320;++i)s.training->push_back(pcl::PointXYZ());for(auto& b:s.blocks)for(int i=0;i<10;++i)b->push_back(pcl::PointXYZ());Mask mask(s);
 Pose tp=Pose::Identity();Candidate n;n.id=0;n.pose=tp;n.finite=n.converged=n.loss_valid=true;n.score=1;n.fitness=.1;n.overlap=.6;n.loss.fill(.5);
 Candidate c=n;c.id=1;c.pose(0,3)=.2;c.score=1.1;c.overlap=.7;c.loss.fill(.4);
 std::vector<Candidate> cs{n,c};
 require(select(Policy::Native,cs,mask,tp)==0,"A uses basic only");
 require(select(Policy::Gate,{n},mask,tp)==-1&&select(Policy::Multi,cs,mask,tp)==-1,"B/C absolute rejection");
 require(select(Policy::Relative,cs,mask,tp)==1,"D frozen relative admission");
 require(select(Policy::Relative,cs,mask,tp)==r21::chooseRelative(cs,mask,tp),"exact R21 selector");
 c.score=n.score;require(select(Policy::Relative,{n,c},mask,tp)==-1,"strict score unchanged");c.score=1.1;
 c.loss.fill(.2);cs={n,c};require(select(Policy::Multi,cs,mask,tp)==r20::choose(cs,mask,tp)&&select(Policy::Relative,cs,mask,tp)==1,"absolute priority");
 n.loss.fill(.1);require(!r20::maySearch(true,!accepted(n,mask),0),"normal frame no extra");
 require(!searchPolicy(Policy::Native)&&!searchPolicy(Policy::Gate)&&searchPolicy(Policy::Multi)&&searchPolicy(Policy::Relative),"only C/D search");
 require(r20::maySearch(true,true,10)&&!r20::maySearch(true,true,12),"budget unchanged");
 Predictor a(100,tp),b(100,tp);r17::advance(a,200,c,true);r17::advance(b,200,n,false);
 require(a.history.back().ns==200&&b.history.back().ns==100&&b.hold,"independent causal state");
 require((b.predict(300)-tp).norm()==0,"fallback is last accepted history");
 require(translation(yawSeed(tp,1),tp)==0&&std::abs(rotation(yawSeed(tp,-1),tp)-5)<1e-8,"old yaw rule");
 std::cout<<"R22_DISPATCH_PASS: exact frozen selectors, normal priority, independent histories, budget; NDT=0\n";
}
