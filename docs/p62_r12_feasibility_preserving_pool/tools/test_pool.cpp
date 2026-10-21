#include "pool_v3.hpp"
#include <iostream>
using namespace pool_v3;
Mask mask(){Split s;s.training->resize(300);for(auto& b:s.blocks)b->resize(10);return Mask(s);}
Candidate candidate(int id,double x,bool legal,double loss=.1){
  Candidate c;c.id=id;c.pose(0,3)=x;c.finite=true;c.converged=legal;c.loss_valid=true;
  c.score=1;c.fitness=.1;c.overlap=.9;c.loss.fill(loss);return c;
}
int main(){try{
  const auto m=mask();const Pose p=Pose::Identity();int tests=0;
  auto test=[&](bool ok,const char* why){require(ok,why);++tests;};
  auto a=candidate(0,0,true),b=candidate(1,.05,true);
  auto q=construct({a,b},m,p);test(q.groups.size()==1&&q.raw[q.groups[0].representative].id==0,"legal nominal remains representative");
  a.converged=false;q=construct({a,b},m,p);test(q.groups[0].representative==1&&q.raw[1].eligible,"illegal nominal cannot hide feasible duplicate");
  b.converged=false;q=construct({a,b},m,p);test(q.groups[0].representative==0&&!q.raw[0].eligible,"all illegal preserve anchor");
  a.converged=true;b.converged=true;auto c=candidate(2,.08,true);
  q=construct({c,b,a},m,p);test(q.groups[0].representative==0&&q.groups[0].members.size()==3,"multiple legal ID tie rule");
  b.score=100;q=construct({a,b},m,p);test(q.groups[0].representative==0,"native score cannot pick group representative");
  a.converged=false;c.converged=false;q=construct({a,b,c},m,p);test(q.groups[0].representative==1,"only feasible member retained");
  b.pose(0,3)=.09;c.pose(0,3)=.18;c.converged=true;
  q=construct({a,b,c},m,p);test(q.groups.size()==2&&q.group_of[0]==q.group_of[1]&&q.group_of[2]!=q.group_of[1],"no transitive closure or reanchoring on representative promotion");
  a=candidate(0,0,false);b=candidate(1,.05,true);
  q=construct({a,b},m,p);test(q.representatives.size()==1&&select(q,m,p).robust<0,"single representative cannot win vacuously against nominal");
  a.loss.fill(.1);b.loss.fill(.2);q=construct({a,b},m,p);test(select(q,m,p).robust<0,"negative D rejects");
  a.loss.fill(.02);b.loss.fill(0);q=construct({a,b},m,p);test(coverage::paired(b.loss,a.loss,m)==.02&&select(q,m,p).robust<0,"strict equality .02 rejects");
  a.loss.fill(.03);q=construct({a,b},m,p);auto pick=select(q,m,p);auto win=decide(q,m,p,pick);test(pick.robust==1&&win.recovery_success&&q.group_of[win.selected]==q.group_of[0],"D greater than .02 can accept same-group replacement but not new physical branch");
  b.converged=false;q=construct({a,b},m,p);test(!decide(q,m,p,Selection{}).observation_valid,"no invalid map observation");
  b.converged=true;q=construct({a,b},m,p);test(q.raw[0].id==0&&std::find(q.evidence.begin(),q.evidence.end(),0)!=q.evidence.end(),"raw nominal evidence survives promotion");
  const auto reverse=construct({b,a},m,p);test(reverse.groups[0].representative==q.groups[0].representative&&reverse.raw[1].eligible,"input permutation preserves eligibility");
  c=candidate(2,.02,true);c.pose(0,0)=NAN;c.finite=false;q=construct({a,b,c},m,p);test(q.group_of[2]<0&&!q.raw[2].eligible,"nonfinite result retained but excluded safely");
  a.finite=false;a.pose(0,0)=NAN;q=construct({a,b},m,p);test(select(q,m,p).robust<0,"unavailable nominal evidence cannot create automatic victory");
  auto unsafe=candidate(0,0,false);unsafe.finite=true;unsafe.pose(0,0)=NAN;
  q=construct({unsafe,b},m,p);test(!q.raw[0].eligible&&q.group_of[0]<0,"finite metadata cannot hide NaN pose");
  a=candidate(0,0,false,.10);b=candidate(1,.05,true,.05);c=candidate(2,.30,true,.04);
  q=construct({a,b,c},m,p);test(q.representatives.size()==2&&select(q,m,p).robust<0,"must dominate every evidence opponent, not only nominal");
  a=candidate(0,0,false);b=candidate(1,.18,true);c=candidate(2,.09,true);
  q=construct({a,b,c},m,p);test(q.group_of[2]==q.group_of[0]&&q.group_of[1]!=q.group_of[2],"first matching frozen anchor wins");
  b.duplicate_of=0;b.eligible=false;q=construct({a,b},m,p);
  test(q.original[1].duplicate_of==0&&!q.original[1].eligible&&q.raw[1].eligible,"original metadata untouched by derived recomputation");
  for(int count=3;count<=8;++count){Split s;s.training->resize(300);for(int block=0;block<count;++block)s.blocks[block]->resize(10);Mask mm(s);
    a=candidate(0,0,false,.03);b=candidate(1,.05,true,0);
    for(int block=count;block<8;++block){a.loss[block]=NAN;b.loss[block]=NAN;}
    q=construct({a,b},mm,p);test((select(q,mm,p).robust==1)==(count>=4),"B3 refuses, B4to8 frozen evidence supports positive fixture");
  }
  Split deficient;deficient.training->resize(299);for(auto& block:deficient.blocks)block->resize(10);Mask bad(deficient);
  q=construct({a,b},bad,p);test(select(q,bad,p).robust<0&&!decide(q,bad,p,Selection{}).observation_valid,"training299 rejected");
  std::cout<<"R12_POOL_TESTS_PASS cases="<<tests<<" new_ndt_align=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
