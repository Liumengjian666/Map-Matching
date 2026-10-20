#include "coverage.hpp"
#include <iostream>
using namespace shadow;
int main(){try{
  for(int n=0;n<=8;++n){Split s;s.training->resize(300);for(int b=0;b<n;++b)s.blocks[b]->resize(10);coverage::Mask m(s);
    require(m.active.size()==size_t(n)&&m.evaluable()==(n>=4),"count-only mask");
    Loss j,k;j.fill(NAN);k.fill(NAN);for(int b=0;b<n;++b){j[b]=.1*(b%4);k[b]=.1*((b+2)%5);}
    if(n<4){bool rejected=false;try{coverage::paired(j,k,m);}catch(...){rejected=true;}require(rejected,"insufficient coverage rejected");continue;}
    if(n==8)require(coverage::paired(j,k,m)==shadow::paired(j,k),"strict B8 V1 parity");
    // Independent capped-simplex vertex enumeration: full caps and one residual.
    const int full=n/2;const double cap=2./n,rem=1.-full*cap;double oracle=1e9;
    for(unsigned bits=0;bits<(1U<<n);++bits)if(__builtin_popcount(bits)==full){double score=0;for(int b=0;b<n;++b)if(bits&(1U<<b))score+=cap*(k[b]-j[b]);
      if(n%2){for(int b=0;b<n;++b)if(!(bits&(1U<<b)))oracle=std::min(oracle,score+rem*(k[b]-j[b]));}
      else oracle=std::min(oracle,score);}
    require(std::abs(coverage::paired(j,k,m)-oracle)<1e-14,"independent common-weight vertices B4..8");
    std::vector<Candidate> c(2);for(auto& a:c){a.finite=a.eligible=a.loss_valid=true;a.loss.fill(NAN);for(int b:m.active)a.loss[b]=.5;}
    for(int b:m.active)c[1].loss[b]=.1;c[1].id=1;c[1].score=1;
    require(coverage::select(c,m).robust==1,"positive supported alternative");c[1].loss=c[0].loss;require(coverage::select(c,m).robust<0,"tie abstains");
    c[0].finite=false;require(coverage::select(c,m).robust<0,"singleton alternative cannot win vacuously");
  }
  Split s;s.training->resize(299);for(auto& b:s.blocks)b->resize(10);coverage::Mask m(s);require(!m.evaluable(),"training still300");
  Vector6 v;v<<.2,-.3,.4,.1,.2,.3;
  for(bool large:{false,true})for(double sign:{1.,-1.}){auto p=coverage::scaledSeed(Pose::Identity(),v,sign,large);
    require(translation(p,Pose::Identity())<=(large?1.2:.30)+1e-12&&rotation(p,Pose::Identity())<=(large?8:3)+1e-9,"two physical seed limits");}
  require((coverage::scaledSeed(Pose::Identity(),v,1,false)-seed(Pose::Identity(),v,1)).norm()==0,"SMALL exact V1 seed");
  Budget budget;for(int i=0;i<20;++i)require(budget.consume(),"token");require(!budget.consume(),"shared budget exhaustion");
  Split sparse;sparse.training->resize(300);for(int b:{1,3,4,7})for(int i=0;i<10;++i)sparse.blocks[b]->push_back(pcl::PointXYZ(0,0,0));sparse.blocks[0]->resize(9);
  coverage::Mask sparse_mask(sparse);require(sparse_mask.active==std::vector<int>({1,3,4,7}),"noncontiguous mask, under10 excluded");
  Cloud::Ptr map(new Cloud);map->push_back(pcl::PointXYZ(0,0,0));pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);
  Candidate a;a.finite=a.converged=true;a.pose(0,3)=.25;coverage::validate(a,sparse,sparse_mask,tree,Pose::Identity());
  require(a.eligible&&coverage::mean(a.loss,sparse_mask)==.25&&a.overlap==1&&std::isnan(a.loss[0]),"active NN and missing losses");
  Split complete;complete.training->resize(300);for(auto& block:complete.blocks)for(int i=0;i<10;++i)block->push_back(pcl::PointXYZ(0,0,0));coverage::Mask all(complete);
  Candidate old=a,modern=a;old.loss.fill(NAN);modern.loss.fill(NAN);shadow::validate(old,complete,tree,Pose::Identity());coverage::validate(modern,complete,all,tree,Pose::Identity());
  require(old.loss==modern.loss&&old.overlap==modern.overlap&&old.eligible==modern.eligible,"strict B8 NN-validation parity");
  unsigned random=19467;for(int trial=0;trial<1000;++trial){Loss x,y;for(int b=0;b<8;++b){random=1664525U*random+1013904223U;x[b]=double(random)/UINT32_MAX;random=1664525U*random+1013904223U;y[b]=double(random)/UINT32_MAX;}
    require(coverage::paired(x,y,all)==shadow::paired(x,y)&&coverage::mean(x,all)==shadow::mean(x)&&coverage::median(x,all)==shadow::median(x),"1000 exact B8 math fixtures");}
  old.loss[1]=NAN;require(!all.valid(old.loss),"nonfinite active loss rejection");
  std::cout<<"V2_MATH_TESTS_PASS B0..8 strict_B8 independent_vertices seeds abstention budget new_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
