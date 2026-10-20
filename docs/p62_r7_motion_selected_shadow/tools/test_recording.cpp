#include "recovery.hpp"
#include <iostream>
using namespace shadow;
int main(){try{
  Candidate c;c.finite=c.loss_valid=c.eligible=true;c.loss.fill(.2);
  std::vector<Candidate> a{c};require(std::isnan(selectedMargin(a,0)),"singleton margin must be NA");
  require(std::isnan(selectedMargin(a,-1)),"abstention margin must be NA");
  c.loss.fill(.5);c.id=1;a.push_back(c);
  require(std::abs(selectedMargin(a,0)-.3)<1e-14,"paired recording value unchanged");
  require(std::abs(selectedMargin(a,1)+.3)<1e-14,"negative paired advantage preserved");
  Budget budget;for(int i=0;i<20;++i)require(budget.consume(),"combined budget token");
  NativeNdt unused;Split split;pcl::KdTreeFLANN<pcl::PointXYZ> tree;bool called=false;
  auto result=recover(unused,split,tree,Pose::Identity(),a,budget,[&](const Cloud::Ptr&,const Pose&,int,const char*){called=true;return Candidate();});
  require(!called&&result.direction=="BUDGET_EXHAUSTED"&&result.extra==0,"budget guard before diagnostic G/align");
  std::cout<<"R7_RECORDING_AND_COMBINED_BUDGET_PASS native_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
