// Count the frozen Coverage V2 requirements on the same eight Sources. No align().
#include "../../p62_r9_coverage_aware_recovery_v2/tools/coverage.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
using namespace shadow;
int main(int argc,char**argv){
 if(argc!=2)return 2;std::ifstream requests(argv[1]);std::string row;
 std::cout<<"raw,source_points,source_fnv,training_points,active_blocks,mask_evaluable,block_counts\n";
 while(std::getline(requests,row)){
  std::stringstream s(row);std::string raw,path,unused,expected;
  std::getline(s,raw,'\t');std::getline(s,path,'\t');std::getline(s,unused,'\t');std::getline(s,expected,'\t');
  Cloud cloud;require(pcl::io::loadPCDFile(path,cloud)==0,"PCD load failed");Cloud::Ptr filtered(new Cloud);
  for(const auto&p:cloud){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;const double r=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);if(r>=.5&&r<=80)filtered->push_back(p);}
  finish(filtered);auto source=voxel(filtered,.25f,1400);require(cloudHash(source)==expected,"Source differs from R29");
  auto split=splitSource(source);coverage::Mask mask(split);
  std::cout<<raw<<','<<source->size()<<','<<cloudHash(source)<<','<<split.training->size()<<','<<mask.active.size()<<','<<mask.evaluable()<<',';
  for(int i=0;i<8;++i)std::cout<<(i?";":"")<<split.blocks[i]->size();std::cout<<'\n';
 }
}
