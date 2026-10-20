#pragma once
#include "../../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp"
#include <istream>

// Stream adaptation of frozen R7 loadSource: same arithmetic, ordering, PCL voxel/cap/split.
inline shadow::Cloud::Ptr preprocessRaw(std::istream& f,uint32_t n){
  using namespace shadow;
  Cloud::Ptr c(new Cloud);std::array<float,3> xyz;uint32_t offset;
  for(uint32_t i=0;i<n;++i){f.read(reinterpret_cast<char*>(xyz.data()),12);f.read(reinterpret_cast<char*>(&offset),4);require(bool(f),"truncated raw Source");
    if(!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2]))continue;
    const double range=std::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);
    if(range>=.5&&range<=80)c->push_back(pcl::PointXYZ(xyz[0],xyz[1],xyz[2]));}
  finish(c);return voxel(c,.25f,1400);
}
