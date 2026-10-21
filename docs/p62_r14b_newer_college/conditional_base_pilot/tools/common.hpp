#pragma once
#include "../../../p62_r13_training_guided_multihypothesis/tools/coarse.hpp"
#include "../../../p62_r9_coverage_aware_recovery_v2/tools/io.hpp"
#include "../../../p62_r10_feasibility_event_recovery/tools/native_capture.hpp"
#include "../../../p62_r12_feasibility_preserving_pool/tools/pool_v3.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <sys/resource.h>

namespace pilot {
using namespace coverage;
using namespace feasibility;
inline Pose readMatrix(const std::string& path){
  std::ifstream f(path);Pose p;
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)require(bool(f>>p(i,j)),"anchor matrix missing");
  require(rigid(p),"invalid anchor SE3");return p;
}
inline long peakKiB(){struct rusage u;getrusage(RUSAGE_SELF,&u);return u.ru_maxrss;}
// Right/body constant velocity using only past admitted CONTROL_FULL results.
// Any rejection disables extrapolation until the next admitted full result.
struct Predictor {
  struct State {int64_t ns;Pose pose;};
  std::vector<State> history;bool hold=false;
  Predictor(int64_t ns,const Pose& anchor){require(rigid(anchor),"bad anchor");history.push_back({ns,anchor});}
  Pose predict(int64_t ns)const{
    require(ns>history.back().ns,"prediction is not future of completed state");
    if(hold||history.size()<2)return history.back().pose;
    const auto& b=history.back();const auto& a=history[history.size()-2];
    require(b.ns>a.ns,"invalid causal history dt");
    const double scale=double(ns-b.ns)/double(b.ns-a.ns);
    const Pose p=b.pose*cause::expSE3(scale*cause::logSE3(a.pose.inverse()*b.pose));
    require(rigid(p),"invalid causal prediction");return p;
  }
  void completed(int64_t ns,const Candidate& full,const Mask& mask,const Pose& tp){
    require(ns>history.back().ns,"noncausal update");
    if(admissible(full,mask,tp)){
      history.push_back({ns,full.pose});if(history.size()>2)history.erase(history.begin());hold=false;
    }else hold=true;
  }
};
struct Source {
  Cloud::Ptr points;Split split;Mask mask;
  double load_ms=0,preprocess_ms=0;size_t declared_points=0;
  Source():mask(split){}
};
inline Source loadSource(const Row& r){
  Source s;const auto begin=Clock::now();pcl::PCLPointCloud2 blob;
  require(pcl::io::loadPCDFile(r.at("path"),blob)==0,"PCD_FORMAT_BLOCKED native reader");
  const size_t n=std::stoul(r.at("points")),step=std::stoul(r.at("point_step"));
  require(n==size_t(blob.width)*blob.height&&blob.point_step==step&&step==29&&!blob.is_bigendian,"PCD_FORMAT_BLOCKED shape/endian");
  require(blob.data.size()==n*step&&blob.fields.size()==9,"native reader must expose declared records only");
  const std::array<std::string,9> names{{"x","y","z","intensity","t","reflectivity","ring","noise","range"}};
  const std::array<int,9> offsets{{0,4,8,12,16,20,22,23,25}};
  const std::array<int,9> types{{7,7,7,7,6,4,2,4,6}};
  for(int i=0;i<9;++i)require(blob.fields[i].name==names[i]&&blob.fields[i].offset==offsets[i]&&blob.fields[i].datatype==types[i]&&blob.fields[i].count==1,"PCL field layout changed");
  std::ifstream original(r.at("path"),std::ios::binary);original.seekg(std::stoul(r.at("header_bytes")));
  std::vector<uint8_t> declared(n*step);original.read(reinterpret_cast<char*>(declared.data()),declared.size());
  require(bool(original)&&declared==blob.data,"native point records differ from original declared bytes");
  Cloud decoded;pcl::fromPCLPointCloud2(blob,decoded);require(decoded.size()==n,"native XYZ count mismatch");
  std::stringstream raw(std::ios::in|std::ios::out|std::ios::binary);
  for(size_t i=0;i<n;++i){
    const auto* p=declared.data()+i*step;const auto& q=decoded[i];
    require(std::memcmp(p,&q.x,4)==0&&std::memcmp(p+4,&q.y,4)==0&&std::memcmp(p+8,&q.z,4)==0,"native XYZ value/bit mismatch");
    raw.write(reinterpret_cast<const char*>(p),12);raw.write(reinterpret_cast<const char*>(p+16),4);
  }
  s.load_ms=milliseconds(begin);const auto pre=Clock::now();raw.seekg(0);
  s.points=preprocessRaw(raw,uint32_t(n));s.split=splitSource(s.points);s.mask=Mask(s.split);
  require(!s.points->empty()&&!s.split.training->empty(),"empty localization Source");
  s.preprocess_ms=milliseconds(pre);s.declared_points=n;return s;
}
} // namespace pilot
