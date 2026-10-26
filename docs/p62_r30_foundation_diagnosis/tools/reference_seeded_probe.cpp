// R30 bounded diagnosis: same frozen Source/target and native R6 PCL NDT.
#include "../../p62_r6_dog_shadow_prototype/tools/native_kernel.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <sys/resource.h>
using namespace shadow;
int main(int argc,char**argv){
 if(argc!=4)return 2;
 try {
  Cloud::Ptr target(new Cloud);std::ifstream files(argv[1]);std::string path;
  while(std::getline(files,path)){Cloud tile;require(pcl::io::loadPCDFile(path,tile)==0,"target PCD failed");*target+=tile;}
  finish(target);const auto target_hash=cloudHash(target);
  require(target->size()==93146 && std::stoull(target_hash,nullptr,16)==4624378881076069786ULL,"target identity differs from R29");
  NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  std::ifstream requests(argv[2]);std::ofstream out(argv[3]);require(bool(out),"output unavailable");
  out<<"raw,source_points,source_fnv,target_points,target_fnv,converged,finite,iterations,score,fitness,align_ms,reference_translation_m,reference_rotation_deg,all_source_overlap_0p5,peak_rss_KiB\n"<<std::setprecision(17);
  std::string line;int calls=0;
  while(std::getline(requests,line)){
   require(calls<8,"8-call diagnostic budget exceeded");
   std::stringstream row(line);std::string raw,pcd,initial,expected;
   require(bool(std::getline(row,raw,'\t'))&&bool(std::getline(row,pcd,'\t'))&&bool(std::getline(row,initial,'\t'))&&bool(std::getline(row,expected,'\t')),"request malformed");
   Cloud cloud;require(pcl::io::loadPCDFile(pcd,cloud)==0,"source PCD failed");Cloud::Ptr filtered(new Cloud);
   for(const auto& p:cloud){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;const double r=std::sqrt(p.x*p.x+p.y*p.y+p.z*p.z);if(r>=.5 && r<=80)filtered->push_back(p);}
   finish(filtered);auto source=voxel(filtered,.25f,1400);require(cloudHash(source)==expected,"source FNV differs from R29");
   std::ifstream pose_file(initial);Pose seed;for(int i=0;i<4;++i)for(int j=0;j<4;++j)require(bool(pose_file>>seed(i,j)),"reference matrix incomplete");
   ++calls;const auto candidate=align(ndt,source,seed,0);
   size_t hits=0;std::vector<int> ids(1);std::vector<float> distances(1);
   for(const auto& p:*source){const auto y=(candidate.pose.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+candidate.pose.block<3,1>(0,3)).eval();pcl::PointXYZ q(y.x(),y.y(),y.z());if(tree.nearestKSearch(q,1,ids,distances)==1&&distances[0]<=.25f)++hits;}
   struct rusage usage;getrusage(RUSAGE_SELF,&usage);
   out<<raw<<','<<source->size()<<','<<cloudHash(source)<<','<<target->size()<<','<<target_hash<<','<<candidate.converged<<','<<candidate.finite<<','<<candidate.iterations<<','<<candidate.score<<','<<candidate.fitness<<','<<candidate.align_ms<<','<<translation(candidate.pose,seed)<<','<<rotation(candidate.pose,seed)<<','<<double(hits)/source->size()<<','<<usage.ru_maxrss<<'\n';out.flush();
   std::cerr<<"NATIVE_ALIGN_COMPLETED raw="<<raw<<" call="<<calls<<'\n';
  }
  require(calls==8,"incomplete diagnostic request list");
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
