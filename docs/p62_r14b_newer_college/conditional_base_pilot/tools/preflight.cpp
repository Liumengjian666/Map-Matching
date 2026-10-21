#include "common.hpp"
#include <iostream>
using namespace pilot;
int main(int argc,char** argv){try{
  require(argc==6,"preflight SCANS ANCHOR OFFICIAL_PLY NEW_MAP_PCD NEW_RECEIPT_DIR");
  require(::mkdir(argv[5],0775)==0,"exclusive native preflight output");
  std::ifstream existing(argv[4]);require(!existing.good(),"refuse map overwrite");
  const std::string out=argv[5];const auto rows=readCsv(argv[1]);require(rows.size()==21,"fixed21");
  Csv records(out+"/PCL_READ_TEST.csv","local_index,stamp_ns,declared_points,point_step,declared_byte_equivalence,xyz_bit_equivalence,source_points,source_fnv,training_points,B,val0,val1,val2,val3,val4,val5,val6,val7,load_ms,preprocess_ms,peak_rss_KiB");
  std::vector<Source> sources;
  for(int i=0;i<21;++i){require(std::stoi(rows[i].at("local_index"))==i,"21 input order");
    auto s=pilot::loadSource(rows[i]);records.stream<<i<<','<<rows[i].at("stamp_ns")<<','<<s.declared_points<<",29,1,1,"<<s.points->size()<<','<<cloudHash(s.points)<<','<<s.split.training->size()<<','<<s.mask.active.size();
    for(auto& b:s.split.blocks)records.stream<<','<<b->size();records.stream<<','<<s.load_ms<<','<<s.preprocess_ms<<','<<peakKiB();records.end();sources.push_back(s);
  }
  const Pose anchor=readMatrix(argv[2]);const Eigen::Vector3d center=anchor.block<3,1>(0,3);
  const auto start=Clock::now();Cloud::Ptr all(new Cloud),roi(new Cloud);
  require(pcl::io::loadPLYFile<pcl::PointXYZ>(argv[3],*all)==0,"official Leica PLY unreadable");
  size_t finite=0;
  for(const auto& p:*all){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;++finite;
    if((Eigen::Vector3d(p.x,p.y,p.z)-center).squaredNorm()<=2500.)roi->push_back(p);}
  finish(roi);require(!roi->empty(),"MAP_COVERAGE_BLOCKED empty fixed50m ROI");
  auto target=voxel(roi,.20f);require(!target->empty(),"MAP_COVERAGE_BLOCKED empty voxel map");
  const double map_ms=milliseconds(start);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  size_t hit=0;std::vector<int> ids(1);std::vector<float> squared(1);
  for(const auto& p:*sources[0].points){const Eigen::Vector3d y=anchor.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+center;
    require(tree.nearestKSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),1,ids,squared)==1,"anchor NN unavailable");hit+=squared[0]<=.25f;}
  const double overlap=double(hit)/sources[0].points->size();
  Csv map(out+"/MAP_BUILD.csv","raw_points,finite_points,roi_points,target_points,target_fnv,leaf_m,roi_radius_m,center_x,center_y,center_z,anchor_overlap_0p5m,build_ms,peak_rss_KiB");
  map.stream<<all->size()<<','<<finite<<','<<roi->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.2,50,"<<center.x()<<','<<center.y()<<','<<center.z()<<','<<overlap<<','<<map_ms<<','<<peakKiB();map.end();
  require(overlap>=.50,"MAP_COVERAGE_BLOCKED anchor support below frozen0.50 admission scale");
  require(pcl::io::savePCDFileBinary(argv[4],*target)==0,"map save failed");
  std::cout<<"PREFLIGHT_PASS originals=21 canonical_copies=0 new_ndt_align=0 map_points="<<target->size()<<" anchor_overlap="<<overlap<<std::endl;
}catch(const std::exception& e){std::cerr<<"PREFLIGHT_STOP "<<e.what()<<std::endl;return 1;}}
