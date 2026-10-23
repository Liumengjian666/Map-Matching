#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include <sys/stat.h>
#include <iostream>

using namespace pilot;

static size_t inspectTail(const Row& row) {
  const std::string path=row.at("path");
  std::ifstream input(path,std::ios::binary);
  require(bool(input),"selected PCD cannot be opened");
  const size_t header=std::stoul(row.at("header_bytes"));
  const size_t points=std::stoul(row.at("points"));
  const size_t step=std::stoul(row.at("point_step"));
  input.seekg(0,std::ios::end);const auto end=input.tellg();
  require(end>=0,"PCD size unavailable");
  const size_t bytes=size_t(end);
  require(bytes==std::stoul(row.at("pcd_bytes")),"PCD byte size differs from preregistration");
  require(bytes>=header+points*step,"PCD shorter than declared binary records");
  input.seekg(0);std::string text(header,'\0');input.read(&text[0],header);
  require(bool(input),"short PCD header");
  for(const std::string token:{"VERSION 0.7\n","FIELDS x y z intensity t reflectivity ring noise range\n",
      "SIZE 4 4 4 4 4 2 1 2 4\n","TYPE F F F F U U U U U\n",
      "COUNT 1 1 1 1 1 1 1 1 1\n","WIDTH 1024\n","HEIGHT 64\n",
      "POINTS 65536\n","DATA binary\n"})
    require(text.find(token)!=std::string::npos,"PCD header contract mismatch");
  const size_t tail=bytes-header-points*step;
  input.seekg(header+points*step);std::array<char,8192> buf{};size_t left=tail;
  while(left){const size_t n=std::min(left,buf.size());input.read(buf.data(),n);
    require(size_t(input.gcount())==n,"short PCD tail");
    for(size_t i=0;i<n;++i)require(buf[i]==0,"PCD trailing bytes are not zero padding");left-=n;}
  return tail;
}

int main(int argc,char** argv){try{
  require(argc==6,"prepare_map SCANS ANCHOR OFFICIAL_PLY NEW_MAP_PCD NEW_RECEIPT_DIR");
  require(::mkdir(argv[5],0775)==0,"exclusive map-preflight directory, no overwrite");
  {std::ifstream exists(argv[4]);require(!exists.good(),"refuse map overwrite");}
  const std::string out=argv[5];const auto rows=readCsv(argv[1]);require(rows.size()==61,"frozen 61 scans required");
  Csv pcd(out+"/PCD_FORMAT_AND_SOURCE_TEST.csv",
      "local_index,raw_ordinal,stamp_ns,pcd_bytes,declared_records_bytes,zero_tail_bytes,pcd_header_ok,native_pcl_ok,source_points,source_fnv,training_points,active_blocks,load_ms,preprocess_ms,peak_rss_KiB");
  std::vector<Source> sources;sources.reserve(rows.size());
  for(size_t i=0;i<rows.size();++i){const auto& row=rows[i];
    require(std::stoul(row.at("local_index"))==i,"frozen local scan order mismatch");
    const size_t tail=inspectTail(row);
    auto s=pilot::loadSource(row);
    require(s.points->size()==std::stoul(row.at("expected_source_points"))&&
            cloudHash(s.points)==row.at("expected_source_fnv"),"preprocessed Source identity differs from frozen prescreen");
    pcd.stream<<i<<','<<row.at("raw_ordinal")<<','<<row.at("stamp_ns")<<','<<row.at("pcd_bytes")<<','
      <<(size_t(std::stoul(row.at("header_bytes")))+size_t(std::stoul(row.at("points")))*size_t(std::stoul(row.at("point_step"))))
      <<','<<tail<<",1,1,"<<s.points->size()<<','<<cloudHash(s.points)<<','<<s.split.training->size()<<','<<s.mask.active.size()
      <<','<<s.load_ms<<','<<s.preprocess_ms<<','<<peakKiB();pcd.end();
    sources.push_back(std::move(s));
  }
  const Pose anchor=readMatrix(argv[2]);const Eigen::Vector3d center=anchor.block<3,1>(0,3);
  const auto start=Clock::now();Cloud::Ptr all(new Cloud),roi(new Cloud);
  require(pcl::io::loadPLYFile<pcl::PointXYZ>(argv[3],*all)==0,"official Leica PLY unreadable");
  size_t finite=0;
  for(const auto& p:*all){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;++finite;
    if((Eigen::Vector3d(p.x,p.y,p.z)-center).squaredNorm()<=2500.)roi->push_back(p);}
  finish(roi);require(!roi->empty(),"MAP_COVERAGE_BLOCKED empty fixed50m ROI");
  const size_t roi_points=roi->size();auto target=voxel(roi,.20f);require(!target->empty(),"MAP_COVERAGE_BLOCKED empty voxel map");
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);size_t hits=0;
  std::vector<int> ids(1);std::vector<float> squared(1);
  for(const auto& p:*sources.front().points){
    const Eigen::Vector3d y=anchor.block<3,3>(0,0)*Eigen::Vector3d(p.x,p.y,p.z)+center;
    require(tree.nearestKSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),1,ids,squared)==1,"anchor NN unavailable");
    hits+=squared[0]<=.25f;
  }
  const double overlap=double(hits)/sources.front().points->size();
  require(pcl::io::savePCDFileBinary(argv[4],*target)==0,"frozen ROI PCD write failed");
  Csv map(out+"/MAP_BUILD.csv","source_ply_points,finite_points,roi_points,target_points,target_fnv,leaf_m,roi_radius_m,center_x,center_y,center_z,anchor_overlap_0p5m,build_ms,peak_rss_KiB");
  map.stream<<all->size()<<','<<finite<<','<<roi_points<<','<<target->size()<<','<<cloudHash(target)
    <<",0.2,50,"<<center.x()<<','<<center.y()<<','<<center.z()<<','<<overlap<<','<<milliseconds(start)<<','<<peakKiB();map.end();
  require(overlap>=.50,"MAP_COVERAGE_BLOCKED anchor overlap below frozen 0.50 support threshold");
  std::cout<<"MAP_PREFLIGHT_PASS scans=61 pcd_format_pass=61 roi_points="<<roi_points
    <<" map_points="<<target->size()<<" anchor_overlap="<<overlap<<" map_build_ms="<<milliseconds(start)
    <<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"MAP_PREFLIGHT_STOP "<<e.what()<<std::endl;return 1;}}
