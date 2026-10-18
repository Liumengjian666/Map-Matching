#include "corrected_ndt.hpp"
#include "metrics.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

using namespace screen;
std::vector<std::string> split(const std::string& line) {
  std::stringstream stream(line); std::string value; std::vector<std::string> fields;
  while(std::getline(stream,value,',')) fields.push_back(value);
  return fields;
}
Cloud::Ptr load(const std::string& path) {
  Cloud::Ptr cloud(new Cloud);
  if(pcl::io::loadPCDFile(path,*cloud)) throw std::runtime_error("PCD read failed");
  return cloud;
}
void audit(Csv& csv,const Ndt& ndt,int index,const std::string& stage) {
  requireCorrectedGrid(ndt); csv.col(index); csv.col(stage);
  for(float leaf:ndt.targetGridLeafSizeMeters()) csv.col(leaf);
  csv.col(1); csv.end();
}
int main(int argc,char** argv) {
  try {
    if(argc!=4) throw std::runtime_error("usage: native_geometry MANIFEST COUNT NEW_OUTPUT_DIR");
    const int expected=std::stoi(argv[2]);
    if(expected!=20 && expected!=100) throw std::runtime_error("only frozen20/100 pair budgets allowed");
    std::ifstream input(argv[1]); std::string line; std::vector<std::vector<std::string>> rows;
    if(!std::getline(input,line)) throw std::runtime_error("missing manifest");
    while(std::getline(input,line)) {auto fields=split(line); if(fields.size()!=7) throw std::runtime_error("bad manifest"); rows.push_back(fields);}
    if(static_cast<int>(rows.size())!=expected) throw std::runtime_error("pair count mismatch");
    if(::mkdir(argv[3],0775)!=0) throw std::runtime_error("output directory must not exist");
    const std::string directory=argv[3]; Csv aligns(directory+"/alignments.csv"), metrics(directory+"/hessian.csv"), grid(directory+"/grid_audit.csv");
    for(auto key:{"scan_index","stamp_ns","previous_stamp_ns","source_points","target_points","source_hash","target_hash",
                 "converged","rigid","fitness","iterations","target_build_ms","align_ms","align_calls"}) aligns.col(key);
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col("pose"+std::to_string(i)+std::to_string(j)); aligns.end();
    metricHeader(metrics);
    for(auto key:{"scan_index","stage","leaf_x","leaf_y","leaf_z","pass"}) grid.col(key); grid.end();
    int calls=0;
    for(const auto& row:rows) {
      const int index=std::stoi(row[0]);
      if(index<2) throw std::runtime_error("previous target absent");
      Cloud::Ptr src=source(load(row[3])), dst=target(load(row[4]));
      if(src->size()<30 || dst->size()<6) throw std::runtime_error("insufficient source/target");
      CorrectedNdt ndt; const auto build_start=std::chrono::steady_clock::now(); ndt.configureTarget(dst);
      const double build_ms=ms(build_start); audit(grid,ndt,index,"TARGET_BUILT");
      ndt.setInputSource(src); Cloud output; const auto start=std::chrono::steady_clock::now();
      ++calls; ndt.align(output,Eigen::Matrix4f::Identity()); const double elapsed=ms(start);
      const auto pose=ndt.getFinalTransformation(); const bool converged=ndt.hasConverged();
      audit(grid,ndt,index,"AFTER_ALIGN");
      aligns.col(index); aligns.col(row[1]); aligns.col(row[2]); aligns.col(src->size()); aligns.col(dst->size());
      aligns.col(hash(src)); aligns.col(hash(dst)); aligns.col(converged); aligns.col(rigid(pose));
      aligns.col(ndt.getFitnessScore()); aligns.col(ndt.getFinalNumIteration()); aligns.col(build_ms); aligns.col(elapsed); aligns.col(1);
      for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col(pose(i,j)); aligns.end();
      measure(metrics,index,row[1],"RAW_NATIVE","OWN_NATIVE_RESULT",ndt,src,dst,pose,converged);
      audit(grid,ndt,index,"AFTER_JET");
      std::cout<<"PAIR_COMPLETE "<<index<<" align_calls="<<calls<<std::endl;
    }
    if(calls!=expected) throw std::runtime_error("align budget mismatch");
    std::cout<<"COMPLETE align_calls="<<calls<<" derivative_requests="<<expected<<std::endl;
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;return 1;}
}
