#include "corrected_ndt.hpp"
#include "metrics.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

using namespace screen;
Cloud::Ptr load(const char* path) {
  Cloud::Ptr result(new Cloud);
  if(pcl::io::loadPCDFile(path,*result)!=0 || result->empty())throw std::runtime_error("invalid PCD");
  return result;
}
void nn(Csv& csv,const char* phase,const Cloud::Ptr& src,const Cloud::Ptr& dst,const Eigen::Matrix4f& pose) {
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(dst);
  Cloud transformed;pcl::transformPointCloud(*src,transformed,pose);
  std::vector<double> distances;std::vector<int> index(1);std::vector<float> squared(1);
  for(const auto& p:transformed)if(tree.nearestKSearch(p,1,index,squared)==1)distances.push_back(std::sqrt(squared[0]));
  std::sort(distances.begin(),distances.end());
  if(distances.size()!=src->size())throw std::runtime_error("NN query incomplete");
  csv.col(phase);csv.col(src->size());csv.col(dst->size());
  csv.col(distances[distances.size()/2]);csv.col(distances[static_cast<size_t>(.9*(distances.size()-1))]);
  for(double threshold:{.5,1.,2.}) {
    const auto count=std::count_if(distances.begin(),distances.end(),[&](double d){return d<=threshold;});
    csv.col(count);csv.col(double(count)/distances.size());
  }
  csv.end();
}
int main(int argc,char** argv) {
  try {
    if(argc!=5)throw std::runtime_error("usage: single_frame RAW_PCD FROZEN_MAP INITIAL_MATRIX NEW_OUTPUT_DIR");
    if(::mkdir(argv[4],0775)!=0)throw std::runtime_error("output already exists");
    const std::string folder=argv[4];
    const auto raw=load(argv[1]),map=load(argv[2]);
    const auto src=source(raw),dst=target(map);
    if(src->size()<30 || dst->size()<6)throw std::runtime_error("insufficient geometry");
    Eigen::Matrix4f initial;std::ifstream input(argv[3]);
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)if(!(input>>initial(i,j)))throw std::runtime_error("bad frozen matrix");
    if(!rigid(initial))throw std::runtime_error("invalid initializer");
    CorrectedNdt ndt;const auto build_start=std::chrono::steady_clock::now();ndt.configureTarget(dst);
    const double build_ms=ms(build_start);requireCorrectedGrid(ndt);ndt.setInputSource(src);
    Csv overlaps(folder+"/geometry.csv");
    for(const auto* key:{"phase","source_points","target_points","nn_median_m","nn_p90_m","matches_0p5m","fraction_0p5m","matches_1m","fraction_1m","matches_2m","fraction_2m"})overlaps.col(key);overlaps.end();
    nn(overlaps,"FROZEN_T0_INITIALIZER_RAW_SCAN",src,dst,initial);
    Cloud output;const auto start=std::chrono::steady_clock::now();
    ndt.align(output,initial); // Exactly one permitted align; no retries or branch selection.
    const double align_ms=ms(start);const auto final=ndt.getFinalTransformation();requireCorrectedGrid(ndt);
    Csv results(folder+"/ndt.csv");
    for(const auto* key:{"raw_source_points","source_points","raw_map_points","target_points","source_hash","target_hash","converged","pose_valid","fitness","iterations","target_build_ms","align_ms","align_calls","correction_translation_m","correction_rotation_deg"})results.col(key);
    for(const auto* key:{"initial","final"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)results.col(std::string(key)+std::to_string(i)+std::to_string(j));results.end();
    results.col(raw->size());results.col(src->size());results.col(map->size());results.col(dst->size());
    results.col(hash(src));results.col(hash(dst));results.col(ndt.hasConverged());results.col(rigid(final));
    results.col(ndt.getFitnessScore());results.col(ndt.getFinalNumIteration());results.col(build_ms);results.col(align_ms);results.col(1);
    results.col((final.block<3,1>(0,3)-initial.block<3,1>(0,3)).norm());
    const Eigen::Matrix3f rotation_delta=final.block<3,3>(0,0)*initial.block<3,3>(0,0).transpose();
    results.col(Eigen::AngleAxisf(rotation_delta).angle()*180./M_PI);
    for(const auto& pose:{initial,final})for(int i=0;i<4;++i)for(int j=0;j<4;++j)results.col(pose(i,j));results.end();
    if(rigid(final))nn(overlaps,"NDT_DIAGNOSTIC_OUTPUT_ONLY",src,dst,final);
    Csv metrics(folder+"/hessian.csv");metricHeader(metrics);
    measure(metrics,2986,"1706671587900364160","RAW_PRIOR_MAP_PROBE","OWN_NDT_OUTPUT",ndt,src,dst,final,ndt.hasConverged());
    Csv grid(folder+"/grid.csv");for(const auto* k:{"leaf_x","leaf_y","leaf_z","align_calls"})grid.col(k);grid.end();
    for(float leaf:ndt.targetGridLeafSizeMeters())grid.col(leaf);grid.col(1);grid.end();
    pcl::io::savePCDFileBinary(folder+"/preprocessed_source.pcd",*src);
    std::cout<<"ONE_ALIGN_COMPLETE converged="<<ndt.hasConverged()<<" align_ms="<<align_ms<<std::endl;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
