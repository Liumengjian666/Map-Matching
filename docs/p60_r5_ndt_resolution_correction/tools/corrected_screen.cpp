#include "corrected_ndt.hpp"
#include "metrics.hpp"
#include <pcl/io/pcd_io.h>
#include <array>
#include <iostream>
#include <map>
#include <sys/stat.h>

using namespace screen;
using Record=std::map<std::string,std::string>;
std::vector<std::string> split(const std::string& text) {
  std::stringstream stream(text); std::vector<std::string> out; std::string value;
  while(std::getline(stream,value,',')) out.push_back(value);
  return out;
}
std::vector<Record> readCsv(const std::string& path) {
  std::ifstream file(path); std::string line;
  if(!std::getline(file,line)) throw std::runtime_error("empty CSV: "+path);
  const auto keys=split(line); std::vector<Record> rows;
  while(std::getline(file,line)) {
    auto values=split(line);
    if(values.size()!=keys.size()) throw std::runtime_error("malformed CSV: "+path);
    Record row; for(size_t i=0;i<keys.size();++i) row.emplace(keys[i],values[i]);
    rows.push_back(row);
  }
  return rows;
}
Cloud::Ptr load(const std::string& path) {
  Cloud::Ptr cloud(new Cloud);
  if(pcl::io::loadPCDFile(path,*cloud)) throw std::runtime_error("PCD load failed: "+path);
  return cloud;
}
struct Pair {
  int index; std::string stamp;
  Cloud::Ptr dst; std::array<Cloud::Ptr,3> sources;
};
Pair prepare(const Record& row) {
  Pair pair; pair.index=std::stoi(row.at("scan_index")); pair.stamp=row.at("stamp_ns");
  auto full=source(load(row.at("source_path"))); auto fov=crop(full);
  pair.dst=target(load(row.at("target_path")));
  pair.sources={{full,fov,matched(full,fov->size(),pair.index)}}; return pair;
}
Eigen::Matrix4f poseOf(const Record& row) {
  Eigen::Matrix4f pose;
  for(int i=0;i<4;++i) for(int j=0;j<4;++j)
    pose(i,j)=std::stof(row.at("pose"+std::to_string(i)+std::to_string(j)));
  if(!rigid(pose)) throw std::runtime_error("invalid frozen R4 reference pose");
  return pose;
}
void audit(Csv& csv,const Ndt& ndt,int index,const std::string& stage,const std::string& arm) {
  requireCorrectedGrid(ndt); const auto leaf=ndt.targetGridLeafSizeMeters();
  csv.col(index); csv.col(stage); csv.col(arm);
  for(int j=0;j<3;++j) csv.col(leaf[j]);
  csv.col(ndt.getResolution()); csv.col(1); csv.end();
}
int main(int argc,char** argv) {
  try {
    if(argc!=4) throw std::runtime_error("usage: corrected_screen R4_SAMPLE_MANIFEST R4_ALIGNMENT_CSV NEW_OUTPUT_DIR");
    const auto samples=readCsv(argv[1]), old=readCsv(argv[2]);
    if(samples.size()!=100 || old.size()!=300) throw std::runtime_error("wrong frozen sample count");
    if(::mkdir(argv[3],0775)!=0) throw std::runtime_error("output directory must not exist");
    const std::string root=argv[3];
    const std::array<std::string,3> names{{"FULL_360","FOV_90","POINT_COUNT_MATCHED"}};
    std::map<std::pair<int,std::string>,Record> reference;
    for(const auto& row:old)
      if(!reference.emplace(std::make_pair(std::stoi(row.at("scan_index")),row.at("arm")),row).second)
        throw std::runtime_error("duplicate R4 alignment key");
    Csv parity(root+"/input_parity.csv");
    for(auto name:{"scan_index","arm","stamp_equal","source_count_equal","target_count_equal","source_hash_equal","target_hash_equal","pass"}) parity.col(name);
    parity.end();
    // Verify ALL regenerated source/target identities before the first align.
    for(const auto& sample:samples) {
      const auto pair=prepare(sample);
      for(int arm=0;arm<3;++arm) {
        const auto& ref=reference.at({pair.index,names[arm]});
        const std::array<bool,5> ok{{pair.stamp==ref.at("stamp_ns"),
          pair.sources[arm]->size()==std::stoul(ref.at("source_points")),
          pair.dst->size()==std::stoul(ref.at("target_points")),
          hash(pair.sources[arm])==ref.at("source_hash"),hash(pair.dst)==ref.at("target_hash")}};
        bool pass=true; parity.col(pair.index); parity.col(names[arm]);
        for(bool item:ok) {parity.col(item); pass=pass&&item;} parity.col(pass); parity.end();
        if(!pass) throw std::runtime_error("R4 input parity failed BEFORE any new align");
      }
    }
    std::cout<<"INPUT_PARITY_PASS rows=300 align_calls=0"<<std::endl;
    Csv aligns(root+"/corrected_alignments.csv"), metrics(root+"/corrected_hessian.csv"), grid(root+"/grid_audit.csv");
    for(auto name:{"scan_index","stamp_ns","arm","source_points","target_points","source_hash","target_hash",
        "status","converged","fitness","iterations","align_ms","align_calls","grid_leaf_x","grid_leaf_y","grid_leaf_z"}) aligns.col(name);
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col("pose"+std::to_string(i)+std::to_string(j)); aligns.end();
    metricHeader(metrics);
    for(auto name:{"scan_index","stage","arm","leaf_x","leaf_y","leaf_z","configured_resolution","pass"}) grid.col(name); grid.end();
    int count=0,align_calls=0,jet_calls=0;
    for(const auto& sample:samples) {
      const auto pair=prepare(sample); CorrectedNdt ndt; ndt.configureTarget(pair.dst);
      audit(grid,ndt,pair.index,"TARGET_BUILT","ALL");
      std::array<Eigen::Matrix4f,3> poses;
      std::array<bool,3> converged;
      for(int arm=0;arm<3;++arm) {
        if(pair.sources[arm]->size()<30) throw std::runtime_error("unexpected insufficient source");
        requireCorrectedGrid(ndt);
        const auto start=std::chrono::steady_clock::now();
        ndt.setInputSource(pair.sources[arm]); Cloud output;
        ++align_calls; ndt.align(output,Eigen::Matrix4f::Identity());
        const auto elapsed=ms(start); poses[arm]=ndt.getFinalTransformation();
        const auto fitness=ndt.getFitnessScore(); converged[arm]=ndt.hasConverged();
        audit(grid,ndt,pair.index,"AFTER_ALIGN",names[arm]);
        aligns.col(pair.index); aligns.col(pair.stamp); aligns.col(names[arm]);
        aligns.col(pair.sources[arm]->size()); aligns.col(pair.dst->size());
        aligns.col(hash(pair.sources[arm])); aligns.col(hash(pair.dst));
        aligns.col(rigid(poses[arm])?(converged[arm]?"CONVERGED":"NOT_CONVERGED"):"NONFINITE_OR_NONRIGID");
        aligns.col(converged[arm]); aligns.col(fitness); aligns.col(ndt.getFinalNumIteration()); aligns.col(elapsed); aligns.col(1);
        for(int j=0;j<3;++j) aligns.col(ndt.targetGridLeafSizeMeters()[j]);
        for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col(poses[arm](i,j)); aligns.end();
      }
      const auto& old_full=reference.at({pair.index,"FULL_360"}); const auto old_pose=poseOf(old_full);
      for(int arm=0;arm<3;++arm) {
        for(int evaluation=0;evaluation<2;++evaluation) {
          const std::string label=evaluation==0?"R4_FULL_360_FROZEN":"R5_FULL_360_COMMON";
          requireCorrectedGrid(ndt);
          const auto jet=measure(metrics,pair.index,pair.stamp,names[arm],label,ndt,pair.sources[arm],pair.dst,
            evaluation==0?old_pose:poses[0],evaluation==0?old_full.at("converged")=="1":converged[0]);
          // Every source/pose was eligible for exactly one native derivative call.
          ++jet_calls; audit(grid,ndt,pair.index,"AFTER_JET_"+label,names[arm]);
          if(!jet.valid) std::cerr<<"INVALID_JET index="<<pair.index<<" arm="<<names[arm]<<" pose="<<label<<std::endl;
        }
      }
      std::cout<<"PAIR_COMPLETE index="<<pair.index<<" pair="<<++count<<"/100"<<std::endl;
    }
    if(count!=100 || align_calls!=300 || jet_calls!=600) throw std::runtime_error("call budget mismatch");
    std::cout<<"COMPLETE pairs="<<count<<" align_calls="<<align_calls<<" derivative_requests="<<jet_calls<<std::endl;
  } catch(const std::exception& error) {std::cerr<<error.what()<<std::endl;return 1;}
}
