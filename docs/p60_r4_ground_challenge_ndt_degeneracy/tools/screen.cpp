#include "metrics.hpp"
#include <pcl/io/pcd_io.h>
#include <array>
#include <iostream>
#include <map>
#include <sys/stat.h>

using namespace screen;
std::vector<std::string> split(const std::string& text) {
  std::stringstream s(text); std::vector<std::string> out; std::string v;
  while (std::getline(s,v,',')) out.push_back(v);
  return out;
}
std::map<std::string,std::string> firstRecord(const std::string& path) {
  std::ifstream f(path); std::string header,line;
  if (!std::getline(f,header) || !std::getline(f,line)) throw std::runtime_error("missing parity reference");
  auto keys=split(header), values=split(line);
  if (keys.size()!=values.size()) throw std::runtime_error("bad parity CSV");
  std::map<std::string,std::string> out;
  for(size_t i=0;i<keys.size();++i) out.emplace(keys[i],values[i]);
  return out;
}
Cloud::Ptr load(const std::string& path) {
  Cloud::Ptr c(new Cloud);
  if (pcl::io::loadPCDFile(path,*c)) throw std::runtime_error("PCD load failed");
  return c;
}
struct Alignment {
  Eigen::Matrix4f pose=Eigen::Matrix4f::Constant(NAN);
  double fitness=NAN, elapsed=0; int iterations=0, calls=0; bool converged=false;
  std::string status="INSUFFICIENT_SOURCE";
};
Alignment alignOne(Ndt& ndt,Cloud::Ptr source) {
  Alignment a;
  if(source->size()<30) return a;
  const auto start=std::chrono::steady_clock::now();
  ndt.setInputSource(source); Cloud output; ++a.calls;
  ndt.align(output,Eigen::Matrix4f::Identity());
  a.elapsed=ms(start); a.pose=ndt.getFinalTransformation(); a.fitness=ndt.getFitnessScore();
  a.iterations=ndt.getFinalNumIteration(); a.converged=ndt.hasConverged();
  a.status=rigid(a.pose)?(a.converged?"CONVERGED":"NOT_CONVERGED"):"NONFINITE_OR_NONRIGID";
  return a;
}
int main(int argc,char** argv) {
  try {
    if(argc!=5) throw std::runtime_error("usage: screen SAMPLE_MANIFEST R2_NDT_CSV R2_HESSIAN_CSV NEW_OUTPUT_DIRECTORY");
    if(::mkdir(argv[4],0775)!=0) throw std::runtime_error("output directory must not exist");
    const auto ref=firstRecord(argv[2]), href=firstRecord(argv[3]);
    Csv aligns(std::string(argv[4])+"/alignments.csv"), metrics(std::string(argv[4])+"/hessian_statistics.csv"),
        parity(std::string(argv[4])+"/control_replay_parity.csv");
    for(auto s:{"scan_index","stamp_ns","arm","source_points","target_points","source_hash","target_hash",
        "status","converged","fitness","iterations","align_ms","align_calls","grid_leaf_m","translation_norm_m","rotation_deg"}) aligns.col(s);
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col("pose"+std::to_string(i)+std::to_string(j)); aligns.end();
    metricHeader(metrics);
    for(auto s:{"scan_index","source_hash_equal","source_count_equal","target_count_equal","converged_equal",
        "iterations_equal","translation_difference_m","rotation_difference_deg","fitness_difference","hessian_max_relative_difference","pass"}) parity.col(s);
    parity.end();
    std::ifstream manifest(argv[1]); std::string line;
    if(!std::getline(manifest,line)) throw std::runtime_error("empty manifest");
    int pairs=0,total_align=0,total_jet=0;
    while(std::getline(manifest,line)) {
      auto row=split(line); if(row.size()!=7) throw std::runtime_error("bad sample row");
      const int index=std::stoi(row[0]); const auto stamp=row[1];
      Cloud::Ptr full=source(load(row[3])), dst=target(load(row[4])), fov=crop(full);
      std::array<Cloud::Ptr,3> sources{{full,fov,matched(full,fov->size(),index)}};
      std::array<std::string,3> names{{"FULL_360","FOV_90","POINT_COUNT_MATCHED"}};
      Ndt ndt; ndt.configureTarget(dst);
      std::array<Alignment,3> results;
      for(int arm=0;arm<3;++arm) {
        results[arm]=alignOne(ndt,sources[arm]); const auto& a=results[arm]; total_align+=a.calls;
        aligns.col(index); aligns.col(stamp); aligns.col(names[arm]); aligns.col(sources[arm]->size()); aligns.col(dst->size());
        aligns.col(hash(sources[arm])); aligns.col(hash(dst)); aligns.col(a.status); aligns.col(a.converged);
        aligns.col(a.fitness); aligns.col(a.iterations); aligns.col(a.elapsed); aligns.col(a.calls);
        aligns.col(ndt.targetGridLeafSizeMeters()[0]);
        aligns.col(rigid(a.pose)?a.pose.block<3,1>(0,3).norm():NAN);
        aligns.col(rigid(a.pose)?Eigen::AngleAxisf(a.pose.block<3,3>(0,0)).angle()*180/std::acos(-1.):NAN);
        for(int i=0;i<4;++i) for(int j=0;j<4;++j) aligns.col(a.pose(i,j)); aligns.end();
      }
      for(int arm=0;arm<3;++arm) {
        const auto jet=measure(metrics,index,stamp,names[arm],"FULL_360_COMMON",ndt,sources[arm],dst,results[0].pose,results[0].converged);
        total_jet+=jet.valid?1:0;
        if(pairs==0 && arm==0) {
          const auto& a=results[0]; Eigen::Matrix4f reference=Eigen::Matrix4f::Identity();
          Eigen::Quaternionf q(std::stof(ref.at("raw_ndt_qw")),std::stof(ref.at("raw_ndt_qx")),std::stof(ref.at("raw_ndt_qy")),std::stof(ref.at("raw_ndt_qz")));
          reference.block<3,3>(0,0)=q.normalized().toRotationMatrix();
          for(int i=0;i<3;++i) reference(i,3)=std::stof(ref.at(std::string("raw_ndt_t")+"xyz"[i]));
          const double dt=(a.pose.block<3,1>(0,3)-reference.block<3,1>(0,3)).norm();
          const Eigen::Matrix3f delta=a.pose.block<3,3>(0,0)*reference.block<3,3>(0,0).transpose();
          const double dr=Eigen::AngleAxisf(delta).angle()*180/std::acos(-1.);
          const double df=std::abs(a.fitness-std::stod(ref.at("ndt_fitness")));
          Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig(jet.H); double dh=INFINITY;
          if(jet.valid && eig.info()==Eigen::Success) {
            dh=0; for(int j=0;j<6;++j) {double l=std::stod(href.at("lambda"+std::to_string(j))); dh=std::max(dh,std::abs(eig.eigenvalues()[j]-l)/std::max(1.,std::abs(l)));}
          }
          const bool sh=hash(full)==ref.at("cloud_hash"), sc=full->size()==std::stoul(ref.at("cloud_size_after_filter")), tc=dst->size()==1827;
          const bool cv=a.converged==(ref.at("ndt_has_converged")=="1"), it=a.iterations==std::stoi(ref.at("ndt_iterations"));
          const bool pass=index==2 && sh && sc && tc && cv && it && dt<=.001 && dr<=.01 && df<=1e-5 && dh<=1e-5;
          parity.col(index); parity.col(sh); parity.col(sc); parity.col(tc); parity.col(cv); parity.col(it);
          parity.col(dt); parity.col(dr); parity.col(df); parity.col(dh); parity.col(pass); parity.end();
          if(!pass) throw std::runtime_error("frozen production first-pair parity FAILED; stop without retry");
        }
        const auto own=measure(metrics,index,stamp,names[arm],"OWN_RESULT",ndt,sources[arm],dst,results[arm].pose,results[arm].converged);
        total_jet+=own.valid?1:0;
      }
      ++pairs; std::cout<<"PAIR_COMPLETE index="<<index<<" pair="<<pairs<<"/100"<<std::endl;
    }
    if(pairs!=100) throw std::runtime_error("did not execute all frozen pairs");
    std::cout<<"COMPLETE pairs="<<pairs<<" align_calls="<<total_align<<" valid_jet_evaluations="<<total_jet<<std::endl;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl; return 1;}
}
