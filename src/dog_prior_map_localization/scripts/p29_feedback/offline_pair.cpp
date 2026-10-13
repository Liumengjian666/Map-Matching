// Fixed 2x2 experiment: existing native PCL NDT, no new search or feedback.
#include "../p23_budgeted_coupled/frozen_ndt_rules.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Ndt=dog_prior_map_fastlio2_frontend_exp::ObservablePclNdt<pcl::PointXYZ>;
using Clock=std::chrono::steady_clock;
struct Case {
  int frame,points,width,height,dense,iterations,converged,lt,lr,selected;
  uint64_t ns,hash; double fitness;
};
std::vector<Case> cases(const std::string& path) {
  std::ifstream in(path); std::vector<Case> out; Case c;
  while(in>>c.frame>>c.ns>>c.points>>c.width>>c.height>>c.dense>>std::hex>>c.hash>>std::dec
        >>c.iterations>>c.converged>>c.fitness>>c.lt>>c.lr>>c.selected)out.push_back(c);
  if(out.size()!=8)throw std::runtime_error("P29_EIGHT_CASES_REQUIRED");
  return out;
}
std::vector<Eigen::Matrix4d> poses(const std::string& path) {
  std::ifstream in(path,std::ios::binary); std::vector<Eigen::Matrix4d> out(5);
  for(auto& T:out)for(int i=0;i<4;++i)for(int j=0;j<4;++j)
    if(!in.read(reinterpret_cast<char*>(&T(i,j)),sizeof(double)))throw std::runtime_error("POSE_READ_FAILED");
  for(const auto& T:out)if(!T.allFinite())throw std::runtime_error("NONFINITE_INPUT_POSE");
  return out;
}
Cloud::Ptr source(const std::string& path,const Case& c) {
  std::ifstream in(path,std::ios::binary); Cloud::Ptr out(new Cloud); float p[3];
  while(in.read(reinterpret_cast<char*>(p),sizeof(p)))out->push_back(pcl::PointXYZ(p[0],p[1],p[2]));
  out->width=c.width;out->height=c.height;out->is_dense=c.dense;
  if(out->size()!=static_cast<std::size_t>(c.points)||p23::FrozenNdtRules::stableCloudHash(out)!=c.hash)
    throw std::runtime_error("FIXED_SOURCE_HASH_FAILED");
  return out;
}
double angle(const Eigen::Matrix4d& a,const Eigen::Matrix4d& b) {
  return Eigen::AngleAxisd(Eigen::Quaterniond(a.block<3,3>(0,0)).normalized().conjugate()*
    Eigen::Quaterniond(b.block<3,3>(0,0)).normalized()).angle()*180/M_PI;
}
void matrix(std::ostream& out,const Eigen::Matrix4d& T) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<T(i,j);
}
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"map.pcd P29-cache\n";return 2;}
  int calls=0;
  try {
    std::string root=argv[2],out=root+"/offline";
    int fd=::open((out+"/started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if(fd<0)throw std::runtime_error("OFFLINE_ALREADY_ATTEMPTED");::close(fd);
    auto A=cases(root+"/cases_A.txt"),B=cases(root+"/cases_B.txt");
    p23::FrozenNdtRules rules;Cloud::Ptr raw(new Cloud),finite(new Cloud);
    if(pcl::io::loadPCDFile(argv[1],*raw))throw std::runtime_error("MAP_READ_FAILED");
    for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);
    p23::finalizeCloud(finite);
    const auto map=rules.voxelDown(finite,.15,.15,0),target=rules.voxelDown(map,.15,.15,0);
    if(target->size()!=226164)throw std::runtime_error("TARGET_POINTS_CHANGED");
    Ndt ndt;ndt.setInputTarget(target);ndt.setResolution(.8);ndt.setStepSize(.08);
    ndt.setTransformationEpsilon(.001);ndt.setMaximumIterations(40);
    std::ofstream results(out+"/fixed_source_pairs.csv"),parity(out+"/optimizer_parity.csv"),ledger(out+"/align_ledger.csv");
    if(!results||!parity||!ledger)throw std::runtime_error("OUTPUT_OPEN_FAILED");
    results<<std::setprecision(17);parity<<std::setprecision(17);
    results<<"frame_index,scan_ns,source,initial,source_hash,points,iterations,converged,fitness,objective_initial,objective_raw,objective_used,align_ms,translation_limited,rotation_limited,online_selected_candidate,candidate_replacement_executed";
    for(const auto prefix:{"initial_m","raw_m","used_m"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)results<<','<<prefix<<i<<j;
    results<<'\n';
    parity<<"frame_index,method,raw_dt_m,raw_dr_deg,used_dt_m,used_dr_deg,fitness_delta,discrete_match,status\n";
    ledger<<"call,frame,source,initial,status\n";
    for(int n=0;n<8;++n) {
      if(A[n].frame!=B[n].frame||A[n].ns!=B[n].ns)throw std::runtime_error("CASE_IDENTITIES_DIFFER");
      const std::string name="/frame_"+std::to_string(A[n].frame);
      const auto PA=poses(root+"/sources/A"+name+".poses.f64"),PB=poses(root+"/sources/B"+name+".poses.f64");
      const auto SA=source(root+"/sources/A"+name+".xyz",A[n]),SB=source(root+"/sources/B"+name+".xyz",B[n]);
      for(int s=0;s<2;++s)for(int i=0;i<2;++i) {
        const Case& c=s?B[n]:A[n]; const auto& history=s?PB:PA; const auto& init=i?PB[1]:PA[1];
        ndt.setInputSource(s?SB:SA);rules.previous_pose_=history[0];
        if(++calls>32)throw std::runtime_error("ALIGN_BUDGET_EXCEEDED");
        ledger<<calls<<','<<c.frame<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<",STARTED\n";ledger.flush();
        Cloud aligned;auto start=Clock::now();ndt.align(aligned,init.cast<float>());
        const double ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        const Eigen::Matrix4d result=ndt.getFinalTransformation().cast<double>();
        const double fitness=ndt.getFitnessScore();const int iterations=ndt.getFinalNumIteration();
        const bool converged=ndt.hasConverged();Eigen::Matrix4d used=result;bool lt=false,lr=false;
        rules.limitNdtStep(result,used,lt,lr);
        const auto cloud=s?SB:SA;
        const double vi=ndt.dynamicScore(cloud,init.cast<float>()),vr=ndt.dynamicScore(cloud,result.cast<float>()),vu=ndt.dynamicScore(cloud,used.cast<float>());
        if(!result.allFinite()||!used.allFinite()||!std::isfinite(fitness)||!std::isfinite(vi)||!std::isfinite(vr)||!std::isfinite(vu))
          throw std::runtime_error("NONFINITE_OFFLINE_RESULT");
        results<<c.frame<<','<<c.ns<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<','<<std::hex<<c.hash<<std::dec<<','<<c.points
          <<','<<iterations<<','<<converged<<','<<fitness<<','<<vi<<','<<vr<<','<<vu<<','<<ms<<','<<lt<<','<<lr<<','<<c.selected<<",0";
        matrix(results,init);matrix(results,result);matrix(results,used);results<<'\n';results.flush();
        ledger<<calls<<','<<c.frame<<','<<(s?'B':'A')<<','<<(i?'B':'A')<<",COMPLETED\n";ledger.flush();
        if(s==i) {
          const double dt=(result.block<3,1>(0,3)-history[3].block<3,1>(0,3)).norm(),dr=angle(result,history[3]);
          const double ut=(used.block<3,1>(0,3)-history[4].block<3,1>(0,3)).norm(),ur=angle(used,history[4]);
          const bool discrete=iterations==c.iterations&&converged==bool(c.converged)&&lt==bool(c.lt)&&lr==bool(c.lr);
          const bool same=dt<=1e-5&&dr<=1e-4&&ut<=1e-5&&ur<=1e-4&&discrete&&std::abs(fitness-c.fitness)<=1e-6*std::max(1.,std::abs(c.fitness));
          const char* status=c.selected>=0?"ONLINE_MULTI_SELECTED":(same?"PASS":"FAIL");
          parity<<c.frame<<','<<(s?'B':'A')<<','<<dt<<','<<dr<<','<<ut<<','<<ur<<','<<fitness-c.fitness<<','<<discrete<<','<<status<<'\n';parity.flush();
          if(c.selected<0&&!same)throw std::runtime_error("NATIVE_OPTIMIZER_PARITY_FAILED");
        }
      }
    }
    rusage usage;getrusage(RUSAGE_SELF,&usage);
    std::ofstream resource(out+"/resource_summary.csv");
    resource<<"target_points,target_hash,full_align_calls,objective_value_calls,peak_rss_kib\n"<<target->size()<<','
      <<std::hex<<rules.stableCloudHash(target)<<std::dec<<','<<calls<<','<<calls*3<<','<<usage.ru_maxrss<<'\n';
    std::cout<<"P29_OFFLINE_PASS frames=8 full_aligns="<<calls<<'\n';return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<" full_aligns="<<calls<<'\n';return 1;}
}
