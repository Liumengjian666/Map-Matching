// P23 fixed real-source experiment. One map/grid; unchanged PCL align resets
// each optimization from its explicit candidate. No trajectory state update.
#include "frozen_ndt_rules.hpp"
#include "local_refinement.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>
namespace f=dog_prior_map_fastlio2_frontend_exp;
using Cloud=pcl::PointCloud<pcl::PointXYZ>;
using Ndt=f::ObservablePclNdt<pcl::PointXYZ>;
struct Case {
  int frame,points,width,height,dense,iterations,converged,limited_t,limited_r;
  uint64_t ns,hash;double fitness;
};
struct Result {
  Eigen::Matrix4d initial,raw,used;
  int iterations=0;bool converged=false,limited_t=false,limited_r=false;
  double fitness=0,align_ms=0,fitness_ms=0,post_ms=0,score=0;
};
void poseHeader(std::ostream& out,const std::string& prefix) {
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) out<<','<<prefix<<i<<j;
}
void poseValues(std::ostream& out,const Eigen::Matrix4d& pose) {
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) out<<','<<pose(i,j);
}
template<typename Derived>std::string vectorText(const Eigen::MatrixBase<Derived>& vector) {
  std::ostringstream text;text<<std::setprecision(17);
  for (int i=0;i<vector.size();++i) {if(i)text<<'|';text<<vector(i);}
  return text.str();
}
double angle(const Eigen::Matrix3d& a,const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(Eigen::Quaterniond(a).normalized().conjugate()*Eigen::Quaterniond(b).normalized()).angle()*180/M_PI;
}
int main(int argc,char**argv) {
  if (argc!=5) {std::cerr<<"map.pcd sources-directory cases.txt new-output-directory\n";return 2;}
  int full_calls=0;
  try {
    const std::string source_directory=argv[2],directory=argv[4];
    const int marker=::open((directory+"/offline_started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if (marker<0)throw std::runtime_error("OFFLINE_ALREADY_ATTEMPTED_OR_INVALID_DIRECTORY");
    ::close(marker);
    std::ifstream case_file(argv[3]);std::vector<Case> cases;Case entry;
    while(case_file>>entry.frame>>entry.ns>>entry.points>>entry.width>>entry.height>>entry.dense>>std::hex>>entry.hash>>std::dec
          >>entry.iterations>>entry.converged>>entry.fitness>>entry.limited_t>>entry.limited_r)cases.push_back(entry);
    if(cases.size()<4 || cases.size()>8)throw std::runtime_error("CASE_COVERAGE_CONTRACT_FAILED");
    std::ofstream ledger(directory+"/align_ledger.csv");ledger<<"call,frame,method,status\n";
    std::ofstream paired(directory+"/ndt_paired_results.csv"),parity(directory+"/nominal_parity.csv"),
        geometry(directory+"/weak_geometry.csv"),candidates(directory+"/weak_candidates.csv"),
        coupling(directory+"/coupled_correction.csv"),runtime(directory+"/runtime_breakdown.csv");
    for(auto* stream:{&ledger,&paired,&parity,&geometry,&candidates,&coupling,&runtime}) {
      if(!*stream)throw std::runtime_error("OUTPUT_STREAM_FAILED");*stream<<std::setprecision(17);
    }
    paired<<"frame_index,scan_ns,method,iterations,converged,fitness,translation_limited,rotation_limited,align_ms,fitness_ms,post_ms,final_score";
    for(auto prefix:{"initial_m","raw_m","used_m"})poseHeader(paired,prefix);paired<<'\n';
    parity<<"frame_index,raw_translation_m,raw_rotation_deg,used_translation_m,used_rotation_deg,fitness_delta,discrete_match,status\n";
    geometry<<"frame_index,valid,status,weak_dimension,explored_dimension,eigenvalues,weak_direction,alpha,hessian_symmetry\n";
    candidates<<"frame_index,sign,coordinate,score,energy,selected,translation_m,rotation_deg";
    poseHeader(candidates,"pose_m");candidates<<'\n';
    coupling<<"frame_index,status,fallback,predictor,correction,strong_gradient,condition,damping,solve_residual,capped,weak_score,predicted_score,coupled_score";
    poseHeader(coupling,"weak_m");poseHeader(coupling,"coupled_m");coupling<<'\n';
    runtime<<"frame_index,nominal_align_ms,nominal_fitness_ms,nominal_post_ms,weak_setup_ms,eigen_ms,coupled_setup_ms,strong_solve_ms,B_align_ms,B_fitness_ms,B_post_ms,C_align_ms,C_fitness_ms,C_post_ms,B_incremental_ms,C_incremental_ms,value_calls,jet_calls,full_align_calls,value_ms,jet_ms\n";
    const auto map_start=p23::Clock::now();
    Cloud::Ptr raw(new Cloud),finite(new Cloud);
    if(pcl::io::loadPCDFile(argv[1],*raw))throw std::runtime_error("MAP_READ_FAILED");
    for(const auto& point:*raw)if(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z))finite->push_back(point);
    p23::finalizeCloud(finite);p23::FrozenNdtRules rules;
    const auto map=rules.voxelDown(finite,.15,.15,0),target=rules.voxelDown(map,.15,.15,0);
    Ndt ndt;ndt.setInputTarget(target);ndt.setResolution(.8);ndt.setStepSize(.08);
    ndt.setTransformationEpsilon(.001);ndt.setMaximumIterations(40);
    const double map_ms=p23::ms(map_start);
    if(target->size()!=226164)throw std::runtime_error("TARGET_POINT_COUNT_CHANGED");
    for(const auto& item:cases) {
      const std::string prefix=source_directory+"/frame_"+std::to_string(item.frame);
      std::ifstream xyz(prefix+".xyz",std::ios::binary),matrices(prefix+".poses.f64",std::ios::binary);
      Cloud::Ptr source(new Cloud);float values[3];
      while(xyz.read(reinterpret_cast<char*>(values),sizeof(values)))source->push_back(pcl::PointXYZ(values[0],values[1],values[2]));
      source->width=item.width;source->height=item.height;source->is_dense=item.dense;
      if(source->size()!=static_cast<std::size_t>(item.points) || rules.stableCloudHash(source)!=item.hash)
        throw std::runtime_error("SOURCE_HASH_FAILED");
      std::vector<Eigen::Matrix4d> poses(5);
      for(auto& pose:poses)for(int i=0;i<4;++i)for(int j=0;j<4;++j) {
        if(!matrices.read(reinterpret_cast<char*>(&pose(i,j)),sizeof(double)))throw std::runtime_error("POSE_FILE_INCOMPLETE");
      }
      rules.previous_pose_=poses[0];ndt.setInputSource(source);
      int value_calls=0,jet_calls=0;double value_ms=0,jet_ms=0;
      f::CoupledNdtBackend backend;
      backend.jet=[&](const Eigen::Matrix4f& pose,const f::CoupledVector6& parameters) {
        const auto start=p23::Clock::now();++jet_calls;auto jet=ndt.nativeJet(source,pose,parameters);
        jet_ms+=p23::ms(start);return jet;
      };
      backend.score=[&](const Eigen::Matrix4f& pose) {
        const auto start=p23::Clock::now();++value_calls;const double value=ndt.dynamicScore(source,pose);
        value_ms+=p23::ms(start);return value;
      };
      auto align=[&](char method,const Eigen::Matrix4d& initial) {
        if(++full_calls>24)throw std::runtime_error("ALIGN_BUDGET_EXCEEDED");
        ledger<<full_calls<<','<<item.frame<<','<<method<<",STARTED\n";ledger.flush();
        Result result;result.initial=initial;Cloud aligned;
        auto start=p23::Clock::now();ndt.align(aligned,initial.cast<float>());
        result.align_ms=p23::ms(start);result.raw=ndt.getFinalTransformation().cast<double>();
        result.converged=ndt.hasConverged();result.iterations=ndt.getFinalNumIteration();
        start=p23::Clock::now();result.fitness=ndt.getFitnessScore();result.fitness_ms=p23::ms(start);
        if(!result.raw.allFinite() || !std::isfinite(result.fitness))throw std::runtime_error("NONFINITE_NDT");
        start=p23::Clock::now();
        result.used=result.raw;rules.limitNdtStep(result.raw,result.used,result.limited_t,result.limited_r);
        if(!result.used.allFinite())throw std::runtime_error("NONFINITE_USED");
        result.score=backend.score(result.raw.cast<float>());
        result.post_ms=p23::ms(start);
        if(!std::isfinite(result.score))throw std::runtime_error("NONFINITE_TERMINAL_SCORE");
        ledger<<full_calls<<','<<item.frame<<','<<method<<",COMPLETED\n";ledger.flush();
        paired<<item.frame<<','<<item.ns<<','<<method<<','<<result.iterations<<','<<result.converged<<','<<result.fitness
              <<','<<result.limited_t<<','<<result.limited_r<<','<<result.align_ms<<','<<result.fitness_ms<<','<<result.post_ms<<','<<result.score;
        poseValues(paired,result.initial);poseValues(paired,result.raw);poseValues(paired,result.used);paired<<'\n';paired.flush();
        return result;
      };
      const auto A=align('A',poses[1]);
      const double raw_dt=(A.raw.block<3,1>(0,3)-poses[3].block<3,1>(0,3)).norm(),
          used_dt=(A.used.block<3,1>(0,3)-poses[4].block<3,1>(0,3)).norm(),
          raw_dr=angle(A.raw.block<3,3>(0,0),poses[3].block<3,3>(0,0)),
          used_dr=angle(A.used.block<3,3>(0,0),poses[4].block<3,3>(0,0));
      const bool discrete=A.iterations==item.iterations && A.converged==bool(item.converged)
          && A.limited_t==bool(item.limited_t) && A.limited_r==bool(item.limited_r);
      const bool same=raw_dt<=1e-5 && used_dt<=1e-5 && raw_dr<=1e-4 && used_dr<=1e-4
          && std::abs(A.fitness-item.fitness)<=1e-6*std::max(1.0,std::abs(item.fitness)) && discrete;
      parity<<item.frame<<','<<raw_dt<<','<<raw_dr<<','<<used_dt<<','<<used_dr<<','<<A.fitness-item.fitness<<','<<discrete<<','<<(same?"PASS":"FAIL")<<'\n';parity.flush();
      if(!same)throw std::runtime_error("OPTIMIZER_PARITY_FAILED");
      const auto proposal=p23::propose(A.raw.cast<float>(),source->size(),backend);
      geometry<<item.frame<<','<<proposal.valid<<','<<proposal.status<<','<<proposal.weak_dimension<<','<<(proposal.valid?1:0)
          <<','<<vectorText(proposal.eigenvalues)<<','<<vectorText(proposal.Q.col(0))<<','<<proposal.alpha<<','<<proposal.symmetry<<'\n';geometry.flush();
      if(proposal.valid)for(int i=0;i<3;++i) {
        const double u=i==0?0:(i==1?proposal.alpha:-proposal.alpha);
        const auto pose=f::coupledPoseAtEta(A.raw.cast<float>(),proposal.Q.col(0)*u,.8);
        candidates<<item.frame<<','<<(i==0?"ZERO":(i==1?"PLUS":"MINUS"))<<','<<u<<','<<proposal.scores[i]
            <<','<<-proposal.scores[i]/source->size()<<','<<(u==proposal.selected_u)<<','
            <<(pose.block<3,1>(0,3).cast<double>()-A.raw.block<3,1>(0,3)).norm()<<','
            <<angle(pose.block<3,3>(0,0).cast<double>(),A.raw.block<3,3>(0,0));
        poseValues(candidates,pose.cast<double>());candidates<<'\n';
      }
      coupling<<item.frame<<','<<proposal.status<<','<<proposal.fallback<<','<<vectorText(proposal.predictor)<<','
          <<vectorText(proposal.correction)<<','<<vectorText(proposal.strong_gradient)<<','<<proposal.condition<<','<<proposal.damping
          <<','<<proposal.residual<<','<<proposal.capped<<','<<(proposal.selected_u==0?proposal.scores[0]:
              (proposal.selected_u>0?proposal.scores[1]:proposal.scores[2]))<<','<<proposal.predicted_score<<','<<proposal.coupled_score;
      poseValues(coupling,proposal.weak.cast<double>());poseValues(coupling,proposal.coupled.cast<double>());coupling<<'\n';coupling.flush();
      const auto B=align('B',proposal.weak.cast<double>()),C=align('C',proposal.coupled.cast<double>());
      runtime<<item.frame<<','<<A.align_ms<<','<<A.fitness_ms<<','<<A.post_ms<<','<<proposal.weak_ms<<','<<proposal.eigen_ms
          <<','<<proposal.coupled_ms<<','<<proposal.solve_ms<<','<<B.align_ms<<','<<B.fitness_ms<<','<<B.post_ms<<','<<C.align_ms<<','<<C.fitness_ms<<','<<C.post_ms
          <<','<<proposal.weak_ms+B.align_ms+B.fitness_ms+B.post_ms
          <<','<<proposal.weak_ms+proposal.coupled_ms+C.align_ms+C.fitness_ms+C.post_ms
          <<','<<value_calls<<','<<jet_calls<<",3,"<<value_ms<<','<<jet_ms<<'\n';runtime.flush();
    }
    rusage usage;getrusage(RUSAGE_SELF,&usage);
    std::ofstream resources(directory+"/resource_summary.csv");
    resources<<std::setprecision(17)<<"map_setup_ms,target_points,target_hash,peak_rss_kib,full_align_calls\n"
        <<map_ms<<','<<target->size()<<','<<std::hex<<rules.stableCloudHash(target)<<std::dec<<','<<usage.ru_maxrss<<','<<full_calls<<'\n';
    std::cout<<"P23_OFFLINE_COMPLETE frames="<<cases.size()<<" align_calls="<<full_calls<<'\n';
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<" full_align_calls="<<full_calls<<'\n';return 1;
  }
}
