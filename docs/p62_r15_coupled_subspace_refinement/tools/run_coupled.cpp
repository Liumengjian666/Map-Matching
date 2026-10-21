#include "coupled.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <sys/resource.h>

using namespace coupled;

namespace {
struct CsvOut {
  std::ofstream f;
  explicit CsvOut(const std::string& path, const std::string& header) : f(path, std::ios::out|std::ios::trunc) {
    require(bool(f), "output CSV open failed");
    f << std::setprecision(17) << header << '\n'; f.flush();
  }
  void row(const std::string& s) { f << s << '\n'; f.flush(); }
};
struct Frame {
  std::string set, key, raw_index, stamp, source_path, source_sha, source_fnv;
  Pose t0=Pose::Identity(), tp=Pose::Identity();
  Cloud::Ptr source; Split split; Mask mask;
  bool native_converged=false; int native_iterations=0;
  double native_score=NAN, native_fitness=NAN;
  int scan_index=-1;
  explicit Frame(const Split& s):split(s),mask(s){}
};
inline Pose poseFrom(const coverage::Row& r,const std::string& prefix) {
  Pose p;
  for(int i=0;i<4;++i) for(int j=0;j<4;++j)
    p(i,j)=std::stod(r.at(prefix+std::to_string(i)+std::to_string(j)));
  require(rigid(p),"frozen pose is not SE3"); return p;
}
inline std::string matrixText(const Pose& p) {
  std::ostringstream o; o<<std::setprecision(17);
  for(int i=0;i<4;++i)for(int j=0;j<4;++j){if(i||j)o<<'|';o<<p(i,j);}return o.str();
}
template<class Derived> inline std::string vectorText(const Eigen::MatrixBase<Derived>& v) {
  std::ostringstream o; o<<std::setprecision(17);
  for(Eigen::Index i=0;i<v.size();++i){if(i)o<<'|';o<<v.derived().coeff(i);}return o.str();
}
inline std::string fixedVectorText(const Vector6& v) { return v.allFinite()?vectorText(v):"NA"; }
inline std::string dynamicVectorText(const Eigen::VectorXd& v) { return v.size()?vectorText(v):"NA"; }
inline std::string matrixText(const Eigen::MatrixXd& m) {
  std::ostringstream o; o<<std::setprecision(17);
  for(Eigen::Index i=0;i<m.rows();++i)for(Eigen::Index j=0;j<m.cols();++j){if(i||j)o<<'|';o<<m(i,j);}return o.str();
}
inline std::string dynamicMatrixText(const Eigen::MatrixXd& m) { return m.size()?matrixText(m):"NA"; }
inline void poseColumns(std::string& header,const std::string& prefix) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)header+=","+prefix+std::to_string(i)+std::to_string(j);
}
inline void appendPose(std::ostream& o,const Pose& p) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)o<<','<<p(i,j);
}
inline std::vector<int> active(const coverage::Mask& m){return m.active;}
inline std::string activeText(const coverage::Mask& m){
  std::string s;for(int i:m.active){if(!s.empty())s+='|';s+=std::to_string(i);}return s.empty()?"NA":s;
}
inline void heldout(Candidate& c,const Frame& frame,pcl::KdTreeFLANN<pcl::PointXYZ>& tree){
  coverage::validate(c,frame.split,frame.mask,tree,frame.tp);
}
inline double dPair(const Candidate& j,const Candidate& k,const coverage::Mask& mask){
  if(!mask.evaluable()||!j.loss_valid||!k.loss_valid)return NAN;
  return coverage::paired(j.loss,k.loss,mask);
}

std::vector<Frame> loadR13(const std::string& archive) {
  const std::string events_path=archive+"/EVENTS.csv";
  const std::string candidates_path=archive+"/CANDIDATE_POOL.csv";
  const auto events=coverage::readCsv(events_path);
  const auto candidates=coverage::readCsv(candidates_path);
  const auto source_identity=coverage::readCsv(archive+"/SOURCE_IDENTITY.csv");
  std::map<int,coverage::Row> source_stats;
  for(const auto& r:source_identity)source_stats.emplace(std::stoi(r.at("scan_index")),r);
  std::map<int,coverage::Row> nominal;
  for(const auto& r:candidates) if(r.at("arm")=="R13_TRAINING_GRID"&&std::stoi(r.at("candidate_id"))==0)
    nominal.emplace(std::stoi(r.at("scan_index")),r);
  std::vector<Frame> frames;
  for(const auto& r:events){
    const int idx=std::stoi(r.at("scan_index")); if(idx<60||idx>64)continue;
    auto found=nominal.find(idx);require(found!=nominal.end(),"R13 frozen nominal missing");
    auto identity=source_stats.find(idx);require(identity!=source_stats.end(),"R13 frozen source identity missing");
    auto source=coverage::loadSource(r);
    require(source->size()==std::stoul(r.at("expected_processed_points"))&&coverage::cloudHash(source)==r.at("expected_processed_fnv"),"R13 frozen Source identity mismatch");
    Split split=coverage::splitSource(source);require(source->size()==std::stoul(identity->second.at("source_points"))&&
      coverage::cloudHash(source)==identity->second.at("source_fnv")&&
      split.training->size()==std::stoul(identity->second.at("training_points")),"R13 frozen Source/split identity mismatch");
    Frame f(split);f.set="GEODE_R13";f.key=std::to_string(idx);f.scan_index=idx;f.raw_index=r.at("raw_index");f.stamp=r.at("header_ns");f.source_path=r.at("raw_xyz_offset_file");f.source_sha=r.at("raw_sha256");f.source_fnv=coverage::cloudHash(source);f.source=source;f.tp=coverage::prediction(r);f.t0=poseFrom(found->second,"final_");
    f.native_converged=found->second.at("converged")=="1";f.native_iterations=std::stoi(found->second.at("iterations"));f.native_score=std::stod(found->second.at("native_score"));f.native_fitness=std::stod(found->second.at("fitness"));
    require(f.mask.evaluable()&&f.mask.active.size()>=4,"R13 frozen coverage not evaluable");frames.push_back(std::move(f));
  }
  require(frames.size()==5,"fixed R13 five-event input incomplete");return frames;
}

std::vector<Frame> loadR14(const std::string& archive) {
  const auto scans=coverage::readCsv(archive+"/SCANS.csv");
  const auto sources=coverage::readCsv(archive+"/SOURCE_RUNTIME.csv");
  const auto native=coverage::readCsv(archive+"/NATIVE_NDT_RESULTS.csv");
  const auto final=coverage::readCsv(archive+"/FINAL_OBSERVATIONS.csv");
  std::map<int,coverage::Row> source_stats,nominal,predictions;
  for(const auto& r:sources)source_stats.emplace(std::stoi(r.at("frame")),r);
  for(const auto& r:native)if(r.at("method")=="CONTROL_SPLIT"&&std::stoi(r.at("candidate_id"))==0)nominal.emplace(std::stoi(r.at("frame")),r);
  for(const auto& r:final)predictions.emplace(std::stoi(r.at("frame")),r);
  std::vector<Frame> frames;
  for(const auto& r:scans){
    const int raw=std::stoi(r.at("raw_ordinal"));if(raw<768||raw>787)continue;
    const int frame=raw-767;
    auto ss=source_stats.find(frame),nn=nominal.find(frame),pp=predictions.find(frame);
    require(ss!=source_stats.end()&&nn!=nominal.end()&&pp!=predictions.end(),"R14 frozen frame record missing");
    pilot::Source source=pilot::loadSource(r);
    require(source.points->size()==std::stoul(ss->second.at("source_points"))&&coverage::cloudHash(source.points)==ss->second.at("source_fnv"),"R14 frozen Source identity mismatch");
    require(source.split.training->size()==std::stoul(ss->second.at("training_points")),"R14 training split count mismatch");
    Frame f(source.split);f.set="NEWER_COLLEGE_R14B";f.key=std::to_string(raw);f.scan_index=raw;f.raw_index=std::to_string(raw);f.stamp=r.at("stamp_ns");f.source_path=r.at("path");f.source=source.points;f.source_fnv=coverage::cloudHash(source.points);
    f.tp=poseFrom(pp->second,"prediction_");f.t0=poseFrom(nn->second,"final_");f.native_converged=nn->second.at("converged")=="1";f.native_iterations=std::stoi(nn->second.at("iterations"));f.native_score=std::stod(nn->second.at("native_score"));f.native_fitness=std::stod(nn->second.at("fitness"));
    require(f.mask.evaluable()&&f.mask.active.size()>=4,"R14 frozen coverage not evaluable");frames.push_back(std::move(f));
  }
  require(frames.size()==20,"fixed R14B frames 768-787 incomplete");return frames;
}

Cloud::Ptr loadMap(const std::string& path,uint32_t points,const std::string& expected_fnv){
  Cloud::Ptr map(new Cloud);require(pcl::io::loadPCDFile(path,*map)==0,"frozen target PCD read failure");
  require(map->size()==points&&coverage::cloudHash(map)==expected_fnv,"frozen target identity mismatch");return map;
}

void processFrame(const Frame& frame,const Cloud::Ptr& map,const std::string& outdir,
                  CsvOut& subspaces,CsvOut& blocks,CsvOut& weakout,CsvOut& coupledout,
                  CsvOut& training,CsvOut& heldoutout,CsvOut& runtime,
                  pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  const auto start=Clock::now();
  const auto grid_start=Clock::now();
  ExactPclObjective ndt;ndt.configureExact(map,frame.split.training);
  const double grid_ms=milliseconds(grid_start);
  const auto model_start=Clock::now();
  LocalModel local(ndt,frame.t0);
  const DerivativeResult deriv=local.validateAndDifferentiate();
  size_t support_direct=0,support_r6=0;
  const Matrix6 g=ndt.directGeometry(frame.t0,support_direct);
  const Matrix6 g_r6=ndt.convertedR6Geometry(frame.t0,support_r6);
  const double g_rel=(g-g_r6).norm()/std::max(1.,g.norm());
  Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
  const bool eig_solver_ok=eig.info()==Eigen::Success&&eig.eigenvalues().allFinite()&&eig.eigenvectors().allFinite();
  Eigen::Matrix<double,6,1> values=Eigen::Matrix<double,6,1>::Constant(NAN);Matrix6 u=Matrix6::Constant(NAN);
  bool geometry_ok=false;double basis_orth=NAN;
  int weak_count=0;std::string weak_ids="NA",unmodeled="NA";double cond=NAN;
  if(eig_solver_ok){
    values=eig.eigenvalues();u=eig.eigenvectors();
    basis_orth=(u.transpose()*u-Matrix6::Identity()).norm();
    geometry_ok=values[5]>1e-12&&values[0]>=-1e-10*std::max(1.,values[5])&&basis_orth<=1e-10;
    const double hi=values[5];if(hi>1e-12){weak_count=(values.array()/hi<=.01).count();cond=values[0]>0?hi/values[0]:INFINITY;std::vector<int> wi,uw;for(int i=0;i<6;++i)if(values[i]/hi<=.01){(wi.size()<2?wi:uw).push_back(i);}weak_ids.clear();unmodeled.clear();for(int i:wi){if(!weak_ids.empty())weak_ids+='|';weak_ids+=std::to_string(i);}for(int i:uw){if(!unmodeled.empty())unmodeled+='|';unmodeled+=std::to_string(i);}if(weak_ids.empty())weak_ids="NA";if(unmodeled.empty())unmodeled="NA";}
  }
  const double model_ms=milliseconds(model_start);
  Candidate nominal;nominal.id=0;nominal.initial=frame.tp;nominal.pose=frame.t0;nominal.converged=frame.native_converged;nominal.iterations=frame.native_iterations;nominal.finite=rigid(frame.t0);nominal.score=frame.native_score;nominal.fitness=frame.native_fitness;
  heldout(nominal,frame,tree);
  const double f0=ndt.objective(frame.t0);
  std::ostringstream srow;srow<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<frame.stamp<<','<<frame.source->size()<<','<<frame.split.training->size()<<','<<frame.mask.active.size()<<','<<frame.mask.points<<','<<activeText(frame.mask)<<','<<support_direct<<','<<support_r6<<','<<g_rel<<',';
  for(int i=0;i<6;++i){if(i)srow<<'|';srow<<(eig_solver_ok?values[i]:NAN);}srow<<','<<weak_count<<','<<weak_ids<<','<<unmodeled<<','<<cond<<','<<basis_orth<<','<<geometry_ok<<','<<deriv.status<<','<<deriv.score_parity<<','<<deriv.gradient_check_max_rel<<','<<deriv.h_antisymmetry<<','<<deriv.hessian_check_max_rel<<','<<deriv.hessian_step_spread_max<<','<<deriv.association_queries<<','<<frame.native_converged<<','<<nominal.eligible<<','<<g_rel<<','<<matrixText(frame.t0)<<','<<matrixText(frame.tp);
  subspaces.row(srow.str());
  for(double alpha:{.5,1.}){
    const auto ref_start=Clock::now();Refinement ref;
    if(geometry_ok&&deriv.ok&&g_rel<=1e-7)ref=refine(ndt,frame.t0,frame.tp,values,u,deriv.h_raw,deriv.g_z,alpha);
    else {ref.alpha=alpha;ref.status=deriv.ok?(g_rel>1e-7?"GEOMETRY_CONVERSION_FAILED":"INVALID_GEOMETRY_SPECTRUM"):deriv.status;ref.weak_status=ref.coupled_status=ref.status;}
    if(!ref.weak_available){
      const auto unavailable=[&](CsvOut& out,const char* method,const std::string& status){
        std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<status<<','<<method<<",POST_NDT_REFINEMENT_CANDIDATE,"<<status
          <<",0,0,0,0,NA,NA,NA,NA,NA,NA,NA,NA,NA";
        for(int b=0;b<8;++b)row<<",NA";out.row(row.str());
      };
      unavailable(weakout,"WEAK_ONLY",ref.weak_status);
      unavailable(coupledout,"COUPLED",ref.coupled_status);
      std::ostringstream tr;tr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<f0<<",NA,NA,NA,NA,NA,NA,NA,NA,NA,NA,"<<ref.status;training.row(tr.str());
      std::ostringstream br;br<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<ref.status;for(int i=0;i<20;++i)br<<",NA";blocks.row(br.str());
      for(const char* method:{"NOMINAL_NDT","WEAK_ONLY","COUPLED"})for(int b=0;b<8;++b){
        std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<method<<','<<b<<','<<frame.split.blocks[b]->size()<<','
          <<(std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())<<",NA";heldoutout.row(row.str());}
      std::ostringstream rr;rr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<grid_ms<<','<<model_ms<<",NA,"<<milliseconds(start)<<','<<peakKiB()<<",0,0,0";runtime.row(rr.str());
      continue;
    }
    Candidate w;w.id=1;w.finite=rigid(ref.weak_pose);w.converged=false;w.pose=ref.weak_pose;w.score=NAN;w.fitness=NAN;heldout(w,frame,tree);
    Candidate c;c.id=2;c.finite=ref.coupled_available&&rigid(ref.coupled_pose);c.converged=false;c.pose=ref.coupled_pose;c.score=NAN;c.fitness=NAN;
    if(ref.coupled_available)heldout(c,frame,tree);
    const double d_weak=dPair(w,nominal,frame.mask),d_coupled=ref.coupled_available?dPair(c,nominal,frame.mask):NAN;
    std::vector<Candidate> pool{nominal,w};if(ref.coupled_available)pool.push_back(c);
    const auto v3=pool_v3::construct(pool,frame.mask,frame.tp);
    const auto selected=pool_v3::select(v3,frame.mask,frame.tp);
    const auto decision=pool_v3::decide(v3,frame.mask,frame.tp,selected);
    ref.weak_only_formal_eligible=pool_v3::admissible(v3.raw[1],frame.mask,frame.tp);
    ref.coupled_formal_eligible=ref.coupled_available&&pool_v3::admissible(v3.raw[2],frame.mask,frame.tp);
    const auto addResult=[&](CsvOut& out,const Candidate& cand,const char* method,const std::string& model_status,double local_delta,double exact_delta,double d){
      std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<"NOT_R12_ACCEPTED"<<','<<method<<",POST_NDT_REFINEMENT_CANDIDATE,"<<model_status<<','<<cand.finite<<','<<cand.converged<<",0,"<<cand.loss_valid<<','<<cand.overlap<<','<<(cand.loss_valid?coverage::mean(cand.loss,frame.mask):NAN)<<','<<d<<','<<translation(cand.pose,frame.t0)<<','<<rotation(cand.pose,frame.t0)<<','<<local_delta<<','<<exact_delta<<','<<matrixText(cand.pose)<<','<<vectorText(std::string(method)=="WEAK_ONLY"?ref.z_weak:ref.z_coupled);
      for(int b=0;b<8;++b){row<<',';if(cand.loss_valid&&std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())row<<cand.loss[b];else row<<"NA";}out.row(row.str());
    };
    addResult(weakout,w,"WEAK_ONLY",ref.weak_status,ref.local_delta_weak,ref.exact_delta_weak,d_weak);
    if(ref.coupled_available)addResult(coupledout,c,"COUPLED",ref.coupled_status,ref.local_delta_coupled,ref.exact_delta_coupled,d_coupled);
    else {
      std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<",NOT_R12_ACCEPTED,COUPLED,POST_NDT_REFINEMENT_CANDIDATE,"<<ref.coupled_status<<",0,0,0,0,NA,NA,NA,NA,NA,NA,NA,NA,NA";for(int b=0;b<8;++b)row<<",NA";coupledout.row(row.str());
    }
    const std::array<std::pair<const char*,const Candidate*>,3> heldout_candidates{{{"NOMINAL_NDT",&nominal},{"WEAK_ONLY",&w},{"COUPLED",&c}}};
    for(int b=0;b<8;++b)for(const auto& item:heldout_candidates){
      std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<item.first<<','<<b<<','<<frame.split.blocks[b]->size()<<','<<(std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())<<',';
      if(item.second->loss_valid&&std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())row<<item.second->loss[b];else row<<"NA";heldoutout.row(row.str());}
    std::ostringstream tr;tr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<f0<<','<<ref.exact_delta_weak+f0<<','<<(ref.coupled_available?ref.exact_delta_coupled+f0:NAN)<<','<<ref.exact_delta_weak<<','<<(ref.coupled_available?ref.exact_delta_coupled:NAN)<<','<<ref.local_delta_weak<<','<<(ref.coupled_available?ref.local_delta_coupled:NAN)<<','<<d_weak<<','<<d_coupled<<','<<ref.coupling_norm<<','<<ref.strong_response_norm<<','<<(ref.coupled_available?"PASS":ref.coupled_status);training.row(tr.str());
    std::ostringstream br;br<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<ref.status<<','<<ref.weak_ids<<','<<ref.strong_ids<<','<<ref.unmodeled_weak_ids<<','<<dynamicVectorText(ref.gs)<<','<<dynamicVectorText(ref.gw)<<','<<dynamicMatrixText(ref.hss)<<','<<dynamicMatrixText(ref.hsw)<<','<<dynamicMatrixText(ref.hww)<<','<<fixedVectorText(ref.dw)<<','<<fixedVectorText(ref.ds)<<','<<ref.raw_hss_antisymmetry<<','<<ref.raw_hss_condition<<','<<ref.regularized_hss_condition<<','<<kLambda<<','<<ref.coupling_norm<<','<<(ref.coupling_increment.size()?ref.coupling_increment.norm():NAN)<<','<<ref.trust_scale<<','<<ref.strong_response_norm<<','<<decision.status<<','<<decision.observation_valid;
    blocks.row(br.str());
    const double refine_ms=milliseconds(ref_start);
    std::ostringstream rr;rr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.raw_index<<','<<alpha<<','<<grid_ms<<','<<model_ms<<','<<refine_ms<<','<<milliseconds(start)<<','<<peakKiB()<<",0,0,0";runtime.row(rr.str());
  }
}

} // namespace

int main(int argc,char** argv){
  try{
    require(argc==4,"usage: run_coupled <outdir> <r13_archive> <r14b_archive>");
    const std::string out=argv[1],r13=argv[2],r14=argv[3];
    auto f13=loadR13(r13);auto f14=loadR14(r14);
    CsvOut sub(out+"/SUBSPACE_DECOMPOSITION.csv","dataset,frame_key,raw_index,stamp_ns,source_points,training_points,B,active_validation_points,active_blocks,geometry_supported_direct,geometry_supported_R6,geometry_conversion_rel,eigenvalues_ascending,weak_count,selected_weak_ids,unmodeled_weak_ids,geometry_condition,geometry_basis_orthogonality,geometry_psd_gate,local_model_status,score_parity_abs,right_tangent_gradient_check_max_rel,hessian_antisymmetry_rel,scalar_hessian_max_rel,hessian_step_spread_max,association_queries,frozen_T0_converged,T0_current_eligibility,g_conversion_rel,T0_pose_pipe,TP_pose_pipe");
    CsvOut blocks(out+"/COUPLING_BLOCKS.csv","dataset,frame_key,alpha,status,selected_weak_ids,strong_ids,unmodeled_weak_ids,gs,gw,Hss_rowmajor,Hsw_rowmajor,Hww_rowmajor,dw,ds,raw_Hss_antisymmetry,raw_Hss_condition,regularized_Hss_condition,lambda,Hsw_frobenius,coupling_increment_norm,trust_scale,strong_response_norm,R12_shadow_decision,observation_valid");
    CsvOut weak(out+"/WEAK_ONLY_RESULTS.csv","dataset,frame_key,raw_index,alpha,status,method,candidate_kind,local_model_status,finite,converged,formal_eligible,heldout_valid,overlap_0p5,heldout_mean,D_candidate_vs_T0,translation_from_T0_m,rotation_from_T0_deg,local_quadratic_delta,exact_training_delta,pose_pipe,z_pipe,loss0,loss1,loss2,loss3,loss4,loss5,loss6,loss7");
    CsvOut coupledout(out+"/COUPLED_RESULTS.csv","dataset,frame_key,raw_index,alpha,status,method,candidate_kind,local_model_status,finite,converged,formal_eligible,heldout_valid,overlap_0p5,heldout_mean,D_candidate_vs_T0,translation_from_T0_m,rotation_from_T0_deg,local_quadratic_delta,exact_training_delta,pose_pipe,z_pipe,loss0,loss1,loss2,loss3,loss4,loss5,loss6,loss7");
    CsvOut training(out+"/TRAINING_OBJECTIVE_COMPARISON.csv","dataset,frame_key,raw_index,alpha,F_NDT_nominal,F_NDT_weak,F_NDT_coupled,delta_weak,delta_coupled,quadratic_delta_weak,quadratic_delta_coupled,D_weak_vs_T0,D_coupled_vs_T0,Hsw_frobenius,strong_response_norm,status");
    CsvOut heldout(out+"/HELDOUT_COMPARISON.csv","dataset,frame_key,alpha,method,block,block_points,active,loss");
    CsvOut runtime(out+"/RUNTIME_AND_MEMORY.csv","dataset,frame_key,raw_index,alpha,grid_build_ms,local_model_ms,refinement_eval_ms,frame_wall_ms,peak_rss_KiB,new_NDT_aligns,EKF_runs,GT_reads");
    const std::string m13="/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd";
    auto map13=loadMap(m13,459154,"ea1e4e69cf9992c6");pcl::KdTreeFLANN<pcl::PointXYZ> tree13;tree13.setInputCloud(map13);
    for(auto& f:f13)processFrame(f,map13,out,sub,blocks,weak,coupledout,training,heldout,runtime,tree13);
    const std::string m14="/home/jian/rosbag/public_datasets/newer_college_r14b/conditional_base_pilot_20261021/map/leica_roi50_leaf020.pcd";
    auto map14=loadMap(m14,224627,"28dc95023de5a54c");pcl::KdTreeFLANN<pcl::PointXYZ> tree14;tree14.setInputCloud(map14);
    for(auto& f:f14)processFrame(f,map14,out,sub,blocks,weak,coupledout,training,heldout,runtime,tree14);
    std::cout<<"R15_LOCAL_MODEL_COMPLETE frames="<<f13.size()+f14.size()<<" align=0 ekf=0 gt_reads=0\n";
  }catch(const std::exception& e){std::cerr<<"R15_LOCAL_MODEL_FAILED "<<e.what()<<'\n';return 1;}
  return 0;
}
