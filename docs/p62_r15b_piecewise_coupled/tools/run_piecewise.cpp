#include "piecewise.hpp"
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
inline std::map<std::string,double> readR15NominalObjectives(const std::string& archive){
  const auto rows=coverage::readCsv(archive+"/run_01/TRAINING_OBJECTIVE_COMPARISON.csv");
  std::map<std::string,double> out;
  for(const auto& r:rows){const std::string key=r.at("dataset")+"|"+r.at("frame_key");
    const double value=std::stod(r.at("F_NDT_nominal"));
    auto it=out.find(key);if(it==out.end())out.emplace(key,value);
    else require(std::abs(it->second-value)<=1e-12,"R15 repeated nominal objective rows disagree");}
  require(out.size()==25,"R15 historical nominal objectives incomplete");return out;
}
inline double matrixRel(const Matrix6& a,const Matrix6& b){return (a-b).norm()/std::max(1.,a.norm());}
inline std::string numberText(double v){if(!std::isfinite(v))return "NA";std::ostringstream o;o<<std::setprecision(17)<<v;return o.str();}

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

void processFrame(const Frame& frame,ExactPclObjective& ndt,double map_grid_build_ms,bool first_map_frame,
                  const std::map<std::string,double>& r15_nominal,
                  CsvOut& associations_out,CsvOut& boundary_out,CsvOut& derivative_out,
                  CsvOut& coupling_out,CsvOut& candidates_out,CsvOut& pair_out,CsvOut& dynamic_out,
                  CsvOut& heldout_out,CsvOut& runtime_out,
                  pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  const auto frame_start=Clock::now();
  const std::string grid_before=ndt.gridFingerprint();
  const auto source_switch_start=Clock::now();ndt.setTraining(frame.split.training);
  const double source_switch_ms=milliseconds(source_switch_start);
  const std::string grid_after=ndt.gridFingerprint();
  require(grid_before==grid_after,"Gaussian Grid changed while switching frozen source");
  const auto model_start=Clock::now();
  NominalAssociationBranch branch(ndt,frame.t0);
  const PiecewiseDerivativeResult deriv=branch.differentiate();
  const BoundaryStencilSummary boundary=branch.boundaryStencilSummary();
  const auto geometry_start=Clock::now();
  size_t support_direct=0,support_r6=0,support_same=0;
  const Matrix6 g=ndt.directGeometry(frame.t0,support_direct);
  const Matrix6 g_r6=ndt.convertedR6Geometry(frame.t0,support_r6);
  const Matrix6 g_same=ndt.convertedSameArithmeticGeometry(frame.t0,support_same);
  const double g_r6_rel=matrixRel(g,g_r6),g_same_rel=matrixRel(g,g_same);
  Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
  const bool eig_ok=eig.info()==Eigen::Success&&eig.eigenvalues().allFinite()&&eig.eigenvectors().allFinite();
  Eigen::Matrix<double,6,1> values=Eigen::Matrix<double,6,1>::Constant(NAN);
  Matrix6 u=Matrix6::Constant(NAN);double basis_orth=NAN,condition=NAN,min_ratio=NAN;
  bool geometry_ok=false;int formal_weak_count=0;
  std::string formal_weak_ids="NA",unmodeled_weak_ids="NA";
  if(eig_ok){
    values=eig.eigenvalues();u=eig.eigenvectors();basis_orth=(u.transpose()*u-Matrix6::Identity()).norm();
    const double high=values[5];
    if(high>1e-12){
      min_ratio=values[0]/high;formal_weak_count=int((values.array()/high<=.01).count());
      condition=values[0]>0?high/values[0]:INFINITY;
      geometry_ok=values[0]>=-1e-10*std::max(1.,high)&&basis_orth<=1e-10&&g.allFinite();
      std::vector<int> formal,tail;
      for(int i=0;i<6;++i)if(values[i]/high<=.01)(formal.size()<2?formal:tail).push_back(i);
      auto text=[](const std::vector<int>& ids){std::string s;for(int i:ids){if(!s.empty())s+='|';s+=std::to_string(i);}return s.empty()?"NA":s;};
      formal_weak_ids=text(formal);unmodeled_weak_ids=text(tail);
    }
  }
  const double geometry_ms=milliseconds(geometry_start);
  const double model_ms=milliseconds(model_start);

  Candidate nominal;nominal.id=0;nominal.initial=frame.tp;nominal.pose=frame.t0;
  nominal.converged=frame.native_converged;nominal.iterations=frame.native_iterations;
  nominal.finite=rigid(frame.t0);nominal.score=frame.native_score;nominal.fitness=frame.native_fitness;
  heldout(nominal,frame,tree);
  const double f_dynamic_0=ndt.objective(frame.t0);
  const std::string baseline_key=frame.set+"|"+frame.key;
  const auto base_it=r15_nominal.find(baseline_key);require(base_it!=r15_nominal.end(),"R15 nominal objective missing for frozen frame");
  const double r15_score_delta=std::abs(f_dynamic_0-base_it->second);
  const bool score_reuse_match=r15_score_delta<=1e-9;
  const bool grid_reuse_ok=grid_before==ndt.initialGridFingerprint()&&grid_after==ndt.initialGridFingerprint();
  require(score_reuse_match,"reused-map Gaussian score differs from R15 fresh-grid baseline");
  require(grid_reuse_ok,"reused-map Gaussian Grid fingerprint differs from initial map Grid");

  for(size_t i=0;i<branch.nominalAssociations().canonical_ids.size();++i){
    std::ostringstream row;row<<frame.set<<','<<frame.key<<','<<frame.scan_index<<','<<i<<',';
    const auto& ids=branch.nominalAssociations().canonical_ids[i];
    if(ids.empty())row<<"NONE";else for(size_t j=0;j<ids.size();++j){if(j)row<<'|';row<<ids[j];}
    associations_out.row(row.str());
  }
  std::ostringstream ar;ar<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.stamp<<','
      <<branch.pointCount()<<','<<branch.nominalAssociations().association_count<<','<<branch.nominalAssociations().canonical_ids.size()<<','
      <<deriv.associated_points<<','<<deriv.nominal_association_digest<<','<<deriv.f_fixed_zero<<','<<deriv.f_dynamic_zero<<','
      <<deriv.fixed_dynamic_parity_abs<<','<<deriv.pcl_derivative_score_parity_abs<<','<<r15_score_delta<<','<<score_reuse_match<<','
      <<boundary.sample_count<<','<<boundary.changed_sample_count<<','<<boundary.changed_point_memberships<<','<<boundary.total_point_memberships<<','
      <<boundary.added_leaf_memberships<<','<<boundary.removed_leaf_memberships<<','<<boundary.nominal_leaf_memberships_per_sample;
  boundary_out.row(ar.str());
  std::ostringstream dr;dr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.stamp<<','<<deriv.status<<','<<deriv.ok<<','
      <<deriv.f_fixed_zero<<','<<deriv.f_dynamic_zero<<','<<deriv.fixed_dynamic_parity_abs<<','<<deriv.pcl_derivative_score_parity_abs<<','
      <<deriv.gradient_check_max_rel<<','<<deriv.hessian_antisymmetry_rel<<','<<deriv.hessian_check_max_rel<<','<<deriv.hessian_step_spread_max<<','
      <<deriv.objective_evaluations<<','<<deriv.nominal_points<<','<<deriv.nominal_association_count<<','<<deriv.nominal_association_digest<<','
      <<(deriv.g_z.allFinite()?vectorText(deriv.g_z):"NA")<<','<<(deriv.h_raw.allFinite()?dynamicMatrixText(deriv.h_raw):"NA");
  derivative_out.row(dr.str());

  for(double alpha:{.5,1.}){
    const auto refinement_start=Clock::now();Refinement ref;
    if(geometry_ok&&deriv.ok)ref=refine(ndt,frame.t0,frame.tp,values,u,deriv.h_raw,deriv.g_z,alpha,true);
    else{ref.alpha=alpha;ref.status=deriv.ok?"INVALID_G_Z_SPECTRUM_OR_BASIS":deriv.status;ref.weak_status=ref.coupled_status=ref.status;}
    Candidate weak;weak.id=1;weak.converged=false;weak.finite=ref.weak_available&&rigid(ref.weak_pose);weak.pose=ref.weak_pose;
    Candidate coupled_candidate;coupled_candidate.id=2;coupled_candidate.converged=false;
    coupled_candidate.finite=ref.coupled_available&&rigid(ref.coupled_pose);coupled_candidate.pose=ref.coupled_pose;
    if(weak.finite)heldout(weak,frame,tree);
    if(coupled_candidate.finite)heldout(coupled_candidate,frame,tree);
    const double d_weak=weak.loss_valid?dPair(weak,nominal,frame.mask):NAN;
    const double d_coupled=coupled_candidate.loss_valid?dPair(coupled_candidate,nominal,frame.mask):NAN;
    std::vector<Candidate> raw_pool{nominal};if(weak.finite)raw_pool.push_back(weak);if(coupled_candidate.finite)raw_pool.push_back(coupled_candidate);
    const auto v3=pool_v3::construct(raw_pool,frame.mask,frame.tp);
    const auto selected=pool_v3::select(v3,frame.mask,frame.tp);
    const auto decision=pool_v3::decide(v3,frame.mask,frame.tp,selected);
    const auto write_candidate=[&](const Candidate& cand,const char* method,const std::string& status,const Vector6& z,
                                   double local_delta,double fixed_delta,double dynamic_delta,double d){
      AssociationDelta ad;double ffixed=NAN,fdynamic=NAN;
      if(cand.finite){ad=branch.dynamicAssociationDelta(cand.pose);fdynamic=ndt.objective(cand.pose);ffixed=branch.fixedObjective(z);}
      const double mean_loss=cand.loss_valid?coverage::mean(cand.loss,frame.mask):NAN;
      const char* kind=std::string(method)=="NOMINAL_NDT"?"FROZEN_NATIVE_NDT":"POST_NDT_REFINEMENT_CANDIDATE";
      std::ostringstream row;row<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.scan_index<<','<<alpha<<','<<method<<','
        <<kind<<','<<status<<','<<cand.finite<<','<<cand.converged<<','<<cand.eligible<<','<<cand.loss_valid<<','<<cand.overlap<<','<<mean_loss<<','<<d<<','
        <<(cand.finite?translation(cand.pose,frame.t0):NAN)<<','<<(cand.finite?rotation(cand.pose,frame.t0):NAN)<<','
        <<local_delta<<','<<fixed_delta<<','<<dynamic_delta<<','<<(std::isfinite(ffixed)?ffixed-f_dynamic_0:NAN)<<','
        <<(std::isfinite(fdynamic)?fdynamic-f_dynamic_0:NAN)<<','<<(std::isfinite(fdynamic)&&std::isfinite(ffixed)?fdynamic-ffixed:NAN)<<','
        <<ad.changed_points<<','<<ad.total_points<<','<<ad.added_leaf_memberships<<','<<ad.removed_leaf_memberships<<','
        <<ad.changed_point_fraction<<','<<ad.changed_membership_fraction<<','<<(cand.finite?matrixText(cand.pose):"NA")<<','<<(z.allFinite()?vectorText(z):"NA");
      for(int b=0;b<8;++b){row<<',';if(cand.loss_valid&&std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())row<<cand.loss[b];else row<<"NA";}
      candidates_out.row(row.str());
      std::ostringstream dyn;dyn<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.scan_index<<','<<alpha<<','<<method<<','
        <<(cand.finite?ffixed:NAN)<<','<<(cand.finite?fdynamic:NAN)<<','<<(cand.finite?ffixed-f_dynamic_0:NAN)<<','
        <<(cand.finite?fdynamic-f_dynamic_0:NAN)<<','<<(cand.finite?fdynamic-ffixed:NAN)<<','<<ad.changed_points<<','<<ad.total_points<<','
        <<ad.added_leaf_memberships<<','<<ad.removed_leaf_memberships<<','<<ad.changed_point_fraction<<','<<ad.changed_membership_fraction;
      dynamic_out.row(dyn.str());
      for(int b=0;b<8;++b){std::ostringstream h;h<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<method<<','<<b<<','
        <<frame.split.blocks[b]->size()<<','<<(std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())<<',';
        if(cand.loss_valid&&std::find(frame.mask.active.begin(),frame.mask.active.end(),b)!=frame.mask.active.end())h<<cand.loss[b];else h<<"NA";heldout_out.row(h.str());}
    };
    write_candidate(nominal,"NOMINAL_NDT","FROZEN_NOMINAL",Vector6::Zero(),0.,0.,0.,NAN);
    write_candidate(weak,"WEAK_ONLY_PROBE",ref.weak_status,ref.z_weak,ref.local_delta_weak,
      ref.weak_available?branch.fixedObjective(ref.z_weak)-deriv.f_fixed_zero:NAN,
      ref.exact_delta_weak,d_weak);
    write_candidate(coupled_candidate,"COUPLED_PROBE",ref.coupled_status,ref.z_coupled,ref.local_delta_coupled,
      ref.coupled_available?branch.fixedObjective(ref.z_coupled)-deriv.f_fixed_zero:NAN,
      ref.exact_delta_coupled,d_coupled);
    const double weak_coupled_t=weak.finite&&coupled_candidate.finite?translation(weak.pose,coupled_candidate.pose):NAN;
    const double weak_coupled_r=weak.finite&&coupled_candidate.finite?rotation(weak.pose,coupled_candidate.pose):NAN;
    std::ostringstream cr;cr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.scan_index<<','<<alpha<<','<<ref.status<<','
      <<"SPECTRAL_TAIL_MECHANISM_PROBE"<<','<<formal_weak_count<<','<<min_ratio<<','<<0<<','
      <<(eig_ok?vectorText(values):"NA")<<','<<(eig_ok?dynamicMatrixText(u):"NA")<<','<<(eig_ok?dynamicMatrixText(g):"NA")<<','
      <<geometry_ok<<','<<condition<<','<<basis_orth<<','<<g_r6_rel<<','<<g_same_rel<<','<<support_direct<<','<<support_r6<<','<<support_same<<','
      <<ref.weak_ids<<','<<ref.strong_ids<<','
      <<ref.unmodeled_weak_ids<<','<<dynamicVectorText(ref.gs)<<','<<dynamicVectorText(ref.gw)<<','<<dynamicMatrixText(ref.hss)<<','
      <<dynamicMatrixText(ref.hsw)<<','<<dynamicMatrixText(ref.hww)<<','<<fixedVectorText(ref.dw)<<','<<fixedVectorText(ref.ds)<<','
      <<ref.raw_hss_antisymmetry<<','<<ref.raw_hss_condition<<','<<ref.regularized_hss_condition<<','<<kLambda<<','<<ref.coupling_norm<<','
      <<ref.coupling_increment.norm()<<','<<ref.trust_scale<<','<<ref.strong_response_norm<<','<<weak_coupled_t<<','<<weak_coupled_r<<','
      <<d_weak<<','<<d_coupled<<','<<decision.status<<','<<decision.observation_valid<<",0,0";
    coupling_out.row(cr.str());
    std::ostringstream wr;wr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<alpha<<','<<ref.status<<','<<ref.weak_status<<','
      <<ref.coupled_status<<','<<ref.exploratory_min_spectral_probe<<','<<formal_weak_count<<','<<min_ratio<<','
      <<(weak.finite?matrixText(weak.pose):"NA")<<','<<(coupled_candidate.finite?matrixText(coupled_candidate.pose):"NA")<<','
      <<(weak.finite?vectorText(ref.z_weak):"NA")<<','<<(coupled_candidate.finite?vectorText(ref.z_coupled):"NA")<<','
      <<ref.local_delta_weak<<','<<ref.local_delta_coupled<<','<<ref.exact_delta_weak<<','<<ref.exact_delta_coupled<<','
      <<(weak.finite?translation(weak.pose,frame.t0):NAN)<<','<<(weak.finite?rotation(weak.pose,frame.t0):NAN)<<','
      <<(coupled_candidate.finite?translation(coupled_candidate.pose,frame.t0):NAN)<<','
      <<(coupled_candidate.finite?rotation(coupled_candidate.pose,frame.t0):NAN)<<','<<weak_coupled_t<<','<<weak_coupled_r<<','
      <<ref.coupling_norm<<','<<ref.strong_response_norm<<','<<ref.trust_scale<<','<<d_weak<<','<<d_coupled<<','
      <<weak.eligible<<','<<coupled_candidate.eligible;
    // Add the held-out mean change separately from per-block records.
    wr<<','<<(weak.loss_valid?coverage::mean(weak.loss,frame.mask):NAN)<<','
      <<(coupled_candidate.loss_valid?coverage::mean(coupled_candidate.loss,frame.mask):NAN);
    // A candidate can be numerically present yet never inherit NDT convergence or formal eligibility.
    wr<<",PAIR_SUMMARY";
    pair_out.row(wr.str());

    const double ref_ms=milliseconds(refinement_start);
    std::ostringstream rr;rr<<std::setprecision(17)<<frame.set<<','<<frame.key<<','<<frame.scan_index<<','<<alpha<<','
      <<(first_map_frame?map_grid_build_ms:0.)<<','<<source_switch_ms<<','<<model_ms<<','<<geometry_ms<<','<<ref_ms<<','
      <<milliseconds(frame_start)<<','<<peakKiB()<<",0,0,0,"<<grid_reuse_ok<<','<<r15_score_delta;
    runtime_out.row(rr.str());
  }
}

} // namespace

int main(int argc,char** argv){
  try{
    require(argc==5,"usage: run_piecewise <outdir> <r13_archive> <r14b_archive> <r15_archive>");
    const std::string out=argv[1],r13=argv[2],r14=argv[3],r15=argv[4];
    auto f13=loadR13(r13);auto f14=loadR14(r14);
    const auto r15_nominal=readR15NominalObjectives(r15);
    CsvOut associations(out+"/NOMINAL_ASSOCIATION_IDENTITIES.csv","dataset,frame_key,raw_index,training_point_index,sorted_gaussian_leaf_keys");
    CsvOut boundary(out+"/ASSOCIATION_BOUNDARY_ANALYSIS.csv","dataset,frame_key,stamp_ns,training_points,nominal_leaf_association_count,association_signature_points,associated_training_points,association_digest,F_fixed_zero,F_dynamic_zero,score_parity_abs,pcl_derivative_score_parity_abs,R15_fresh_grid_nominal_delta,fresh_grid_score_match,finite_difference_stencil_samples,stencil_samples_with_changed_membership,stencil_point_membership_changes,total_stencil_point_memberships,added_leaf_memberships,removed_leaf_memberships,nominal_leaf_memberships_per_stencil_sample");
    CsvOut derivatives(out+"/LOCAL_MODEL_DERIVATIVES.csv","dataset,frame_key,stamp_ns,status,derivatives_valid,F_fixed_zero,F_dynamic_zero,fixed_dynamic_abs_error,pcl_derivative_score_abs_error,gradient_directional_max_relative_error,Hessian_antisymmetry_relative_error,Hessian_scalar_max_relative_error,Hessian_step_spread_max,objective_evaluations,training_points,nominal_associations,association_digest,g_z,H_raw_rowmajor");
    CsvOut coupling(out+"/SUBSPACE_AND_COUPLING.csv","dataset,frame_key,raw_index,alpha,probe_status,direction_semantics,formal_weak_count,minimum_eigenvalue_ratio,exploratory_direction_index,eigenvalues_ascending,eigenvectors_rowmajor,G_z_rowmajor,geometry_valid,geometry_condition,basis_orthogonality,R6_actual_conversion_relative_error,same_path_algebraic_conversion_relative_error,geometry_support_direct,geometry_support_R6,geometry_support_same_path,selected_probe_ids,solved_complement_ids,unmodeled_formal_weak_ids,g_s,g_w,H_ss,H_sw,H_ww,d_w,d_s,raw_Hss_antisymmetry,raw_Hss_condition,regularized_Hss_condition,lambda,Hsw_frobenius,coupling_increment_norm,trust_scale,strong_response_norm,weak_coupled_translation_m,weak_coupled_rotation_deg,D_weak_vs_T0,D_coupled_vs_T0,R12_shadow_decision,observation_valid,formal_candidate_accepted,new_NDT_aligns");
    CsvOut candidates(out+"/CANDIDATE_OBJECTIVES.csv","dataset,frame_key,raw_index,alpha,method,candidate_kind,status,finite,converged,formal_eligible,heldout_valid,overlap_0p5,heldout_mean,D_candidate_vs_T0,translation_from_T0_m,rotation_from_T0_deg,local_quadratic_delta,fixed_branch_actual_delta,dynamic_PCL_actual_delta,fixed_branch_delta_from_nominal,dynamic_PCL_delta_from_nominal,dynamic_minus_fixed_delta,changed_training_points,total_training_points,added_leaf_memberships,removed_leaf_memberships,changed_point_fraction,changed_membership_fraction,pose_pipe,z_pipe,loss0,loss1,loss2,loss3,loss4,loss5,loss6,loss7");
    CsvOut pair(out+"/WEAK_ONLY_VS_COUPLED.csv","dataset,frame_key,alpha,status,weak_status,coupled_status,exploratory_probe,formal_weak_count,minimum_eigenvalue_ratio,weak_pose,coupled_pose,z_weak,z_coupled,local_delta_weak,local_delta_coupled,dynamic_delta_weak,dynamic_delta_coupled,weak_translation_from_T0_m,weak_rotation_from_T0_deg,coupled_translation_from_T0_m,coupled_rotation_from_T0_deg,weak_to_coupled_translation_m,weak_to_coupled_rotation_deg,Hsw_norm,strong_response_norm,trust_scale,D_weak_vs_T0,D_coupled_vs_T0,weak_formal_eligible,coupled_formal_eligible,weak_heldout_mean,coupled_heldout_mean,row_kind");
    CsvOut dynamic(out+"/DYNAMIC_PCL_OBJECTIVE.csv","dataset,frame_key,raw_index,alpha,method,F_fixed_association,F_dynamic_PCL,fixed_delta_from_T0,dynamic_delta_from_T0,dynamic_minus_fixed,changed_training_points,total_training_points,added_leaf_memberships,removed_leaf_memberships,changed_point_fraction,changed_membership_fraction");
    CsvOut heldout(out+"/HELDOUT_COMPARISON.csv","dataset,frame_key,alpha,method,block,block_points,active,loss");
    CsvOut runtime(out+"/RUNTIME_AND_MEMORY.csv","dataset,frame_key,raw_index,alpha,map_grid_build_ms,source_switch_ms,local_model_and_boundary_ms,geometry_ms,refinement_and_validation_ms,frame_wall_ms,peak_rss_KiB,new_NDT_aligns,EKF_runs,GT_pose_reads,grid_fingerprint_unchanged,R15_fresh_grid_score_delta");
    const std::string m13="/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd";
    auto map13=loadMap(m13,459154,"ea1e4e69cf9992c6");pcl::KdTreeFLANN<pcl::PointXYZ> tree13;tree13.setInputCloud(map13);
    ExactPclObjective ndt13;const auto grid13_start=Clock::now();ndt13.configureMap(map13);const double grid13_ms=milliseconds(grid13_start);
    require(ndt13.frozenParameters(),"GEODE PCL parameter identity mismatch");
    for(size_t i=0;i<f13.size();++i)processFrame(f13[i],ndt13,grid13_ms,i==0,r15_nominal,associations,boundary,derivatives,coupling,candidates,pair,dynamic,heldout,runtime,tree13);
    const std::string m14="/home/jian/rosbag/public_datasets/newer_college_r14b/conditional_base_pilot_20261021/map/leica_roi50_leaf020.pcd";
    auto map14=loadMap(m14,224627,"28dc95023de5a54c");pcl::KdTreeFLANN<pcl::PointXYZ> tree14;tree14.setInputCloud(map14);
    ExactPclObjective ndt14;const auto grid14_start=Clock::now();ndt14.configureMap(map14);const double grid14_ms=milliseconds(grid14_start);
    require(ndt14.frozenParameters(),"Newer College PCL parameter identity mismatch");
    for(size_t i=0;i<f14.size();++i)processFrame(f14[i],ndt14,grid14_ms,i==0,r15_nominal,associations,boundary,derivatives,coupling,candidates,pair,dynamic,heldout,runtime,tree14);
    std::cout<<"R15B_PIECEWISE_COMPLETE frames="<<f13.size()+f14.size()<<" gaussian_grid_initializations=2 align=0 ekf=0 gt_reads=0\n";
  }catch(const std::exception& e){std::cerr<<"R15B_PIECEWISE_FAILED "<<e.what()<<'\n';return 1;}
  return 0;
}
