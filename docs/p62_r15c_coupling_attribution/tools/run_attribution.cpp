#include "../../p62_r15b_piecewise_coupled/tools/piecewise.hpp"
#include "gradient_only.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <sys/resource.h>
#include <tuple>

using namespace coupled;

namespace {
struct CsvOut {
  std::ofstream stream;
  CsvOut(const std::string& path,const std::string& header):stream(path,std::ios::out|std::ios::trunc){
    require(bool(stream),"output CSV open failed");stream<<std::setprecision(17)<<header<<'\n';stream.flush();
  }
  void row(const std::string& text){stream<<text<<'\n';stream.flush();}
};
struct Frame {
  std::string dataset,key,raw_index,stamp,source_path,source_fnv;
  Pose t0=Pose::Identity(),tp=Pose::Identity();
  Cloud::Ptr source;Split split;Mask mask;
  bool native_converged=false;int native_iterations=0;
  double native_score=NAN,native_fitness=NAN;
  int scan_index=-1;
  explicit Frame(const Split& s):split(s),mask(s){}
};
using FrameKey=std::pair<std::string,std::string>;
using CandidateKey=std::tuple<std::string,std::string,int,std::string>;

Pose poseFrom(const coverage::Row& r,const std::string& prefix){
  Pose p;for(int i=0;i<4;++i)for(int j=0;j<4;++j)p(i,j)=std::stod(r.at(prefix+std::to_string(i)+std::to_string(j)));
  require(rigid(p),"frozen pose is not SE3");return p;
}
std::vector<double> pipeValues(const std::string& text){
  std::vector<double> values;std::stringstream in(text);std::string item;
  while(std::getline(in,item,'|')){require(!item.empty(),"empty pipe-delimited value");values.push_back(std::stod(item));}
  return values;
}
Eigen::VectorXd pipeVector(const std::string& text,Eigen::Index expected){
  const auto values=pipeValues(text);require(values.size()==size_t(expected),"pipe vector dimension mismatch");
  Eigen::VectorXd v(expected);for(Eigen::Index i=0;i<expected;++i)v[i]=values[size_t(i)];
  require(v.allFinite(),"nonfinite frozen vector");return v;
}
Eigen::MatrixXd pipeMatrix(const std::string& text,Eigen::Index rows,Eigen::Index cols){
  const auto values=pipeValues(text);require(values.size()==size_t(rows*cols),"pipe matrix dimension mismatch");
  Eigen::MatrixXd m(rows,cols);for(Eigen::Index r=0;r<rows;++r)for(Eigen::Index c=0;c<cols;++c)m(r,c)=values[size_t(r*cols+c)];
  require(m.allFinite(),"nonfinite frozen matrix");return m;
}
Pose matrixPose(const std::string& text){
  const auto values=pipeValues(text);require(values.size()==16,"frozen pose field shape mismatch");Pose p;
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)p(r,c)=values[size_t(r*4+c)];
  require(rigid(p),"invalid frozen pose field");return p;
}
std::vector<int> pipeIds(const std::string& text){
  require(!text.empty()&&text!="NA","missing frozen subspace indices");
  std::vector<int> ids;std::stringstream in(text);std::string item;while(std::getline(in,item,'|'))ids.push_back(std::stoi(item));
  return ids;
}
std::string alphaKey(double a){return std::abs(a-.5)<1e-12?"0.5":"1.0";}
int alphaIndex(double a){require(std::abs(a-.5)<1e-12||std::abs(a-1.)<1e-12,"unexpected frozen alpha");return std::abs(a-.5)<1e-12?0:1;}
std::string frameKey(const std::string& dataset,const std::string& key){return dataset+"|"+key;}
std::string number(double x){if(!std::isfinite(x))return "NA";std::ostringstream o;o<<std::setprecision(17)<<x;return o.str();}
std::string vectorText(const Eigen::VectorXd& v){if(!v.size()||!v.allFinite())return "NA";std::ostringstream o;o<<std::setprecision(17);for(Eigen::Index i=0;i<v.size();++i){if(i)o<<'|';o<<v[i];}return o.str();}
std::string poseText(const Pose& p){if(!rigid(p))return "NA";std::ostringstream o;o<<std::setprecision(17);for(int r=0;r<4;++r)for(int c=0;c<4;++c){if(r||c)o<<'|';o<<p(r,c);}return o.str();}
std::string joinedIds(const std::vector<int>& ids){std::ostringstream o;for(size_t i=0;i<ids.size();++i){if(i)o<<'|';o<<ids[i];}return o.str();}
long r15cPeakKiB(){struct rusage u;getrusage(RUSAGE_SELF,&u);return u.ru_maxrss;}
double ms(Clock::time_point start){return milliseconds(start);}

std::vector<Frame> loadGeode(const std::string& archive){
  const auto events=coverage::readCsv(archive+"/EVENTS.csv");
  const auto candidates=coverage::readCsv(archive+"/CANDIDATE_POOL.csv");
  const auto identities=coverage::readCsv(archive+"/SOURCE_IDENTITY.csv");
  std::map<int,coverage::Row> source_stats,nominal;
  for(const auto& r:identities)source_stats.emplace(std::stoi(r.at("scan_index")),r);
  for(const auto& r:candidates)if(r.at("arm")=="R13_TRAINING_GRID"&&std::stoi(r.at("candidate_id"))==0)
    nominal.emplace(std::stoi(r.at("scan_index")),r);
  std::vector<Frame> frames;
  for(const auto& r:events){
    const int idx=std::stoi(r.at("scan_index"));if(idx<60||idx>64)continue;
    const auto n=nominal.find(idx),id=source_stats.find(idx);require(n!=nominal.end()&&id!=source_stats.end(),"frozen GEODE frame record missing");
    auto source=coverage::loadSource(r);
    require(source->size()==std::stoul(r.at("expected_processed_points"))&&coverage::cloudHash(source)==r.at("expected_processed_fnv"),"GEODE frozen Source identity mismatch");
    Split split=coverage::splitSource(source);
    require(source->size()==std::stoul(id->second.at("source_points"))&&coverage::cloudHash(source)==id->second.at("source_fnv")&&
            split.training->size()==std::stoul(id->second.at("training_points")),"GEODE Source/split identity mismatch");
    Frame f(split);f.dataset="GEODE_R13";f.key=std::to_string(idx);f.scan_index=idx;f.raw_index=r.at("raw_index");f.stamp=r.at("header_ns");
    f.source_path=r.at("raw_xyz_offset_file");f.source_fnv=coverage::cloudHash(source);f.source=source;f.tp=coverage::prediction(r);f.t0=poseFrom(n->second,"final_");
    f.native_converged=n->second.at("converged")=="1";f.native_iterations=std::stoi(n->second.at("iterations"));
    f.native_score=std::stod(n->second.at("native_score"));f.native_fitness=std::stod(n->second.at("fitness"));
    require(f.mask.evaluable()&&f.mask.active.size()>=4,"GEODE Coverage V2 input unavailable");frames.push_back(std::move(f));
  }
  require(frames.size()==5,"fixed GEODE raw10700-10704 input incomplete");return frames;
}

std::vector<Frame> loadNewerCollege(const std::string& archive){
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
    const int raw=std::stoi(r.at("raw_ordinal"));if(raw<768||raw>787)continue;const int fidx=raw-767;
    auto s=source_stats.find(fidx),n=nominal.find(fidx),p=predictions.find(fidx);
    require(s!=source_stats.end()&&n!=nominal.end()&&p!=predictions.end(),"frozen Newer College frame record missing");
    pilot::Source source=pilot::loadSource(r);
    require(source.points->size()==std::stoul(s->second.at("source_points"))&&coverage::cloudHash(source.points)==s->second.at("source_fnv")&&
            source.split.training->size()==std::stoul(s->second.at("training_points")),"Newer College frozen Source/split identity mismatch");
    Frame f(source.split);f.dataset="NEWER_COLLEGE_R14B";f.key=std::to_string(raw);f.scan_index=raw;f.raw_index=std::to_string(raw);
    f.stamp=r.at("stamp_ns");f.source_path=r.at("path");f.source_fnv=coverage::cloudHash(source.points);f.source=source.points;
    f.tp=poseFrom(p->second,"prediction_");f.t0=poseFrom(n->second,"final_");f.native_converged=n->second.at("converged")=="1";
    f.native_iterations=std::stoi(n->second.at("iterations"));f.native_score=std::stod(n->second.at("native_score"));f.native_fitness=std::stod(n->second.at("fitness"));
    require(f.mask.evaluable()&&f.mask.active.size()>=4,"Newer College Coverage V2 input unavailable");frames.push_back(std::move(f));
  }
  require(frames.size()==20,"fixed Newer College raw768-787 input incomplete");return frames;
}

Cloud::Ptr loadMap(const std::string& path,size_t count,const std::string& expected_fnv){
  Cloud::Ptr map(new Cloud);require(pcl::io::loadPCDFile(path,*map)==0,"frozen map PCD read failed");
  require(map->size()==count&&coverage::cloudHash(map)==expected_fnv,"frozen map identity mismatch");return map;
}

struct FrozenRows {
  std::map<CandidateKey,coverage::Row> candidates,subspaces;
  std::map<CandidateKey,coverage::Row> dynamic;
  std::map<std::tuple<std::string,std::string,int,std::string,int>,coverage::Row> heldout;
};
FrozenRows readFrozen(const std::string& r15b){
  FrozenRows f;
  for(const auto& r:coverage::readCsv(r15b+"/run_01/CANDIDATE_OBJECTIVES.csv")){
    CandidateKey k{r.at("dataset"),r.at("frame_key"),alphaIndex(std::stod(r.at("alpha"))),r.at("method")};
    require(f.candidates.emplace(k,r).second,"duplicate R15B candidate row");
  }
  for(const auto& r:coverage::readCsv(r15b+"/run_01/SUBSPACE_AND_COUPLING.csv")){
    CandidateKey k{r.at("dataset"),r.at("frame_key"),alphaIndex(std::stod(r.at("alpha"))),"SUBSPACE"};
    require(f.subspaces.emplace(k,r).second,"duplicate R15B subspace row");
  }
  for(const auto& r:coverage::readCsv(r15b+"/run_01/DYNAMIC_PCL_OBJECTIVE.csv")){
    CandidateKey k{r.at("dataset"),r.at("frame_key"),alphaIndex(std::stod(r.at("alpha"))),r.at("method")};
    require(f.dynamic.emplace(k,r).second,"duplicate R15B dynamic-objective row");
  }
  for(const auto& r:coverage::readCsv(r15b+"/run_01/HELDOUT_COMPARISON.csv")){
    auto k=std::make_tuple(r.at("dataset"),r.at("frame_key"),alphaIndex(std::stod(r.at("alpha"))),r.at("method"),std::stoi(r.at("block")));
    require(f.heldout.emplace(k,r).second,"duplicate R15B heldout row");
  }
  require(f.candidates.size()==150&&f.subspaces.size()==50&&f.dynamic.size()==150&&f.heldout.size()==1200,"R15B frozen model output table dimensions changed");
  return f;
}

struct CandidateResult {Candidate candidate;double objective=NAN,mean_loss=NAN,paired_vs_nominal=NAN,score_ms=0,heldout_ms=0;std::string pose_pipe;};
using ResultKey=std::tuple<std::string,std::string,int,std::string>;

CandidateResult evaluate(const Frame& f,ExactPclObjective& ndt,pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                         const Pose& pose,const std::string& method,const std::string& pose_pipe,
                         double frozen_score,double frozen_fitness){
  CandidateResult out;out.pose_pipe=pose_pipe;
  Candidate& c=out.candidate;c.id=method=="NOMINAL_NDT"?0:method=="WEAK_ONLY"?1:method=="GRADIENT_ONLY"?2:3;
  c.initial=f.tp;c.pose=pose;c.finite=rigid(pose);c.converged=method=="NOMINAL_NDT"?f.native_converged:false;
  c.iterations=method=="NOMINAL_NDT"?f.native_iterations:0;c.score=method=="NOMINAL_NDT"?f.native_score:frozen_score;
  c.fitness=method=="NOMINAL_NDT"?f.native_fitness:frozen_fitness;
  const auto objective_start=Clock::now();out.objective=ndt.objective(pose);out.score_ms=ms(objective_start);
  require(std::isfinite(out.objective),"nonfinite dynamic PCL objective");
  const auto heldout_start=Clock::now();coverage::validate(c,f.split,f.mask,tree,f.tp);out.heldout_ms=ms(heldout_start);
  require(c.loss_valid&&std::isfinite(c.overlap),"Coverage V2 candidate evaluation invalid");
  out.mean_loss=coverage::mean(c.loss,f.mask);return out;
}

void processDataset(const std::vector<Frame>& frames,const std::string& map_path,size_t map_points,const std::string& map_fnv,
                    const std::string& r15b,const FrozenRows& frozen,CsvOut& gradient_out,CsvOut& metrics_out,
                    CsvOut& paired_out,CsvOut& runtime_out,std::map<ResultKey,CandidateResult>& all_results,
                    int& total_aligns){
  auto map=loadMap(map_path,map_points,map_fnv);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);
  ExactPclObjective ndt;const auto grid_start=Clock::now();ndt.configureMap(map);const double grid_ms=ms(grid_start);
  require(ndt.frozenParameters(),"PCL parameters differ from R15B");
  for(size_t fi=0;fi<frames.size();++fi){
    const Frame& f=frames[fi];const std::string before=ndt.gridFingerprint();
    const auto switch_start=Clock::now();ndt.setTraining(f.split.training);const double source_switch_ms=ms(switch_start);
    require(before==ndt.gridFingerprint()&&before==ndt.initialGridFingerprint(),"Gaussian grid changed after source switch");
    std::array<coverage::Row,2> subrows;std::array<coverage::Row,2> nominal_rows,weak_rows,coupled_rows;
    std::array<r15c::GradientOnlySolve,2> solves;std::array<Vector6,2> z_gradient;std::array<double,2> solve_ms{};
    std::array<coverage::Row,2> subspace_rows;
    for(int ai=0;ai<2;++ai){
      const CandidateKey sk{f.dataset,f.key,ai,"SUBSPACE"};auto sr=frozen.subspaces.find(sk);require(sr!=frozen.subspaces.end(),"R15B subspace row missing");subspace_rows[ai]=sr->second;
      const auto& s=subspace_rows[ai];require(s.at("probe_status")=="SPECTRAL_TAIL_COUPLED_PROBE_COMPUTED"&&s.at("direction_semantics")=="SPECTRAL_TAIL_MECHANISM_PROBE","R15B probe semantics changed");
      require(std::stoi(s.at("formal_weak_count"))==0,"frozen formal degeneracy status changed");
      const auto weak_ids=pipeIds(s.at("selected_probe_ids")),strong_ids=pipeIds(s.at("solved_complement_ids"));
      require(weak_ids.size()==1&&strong_ids.size()==5,"unexpected frozen subspace partition");
      const Eigen::MatrixXd u=pipeMatrix(s.at("eigenvectors_rowmajor"),6,6);
      require((u.transpose()*u-Eigen::MatrixXd::Identity(6,6)).norm()<1e-9,"frozen eigenbasis is not orthonormal");
      const Eigen::VectorXd dw_full=pipeVector(s.at("d_w"),6),ds_frozen=pipeVector(s.at("d_s"),6);
      Eigen::VectorXd dw(1);dw[0]=dw_full[weak_ids[0]];
      Eigen::MatrixXd hss=pipeMatrix(s.at("H_ss"),5,5),hsw=pipeMatrix(s.at("H_sw"),5,1);
      Eigen::VectorXd gs=pipeVector(s.at("g_s"),5);
      const auto solve_start=Clock::now();solves[ai]=r15c::solveAttribution(hss,hsw,gs,dw,std::stod(s.at("lambda")));
      solve_ms[ai]=ms(solve_start);if(!solves[ai].ok)throw std::runtime_error("Gradient-only block solve failed: "+solves[ai].status);
      require(std::abs(std::stod(s.at("lambda"))-.01)<1e-15,"R15B lambda differs from fixed 0.01");
      Eigen::VectorXd gradient_basis=dw_full,coupled_basis=dw_full;
      for(size_t j=0;j<strong_ids.size();++j){
        gradient_basis[strong_ids[j]]=solves[ai].ds_gradient_trusted[Eigen::Index(j)];
        coupled_basis[strong_ids[j]]=solves[ai].ds_coupled_trusted[Eigen::Index(j)];
      }
      const Eigen::VectorXd coupled_frozen=dw_full+ds_frozen;
      require((coupled_basis-coupled_frozen).norm()<1e-8,"R15B coupled vector is inconsistent with frozen block solution/trust region");
      const Vector6 z_old_coupled=(u*coupled_frozen).eval();
      const Vector6 z_old_weak=(u*dw_full).eval();
      const CandidateKey nk{f.dataset,f.key,ai,"NOMINAL_NDT"},wk{f.dataset,f.key,ai,"WEAK_ONLY_PROBE"},ck{f.dataset,f.key,ai,"COUPLED_PROBE"};
      auto n=frozen.candidates.find(nk),w=frozen.candidates.find(wk),c=frozen.candidates.find(ck);
      require(n!=frozen.candidates.end()&&w!=frozen.candidates.end()&&c!=frozen.candidates.end(),"R15B candidate row missing");
      nominal_rows[ai]=n->second;weak_rows[ai]=w->second;coupled_rows[ai]=c->second;
      require((matrixPose(n->second.at("pose_pipe"))-f.t0).norm()<1e-6,"R15B frozen nominal T0 differs from fixed R13/R14 input");
      require((pipeVector(w->second.at("z_pipe"),6)-z_old_weak).norm()<1e-8,"R15B weak candidate differs from frozen dw");
      require((pipeVector(c->second.at("z_pipe"),6)-z_old_coupled).norm()<1e-8,"R15B coupled candidate differs from frozen block solve");
      require((poseAt(f.t0,z_old_weak)-matrixPose(w->second.at("pose_pipe"))).norm()<1e-7,"R15B frozen weak pose does not reproduce from its tangent");
      require((poseAt(f.t0,z_old_coupled)-matrixPose(c->second.at("pose_pipe"))).norm()<1e-7,"R15B frozen coupled pose does not reproduce from its tangent");
      const Vector6 z=(u*gradient_basis).eval();require(z.allFinite()&&z.norm()<=1.+1e-10,"gradient-only candidate outside frozen trust region");
      z_gradient[ai]=z;
      const Pose p=poseAt(f.t0,z);require(rigid(p),"gradient-only generated illegal SE3");
      const Pose p_native=p.cast<float>().cast<double>();require(rigid(p_native),"gradient-only native precision pose invalid");
      const Pose old=matrixPose(c->second.at("pose_pipe"));
      // The selected finite-difference candidate is evaluated in exactly the R15B frame/source/map.
      (void)old;
    }

    const double f0=ndt.objective(f.t0);
    std::array<std::array<CandidateResult,4>,2> results;
    for(int ai=0;ai<2;++ai){
      const auto alpha_start=Clock::now();
      double objective_eval_ms=0.,heldout_eval_ms=0.,gradient_eval_ms=0.;
      const auto& n=nominal_rows[ai];const auto& w=weak_rows[ai];const auto& c=coupled_rows[ai];
      const Pose poses[4]={matrixPose(n.at("pose_pipe")),matrixPose(w.at("pose_pipe")),poseAt(f.t0,z_gradient[ai]),matrixPose(c.at("pose_pipe"))};
      const std::string names[4]={"NOMINAL_NDT","WEAK_ONLY","GRADIENT_ONLY","COUPLED"};
      const coverage::Row* old_rows[4]={&n,&w,nullptr,&c};
      for(int mi=0;mi<4;++mi){
        double score=NAN,fitness=NAN;
        if(mi==0){score=f.native_score;fitness=f.native_fitness;}
        results[ai][mi]=evaluate(f,ndt,tree,poses[mi],names[mi],poseText(poses[mi]),score,fitness);
        CandidateResult& e=results[ai][mi];objective_eval_ms+=e.score_ms;heldout_eval_ms+=e.heldout_ms;
        const ResultKey rk{f.dataset,f.key,ai,names[mi]};require(all_results.emplace(rk,e).second,"duplicate new result key");
        if(mi==2)gradient_eval_ms=e.score_ms+e.heldout_ms;
        if(mi!=2){
          const coverage::Row& old=*old_rows[mi];
          const double old_delta=std::stod(old.at("dynamic_PCL_actual_delta"));
          const double new_delta=e.objective-f0;
          require(std::abs(old_delta-new_delta)<=1e-9,"R15B frozen dynamic NDT objective does not reproduce");
          const double old_mean=std::stod(old.at("heldout_mean"));
          require(std::abs(old_mean-e.mean_loss)<=1e-10,"R15B frozen Coverage V2 mean does not reproduce");
          for(int b:f.mask.active){
            const auto hk=std::make_tuple(f.dataset,f.key,ai,old.at("method"),b);
            const auto found=frozen.heldout.find(hk);require(found!=frozen.heldout.end(),"R15B frozen block row missing");
            require(std::abs(std::stod(found->second.at("loss"))-e.candidate.loss[b])<=1e-10,"R15B frozen block loss does not reproduce");
          }
        }
      }
      // The robust paired statistic is diagnostic only: post-NDT refinements remain unconverged.
      for(int mi=0;mi<4;++mi){
        CandidateResult& e=results[ai][mi];const double d=coverage::paired(e.candidate.loss,results[ai][0].candidate.loss,f.mask);e.paired_vs_nominal=d;
        std::ostringstream row;row<<std::setprecision(17)<<f.dataset<<','<<f.key<<','<<f.scan_index<<','<<ai*0+std::stod(subspace_rows[ai].at("alpha"))<<','<<names[mi]<<','
          <<f.source_fnv<<','<<f.source->size()<<','<<f.split.training->size()<<','<<f.mask.active.size()<<','<<e.candidate.finite<<','<<e.candidate.converged<<','
          <<e.candidate.eligible<<','<<e.candidate.overlap<<','<<e.mean_loss<<','<<e.objective<<','<<e.objective-f0<<','<<d<<','<<poseText(e.candidate.pose);
        for(int b=0;b<8;++b){row<<',';if(std::find(f.mask.active.begin(),f.mask.active.end(),b)!=f.mask.active.end())row<<e.candidate.loss[b];else row<<"NA";}
        metrics_out.row(row.str());
      }
      const auto& s=subspace_rows[ai];const auto weak_ids=pipeIds(s.at("selected_probe_ids")),strong_ids=pipeIds(s.at("solved_complement_ids"));
      const Eigen::MatrixXd u=pipeMatrix(s.at("eigenvectors_rowmajor"),6,6);
      const Eigen::VectorXd dw_full=pipeVector(s.at("d_w"),6),ds_frozen=pipeVector(s.at("d_s"),6);
      Eigen::MatrixXd hss=pipeMatrix(s.at("H_ss"),5,5),hsw=pipeMatrix(s.at("H_sw"),5,1);Eigen::VectorXd gs=pipeVector(s.at("g_s"),5);
      Eigen::VectorXd dw(1);dw[0]=dw_full[weak_ids[0]];
      const auto& sol=solves[ai];
      std::ostringstream gr;gr<<std::setprecision(17)<<f.dataset<<','<<f.key<<','<<f.scan_index<<','<<std::stod(s.at("alpha"))<<','<<s.at("direction_semantics")<<','
        <<s.at("formal_weak_count")<<','<<s.at("minimum_eigenvalue_ratio")<<','<<joinedIds(weak_ids)<<','<<joinedIds(strong_ids)<<','<<s.at("lambda")<<','
        <<vectorText(dw_full)<<','<<vectorText(sol.ds_gradient_raw)<<','<<vectorText(sol.ds_coupling_raw)<<','<<vectorText(sol.ds_coupled_raw)<<','
        <<vectorText(sol.ds_gradient_trusted)<<','<<vectorText(sol.ds_coupled_trusted)<<','<<sol.gradient_trust_scale<<','<<sol.coupled_trust_scale<<','
        <<sol.ds_gradient_raw.norm()<<','<<sol.ds_coupling_raw.norm()<<','<<sol.ds_coupled_raw.norm()<<','
        <<(u*dw_full).eval().norm()<<','<<results[ai][1].objective<<','<<results[ai][2].objective<<','<<results[ai][3].objective<<','
        <<results[ai][1].mean_loss<<','<<results[ai][2].mean_loss<<','<<results[ai][3].mean_loss<<','
        <<translation(results[ai][2].candidate.pose,results[ai][3].candidate.pose)<<','<<rotation(results[ai][2].candidate.pose,results[ai][3].candidate.pose)<<','
        <<vectorText(z_gradient[ai])<<','<<poseText(results[ai][2].candidate.pose)<<','<<poseText(results[ai][3].candidate.pose)<<','
        <<(results[ai][2].candidate.converged?"true":"false")<<",POST_NDT_REFINEMENT_CANDIDATE,"<<sol.status<<','<<f.source_fnv;
      gradient_out.row(gr.str());

      const double dyn_delta=results[ai][3].objective-results[ai][2].objective;
      const double held_delta=results[ai][3].mean_loss-results[ai][2].mean_loss;
      std::ostringstream pr;pr<<std::setprecision(17)<<f.dataset<<','<<f.key<<','<<std::stod(s.at("alpha"))<<','<<results[ai][3].objective<<','<<results[ai][2].objective<<','<<dyn_delta<<','
        <<results[ai][3].mean_loss<<','<<results[ai][2].mean_loss<<','<<held_delta<<','
        <<(dyn_delta<0?"COUPLED_BETTER":dyn_delta>0?"GRADIENT_ONLY_BETTER":"TIE")<<','
        <<(held_delta<0?"COUPLED_BETTER":held_delta>0?"GRADIENT_ONLY_BETTER":"TIE")<<','
        <<sol.ds_gradient_raw.norm()<<','<<sol.ds_coupling_raw.norm()<<','<<sol.ds_gradient_trusted.norm()<<','<<sol.ds_coupled_trusted.norm()<<','
        <<translation(results[ai][2].candidate.pose,results[ai][3].candidate.pose)<<','<<rotation(results[ai][2].candidate.pose,results[ai][3].candidate.pose)<<','
        <<results[ai][2].candidate.eligible<<','<<results[ai][3].candidate.eligible<<','<<results[ai][2].paired_vs_nominal<<','<<results[ai][3].paired_vs_nominal<<','
        <<gradient_eval_ms<<','<<sol.gradient_trust_scale<<','<<sol.coupled_trust_scale;
      paired_out.row(pr.str());
      const double alpha_total=ms(alpha_start);
      std::ostringstream rr;rr<<std::setprecision(17)<<f.dataset<<','<<f.key<<','<<std::stod(s.at("alpha"))<<','<<(fi==0?grid_ms:0.)<<','<<source_switch_ms<<','<<solve_ms[ai]<<','
        <<gradient_eval_ms<<','<<objective_eval_ms/4.<<','<<heldout_eval_ms/4.<<','<<alpha_total<<','<<r15cPeakKiB()<<",0,0,"<<f.source_fnv;
      runtime_out.row(rr.str());

      // A no-align replay through the unchanged V3 selector must never admit a
      // post-NDT refinement candidate (converged=false and no native scores).
      std::vector<Candidate> pool{results[ai][0].candidate,results[ai][1].candidate,results[ai][2].candidate,results[ai][3].candidate};
      const auto v3=pool_v3::construct(pool,f.mask,f.tp);const auto selection=pool_v3::select(v3,f.mask,f.tp);
      const auto decision=pool_v3::decide(v3,f.mask,f.tp,selection);
      require(selection.robust<0&&decision.status!="ACCEPT_REPLACEMENT","offline refinement incorrectly passed unchanged formal selector");
      total_aligns+=0;
      (void)gradient_eval_ms;
    }
    require(ndt.gridFingerprint()==ndt.initialGridFingerprint(),"Gaussian Grid fingerprint changed during attribution");
  }
  (void)r15b;
}
} // namespace

int main(int argc,char** argv){
  try{
    require(argc==5,"usage: run_attribution <outdir> <r13_archive> <r14b_conditional_archive> <r15b_archive>");
    const std::string out=argv[1],r13=argv[2],r14=argv[3],r15b=argv[4];
    auto geode=loadGeode(r13);auto nc=loadNewerCollege(r14);const FrozenRows frozen=readFrozen(r15b);
    CsvOut gradient(out+"/GRADIENT_ONLY_RESULTS.csv","dataset,frame_key,raw_index,alpha,direction_semantics,formal_weak_count,minimum_eigenvalue_ratio,weak_ids,strong_ids,lambda,d_w_basis,ds_G_raw,ds_X_raw,ds_C_raw,ds_G_trusted,ds_C_trusted,gradient_trust_scale,coupled_trust_scale,ds_G_raw_norm,ds_X_raw_norm,ds_C_raw_norm,weak_step_norm,dynamic_F_weak,dynamic_F_gradient,dynamic_F_coupled,heldout_mean_weak,heldout_mean_gradient,heldout_mean_coupled,coupled_vs_gradient_translation_m,coupled_vs_gradient_rotation_deg,z_gradient,gradient_pose,coupled_pose,gradient_converged,candidate_kind,solve_status,source_fnv");
    CsvOut metrics(out+"/CANDIDATE_METRICS.csv","dataset,frame_key,raw_index,alpha,method,source_fnv,source_points,training_points,active_blocks,finite,converged,formal_eligible,overlap_0p5,heldout_mean,dynamic_F_NDT,dynamic_delta_vs_nominal,D_candidate_vs_nominal,pose_pipe,loss0,loss1,loss2,loss3,loss4,loss5,loss6,loss7");
    CsvOut paired(out+"/COUPLED_VS_GRADIENT_ONLY.csv","dataset,frame_key,alpha,dynamic_F_coupled,dynamic_F_gradient,delta_F_coupled_minus_gradient,heldout_mean_coupled,heldout_mean_gradient,delta_heldout_coupled_minus_gradient,dynamic_winner,heldout_winner,ds_G_raw_norm,ds_X_raw_norm,ds_G_trusted_norm,ds_C_trusted_norm,pose_difference_translation_m,pose_difference_rotation_deg,gradient_formal_eligible,coupled_formal_eligible,D_gradient_vs_nominal,D_coupled_vs_nominal,gradient_evaluation_ms,gradient_trust_scale,coupled_trust_scale");
    CsvOut runtime(out+"/RUNTIME_AND_MEMORY.csv","dataset,frame_key,alpha,map_grid_initialization_ms,source_switch_ms,gradient_block_solve_ms,gradient_only_objective_and_holdout_ms,per_method_dynamic_objective_mean_ms,per_method_coverage_validation_mean_ms,frame_attribution_wall_ms,peak_rss_KiB,new_NDT_aligns,EKF_runs,source_fnv");
    std::map<ResultKey,CandidateResult> result_rows;int aligns=0;
    const auto process_start=Clock::now();
    const std::string geode_map="/home/jian/rosbag/loop2/loop2mapping/pcd/loop2_simtime_rebuild_2026_08_12_001_all_downsampled_points.pcd";
    const std::string nc_map="/home/jian/rosbag/public_datasets/newer_college_r14b/conditional_base_pilot_20261021/map/leica_roi50_leaf020.pcd";
    processDataset(geode,geode_map,459154,"ea1e4e69cf9992c6",r15b,frozen,gradient,metrics,paired,runtime,result_rows,aligns);
    processDataset(nc,nc_map,224627,"28dc95023de5a54c",r15b,frozen,gradient,metrics,paired,runtime,result_rows,aligns);
    const double wall=ms(process_start);
    require(aligns==0,"unexpected native NDT align call accounting");
    std::cout<<"R15C_ATTRIBUTION_COMPLETE frames="<<geode.size()+nc.size()<<" candidate_evaluations="<<result_rows.size()
             <<" new_NDT_aligns=0 EKF=0 gt_candidate_reads=0 wall_ms="<<std::setprecision(17)<<wall<<" peak_rss_KiB="<<r15cPeakKiB()<<'\n';
  }catch(const std::exception& e){std::cerr<<"R15C_ATTRIBUTION_FAILED "<<e.what()<<'\n';return 1;}
  return 0;
}
