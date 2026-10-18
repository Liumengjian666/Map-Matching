#include "corrected_ndt.hpp"
#include "metrics.hpp"
#include <pcl/io/pcd_io.h>
#include <Eigen/Cholesky>
#include <map>
#include <set>
#include <iostream>
#include <sys/stat.h>

using namespace screen;
using Row = std::map<std::string,std::string>;
std::vector<std::string> split(const std::string& line) {
  std::stringstream stream(line); std::string v; std::vector<std::string> out;
  while(std::getline(stream,v,',')) out.push_back(v); return out;
}
std::vector<Row> read(const std::string& file) {
  std::ifstream stream(file); std::string line;
  if(!std::getline(stream,line)) throw std::runtime_error("missing preregistration");
  const auto header=split(line); std::vector<Row> out;
  while(std::getline(stream,line)) {
    auto fields=split(line); if(fields.size()!=header.size()) throw std::runtime_error("CSV shape");
    Row r; for(size_t k=0;k<header.size();++k) r[header[k]]=fields[k]; out.push_back(r);
  }
  if(out.size()!=240) throw std::runtime_error("exactly 240 predeclared starts required");
  std::map<std::string,int> counts; std::set<std::string> ids;
  for(const auto& r:out) {++counts[r.at("scan_index")]; if(!ids.insert(r.at("candidate_id")).second) throw std::runtime_error("duplicate id");}
  if(counts.size()!=20) throw std::runtime_error("exactly 20 pairs required");
  for(const auto& r:counts) if(r.second!=12) throw std::runtime_error("12 starts per pair required");
  return out;
}
double number(const Row& r,const std::string& key) { return std::stod(r.at(key)); }
Eigen::Matrix4f matrix(const Row& r,const std::string& prefix) {
  Eigen::Matrix4f T;
  for(int i=0;i<4;++i) for(int j=0;j<4;++j) T(i,j)=number(r,prefix+std::to_string(i)+std::to_string(j));
  return T;
}
Cloud::Ptr load(const std::string& path) {
  if(path.find("/home/jian/")!=0) throw std::runtime_error("internal inputs only");
  Cloud::Ptr c(new Cloud); if(pcl::io::loadPCDFile(path,*c)) throw std::runtime_error("PCD unavailable"); return c;
}
CoupledLocalJet jetAt(CorrectedNdt& ndt,const Cloud::Ptr& src,const Eigen::Matrix4f& pose) {
  CoupledNdtBackend backend;
  backend.jet=[&](const Eigen::Matrix4f& p,const CoupledVector6& x){return ndt.nativeJet(src,p,x);};
  return coupledNominalJet(pose,src->size(),backend,.8);
}
void grid(Csv& csv,const Row& r,const CorrectedNdt& ndt,const std::string& stage) {
  requireCorrectedGrid(ndt); csv.col(r.at("candidate_id")); csv.col(stage);
  for(float leaf:ndt.targetGridLeafSizeMeters()) csv.col(leaf); csv.end();
}
void derivatives(Csv& csv,const Row& r,CorrectedNdt& ndt,const Cloud::Ptr& src,
                 const Eigen::Matrix4f& pose,bool run) {
  CoupledLocalJet jet; size_t support=0; const auto start=std::chrono::steady_clock::now();
  if(run && rigid(pose)) { jet=jetAt(ndt,src,pose); support=ndt.supported(src,pose); }
  Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig;
  if(jet.valid) eig.compute(jet.H);
  bool spd=jet.valid && eig.info()==Eigen::Success && eig.eigenvalues()[0]>0;
  double step=NAN,decrement=NAN,residual=NAN,scaled=NAN;
  if(spd) {
    Eigen::LLT<CoupledMatrix6> chol(jet.H); const CoupledVector6 d=chol.solve(jet.gradient);
    if(chol.info()==Eigen::Success && d.allFinite()) {
      step=d.norm(); decrement=std::sqrt(std::max(0.,jet.gradient.dot(d)));
      residual=(jet.H*d-jet.gradient).norm()/std::max(1.,jet.gradient.norm());
      scaled=jet.gradient.norm()/std::max(1.,eig.eigenvalues()[5]);
    }
  }
  const double fraction=double(support)/src->size();
  csv.col(r.at("candidate_id"));csv.col(run);csv.col(jet.valid);csv.col(spd);csv.col(support);csv.col(fraction);
  csv.col(jet.score);csv.col(jet.valid?-jet.score/src->size():NAN);
  csv.col(jet.valid?jet.gradient.norm():NAN);csv.col(scaled);csv.col(step);csv.col(decrement);csv.col(residual);
  csv.col(spd && fraction>=.5 && jet.gradient.norm()<=1e-3 && step<=1e-3 && residual<=1e-8);
  csv.col(ms(start));csv.col(run && rigid(pose)?1:0);
  for(int i=0;i<6;++i) csv.col(jet.valid?jet.gradient[i]:NAN);
  for(int i=0;i<6;++i) for(int j=0;j<6;++j) csv.col(jet.valid?jet.H(i,j):NAN);
  for(int i=0;i<6;++i) csv.col(jet.valid && eig.info()==Eigen::Success?eig.eigenvalues()[i]:NAN);
  csv.end();
}
int main(int argc,char** argv) {
  try {
    if(argc!=4) throw std::runtime_error("usage: basin_probe verify|align INITIAL_MANIFEST NEW_DIR");
    const std::string mode=argv[1]; if(mode!="verify" && mode!="align") throw std::runtime_error("unknown mode");
    const auto rows=read(argv[2]);
    if(::mkdir(argv[3],0775)!=0) throw std::runtime_error("new output directory required");
    const std::string out=argv[3];
    Csv result(out+"/alignments.csv"), deriv(out+"/derivatives.csv"), audit(out+"/grid.csv"), parity(out+"/nominal_parity.csv");
    for(auto k:{"scan_index","candidate_id","family","status","align_calls","converged","rigid","fitness","iterations","align_ms","build_ms","source_points","target_points","source_hash","target_hash"}) result.col(k);
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) result.col("pose"+std::to_string(i)+std::to_string(j));result.end();
    for(auto k:{"candidate_id","requested","jet_valid","spd","supported_points","support_fraction","score_sum","mean_energy","gradient_norm","scaled_gradient","newton_step_norm","newton_decrement","newton_residual","strict_stationary","jet_support_ms","jet_calls"}) deriv.col(k);
    for(int i=0;i<6;++i) deriv.col("g"+std::to_string(i));
    for(int i=0;i<6;++i) for(int j=0;j<6;++j) deriv.col("H"+std::to_string(i)+std::to_string(j));
    for(int i=0;i<6;++i) deriv.col("lambda"+std::to_string(i));deriv.end();
    for(auto k:{"candidate_id","stage","leaf_x","leaf_y","leaf_z"}) audit.col(k);audit.end();
    for(auto k:{"scan_index","source_hash_equal","target_hash_equal","energy_abs_diff","hessian_relative_diff","pass"}) parity.col(k);parity.end();
    int aligns=0, requests=0, pairs=0; std::string current;
    Cloud::Ptr src,dst; std::unique_ptr<CorrectedNdt> ndt; double build_ms=0;
    for(const auto& r:rows) {
      const bool new_pair=current!=r.at("scan_index");
      if(new_pair) {
        current=r.at("scan_index"); ++pairs;
        src=source(load(r.at("source_path")));dst=target(load(r.at("target_path")));
        if(hash(src)!=r.at("source_hash") || hash(dst)!=r.at("target_hash") ||
           src->size()!=std::stoul(r.at("source_points")) || dst->size()!=std::stoul(r.at("target_points"))) throw std::runtime_error("R2 processed input mismatch");
        ndt.reset(new CorrectedNdt); const auto start=std::chrono::steady_clock::now();
        ndt->configureTarget(dst);ndt->setInputSource(src);build_ms=ms(start);grid(audit,r,*ndt,"TARGET_BUILT");
        if(mode=="verify") {
          const auto nominal=matrix(r,"nominal"); const auto jet=jetAt(*ndt,src,nominal);++requests;
          CoupledMatrix6 H;
          for(int i=0;i<6;++i) for(int j=0;j<6;++j) H(i,j)=number(r,"nominal_H"+std::to_string(i)+std::to_string(j));
          const double E=std::abs(-jet.score/src->size()-number(r,"nominal_energy"));
          const double error=(jet.H-H).norm()/std::max(1.,H.norm());
          const bool ok=jet.valid && E<=1e-10 && error<=1e-8;
          parity.col(current);parity.col(1);parity.col(1);parity.col(E);parity.col(error);parity.col(ok);parity.end();
          grid(audit,r,*ndt,"AFTER_NOMINAL_JET");
          if(!ok) throw std::runtime_error("R2 nominal objective/Hessian parity failed");
        }
      }
      const auto initial=matrix(r,"initial"),nominal=matrix(r,"nominal");CoupledVector6 eta;
      for(int i=0;i<6;++i) eta[i]=number(r,"eta"+std::to_string(i));
      const bool ready=r.at("status")=="READY";
      if(!eta.allFinite() || !rigid(initial) ||
         (initial-coupledPoseAtEta(nominal,eta,.8)).cwiseAbs().maxCoeff()>1e-6 ||
         ready!=(eta.tail<3>().norm()*180./M_PI<=5.)) throw std::runtime_error("invalid frozen start/chart/OOB");
      if(mode=="verify") continue;
      Eigen::Matrix4f final=Eigen::Matrix4f::Constant(NAN); bool converged=false;double elapsed=0,fitness=NAN;int iterations=0;
      std::string status=r.at("status");
      if(ready) {
        grid(audit,r,*ndt,"BEFORE_ALIGN");Cloud output;const auto start=std::chrono::steady_clock::now();
        ++aligns;ndt->align(output,initial);elapsed=ms(start);final=ndt->getFinalTransformation();
        converged=ndt->hasConverged();fitness=ndt->getFitnessScore();iterations=ndt->getFinalNumIteration();
        grid(audit,r,*ndt,"AFTER_ALIGN");status=converged && rigid(final)?"COMPLETED":"NDT_NOT_CONVERGED_OR_NONRIGID";
      }
      result.col(current);result.col(r.at("candidate_id"));result.col(r.at("family"));result.col(status);
      result.col(ready?1:0);result.col(converged);result.col(ready && rigid(final));result.col(fitness);result.col(iterations);result.col(elapsed);result.col(new_pair?build_ms:0.);
      result.col(src->size());result.col(dst->size());result.col(hash(src));result.col(hash(dst));
      for(int i=0;i<4;++i) for(int j=0;j<4;++j) result.col(final(i,j));result.end();
      derivatives(deriv,r,*ndt,src,final,ready);if(ready && rigid(final)) ++requests;
      grid(audit,r,*ndt,"AFTER_DIAGNOSTIC");
      std::cout<<"COMPLETE "<<r.at("candidate_id")<<" aligns="<<aligns<<std::endl;
    }
    if(pairs!=20 || aligns>240) throw std::runtime_error("budget/order mismatch");
    std::cout<<"FINISHED mode="<<mode<<" pairs="<<pairs<<" align_calls="<<aligns<<" jet_requests="<<requests<<std::endl;
  } catch(const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
