// Offline predictor-conditioned proposal gate. No estimator state is accessed.
#define P9_NDT_ENERGY_CONTRACT_LIBRARY
#include "p9_ndt_energy_contract.cpp"

#include <cstdio>
#include <tuple>
#include <unistd.h>

namespace {
const std::vector<std::uint64_t> frames32{120,244,368,616,740,838,839,864,924,925,1111,1235,
    1359,1497,1498,1556,1557,1606,1730,1854,2102,2226,2350,2598,2722,2846,3094,3217,3341,3631,3796,3962};
using Basis2 = Eigen::Matrix<double,6,2>;

template<class Derived> std::string values(const Eigen::MatrixBase<Derived>& v) {
  std::ostringstream out; out << std::setprecision(17);
  for (Eigen::Index i=0;i<v.size();++i) { if(i) out<<';'; out<<v.derived().coeff(i); }
  return out.str();
}
std::string matrixText(const Eigen::Matrix4f& pose) {
  std::ostringstream out; out<<std::setprecision(17);
  for(int i=0;i<16;++i){if(i)out<<';';out<<pose(i/4,i%4);} return out.str();
}
std::string poseText(const Eigen::Matrix4f& pose) {
  const Eigen::Quaterniond q=Eigen::Quaterniond(pose.block<3,3>(0,0).cast<double>()).normalized();
  Eigen::Matrix<double,7,1> v; v<<pose(0,3),pose(1,3),pose(2,3),q.x(),q.y(),q.z(),q.w();
  return values(v);
}
Eigen::Matrix4f generatorCarrier(const std::string& text) {
  const auto v=parseSemicolonDoubles(text,7);
  Eigen::Quaterniond q(v[6],v[3],v[4],v[5]);
  if(!q.coeffs().allFinite()||q.norm()<1e-12)throw std::runtime_error("invalid predictor");
  q.normalize(); Eigen::Matrix4f pose=Eigen::Matrix4f::Identity();
  pose.block<3,3>(0,0)=q.toRotationMatrix().cast<float>();
  pose.block<3,1>(0,3)=Eigen::Vector3d(v[0],v[1],v[2]).cast<float>();
  if(!pose.allFinite())throw std::runtime_error("nonfinite predictor pose");return pose;
}
Vector6d conditioned(const Vector6d& pred,const Vector6d& eta,const Basis2& q) {
  return pred+q*(q.transpose()*(eta-pred));
}
bool selfTest() {
  if(!runMathSelfTests())return false;
  const Basis2 q=Matrix6d::Identity().leftCols<2>();
  Vector6d pred,eta; pred<<.2,-.3,.4,.02,-.03,.04;eta<<2,-1,-.8,.1,-.2,.3;
  const auto cond=conditioned(pred,eta,q); const Matrix6d orth=Matrix6d::Identity()-q*q.transpose();
  if((orth*(cond-pred)).norm()>1e-12||(conditioned(pred,pred,q)-pred).norm()>1e-12||
     (cond-q*(q.transpose()*eta)).norm()<.1)return false;
  const auto nominal=generatorCarrier("10;-4;2;0.1;-0.2;0.3;0.9");
  const auto start=poseAtEta(nominal,cond), round=poseAtEta(nominal,mapChartDisplacement(nominal,start));
  return translationDistance(start,round)<=1e-5&&rotationDistanceDeg(start,round)<=1e-4;
}

void prepare(const std::string& cohortPath,const std::string& uobsPath,const std::string& candidatesPath,
             const std::string& randomPath,const std::string& directory) {
  const auto cohort=readCsv(cohortPath),candidates=readCsv(candidatesPath),random=readCsv(randomPath);
  const auto obs=readArchivedUObs(uobsPath);
  const auto sources=readFrameSources(cohortPath);
  std::map<std::uint64_t,Eigen::Matrix4f> predictors;
  std::map<std::uint64_t,std::string> predictorText;
  for(const auto& r:cohort.rows){const auto tx=parseU64(cohort.get(r,"transaction_id"));
    predictorText[tx]=cohort.get(r,"initial_pose_xyz_q_xyzw");}
  std::map<std::pair<std::uint64_t,int>,Basis2> randomBases;
  for(const auto& r:random.rows){
    const auto tx=parseU64(random.get(r,"frame")); const int rep=std::stoi(random.get(r,"random_rep"));
    const auto v=parseSemicolonDoubles(random.get(r,"basis_rowmajor"),12); Basis2 q;
    for(int i=0;i<12;++i)q(i/2,i%2)=v[i];
    if(rep<0||rep>=3||(q.transpose()*q-Eigen::Matrix2d::Identity()).norm()>1e-12||
       !randomBases.emplace(std::make_pair(tx,rep),q).second)throw std::runtime_error("invalid random basis");
  }
  if(sources.size()!=32||randomBases.size()!=96||candidates.rows.size()!=8800)
    throw std::runtime_error("frozen input count mismatch");
  for(const auto& r:candidates.rows){
    if(std::stoi(candidates.get(r,"seed_index"))==122){const auto tx=parseU64(candidates.get(r,"transaction_id"));
      for(const char* field:{"seed_dx_m","seed_dy_m","seed_dz_m","seed_roll_deg","seed_pitch_deg","seed_yaw_deg"})
        if(std::stod(candidates.get(r,field))!=0)throw std::runtime_error("seed122 is not zero perturbation");
      if(!predictors.emplace(tx,generatorCarrier(candidates.get(r,"start_pose_xyz_q_xyzw"))).second)
        throw std::runtime_error("duplicate predictor seed");
    }
  }
  std::ofstream parity(directory+"/predictor_parity.csv"),roundtrip(directory+"/proposal_roundtrip.csv"),
      pool(directory+"/conditioned_proposals.csv");
  if(!parity||!roundtrip||!pool)throw std::runtime_error("cannot open proposal outputs");
  for(auto* stream:{&parity,&roundtrip,&pool})stream->exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(parity,{"frame","seed_index","translation_error_m","rotation_error_deg","pass",
    "predictor_pose_matrix16","nominal_pose_matrix16","eta_pred","predictor_complement_norm"});
  writeHeader(roundtrip,{"frame","method","random_rep","seed_index","translation_error_m","rotation_error_deg",
    "complement_preservation_error","pass"});
  writeHeader(pool,{"frame","frame_id","method","random_rep","seed_index","coord0","coord1",
    "eta_original","eta_pred","eta_conditioned","nominal_pose_matrix16","predictor_pose_matrix16","start_pose_matrix16"});
  for(const auto tx:frames32){
    const auto& o=obs.at(tx); const auto& source=sources.at(tx);
    if(!o.valid||o.source_hash!=source.expected_hash||o.source_points!=source.expected_points||
       o.configured_resolution_m!=.8||o.step_size!=.08||o.epsilon!=1e-5||o.maximum_iterations!=80||
       (o.eigenvectors.transpose()*o.eigenvectors-Matrix6d::Identity()).norm()>1e-12||
       o.eigenvalues.minCoeff()<=0)throw std::runtime_error("U_obs/source contract mismatch");
    const auto& pred=predictors.at(tx); const auto expected=generatorCarrier(predictorText.at(tx));
    const double dt=translationDistance(pred,expected),dr=rotationDistanceDeg(pred,expected);
    const Vector6d ep=mapChartDisplacement(o.pose,pred);
    const auto w=o.eigenvectors.leftCols<2>().eval();
    const double norm=(ep-w*(w.transpose()*ep)).norm();
    writeRow(parity,{std::to_string(tx),"122",number(dt),number(dr),dt<=1e-5&&dr<=1e-4?"1":"0",
      matrixText(pred),matrixText(o.pose),values(ep),number(norm)});
    if(dt>1e-5||dr>1e-4)throw std::runtime_error("PREDICTOR_SEED122_PARITY_FAIL");
  }
  std::map<std::uint64_t,std::set<int>> seen;
  for(const auto& r:candidates.rows){
    const auto tx=parseU64(candidates.get(r,"transaction_id")); const int seed=std::stoi(candidates.get(r,"seed_index"));
    if(seed<0||seed>=263)continue;
    if(!seen[tx].insert(seed).second)throw std::runtime_error("duplicate base seed");
    const auto& o=obs.at(tx); const auto& pred=predictors.at(tx); const auto& source=sources.at(tx);
    const Vector6d ep=mapChartDisplacement(o.pose,pred);
    const auto archived=generatorCarrier(candidates.get(r,"start_pose_xyz_q_xyzw"));
    const Vector6d eta=mapChartDisplacement(o.pose,archived); const auto archiveRound=poseAtEta(o.pose,eta);
    if(translationDistance(archived,archiveRound)>1e-5||rotationDistanceDeg(archived,archiveRound)>1e-4)
      throw std::runtime_error("ARCHIVED_PROPOSAL_CHART_PARITY_FAIL");
    const auto emit=[&](const std::string& method,int rep,const Basis2& q){
      const Eigen::Vector2d c=q.transpose()*(eta-ep); const Vector6d cond=conditioned(ep,eta,q);
      const double complement=(cond-ep-q*(q.transpose()*(cond-ep))).norm();
      const auto start=seed==122?pred:poseAtEta(o.pose,cond);
      const auto round=poseAtEta(o.pose,mapChartDisplacement(o.pose,start));
      const double dt=translationDistance(start,round),dr=rotationDistanceDeg(start,round);
      const bool pass=dt<=1e-5&&dr<=1e-4&&complement<=1e-12;
      writeRow(roundtrip,{std::to_string(tx),method,std::to_string(rep),std::to_string(seed),
        number(dt),number(dr),number(complement),pass?"1":"0"});
      if(!pass)throw std::runtime_error("CONDITIONED_PROPOSAL_PARITY_FAIL");
      writeRow(pool,{std::to_string(tx),source.frame_id,method,std::to_string(rep),std::to_string(seed),
        number(c(0)),number(c(1)),values(eta),values(ep),values(cond),matrixText(o.pose),matrixText(pred),matrixText(start)});
    };
    emit("COND_WEAK2",-1,o.eigenvectors.leftCols<2>());
    emit("COND_STRONG2",-1,o.eigenvectors.rightCols<2>());
    for(int rep=0;rep<3;++rep)emit("COND_RANDOM2",rep,randomBases.at({tx,rep}));
  }
  if(seen.size()!=32)throw std::runtime_error("missing frame proposals");
  for(const auto& entry:seen)if(entry.second.size()!=263)throw std::runtime_error("base seed coverage mismatch");
  std::cout<<"R2A_PREDICTOR_PARITY=PASS PROPOSAL_PARITY=PASS PROPOSALS=42080 NEW_NDT_CALLS=0\n";
}

void alignAll(const std::string& mapPath,const std::string& cohortPath,const std::string& uobsPath,
              const std::string& probesPath,const std::string& directory) {
  const auto cohort=readFrameSources(cohortPath);const auto obs=readArchivedUObs(uobsPath);
  const auto probes=readCsv(probesPath);
  using Key=std::tuple<std::uint64_t,std::string,int>;
  std::map<Key,std::vector<std::vector<std::string>>> groups;
  std::set<std::tuple<std::uint64_t,std::string,int,int>> seen;
  for(const auto& r:probes.rows){
    const auto tx=parseU64(probes.get(r,"frame"));const auto method=probes.get(r,"method");
    const int rep=std::stoi(probes.get(r,"random_rep")),rank=std::stoi(probes.get(r,"probe_rank"));
    if((method!="COND_WEAK2"&&method!="COND_STRONG2"&&method!="COND_RANDOM2")||
       (method=="COND_RANDOM2"?(rep<0||rep>=3):rep!=-1)||rank<1||rank>16||
       !seen.emplace(tx,method,rep,rank).second)throw std::runtime_error("invalid probe identity");
    groups[{tx,method,rep}].push_back(r);
  }
  if(probes.rows.size()!=2560||groups.size()!=160)throw std::runtime_error("probe count mismatch");
  for(auto& item:groups){auto& rows=item.second;
    std::sort(rows.begin(),rows.end(),[&](const auto&a,const auto&b){return std::stoi(probes.get(a,"probe_rank"))<std::stoi(probes.get(b,"probe_rank"));});
    if(rows.size()!=16||std::stoi(probes.get(rows.front(),"seed_index"))!=122)
      throw std::runtime_error("probes must start at seed122 and contain16 calls");
  }
  const auto target=loadTarget(mapPath);if(target->size()!=549606)throw std::runtime_error("target count mismatch");
  ExactPclNdt ndt;ndt.setResolution(.8f);ndt.setOulierRatio(.55);ndt.configureScoreConstants();
  ndt.setInputTarget(target);ndt.setStepSize(.08);ndt.setTransformationEpsilon(1e-5);ndt.setMaximumIterations(80);
  for(const float leaf:ndt.actualGridLeaf())if(std::abs(leaf-.8f)>1e-6)throw std::runtime_error("grid leaf mismatch");
  std::size_t calls=0;
  for(const auto tx:frames32){
    const auto& f=cohort.at(tx);const auto source=preprocessSource(loadPackedSource(f.path));
    if(source->size()!=f.expected_points||sourceHash(*source)!=f.expected_hash)throw std::runtime_error("source hash/count mismatch");
    ndt.setInputSource(source);const auto nominal=obs.at(tx).pose;
    // Same normalized Pose3d numerical carrier used to archive the original raw objective.
    const double nominalScore=ndt.dynamicValueOnly(source,generatorCarrier(poseText(nominal)));
    for(const auto& item:groups){if(std::get<0>(item.first)!=tx)continue;
      const auto& method=std::get<1>(item.first);const int rep=std::get<2>(item.first);
      const auto path=directory+"/runs/tx_"+std::to_string(tx)+"_"+method+"_r"+std::to_string(rep)+".csv";
      std::ifstream old(path);if(old.good())throw std::runtime_error("existing NDT shard; refusing repeat: "+path);
      const auto partial=path+".partial."+std::to_string(getpid());std::ofstream out(partial);
      if(!out)throw std::runtime_error("cannot create shard");out.exceptions(std::ios::badbit|std::ios::failbit);
      writeHeader(out,{"frame","frame_id","method","random_rep","probe_rank","seed_index","start_pose_matrix16",
        "terminal_pose_matrix16","terminal_pose_xyz_q_xyzw","converged","iterations","runtime_ms","raw_ndt_score_sum",
        "nominal_ndt_score_sum","delta_score_sum","translation_from_nominal_m","rotation_from_nominal_deg",
        "source_points","source_hash","target_points","status"});
      for(const auto& r:item.second){
        const auto start=parseMatrix16(probes.get(r,"start_pose_matrix16"));Cloud aligned;
        const auto begin=std::chrono::steady_clock::now();ndt.align(aligned,start);
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
        const auto terminal=ndt.getFinalTransformation();const int iterations=ndt.getFinalNumIteration();
        const bool converged=ndt.hasConverged();const auto text=poseText(terminal);
        const double score=ndt.dynamicValueOnly(source,generatorCarrier(text));
        if(!terminal.allFinite()||!std::isfinite(score)||!std::isfinite(nominalScore)||iterations<0||iterations>80||!std::isfinite(ms))
          throw std::runtime_error("invalid NDT result");
        writeRow(out,{std::to_string(tx),f.frame_id,method,std::to_string(rep),probes.get(r,"probe_rank"),
          probes.get(r,"seed_index"),matrixText(start),matrixText(terminal),text,converged?"1":"0",
          std::to_string(iterations),number(ms),number(score),number(nominalScore),number(score-nominalScore),
          number(translationDistance(nominal,terminal)),number(rotationDistanceDeg(nominal,terminal)),
          std::to_string(source->size()),std::to_string(sourceHash(*source)),std::to_string(target->size()),
          iterations>=80?"ITERATION_LIMIT":(converged?"SUCCESS":"NOT_CONVERGED")});++calls;
      }
      out.close();if(std::rename(partial.c_str(),path.c_str())!=0)throw std::runtime_error("cannot finalize shard");
      std::cerr<<"R2A_SHARD_COMPLETE frame="<<tx<<" method="<<method<<" rep="<<rep<<" calls=16\n";
    }
  }
  if(calls!=2560)throw std::runtime_error("new NDT count mismatch");std::cout<<"R2A_ALIGN_COMPLETE CALLS="<<calls<<'\n';
}
}
int main(int argc,char**argv){
  try{
    if(argc==2&&std::string(argv[1])=="--self-test"){
      if(!selfTest())throw std::runtime_error("proposal self-test failed");std::cout<<"P9_R2A_PROPOSAL_SELF_TEST=PASS\n";return 0;
    }
    if(argc==7&&std::string(argv[1])=="--prepare")prepare(argv[2],argv[3],argv[4],argv[5],argv[6]);
    else if(argc==7&&std::string(argv[1])=="--align-all")alignAll(argv[2],argv[3],argv[4],argv[5],argv[6]);
    else throw std::runtime_error("usage: --prepare COHORT UOBS CANDIDATES RANDOM OUTDIR | --align-all MAP COHORT UOBS PROBES OUTDIR | --self-test");
    return 0;
  }catch(const std::exception&e){std::cerr<<"P9_R2A_CONTRACT_FAIL="<<e.what()<<'\n';return 1;}
}
