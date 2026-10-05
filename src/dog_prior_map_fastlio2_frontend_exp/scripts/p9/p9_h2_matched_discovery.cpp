// Independent H2 runner: frozen archived proposals, no continuation or EKF.
#define P9_NDT_ENERGY_CONTRACT_LIBRARY
#include "p9_ndt_energy_contract.cpp"

namespace {
const std::set<std::uint64_t> h2Frames{368,616,2226,2350,2722,2846,3341,3796,3962};

Eigen::Matrix4f archivedPose(const std::string& text) {
  const auto f = parseSemicolonDoubles(text,7);
  Eigen::Quaterniond q(f[6],f[3],f[4],f[5]);
  if (!q.coeffs().allFinite() || q.norm()<1e-12) throw std::runtime_error("invalid archived quaternion");
  q.normalize();
  Eigen::Matrix4f p=Eigen::Matrix4f::Identity();
  p.block<3,3>(0,0)=q.toRotationMatrix().cast<float>();
  p.block<3,1>(0,3)=Eigen::Vector3d(f[0],f[1],f[2]).cast<float>();
  if (!p.allFinite()) throw std::runtime_error("invalid archived pose");
  return p;
}

std::string matrixText(const Eigen::Matrix4f& p) {
  std::ostringstream out; out<<std::setprecision(17);
  for(int i=0;i<16;++i){if(i)out<<';';out<<p(i/4,i%4);}return out.str();
}
std::string vectorText(const Eigen::VectorXd& v) {
  std::ostringstream out;out<<std::setprecision(17);
  for(int i=0;i<v.size();++i){if(i)out<<';';out<<v(i);}return out.str();
}
std::string poseText(const Eigen::Matrix4f& p) {
  const Eigen::Quaterniond q=Eigen::Quaterniond(p.block<3,3>(0,0).cast<double>()).normalized();
  Eigen::VectorXd v(7);v<<p(0,3),p(1,3),p(2,3),q.x(),q.y(),q.z(),q.w();return vectorText(v);
}

bool selfTest() {
  if(!runMathSelfTests())return false;
  const Eigen::Matrix4f base=archivedPose("12;-4;2;0.1;-0.2;0.3;0.9");
  Vector6d e;e<<1,-2,.3,.1,-.05,.08;
  const auto target=poseAtEta(base,e);
  const auto recovered=mapChartDisplacement(base,target);
  if((recovered-e).norm()>2e-6)return false;
  const auto round=poseAtEta(base,recovered);
  if(translationDistance(round,target)>1e-5||rotationDistanceDeg(round,target)>1e-4)return false;
  const Eigen::Matrix<double,6,2> w=Matrix6d::Identity().leftCols<2>();
  const Vector6d projected=w*(w.transpose()*e);
  return projected.norm()<=e.norm()&&(projected-w*(w.transpose()*projected)).norm()<1e-12;
}

void proposals(const std::string& uobsPath,const std::string& candidatePath,
               const std::string& randomPath,const std::string& directory) {
  const auto obs=readArchivedUObs(uobsPath);
  const auto candidates=readCsv(candidatePath), random=readCsv(randomPath);
  std::map<std::pair<std::uint64_t,int>,Eigen::Matrix<double,6,2>> randomBases;
  for(const auto& r:random.rows){
    const auto tx=parseU64(random.get(r,"frame"));const int rep=std::stoi(random.get(r,"random_rep"));
    const auto v=parseSemicolonDoubles(random.get(r,"basis_rowmajor"),12);
    Eigen::Matrix<double,6,2> q;for(int i=0;i<12;++i)q(i/2,i%2)=v[i];
    if(!h2Frames.count(tx)||rep<0||rep>=5||!q.allFinite()||
       (q.transpose()*q-Eigen::Matrix2d::Identity()).norm()>1e-12||
       !randomBases.emplace(std::make_pair(tx,rep),q).second)throw std::runtime_error("invalid random basis manifest");
  }
  if(randomBases.size()!=45)throw std::runtime_error("random basis count mismatch");
  std::ofstream output(directory+"/proposal_contract.csv"), parity(directory+"/projected_seed_parity.csv");
  if(!output||!parity)throw std::runtime_error("cannot open proposal outputs");
  output.exceptions(std::ios::badbit|std::ios::failbit);parity.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(output,{"frame","method","random_rep","seed_index","eta_original","eta_projected","start_pose_matrix16","archived_start_pose_xyz_q_xyzw"});
  writeHeader(parity,{"frame","seed_index","translation_error_m","rotation_error_deg","pass"});
  std::map<std::uint64_t,std::set<int>> seen;
  bool passed=true;double maxT=0,maxR=0;
  for(const auto& r:candidates.rows){
    const auto tx=parseU64(candidates.get(r,"transaction_id"));if(!h2Frames.count(tx))continue;
    const int seed=std::stoi(candidates.get(r,"seed_index"));
    if(seed<0||seed>=263||!seen[tx].insert(seed).second)throw std::runtime_error("candidate seed coverage mismatch");
    const auto& o=obs.at(tx);
    if(!o.valid||(o.eigenvectors.transpose()*o.eigenvectors-Matrix6d::Identity()).norm()>1e-12||
       o.eigenvalues.minCoeff()<=0)throw std::runtime_error("invalid frozen U_obs");
    const auto text=candidates.get(r,"start_pose_xyz_q_xyzw");
    const Eigen::Matrix4f start=archivedPose(text);
    const Vector6d eta=mapChartDisplacement(o.pose,start);
    const auto round=poseAtEta(o.pose,eta);
    const double dt=translationDistance(start,round),dr=rotationDistanceDeg(start,round);
    const bool pass=dt<=1e-5&&dr<=1e-4;passed=passed&&pass;maxT=std::max(maxT,dt);maxR=std::max(maxR,dr);
    writeRow(parity,{std::to_string(tx),std::to_string(seed),number(dt),number(dr),pass?"1":"0"});
    for(int method=0;method<8;++method){
      std::string name;int rep=-1;Vector6d e=eta;
      if(method==0)name="FULL6D";
      else {
        Eigen::Matrix<double,6,2> q;
        if(method==1){name="WEAK2";q=o.eigenvectors.leftCols<2>();}
        else if(method==2){name="STRONG2";q=o.eigenvectors.rightCols<2>();}
        else{name="RANDOM2";rep=method-3;q=randomBases.at({tx,rep});}
        e=q*(q.transpose()*eta);
        if(e.norm()>eta.norm()+1e-10)throw std::runtime_error("projection increases norm");
      }
      const auto p=method==0?start:poseAtEta(o.pose,e);
      writeRow(output,{std::to_string(tx),name,std::to_string(rep),std::to_string(seed),
          vectorText(eta),vectorText(e),matrixText(p),text});
    }
  }
  if(seen.size()!=9)throw std::runtime_error("frame coverage mismatch");
  for(const auto& entry:seen)if(entry.second.size()!=263)throw std::runtime_error("seed count mismatch");
  std::cout<<"ROUND_TRIP_PASS="<<passed<<" MAX_TRANSLATION_M="<<std::setprecision(17)<<maxT
           <<" MAX_ROTATION_DEG="<<maxR<<" PROPOSALS=18936 NEW_NDT_CALLS=0\n";
  if(!passed)throw std::runtime_error("MATCHED_PROPOSAL_CHART_PARITY_FAIL");
}

void alignProposals(const std::string& mapPath,const std::string& cohortPath,
                    const std::string& proposalPath,const std::string& method,int rep,
                    const std::string& outputPath) {
  if(method!="WEAK2"&&method!="STRONG2"&&method!="RANDOM2")throw std::runtime_error("FULL6D must be reused, not run");
  if((method=="RANDOM2"&&(rep<0||rep>=5))||(method!="RANDOM2"&&rep!=-1))throw std::runtime_error("invalid replicate");
  const auto frames=readFrameSources(cohortPath);const auto proposals=readCsv(proposalPath);
  Cloud::Ptr target=loadTarget(mapPath);if(target->size()!=549606)throw std::runtime_error("target count mismatch");
  ExactPclNdt ndt;ndt.setResolution(.8f);ndt.setOulierRatio(.55);ndt.configureScoreConstants();
  ndt.setInputTarget(target);ndt.setStepSize(.08);ndt.setTransformationEpsilon(1e-5);ndt.setMaximumIterations(80);
  const auto leaf=ndt.actualGridLeaf();for(float x:leaf)if(std::abs(x-.8f)>1e-6)throw std::runtime_error("target grid mismatch");
  std::ofstream output(outputPath);if(!output)throw std::runtime_error("cannot open run output");
  output.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(output,{"frame","method","random_rep","seed_index","start_pose_matrix16","terminal_pose_matrix16",
    "terminal_pose_xyz_q_xyzw","converged","iterations","runtime_ms","raw_ndt_score_sum","transformation_probability",
    "status","source_points","source_hash","target_points"});
  std::uint64_t current=0;Cloud::Ptr source;std::size_t count=0;
  std::set<std::pair<std::uint64_t,int>> seen;
  for(const auto& r:proposals.rows){
    if(proposals.get(r,"method")!=method||std::stoi(proposals.get(r,"random_rep"))!=rep)continue;
    const auto tx=parseU64(proposals.get(r,"frame"));const int seed=std::stoi(proposals.get(r,"seed_index"));
    if(!h2Frames.count(tx)||seed<0||seed>=263||!seen.emplace(tx,seed).second)throw std::runtime_error("proposal identity mismatch");
    if(tx!=current){
      source=preprocessSource(loadPackedSource(frames.at(tx).path));
      if(source->size()!=frames.at(tx).expected_points||sourceHash(*source)!=frames.at(tx).expected_hash)
        throw std::runtime_error("prepared source hash/count mismatch");
      current=tx;ndt.setInputSource(source);
    }
    const auto start=parseMatrix16(proposals.get(r,"start_pose_matrix16"));
    Cloud aligned;const auto begin=std::chrono::steady_clock::now();ndt.align(aligned,start);
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    const auto terminal=ndt.getFinalTransformation();const int iterations=ndt.getFinalNumIteration();
    const std::string terminalText=poseText(terminal);
    // Same archived raw objective: evaluate at the normalized terminal Pose3d carrier.
    const double score=ndt.dynamicValueOnly(source,archivedPose(terminalText));
    if(!terminal.allFinite()||!std::isfinite(score)||iterations<0||iterations>80||!std::isfinite(ms))
      throw std::runtime_error("invalid returned NDT terminal");
    const bool converged=ndt.hasConverged();
    const std::string status=iterations>=80?"ITERATION_LIMIT":iterations==0?"ZERO_ITERATION":converged?"SUCCESS":"NOT_CONVERGED";
    writeRow(output,{std::to_string(tx),method,std::to_string(rep),std::to_string(seed),matrixText(start),matrixText(terminal),
       terminalText,converged?"1":"0",std::to_string(iterations),number(ms),number(score),number(ndt.getTransformationProbability()),
       status,std::to_string(source->size()),std::to_string(sourceHash(*source)),std::to_string(target->size())});
    ++count;if(seed==262){output.flush();std::cerr<<"H2_FRAME="<<tx<<" METHOD="<<method<<" REP="<<rep<<" COMPLETED="<<count<<'\n';}
  }
  if(count!=2367)throw std::runtime_error("new alignment count mismatch");
  std::cout<<"METHOD="<<method<<" REP="<<rep<<" NEW_NDT_CALLS="<<count<<" COMPLETE=1\n";
}
} // namespace

int main(int argc,char** argv){
  try {
    if(argc==2&&std::string(argv[1])=="--self-test"){
      if(!selfTest())throw std::runtime_error("proposal self-test failed");std::cout<<"H2_PROPOSAL_SELF_TEST=PASS\n";return 0;
    }
    if(argc==6&&std::string(argv[1])=="--proposals")proposals(argv[2],argv[3],argv[4],argv[5]);
    else if(argc==8&&std::string(argv[1])=="--align")alignProposals(argv[2],argv[3],argv[4],argv[5],std::stoi(argv[6]),argv[7]);
    else throw std::runtime_error("usage: --proposals UOBS CANDIDATES RANDOM OUTDIR | --align MAP COHORT PROPOSALS METHOD REP OUTPUT | --self-test");
    return 0;
  }catch(const std::exception& e){std::cerr<<"H2_CONTRACT_FAIL="<<e.what()<<'\n';return 1;}
}
