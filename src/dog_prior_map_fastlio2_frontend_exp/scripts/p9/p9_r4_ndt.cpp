// Offline R4 engine. Frozen R2A helpers are included verbatim, never modified.
#define main p9_r2a_unused_main
#include "p9_r2a_predictor_search.cpp"
#undef main
#include "p9_r4_frozen_seeds.hpp"
#include <dirent.h>
#include <fcntl.h>

namespace {
std::vector<r4_frozen_seed::Seed> base263() {
  auto seeds=r4_frozen_seed::planarSeeds(.8);
  const auto axial=r4_frozen_seed::axialSeeds(.8);
  seeds.insert(seeds.end(),axial.begin(),axial.end());
  if(seeds.size()!=263||seeds[122].dx!=0||seeds[122].dy!=0||seeds[122].yaw_deg!=0)
    throw std::runtime_error("frozen BASE263 contract mismatch");
  return seeds;
}
std::string historicalPoseText(const dog_prior_map_fastlio2_frontend_exp::Pose3d& pose) {
  std::ostringstream out; r4_frozen_seed::poseString(out,pose); return out.str();
}
std::string historicalMatrixText(const dog_prior_map_fastlio2_frontend_exp::Pose3d& pose) {
  std::ostringstream out; r4_frozen_seed::matrixString(out,pose); return out.str();
}
void markFrameStarted(const std::string& directory,uint64_t tx) {
  const auto name="tx_"+std::to_string(tx)+".csv";
  DIR* dir=opendir(directory.c_str());if(!dir)throw std::runtime_error("missing shard directory");
  bool existing=false;
  while(const auto* item=readdir(dir)) {
    const std::string found=item->d_name;
    if(found==name||found.compare(0,name.size()+9,name+".partial.")==0||found==name+".started")existing=true;
  }
  closedir(dir);if(existing)throw std::runtime_error("existing completed/partial/started frame; refusing repeated NDT calls");
  const auto marker=directory+"/"+name+".started";
  const int fd=open(marker.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
  if(fd<0)throw std::runtime_error("cannot exclusively mark NDT frame started");close(fd);
}
bool recoveryGuardSelfTest() {
  char pattern[]="/tmp/p9_r4_recovery_self_test.XXXXXX";
  const char* created=mkdtemp(pattern);if(!created)return false;
  const std::string directory=created,partial=directory+"/tx_1.csv.partial.fixture";
  {std::ofstream fixture(partial);fixture<<"no NDT calls\n";}
  bool rejected=false;try{markFrameStarted(directory,1);}catch(const std::runtime_error&){rejected=true;}
  unlink(partial.c_str());rmdir(directory.c_str());return rejected;
}
std::string terminalStatus(bool converged,int iterations) {
  if(!converged)return "NOT_CONVERGED";
  if(iterations<=0)return "ZERO_ITERATION_PASSTHROUGH";
  if(iterations>=80)return "ITERATION_LIMIT";
  return "SUCCESS";
}
Eigen::Matrix4f historicalAlignCarrier(const dog_prior_map_fastlio2_frontend_exp::Pose3d& pose) {
  Eigen::Matrix4d matrix=Eigen::Matrix4d::Identity();
  matrix.block<3,3>(0,0)=pose.orientation.toRotationMatrix();
  matrix.block<3,1>(0,3)=pose.position; return matrix.cast<float>();
}
void poolR4(const std::string& cohortPath,const std::string& obsPath,const std::string& outputPath) {
  const auto cohort=readCsv(cohortPath); const auto obs=readArchivedUObs(obsPath); const auto seeds=base263();
  std::ofstream out(outputPath); if(!out)throw std::runtime_error("cannot create proposal pool");
  out.exceptions(std::ios::badbit|std::ios::failbit);
  writeHeader(out,{"frame","frame_id","method","random_rep","seed_index","seed_domain",
    "seed_dx_m","seed_dy_m","seed_dz_m","seed_roll_deg","seed_pitch_deg","seed_yaw_deg",
    "wide_radius_m","wide_angle_deg","grid_ix","grid_iy","grid_iyaw",
    "start_pose_xyz_q_xyzw","oracle_start_pose_matrix16","coord0","coord1","eta_original","eta_pred",
    "eta_conditioned","nominal_pose_matrix16","predictor_pose_matrix16","start_pose_matrix16",
    "roundtrip_translation_m","roundtrip_rotation_deg","complement_preservation_error"});
  std::set<uint64_t> seen;
  for(const auto& row:cohort.rows) {
    const auto tx=parseU64(cohort.get(row,"transaction_id")); const auto& o=obs.at(tx);
    if(!seen.insert(tx).second||!o.valid||o.source_hash!=parseU64(cohort.get(row,"prepared_source_hash"))||
       o.source_points!=parseU64(cohort.get(row,"prepared_source_point_count"))||o.target_points!=549606||
       o.length_scale_m!=static_cast<double>(.8f)||o.configured_resolution_m!=.8||o.step_size!=.08||o.epsilon!=1e-5||
       o.maximum_iterations!=80||(o.eigenvectors.transpose()*o.eigenvectors-Matrix6d::Identity()).norm()>1e-12||
       o.eigenvalues.minCoeff()<=0)throw std::runtime_error("invalid frozen U_obs/source configuration tx="+std::to_string(tx));
    const auto prediction=r4_frozen_seed::parsePose(cohort.get(row,"initial_pose_xyz_q_xyzw"));
    const auto zero=r4_frozen_seed::fromMatrix(r4_frozen_seed::rightPerturb(prediction,seeds[122]));
    const auto pred=generatorCarrier(historicalPoseText(zero)); const auto ep=mapChartDisplacement(o.pose,pred);
    if(translationDistance(pred,generatorCarrier(cohort.get(row,"initial_pose_xyz_q_xyzw")))>1e-5||
       rotationDistanceDeg(pred,generatorCarrier(cohort.get(row,"initial_pose_xyz_q_xyzw")))>1e-4)
      throw std::runtime_error("predictor seed122 carrier parity failed");
    const Basis2 w=o.eigenvectors.leftCols<2>();
    for(int j=0;j<263;++j) {
      const auto& seed=seeds[j];
      const auto original=r4_frozen_seed::fromMatrix(r4_frozen_seed::rightPerturb(prediction,seed));
      const auto originalText=historicalPoseText(original);
      const auto carrier=generatorCarrier(originalText);
      const auto eta=mapChartDisplacement(o.pose,carrier); const auto cond=conditioned(ep,eta,w);
      const Eigen::Vector2d coord=w.transpose()*(eta-ep);
      const auto start=j==122?pred:poseAtEta(o.pose,cond);
      const auto round=poseAtEta(o.pose,mapChartDisplacement(o.pose,start));
      const double dt=translationDistance(round,start),dr=rotationDistanceDeg(round,start);
      const double complement=(cond-ep-w*(w.transpose()*(cond-ep))).norm();
      const auto originalRound=poseAtEta(o.pose,eta);
      if(dt>1e-5||dr>1e-4||complement>1e-12||translationDistance(originalRound,carrier)>1e-5||
         rotationDistanceDeg(originalRound,carrier)>1e-4)throw std::runtime_error("R4 proposal chart parity failed");
      writeRow(out,{std::to_string(tx),cohort.get(row,"frame_id"),"COND_WEAK2","-1",std::to_string(j),seed.domain,
        number(seed.dx),number(seed.dy),number(seed.dz),number(seed.roll_deg),number(seed.pitch_deg),number(seed.yaw_deg),
        number(seed.wide_radius_m),number(seed.wide_angle_deg),std::to_string(seed.grid_ix),std::to_string(seed.grid_iy),
        std::to_string(seed.grid_iyaw),originalText,matrixText(historicalAlignCarrier(original)),number(coord(0)),
        number(coord(1)),values(eta),values(ep),values(cond),matrixText(o.pose),matrixText(pred),matrixText(start),
        number(dt),number(dr),number(complement)});
    }
  }
  std::cout<<"R4_POOL_PASS frames="<<seen.size()<<" seeds_per_frame=263 NEW_NDT_CALLS=0\n";
}

void runR4(const std::string& kind,const std::string& mapPath,const std::string& cohortPath,
           const std::string& poolPath,const std::string& directory) {
  const auto cohort=readCsv(cohortPath),proposals=readCsv(poolPath);
  std::map<uint64_t,std::vector<std::vector<std::string>>> byFrame;
  for(const auto& row:proposals.rows)byFrame[parseU64(proposals.get(row,"frame"))].push_back(row);
  const int expected=kind=="oracle"?263:12;
  const auto target=loadTarget(mapPath);if(target->size()!=549606)throw std::runtime_error("target count mismatch");
  ExactPclNdt ndt;ndt.setResolution(.8f);ndt.setOulierRatio(.55);ndt.configureScoreConstants();
  ndt.setInputTarget(target);ndt.setStepSize(.08);ndt.setTransformationEpsilon(1e-5);ndt.setMaximumIterations(80);
  for(const float leaf:ndt.actualGridLeaf())if(std::abs(leaf-.8f)>1e-6)throw std::runtime_error("target leaf mismatch");
  std::set<uint64_t> seen;
  for(const auto& frame:cohort.rows) {
    const auto tx=parseU64(cohort.get(frame,"transaction_id"));
    if(!seen.insert(tx).second)throw std::runtime_error("duplicate input frame");
    auto rows=byFrame.at(tx);
    std::sort(rows.begin(),rows.end(),[&](const auto&a,const auto&b){
      const char* key=kind=="oracle"?"seed_index":"probe_rank";
      return std::stoi(proposals.get(a,key))<std::stoi(proposals.get(b,key));});
    if(int(rows.size())!=expected)throw std::runtime_error("wrong proposal call count");
    for(int i=0;i<expected;++i)if(std::stoi(proposals.get(rows[i],kind=="oracle"?"seed_index":"probe_rank"))!=
       (kind=="oracle"?i:i+1))throw std::runtime_error("wrong proposal order");
    if(kind!="oracle"&&std::stoi(proposals.get(rows[0],"seed_index"))!=122)
      throw std::runtime_error("candidate seed122 must be first");
    const auto source=preprocessSource(loadPackedSource(cohort.get(frame,"raw_cloud_file")));
    if(source->size()!=parseU64(cohort.get(frame,"prepared_source_point_count"))||
       sourceHash(*source)!=parseU64(cohort.get(frame,"prepared_source_hash")))
      throw std::runtime_error("source hash/count mismatch");
    ndt.setInputSource(source);
    const auto nominal=parseMatrix16(proposals.get(rows[0],"nominal_pose_matrix16"));
    const double nominalScore=ndt.dynamicValueOnly(source,generatorCarrier(poseText(nominal)));
    const auto path=directory+"/tx_"+std::to_string(tx)+".csv";
    markFrameStarted(directory,tx);
    const auto partial=path+".partial."+std::to_string(getpid());std::ofstream out(partial);
    if(!out)throw std::runtime_error("cannot create NDT shard");out.exceptions(std::ios::badbit|std::ios::failbit);
    auto header=proposals.header;
    std::vector<std::size_t> keep;
    if(kind=="oracle") {
      header={"frame","frame_id","seed_index","seed_domain","seed_dx_m","seed_dy_m","seed_dz_m",
        "seed_roll_deg","seed_pitch_deg","seed_yaw_deg","wide_radius_m","wide_angle_deg",
        "grid_ix","grid_iy","grid_iyaw","start_pose_xyz_q_xyzw","oracle_start_pose_matrix16"};
      for(const auto& key:header)keep.push_back(proposals.col(key));
    }
    for(const char* key:{"transaction_id","time_s","selection_labels","segment","final_pose_xyz_q_xyzw",
      "final_pose_matrix16","terminal_pose_matrix16","terminal_pose_xyz_q_xyzw","converged","iterations",
      "runtime_ms","raw_ndt_score_sum","nominal_ndt_score_sum","source_points","target_points",
      "source_hash_expected","source_hash_actual","source_hash","status","fitness","transformation_probability"})header.push_back(key);
    writeHeader(out,header);
    for(const auto& row:rows) {
      const auto start=parseMatrix16(proposals.get(row,kind=="oracle"?"oracle_start_pose_matrix16":"start_pose_matrix16"));
      Cloud aligned;const auto tick=std::chrono::steady_clock::now();ndt.align(aligned,start);
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
      const auto terminal=ndt.getFinalTransformation();const int iterations=ndt.getFinalNumIteration();
      const bool converged=ndt.hasConverged();const auto terminalPose=r4_frozen_seed::fromMatrix(terminal);
      const auto text=historicalPoseText(terminalPose),matrix=historicalMatrixText(terminalPose);
      const double score=ndt.dynamicValueOnly(source,generatorCarrier(text));
      if(!terminal.allFinite()||!std::isfinite(score)||!std::isfinite(nominalScore)||!std::isfinite(ms)||
         iterations<0||iterations>80)throw std::runtime_error("invalid NDT terminal");
      auto output=row;
      if(kind=="oracle") {output.clear();for(const auto col:keep)output.push_back(row.at(col));}
      const std::vector<std::string> extra={std::to_string(tx),cohort.get(frame,"time_s"),"LABEL_BLIND","Floor01",
        text,matrix,matrixText(terminal),text,converged?"1":"0",std::to_string(iterations),number(ms),number(score),
        number(nominalScore),std::to_string(source->size()),std::to_string(target->size()),
        std::to_string(sourceHash(*source)),std::to_string(sourceHash(*source)),std::to_string(sourceHash(*source)),
        terminalStatus(converged,iterations),
        number(ndt.getFitnessScore()),number(ndt.getTransformationProbability())};
      output.insert(output.end(),extra.begin(),extra.end());writeRow(out,output);
    }
    out.close();if(std::rename(partial.c_str(),path.c_str())!=0)throw std::runtime_error("shard finalize failed");
    std::cerr<<"R4_NDT_SHARD_COMPLETE kind="<<kind<<" frame="<<tx<<" calls="<<expected<<'\n';
  }
  std::cout<<"R4_NDT_COMPLETE kind="<<kind<<" calls="<<seen.size()*expected<<'\n';
}
}
int main(int argc,char**argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--self-test") {
      if(!selfTest()||base263().size()!=263||!recoveryGuardSelfTest()||
         terminalStatus(true,0)!="ZERO_ITERATION_PASSTHROUGH"||terminalStatus(false,80)!="NOT_CONVERGED")
        throw std::runtime_error("frozen R2A math/BASE263/recovery self-test failed");
      std::cout<<"P9_R4_NDT_PROPOSAL_SELF_TEST=PASS\n";
    } else if(argc==5&&std::string(argv[1])=="--pool")poolR4(argv[2],argv[3],argv[4]);
    else if(argc==6&&(std::string(argv[1])=="--oracle"||std::string(argv[1])=="--candidate"))
      runR4(std::string(argv[1]).substr(2),argv[2],argv[3],argv[4],argv[5]);
    else throw std::runtime_error("usage: --pool COHORT UOBS OUT | --oracle/--candidate MAP COHORT POOL DIR | --self-test");
    return 0;
  } catch(const std::exception&e) { std::cerr<<"P9_R4_INPUT_CONTRACT_FAIL="<<e.what()<<'\n';return 1; }
}
