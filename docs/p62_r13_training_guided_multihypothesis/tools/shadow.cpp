#include "coarse.hpp"
#include "../../p62_r9_coverage_aware_recovery_v2/tools/io.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/native_capture.hpp"
#include "../../p62_r12_feasibility_preserving_pool/tools/pool_v3.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sys/resource.h>
using namespace coverage;
using namespace feasibility;
using training_grid::Seed;
Pose matrix(const Row& r,const std::string& prefix){Pose p;for(int a=0;a<4;++a)for(int b=0;b<4;++b)p(a,b)=std::stod(r.at(prefix+std::to_string(a)+std::to_string(b)));return p;}
std::string list(const std::vector<int>& v){std::string s;for(int x:v){if(!s.empty())s+='|';s+=std::to_string(x);}return s.empty()?"NA":s;}
struct EventData {
  Row row;Cloud::Ptr source;Split split;Mask mask;Pose prediction_pose;Candidate nominal;
  std::vector<Seed> seeds;std::vector<int> chosen;std::vector<size_t> subset;
  double preprocessing_ms=0,coarse_ms=0,selection_ms=0,coarse_phase_ms=0;size_t coarse_queries=0;
  EventData(const Row& r):row(r),source(loadSource(r)),split(splitSource(source)),mask(split),prediction_pose(prediction(r)){}
};
int main(int argc,char** argv){try{
  require(argc==6,"usage: shadow EVENTS MAP R11_DOC R12_DOC NEW_DIR");const std::string root=argv[5];require(::mkdir(argv[5],0775)==0,"exclusive formal run directory");
  auto rows=readCsv(argv[1]);require(rows.size()==5,"only5 events");
  const std::string r11=argv[3],r12=argv[4];auto original=readCsv(r11+"/CANDIDATE_POOL.csv"),cov=readCsv(r11+"/COVERAGE.csv");
  auto observations=readCsv(r11+"/FINAL_OBSERVATIONS.csv"),baseline=readCsv(r12+"/ACCEPTANCE_COMPARISON.csv");
  Csv inputs(root+"/SOURCE_IDENTITY.csv",coverageHeader());
  std::string gh="scan_index,raw_index,grid_id,dx_m,dy_m,yaw_deg,coarse_points,coarse_loss,coarse_overlap,local_excluded,selected_rank,reason,translation_from_prediction_m,rotation_from_prediction_deg";poseHeader(gh,"double_seed_");poseHeader(gh,"native_seed_");
  Csv grids(root+"/COARSE_GRID_SCORES.csv",gh),selected(root+"/SELECTED_SEEDS.csv",gh);
  Csv samples(root+"/COARSE_TRAINING_POINTS.csv","scan_index,training_index,training_sector,x_L,y_L,z_L");
  Csv map(root+"/MAP_RUNTIME.csv","map_points,target_points,target_fnv,actual_grid_m,load_grid_tree_ms");
  std::string ch="arm,scan_index,raw_index,candidate_id,source_kind,source_call_or_grid,finite,converged,iterations,native_score,fitness,overlap,mean_loss,eligible,duplicate_of,group_id,translation_from_prediction_m,rotation_from_prediction_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms";poseHeader(ch,"seed_");poseHeader(ch,"final_");
  Csv candidates(root+"/CANDIDATE_POOL.csv",ch),native(root+"/NATIVE_NDT_RESULTS.csv",ch);
  std::string raw_header="scan_index,candidate_id,new_call_id,grid_id,converged,iterations,native_score,align_ms";poseHeader(raw_header,"seed_");poseHeader(raw_header,"final_");
  Csv raw_results(root+"/RAW_OPTIMIZER_OUTPUTS.csv",raw_header);
  Csv calls(root+"/ALIGN_CALLS.csv","scan_index,candidate_id,grid_id,new_call_id,status");
  Csv blocks(root+"/HELDOUT_BLOCK_LOSSES.csv","scan_index,candidate_id,block_id,points,active,loss");
  Csv pairs(root+"/PAIRWISE_EVIDENCE.csv","scan_index,B,j,k,D_j_k");
  Csv selectors(root+"/SELECTOR_COMPARISON.csv","arm,scan_index,pool_ids,evidence_ids,native,heldout_mean,robust,selected,final_status,prediction_fallback,distinct_groups,admissible_distinct_groups,robust_distinct_preference,accepted_replacement");
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","scan_index,source_ms,training_points,coarse_points,coarse_NN_queries,coarse_45_ms,seed_selection_ms,coarse_phase_wall_ms,new_align_count,align_ms,native_capture_and_fitness_aux_ms,heldout_validation_ms,V3_decision_ms,extra_event_wall_ms,process_high_water_RSS_KiB");
  std::string final_header="scan_index,observation_valid";poseHeader(final_header,"accepted_");poseHeader(final_header,"prediction_");Csv final(root+"/FINAL_OBSERVATIONS.csv",final_header);
  Csv failures(root+"/FAILURES.csv","scan_index,candidate_id,reason");
  for(Csv* f:{&inputs,&grids,&selected,&samples,&map,&candidates,&native,&raw_results,&calls,&blocks,&pairs,&selectors,&runtime,&final,&failures})f->stream.exceptions(std::ios::badbit|std::ios::failbit);
  std::vector<EventData> events;
  // Verify all five exact Source/TP/T0 contracts before configuring or aligning NDT.
  for(int event=0;event<5;++event){const auto start=Clock::now();const auto& r=rows[event];const int i=60+event;
    require(std::stoi(r.at("scan_index"))==i&&std::stoi(r.at("raw_index"))==10700+event,"fixed event identity");
    EventData e(r);e.preprocessing_ms=milliseconds(start);
    require(e.source->size()==std::stoul(r.at("expected_processed_points"))&&cloudHash(e.source)==r.at("expected_processed_fnv"),"Source FNV/points mismatch STOP");
    const auto& c=cov.at(i-1);require(c.at("source_fnv")==cloudHash(e.source)&&std::stoul(c.at("training_points"))==e.split.training->size()&&std::stoi(c.at("B"))==int(e.mask.active.size())&&e.mask.evaluable(),"coverage provenance STOP");
    for(int b=0;b<8;++b)require(std::stoul(c.at("val"+std::to_string(b)))==e.split.blocks[b]->size(),"block count identity STOP");
    int nominal_count=0,prediction_count=0;
    for(const auto& old:original)if(old.at("arm")=="PROPOSED_DISAGREEMENT"&&std::stoi(old.at("scan_index"))==i&&old.at("candidate_id")=="0"){
      ++nominal_count;auto& n=e.nominal;n.id=0;n.pose=matrix(old,"final_");n.initial=matrix(old,"seed_");n.finite=std::stoi(old.at("finite"));n.converged=std::stoi(old.at("converged"));n.iterations=std::stoi(old.at("iterations"));n.score=std::stod(old.at("native_score"));n.fitness=std::stod(old.at("fitness"));n.overlap=std::stod(old.at("overlap"));
      require((n.initial-e.prediction_pose).norm()==0,"frozen causal prediction mismatch");
    }
    for(const auto& old:observations)if(old.at("arm")=="PROPOSED_DISAGREEMENT"&&std::stoi(old.at("scan_index"))==i){++prediction_count;require((matrix(old,"prediction_")-e.prediction_pose).norm()==0,"frozen prediction stamp/pose mismatch");}
    require(nominal_count==1&&prediction_count==1&&rigid(e.nominal.pose),"unique frozen nominal required");
    coverageRow(inputs,r,e.source,e.split,e.mask);events.push_back(e);
  }
  const auto map_start=Clock::now();Cloud::Ptr raw(new Cloud),finite(new Cloud);
  require(pcl::io::loadPCDFile(argv[2],*raw)==0,"frozen map load");for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);finish(finite);
  auto target=voxel(voxel(finite,.15f),.15f);require(raw->size()==459154&&target->size()==459154&&cloudHash(target)=="ea1e4e69cf9992c6","target identity mismatch STOP");
  NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  map.stream<<raw->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(map_start);map.end();
  // All225 scores and <=10 selected initial poses are fixed before ANY align.
  for(auto& e:events){const auto coarse_phase=Clock::now();const int i=std::stoi(e.row.at("scan_index"));e.seeds=training_grid::grid(e.prediction_pose);e.subset=training_grid::subset(*e.split.training);
    for(size_t k:e.subset){const auto& p=(*e.split.training)[k];samples.stream<<i<<','<<k<<','<<sector(p.x,p.y)<<','<<p.x<<','<<p.y<<','<<p.z;samples.end();}
    auto start=Clock::now();std::vector<int> ids(1);std::vector<float> squared(1);
    e.coarse_queries=training_grid::score(e.seeds,*e.split.training,e.subset,[&](const Eigen::Vector3d& y){require(tree.nearestKSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),1,ids,squared)==1,"coarse NN failed");return double(squared[0]);});
    e.coarse_ms=milliseconds(start);start=Clock::now();e.chosen=training_grid::select(e.seeds,e.nominal.pose);e.selection_ms=milliseconds(start);
    for(const auto& s:e.seeds){auto emit=[&](Csv& f){f.stream<<i<<','<<e.row.at("raw_index")<<','<<s.id<<','<<s.dx<<','<<s.dy<<','<<s.yaw_deg<<','<<e.subset.size()<<','<<s.loss<<','<<s.overlap<<','<<s.local_excluded<<','<<s.selected_rank<<','<<s.reason<<','<<translation(s.native_pose,e.prediction_pose)<<','<<rotation(s.native_pose,e.prediction_pose);emitPose(f.stream,s.pose);emitPose(f.stream,s.native_pose);f.end();};emit(grids);if(s.selected_rank)emit(selected);}
    e.coarse_phase_ms=milliseconds(coarse_phase);
  }
  std::cout<<"COARSE_SELECTION_FROZEN events=5 grids=225 new_aligns=0"<<std::endl;
  // Only now consume heldout losses, solely for final evidence, never coarse ordering.
  const auto losses=readCsv(r11+"/BLOCK_LOSSES.csv");
  for(auto& e:events){const int i=std::stoi(e.row.at("scan_index"));int n=0;
    for(const auto& r:losses)if(r.at("arm")=="PROPOSED_DISAGREEMENT"&&std::stoi(r.at("scan_index"))==i&&r.at("candidate_id")=="0"){
      const int b=std::stoi(r.at("block_id"));require(b>=0&&b<8&&std::stoul(r.at("points"))==e.split.blocks[b]->size(),"nominal block identity");e.nominal.loss[b]=r.at("loss")=="NA"?NAN:std::stod(r.at("loss"));++n;
    }
    e.nominal.loss_valid=e.mask.valid(e.nominal.loss);e.nominal.eligible=admissible(e.nominal,e.mask,e.prediction_pose);
    const auto event=detect(e.nominal,e.mask,e.prediction_pose,0);require(n==8&&event.reasonText()=="ROTATION_LIMIT|NOMINAL_ADMISSION_REJECTED"&&event.permitted,"frozen five event contract mismatch STOP");
  }
  int count=0;
  for(auto& e:events){const int i=std::stoi(e.row.at("scan_index"));const auto event_start=Clock::now();std::vector<Candidate> raw_candidates{e.nominal};
    std::vector<int> origins{2*i};double align_ms=0,fitness_ms=0,validation_ms=0;
    for(int grid_id:e.chosen){require(count<10,"R13 ten-align cap");const auto& seed=e.seeds[grid_id];const int id=raw_candidates.size(),call=++count;
      calls.stream<<i<<','<<id<<','<<grid_id<<','<<call<<",START";calls.end();const auto phase=Clock::now();
      auto capture=[&](const Candidate& c){raw_results.stream<<i<<','<<id<<','<<call<<','<<grid_id<<','<<c.converged<<','<<c.iterations<<',';number(raw_results.stream,c.score);raw_results.stream<<','<<c.align_ms;emitPose(raw_results.stream,c.initial);emitPose(raw_results.stream,c.pose);raw_results.end();};
      Candidate c;try{c=capturedAlign(ndt,e.split.training,seed.pose,id,capture);}catch(...){calls.stream<<i<<','<<id<<','<<grid_id<<','<<call<<",THREW";calls.end();throw;}
      const double native_phase_ms=milliseconds(phase);fitness_ms+=std::max(0.,native_phase_ms-c.align_ms);align_ms+=c.align_ms;
      calls.stream<<i<<','<<id<<','<<grid_id<<','<<call<<",RETURNED";calls.end();
      if(!rigid(c.pose)){failures.stream<<i<<','<<id<<",INVALID_NATIVE_SE3_STOP";failures.end();throw std::runtime_error("invalid native SE3 STOP");}
      if(!c.finite){failures.stream<<i<<','<<id<<",NONFINITE_NATIVE_METRICS_REJECTED";failures.end();}
      coverage::validate(c,e.split,e.mask,tree,e.prediction_pose);validation_ms+=c.validation_ms;
      require(c.eligible==admissible(c,e.mask,e.prediction_pose),"new validation/admission mismatch");raw_candidates.push_back(c);origins.push_back(grid_id);
    }
    const auto phase=Clock::now();const auto pool=pool_v3::construct(raw_candidates,e.mask,e.prediction_pose);const auto selection=pool_v3::select(pool,e.mask,e.prediction_pose);const auto decision=pool_v3::decide(pool,e.mask,e.prediction_pose,selection);const double decision_ms=milliseconds(phase);
    int distinct=0,eligible_distinct=0;for(size_t g=0;g<pool.groups.size();++g)if(int(g)!=pool.group_of[0]){++distinct;eligible_distinct+=pool.raw[pool.groups[g].representative].eligible;}
    for(size_t j=0;j<pool.raw.size();++j){const auto& c=pool.raw[j];auto emit=[&](Csv& f){f.stream<<"R13_TRAINING_GRID,"<<i<<','<<e.row.at("raw_index")<<','<<c.id<<','<<(j?"R13_GRID":"R11_NOMINAL")<<','<<origins[j]<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';number(f.stream,c.score);f.stream<<',';number(f.stream,c.fitness);f.stream<<',';number(f.stream,c.overlap);f.stream<<',';number(f.stream,c.loss_valid?coverage::mean(c.loss,e.mask):NAN);f.stream<<','<<admissible(c,e.mask,e.prediction_pose)<<','<<c.duplicate_of<<','<<pool.group_of[j]<<','<<translation(c.pose,e.prediction_pose)<<','<<rotation(c.pose,e.prediction_pose)<<','<<translation(c.pose,e.nominal.pose)<<','<<rotation(c.pose,e.nominal.pose)<<','<<c.align_ms<<','<<c.validation_ms;emitPose(f.stream,c.initial);emitPose(f.stream,c.pose);f.end();};emit(candidates);if(j)emit(native);
      for(int b=0;b<8;++b){blocks.stream<<i<<','<<c.id<<','<<b<<','<<e.split.blocks[b]->size()<<','<<(std::find(e.mask.active.begin(),e.mask.active.end(),b)!=e.mask.active.end())<<',';number(blocks.stream,c.loss[b]);blocks.end();}}
    for(int j:pool.evidence)for(int k:pool.evidence)if(j!=k){pairs.stream<<i<<','<<e.mask.active.size()<<','<<j<<','<<k<<','<<coverage::paired(pool.raw[j].loss,pool.raw[k].loss,e.mask);pairs.end();}
    for(const auto& old:baseline)if(std::stoi(old.at("scan_index"))==i){selectors.stream<<"R11_FROZEN_"<<old.at("arm")<<','<<i<<','<<old.at("new_representative_ids")<<','<<old.at("new_evidence_ids")<<','<<old.at("new_native")<<','<<old.at("new_mean")<<','<<old.at("new_robust")<<','<<old.at("new_selected")<<','<<old.at("new_status")<<','<<old.at("new_fallback")<<",NA,NA,"<<old.at("ROBUST_EVIDENCE_SUPPORTED")<<','<<old.at("ACCEPTED_REPLACEMENT");selectors.end();}
    const bool robust_distinct=selection.robust>=0&&pool.group_of[selection.robust]!=pool.group_of[0];
    selectors.stream<<"R13_TRAINING_GRID,"<<i<<','<<list(pool.representatives)<<','<<list(pool.evidence)<<','<<selection.native<<','<<selection.heldout_mean<<','<<selection.robust<<','<<decision.selected<<','<<decision.status<<','<<decision.prediction_fallback<<','<<distinct<<','<<eligible_distinct<<','<<robust_distinct<<','<<decision.recovery_success;selectors.end();
    final.stream<<i<<','<<decision.observation_valid;
    if(decision.observation_valid){require(admissible(pool.raw[decision.selected],e.mask,e.prediction_pose),"invalid final observation");emitPose(final.stream,pool.raw[decision.selected].pose);}else for(int n=0;n<16;++n)final.stream<<",NA";emitPose(final.stream,e.prediction_pose);final.end();
    struct rusage usage;getrusage(RUSAGE_SELF,&usage);runtime.stream<<i<<','<<e.preprocessing_ms<<','<<e.split.training->size()<<','<<e.subset.size()<<','<<e.coarse_queries<<','<<e.coarse_ms<<','<<e.selection_ms<<','<<e.coarse_phase_ms<<','<<e.chosen.size()<<','<<align_ms<<','<<fitness_ms<<','<<validation_ms<<','<<decision_ms<<','<<e.coarse_phase_ms+milliseconds(event_start)<<','<<usage.ru_maxrss;runtime.end();
    std::cout<<"EVENT "<<i<<" selected="<<list(e.chosen)<<" distinct="<<distinct<<" admissible_distinct="<<eligible_distinct<<" robust="<<selection.robust<<" final="<<decision.status<<std::endl;
  }
  require(count<=10,"final extra cap");std::cout<<"R13_COMPLETE events=5 coarse_grids=225 new_ndt_align="<<count<<" nominal_align=0 feedback=0"<<std::endl;
}catch(const std::exception& e){std::cerr<<"R13_STOP "<<e.what()<<std::endl;return 1;}}
