#include "../../p62_r9_coverage_aware_recovery_v2/tools/io.hpp"
#include "../../p62_r9_coverage_aware_recovery_v2/tools/direction.hpp"
#include "events.hpp"
#include "native_capture.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sys/resource.h>
using namespace feasibility;
int main(int argc,char** argv){try{
  require(argc==4,"usage: shadow SCANS_CSV MAP NEW_OUTPUT_DIR");require(::mkdir(argv[3],0775)==0,"refuse existing formal output");
  const std::string root=argv[3];auto rows=readCsv(argv[1]);require(rows.size()==100,"fixed100 only");
  std::vector<Cloud::Ptr> sources;std::vector<Split> splits;std::vector<Mask> masks;std::vector<double> preprocessing;
  Csv cov(root+"/COVERAGE_COMPARISON.csv",coverageHeader());
  for(int i=0;i<100;++i){auto start=Clock::now();const auto& r=rows[i];require(std::stoi(r.at("raw_index"))==10641+i,"frozen window order");prediction(r);auto c=loadSource(r);
    require(c->size()==std::stoul(r.at("expected_processed_points"))&&cloudHash(c)==r.at("expected_processed_fnv"),"Source parity before align");
    auto s=splitSource(c);Mask m(s);coverageRow(cov,r,c,s,m);sources.push_back(c);splits.push_back(s);masks.push_back(m);preprocessing.push_back(milliseconds(start));}
  std::cout<<"INPUT100_PASS native_aligns=0\n";
  auto map_start=Clock::now();Cloud::Ptr raw(new Cloud),finite(new Cloud);require(pcl::io::loadPCDFile(argv[2],*raw)==0,"map load");
  for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);finish(finite);
  auto target=voxel(voxel(finite,.15f),.15f);NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  Csv map(root+"/MAP_RUNTIME.csv","raw_points,target_points,target_fnv,actual_grid_m,build_ms");map.stream<<raw->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(map_start);map.end();
  std::string header="phase,scan_index,raw_index,candidate_id,scale,duplicate_of,finite,converged,iterations,native_mean_score,fitness,active_overlap_0p5m,active_mean_loss,formal_eligible,final_within_formal_bounds,translation_from_prediction_m,rotation_from_prediction_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms";
  poseHeader(header,"seed_");poseHeader(header,"final_");
  Csv candidates(root+"/CANDIDATE_POOL.csv",header);
  Csv blocks(root+"/HELDOUT_BLOCK_LOSSES.csv","phase,scan_index,candidate_id,block_id,points,active,loss");
  Csv selectors(root+"/SELECTOR_COMPARISON.csv","phase,scan_index,selector,pool_ids,selected_id,outcome,min_pairwise_advantage");
  Csv pairs(root+"/PAIRWISE_EVIDENCE.csv","phase,scan_index,B,candidate_j,candidate_k,D_j_k");
  Csv frames(root+"/FRAME_RESULTS.csv","scan_index,raw_index,header_ns,reference_ns,B,old_trigger_defined,old_trigger,new_trigger,new_only_trigger,event_reasons,recovery_permitted,permission_status,extra_calls,direction_status,weak_count,seeds_generated,budget_used,pool_size,distinct_alternatives,eligible_alternatives,robust_selected,final_status,observation_valid,final_selected,recovery_success,prediction_fallback,fallback_status");
  std::string final_header="scan_index,raw_index,observation_valid";poseHeader(final_header,"accepted_");poseHeader(final_header,"common_prediction_");Csv finalposes(root+"/FINAL_OBSERVATIONS.csv",final_header);
  Csv gs(root+"/G_STATISTICS.csv","phase,scan_index,status,weak_count,support_points,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,v0,v1,v2,v3,v4,v5");
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","phase,scan_index,preprocess_ms,full_align_ms,nominal_align_ms,extra_align_ms,G_ms,validation_ms,selectors_and_record_ms,frame_compute_ms,peak_ru_maxrss_KiB");
  Csv calls(root+"/NATIVE_ALIGN_CALLS.csv","phase,scan_index,candidate_id,scale,status,cumulative_attempts,budget_used");
  std::string raw_header="scan_index,raw_index,candidate_id,scale,finite,converged,iterations,native_mean_score,fitness,align_ms";poseHeader(raw_header,"seed_");poseHeader(raw_header,"final_");Csv raw_outputs(root+"/RAW_NATIVE_OUTPUTS.csv",raw_header);
  std::string optimizer_header="scan_index,raw_index,candidate_id,scale,pose_safe,score_finite,converged,iterations,native_mean_score,align_ms";poseHeader(optimizer_header,"seed_");poseHeader(optimizer_header,"final_");Csv optimizer_outputs(root+"/RAW_OPTIMIZER_OUTPUTS.csv",optimizer_header);
  Csv validation_failures(root+"/VALIDATION_FAILURES.csv","scan_index,candidate_id,scale,reason");
  Budget budget;int total=0,scan=0,old_events=0,new_events=0,new_only_events=0;std::string phase="NATURAL";
  auto run=[&](int i,const Pose& initial,int id,const std::string& scale,bool full){
    ++total;calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",START,"<<total<<','<<budget.used;calls.end();
    auto capture=[&](const Candidate& value){optimizer_outputs.stream<<scan<<','<<rows[i].at("raw_index")<<','<<id<<','<<scale<<','<<rigid(value.pose)<<','<<std::isfinite(value.score)<<','<<value.converged<<','<<value.iterations<<',';number(optimizer_outputs.stream,value.score);optimizer_outputs.stream<<','<<value.align_ms;
      for(const Pose* p:{&value.initial,&value.pose})for(int a=0;a<4;++a)for(int b=0;b<4;++b){optimizer_outputs.stream<<',';number(optimizer_outputs.stream,(*p)(a,b));}optimizer_outputs.end();};
    Candidate c;try{c=capturedAlign(ndt,full?sources[i]:splits[i].training,initial,id,capture);}
    catch(...){calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",THREW,"<<total<<','<<budget.used;calls.end();throw;}
    calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",RETURNED,"<<total<<','<<budget.used;calls.end();
    raw_outputs.stream<<scan<<','<<rows[i].at("raw_index")<<','<<id<<','<<scale<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';number(raw_outputs.stream,c.score);raw_outputs.stream<<',';number(raw_outputs.stream,c.fitness);raw_outputs.stream<<','<<c.align_ms;
    for(const Pose* p:{&c.initial,&c.pose})for(int a=0;a<4;++a)for(int b=0;b<4;++b){raw_outputs.stream<<',';number(raw_outputs.stream,(*p)(a,b));}raw_outputs.end();
    try{coverage::validate(c,splits[i],masks[i],tree,prediction(rows[i]));require(c.eligible==admissible(c,masks[i],prediction(rows[i])),"admission metadata mismatch");}
    catch(...){validation_failures.stream<<scan<<','<<id<<','<<scale<<",VALIDATION_OR_METADATA_EXCEPTION";validation_failures.end();throw;}return c;
  };
  auto record=[&](int i,const std::vector<Candidate>& cs,const std::vector<std::string>& scales,const Candidate* full){
    const auto start=Clock::now();const auto& m=masks[i];auto ids=pool(cs);auto selection=coverage::select(cs,m);std::string idlist;
    for(int j:ids){if(!idlist.empty())idlist+='|';idlist+=std::to_string(j);}
    for(auto s:std::vector<std::pair<std::string,int>>{{"NATIVE_SCORE",selection.native},{"HELDOUT_MEAN",selection.heldout_mean},{"COVERAGE_ROBUST",selection.robust}}){
      selectors.stream<<phase<<','<<scan<<','<<s.first<<','<<idlist<<','<<s.second<<','<<(!m.evaluable()?"COVERAGE_INSUFFICIENT":s.second<0?"ABSTAIN":s.second==0?"KEEP_NOMINAL":"SELECT_ALTERNATIVE")<<',';
      double margin=NAN;if(s.second>=0&&ids.size()>1){margin=INFINITY;for(int k:ids)if(k!=s.second)margin=std::min(margin,coverage::paired(cs[s.second].loss,cs[k].loss,m));}number(selectors.stream,margin);selectors.end();}
    for(int j:ids)for(int k:ids)if(j!=k){pairs.stream<<phase<<','<<scan<<','<<m.active.size()<<','<<j<<','<<k<<','<<coverage::paired(cs[j].loss,cs[k].loss,m);pairs.end();}
    auto save=[&](const Candidate& c,const std::string& scale){const Pose initial=prediction(rows[i]);const bool valid=rigid(c.pose),center=rigid(cs[0].pose);
      candidates.stream<<phase<<','<<scan<<','<<rows[i].at("raw_index")<<','<<c.id<<','<<scale<<','<<c.duplicate_of<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';number(candidates.stream,c.score);candidates.stream<<',';number(candidates.stream,c.fitness);candidates.stream<<',';number(candidates.stream,c.overlap);candidates.stream<<',';number(candidates.stream,c.loss_valid?coverage::mean(c.loss,m):NAN);
      candidates.stream<<','<<admissible(c,m,initial)<<','<<bounded(c.pose,initial);
      for(double v:{valid?translation(c.pose,initial):NAN,valid?rotation(c.pose,initial):NAN,valid&&center?translation(c.pose,cs[0].pose):NAN,valid&&center?rotation(c.pose,cs[0].pose):NAN}){candidates.stream<<',';number(candidates.stream,v);}
      candidates.stream<<','<<c.align_ms<<','<<c.validation_ms;
      for(const Pose* p:{&c.initial,&c.pose})for(int a=0;a<4;++a)for(int b=0;b<4;++b){candidates.stream<<',';number(candidates.stream,(*p)(a,b));}candidates.end();
      for(int b=0;b<8;++b){const bool active=std::find(m.active.begin(),m.active.end(),b)!=m.active.end();blocks.stream<<phase<<','<<scan<<','<<c.id<<','<<b<<','<<splits[i].blocks[b]->size()<<','<<active<<',';number(blocks.stream,c.loss[b]);blocks.end();}
    };
    if(full)save(*full,"CONTROL_FULL");for(size_t j=0;j<cs.size();++j)save(cs[j],scales[j]);
    return std::make_pair(selection,milliseconds(start));
  };
  auto recordG=[&](const Recovery& g){gs.stream<<phase<<','<<scan<<','<<g.direction<<',';if(g.weak_count<0)gs.stream<<"NA";else gs.stream<<g.weak_count;gs.stream<<','<<g.supported;
    for(int k=0;k<6;++k){gs.stream<<',';number(gs.stream,g.eigenvalues[k]);}for(int k=0;k<6;++k){gs.stream<<',';number(gs.stream,g.weak_vector[k]);}gs.end();};
  auto recordRuntime=[&](int i,const Candidate* full,const std::vector<Candidate>& cs,const Recovery& g,double select_ms,Clock::time_point start){
    double validation=full?full->validation_ms:0,extra=0;for(size_t j=0;j<cs.size();++j){validation+=cs[j].validation_ms;if(j>0)extra+=cs[j].align_ms;}
    struct rusage usage;getrusage(RUSAGE_SELF,&usage);
    runtime.stream<<phase<<','<<scan<<','<<(phase=="NATURAL"?preprocessing[i]:0)<<','<<(full?full->align_ms:0)<<','<<(phase=="NATURAL"?cs[0].align_ms:0)<<','<<extra<<','<<g.g_ms<<','<<validation<<','<<select_ms<<','<<milliseconds(start)<<','<<usage.ru_maxrss;runtime.end();
  };
  for(int i=0;i<100;++i){
    scan=i+1;const auto start=Clock::now();const Pose initial=prediction(rows[i]);
    auto full=run(i,initial,-1,"CONTROL_FULL",true);
    std::vector<Candidate> cs{run(i,initial,0,"NOMINAL",false)};std::vector<std::string> scales{"NOMINAL"};
    const Event event=detect(cs[0],masks[i],initial,budget.used);
    old_events+=event.old_trigger;new_events+=event.detected;new_only_events+=event.detected&&!event.old_trigger;
    Recovery g;int extra=0,seeds=0;
    if(event.permitted){
      require(safeNative(cs[0]),"unsafe nominal blocked before G");
      g=direction(ndt,splits[i],cs[0].pose);
      if(g.direction=="WEAK_DIRECTION")for(double sign:{1.,-1.}){
        if(!budget.consume())break;
        const Pose guess=shadow::seed(cs[0].pose,g.weak_vector,sign);++seeds;
        auto candidate=run(i,guess,cs.size(),"SMALL",false);++extra;cs.push_back(candidate);scales.push_back("SMALL");
        if(!candidate.finite)break;
      }
    }
    deduplicate(cs);auto comparison=record(i,cs,scales,&full);const Decision decision=decide(cs,masks[i],initial,comparison.first);
    recordG(g);const auto ids=pool(cs);int distinct=0,eligible=0;for(int j:ids)if(j){++distinct;eligible+=admissible(cs[j],masks[i],initial);}
    frames.stream<<scan<<','<<rows[i].at("raw_index")<<','<<rows[i].at("header_ns")<<','<<rows[i].at("reference_ns")<<','<<masks[i].active.size()<<','<<event.old_available<<',';
    if(event.old_available)frames.stream<<event.old_trigger;else frames.stream<<"NA";
    frames.stream<<','<<event.detected<<','<<(event.detected&&!event.old_trigger)<<','<<event.reasonText()<<','<<event.permitted<<','<<event.permission<<','<<extra<<','<<g.direction<<',';
    if(g.weak_count<0)frames.stream<<"NA";else frames.stream<<g.weak_count;
    frames.stream<<','<<seeds<<','<<budget.used<<','<<ids.size()<<','<<distinct<<','<<eligible<<','<<comparison.first.robust<<','<<decision.status<<','<<decision.observation_valid<<','<<decision.selected<<','<<decision.recovery_success<<','<<decision.prediction_fallback<<','<<(decision.prediction_fallback?"SHADOW_PREDICTION_ONLY_NOT_EXECUTED":decision.observation_valid?"NOT_NEEDED":"INVALID_PREDICTION_STOP");frames.end();
    finalposes.stream<<scan<<','<<rows[i].at("raw_index")<<','<<decision.observation_valid;
    if(decision.observation_valid){require(admissible(cs[decision.selected],masks[i],initial),"illegal observation must not escape");emitPose(finalposes.stream,cs[decision.selected].pose);}
    else for(int k=0;k<16;++k)finalposes.stream<<",NA";
    emitPose(finalposes.stream,initial);finalposes.end();
    recordRuntime(i,&full,cs,g,comparison.second,start);
    require(extra<=2&&budget.used<=20,"budget");if(!event.detected)require(extra==0,"ordinary extra align");
    if(scan==20)std::cout<<"SMOKE20_PASS"<<std::endl;
    std::cout<<"FRAME "<<scan<<" old="<<(event.old_available?std::to_string(event.old_trigger):"NA")<<" event="<<event.detected<<" reasons="<<event.reasonText()<<" permission="<<event.permission<<" G="<<g.direction<<" extra="<<extra<<" final="<<decision.status<<" budget="<<budget.used<<std::endl;
  }
  require(total==200+budget.used&&budget.used<=20,"final native accounting");
  std::cout<<"COMPLETE frames=100 native_aligns="<<total<<" old_events="<<old_events<<" detected_events="<<new_events<<" new_only_events="<<new_only_events<<" extra="<<budget.used<<" feedback=0"<<std::endl;
}catch(const std::exception& e){std::cerr<<"STOP "<<e.what()<<'\n';return 1;}}

