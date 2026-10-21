#include "../../p62_r9_coverage_aware_recovery_v2/tools/io.hpp"
#include "../../p62_r9_coverage_aware_recovery_v2/tools/direction.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/events.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/native_capture.hpp"
#include "se3.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sys/resource.h>
using namespace feasibility;
using cause::Seeds;

struct Arm {
  std::vector<Candidate> candidates;
  std::vector<int> native_calls;
};
int main(int argc,char** argv){try{
  require(argc==4,"usage: shadow SCANS MAP NEW_DIR");require(::mkdir(argv[3],0775)==0,"refuse existing output");
  const std::string root=argv[3];auto rows=readCsv(argv[1]);require(rows.size()==100,"fixed100 input");
  std::vector<Cloud::Ptr> sources;std::vector<Split> splits;std::vector<Mask> masks;
  std::vector<double> preprocessing;std::vector<Candidate> full,nominal;std::vector<Event> events;
  Csv coverage_file(root+"/COVERAGE.csv",coverageHeader());
  for(int i=0;i<100;++i){const auto start=Clock::now();const auto& r=rows[i];
    require(std::stoi(r.at("raw_index"))==10641+i,"fixed raw order");prediction(r);auto c=loadSource(r);
    require(c->size()==std::stoul(r.at("expected_processed_points"))&&cloudHash(c)==r.at("expected_processed_fnv"),"Source identity");
    auto s=splitSource(c);Mask m(s);require(m.evaluable(),"frozen100 coverage identity");
    coverageRow(coverage_file,r,c,s,m);sources.push_back(c);splits.push_back(s);masks.push_back(m);preprocessing.push_back(milliseconds(start));}
  std::cout<<"INPUT100_PASS align=0\n";
  const auto map_start=Clock::now();Cloud::Ptr raw(new Cloud),finite(new Cloud);
  require(pcl::io::loadPCDFile(argv[2],*raw)==0,"map load");
  for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);finish(finite);
  auto target=voxel(voxel(finite,.15f),.15f);NativeNdt ndt;ndt.configure(target);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  Csv map(root+"/MAP_RUNTIME.csv","map_points,target_points,target_fnv,actual_grid_m,build_ms");
  map.stream<<raw->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(map_start);map.end();
  std::string candidate_header="arm,scan_index,raw_index,candidate_id,native_call_id,duplicate_of,finite,converged,iterations,native_score,fitness,overlap,mean_loss,eligible,translation_from_prediction_m,rotation_from_prediction_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms";
  poseHeader(candidate_header,"seed_");poseHeader(candidate_header,"final_");
  Csv candidates(root+"/CANDIDATE_POOL.csv",candidate_header),common(root+"/COMMON_RESULTS.csv",candidate_header);
  Csv blocks(root+"/BLOCK_LOSSES.csv","arm,scan_index,candidate_id,block_id,points,active,loss");
  Csv pairs(root+"/PAIRWISE_EVIDENCE.csv","arm,scan_index,B,j,k,D_j_k");
  Csv selection(root+"/SELECTORS.csv","arm,scan_index,selector,pool_ids,selected,min_pairwise_advantage");
  Csv admission(root+"/ADMISSION_AND_SELECTION.csv","arm,scan_index,raw_index,permission,generator_status,seeds_generated,extra_calls,pool_size,distinct_alternatives,eligible_alternatives,robust_selected,final_selected,final_status,observation_valid,recovery_success,prediction_fallback,budget_used");
  Csv event_file(root+"/EVENT_COMPARISON.csv","scan_index,raw_index,header_ns,reference_ns,old_trigger,new_event,reasons,nominal_eligible,prediction_disagreement,G_status,weak_count");
  Csv spectra(root+"/G_SPECTRA.csv","scan_index,status,weak_count,support,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5");
  std::string seed_header="arm,scan_index,seed_id,sign,status,scale,rho_x,rho_y,rho_z,phi_x,phi_y,phi_z,reconstruction_error,translation_from_center_m,rotation_from_center_deg";
  poseHeader(seed_header,"seed_");Csv seeds_file(root+"/SEED_COMPARISON.csv",seed_header);
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","arm,scan_index,preprocess_ms,native_align_ms,G_or_seed_ms,validation_ms,phase_wall_ms,process_high_water_rss_KiB");
  Csv calls(root+"/ALIGN_CALLS.csv","arm,scan_index,candidate_id,native_call_id,status,budget_used");
  std::string raw_header="arm,scan_index,candidate_id,native_call_id,pose_safe,score_finite,converged,iterations,native_score,align_ms";
  poseHeader(raw_header,"seed_");poseHeader(raw_header,"final_");Csv raw_outputs(root+"/RAW_OPTIMIZER_OUTPUTS.csv",raw_header);
  Csv raw_metrics(root+"/RAW_NATIVE_METRICS.csv","arm,scan_index,candidate_id,native_call_id,finite,converged,iterations,native_score,fitness,align_ms");
  Csv failures(root+"/FAILURES.csv","arm,scan_index,candidate_id,reason");
  std::string obs_header="arm,scan_index,observation_valid";poseHeader(obs_header,"accepted_");poseHeader(obs_header,"prediction_");Csv observations(root+"/FINAL_OBSERVATIONS.csv",obs_header);
  // Make raw-record-before-fitness a real I/O gate, not just an attempted flush.
  for(Csv* file:{&coverage_file,&map,&candidates,&common,&blocks,&pairs,&selection,&admission,&event_file,&spectra,&seeds_file,&runtime,&calls,&raw_outputs,&raw_metrics,&failures,&observations})
    file->stream.exceptions(std::ios::failbit|std::ios::badbit);
  Budget budget;int total=0;
  auto run=[&](const std::string& arm,int i,const Pose& initial,int id,bool use_full){
    const int call=++total;calls.stream<<arm<<','<<i+1<<','<<id<<','<<call<<",START,"<<budget.used;calls.end();
    auto capture=[&](const Candidate& c){raw_outputs.stream<<arm<<','<<i+1<<','<<id<<','<<call<<','<<rigid(c.pose)<<','<<std::isfinite(c.score)<<','<<c.converged<<','<<c.iterations<<',';number(raw_outputs.stream,c.score);raw_outputs.stream<<','<<c.align_ms;
      for(const Pose* p:{&c.initial,&c.pose})for(int a=0;a<4;++a)for(int b=0;b<4;++b){raw_outputs.stream<<',';number(raw_outputs.stream,(*p)(a,b));}raw_outputs.end();};
    Candidate c;try{c=capturedAlign(ndt,use_full?sources[i]:splits[i].training,initial,id,capture);}
    catch(...){calls.stream<<arm<<','<<i+1<<','<<id<<','<<call<<",THREW,"<<budget.used;calls.end();throw;}
    calls.stream<<arm<<','<<i+1<<','<<id<<','<<call<<",RETURNED,"<<budget.used;calls.end();
    raw_metrics.stream<<arm<<','<<i+1<<','<<id<<','<<call<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';number(raw_metrics.stream,c.score);raw_metrics.stream<<',';number(raw_metrics.stream,c.fitness);raw_metrics.stream<<','<<c.align_ms;raw_metrics.end();
    if(!rigid(c.pose)){failures.stream<<arm<<','<<i+1<<','<<id<<",INVALID_SE3_STOP";failures.end();throw std::runtime_error("invalid native SE3");}
    if(!c.finite){failures.stream<<arm<<','<<i+1<<','<<id<<",NONFINITE_NATIVE_METRICS_REJECTED";failures.end();}
    try{coverage::validate(c,splits[i],masks[i],tree,prediction(rows[i]));require(c.eligible==admissible(c,masks[i],prediction(rows[i])),"admission metadata");}
    catch(...){failures.stream<<arm<<','<<i+1<<','<<id<<",VALIDATION_EXCEPTION";failures.end();throw;}
    return c;
  };
  auto save=[&](Csv& out,const std::string& arm,int i,const Candidate& c,int call,const Candidate& center){
    const auto p=prediction(rows[i]);const auto& m=masks[i];const bool safe=rigid(c.pose);
    out.stream<<arm<<','<<i+1<<','<<rows[i].at("raw_index")<<','<<c.id<<','<<call<<','<<c.duplicate_of<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';
    for(double v:{c.score,c.fitness,c.overlap,c.loss_valid?coverage::mean(c.loss,m):NAN}){number(out.stream,v);out.stream<<',';}
    out.stream<<admissible(c,m,p);
    for(double v:{safe?translation(c.pose,p):NAN,safe?rotation(c.pose,p):NAN,safe?translation(c.pose,center.pose):NAN,safe?rotation(c.pose,center.pose):NAN,c.align_ms,c.validation_ms}){out.stream<<',';number(out.stream,v);}
    for(const Pose* t:{&c.initial,&c.pose})for(int a=0;a<4;++a)for(int b=0;b<4;++b){out.stream<<',';number(out.stream,(*t)(a,b));}out.end();
    for(int b=0;b<8;++b){const bool active=std::find(m.active.begin(),m.active.end(),b)!=m.active.end();blocks.stream<<arm<<','<<i+1<<','<<c.id<<','<<b<<','<<splits[i].blocks[b]->size()<<','<<active<<',';number(blocks.stream,c.loss[b]);blocks.end();}
  };
  auto timing=[&](const std::string& arm,int i,double native,double direction,double validation,Clock::time_point start){
    struct rusage usage;getrusage(RUSAGE_SELF,&usage);runtime.stream<<arm<<','<<i+1<<','<<(arm=="COMMON"?preprocessing[i]:0)<<','<<native<<','<<direction<<','<<validation<<','<<milliseconds(start)<<','<<usage.ru_maxrss;runtime.end();};
  std::vector<int> actual_events;
  for(int i=0;i<100;++i){const auto start=Clock::now();const Pose p=prediction(rows[i]);
    auto f=run("CONTROL_FULL",i,p,-1,true);auto n=run("CONTROL_SPLIT",i,p,0,false);full.push_back(f);nominal.push_back(n);
    auto e=detect(n,masks[i],p,0);events.push_back(e);if(e.detected)actual_events.push_back(i+1);
    save(common,"CONTROL_FULL",i,f,2*i+1,n);save(common,"CONTROL_SPLIT",i,n,2*i+2,n);
    timing("COMMON",i,f.align_ms+n.align_ms,0,f.validation_ms+n.validation_ms,start);
    if(i==19)std::cout<<"SMOKE20_PASS full=20 split=20 extra=0"<<std::endl;
  }
  require(actual_events==std::vector<int>({60,61,62,63,64}),"known natural events did not reproduce STOP");
  for(const auto& e:events)require(e.old_available&&!e.old_trigger&&(!e.detected||e.reasonText()=="ROTATION_LIMIT|NOMINAL_ADMISSION_REJECTED"),"event predicate identity STOP");
  require(total==200&&budget.used==0,"common nominal must run exactly once before arms");
  std::cout<<"EVENT_IDENTITY_PASS five_events=60|61|62|63|64 nominal_aligns=200"<<std::endl;
  for(int i=0;i<100;++i){
    const Pose p=prediction(rows[i]);Recovery g;const bool disagreement_event=std::find(events[i].reasons.begin(),events[i].reasons.end(),"ROTATION_LIMIT")!=events[i].reasons.end()||std::find(events[i].reasons.begin(),events[i].reasons.end(),"TRANSLATION_LIMIT")!=events[i].reasons.end();
    for(const std::string arm:{"BASELINE_WEAK","PROPOSED_DISAGREEMENT"}){
      const auto start=Clock::now();Arm local{{nominal[i]},{2*i+2}};const Event e=detect(nominal[i],masks[i],p,budget.used);
      std::string status=e.detected?e.permission:"NOT_REQUESTED";double direction_ms=0;int generated=0,extra=0;
      std::vector<Pose> guesses;Seeds residual;
      if(e.permitted){require(safeNative(nominal[i]),"unsafe nominal before generator");
        if(arm=="BASELINE_WEAK"){
          g=direction(ndt,splits[i],nominal[i].pose);direction_ms=g.g_ms;status=g.direction;
          require(!g.invalid,"invalid G STOP");
          if(g.direction=="WEAK_DIRECTION")for(double sign:{1.,-1.})guesses.push_back(shadow::seed(nominal[i].pose,g.weak_vector,sign));
        }else if(disagreement_event){const auto seed_start=Clock::now();residual=cause::disagreement(p,nominal[i].pose);direction_ms=milliseconds(seed_start);status=residual.status;
          if(residual.valid)guesses.assign(residual.poses.begin(),residual.poses.end());
        }else status="NOT_DISAGREEMENT_EVENT";
      }
      if(guesses.empty()){seeds_file.stream<<arm<<','<<i+1<<",NA,NA,"<<status;for(int z=0;z<26;++z)seeds_file.stream<<",NA";seeds_file.end();}
      for(size_t k=0;k<guesses.size();++k){
        const Pose center=arm=="BASELINE_WEAK"?nominal[i].pose:p;
        require(rigid(guesses[k])&&translation(guesses[k],center)<=.30+1e-12&&rotation(guesses[k],center)<=3.+1e-10,"seed numerical failure STOP");
        seeds_file.stream<<arm<<','<<i+1<<','<<k+1<<','<<(k==0?1:-1)<<','<<status<<',';number(seeds_file.stream,residual.scale);
        for(int z=0;z<6;++z){seeds_file.stream<<',';number(seeds_file.stream,residual.xi[z]);}seeds_file.stream<<',';number(seeds_file.stream,residual.reconstruction_error);
        seeds_file.stream<<','<<translation(guesses[k],center)<<','<<rotation(guesses[k],center);emitPose(seeds_file.stream,guesses[k]);seeds_file.end();++generated;
        if(!budget.consume()){status="BUDGET_EXHAUSTED";break;}
        local.candidates.push_back(run(arm,i,guesses[k],int(local.candidates.size()),false));local.native_calls.push_back(total);++extra;
        if(!local.candidates.back().finite)break;
      }
      deduplicate(local.candidates);const auto ids=pool(local.candidates);const auto selected=coverage::select(local.candidates,masks[i]);
      const auto decision=decide(local.candidates,masks[i],p,selected);std::string idlist;for(int j:ids){if(!idlist.empty())idlist+='|';idlist+=std::to_string(j);}
      for(auto method:std::vector<std::pair<std::string,int>>{{"NATIVE_SCORE",selected.native},{"HELDOUT_MEAN",selected.heldout_mean},{"COVERAGE_ROBUST",selected.robust}}){
        selection.stream<<arm<<','<<i+1<<','<<method.first<<','<<idlist<<','<<method.second<<',';double margin=NAN;
        if(method.second>=0&&ids.size()>1){margin=INFINITY;for(int k:ids)if(k!=method.second)margin=std::min(margin,coverage::paired(local.candidates[method.second].loss,local.candidates[k].loss,masks[i]));}
        number(selection.stream,margin);selection.end();}
      for(int j:ids)for(int k:ids)if(j!=k){pairs.stream<<arm<<','<<i+1<<','<<masks[i].active.size()<<','<<j<<','<<k<<','<<coverage::paired(local.candidates[j].loss,local.candidates[k].loss,masks[i]);pairs.end();}
      int distinct=0,eligible=0;double native_ms=0,validation_ms=0;
      for(size_t j=0;j<local.candidates.size();++j){save(candidates,arm,i,local.candidates[j],local.native_calls[j],nominal[i]);if(j){native_ms+=local.candidates[j].align_ms;validation_ms+=local.candidates[j].validation_ms;}}
      for(int j:ids)if(j){++distinct;eligible+=admissible(local.candidates[j],masks[i],p);}
      admission.stream<<arm<<','<<i+1<<','<<rows[i].at("raw_index")<<','<<e.permission<<','<<status<<','<<generated<<','<<extra<<','<<ids.size()<<','<<distinct<<','<<eligible<<','<<selected.robust<<','<<decision.selected<<','<<decision.status<<','<<decision.observation_valid<<','<<decision.recovery_success<<','<<decision.prediction_fallback<<','<<budget.used;admission.end();
      observations.stream<<arm<<','<<i+1<<','<<decision.observation_valid;
      if(decision.observation_valid){require(admissible(local.candidates[decision.selected],masks[i],p),"invalid final observation");emitPose(observations.stream,local.candidates[decision.selected].pose);}
      else for(int z=0;z<16;++z)observations.stream<<",NA";emitPose(observations.stream,p);observations.end();
      timing(arm,i,native_ms,direction_ms,validation_ms,start);
      require(extra<=2&&budget.used<=20&&(!e.detected?extra==0:true),"arm budget violation");
      if(e.detected)std::cout<<"EVENT "<<i+1<<" arm="<<arm<<" G="<<status<<" extra="<<extra<<" distinct="<<distinct<<" eligible="<<eligible<<" final="<<decision.status<<std::endl;
    }
    event_file.stream<<i+1<<','<<rows[i].at("raw_index")<<','<<rows[i].at("header_ns")<<','<<rows[i].at("reference_ns")<<','<<events[i].old_trigger<<','<<events[i].detected<<','<<events[i].reasonText()<<','<<admissible(nominal[i],masks[i],p)<<','<<disagreement_event<<','<<g.direction<<',';
    if(g.weak_count<0)event_file.stream<<"NA";else event_file.stream<<g.weak_count;event_file.end();
    spectra.stream<<i+1<<','<<g.direction<<','<<g.weak_count<<','<<g.supported;for(int k=0;k<6;++k){spectra.stream<<',';number(spectra.stream,g.eigenvalues[k]);}spectra.end();
  }
  require(total==200+budget.used&&budget.used<=20,"final native accounting");
  std::cout<<"COMPLETE frames=100 full=100 split=100 events=5 extra="<<budget.used<<" total="<<total<<" feedback=0"<<std::endl;
}catch(const std::exception& e){std::cerr<<"STOP "<<e.what()<<std::endl;return 1;}}
