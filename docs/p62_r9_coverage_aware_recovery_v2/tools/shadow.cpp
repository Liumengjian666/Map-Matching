#include "io.hpp"
#include "direction.hpp"
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <sys/resource.h>
using namespace coverage;
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
  Csv candidates(root+"/CANDIDATE_SCALE_COMPARISON.csv",header);
  Csv blocks(root+"/HELDOUT_BLOCK_LOSSES.csv","phase,scan_index,candidate_id,block_id,points,active,loss");
  Csv selectors(root+"/SELECTOR_COMPARISON.csv","phase,scan_index,selector,pool_ids,selected_id,outcome,min_pairwise_advantage");
  Csv pairs(root+"/PAIRWISE_EVIDENCE.csv","phase,scan_index,B,candidate_j,candidate_k,D_j_k");
  Csv frames(root+"/NATURAL_SHADOW_RESULTS.csv","scan_index,raw_index,header_ns,reference_ns,B,coverage_status,natural_trigger,optimizer_failure_trigger,extra_calls,direction_status,budget_used,pool_size,distinct_alternatives,eligible_alternatives,robust_selected");
  Csv diagnostic(root+"/DIAGNOSTIC_SCHEDULE.csv","scan_index,scale,sign,status,candidate_id,budget_used");
  Csv gs(root+"/G_STATISTICS.csv","phase,scan_index,status,weak_count,support_points,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,v0,v1,v2,v3,v4,v5");
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","phase,scan_index,preprocess_ms,full_align_ms,nominal_align_ms,extra_align_ms,G_ms,validation_ms,selectors_and_record_ms,frame_compute_ms,peak_ru_maxrss_KiB");
  Csv calls(root+"/NATIVE_ALIGN_CALLS.csv","phase,scan_index,candidate_id,scale,status,cumulative_attempts,budget_used");
  Budget budget;int total=0,scan=0,events=0;std::string phase="NATURAL";
  auto run=[&](int i,const Pose& initial,int id,const std::string& scale,bool full){
    ++total;calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",START,"<<total<<','<<budget.used;calls.end();
    Candidate c;try{c=align(ndt,full?sources[i]:splits[i].training,initial,id);}
    catch(...){calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",THREW,"<<total<<','<<budget.used;calls.end();throw;}
    calls.stream<<phase<<','<<scan<<','<<id<<','<<scale<<",RETURNED,"<<total<<','<<budget.used;calls.end();
    coverage::validate(c,splits[i],masks[i],tree,prediction(rows[i]));return c;
  };
  auto record=[&](int i,const std::vector<Candidate>& cs,const std::vector<std::string>& scales,const Candidate* full){
    const auto start=Clock::now();const auto& m=masks[i];auto ids=pool(cs);auto selection=coverage::select(cs,m);std::string idlist;
    for(int j:ids){if(!idlist.empty())idlist+='|';idlist+=std::to_string(j);}
    for(auto s:std::vector<std::pair<std::string,int>>{{"NATIVE_SCORE",selection.native},{"HELDOUT_MEAN",selection.heldout_mean},{"COVERAGE_ROBUST",selection.robust}}){
      selectors.stream<<phase<<','<<scan<<','<<s.first<<','<<idlist<<','<<s.second<<','<<(!m.evaluable()?"COVERAGE_INSUFFICIENT":s.second<0?"ABSTAIN":s.second==0?"KEEP_NOMINAL":"SELECT_ALTERNATIVE")<<',';
      double margin=NAN;if(s.second>=0&&ids.size()>1){margin=INFINITY;for(int k:ids)if(k!=s.second)margin=std::min(margin,coverage::paired(cs[s.second].loss,cs[k].loss,m));}number(selectors.stream,margin);selectors.end();}
    for(int j:ids)for(int k:ids)if(j!=k){pairs.stream<<phase<<','<<scan<<','<<m.active.size()<<','<<j<<','<<k<<','<<coverage::paired(cs[j].loss,cs[k].loss,m);pairs.end();}
    auto save=[&](const Candidate& c,const std::string& scale){const Pose initial=prediction(rows[i]);
      candidates.stream<<phase<<','<<scan<<','<<rows[i].at("raw_index")<<','<<c.id<<','<<scale<<','<<c.duplicate_of<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<','<<c.score<<','<<c.fitness<<',';number(candidates.stream,c.overlap);candidates.stream<<',';number(candidates.stream,c.loss_valid?coverage::mean(c.loss,m):NAN);
      candidates.stream<<','<<c.eligible<<','<<bounded(c.pose,initial)<<','<<translation(c.pose,initial)<<','<<rotation(c.pose,initial)<<','<<translation(c.pose,cs[0].pose)<<','<<rotation(c.pose,cs[0].pose)<<','<<c.align_ms<<','<<c.validation_ms;emitPose(candidates.stream,c.initial);emitPose(candidates.stream,c.pose);candidates.end();
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
  std::vector<Candidate> cache;std::vector<Recovery> directions;
  for(int i=0;i<100;++i){scan=i+1;auto start=Clock::now();auto full=run(i,prediction(rows[i]),-1,"CONTROL_FULL",true);std::vector<Candidate> cs{run(i,prediction(rows[i]),0,"NOMINAL",false)};std::vector<std::string> scales{"NOMINAL"};
    cache.push_back(cs[0]);Recovery g;const bool available=masks[i].evaluable()&&cs[0].loss_valid;
    const bool optimizer_event=!cs[0].converged||cs[0].iterations>=40;
    const bool event=available&&(optimizer_event||coverage::median(cs[0].loss,masks[i])>.25);events+=event;int extra=0;
    if(event&&cs[0].finite&&full.finite){if(budget.used>=20)g.direction="BUDGET_EXHAUSTED";else{
      g=direction(ndt,splits[i],cs[0].pose);if(g.direction=="WEAK_DIRECTION")for(double sign:{1.,-1.}){
        if(!budget.consume())break;auto c=run(i,scaledSeed(cs[0].pose,g.weak_vector,sign,false),cs.size(),"SMALL",false);++extra;cs.push_back(c);scales.push_back("SMALL");if(!c.finite)break;}}}
    directions.push_back(g);deduplicate(cs);auto choice=record(i,cs,scales,&full);recordG(g);auto ids=pool(cs);int distinct=0,eligible=0;for(int j:ids)if(j){++distinct;eligible+=cs[j].eligible;}
    frames.stream<<scan<<','<<rows[i].at("raw_index")<<','<<rows[i].at("header_ns")<<','<<rows[i].at("reference_ns")<<','<<masks[i].active.size()<<','<<(masks[i].evaluable()?"EVALUABLE":"COVERAGE_INSUFFICIENT")<<',';
    if(available)frames.stream<<event;else frames.stream<<"NA";frames.stream<<','<<optimizer_event<<','<<extra<<','<<g.direction<<','<<budget.used<<','<<ids.size()<<','<<distinct<<','<<eligible<<','<<choice.first.robust;frames.end();
    recordRuntime(i,&full,cs,g,choice.second,start);require(!g.invalid&&full.finite&&std::all_of(cs.begin(),cs.end(),[](const Candidate& c){return c.finite;}),"invalid native/G result saved; stop");
    require(extra<=2&&budget.used<=20,"budget");if(!event)require(extra==0,"ordinary extra");if(scan==20)std::cout<<"SMOKE20_PASS\n";
    std::cout<<"NATURAL "<<scan<<" event="<<(available?std::to_string(event):"NA")<<" extra="<<extra<<" budget="<<budget.used<<std::endl;
  }
  const int natural_extra=budget.used;require(total==200+natural_extra,"natural accounting");phase="DIAGNOSTIC_ONLY";
  for(int local:{20,40,60,80}){scan=local;int i=local-1;auto start=Clock::now();std::vector<Candidate> cs{cache[i]};cs[0].align_ms=cs[0].validation_ms=0;std::vector<std::string> scales{"CACHED_NOMINAL"};
    Recovery g=directions[i];if(budget.used<20&&g.direction=="NOT_REQUESTED")g=direction(ndt,splits[i],cache[i].pose);else g.g_ms=0;
    bool diagnostic_invalid=false;
    for(bool large:{false,true})for(double sign:{1.,-1.}){std::string scale=large?"LARGE":"SMALL";std::string status;int id=-1;
      if(diagnostic_invalid)status="NOT_RUN_INVALID_NATIVE";else if(budget.used>=20)status="NOT_RUN_BUDGET";else if(g.direction!="WEAK_DIRECTION")status=g.direction;else{
        require(budget.consume(),"token");id=cs.size();auto c=run(i,scaledSeed(cache[i].pose,g.weak_vector,sign,large),id,scale,false);cs.push_back(c);scales.push_back(scale);status=c.finite?"ALIGNED_DIAGNOSTIC_ONLY":"INVALID_NATIVE";diagnostic_invalid=!c.finite;}
      diagnostic.stream<<scan<<','<<scale<<','<<sign<<','<<status<<','<<id<<','<<budget.used;diagnostic.end();
    }
    deduplicate(cs);auto choice=record(i,cs,scales,nullptr);recordG(g);recordRuntime(i,nullptr,cs,g,choice.second,start);
    require(!g.invalid&&std::all_of(cs.begin(),cs.end(),[](const Candidate& c){return c.finite;}),"diagnostic invalid result saved");
    std::cout<<"DIAGNOSTIC "<<scan<<" G="<<g.direction<<" pool="<<pool(cs).size()<<" budget="<<budget.used<<std::endl;
  }
  require(budget.used-natural_extra<=16&&budget.used<=20&&total==200+budget.used,"final shared accounting");
  std::cout<<"COMPLETE frames=100 native_aligns="<<total<<" natural_events="<<events<<" natural_extra="<<natural_extra<<" diagnostic_extra="<<budget.used-natural_extra<<" feedback=0\n";
}catch(const std::exception& e){std::cerr<<"STOP "<<e.what()<<'\n';return 1;}}
