#include "recovery.hpp"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sys/stat.h>
#include <sys/resource.h>
using namespace shadow;
using Row=std::map<std::string,std::string>;
struct Csv {
  std::ofstream stream;
  Csv(const std::string& path,const std::string& header):stream(path){require(bool(stream),"CSV creation failed");stream<<std::setprecision(17)<<header<<'\n';stream.flush();}
  void end(){stream<<'\n';stream.flush();}
};
std::vector<Row> readCsv(const std::string& path){
  std::ifstream f(path);std::string line;require(bool(std::getline(f,line)),"missing frozen CSV");auto keys=csvFields(line);std::vector<Row> rows;
  while(std::getline(f,line)){auto vals=csvFields(line);require(vals.size()==keys.size(),"CSV width");Row r;for(size_t i=0;i<keys.size();++i)r[keys[i]]=vals[i];rows.push_back(r);}return rows;
}
Pose prediction(const Row& r){Pose p=Pose::Identity();Eigen::Quaterniond q(std::stod(r.at("qw")),std::stod(r.at("qx")),std::stod(r.at("qy")),std::stod(r.at("qz")));
  p.block<3,3>(0,0)=q.normalized().toRotationMatrix();p.block<3,1>(0,3)=Eigen::Vector3d(std::stod(r.at("tx")),std::stod(r.at("ty")),std::stod(r.at("tz")));require(rigid(p),"invalid common prediction");return p;}
Cloud::Ptr loadSource(const Row& r){
  std::ifstream f(r.at("raw_xyz_offset_file"),std::ios::binary);require(bool(f),"raw source missing");Cloud::Ptr c(new Cloud);std::array<float,3> xyz;uint32_t offset;
  for(int i=0;i<std::stoi(r.at("raw_points"));++i){f.read(reinterpret_cast<char*>(xyz.data()),12);f.read(reinterpret_cast<char*>(&offset),4);require(bool(f),"truncated Source");
    if(!std::isfinite(xyz[0])||!std::isfinite(xyz[1])||!std::isfinite(xyz[2]))continue;
    const double range=std::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);
    if(range>=.5&&range<=80)c->push_back(pcl::PointXYZ(xyz[0],xyz[1],xyz[2]));}
  require(f.peek()==std::char_traits<char>::eof(),"unexpected Source suffix");finish(c);return voxel(c,.25f,1400);
}
void poseHeader(std::string& s,const std::string& prefix){for(int i=0;i<4;++i)for(int j=0;j<4;++j)s+=","+prefix+std::to_string(i)+std::to_string(j);}
void emitPose(std::ostream& o,const Pose& p){for(int i=0;i<4;++i)for(int j=0;j<4;++j)o<<','<<p(i,j);}
void number(std::ostream& o,double v){if(std::isfinite(v))o<<v;else o<<"NA";}
int main(int argc,char** argv){try{
  require(argc==4,"usage: shadow FROZEN_SCANS_CSV FROZEN_MAP NEW_OUTPUT_DIR");require(::mkdir(argv[3],0775)==0,"refuse existing output");const std::string root=argv[3];
  const auto rows=readCsv(argv[1]);require(rows.size()==100,"not frozen100 scans");
  Csv preflight(root+"/input_preflight.csv","scan_index,raw_index,source_points,source_fnv,training_points,val0,val1,val2,val3,val4,val5,val6,val7,evaluable");
  std::vector<Cloud::Ptr> sources;std::vector<Split> splits;std::vector<double> preprocess_ms;
  bool all_evaluable=true;
  for(const auto& r:rows){auto start=Clock::now();auto c=loadSource(r);auto s=splitSource(c);prediction(r);preprocess_ms.push_back(milliseconds(start));
    require(c->size()==std::stoul(r.at("expected_processed_points"))&&cloudHash(c)==r.at("expected_processed_fnv"),"Source parity failure before any align");
    preflight.stream<<r.at("scan_index")<<','<<r.at("raw_index")<<','<<c->size()<<','<<cloudHash(c)<<','<<s.training->size();for(auto& b:s.blocks)preflight.stream<<','<<b->size();preflight.stream<<','<<s.evaluable();preflight.end();
    all_evaluable=all_evaluable&&s.evaluable();sources.push_back(c);splits.push_back(s);
  }
  require(all_evaluable,"Selected window has unevaluable Source; do not select runner-up");
  std::cout<<"INPUT_PREFLIGHT_PASS scans=100 native_aligns=0"<<std::endl;
  auto map_start=Clock::now();Cloud::Ptr raw(new Cloud),finite(new Cloud);
  require(pcl::io::loadPCDFile(argv[2],*raw)==0,"map load failed");for(const auto& p:*raw)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite->push_back(p);finish(finite);
  auto target=voxel(voxel(finite,.15f),.15f);NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  Csv map(root+"/map_runtime.csv","original_points,target_points,target_fnv,actual_grid_m,map_build_ms");map.stream<<raw->size()<<','<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(map_start);map.end();
  std::string header="phase,scan_index,raw_index,candidate_id,kind,duplicate_of,finite,converged,iterations,native_mean_score,fitness,translation_from_prediction_m,rotation_from_prediction_deg,heldout_overlap_0p5m,mean_block_loss,eligible,align_ms,validation_ms";poseHeader(header,"seed_");poseHeader(header,"final_");
  Csv candidates(root+"/CANDIDATE_POOL.csv",header),blocks(root+"/HELDOUT_BLOCK_LOSSES.csv","phase,scan_index,candidate_id,block_id,points,loss,NN_queries");
  Csv selection(root+"/SELECTOR_COMPARISON.csv","phase,scan_index,selector,pool_ids,selected_id,outcome,min_pairwise_advantage");
  Csv pairs(root+"/PAIRWISE_ADVANTAGES.csv","phase,scan_index,candidate_j,candidate_k,D_j_k");
  const std::string frame_header="scan_index,raw_index,header_ns,reference_ns,source_fnv,training_fnv,full_points,training_points,validation_evaluable,full_calls,split_calls,extra_calls,natural_trigger,direction_status,weak_count,seeds_generated,budget_used,unique_pool_size,distinct_alternatives,eligible_distinct_alternatives,proposed_selected,proposed_status,lambda_min,lambda_max,G_support_points";
  Csv frames(root+"/NATURAL_SHADOW_RESULTS.csv",frame_header),diagnostics(root+"/DIAGNOSTIC_PROBE_RESULTS.csv",frame_header);
  Csv runtime(root+"/RUNTIME_AND_MEMORY.csv","phase,scan_index,preprocess_ms,full_align_ms,nominal_align_ms,extra_align_ms,G_ms,validation_ms,selectors_and_pair_record_ms,frame_compute_ms,process_ru_maxrss_KiB");
  Csv calls(root+"/NATIVE_ALIGN_CALLS.csv","phase,scan_index,candidate_id,kind,status,cumulative_attempts,budget_used");
  Csv spectrum(root+"/G_STATISTICS.csv","phase,scan_index,status,weak_count,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,weak_v0,weak_v1,weak_v2,weak_v3,weak_v4,weak_v5");
  std::string failure_header="phase,scan_index,raw_index,candidate_id,kind,finite,converged,iterations,native_mean_score,fitness";poseHeader(failure_header,"seed_");poseHeader(failure_header,"final_");
  Csv failures(root+"/FAILED_NATIVE_OUTPUT.csv",failure_header);
  Budget budget;int total=0,natural_events=0;std::vector<Candidate> cache;
  std::string phase="NATURAL";int scan=0;
  const Runner run=[&](const Cloud::Ptr& source,const Pose& guess,int id,const char* kind){
    ++total;calls.stream<<phase<<','<<scan<<','<<id<<','<<kind<<",START,"<<total<<','<<budget.used;calls.end();
    try{auto c=align(ndt,source,guess,id);calls.stream<<phase<<','<<scan<<','<<id<<','<<kind<<",RETURNED,"<<total<<','<<budget.used;calls.end();return c;}
    catch(...){calls.stream<<phase<<','<<scan<<','<<id<<','<<kind<<",THREW,"<<total<<','<<budget.used;calls.end();throw;}
  };
  auto preserve_invalid=[&](const Candidate& c,const Row& row,const char* kind){
    if(c.finite)return;
    failures.stream<<phase<<','<<scan<<','<<row.at("raw_index")<<','<<c.id<<','<<kind<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<','<<c.score<<','<<c.fitness;emitPose(failures.stream,c.initial);emitPose(failures.stream,c.pose);failures.end();
    std::cerr<<"INVALID_NATIVE_RECORDED phase="<<phase<<" scan="<<scan<<" kind="<<kind<<std::endl;
    require(false,"Invalid native candidate preserved in FAILED_NATIVE_OUTPUT.csv; stop without selection");
  };
  auto record_pool=[&](const Row& row,const Split& split,const Pose& initial,const std::vector<Candidate>& cs,const Candidate* full){
    const auto ids=pool(cs);std::string id_list;for(int j:ids){if(!id_list.empty())id_list+='|';id_list+=std::to_string(j);}
    const auto selected=select(cs);
    for(const auto& choice:std::vector<std::pair<std::string,int>>{{"NATIVE_SCORE",selected.native},{"HELDOUT_MEAN",selected.heldout_mean},{"ROBUST_PAIRED",selected.robust}}){
      selection.stream<<phase<<','<<scan<<','<<choice.first<<','<<id_list<<','<<choice.second<<','<<(choice.second<0?"ABSTAIN":(choice.second==0?"KEEP_NOMINAL":"SELECT_ALTERNATIVE"))<<',';
      number(selection.stream,selectedMargin(cs,choice.second));selection.end();
    }
    for(int j:ids)for(int k:ids)if(j!=k){pairs.stream<<phase<<','<<scan<<','<<j<<','<<k<<','<<paired(cs[j].loss,cs[k].loss);pairs.end();}
    auto save=[&](const Candidate& c,const std::string& kind){
      candidates.stream<<phase<<','<<scan<<','<<row.at("raw_index")<<','<<c.id<<','<<kind<<','<<c.duplicate_of<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<','<<c.score<<','<<c.fitness<<','<<translation(c.pose,initial)<<','<<rotation(c.pose,initial)<<',';
      number(candidates.stream,c.overlap);candidates.stream<<',';number(candidates.stream,c.loss_valid?mean(c.loss):NAN);candidates.stream<<','<<c.eligible<<','<<c.align_ms<<','<<c.validation_ms;emitPose(candidates.stream,c.initial);emitPose(candidates.stream,c.pose);candidates.end();
      for(int b=0;b<8;++b){blocks.stream<<phase<<','<<scan<<','<<c.id<<','<<b<<','<<split.blocks[b]->size()<<',';number(blocks.stream,c.loss[b]);blocks.stream<<','<<(c.loss_valid?split.blocks[b]->size():0);blocks.end();}
    };
    if(full)save(*full,"CONTROL_FULL");for(const auto& c:cs)save(c,c.id==0?(phase=="NATURAL"?"CONTROL_SPLIT_NOMINAL":"CACHED_NOMINAL"):"RECOVERY");
    return selected;
  };
  auto record_frame=[&](const Row& row,int index,const std::vector<Candidate>& cs,const Recovery& rec,bool event,const Candidate* full,Clock::time_point start){
    const auto select_start=Clock::now();const auto choice=record_pool(row,splits[index],prediction(row),cs,full);const double select_ms=milliseconds(select_start);
    const auto ids=pool(cs);int distinct=0,eligible=0;for(int j:ids)if(j!=0){++distinct;if(cs[j].eligible)++eligible;}
    std::string status=phase=="DIAGNOSTIC_FORCED"?"DIAGNOSTIC_ONLY":(event?"NO_SUPPORTED_RECOVERY":"ORDINARY_NO_RECOVERY");
    if(event&&rec.direction=="BUDGET_EXHAUSTED")status="BUDGET_EXHAUSTED";
    if(choice.robust>=0)status=phase=="NATURAL"?"SUPPORTED_RECOVERY":"DIAGNOSTIC_SUPPORTED_CHOICE";
    if(rec.extra>0&&distinct==0)status="CANDIDATE_COLLAPSE_TO_NOMINAL";
    if(rec.invalid)status="NUMERICAL_FAILURE_STOP";
    auto& out=phase=="NATURAL"?frames:diagnostics;
    out.stream<<scan<<','<<row.at("raw_index")<<','<<row.at("header_ns")<<','<<row.at("reference_ns")<<','<<cloudHash(sources[index])<<','<<cloudHash(splits[index].training)<<','<<sources[index]->size()<<','<<splits[index].training->size()<<','<<splits[index].evaluable()<<','<<(full?1:0)<<','<<(phase=="NATURAL"?1:0)<<','<<rec.extra<<','<<event<<','<<rec.direction<<',';
    if(rec.weak_count<0)out.stream<<"NA";else out.stream<<rec.weak_count;
    out.stream<<','<<rec.seeds<<','<<budget.used<<','<<ids.size()<<','<<distinct<<','<<eligible<<','<<choice.robust<<','<<status<<',';number(out.stream,rec.eigenvalues[0]);out.stream<<',';number(out.stream,rec.eigenvalues[5]);out.stream<<','<<rec.supported;out.end();
    spectrum.stream<<phase<<','<<scan<<','<<rec.direction<<',';if(rec.weak_count<0)spectrum.stream<<"NA";else spectrum.stream<<rec.weak_count;
    for(int k=0;k<6;++k){spectrum.stream<<',';number(spectrum.stream,rec.eigenvalues[k]);}for(int k=0;k<6;++k){spectrum.stream<<',';number(spectrum.stream,rec.weak_vector[k]);}spectrum.end();
    double validation=full?full->validation_ms:0;for(const auto& c:cs)validation+=c.validation_ms;
    struct rusage usage;getrusage(RUSAGE_SELF,&usage);
    runtime.stream<<phase<<','<<scan<<','<<(phase=="NATURAL"?preprocess_ms[index]:0)<<','<<(full?full->align_ms:0)<<','<<(phase=="NATURAL"?cs[0].align_ms:0)<<','<<rec.extra_ms<<','<<rec.g_ms<<','<<validation<<','<<select_ms<<','<<milliseconds(start)<<','<<usage.ru_maxrss;runtime.end();
    require(!rec.invalid,"Invalid G/recovery candidate; records preserved");require(rec.extra<=2&&budget.used<=20,"combined budget violation");
  };
  for(int index=0;index<100;++index){scan=index+1;const auto start=Clock::now();const auto& row=rows[index];require(std::stoi(row.at("scan_index"))==scan,"Frozen Source order changed");const Pose initial=prediction(row);
    auto full=run(sources[index],initial,-1,"CONTROL_FULL");validate(full,splits[index],tree,initial);preserve_invalid(full,row,"CONTROL_FULL");
    std::vector<Candidate> cs{run(splits[index].training,initial,0,"CONTROL_SPLIT")};validate(cs[0],splits[index],tree,initial);preserve_invalid(cs[0],row,"CONTROL_SPLIT");cache.push_back(cs[0]);
    const bool event=trigger(cs[0].converged,cs[0].iterations,cs[0].loss);natural_events+=event;
    Recovery rec;if(event)rec=recover(ndt,splits[index],tree,initial,cs,budget,run);deduplicate(cs);record_frame(row,index,cs,rec,event,&full,start);
    if(!event)require(rec.extra==0,"ordinary frame incurred extra align");
    if(index==19)std::cout<<"SMOKE20_PASS no_feedback=1"<<std::endl;
    std::cout<<"NATURAL_FRAME "<<scan<<" event="<<event<<" extra="<<rec.extra<<" budget="<<budget.used<<std::endl;
  }
  const int natural_extra=budget.used;require(total==200+natural_extra,"natural align accounting");
  std::cout<<"NATURAL_COMPLETE events="<<natural_events<<" extra="<<natural_extra<<std::endl;
  if(natural_events==0){phase="DIAGNOSTIC_FORCED";
    for(int local:{20,40,60,80}){scan=local;const int index=local-1;const auto start=Clock::now();const auto& row=rows[index];const Pose initial=prediction(row);
      std::vector<Candidate> cs{cache[index]};cs[0].align_ms=0;cs[0].validation_ms=0;
      const auto rec=recover(ndt,splits[index],tree,initial,cs,budget,run);deduplicate(cs);record_frame(row,index,cs,rec,false,nullptr,start);
      std::cout<<"DIAGNOSTIC_FORCED frame="<<scan<<" direction="<<rec.direction<<" extra="<<rec.extra<<std::endl;
    }
    require(budget.used-natural_extra<=8,"diagnostic budget exceeded");
  }else std::cout<<"DIAGNOSTIC_SKIPPED natural_events_present"<<std::endl;
  require(total==200+budget.used&&budget.used<=20,"complete native accounting");
  std::cout<<"COMPLETE frames=100 native_aligns="<<total<<" natural_events="<<natural_events<<" natural_extra="<<natural_extra<<" diagnostic_extra="<<budget.used-natural_extra<<" feedback=0"<<std::endl;
}catch(const std::exception& error){std::cerr<<"STOP "<<error.what()<<std::endl;return 1;}}
