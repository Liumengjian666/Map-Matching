#pragma once
#include "common.hpp"
namespace pilot {
inline std::string withPoses(std::string s,std::initializer_list<std::string> names){for(const auto& n:names)poseHeader(s,n);return s;}
inline std::string idsText(const std::vector<int>& ids){std::string s;for(int i:ids){if(!s.empty())s+='|';s+=std::to_string(i);}return s.empty()?"NA":s;}
struct Records {
  Csv raw,candidates,blocks,pairs,final,events,runtime,coarse,selectors,calls,sources,map;
  explicit Records(const std::string& p):
    raw(p+"/RAW_OPTIMIZER_OUTPUTS.csv",withPoses("frame,method,candidate_id,grid_id,call,stamp_ns,converged,iterations,native_score,align_ms",{"seed_","final_"})),
    candidates(p+"/NATIVE_NDT_RESULTS.csv",withPoses("frame,raw_ordinal,stamp_ns,method,candidate_id,grid_id,finite,converged,iterations,native_score,fitness,overlap,B,mean_loss,eligible,duplicate_of,group_id,translation_from_TP_m,rotation_from_TP_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms",{"seed_","final_"})),
    blocks(p+"/BLOCK_LOSSES.csv","frame,method,candidate_id,block,points,active,loss"),
    pairs(p+"/PAIRWISE_EVIDENCE.csv","frame,B,j,k,D_j_k"),
    final(p+"/FINAL_OBSERVATIONS.csv",withPoses("frame,stamp_ns,proposed_observation_valid,status,full_admitted,full_history_last_ns,full_history_count,predictor_hold_next",{"prediction_","full_","split_","accepted_"})),
    events(p+"/ACCEPTANCE_AND_EVENTS.csv","frame,stamp_ns,B,full_eligible,split_eligible,old_event_available,old_event,detected,permitted,reasons,permission,extra_calls,budget_used,distinct_groups,eligible_raw_alternatives,eligible_distinct_groups,robust_supported,recovery_accepted,observation_valid,status"),
    runtime(p+"/RUNTIME_AND_MEMORY.csv","frame,pcd_read_convert_ms,source_preprocess_ms,prediction_ms,full_align_ms,split_align_ms,event_ms,coarse_grid_ms,seed_selection_ms,extra_align_ms,fitness_aux_ms,coverage_ms,V3_decision_ms,frame_wall_ms,peak_rss_KiB"),
    coarse(p+"/COARSE_GRID_SCORES.csv",withPoses("frame,grid_id,dx_m,dy_m,yaw_deg,coarse_train_points,coarse_loss,coarse_overlap,local_excluded,selected_rank,reason",{"native_seed_"})),
    selectors(p+"/SELECTOR_COMPARISON.csv","frame,representatives,evidence,native,heldout_mean,robust,selected,status"),
    calls(p+"/ALIGN_CALLS.csv","frame,method,candidate_id,grid_id,call,status,extra_budget_used"),
    sources(p+"/SOURCE_RUNTIME.csv","frame,stamp_ns,source_points,source_fnv,training_points,B,val0,val1,val2,val3,val4,val5,val6,val7"),
    map(p+"/MAP_RUNTIME.csv","target_points,target_fnv,Gaussian_grid_m,map_load_grid_tree_ms,peak_rss_KiB"){
    for(Csv* c:{&raw,&candidates,&blocks,&pairs,&final,&events,&runtime,&coarse,&selectors,&calls,&sources,&map})c->stream.exceptions(std::ios::badbit|std::ios::failbit);
  }
  void capture(int frame,int64_t ns,const std::string& method,int id,int grid,int call,const Candidate& c){
    raw.stream<<frame<<','<<method<<','<<id<<','<<grid<<','<<call<<','<<ns<<','<<c.converged<<','<<c.iterations<<',';number(raw.stream,c.score);raw.stream<<','<<c.align_ms;emitPose(raw.stream,c.initial);emitPose(raw.stream,c.pose);raw.end();
  }
  void candidate(int frame,const Row& row,const std::string& method,int grid,const Candidate& c,const Source& s,const Pose& tp,const Pose& nominal,int group){
    candidates.stream<<frame<<','<<row.at("raw_ordinal")<<','<<row.at("stamp_ns")<<','<<method<<','<<c.id<<','<<grid<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';
    number(candidates.stream,c.score);candidates.stream<<',';number(candidates.stream,c.fitness);candidates.stream<<',';number(candidates.stream,c.overlap);
    candidates.stream<<','<<s.mask.active.size()<<',';number(candidates.stream,c.loss_valid?coverage::mean(c.loss,s.mask):NAN);
    candidates.stream<<','<<admissible(c,s.mask,tp)<<','<<c.duplicate_of<<','<<group<<','<<translation(c.pose,tp)<<','<<rotation(c.pose,tp)<<','<<translation(c.pose,nominal)<<','<<rotation(c.pose,nominal)<<','<<c.align_ms<<','<<c.validation_ms;
    emitPose(candidates.stream,c.initial);emitPose(candidates.stream,c.pose);candidates.end();
    for(int b=0;b<8;++b){blocks.stream<<frame<<','<<method<<','<<c.id<<','<<b<<','<<s.split.blocks[b]->size()<<','<<(std::find(s.mask.active.begin(),s.mask.active.end(),b)!=s.mask.active.end())<<',';number(blocks.stream,c.loss[b]);blocks.end();}
  }
};
} // namespace pilot
