#include "policy.hpp"
#include <iostream>
using namespace r21;
static void matrix(std::ostream& out,const Pose& p){for(int i=0;i<4;++i)for(int j=0;j<4;++j){out<<',';number(out,p(i,j));}}
static void value(std::ostream& out,double x){out<<',';number(out,x);}
int main(int argc,char** argv){try{
  require(argc==7,"run_relative ROLE R19_SCANS R19_ANCHOR MAP R19_MAP_RECEIPT NEW_RUN_DIR");
  require(::mkdir(argv[6],0775)==0,"exclusive run directory; no overwrite");
  const auto process_start=Clock::now();const std::string role=argv[1],out=argv[6];require(role=="WEAK"||role=="STRONG","invalid window");
  const auto rows=readCsv(argv[2]),map_rows=readCsv(argv[5]);require(rows.size()==61&&map_rows.size()==1,"input shape");
  const auto map_start=Clock::now();Cloud::Ptr map(new Cloud);require(pcl::io::loadPCDFile(argv[4],*map)==0,"map unreadable");finish(map);
  require(map->size()==std::stoul(map_rows[0].at("target_points"))&&cloudHash(map)==map_rows[0].at("target_fnv"),"frozen map changed");
  NativeNdt ndt;ndt.configure(map);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);const double map_ms=milliseconds(map_start);
  const Pose anchor=readMatrix(argv[3]);const int64_t anchor_ns=std::stoll(rows[0].at("stamp_ns"));
  std::string head="role,arm,policy,frame,raw_ordinal,stamp_ns,source_fnv,prediction_mode,history_last_stamp_ns,nominal_basic_quality,nominal_gate_accepted,quality_event,event_reason,budget_before,extra_aligns,budget_after,candidate_count,finite_distinct_terminals,selected_id,selected_extra,accepted_observation,feedback_action,nominal_score,nominal_overlap,nominal_heldout_mean,selected_score,selected_overlap,selected_heldout_mean,shadow_5deg_selected,relative_candidate_count,relative_only_admitted,pending_created,confirmation_attempted,confirmation_success,confirmation_failed,pending_in,pending_out,admission_reason";
  poseHeader(head,"tp");poseHeader(head,"raw");poseHeader(head,"selected");poseHeader(head,"output");Csv causal(out+"/CAUSAL_RESULTS.csv",head);
  std::string ch="role,arm,policy,frame,raw_ordinal,stamp_ns,candidate_id,seed_kind,finite,converged,iterations,native_score,fitness,overlap_0p5,coverage_valid,active_blocks,heldout_mean,basic_quality,quality_gate_accepted,chosen,translation_from_prediction_m,rotation_from_prediction_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms,nominal_comparable,score_strictly_better,overlap_strictly_better,loss_strictly_better,physical_distinct,pareto_dominates,relative_only_candidate";
  for(int b=0;b<8;++b)ch+=",loss"+std::to_string(b);poseHeader(ch,"seed");poseHeader(ch,"pose");Csv candidate_log(out+"/RELATIVE_ADMISSION_LOG.csv",ch);
  std::string sh="role,arm,policy,frame,raw_ordinal,stamp_ns,pending_in,pending_source_frame,pending_source_stamp_ns,pending_source_candidate_id,confirmation_attempted,confirmation_candidate_id,confirmation_absolute,confirmation_relative,confirmation_success,confirmation_failed,pending_created,pending_out,status,formal_history_before_stamp_ns,formal_history_after_stamp_ns";
  poseHeader(sh,"pending_pose");poseHeader(sh,"confirmation_seed");Csv shadows(out+"/SHADOW_CONFIRMATION_LOG.csv",sh);
  Csv calls(out+"/ALIGN_CALLS.csv","role,arm,policy,frame,raw_ordinal,candidate_id,kind,align_ms,converged,iterations");
  Csv timing(out+"/RUNTIME_AND_MEMORY.csv","role,arm,policy,frame,raw_ordinal,event,extra_aligns,pcd_load_ms,preprocess_ms,prediction_ms,nominal_align_ms,extra_align_ms,confirmation_align_ms,aux_fitness_ms,validation_ms,decision_ms,frame_wall_ms,map_init_ms,peak_rss_KiB");
  timing.stream<<role<<",NA,MAP_INIT,0,"<<rows[0].at("raw_ordinal")<<",0,0,0,0,0,0,0,0,0,0,0,0,"<<map_ms<<','<<peakKiB();timing.end();
  Csv receipt(out+"/TRAJECTORY_RECEIPT.csv","role,arm,policy,nominal_calls,extra_calls,quality_events,accepted_extra,relative_only_admissions,pending_created,confirmation_attempts,confirmation_success,confirmation_failed,final_fallbacks,trajectory_wall_ms,peak_rss_KiB");
  int total_nominal=0,total_extra=0;
  for(const std::string arm:{"C0","C3"})for(bool temporal:{false,true}){
    const std::string policy=temporal?"TWO_FRAME_SHADOW_CONFIRMATION":"SINGLE_FRAME_RELATIVE_ADMISSION";
    Pose initial=anchor;if(arm=="C3")initial.block<3,3>(0,0)=Eigen::AngleAxisd(8.*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix()*anchor.block<3,3>(0,0);
    Predictor predictor(anchor_ns,initial);Pending pending;
    int used=0,events=0,extras_accepted=0,relative_accepted=0,created=0,attempts=0,confirmed=0,failed=0,fallbacks=0;const auto trajectory_start=Clock::now();
    for(int frame=1;frame<=60;++frame){
      const auto begin=Clock::now();const auto& row=rows[frame];require(std::stoi(row.at("local_index"))==frame,"index mismatch");
      const int64_t ns=std::stoll(row.at("stamp_ns"));require(ns>std::stoll(rows[frame-1].at("stamp_ns")),"nonmonotonic time");
      const Source source=pilot::loadSource(row);require(source.points->size()==std::stoul(row.at("expected_source_points"))&&cloudHash(source.points)==row.at("expected_source_fnv"),"Source changed");
      const auto pred_start=Clock::now();const std::string mode=predictor.hold?"HOLD_LAST":(predictor.history.size()<2?"LAST_POSE":"CONSTANT_VELOCITY");
      const auto history_ns=predictor.history.back().ns;const Pose tp=predictor.predict(ns);const double pred_ms=milliseconds(pred_start);
      const Pending prior=pending;require(!prior.active||temporal,"single-frame policy has pending state");
      if(prior.active)require(prior.frame==frame-1&&prior.ns<ns,"stale pending branch");
      std::vector<Candidate> cs;double fitness_ms=0,validation_ms=0,extra_ms=0,confirm_ms=0,decision_ms=0,extra_work_ms=0;
      auto native=[&](const Pose& seed,int id){
        const auto start=Clock::now();Candidate c=capturedAlign(ndt,source.points,seed,id,[](const Candidate&){});fitness_ms+=std::max(0.,milliseconds(start)-c.align_ms);
        calls.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<id<<','<<(id==0?"NOMINAL_FULL":id==3?"CONFIRMATION_FULL":"SEARCH_FULL")<<','<<c.align_ms<<','<<c.converged<<','<<c.iterations;calls.end();
        if(c.finite)coverage::validate(c,source.split,source.mask,tree,tp);validation_ms+=c.validation_ms;if(id>0)extra_work_ms+=milliseconds(start);return c;
      };
      cs.push_back(native(tp,0));++total_nominal;
      const auto decision_start=Clock::now();
      const bool basic=r17::quality(cs[0],source.mask),nominal_ok=accepted(cs[0],source.mask),event=!nominal_ok;if(event)++events;
      const int before=used;int chosen=-1;bool made=false,attempt=false,success=false,fail=false,relative=false,confirm_abs=false,confirm_relative=false;
      std::string reason="NONE",shadow_status="NONE";Pose confirm_seed=Pose::Constant(NAN);
      
      if(nominal_ok){
        chosen=0;reason="ABSOLUTE_NOMINAL";
        if(prior.active){pending.clear();shadow_status="CANCELLED_NOMINAL_ABSOLUTE_PASS";}
      }else if(prior.active){
        // Priority: one true next-frame confirmation, no new +/- search this frame.
        pending.clear();
        if(canConfirm(used)){
          confirm_seed=prior.seed(frame,ns);++used;++total_extra;attempt=true;++attempts;
          cs.push_back(native(confirm_seed,3));extra_ms+=cs.back().align_ms;confirm_ms=cs.back().align_ms;
          confirm_abs=accepted(cs.back(),source.mask);confirm_relative=!confirm_abs&&dominates(cs.back(),cs[0],source.mask);
          if(confirm_abs||confirm_relative){chosen=1;success=true;++confirmed;relative=confirm_relative;reason=confirm_abs?"CONFIRMED_ABSOLUTE":"CONFIRMED_RELATIVE_ONLY_ADMISSION";shadow_status=reason;}
          else{fail=true;++failed;reason="CONFIRMATION_REJECTED";shadow_status=reason;}
        }else{reason="CONFIRMATION_BUDGET_EXHAUSTED";shadow_status=reason;}
      }else if(canSearch(used)){
        const std::array<Pose,2> seeds{{yawSeed(tp,1),yawSeed(tp,-1)}};
        for(int k=0;k<2;++k){++used;++total_extra;cs.push_back(native(seeds[k],k+1));extra_ms+=cs.back().align_ms;}
        chosen=choose(cs,source.mask,tp);
        if(chosen>=0)reason="ABSOLUTE_SEARCH";
        else{
          const int rel=chooseRelative(cs,source.mask,tp);
          if(rel<0)reason=comparable(cs[0],source.mask)?"NO_RELATIVE_DOMINANCE":"NOMINAL_NOT_COMPARABLE";
          else if(!temporal){chosen=rel;relative=true;reason="RELATIVE_ONLY_ADMISSION";}
          else{
            pending.establish(frame,ns,cs[rel]);made=true;++created;reason="PENDING_SHADOW_CANDIDATE";shadow_status=reason;
            if(frame==60){pending.clear();shadow_status="PENDING_EXPIRED_WINDOW_END";}
          }
        }
      }else reason="SEARCH_BUDGET_EXHAUSTED";
      const bool accept=chosen>=0;
      const int chosen_id=accept?cs[chosen].id:-1;const Pose selected=accept?cs[chosen].pose:Pose::Constant(NAN);const Pose output=accept?selected:tp;require(rigid(output),"illegal causal output");
      int relative_count=0;for(size_t i=1;i<cs.size();++i)relative_count+=dominates(cs[i],cs[0],source.mask);
      if(accept&&chosen_id>0)++extras_accepted;if(relative)++relative_accepted;if(!accept)++fallbacks;
      const bool shadow=accept&&r17::base(cs[chosen],source.mask,tp,5.);
      r17::advance(predictor,ns,accept?cs[chosen]:cs[0],accept);
      require(used<=12&&used-before<=2,"extra budget exceeded");
      decision_ms=std::max(0.,milliseconds(decision_start)-extra_work_ms);
      causal.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<row.at("expected_source_fnv")<<','<<mode<<','<<history_ns<<','<<basic<<','<<nominal_ok<<','<<event<<','<<(event?(basic?"HELDOUT_GT_0P30":"BASIC_QUALITY_FAILED"):"NONE")<<','<<before<<','<<used-before<<','<<used<<','<<cs.size()<<','<<distinctCount(cs)<<','<<chosen_id<<','<<(chosen_id>0)<<','<<accept<<','<<(accept?chosen_id>0?"ACCEPT_EXTRA_FULL":"ACCEPT_NOMINAL_FULL":"PREDICTION_FALLBACK");
      value(causal.stream,cs[0].score);value(causal.stream,cs[0].overlap);value(causal.stream,lossMean(cs[0],source.mask));value(causal.stream,accept?cs[chosen].score:NAN);value(causal.stream,accept?cs[chosen].overlap:NAN);value(causal.stream,accept?lossMean(cs[chosen],source.mask):NAN);
      causal.stream<<','<<shadow<<','<<relative_count<<','<<relative<<','<<made<<','<<attempt<<','<<success<<','<<fail<<','<<prior.active<<','<<pending.active<<','<<reason;
      matrix(causal.stream,tp);matrix(causal.stream,cs[0].pose);matrix(causal.stream,selected);matrix(causal.stream,output);causal.end();
      for(const auto& c:cs){
        candidate_log.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<c.id<<','<<(c.id==0?"TP":c.id==1?"MAP_YAW_PLUS5":c.id==2?"MAP_YAW_MINUS5":"PENDING_LAST_POSE")<<','<<c.finite<<','<<c.converged<<','<<c.iterations;
        value(candidate_log.stream,c.score);value(candidate_log.stream,c.fitness);value(candidate_log.stream,c.overlap);candidate_log.stream<<','<<source.mask.evaluable()<<','<<source.mask.active.size();value(candidate_log.stream,lossMean(c,source.mask));
        candidate_log.stream<<','<<r17::quality(c,source.mask)<<','<<accepted(c,source.mask)<<','<<(c.id==chosen_id);
        value(candidate_log.stream,rigid(c.pose)?translation(c.pose,tp):NAN);value(candidate_log.stream,rigid(c.pose)?rotation(c.pose,tp):NAN);value(candidate_log.stream,rigid(c.pose)&&rigid(cs[0].pose)?translation(c.pose,cs[0].pose):NAN);value(candidate_log.stream,rigid(c.pose)&&rigid(cs[0].pose)?rotation(c.pose,cs[0].pose):NAN);value(candidate_log.stream,c.align_ms);value(candidate_log.stream,c.validation_ms);
        const bool comp=comparable(cs[0],source.mask),dom=c.id>0&&dominates(c,cs[0],source.mask);
        candidate_log.stream<<','<<comp<<','<<(comp&&std::isfinite(c.score)&&c.score>cs[0].score)<<','<<(comp&&std::isfinite(c.overlap)&&c.overlap>cs[0].overlap)<<','<<(comp&&std::isfinite(lossMean(c,source.mask))&&lossMean(c,source.mask)<lossMean(cs[0],source.mask))<<','<<(safeNative(c)&&safeNative(cs[0])&&!duplicate(c.pose,cs[0].pose))<<','<<dom<<','<<(dom&&!accepted(c,source.mask));
        for(int b=0;b<8;++b)value(candidate_log.stream,source.split.blocks[b]->size()>=10&&c.loss_valid?c.loss[b]:NAN);matrix(candidate_log.stream,c.initial);matrix(candidate_log.stream,c.pose);candidate_log.end();
      }
      shadows.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<prior.active<<','<<(prior.active?prior.frame:-1)<<','<<(prior.active?prior.ns:0)<<','<<(prior.active?prior.id:-1)<<','<<attempt<<','<<(attempt?3:-1)<<','<<confirm_abs<<','<<confirm_relative<<','<<success<<','<<fail<<','<<made<<','<<pending.active<<','<<shadow_status<<','<<history_ns<<','<<predictor.history.back().ns;
      matrix(shadows.stream,prior.active?prior.pose:Pose::Constant(NAN));matrix(shadows.stream,confirm_seed);shadows.end();
      timing.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<event<<','<<used-before<<','<<source.load_ms<<','<<source.preprocess_ms<<','<<pred_ms<<','<<cs[0].align_ms<<','<<extra_ms<<','<<confirm_ms<<','<<fitness_ms<<','<<validation_ms<<','<<decision_ms<<','<<milliseconds(begin)<<",0,"<<peakKiB();timing.end();
    }
    require(!pending.active&&used<=12,"unfinished pending or excess budget");
    receipt.stream<<role<<','<<arm<<','<<policy<<",60,"<<used<<','<<events<<','<<extras_accepted<<','<<relative_accepted<<','<<created<<','<<attempts<<','<<confirmed<<','<<failed<<','<<fallbacks<<','<<milliseconds(trajectory_start)<<','<<peakKiB();receipt.end();
    std::cout<<"TRAJECTORY_COMPLETE "<<role<<' '<<arm<<' '<<policy<<" extra="<<used<<" events="<<events<<" accepted_extra="<<extras_accepted<<" relative="<<relative_accepted<<" pending="<<created<<" confirm="<<confirmed<<'/'<<attempts<<" fallbacks="<<fallbacks<<std::endl;
  }
  require(total_nominal==240&&total_extra<=48,"window call budget");std::cout<<"R21_NATIVE_COMPLETE "<<role<<" nominal="<<total_nominal<<" extra="<<total_extra<<" wall_ms="<<milliseconds(process_start)<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"R21_NATIVE_STOP "<<e.what()<<std::endl;return 1;}}
