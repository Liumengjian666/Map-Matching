#include "policy.hpp"
#include <iostream>
using namespace r20;
static void matrix(std::ostream& out,const Pose& p){for(int i=0;i<4;++i)for(int j=0;j<4;++j){out<<',';number(out,p(i,j));}}
static void value(std::ostream& out,double x){out<<',';number(out,x);}

int main(int argc,char** argv){try{
  require(argc==7,"run_quality ROLE R19_SCANS R19_ANCHOR MAP R19_MAP_RECEIPT NEW_RUN_DIR");
  require(::mkdir(argv[6],0775)==0,"exclusive run directory; no overwrite");
  const auto process_start=Clock::now();const std::string role=argv[1],out=argv[6];
  require(role=="WEAK"||role=="STRONG","invalid window");
  const auto rows=readCsv(argv[2]),map_rows=readCsv(argv[5]);
  require(rows.size()==61&&map_rows.size()==1,"frozen input shape changed");
  const auto map_start=Clock::now();Cloud::Ptr map(new Cloud);
  require(pcl::io::loadPCDFile(argv[4],*map)==0,"map unreadable");finish(map);
  require(map->size()==std::stoul(map_rows[0].at("target_points"))&&cloudHash(map)==map_rows[0].at("target_fnv"),"frozen map identity mismatch");
  NativeNdt ndt;ndt.configure(map);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);const double map_ms=milliseconds(map_start);
  const Pose anchor=readMatrix(argv[3]);const int64_t anchor_ns=std::stoll(rows[0].at("stamp_ns"));
  std::string head="role,arm,policy,frame,raw_ordinal,stamp_ns,source_fnv,prediction_mode,history_last_stamp_ns,nominal_basic_quality,nominal_gate_accepted,quality_event,event_reason,budget_before,extra_aligns,budget_after,candidate_count,finite_distinct_terminals,selected_id,selected_extra,accepted_observation,feedback_action,nominal_score,nominal_overlap,nominal_heldout_mean,selected_score,selected_overlap,selected_heldout_mean,shadow_5deg_selected";
  poseHeader(head,"tp");poseHeader(head,"raw");poseHeader(head,"selected");poseHeader(head,"output");Csv causal(out+"/CAUSAL_RESULTS.csv",head);
  std::string ch="role,arm,policy,frame,raw_ordinal,stamp_ns,candidate_id,seed_kind,finite,converged,iterations,native_score,fitness,overlap_0p5,coverage_valid,active_blocks,heldout_mean,basic_quality,quality_gate_accepted,chosen,translation_from_prediction_m,rotation_from_prediction_deg,translation_from_nominal_m,rotation_from_nominal_deg,align_ms,validation_ms";
  for(int b=0;b<8;++b)ch+=",loss"+std::to_string(b);poseHeader(ch,"seed");poseHeader(ch,"pose");Csv candidates_log(out+"/EVENT_AND_CANDIDATE_LOG.csv",ch);
  Csv calls(out+"/ALIGN_CALLS.csv","role,arm,policy,frame,raw_ordinal,candidate_id,kind,align_ms,converged,iterations");
  Csv timing(out+"/RUNTIME_AND_MEMORY.csv","role,arm,policy,frame,raw_ordinal,event,extra_aligns,pcd_load_ms,preprocess_ms,prediction_ms,nominal_align_ms,extra_align_ms,aux_fitness_ms,validation_ms,decision_ms,frame_wall_ms,map_init_ms,peak_rss_KiB");
  timing.stream<<role<<",NA,MAP_INIT,0,"<<rows[0].at("raw_ordinal")<<",0,0,0,0,0,0,0,0,0,0,0,"<<map_ms<<','<<peakKiB();timing.end();
  Csv trajectories(out+"/TRAJECTORY_RECEIPT.csv","role,arm,policy,nominal_calls,extra_calls,quality_events,accepted_extra,final_fallbacks,trajectory_wall_ms,peak_rss_KiB");
  int total_nominal=0,total_extra=0;
  for(const std::string arm:{"C0","C3"})for(bool multi:{false,true}){
    const std::string policy=multi?"QUALITY_TRIGGERED_MULTISTART":"QUALITY_GATE_ONLY";
    Pose initial=anchor;if(arm=="C3")initial.block<3,3>(0,0)=Eigen::AngleAxisd(8.*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix()*anchor.block<3,3>(0,0);
    require(rigid(initial),"invalid anchor");Predictor predictor(anchor_ns,initial);
    int used=0,events=0,recovered=0,fallbacks=0;const auto trajectory_start=Clock::now();
    for(int frame=1;frame<=60;++frame){
      const auto begin=Clock::now();const auto& row=rows[frame];
      require(std::stoi(row.at("local_index"))==frame,"frozen index mismatch");
      const int64_t ns=std::stoll(row.at("stamp_ns"));require(ns>std::stoll(rows[frame-1].at("stamp_ns")),"nonmonotonic time");
      const Source source=pilot::loadSource(row);
      require(source.points->size()==std::stoul(row.at("expected_source_points"))&&cloudHash(source.points)==row.at("expected_source_fnv"),"Source identity changed");
      const auto pred_start=Clock::now();const std::string mode=predictor.hold?"HOLD_LAST":(predictor.history.size()<2?"LAST_POSE":"CONSTANT_VELOCITY");
      const auto history_ns=predictor.history.back().ns;const Pose tp=predictor.predict(ns);const double pred_ms=milliseconds(pred_start);
      std::vector<Candidate> cs;double fitness_ms=0,validation_ms=0,extra_ms=0;
      auto native=[&](const Pose& seed,int id){
        const auto start=Clock::now();Candidate c=capturedAlign(ndt,source.points,seed,id,[](const Candidate&){});
        fitness_ms+=std::max(0.,milliseconds(start)-c.align_ms);
        calls.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<id<<','<<(id==0?"NOMINAL_FULL":"EXTRA_FULL")<<','<<c.align_ms<<','<<c.converged<<','<<c.iterations;calls.end();
        if(c.finite)coverage::validate(c,source.split,source.mask,tree,tp);validation_ms+=c.validation_ms;return c;
      };
      cs.push_back(native(tp,0));++total_nominal;
      const bool basic=r17::quality(cs[0],source.mask),nominal_ok=accepted(cs[0],source.mask),event=!nominal_ok;
      if(event)++events;const int before=used;std::string reason=event?(basic?"HELDOUT_GT_0P30":"BASIC_QUALITY_FAILED"):"NONE";
      if(maySearch(multi,event,used)){
        // Both seeds fixed before the first extra result. No sequential adaptation.
        const std::array<Pose,2> seeds{{yawSeed(tp,1),yawSeed(tp,-1)}};
        for(int k=0;k<2;++k){++used;++total_extra;cs.push_back(native(seeds[k],k+1));extra_ms+=cs.back().align_ms;}
      }
      const auto decision_start=Clock::now();const int chosen=choose(cs,source.mask,tp);const bool accept=chosen>=0;
      const int distinct=distinctCount(cs);const Pose missing=Pose::Constant(NAN);const Pose selected=accept?cs[chosen].pose:missing;
      const Pose output=accept?selected:tp;require(rigid(output),"illegal causal output");
      if(accept&&chosen>0)++recovered;if(!accept)++fallbacks;
      const bool shadow=accept&&r17::base(cs[chosen],source.mask,tp,5.);
      r17::advance(predictor,ns,accept?cs[chosen]:cs[0],accept);const double decision_ms=milliseconds(decision_start);
      causal.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<row.at("expected_source_fnv")<<','<<mode<<','<<history_ns<<','<<basic<<','<<nominal_ok<<','<<event<<','<<reason<<','<<before<<','<<used-before<<','<<used<<','<<cs.size()<<','<<distinct<<','<<chosen<<','<<(chosen>0)<<','<<accept<<','<<(accept?chosen>0?"ACCEPT_EXTRA_FULL":"ACCEPT_NOMINAL_FULL":"PREDICTION_FALLBACK");
      value(causal.stream,cs[0].score);value(causal.stream,cs[0].overlap);value(causal.stream,lossMean(cs[0],source.mask));
      value(causal.stream,accept?cs[chosen].score:NAN);value(causal.stream,accept?cs[chosen].overlap:NAN);value(causal.stream,accept?lossMean(cs[chosen],source.mask):NAN);causal.stream<<','<<shadow;
      matrix(causal.stream,tp);matrix(causal.stream,cs[0].pose);matrix(causal.stream,selected);matrix(causal.stream,output);causal.end();
      for(const auto& c:cs){
        candidates_log.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<c.id<<','<<(c.id==0?"TP":c.id==1?"MAP_YAW_PLUS5":"MAP_YAW_MINUS5")<<','<<c.finite<<','<<c.converged<<','<<c.iterations;
        value(candidates_log.stream,c.score);value(candidates_log.stream,c.fitness);value(candidates_log.stream,c.overlap);
        candidates_log.stream<<','<<source.mask.evaluable()<<','<<source.mask.active.size();value(candidates_log.stream,lossMean(c,source.mask));
        candidates_log.stream<<','<<r17::quality(c,source.mask)<<','<<accepted(c,source.mask)<<','<<(c.id==chosen);
        value(candidates_log.stream,rigid(c.pose)?translation(c.pose,tp):NAN);value(candidates_log.stream,rigid(c.pose)?rotation(c.pose,tp):NAN);
        value(candidates_log.stream,rigid(c.pose)&&rigid(cs[0].pose)?translation(c.pose,cs[0].pose):NAN);value(candidates_log.stream,rigid(c.pose)&&rigid(cs[0].pose)?rotation(c.pose,cs[0].pose):NAN);
        value(candidates_log.stream,c.align_ms);value(candidates_log.stream,c.validation_ms);
        for(int b=0;b<8;++b)value(candidates_log.stream,source.split.blocks[b]->size()>=10&&c.loss_valid?c.loss[b]:NAN);
        matrix(candidates_log.stream,c.initial);matrix(candidates_log.stream,c.pose);candidates_log.end();
      }
      timing.stream<<role<<','<<arm<<','<<policy<<','<<frame<<','<<row.at("raw_ordinal")<<','<<event<<','<<used-before<<','<<source.load_ms<<','<<source.preprocess_ms<<','<<pred_ms<<','<<cs[0].align_ms<<','<<extra_ms<<','<<fitness_ms<<','<<validation_ms<<','<<decision_ms<<','<<milliseconds(begin)<<",0,"<<peakKiB();timing.end();
    }
    require(used<=EXTRA_LIMIT&&(!multi?used==0:true),"trajectory budget exceeded");
    trajectories.stream<<role<<','<<arm<<','<<policy<<",60,"<<used<<','<<events<<','<<recovered<<','<<fallbacks<<','<<milliseconds(trajectory_start)<<','<<peakKiB();trajectories.end();
    std::cout<<"TRAJECTORY_COMPLETE "<<role<<' '<<arm<<' '<<policy<<" extra="<<used<<" events="<<events<<" accepted_extra="<<recovered<<" fallbacks="<<fallbacks<<std::endl;
  }
  require(total_nominal==240&&total_extra<=24,"window align budget exceeded");
  std::cout<<"R20_NATIVE_COMPLETE "<<role<<" nominal="<<total_nominal<<" extra="<<total_extra<<" wall_ms="<<milliseconds(process_start)<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"R20_NATIVE_STOP "<<e.what()<<std::endl;return 1;}}
