#include "policy.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/native_capture.hpp"
#include <iostream>
#include <memory>
#include <sys/stat.h>

using namespace r17;
static std::string flat(const Pose& p){std::ostringstream o;o<<std::setprecision(17);
  for(int i=0;i<4;++i)for(int j=0;j<4;++j){if(i||j)o<<'|';o<<p(i,j);}return o.str();}
struct Arm {
  Policy policy;Predictor predictor;int accepted=0,rejected=0,events=0,extra=0,full_calls=0,max_hold=0,hold_run=0;
  Arm(Policy p,int64_t ns,const Pose& anchor):policy(p),predictor(ns,anchor){}
};
int main(int argc,char** argv){try{
  require(argc==6,"run_policies SCANS ANCHOR MAP MAP_BUILD OUTPUT_DIRECTORY");
  require(::mkdir(argv[5],0775)==0,"exclusive output directory required");
  const auto process_start=Clock::now();const std::string out=argv[5];
  const auto rows=readCsv(argv[1]),map_rows=readCsv(argv[4]);
  require(rows.size()==31&&map_rows.size()==1,"fixed R16 inputs missing");
  const auto map_start=Clock::now();Cloud::Ptr target(new Cloud);
  require(pcl::io::loadPCDFile(argv[3],*target)==0,"frozen map unreadable");finish(target);
  require(target->size()==std::stoul(map_rows[0].at("target_points"))&&cloudHash(target)==map_rows[0].at("target_fnv"),"frozen map changed");
  NativeNdt ndt;ndt.configure(target);const double map_ms=milliseconds(map_start);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  Pose anchor=readMatrix(argv[2]);const int64_t t0=std::stoll(rows[0].at("stamp_ns"));
  std::vector<Arm> arms;arms.emplace_back(Policy::Baseline,t0,anchor);arms.emplace_back(Policy::Relaxed,t0,anchor);arms.emplace_back(Policy::Consensus,t0,anchor);
  Csv decision(out+"/ADMISSION_DECISIONS.csv","frame,raw_ordinal,stamp_ns,policy,full_converged,full_finite,coverage_valid,full_overlap,full_translation_TP_m,full_rotation_TP_deg,confirmation_requested,split_executed,split_converged,split_finite,split_overlap,full_split_translation_m,full_split_rotation_deg,accepted,decision,hold_run,full_iterations,full_score,tp_pose,full_pose,split_pose,output_pose");
  Csv runtime(out+"/RUNTIME_AND_MEMORY.csv","frame,raw_ordinal,policy,pcd_load_ms,preprocess_ms,predict_ms,full_align_ms,full_aux_ms,full_validation_ms,split_align_ms,split_aux_ms,split_validation_ms,decision_ms,arm_wall_ms,map_init_ms,peak_rss_KiB");
  Csv calls(out+"/ALIGN_CALLS.csv","frame,raw_ordinal,policy,kind,align_ms,converged,iterations");
  int extra_total=0;int full_total=0;
  for(int frame=1;frame<=30;++frame){
    const auto& row=rows[frame];const int64_t ns=std::stoll(row.at("stamp_ns"));
    require(std::stoi(row.at("local_index"))==frame&&std::stoi(row.at("raw_ordinal"))==7552+frame&&ns>std::stoll(rows[frame-1].at("stamp_ns")),"window identity changed");
    Source source=pilot::loadSource(row);
    require(source.points->size()==std::stoul(row.at("expected_source_points"))&&cloudHash(source.points)==row.at("expected_source_fnv")&&source.mask.evaluable(),"frozen source/coverage changed");
    for(auto& arm:arms){
      const auto arm_start=Clock::now();const auto pred_start=Clock::now();Pose tp=arm.predictor.predict(ns);double pred_ms=milliseconds(pred_start);
      auto run=[&](const Cloud::Ptr& cloud,int id,const char* kind){
        const auto start=Clock::now();Candidate c=capturedAlign(ndt,cloud,tp,id,[](const Candidate&){});
        double aux=milliseconds(start)-c.align_ms;
        if(c.finite)coverage::validate(c,source.split,source.mask,tree,tp);
        calls.stream<<frame<<','<<row.at("raw_ordinal")<<','<<name(arm.policy)<<','<<kind<<','<<c.align_ms<<','<<c.converged<<','<<c.iterations;calls.end();
        return std::make_pair(c,aux);
      };
      auto full_run=run(source.points,0,"FULL");Candidate full=full_run.first;Candidate split;bool split_executed=false;double split_aux=0;
      ++full_total;++arm.full_calls;
      const auto decision_start=Clock::now();bool requested=needsConfirmation(full,source.mask,tp);
      bool accepted=base(full,source.mask,tp,arm.policy==Policy::Relaxed?6.:5.);
      if(arm.policy==Policy::Consensus&&requested){
        ++arm.events;
        if(extra_total<3){auto split_run=run(source.split.training,1,"EVENT_SPLIT");split=split_run.first;split_aux=split_run.second;
          split_executed=true;++extra_total;++arm.extra;
          accepted=confirmed(full,split,source.mask); // only the (5,6] exception
        }
      }
      // A relaxed-arm event is the same Full predicate, without an extra align.
      if(arm.policy==Policy::Relaxed&&requested)++arm.events;
      if(arm.policy==Policy::Baseline&&requested)++arm.events;
      std::string status=accepted?(arm.policy==Policy::Consensus&&requested?"CONDITIONAL_ACCEPT":"ACCEPT_FULL"):
        (requested&&arm.policy==Policy::Consensus&&!split_executed?"BUDGET_EXHAUSTED_FALLBACK":"REJECT_PREDICTION_FALLBACK");
      advance(arm.predictor,ns,full,accepted);
      if(accepted){++arm.accepted;arm.hold_run=0;}else{++arm.rejected;++arm.hold_run;arm.max_hold=std::max(arm.max_hold,arm.hold_run);}
      // The fallback is the causal TP used at this scan; next prediction holds last admitted pose.
      const Pose output_pose=accepted?full.pose:tp;const double decision_ms=milliseconds(decision_start);
      const double dt=rigid(full.pose)?translation(full.pose,tp):NAN;
      const double dr=rigid(full.pose)?rotation(full.pose,tp):NAN;
      decision.stream<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<name(arm.policy)<<','<<full.converged<<','<<full.finite<<','<<source.mask.evaluable()<<',';
      number(decision.stream,full.overlap);decision.stream<<',';number(decision.stream,dt);decision.stream<<',';number(decision.stream,dr);
      decision.stream<<','<<requested<<','<<split_executed<<','<<split.converged<<','<<split.finite<<',';
      number(decision.stream,split.overlap);decision.stream<<',';
      number(decision.stream,split_executed&&rigid(split.pose)?translation(full.pose,split.pose):NAN);decision.stream<<',';
      number(decision.stream,split_executed&&rigid(split.pose)?rotation(full.pose,split.pose):NAN);
      decision.stream<<','<<accepted<<','<<status<<','<<arm.hold_run<<','<<full.iterations<<',';number(decision.stream,full.score);
      decision.stream<<','<<flat(tp)<<','<<flat(full.pose)<<','<<(split_executed?flat(split.pose):"NA")<<','<<flat(output_pose);decision.end();
      runtime.stream<<frame<<','<<row.at("raw_ordinal")<<','<<name(arm.policy)<<','<<source.load_ms<<','<<source.preprocess_ms<<','<<pred_ms<<','<<full.align_ms<<','<<full_run.second<<','<<full.validation_ms<<','
        <<(split_executed?split.align_ms:0)<<','<<split_aux<<','<<(split_executed?split.validation_ms:0)<<','<<decision_ms<<','<<milliseconds(arm_start)<<",0,"<<peakKiB();runtime.end();
    }
    std::cout<<"FRAME "<<frame<<" raw="<<row.at("raw_ordinal")<<" accepted="<<arms[0].accepted<<'/'<<arms[1].accepted<<'/'<<arms[2].accepted<<" extra="<<extra_total<<std::endl;
  }
  require(full_total==90&&extra_total<=3,"NDT call budget/inventory mismatch");
  for(const auto& arm:arms)std::cout<<name(arm.policy)<<" accepted="<<arm.accepted<<" rejected="<<arm.rejected<<" events="<<arm.events<<" extra="<<arm.extra<<" max_hold="<<arm.max_hold<<std::endl;
  std::cout<<"R17_NATIVE_COMPLETE FULL="<<full_total<<" EXTRA_SPLIT="<<extra_total<<" map_init_ms="<<map_ms<<" wall_ms="<<milliseconds(process_start)<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"R17_NATIVE_STOP "<<e.what()<<std::endl;return 1;}}
