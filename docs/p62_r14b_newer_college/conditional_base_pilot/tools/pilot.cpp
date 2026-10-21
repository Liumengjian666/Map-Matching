#include "records.hpp"
#include <iostream>
using namespace pilot;
int main(int argc,char** argv){try{
  require(argc==7,"pilot SCANS ANCHOR MAP SOURCE_PREFLIGHT MAP_PREFLIGHT NEW_OUT");
  require(::mkdir(argv[6],0775)==0,"exclusive formal run, no retry");const auto process_start=Clock::now();
  const auto rows=readCsv(argv[1]),expected=readCsv(argv[4]),maps=readCsv(argv[5]);
  require(rows.size()==21&&expected.size()==21&&maps.size()==1,"fixed21 plus one map");
  Records log(argv[6]);const auto map_start=Clock::now();Cloud::Ptr target(new Cloud);
  require(pcl::io::loadPCDFile(argv[3],*target)==0,"frozen map read failed");
  for(const auto& p:*target)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"nonfinite frozen map");finish(target);
  require(target->size()==std::stoul(maps[0].at("target_points"))&&cloudHash(target)==maps[0].at("target_fnv"),"frozen map identity");
  NativeNdt ndt;ndt.configure(target);pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);
  log.map.stream<<target->size()<<','<<cloudHash(target)<<",0.8,"<<milliseconds(map_start)<<','<<peakKiB();log.map.end();
  const Pose anchor=readMatrix(argv[2]);Predictor predictor(std::stoll(rows[0].at("stamp_ns")),anchor);
  Budget budget;int total=0,events=0,accepted=0,recoveries=0;
  for(int frame=1;frame<=20;++frame){
    const auto frame_start=Clock::now();const auto& row=rows[frame];const int64_t ns=std::stoll(row.at("stamp_ns"));
    require(std::stoi(row.at("local_index"))==frame&&ns>std::stoll(rows[frame-1].at("stamp_ns")),"formal frame identity/time");
    auto s=pilot::loadSource(row);
    require(s.points->size()==std::stoul(expected[frame].at("source_points"))&&cloudHash(s.points)==expected[frame].at("source_fnv"),"Source changed after preregistered native preflight");
    require(std::to_string(s.mask.active.size())==expected[frame].at("B")&&s.split.training->size()==std::stoul(expected[frame].at("training_points")),"training/active identity");
    log.sources.stream<<frame<<','<<ns<<','<<s.points->size()<<','<<cloudHash(s.points)<<','<<s.split.training->size()<<','<<s.mask.active.size();
    for(int b=0;b<8;++b){require(s.split.blocks[b]->size()==std::stoul(expected[frame].at("val"+std::to_string(b))),"block identity");log.sources.stream<<','<<s.split.blocks[b]->size();}log.sources.end();
    auto clock=Clock::now();const Pose tp=predictor.predict(ns);const double prediction_ms=milliseconds(clock);
    double fitness_aux=0,coverage_ms=0,extra_ms=0,coarse_ms=0,seed_ms=0;int frame_extra=0;
    auto run=[&](const std::string& method,const Cloud::Ptr& cloud,const Pose& seed,int id,int grid){
      ++total;require(total<=60,"40nominal plus20extra maximum");
      log.calls.stream<<frame<<','<<method<<','<<id<<','<<grid<<','<<total<<",START,"<<budget.used;log.calls.end();
      const auto phase=Clock::now();auto capture=[&](const Candidate& c){log.capture(frame,ns,method,id,grid,total,c);};
      auto c=capturedAlign(ndt,cloud,seed,id,capture);fitness_aux+=std::max(0.,milliseconds(phase)-c.align_ms);
      log.calls.stream<<frame<<','<<method<<','<<id<<','<<grid<<','<<total<<",RETURNED,"<<budget.used;log.calls.end();
      require(rigid(c.pose)&&safeNative(c),"native invalid/nonfinite result recorded; stop before next align");
      coverage::validate(c,s.split,s.mask,tree,tp);coverage_ms+=c.validation_ms;
      require(c.eligible==admissible(c,s.mask,tp),"unchanged validation/admission mismatch");return c;
    };
    Candidate full=run("CONTROL_FULL",s.points,tp,-1,-1);
    std::vector<Candidate> candidates{run("CONTROL_SPLIT",s.split.training,tp,0,-1)};
    std::vector<int> origins{-1};clock=Clock::now();const auto event=detect(candidates[0],s.mask,tp,budget.used);const double event_ms=milliseconds(clock);events+=event.detected;
    if(event.permitted){
      auto seeds=training_grid::grid(tp);const auto subset=training_grid::subset(*s.split.training);
      std::vector<int> ids(1);std::vector<float> squared(1);clock=Clock::now();
      const auto queries=training_grid::score(seeds,*s.split.training,subset,[&](const Eigen::Vector3d& y){require(tree.nearestKSearch(pcl::PointXYZ(y.x(),y.y(),y.z()),1,ids,squared)==1,"coarse NN failure");return double(squared[0]);});
      require(queries==45*subset.size(),"fixed45 coarse query count");coarse_ms=milliseconds(clock);clock=Clock::now();
      const auto selected=training_grid::select(seeds,candidates[0].pose);seed_ms=milliseconds(clock);
      for(const auto& seed:seeds){log.coarse.stream<<frame<<','<<seed.id<<','<<seed.dx<<','<<seed.dy<<','<<seed.yaw_deg<<','<<subset.size()<<','<<seed.loss<<','<<seed.overlap<<','<<seed.local_excluded<<','<<seed.selected_rank<<','<<seed.reason;emitPose(log.coarse.stream,seed.native_pose);log.coarse.end();}
      for(int grid:selected){if(!budget.consume())break;require(frame_extra<2,"max2/event");
        auto c=run("R13_EXTRA",s.split.training,seeds[grid].pose,candidates.size(),grid);extra_ms+=c.align_ms;++frame_extra;candidates.push_back(c);origins.push_back(grid);}
    }
    if(!event.detected)require(frame_extra==0,"ordinary frame extra must0");
    clock=Clock::now();const auto pool=pool_v3::construct(candidates,s.mask,tp);const auto selection=pool_v3::select(pool,s.mask,tp);const auto decision=pool_v3::decide(pool,s.mask,tp,selection);const double decision_ms=milliseconds(clock);
    int distinct=0,eligible_distinct=0,eligible_raw=0;
    for(size_t g=0;g<pool.groups.size();++g)if(int(g)!=pool.group_of[0]){++distinct;eligible_distinct+=pool.raw[pool.groups[g].representative].eligible;}
    log.candidate(frame,row,"CONTROL_FULL",-1,full,s,tp,pool.raw[0].pose,-1);
    for(size_t j=0;j<pool.raw.size();++j){const auto& c=pool.raw[j];if(j)eligible_raw+=admissible(c,s.mask,tp);log.candidate(frame,row,j?"R13_EXTRA":"CONTROL_SPLIT",origins[j],c,s,tp,pool.raw[0].pose,pool.group_of[j]);}
    for(int j:pool.evidence)for(int k:pool.evidence)if(j!=k){log.pairs.stream<<frame<<','<<s.mask.active.size()<<','<<j<<','<<k<<','<<coverage::paired(pool.raw[j].loss,pool.raw[k].loss,s.mask);log.pairs.end();}
    log.selectors.stream<<frame<<','<<idsText(pool.representatives)<<','<<idsText(pool.evidence)<<','<<selection.native<<','<<selection.heldout_mean<<','<<selection.robust<<','<<decision.selected<<','<<decision.status;log.selectors.end();
    const bool full_admitted=admissible(full,s.mask,tp);predictor.completed(ns,full,s.mask,tp);
    if(decision.observation_valid)require(admissible(pool.raw[decision.selected],s.mask,tp),"invalid final map observation");
    log.final.stream<<frame<<','<<ns<<','<<decision.observation_valid<<','<<decision.status<<','<<full_admitted<<','<<predictor.history.back().ns<<','<<predictor.history.size()<<','<<predictor.hold;
    emitPose(log.final.stream,tp);emitPose(log.final.stream,full.pose);emitPose(log.final.stream,pool.raw[0].pose);
    if(decision.observation_valid)emitPose(log.final.stream,pool.raw[decision.selected].pose);else for(int n=0;n<16;++n)log.final.stream<<",NA";log.final.end();
    log.events.stream<<frame<<','<<ns<<','<<s.mask.active.size()<<','<<full_admitted<<','<<pool.raw[0].eligible<<','<<event.old_available<<','<<event.old_trigger<<','<<event.detected<<','<<event.permitted<<','<<event.reasonText()<<','<<event.permission<<','<<frame_extra<<','<<budget.used<<','<<distinct<<','<<eligible_raw<<','<<eligible_distinct<<','<<(selection.robust>0)<<','<<decision.recovery_success<<','<<decision.observation_valid<<','<<decision.status;log.events.end();
    log.runtime.stream<<frame<<','<<s.load_ms<<','<<s.preprocess_ms<<','<<prediction_ms<<','<<full.align_ms<<','<<candidates[0].align_ms<<','<<event_ms<<','<<coarse_ms<<','<<seed_ms<<','<<extra_ms<<','<<fitness_aux<<','<<coverage_ms<<','<<decision_ms<<','<<milliseconds(frame_start)<<','<<peakKiB();log.runtime.end();
    accepted+=decision.observation_valid;recoveries+=decision.recovery_success;
    std::cout<<"FRAME "<<frame<<" event="<<event.detected<<" extra="<<frame_extra<<" status="<<decision.status<<" full_admitted="<<full_admitted<<std::endl;
  }
  require(total==40+budget.used&&budget.used<=20,"final actual call accounting");
  std::cout<<"PILOT_COMPLETE frames=20 FULL=20 SPLIT=20 EXTRA="<<budget.used<<" total="<<total<<" events="<<events<<" proposed_observations="<<accepted<<" recoveries="<<recoveries<<" process_wall_ms="<<milliseconds(process_start)<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"PILOT_STOP "<<e.what()<<std::endl;return 1;}}
