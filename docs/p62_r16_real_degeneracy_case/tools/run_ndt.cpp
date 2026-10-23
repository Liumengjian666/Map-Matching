#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/native_capture.hpp"
#include "../../p62_r10_feasibility_event_recovery/tools/events.hpp"
#include "../../p62_r9_coverage_aware_recovery_v2/tools/direction.hpp"
#include <iostream>

using namespace pilot;

static void emitPoseCsv(std::ostream& out,const Pose& pose){
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<pose(i,j);
}
static void emitLoss(std::ostream& out,const Candidate& c,const Mask& mask,bool median){
  if(!c.loss_valid||!mask.evaluable()){out<<",NA";return;}
  out<<','<<(median?coverage::median(c.loss,mask):coverage::mean(c.loss,mask));
}

int main(int argc,char** argv){try{
  require(argc==6,"run_ndt SCANS ANCHOR MAP_PCD MAP_BUILD_CSV NEW_RUN_DIRECTORY");
  require(::mkdir(argv[5],0775)==0,"exclusive R16 native run directory; no rerun/overwrite");
  const auto process_start=Clock::now();const std::string out=argv[5];
  const auto rows=readCsv(argv[1]);require(rows.size()==31,"fixed 31-frame window required");
  const auto map_rows=readCsv(argv[4]);require(map_rows.size()==1,"frozen map build receipt required");
  const auto map_start=Clock::now();Cloud::Ptr target(new Cloud);
  require(pcl::io::loadPCDFile(argv[3],*target)==0,"frozen map PCD unreadable");
  for(const auto& p:*target)require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"nonfinite frozen map point");
  finish(target);require(!target->empty(),"empty fixed map");
  require(target->size()==std::stoul(map_rows[0].at("target_points"))&&cloudHash(target)==map_rows[0].at("target_fnv"),"frozen map identity mismatch");
  NativeNdt ndt;ndt.configure(target);const double map_ms=milliseconds(map_start);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(target);

  Csv native(out+"/NDT_RESULTS.csv",
    "frame,raw_ordinal,stamp_ns,method,finite,converged,iterations,native_score,fitness,align_ms,auxiliary_fitness_ms,validation_ms,coverage_valid,active_blocks,overlap_0p5,heldout_mean,heldout_median,admissible,translation_from_TP_m,rotation_from_TP_deg,pose00,pose01,pose02,pose03,pose10,pose11,pose12,pose13,pose20,pose21,pose22,pose23,pose30,pose31,pose32,pose33");
  Csv geometry(out+"/GEOMETRY_AND_DEGENERACY.csv",
    "frame,raw_ordinal,stamp_ns,center_method,G_valid,supported_training_points,formal_weak_count,lambda_max,condition_number,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,ratio0,ratio1,ratio2,ratio3,ratio4,ratio5,u00,u10,u20,u30,u40,u50,u01,u11,u21,u31,u41,u51,u02,u12,u22,u32,u42,u52,u03,u13,u23,u33,u43,u53,u04,u14,u24,u34,u44,u54,u05,u15,u25,u35,u45,u55");
  Csv relations(out+"/PREDICTION_DISAGREEMENT.csv",
    "frame,raw_ordinal,stamp_ns,method,translation_from_prediction_m,rotation_from_prediction_deg,admission_rejected,converged,iterations,native_score,fitness,tp00,tp01,tp02,tp03,tp10,tp11,tp12,tp13,tp20,tp21,tp22,tp23,tp30,tp31,tp32,tp33");
  Csv events(out+"/EVENTS.csv",
    "frame,raw_ordinal,stamp_ns,method,old_trigger_available,old_trigger,detected,reasons,recovery_permission,admissible,formal_weak_count");
  Csv runtime(out+"/RUNTIME_AND_MEMORY.csv",
    "frame,pcd_load_ms,source_preprocess_ms,prediction_ms,full_ndt_ms,full_aux_ms,split_ndt_ms,split_aux_ms,validation_ms,geometry_ms,event_ms,frame_wall_ms,map_gaussian_grid_init_ms,peak_rss_KiB");
  Csv calls(out+"/ALIGN_CALLS.csv","frame,method,align_index,status,align_ms,converged,iterations");
  runtime.stream<<"0,0,0,0,0,0,0,0,0,0,0,0,"<<map_ms<<",0,"<<peakKiB();runtime.end();

  const Pose anchor=readMatrix(argv[2]);Predictor predictor(std::stoll(rows[0].at("stamp_ns")),anchor);
  size_t total_align=0;int events_count=0,formal_weak_frames=0;
  for(int frame=1;frame<=30;++frame){
    const auto frame_start=Clock::now();const auto& row=rows[frame];
    const int64_t ns=std::stoll(row.at("stamp_ns"));
    require(std::stoi(row.at("local_index"))==frame&&ns>std::stoll(rows[frame-1].at("stamp_ns")),"frozen scan order/timestamp contract");
    const Source source=pilot::loadSource(row);
    require(source.points->size()==std::stoul(row.at("expected_source_points"))&&
            cloudHash(source.points)==row.at("expected_source_fnv"),"frozen Source changed");
    require(source.split.training->size()>=300&&source.mask.evaluable(),"fixed frame no longer satisfies Training/Coverage V2 input contract");
    const auto pred_start=Clock::now();const Pose tp=predictor.predict(ns);const double pred_ms=milliseconds(pred_start);
    auto run=[&](const char* method,const Cloud::Ptr& cloud,int ordinal){
      const auto phase=Clock::now();++total_align;
      Candidate c=capturedAlign(ndt,cloud,tp,ordinal,[](const Candidate&){});
      const double aux_ms=std::max(0.,milliseconds(phase)-c.align_ms);
      calls.stream<<frame<<','<<method<<','<<ordinal<<",RETURNED,"<<c.align_ms<<','<<c.converged<<','<<c.iterations;calls.end();
      if(c.finite)coverage::validate(c,source.split,source.mask,tree,tp);
      return std::pair<Candidate,double>(c,aux_ms);
    };
    const auto full_pair=run("CONTROL_FULL",source.points,0);
    const auto split_pair=run("CONTROL_SPLIT",source.split.training,1);
    Candidate full=full_pair.first,split=split_pair.first;

    auto write_candidate=[&](const char* method,const Candidate& c,double aux){
      native.stream<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<method<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';
      number(native.stream,c.score);native.stream<<',';number(native.stream,c.fitness);native.stream<<','<<c.align_ms<<','<<aux<<','<<c.validation_ms<<','<<c.loss_valid<<','<<source.mask.active.size()<<',';
      number(native.stream,c.overlap);emitLoss(native.stream,c,source.mask,false);emitLoss(native.stream,c,source.mask,true);
      const bool admitted=admissible(c,source.mask,tp);native.stream<<','<<admitted<<',';
      number(native.stream,rigid(c.pose)?translation(c.pose,tp):NAN);native.stream<<',';
      number(native.stream,rigid(c.pose)?rotation(c.pose,tp):NAN);emitPoseCsv(native.stream,c.pose);native.end();
      relations.stream<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<method<<',';
      number(relations.stream,rigid(c.pose)?translation(c.pose,tp):NAN);relations.stream<<',';
      number(relations.stream,rigid(c.pose)?rotation(c.pose,tp):NAN);relations.stream<<','<<(!admitted)<<','<<c.converged<<','<<c.iterations<<',';
      number(relations.stream,c.score);relations.stream<<',';number(relations.stream,c.fitness);
      emitPoseCsv(relations.stream,tp);relations.end();
    };
    write_candidate("CONTROL_FULL",full,full_pair.second);
    write_candidate("CONTROL_SPLIT",split,split_pair.second);

    size_t supported=0;double geometry_ms=0;int weak_count=-1;bool g_valid=false;
    Eigen::Matrix<double,6,1> values=Eigen::Matrix<double,6,1>::Constant(NAN);
    Eigen::Matrix<double,6,6> vectors=Eigen::Matrix<double,6,6>::Constant(NAN);
    double cond=NAN,lambda_max=NAN;
    const auto geometry_start=Clock::now();
    if(split.finite&&rigid(split.pose)){
      const Matrix6 g=ndt.geometry(source.split.training,split.pose,supported);
      if(g.allFinite()){
        Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
        if(eig.info()==Eigen::Success&&eig.eigenvalues().allFinite()&&eig.eigenvectors().allFinite()){
          values=eig.eigenvalues();vectors=eig.eigenvectors();lambda_max=values[5];
          if(lambda_max>1e-12&&values[0]>=-1e-8*lambda_max){
            g_valid=true;weak_count=int((values.array()/lambda_max<=.01).count());
            const double lambda_min=std::max(values[0],1e-15*lambda_max);cond=lambda_max/lambda_min;
          }
        }
      }
    }
    geometry_ms=milliseconds(geometry_start);
    if(g_valid&&weak_count>0)++formal_weak_frames;
    geometry.stream<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<",CONTROL_SPLIT,"<<g_valid<<','<<supported<<','<<weak_count<<',';
    number(geometry.stream,lambda_max);geometry.stream<<',';number(geometry.stream,cond);
    for(int i=0;i<6;++i){geometry.stream<<',';number(geometry.stream,values[i]);}
    for(int i=0;i<6;++i){geometry.stream<<',';number(geometry.stream,g_valid?values[i]/lambda_max:NAN);}
    for(int col=0;col<6;++col)for(int row_i=0;row_i<6;++row_i){geometry.stream<<',';number(geometry.stream,vectors(row_i,col));}
    geometry.end();

    const auto event_start=Clock::now();const auto event=feasibility::detect(split,source.mask,tp,0);const double event_ms=milliseconds(event_start);
    events_count+=event.detected;
    events.stream<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<",CONTROL_SPLIT,"<<event.old_available<<','<<event.old_trigger<<','<<event.detected<<','<<event.reasonText()<<','<<event.permission<<','<<admissible(split,source.mask,tp)<<','<<weak_count;events.end();

    predictor.completed(ns,full,source.mask,tp);
    const double validation_ms=full.validation_ms+split.validation_ms;
    runtime.stream<<frame<<','<<source.load_ms<<','<<source.preprocess_ms<<','<<pred_ms<<','<<full.align_ms<<','<<full_pair.second<<','
      <<split.align_ms<<','<<split_pair.second<<','<<validation_ms<<','<<geometry_ms<<','<<event_ms<<','<<milliseconds(frame_start)
      <<",0,"<<peakKiB();runtime.end();
    std::cout<<"FRAME "<<frame<<" ordinal="<<row.at("raw_ordinal")<<" full_conv="<<full.converged
      <<" split_conv="<<split.converged<<" split_admitted="<<admissible(split,source.mask,tp)
      <<" event="<<event.detected<<" weak="<<weak_count<<std::endl;
  }
  require(total_align==60,"expected exactly 30 Full and 30 Split native align calls");
  std::cout<<"R16_NATIVE_COMPLETE frames=30 FULL=30 SPLIT=30 EXTRA=0 events="<<events_count
    <<" formal_weak_frames="<<formal_weak_frames<<" process_wall_ms="<<milliseconds(process_start)
    <<" map_grid_init_ms="<<map_ms<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"R16_NDT_STOP "<<e.what()<<std::endl;return 1;}}
