#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include "../../p62_r17_consensus_admission/tools/policy.hpp"
#include <Eigen/Eigenvalues>
#include <iostream>

using namespace pilot;

static Eigen::Vector3d horizontalAxis(const Cloud::Ptr& scan){
  require(scan->size()>=300,"anchor source insufficient for horizontal PCA");
  Eigen::Vector2d mean=Eigen::Vector2d::Zero();
  for(const auto& p:*scan)mean+=Eigen::Vector2d(p.x,p.y);
  mean/=double(scan->size());Eigen::Matrix2d cov=Eigen::Matrix2d::Zero();
  for(const auto& p:*scan){const Eigen::Vector2d d=Eigen::Vector2d(p.x,p.y)-mean;cov.noalias()+=d*d.transpose();}
  cov/=double(scan->size()-1);Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);
  require(eig.info()==Eigen::Success&&eig.eigenvalues().allFinite()&&eig.eigenvalues()[1]>eig.eigenvalues()[0]+1e-8,"anchor horizontal axis ambiguous");
  Eigen::Vector2d v=eig.eigenvectors().col(1);
  if(v.x()<0||(std::abs(v.x())<1e-12&&v.y()<0))v=-v;
  return Eigen::Vector3d(v.x(),v.y(),0);
}

static void emitMatrix(std::ostream& out,const Pose& pose){
  for(int i=0;i<4;++i)for(int j=0;j<4;++j){out<<',';number(out,pose(i,j));}
}

int main(int argc,char** argv){try{
  require(argc==7,"run_basin ROLE SCANS ANCHOR MAP_PCD MAP_BUILD_CSV NEW_RUN_DIRECTORY");
  require(::mkdir(argv[6],0775)==0,"exclusive native run directory; no overwrite");
  const std::string role=argv[1],out=argv[6];require(role=="WEAK"||role=="STRONG","invalid frozen window role");
  const auto rows=readCsv(argv[2]);require(rows.size()==61,"exactly 61 frozen scans required");
  const auto map_rows=readCsv(argv[5]);require(map_rows.size()==1,"single frozen map build receipt required");
  const auto total_start=Clock::now(),map_start=Clock::now();
  Cloud::Ptr map(new Cloud);require(pcl::io::loadPCDFile(argv[4],*map)==0,"frozen map unreadable");
  finish(map);require(map->size()==std::stoul(map_rows[0].at("target_points"))&&cloudHash(map)==map_rows[0].at("target_fnv"),"frozen target identity changed");
  NativeNdt ndt;ndt.configure(map);const double map_ms=milliseconds(map_start);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;tree.setInputCloud(map);
  const Pose base=readMatrix(argv[3]);
  Source anchor_source=pilot::loadSource(rows[0]);
  require(anchor_source.points->size()==std::stoul(rows[0].at("expected_source_points"))&&cloudHash(anchor_source.points)==rows[0].at("expected_source_fnv"),"anchor source mismatch");
  const Eigen::Vector3d axis_l=horizontalAxis(anchor_source.points);
  const Eigen::Vector3d axis_m=base.block<3,3>(0,0)*axis_l;
  require(std::abs(axis_m.norm()-1.)<1e-8&&std::abs(axis_l.z())<1e-12,"horizontal axis invalid");
  Csv axes(out+"/INITIALIZATION_AXES.csv","role,axis_l_x,axis_l_y,axis_l_z,axis_m_x,axis_m_y,axis_m_z,anchor_raw_ordinal");
  axes.stream<<role<<','<<axis_l.x()<<','<<axis_l.y()<<','<<axis_l.z()<<','<<axis_m.x()<<','<<axis_m.y()<<','<<axis_m.z()<<','<<rows[0].at("raw_ordinal");axes.end();
  std::string header="role,arm,frame,raw_ordinal,stamp_ns,source_fnv,prediction_mode,history_last_stamp_ns,raw_finite,converged,iterations,native_score,fitness,overlap_0p5,coverage_valid,active_blocks,heldout_mean,quality_feedback,admission_shadow_5deg,translation_from_prediction_m,rotation_from_prediction_deg,feedback_action,align_ms,validation_ms,geometry_ms,G_valid,G_supported_points,formal_weak_count";
  for(int i=0;i<6;++i)header+=",lambda"+std::to_string(i)+",ratio"+std::to_string(i);
  poseHeader(header,"tp");poseHeader(header,"raw");poseHeader(header,"output");
  Csv native(out+"/NDT_RESULTS.csv",header);
  Csv runtime(out+"/RUNTIME_AND_MEMORY.csv","role,arm,frame,raw_ordinal,pcd_load_ms,preprocess_ms,prediction_ms,full_align_ms,aux_fitness_ms,validation_ms,geometry_ms,frame_wall_ms,map_init_ms,peak_rss_KiB");
  Csv calls(out+"/ALIGN_CALLS.csv","role,arm,frame,raw_ordinal,kind,align_ms,converged,iterations");
  runtime.stream<<role<<",MAP_INIT,0,"<<rows[0].at("raw_ordinal")<<",0,0,0,0,0,0,0,0,"<<map_ms<<','<<peakKiB();runtime.end();
  int aligns=0,quality_count=0,shadow_reject=0;
  const int64_t anchor_ns=std::stoll(rows[0].at("stamp_ns"));
  const std::array<std::string,4> names{{"C0","C1","C2","C3"}};
  for(int arm=0;arm<4;++arm){
    Pose initial=base;
    if(arm==1||arm==2)initial.block<3,1>(0,3)+=(arm==1?.8:-.8)*axis_m;
    if(arm==3)initial.block<3,3>(0,0)=Eigen::AngleAxisd(8*PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix()*base.block<3,3>(0,0);
    require(rigid(initial),"nonrigid disturbed anchor");
    if(arm==1||arm==2)require(std::abs(translation(initial,base)-.8)<1e-8,"translation disturbance mismatch");
    if(arm==3)require(std::abs(rotation(initial,base)-8)<1e-8,"yaw disturbance mismatch");
    Predictor predictor(anchor_ns,initial);
    for(int frame=1;frame<=60;++frame){
      const auto begin=Clock::now();const auto& row=rows[frame];
      require(std::stoi(row.at("local_index"))==frame,"frozen local index mismatch");
      const int64_t ns=std::stoll(row.at("stamp_ns"));require(ns>std::stoll(rows[frame-1].at("stamp_ns")),"nonmonotonic scan time");
      const Source source=pilot::loadSource(row);
      require(source.points->size()==std::stoul(row.at("expected_source_points"))&&cloudHash(source.points)==row.at("expected_source_fnv"),"frozen source changed");
      const std::string mode=predictor.hold?"HOLD_LAST":(predictor.history.size()<2?"LAST_POSE":"CONSTANT_VELOCITY");
      const int64_t from_ns=predictor.history.back().ns;
      const auto pred_start=Clock::now();const Pose tp=predictor.predict(ns);const double pred_ms=milliseconds(pred_start);
      const auto align_start=Clock::now();++aligns;
      Candidate c=capturedAlign(ndt,source.points,tp,0,[](const Candidate&){});
      const double aux_ms=std::max(0.,milliseconds(align_start)-c.align_ms);
      calls.stream<<role<<','<<names[arm]<<','<<frame<<','<<row.at("raw_ordinal")<<",FULL,"<<c.align_ms<<','<<c.converged<<','<<c.iterations;calls.end();
      if(c.finite)coverage::validate(c,source.split,source.mask,tree,tp);
      const bool quality=r17::quality(c,source.mask);
      const bool admission=r17::base(c,source.mask,tp,5.);
      if(quality)++quality_count;if(quality&&!admission)++shadow_reject;
      const Pose output=quality?c.pose:tp;
      size_t supported=0;int weak=-1;bool g_valid=false;double geometry_ms=0;
      Eigen::Matrix<double,6,1> eigvals=Eigen::Matrix<double,6,1>::Constant(NAN);
      const auto g_start=Clock::now();
      if(c.finite&&rigid(c.pose)){
        const Matrix6 g=ndt.geometry(source.split.training,c.pose,supported);
        if(g.allFinite()){
          Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
          if(eig.info()==Eigen::Success&&eig.eigenvalues().allFinite()&&eig.eigenvalues()[5]>1e-12&&eig.eigenvalues()[0]>=-1e-8*eig.eigenvalues()[5]){
            eigvals=eig.eigenvalues();weak=int((eigvals.array()/eigvals[5]<=.01).count());g_valid=true;}
        }
      }
      geometry_ms=milliseconds(g_start);
      native.stream<<role<<','<<names[arm]<<','<<frame<<','<<row.at("raw_ordinal")<<','<<ns<<','<<row.at("expected_source_fnv")<<','<<mode<<','<<from_ns<<','<<c.finite<<','<<c.converged<<','<<c.iterations<<',';
      number(native.stream,c.score);native.stream<<',';number(native.stream,c.fitness);native.stream<<',';number(native.stream,c.overlap);
      native.stream<<','<<source.mask.evaluable()<<','<<source.mask.active.size()<<',';
      number(native.stream,c.loss_valid&&source.mask.evaluable()?coverage::mean(c.loss,source.mask):NAN);
      native.stream<<','<<quality<<','<<admission<<',';
      number(native.stream,rigid(c.pose)?translation(c.pose,tp):NAN);native.stream<<',';
      number(native.stream,rigid(c.pose)?rotation(c.pose,tp):NAN);
      native.stream<<','<<(quality?"ACCEPT_RAW":"PREDICTION_FALLBACK")<<','<<c.align_ms<<','<<c.validation_ms<<','<<geometry_ms<<','<<g_valid<<','<<supported<<','<<weak;
      for(int i=0;i<6;++i){native.stream<<',';number(native.stream,eigvals[i]);native.stream<<',';number(native.stream,g_valid?eigvals[i]/eigvals[5]:NAN);}
      emitMatrix(native.stream,tp);emitMatrix(native.stream,c.pose);emitMatrix(native.stream,output);native.end();
      if(quality){predictor.history.push_back({ns,c.pose});if(predictor.history.size()>2)predictor.history.erase(predictor.history.begin());predictor.hold=false;}
      else predictor.hold=true;
      runtime.stream<<role<<','<<names[arm]<<','<<frame<<','<<row.at("raw_ordinal")<<','<<source.load_ms<<','<<source.preprocess_ms<<','<<pred_ms<<','<<c.align_ms<<','<<aux_ms<<','<<c.validation_ms<<','<<geometry_ms<<','<<milliseconds(begin)<<",0,"<<peakKiB();runtime.end();
    }
    std::cout<<"ARM_COMPLETE "<<role<<' '<<names[arm]<<" aligns="<<aligns<<" rss_kib="<<peakKiB()<<std::endl;
  }
  require(aligns==240,"native Full align count mismatch");
  std::cout<<"R19_NATIVE_COMPLETE "<<role<<" aligns="<<aligns<<" quality="<<quality_count<<" shadow_reject="<<shadow_reject<<" wall_ms="<<milliseconds(total_start)<<" peak_rss_KiB="<<peakKiB()<<std::endl;
}catch(const std::exception& e){std::cerr<<"R19_NATIVE_STOP "<<e.what()<<std::endl;return 1;}}
