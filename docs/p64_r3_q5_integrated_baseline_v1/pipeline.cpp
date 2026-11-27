#include <hdl_localization/pose_estimator.hpp>
#include <functional>
#include <cstdlib>
#include <numeric>
#include <pcl/kdtree/kdtree_flann.h>

#define main archived_q2_search_main
#include "search_reuse.hpp"
#undef main

struct Candidate {
  Pose start, end;
  bool boundary;
  Score score;
  int iterations;
};

struct Diagnostic {
  Eigen::Matrix<double,6,6> h = Eigen::Matrix<double,6,6>::Zero();
  Eigen::Matrix<double,6,1> eigen = Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,6> basis = Eigen::Matrix<double,6,6>::Identity();
};

Diagnostic local_information(InstrumentedScorer& ndt,const Cloud& source,const Pose& pose,
                             const std::vector<q1::PointScore>& details) {
  Diagnostic d;
  for (std::size_t i=0;i<source.size();++i) {
    const auto& cells=details[i].neighborhoods;
    if (cells.empty()) continue;
    const auto p=source[i].getVector3fMap().cast<double>().eval();
    Eigen::Matrix3d skew;
    skew<<0,-p.z(),p.y(),p.z(),0,-p.x(),-p.y(),p.x(),0;
    const Eigen::Matrix3d r=pose.topLeftCorner<3,3>().cast<double>();
    Eigen::Matrix<double,3,6> j;
    j<<r,-r*skew/5.;
    Eigen::Matrix3d w=Eigen::Matrix3d::Zero();
    for (const auto& term:cells) w+=term.score*ndt.cells().at(term.cell).getInverseCov();
    d.h.noalias()+=j.transpose()*w*j;
  }
  Eigen::SelfAdjointEigenSolver<Mat6> es(d.h);
  if(es.info()!=Eigen::Success) throw std::runtime_error("information eigen decomposition failed");
  d.eigen=es.eigenvalues();d.basis=es.eigenvectors();
  return d;
}

template<typename M>
void list(std::ostream& out,const Eigen::MatrixBase<M>& a) {
  for (Eigen::Index r=0;r<a.rows();++r) for(Eigen::Index c=0;c<a.cols();++c) {
    if(r||c)out<<';';
    out<<a(r,c);
  }
}
void pose_csv(std::ostream& out,const Pose& p) {
  for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<p(i,j);
}

std::string quote(const fs::path& path) {
  std::string s="'";
  for(char c:path.string())s+=(c=='\'' ? "'\\''" : std::string(1,c));
  return s+"'";
}

struct Geometry {
  std::vector<int> ids;
  int blocks=0,common=0,unknown=0,supported=0;
  std::string status="NOT_RUN";
};

Geometry geometry(const fs::path& out,const fs::path& helper,const fs::path& map,const Cloud& source,
                  const std::vector<Candidate>& candidates,double available) {
  Geometry result;
  if(available<=0){result.status="DEADLINE";return result;}
  fs::create_directory(out);
  pcl::io::savePCDFileBinary((out/"source.pcd").string(),source);
  {
    std::ofstream file(out/"poses.tsv");
    file<<std::setprecision(10);
    for(const auto& c:candidates) {
      for(int i=0;i<4;++i)for(int j=0;j<4;++j)file<<c.end(i,j)<<' ';
      file<<'\n';
    }
  }
  const double limit=std::min(4.,available);
  const std::string command="OPENBLAS_NUM_THREADS=1 timeout "+std::to_string(limit)+
    " /usr/bin/python3 "+quote(helper)+" "+quote(out)+" "+quote(map)+" > "+quote(out/"geometry.log")+" 2>&1";
  if(std::system(command.c_str())!=0) {result.status="MODEL_OR_TIMEOUT";return result;}
  std::ifstream in(out/"mask.tsv");
  in>>result.status>>result.common>>result.unknown>>result.blocks>>result.supported;
  int i;
  while(in>>i){
    need(i>=0 && static_cast<std::size_t>(i)<source.size(),"geometry returned invalid point id");
    result.ids.push_back(i);
  }
  need(std::is_sorted(result.ids.begin(),result.ids.end()) &&
       std::adjacent_find(result.ids.begin(),result.ids.end())==result.ids.end(),"duplicate geometry point id");
  return result;
}

int main(int argc,char** argv) {
  try {
    if(argc!=9) throw std::runtime_error("v1 INPUT MAP NEW_OUTPUT MODE FORCE_INDEX DEADLINE_S STYLE GEOMETRY_SCRIPT");
    const fs::path dir=argv[1],map=fs::absolute(argv[2]),out=fs::absolute(argv[3]),helper=fs::absolute(argv[8]);
    const std::string mode=argv[4],style=argv[7];
    const int force=std::stoi(argv[5]); const double limit=std::stod(argv[6]);
    need(mode=="A"||mode=="OFF"||mode=="B"||mode=="C"||mode=="SHADOW"||mode=="ACTIVE","bad mode");
    need(style=="S0"||style=="S3","bad search");
    need(fs::create_directory(out),"output already exists");
    std::ifstream anchor(dir/"anchor.tsv"),scans(dir/"scans.tsv"),imu(dir/"imu.tsv");
    Eigen::Vector3f seed;float yaw;
    need(bool(anchor>>seed.x()>>seed.y()>>seed.z()>>yaw)&&scans&&imu,"missing sensor/anchor");
    Eigen::Quaternionf orientation(Eigen::AngleAxisf(yaw/deg,Eigen::Vector3f::UnitZ()));
    float qx,qy,qz,qw;
    if(anchor>>qx>>qy>>qz>>qw) orientation=Eigen::Quaternionf(qw,qx,qy,qz).normalized();
    auto target=downsample(load(map));
    auto* ndt=new InstrumentedScorer(target);
    pcl::Registration<Point,Point>::Ptr registration(ndt);
    hdl_localization::PoseEstimator tracker(registration,seed,
       orientation,2.0);
    InstrumentedScorer searcher(target);
    auto poses=file(out/"trajectory.csv","frame,stamp_ns,latest_imu_ns,imu_count,iterations,converged,probability,source,supported,updates,last_update_ns,native_reward,normal_s,total_s"+
                      std::string(",")+[](){std::string s;for(auto k:{"pred","ndt","state"})for(int i=0;i<4;++i)for(int j=0;j<4;++j)s+=std::string(s.empty()?"":",")+k+std::to_string(i)+std::to_string(j);return s;}());
    auto diagnostics=file(out/"diagnostics.csv","frame,stamp_ns,weak,local_weak_rank,risk,spectrum,weak_subspace,proxy_matrix,native_hessian,covariance_before,covariance_after");
    auto actions=file(out/"decisions.csv","frame,stamp_ns,trigger,decision,actual,geometry_state,difference_support,common,unknown,blocks,co_support,score_calls,candidate_fits,refinement,search_s,view_s,refine_s,decision_s");
    auto candidates=file(out/"candidates.csv","frame,stamp_ns,candidate,source,boundary,iterations,support,full_native,max_native,mean_native,park,shared_native,n_points,half0,half1"+std::string(",start_matrix,end_matrix"));
    auto refining=file(out/"refinements.csv","frame,stamp_ns,selected,reported_converged,iterations,moved_m,moved_deg,mode_escape,prior_reward,after_reward,reference_free_R0,reference_free_R1,reference_free_R2");
    auto peak_meta=file(out/"peaks.csv","sample_id,method,metric,policy,index,ranking,retained,boundary,mode,reason,relative");
    std::uint64_t ns=0,imu_ns=0,last_imu=0,last_scan=0;
    Eigen::Vector3f acc,gyro;
    auto read_imu=[&](){return bool(imu>>imu_ns>>acc.x()>>acc.y()>>acc.z()>>gyro.x()>>gyro.y()>>gyro.z());};
    bool has_imu=read_imu();int frame=0,searches=0;
    std::string scan_file;
    while(scans>>ns>>scan_file) {
      const auto total_begin=Clock::now();
      need(frame<300 && ns>last_scan,"scan time order or frame budget");
      last_scan=ns; int imu_n=0;
      while(has_imu&&imu_ns<=ns) {
        need(imu_ns>last_imu,"IMU time order");
        ros::Time t; t.fromNSec(imu_ns);
        tracker.predict(t,acc,gyro); last_imu=imu_ns; ++imu_n;has_imu=read_imu();
      }
      const auto scan=downsample(load(dir/scan_file)); ros::Time stamp;stamp.fromNSec(ns);
      Pose pred=tracker.matrix(),base=pred;
      int iter=0,co=0; double probability=0,normal_s=0;
      bool converged=false,weak=false,risk=false;std::string trigger="NONE",proposal="NATIVE",actual="NATIVE";
      const auto cov_before=tracker.covariance();
      std::vector<q1::PointScore> native_points;
      Score base_score;Mat6 native_h=Mat6::Zero();Diagnostic info;
      Geometry geo;
      Timings timings;double search_s=0,view_s=0,refine_s=0,decision_s=0;
      std::string refinement="NOT_RUN";
      const auto before=tracker.map_updates;
      const auto ndt_begin=Clock::now();
      if(mode!="A") tracker.decision_hook=[&](const Pose& initial,Pose& update) {
        normal_s=elapsed(ndt_begin); const auto begun=Clock::now();
        const auto within=[&](){return elapsed(begun)<limit;};
        pred=initial;base=update;
        iter=ndt->getFinalNumIteration();converged=ndt->hasConverged();probability=ndt->getTransformationProbability();
        if(!base.allFinite()) {proposal="PROPAGATE";actual="PROPAGATE";return false;}
        base_score=ndt->fixed(*scan,base,&native_points);
        ndt->oracle(scan,base,1,&native_h);
        if(mode!="OFF") {
          info=local_information(*ndt,*scan,base,native_points);
          weak=info.eigen[0]<.01*std::max(1e-12,info.eigen[5]);
          const auto support=double(base_score.associated)/scan->size();
          risk=!converged||iter<=0||support<.75||distance(pred,base)>.20||angle(pred,base)>4;
          const bool fallback=converged&&iter>0&&support>=.75&&distance(pred,base)<=.35&&angle(pred,base)<=6;
          proposal=fallback?"NATIVE":"PROPAGATE";
          if(frame==force)trigger="FORCED";
          else if(risk)trigger="NATIVE_RISK";
          else if(weak)trigger="LOCAL_WEAK";
          if(trigger!="NONE"&&mode!="B"&&searches<3){
            ++searches;const auto tick=Clock::now();
            try {
              Grid grid(searcher,*scan,timings);grid.budget=within;
              Frame f{std::to_string(frame),map,dir/scan_file,pred,base};
              if(style=="S3")makeS3(grid,f);
              else grid.lattice(base,"ndt","base",10,5,.2,1.,false);
              auto sel=extract(grid.trials,f.id,style,peak_meta,timings,{});
              search_s=elapsed(tick);
              std::vector<Candidate> pool;
              if(converged&&iter>0&&base_score.associated*2>=scan->size())pool.push_back({pred,base,false,base_score,iter});
              std::vector<int> extras;
              for(int metric:{0,1}) for(int k:diverse(peaks(grid.trials,metric),grid.trials,3)) {
                if(std::find(extras.begin(),extras.end(),k)==extras.end())extras.push_back(k);
              }
              searcher.setInputSource(scan);
              for(int k:extras) {
                if(timings.alignCalls>=2||!within())break;
                const Trial& trial=grid.trials[k];
                if(!pool.empty()&&close(trial.pose,base))continue;
                Cloud matched; searcher.align(matched,trial.pose);++timings.alignCalls;
                const Pose end=searcher.getFinalTransformation();
                if(!searcher.hasConverged()||searcher.getFinalNumIteration()<=0||!end.allFinite())continue;
                const auto score=searcher.fixed(*scan,end);
                if(score.associated*2<scan->size())continue;
                if(std::any_of(pool.begin(),pool.end(),[&](const Candidate& c){return close(c.end,end);}))continue;
                pool.push_back({trial.pose,end,trial.boundary,score,searcher.getFinalNumIteration()});
              }
              if(pool.empty()) {
                geo.status="NO_VALID_MODE";
                throw std::runtime_error("no valid mode");
              }
              if(pool.size()<2)geo.status="NO_INDEPENDENT_CANDIDATE";
              else if(mode!="C"){
                const auto gv=Clock::now();
                geo=geometry(out/("event_"+std::to_string(frame)),helper,map,*scan,pool,limit-elapsed(begun));
                view_s=elapsed(gv);
              }
              std::vector<double> total,first,second;
              for(std::size_t k=0;k<pool.size();++k){
                std::vector<PointScore> data;searcher.fixed(*scan,pool[k].end,&data);
                double max_sum=0,mean_sum=0,sub=0,ha=0,hb=0;
                for(const auto& p:data){
                  double m=0;for(const auto& v:p.neighborhoods)m=std::max(m,v.score);
                  max_sum+=m;mean_sum+=p.total/std::max<std::size_t>(1,p.neighborhoods.size());
                }
                for(int i:geo.ids){
                  sub+=data[i].total;
                  ((*scan)[i].y<0?ha:hb)+=data[i].total;
                }
                total.push_back(sub);first.push_back(ha);second.push_back(hb);
                candidates<<frame<<','<<ns<<','<<k<<','<<(k?"GRID_FIT":"NATIVE")<<','<<pool[k].boundary<<','<<pool[k].iterations<<','<<pool[k].score.associated
                   <<','<<pool[k].score.native<<','<<max_sum<<','<<mean_sum<<','<<pool[k].score.park<<','<<sub<<','<<geo.ids.size()<<','<<ha<<','<<hb<<',';
                list(candidates,pool[k].start);candidates<<',';list(candidates,pool[k].end);candidates<<'\n';
              }
              const auto winner=std::max_element(total.begin(),total.end())-total.begin();
              const auto full=std::max_element(pool.begin(),pool.end(),[](const auto& a,const auto& b){return a.score.native<b.score.native;})-pool.begin();
              auto unique_lead=[&](const std::vector<double>& v){
                for(std::size_t j=0;j<v.size();++j)if(j!=static_cast<std::size_t>(winner)&&v[winner]<=v[j]+1e-9)return false;
                return true;
              };
              const bool reliable=pool.size()>1&&geo.ids.size()>=64&&geo.blocks>=5&&
                 winner==full&&unique_lead(total)&&unique_lead(first)&&unique_lead(second)&&!pool[winner].boundary;
              if(reliable&&within()) {
                const auto begin_ref=Clock::now();
                Cloud refined;searcher.align(refined,pool[winner].end);
                const Pose terminal=searcher.getFinalTransformation();
                const auto moved=distance(terminal,pool[winner].end),ang=angle(terminal,pool[winner].end);
                bool escape=moved>.25||ang>2;
                for(std::size_t j=0;j<pool.size();++j)if(j!=static_cast<std::size_t>(winner)&&close(terminal,pool[j].end))escape=true;
                const auto rr=searcher.fixed(*scan,terminal);
                const bool accepted=within()&&terminal.allFinite()&&searcher.hasConverged()&&searcher.getFinalNumIteration()>0&&
                    rr.associated*2>=scan->size()&&!escape;
                refinement=accepted?"PASSED":"REJECTED";
                refining<<frame<<','<<ns<<','<<winner<<','<<searcher.hasConverged()<<','<<searcher.getFinalNumIteration()<<','
                        <<moved<<','<<ang<<','<<escape<<','<<pool[winner].score.native<<','<<rr.native<<',';
                list(refining,pool[winner].end);refining<<',';list(refining,terminal);refining<<',';
                list(refining,accepted?terminal:pool[winner].end);refining<<'\n';
                if(within()){
                  proposal=accepted?"REFINED":"CANDIDATE";
                  if(mode=="ACTIVE") update=accepted?terminal:pool[winner].end;
                }
                refine_s=elapsed(begin_ref);
              } else if(geo.status=="OK")geo.status=geo.ids.size()<64?"LOW_SUPPORT":"REVIEW_DISAGREEMENT";
            } catch(const std::runtime_error& e) {
              search_s=elapsed(tick);
              const std::string reason=e.what();
              if(reason=="search deadline")geo.status="SEARCH_TIMEOUT";
              else if(reason!="no valid mode")throw;
            }
          }
          if(mode=="ACTIVE")actual=proposal;
        }
        decision_s=elapsed(begun);
        return actual!="PROPAGATE";
      };
      tracker.correct(stamp,scan);
      if(mode=="A"){
        normal_s=elapsed(ndt_begin);
        base=ndt->getFinalTransformation();iter=ndt->getFinalNumIteration();converged=ndt->hasConverged();
        probability=ndt->getTransformationProbability();base_score=ndt->fixed(*scan,base);
        ndt->oracle(scan,base,1,&native_h);
      }
      const auto updates=tracker.map_updates-before;
      need(updates<=1,"duplicate physical UKF correction");
      need((updates==0)==(actual=="PROPAGATE"),"UKF commit disagrees with policy");
      const Pose post=tracker.matrix(); const auto cov_after=tracker.covariance();
      poses<<frame<<','<<ns<<','<<last_imu<<','<<imu_n<<','<<iter<<','<<converged<<','<<probability<<','<<scan->size()<<','<<base_score.associated
           <<','<<updates<<','<<tracker.last_correction_time().toNSec()<<','<<base_score.native<<','<<normal_s<<','<<elapsed(total_begin);
      pose_csv(poses,pred);pose_csv(poses,base);pose_csv(poses,post);poses<<'\n';
      const auto count=(info.eigen.array()<.01*std::max(1e-12,info.eigen[5])).count();
      diagnostics<<frame<<','<<ns<<','<<weak<<','<<(mode=="A"||mode=="OFF"?0:count)<<','<<risk<<',';
      list(diagnostics,info.eigen);diagnostics<<',';list(diagnostics,info.basis.leftCols(mode=="A"||mode=="OFF"?0:count));
      diagnostics<<',';list(diagnostics,info.h);diagnostics<<',';list(diagnostics,native_h);diagnostics<<',';
      list(diagnostics,cov_before);diagnostics<<',';list(diagnostics,cov_after);diagnostics<<'\n';
      actions<<frame<<','<<ns<<','<<trigger<<','<<proposal<<','<<actual<<','<<geo.status<<','<<geo.ids.size()<<','<<geo.common
         <<','<<geo.unknown<<','<<geo.blocks<<','<<geo.supported<<','<<timings.scoreCalls<<','<<timings.alignCalls<<','<<refinement<<','<<search_s
         <<','<<view_s<<','<<refine_s<<','<<decision_s<<'\n';
      ++frame;
    }
    need(frame>=1&&!has_imu&&imu.eof()&&scans.eof(),"incomplete causal window");
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
