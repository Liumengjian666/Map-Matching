#include "causal_rotation.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>

namespace b = corridor_benchmark;
namespace p = dog_prior_map_fastlio2_frontend_exp;
using Matrix = Eigen::Matrix4d;
using Clock = std::chrono::steady_clock;
double milliseconds(Clock::time_point t) {
  return std::chrono::duration<double,std::milli>(Clock::now()-t).count();
}
Matrix matrix(const p::Pose3d& pose) { return p::asIsometry(pose).matrix(); }
p::Pose3d pose(const Matrix& T) {
  p::Pose3d out;
  out.position=T.block<3,1>(0,3);
  out.orientation=Eigen::Quaterniond(T.block<3,3>(0,0)).normalized();
  return out;
}
double angle(const Matrix& a, const Matrix& c) {
  return Eigen::Quaterniond(a.block<3,3>(0,0)).normalized().angularDistance(
      Eigen::Quaterniond(c.block<3,3>(0,0)).normalized())*180/std::acos(-1.0);
}
bool rigid(const Matrix& T) {
  const Eigen::Matrix3d R=T.block<3,3>(0,0);
  return T.allFinite() && (T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-6 &&
      (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-4 && std::abs(R.determinant()-1)<1e-4;
}
void writePose(std::ostream& file, const Matrix& T) {
  for(int r=0;r<3;++r) for(int c=0;c<4;++c) file<<','<<T(r,c);
}
struct Arm {
  std::string name;
  Matrix current=Matrix::Identity(), accepted=Matrix::Identity();
  Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
  uint64_t stamp=0, accepted_stamp=0;
  p::CoupledAnchorState anchor;
  unsigned success=0, feedback=0, failures=0;
};
void selfTest() {
  p::P7ImuVector imu;
  for(uint64_t t=1000000000;t<=1100000000;t+=5000000) {
    p::ImuSample s;s.stamp_ns=t;s.angular_velocity=Eigen::Vector3d(.1,-.2,.3);imu.push_back(s);
  }
  const b::RotationTimeline timeline(imu,1000000000,1099000000);
  b::require((timeline.rotations.back()-b::exp(imu.front().angular_velocity*.099)).norm()<1e-12,"integration_test");
  imu.back().angular_velocity.setConstant(10000);
  const b::RotationTimeline tampered(imu,1000000000,1099000000);
  b::require((timeline.rotations.back()-tampered.rotations.back()).norm()==0,"future_IMU_test");
  Matrix E=Matrix::Identity();E.block<3,3>(0,0)=b::exp(Eigen::Vector3d(.2,-.1,.3));
  E.block<3,1>(0,3)=Eigen::Vector3d(.3,-.2,.1);
  Matrix D=Matrix::Identity();D.block<3,3>(0,0)=timeline.rotations.back().transpose();
  const Eigen::Vector3d point(1,2,3);
  const Eigen::Vector4d homogeneous(point.x(),point.y(),point.z(),1);
  const Eigen::Vector3d expected=(E.inverse()*D*E*homogeneous).head<3>();
  b::require((expected-b::rotationDeskew(point,D.block<3,3>(0,0),E)).norm()<1e-12,"lever_arm_test");
  const Matrix prediction=b::predict(Matrix::Identity(),E,b::exp(Eigen::Vector3d(.01,.02,-.03)),Eigen::Vector3d(.4,.2,0),.1);
  b::require(rigid(prediction) && (prediction*prediction.inverse()-Matrix::Identity()).norm()<1e-12,"SE3_roundtrip_test");
  bool refused=false;try {b::RotationTimeline missing(imu,999999999,1099000000);}catch(...) {refused=true;}
  b::require(refused,"leading_IMU_test");
  std::cout<<"CAUSAL_ROTATION_FRAME_TEST_PASS\n";
}
int main(int argc,char** argv) {
  uint64_t active_transaction=0;
  unsigned frames=0,align_calls=0,align_attempts=0;
  std::string active_arm="NONE",active_stage="SETUP";
  p::CurrentFrameNdtResult active_nominal;
  bool nominal_completed=false,current_state_committed=false;
  const auto run_start=Clock::now();
  try {
    if(argc==2 && std::string(argv[1])=="--self-test") {selfTest();return 0;}
    b::require(argc==5,"usage: benchmark INPUT MAP OUT EXTRINSIC_TXT");
    const std::string input=argv[1],outdir=argv[3];
    Matrix E;std::ifstream extrinsic(argv[4]);
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) b::require(bool(extrinsic>>E(r,c)),"extrinsic_read");
    b::require(rigid(E),"extrinsic_not_rigid");
    const auto scans=p::readP7TimedScans(input+"/filter_scans.csv",input+"/raw_timed_scan_index.csv");
    const auto imu=p::readP7Imu(input+"/imu.csv");
    b::require(scans.size()==2777 && imu.size()==55957,"input_counts_changed");
    p::CurrentFrameNdtRegistration registration{p::CurrentFrameNdtParameters{}};
    std::string reason;
    if(!registration.loadMap(argv[2],&reason)) throw std::runtime_error(reason);
    b::require(registration.targetPointCount()>0,"empty_target");
    std::ofstream log(outdir+"/frames.csv"),ledger(outdir+"/source_ledger.csv");
    b::require(bool(log) && bool(ledger),"cannot_open_output");
    log<<std::setprecision(17);
    log<<"method,transaction_id,scan_start_ns,scan_end_ns,source_count,source_hash,target_count,ndt_effective,pcl_converged,iterations,ndt_status,raw_score,fitness_m2,triggered,anchor_valid,weak_attempted,weak_dimension,weak_quality,feedback,strong_selected,jet_calls,value_calls,extra_align_calls,weak_m,strong_m,weak_deg,strong_deg,lambda0,lambda1,lambda2,lambda3,lambda4,lambda5,weak_score,coupled_score,candidate_score,displaced_cross_norm,refinement_status,strong_status,innovation_m,innovation_deg,step_m,step_deg,failure_streak,common_source_ms,common_source_cpu_ms,align_ms,refinement_ms,total_ms,total_cpu_ms,peak_RSS_KiB";
    for(const std::string prefix:{"prediction","nominal","weak","coupled","executed"})
      for(int r=0;r<3;++r) for(int c=0;c<4;++c) log<<','<<prefix<<"_r"<<r<<'c'<<c;
    log<<'\n';
    ledger<<"transaction_id,scan_start_ns,scan_end_ns,raw_points,IMU_max_consumed_ns,IMU_gap_ns,endpoint_hold_ns,point_count_preserved,translation_deskew,gyro_bias_status,source_hash_identical\n";
    std::array<Arm,3> arms;arms[0].name="NOMINAL";arms[1].name="WEAK_ONLY";arms[2].name="COUPLED";
    std::string stop="FULL_ELIGIBLE_INPUT_PROCESSED";
    for(const auto& scan:scans) {
      if(scan.transaction_id<52) continue;
      active_transaction=scan.transaction_id;
      active_arm="NONE";active_stage="SOURCE_READ_AND_ROTATIONAL_DESKEW";nominal_completed=false;current_state_committed=false;
      b::require(scan.scan_start_ns>=1517157224188980000ULL,"scan_before_prior_available");
      const auto source_start=Clock::now();
      const auto source_cpu_start=std::clock();
      const auto raw=p::readP7PackedTimedCloud(input+"/raw_timed_points.bin",scan);
      const b::RotationTimeline within(imu,scan.scan_start_ns,scan.scan_end_ns);
      p::RegistrationCloud cloud;cloud.reserve(raw.size());
      for(const auto& point:raw) {
        const Eigen::Matrix3d rotation=within.rotations.back().transpose()*within.at(point.stamp_ns);
        const Eigen::Vector3d q=b::rotationDeskew(point.position,rotation,E);
        b::require(q.allFinite(),"nonfinite_source");
        cloud.push_back({float(q.x()),float(q.y()),float(q.z())});
      }
      const double source_ms=milliseconds(source_start);
      const double source_cpu_ms=1000.*(std::clock()-source_cpu_start)/CLOCKS_PER_SEC;
      uint64_t shared_hash=0;std::size_t shared_count=0;
      for(std::size_t index=0;index<arms.size();++index) {
        auto& arm=arms[index];const auto start=Clock::now();
        const auto arm_cpu_start=std::clock();
        active_arm=arm.name;active_stage="PREDICTION";nominal_completed=false;current_state_committed=false;
        Matrix prediction=arm.current;
        if(arm.stamp) {
          const b::RotationTimeline between(imu,arm.stamp,scan.scan_end_ns);
          prediction=b::predict(arm.current,E,between.rotations.back(),arm.velocity,
              double(scan.scan_end_ns-arm.stamp)*1e-9);
          if(index) p::advanceCoupledAnchor(&arm.anchor,scan.scan_end_ns,arm.current.inverse()*prediction);
        }
        p::CurrentFrameNdtResult nominal;
        ++align_attempts;active_stage="NOMINAL_ALIGN";
        if(!registration.align(scan.scan_end_ns,cloud,pose(prediction),&nominal,&reason))
          throw std::runtime_error(reason);
        active_nominal=nominal;nominal_completed=true;
        if(nominal.source_point_count>=50) ++align_calls;
        if(index==0) {shared_hash=nominal.source_cloud_hash;shared_count=nominal.source_point_count;}
        b::require(nominal.source_cloud_hash==shared_hash && nominal.source_point_count==shared_count,"arm_source_mismatch");
        p::WeakCoupledResult refinement;
        const Matrix nominal_pose=matrix(nominal.raw_map_T_lidar);
        refinement.nominal=refinement.weak_pose=refinement.coupled_pose=refinement.candidate=nominal_pose.cast<float>();
        refinement.status="NOMINAL_CONTROL";
        if(index) {
          active_stage="R6_REFINEMENT";
          p::WeakCoupledConfig config;config.coupled=index==2;
          if(!registration.weakRefinement(nominal,arm.anchor,config,&refinement,&reason))
            throw std::runtime_error(reason);
          if(!nominal.effective) {
            arm.anchor.valid=false;arm.anchor.frozen=false;arm.anchor.invalidated_stamp_ns=scan.scan_end_ns;
            arm.anchor.status="NOMINAL_INEFFECTIVE";
          }
        }
        const bool feedback=index && nominal.effective && refinement.recommended;
        active_stage="EXECUTED_POSE_BUDGET_GUARD";
        const Matrix executed=feedback ? matrix(pose(refinement.candidate.cast<double>())) :
            nominal.effective ? nominal_pose : prediction;
        b::require(rigid(executed) && refinement.jet_calls<=2 && refinement.value_calls<=3,"pose_or_budget_invalid");
        b::require(refinement.triggered || (refinement.jet_calls==0 && refinement.value_calls==0),"untriggered_extra_work");
        if(nominal.effective) {
          ++arm.success;arm.failures=0;
          if(arm.accepted_stamp) arm.velocity=(executed*E.inverse()).block<3,1>(0,3)-
              (arm.accepted*E.inverse()).block<3,1>(0,3);
          if(arm.accepted_stamp) arm.velocity/=double(scan.scan_end_ns-arm.accepted_stamp)*1e-9;
          arm.accepted=executed;arm.accepted_stamp=scan.scan_end_ns;
        } else ++arm.failures;
        if(feedback) ++arm.feedback;
        if(index) p::settleWeakRefinementAnchor(&arm.anchor,scan.scan_end_ns,executed,
            nominal.effective && !refinement.triggered);
        const double step=(executed.block<3,1>(0,3)-arm.current.block<3,1>(0,3)).norm();
        const double step_deg=angle(executed,arm.current);
        arm.current=executed;arm.stamp=scan.scan_end_ns;
        current_state_committed=true;
        struct rusage rss;getrusage(RUSAGE_SELF,&rss);
        log<<arm.name<<','<<scan.transaction_id<<','<<scan.scan_start_ns<<','<<scan.scan_end_ns
           <<','<<nominal.source_point_count<<','<<nominal.source_cloud_hash<<','<<nominal.target_point_count
           <<','<<nominal.effective<<','<<nominal.converged<<','<<nominal.iterations<<','<<p::currentFrameNdtStatusName(nominal.status)
           <<','<<nominal.transformation_probability*nominal.source_point_count<<','<<nominal.fitness
           <<','<<refinement.triggered<<','<<refinement.anchor_valid<<','<<refinement.attempted<<','<<refinement.weak_dimension
           <<','<<refinement.weak_quality_valid<<','<<feedback<<','<<refinement.strong_selected<<','<<refinement.jet_calls
           <<','<<refinement.value_calls<<",0,"<<.8*refinement.weak_eta.head<3>().norm()<<','<<.8*refinement.strong_eta.head<3>().norm()
           <<','<<refinement.weak_eta.tail<3>().norm()*180/std::acos(-1.)<<','<<refinement.strong_eta.tail<3>().norm()*180/std::acos(-1.);
        for(int k=0;k<6;++k) log<<','<<refinement.eigenvalues(k);
        log<<','<<refinement.weak_score<<','<<refinement.coupled_score<<','<<refinement.candidate_score
           <<','<<refinement.displaced_cross_norm<<','<<refinement.status<<','<<refinement.strong_status
           <<','<<(nominal_pose.block<3,1>(0,3)-prediction.block<3,1>(0,3)).norm()<<','<<angle(nominal_pose,prediction)
           <<','<<step<<','<<step_deg<<','<<arm.failures<<','<<source_ms<<','<<source_cpu_ms<<','<<nominal.alignment_ms
           <<','<<refinement.total_ms<<','<<source_ms+milliseconds(start)
           <<','<<source_cpu_ms+1000.*(std::clock()-arm_cpu_start)/CLOCKS_PER_SEC<<','<<rss.ru_maxrss;
        writePose(log,prediction);writePose(log,nominal_pose);
        writePose(log,refinement.weak_pose.cast<double>());writePose(log,refinement.coupled_pose.cast<double>());
        writePose(log,executed);log<<'\n';
      }
      ledger<<scan.transaction_id<<','<<scan.scan_start_ns<<','<<scan.scan_end_ns<<','<<raw.size()
            <<','<<within.maximum_consumed_ns<<','<<within.maximum_gap_ns<<','<<within.endpoint_hold_ns
            <<",1,NOT_PERFORMED,PROVISIONAL_ZERO_NOT_ESTIMATED,1\n";
      active_stage="OUTPUT_FLUSH";log.flush();ledger.flush();
      b::require(bool(log) && bool(ledger),"output_write_or_flush_failed");
      ++frames;
      if(frames==1 && arms[0].success!=1) {stop="FIRST_NDT_STRICT_STARTUP_FAILED";break;}
      if(frames==300) {
        std::cout<<"PHASE300";for(const auto& arm:arms) std::cout<<' '<<arm.name<<'='<<arm.success;
        std::cout<<std::endl;
        if(std::any_of(arms.begin(),arms.end(),[](const Arm& arm){return arm.success<270;})) {
          stop="PHASE300_EFFECTIVE_RATE_BELOW_90_PERCENT";break;
        }
      }
      if(frames%100==0) std::cout<<"PROGRESS frames="<<frames<<" tx="<<scan.transaction_id<<std::endl;
    }
    active_stage="EXECUTION_RECEIPT_WRITE";
    std::ofstream receipt(outdir+"/execution.json");
    receipt<<std::setprecision(17)<<"{\"frames_per_arm\":"<<frames<<",\"full_align_calls\":"<<align_calls
           <<",\"registration_align_attempts\":"<<align_attempts
           <<",\"wall_s\":"<<milliseconds(run_start)/1000<<",\"status\":\""<<stop
           <<"\",\"GT_LOADED\":false,\"IKFOM_RUN\":false,\"MAP_INSTANCES\":1}\n";
    receipt.flush();b::require(bool(receipt),"execution_receipt_write_failed");
    std::cout<<stop<<" frames="<<frames<<std::endl;
    return 0;
  }catch(const std::exception& error){
    std::cerr<<"BENCHMARK_STOP tx="<<active_transaction<<" align_calls="<<align_calls<<' '<<error.what()<<'\n';
    if(argc==5) {
      std::ofstream receipt(std::string(argv[3])+"/execution_failure.json");
      receipt<<"{\"status\":\"EXCEPTION_SEE_RUN_LOG\",\"failed_transaction\":"<<active_transaction
             <<",\"completed_frames_per_arm\":"<<frames<<",\"attempted_align_calls\":"<<align_attempts
             <<",\"completed_full_align_calls\":"<<align_calls<<",\"active_arm\":\""<<active_arm
             <<"\",\"active_stage\":\""<<active_stage<<"\",\"current_nominal_completed\":"<<(nominal_completed?"true":"false")
             <<",\"current_state_committed\":"<<(current_state_committed?"true":"false")
             <<",\"archival_acceptance\":false,\"source_count\":"<<(nominal_completed?active_nominal.source_point_count:0)
             <<",\"source_hash\":\""<<(nominal_completed?active_nominal.source_cloud_hash:0)
             <<"\",\"current_nominal_status\":\""<<(nominal_completed?p::currentFrameNdtStatusName(active_nominal.status):"NOT_RUN")<<"\"";
      if(nominal_completed) {
        receipt<<",\"nominal_pose_3x4\":[";
        const Matrix T=matrix(active_nominal.raw_map_T_lidar);
        for(int r=0;r<3;++r) for(int c=0;c<4;++c) {
          if(r||c) receipt<<',';
          if(std::isfinite(T(r,c))) receipt<<std::setprecision(17)<<T(r,c);else receipt<<"null";
        }
        receipt<<"]";
      }
      receipt<<",\"wall_s\":"<<milliseconds(run_start)/1000<<",\"GT_LOADED\":false,\"IKFOM_RUN\":false}\n";
      receipt.flush();
      if(!receipt) std::cerr<<"FAILURE_RECEIPT_WRITE_FAILED\n";
    }
    return 2;
  }
}
