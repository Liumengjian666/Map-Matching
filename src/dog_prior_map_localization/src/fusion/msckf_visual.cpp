#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"
#include "dog_prior_map_localization/core/math_utils.hpp"
#include "dog_prior_map_localization/core/square_root_covariance.hpp"
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <boost/math/distributions/chi_squared.hpp>

namespace dog_prior_map_localization {
namespace {
constexpr uint64_t kVisualId = uint64_t{1} << 62;
Eigen::Matrix3d expRotation(const Eigen::Vector3d& x) {
  if (x.norm()<1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(x.norm(),x.normalized()).toRotationMatrix();
}
Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& x) {
  const double a=x.norm(); const Eigen::Matrix3d X=skew(x);
  if(a<1e-6) return Eigen::Matrix3d::Identity()-0.5*X+X*X/6.0;
  return Eigen::Matrix3d::Identity()-(1-std::cos(a))/(a*a)*X+(a-std::sin(a))/(a*a*a)*X*X;
}
Eigen::MatrixXd compressed(const Eigen::MatrixXd& columns) {
  if(!columns.allFinite()) throw std::runtime_error("MSCKF_NONFINITE_FACTOR");
  if(columns.cols()<columns.rows()) return columns;
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(columns.transpose());
  Eigen::MatrixXd R=qr.matrixQR().topLeftCorner(columns.rows(),columns.rows()).triangularView<Eigen::Upper>();
  for(int i=0;i<R.rows();++i) if(R(i,i)<0) R.row(i)*=-1;
  return R.transpose();
}
std::vector<std::string> csvCells(const std::string& row) {
  std::vector<std::string> out; std::stringstream stream(row); std::string cell;
  while(std::getline(stream,cell,',')) out.push_back(cell);
  return out;
}
}

void DogPriorMapEkfNode::initializeMsckf() {
  msckf_time_enable_=getParam<bool>("mature_visual/time_adapter_enable",false);
  msckf_update_enable_=getParam<bool>("mature_visual/enable",false);
  if(!msckf_time_enable_) {
    if(msckf_update_enable_) throw std::runtime_error("MSCKF_REQUIRES_SHARED_TIME_ADAPTER");
    return;
  }
  if(!square_root_covariance_enable_ || !oosm_fixed_lag_enable_ || skf_relative_visual_update_enable_ ||
      getParam<bool>("lidar_update/visual_initial_guess_enable",false))
    throw std::runtime_error("MSCKF_REQUIRES_SQRT_FIXED_LAG_AND_OLD_VISION_OFF");
  if(!use_acc_for_position_) throw std::runtime_error("MSCKF_REQUIRES_ACCELERATION_PROPAGATION");
  msckf_max_clones_=getParam<int>("mature_visual/max_clones",11);
  msckf_pixel_sigma_=getParam<double>("mature_visual/pixel_sigma",1.0);
  if(msckf_max_clones_<6 || msckf_max_clones_>11 || msckf_pixel_sigma_<=0)
    throw std::runtime_error("MSCKF_INVALID_CONFIG");
  std::vector<double> K,T;
  if(!nh_.getParam("mature_visual/rectified_K",K) || K.size()!=9 ||
     !nh_.getParam("mature_visual/T_imu_camera",T) || T.size()!=16)
    throw std::runtime_error("MSCKF_CAMERA_CALIBRATION_REQUIRED");
  for(int i=0;i<3;++i) {
    msckf_camera_.p_imu_camera(i)=T[4*i+3];
    for(int j=0;j<3;++j) { msckf_camera_.K(i,j)=K[3*i+j]; msckf_camera_.R_imu_camera(i,j)=T[4*i+j]; }
  }
  // Calibration contains rounding at ~1e-8; normalize the existing rotation,
  // as in the original LiDAR extrinsic loader. No extrinsic optimization.
  msckf_camera_.R_imu_camera=Eigen::Quaterniond(msckf_camera_.R_imu_camera).normalized().toRotationMatrix();
  std::ifstream input(getParam<std::string>("mature_visual/tracks_csv",""));
  std::string row;
  if(!input || !std::getline(input,row) || row!="scan_ns,image_ns,feature_id,observation_ns,u,v")
    throw std::runtime_error("MSCKF_TRACK_INPUT_HEADER");
  while(std::getline(input,row)) {
    if(!row.empty() && row.back()=='\r') row.pop_back();
    auto cells=csvCells(row); if(cells.size()!=6) throw std::runtime_error("MSCKF_TRACK_INPUT_ROW");
    const uint64_t scan=std::stoull(cells[0]),image=std::stoull(cells[1]),id=std::stoull(cells[2]);
    if(image==0 || image>scan) throw std::runtime_error("MSCKF_FUTURE_IMAGE");
    auto& frame=visual_frames_[image];
    if(frame.scan_ns && frame.scan_ns!=scan) throw std::runtime_error("MSCKF_DUPLICATE_IMAGE_IDENTITY");
    frame.scan_ns=scan; frame.image_ns=image;
    if(id) {
      const uint64_t observation=std::stoull(cells[3]);
      if(!causalVisualTime(observation,image)) throw std::runtime_error("MSCKF_FUTURE_PIXEL");
      Eigen::Vector2d pixel(std::stod(cells[4]),std::stod(cells[5]));
      if(!pixel.allFinite()) throw std::runtime_error("MSCKF_NONFINITE_PIXEL");
      frame.tracks[id].push_back({observation,pixel});
    }
  }
  visual_joint_.factor=S_;
  const auto path=getParam<std::string>("mature_visual/diagnostics_csv","");
  std::ifstream existing(path); if(path.empty() || existing.good()) throw std::runtime_error("MSCKF_OUTPUT_EXISTS_OR_EMPTY");
  msckf_csv_.open(path);
  msckf_csv_ << "image_ns,scan_ns,status,tracks,triangulated,accepted,residual_rows,nis_rejected,geometry_rejected,selected_directions,trace_before,trace_after,correction_m,correction_rad,max_residual_px,update_ms,replay_imu_count,clones,degeneracy_end_ns\n";
  sub_visual_degeneracy_=nh_.subscribe("/dog_livo/visual_degeneracy",32,&DogPriorMapEkfNode::visualDegeneracyCallback,this);
}

void DogPriorMapEkfNode::synchronizeMsckfCoreRoot() {
  if(!covarianceRootFromColumns(visual_joint_.factor.topRows(15),&S_))
    throw std::runtime_error("MSCKF_CORE_MARGINAL_FAILED");
}

void DogPriorMapEkfNode::visualDegeneracyCallback(const dog_prior_map_interfaces::VisualDegeneracyConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!msckf_time_enable_ || !msg || msg->header.frame_id!=map_frame_ ||
     msg->scan_end_ns<msg->header.stamp.toNSec()) return;
  visual_degeneracy_.emplace(msg->scan_end_ns,msg);
  while(visual_degeneracy_.size()>32) visual_degeneracy_.erase(visual_degeneracy_.begin());
}

// Causal ZOH kinematics. One acceleration/gyro noise driver is held for an
// entire IMU interval, INCLUDING all camera subdivisions. Thus a partial
// interval and its remainder share their process-noise columns. Bias random
// walks use independent increments. No future tail value is read at an image.
void DogPriorMapEkfNode::propagateMsckfImu(const Eigen::Vector3d& tail_acc,
                                        const Eigen::Vector3d& tail_gyro,double dt) {
  const double end=state_stamp_+dt;
  const Eigen::Vector3d head_acc=last_acc_measurement_,head_gyro=last_gyro_measurement_;
  if(!visual_joint_.driver_active) {
    const auto L=visual_joint_.factor;
    visual_joint_.factor=Eigen::MatrixXd::Zero(L.rows()+6,L.cols()+6);
    visual_joint_.factor.topLeftCorner(L.rows(),L.cols())=L;
    visual_joint_.factor.bottomRightCorner(6,6).diagonal()<<acc_noise_,acc_noise_,acc_noise_,gyro_noise_,gyro_noise_,gyro_noise_;
    visual_joint_.driver_active=true;
    visual_joint_.driver_head_stamp=state_stamp_;
  }
  auto step=[&](double target) {
    const double d=target-state_stamp_; if(d<=0) return;
    const Eigen::Vector3d a=head_acc-ba_,w=head_gyro-bg_,phi=w*d;
    const Eigen::Matrix3d Rend=R_*expRotation(phi);
    Matrix15d F; Eigen::Matrix<double,15,6> G;
    linearizeCausalImu(Rend,a,w,d,&F,&G);
    const auto L=visual_joint_.factor;
    Eigen::MatrixXd columns=Eigen::MatrixXd::Zero(L.rows(),L.cols()+6);
    columns.leftCols(L.cols())=L;
    columns.topLeftCorner(15,L.cols())=F*L.topRows(15)+G*L.bottomRows(6);
    columns.block(9,L.cols(),3,3)=acc_bias_noise_*std::sqrt(d)*Eigen::Matrix3d::Identity();
    columns.block(12,L.cols()+3,3,3)=gyro_bias_noise_*std::sqrt(d)*Eigen::Matrix3d::Identity();
    visual_joint_.factor=compressed(columns);
    R_=Rend; last_acc_world_=R_*a+g_; last_unbiased_gyro_=w;
    p_+=v_*d+0.5*last_acc_world_*d*d; v_+=last_acc_world_*d;
    state_stamp_=target; last_imu_time_=target;
    last_interval_acc_input_=head_acc; last_interval_gyro_input_=head_gyro; has_last_interval_input_=true;
    synchronizeMsckfCoreRoot();
  };
  // Times are only used as an event schedule. Pixel values are not consumed
  // until their own fixed-lag transaction is released.
  for(auto& item:visual_frames_) {
    const double t=ros::Time().fromNSec(item.first).toSec();
    if(t<=state_stamp_+1e-9 || t>end+1e-9) continue;
    step(t);
    VisualPoseClone clone; clone.image_ns=item.first; clone.p=clone.p_fej=p_; clone.R=clone.R_fej=R_;
    if(!item.second.first_estimate_set) {
      item.second.first_estimate=clone; item.second.first_estimate_set=true;
    } else {
      // OOSM rewrites nominal poses, not the immutable first-estimate chart.
      clone.p_fej=item.second.first_estimate.p;
      clone.R_fej=item.second.first_estimate.R;
    }
    const auto L=visual_joint_.factor; const int pose_rows=L.rows()-6;
    Eigen::MatrixXd augmented(L.rows()+6,L.cols());
    augmented.topRows(pose_rows)=L.topRows(pose_rows);
    augmented.block(pose_rows,0,3,L.cols())=L.topRows(3);
    augmented.block(pose_rows+3,0,3,L.cols())=L.middleRows(6,3);
    augmented.bottomRows(6)=L.bottomRows(6);
    visual_joint_.factor=augmented; visual_joint_.clones.push_back(clone);
    if(static_cast<int>(visual_joint_.clones.size())>msckf_max_clones_) {
      Eigen::MatrixXd selected(augmented.rows()-6,augmented.cols());
      selected.topRows(15)=augmented.topRows(15); selected.bottomRows(augmented.rows()-21)=augmented.bottomRows(augmented.rows()-21);
      visual_joint_.factor=compressed(selected); visual_joint_.clones.erase(visual_joint_.clones.begin());
    }
    saveStateSnapshot(t);
  }
  step(end);
  // Marginalize, do not condition on or reset the correlated driver.
  visual_joint_.factor=compressed(visual_joint_.factor.topRows(visual_joint_.factor.rows()-6).eval());
  visual_joint_.driver_active=false;
  last_acc_measurement_=tail_acc; last_gyro_measurement_=tail_gyro;
  synchronizeMsckfCoreRoot();
}

void DogPriorMapEkfNode::queueMsckfFramesLocked() {
  if(!msckf_time_enable_ || !has_state_stamp_) return;
  for(const auto& pair:visual_frames_) {
    const auto& frame=pair.second;
    if(ros::Time().fromNSec(frame.scan_ns).toSec()>state_stamp_+1e-9 || visual_queued_.count(frame.image_ns)) continue;
    visual_queued_.insert(frame.image_ns);
    const auto status=oosm_fixed_lag_scheduler_.enqueue(frame.image_ns|kVisualId,
        ros::Time().fromNSec(frame.image_ns).toSec(),state_stamp_,ros::WallTime::now().toSec());
    if(status!=FixedLagEnqueueStatus::kQueued) throw std::runtime_error("MSCKF_FIXED_LAG_ENQUEUE_FAILED");
    writeFixedLagDiagnostic(frame.image_ns|kVisualId,ros::Time().fromNSec(frame.image_ns).toSec(),
      ros::Time().fromNSec(frame.image_ns).toSec()+oosm_fixed_lag_sec_,state_stamp_,ros::WallTime::now().toSec(),
      std::numeric_limits<double>::quiet_NaN(),ros::WallTime::now().toSec(),0,false,"QUEUED_MSCKF");
  }
}

void DogPriorMapEkfNode::processMsckfLocked(uint64_t image_ns) {
  const auto begin=ros::WallTime::now(); const auto& frame=visual_frames_.at(image_ns);
  const double target=ros::Time().fromNSec(image_ns).toSec(),now=state_stamp_;
  int triangulated=0,accepted=0,nis_rejected=0,geometry_rejected=0,rows=0,directions=0,replayed=0;
  double before=S_.squaredNorm(),after=before,dp_norm=0,dr_norm=0,max_residual=0;
  uint64_t report_end=0; std::string status="DISABLED";
  auto log=[&]() {
    msckf_last_status_=status; msckf_last_replay_=replayed;
    msckf_csv_<<std::setprecision(17)<<image_ns<<','<<frame.scan_ns<<','<<status<<','<<frame.tracks.size()<<','
      <<triangulated<<','<<accepted<<','<<rows<<','<<nis_rejected<<','<<geometry_rejected<<','<<directions<<','
      <<before<<','<<after<<','<<dp_norm<<','<<dr_norm<<','<<max_residual<<','
      <<(ros::WallTime::now()-begin).toSec()*1000<<','<<replayed<<','<<visual_joint_.clones.size()<<','<<report_end<<'\n';
    msckf_csv_.flush();
  };
  if(!msckf_update_enable_) { log(); return; }
  std::size_t index=0; double age=0;
  if(!state_history_.findAtOrBefore(target,index,age) || age>1e-6 || state_history_.at(index).visual_joint.factor.rows()<15) {
    status="NO_EXACT_IMAGE_CLONE_STATE"; log(); return;
  }
  auto report=visual_degeneracy_.upper_bound(image_ns);
  if(report==visual_degeneracy_.begin()) { status="NO_CAUSAL_DCREG_REPORT"; log(); return; }
  --report; report_end=report->first;
  if(!report->second->valid || image_ns-report_end>uint64_t{200000000}) { status="INVALID_OR_STALE_DCREG"; log(); return; }
  Eigen::Matrix3d Pt=Eigen::Matrix3d::Zero(),Pr=Eigen::Matrix3d::Zero();
  const auto& degeneracy=*report->second;
  const double ratio=getParam<double>("mature_integration/degeneracy_eigen_ratio",0.03);
  for(int k=0;k<3;++k) {
    if(degeneracy.translation_degenerate && std::abs(degeneracy.translation_eigenvalues[k])<=ratio*std::abs(degeneracy.translation_eigenvalues[2])) {
      Eigen::Vector3d u; for(int j=0;j<3;++j) u(j)=degeneracy.translation_axes[3*j+k]; Pt+=u*u.transpose(); ++directions;
    }
    if(degeneracy.rotation_degenerate && std::abs(degeneracy.rotation_eigenvalues[k])<=ratio*std::abs(degeneracy.rotation_eigenvalues[2])) {
      Eigen::Vector3d u; for(int j=0;j<3;++j) u(j)=degeneracy.rotation_axes[3*j+k]; Pr+=u*u.transpose(); ++directions;
    }
  }
  if(!directions) { status="NO_WEAK_DIRECTION"; log(); return; }
  const auto original=state_history_.at(index); auto working=original;
  auto& joint=working.visual_joint; const int dim=joint.factor.rows();
  Eigen::MatrixXd selector=Eigen::MatrixXd::Zero(dim,dim);
  selector.block<3,3>(0,0)=Pt; selector.block<3,3>(6,6)=working.R.transpose()*Pr*working.R;
  for(std::size_t k=0;k<joint.clones.size();++k) {
    selector.block<3,3>(15+6*k,15+6*k)=Pt;
    selector.block<3,3>(18+6*k,18+6*k)=joint.clones[k].R.transpose()*Pr*joint.clones[k].R;
  }
  std::vector<Eigen::MatrixXd> Hs; std::vector<Eigen::VectorXd> rs;
  for(const auto& track:frame.tracks) {
    std::vector<PixelObservation> observations;
    bool available=true;
    for(const auto& pixel:track.second) {
      auto clone=std::find_if(joint.clones.begin(),joint.clones.end(),[&](const VisualPoseClone& c){return c.image_ns==pixel.image_ns;});
      if(clone==joint.clones.end()) { available=false; break; }
      observations.push_back({static_cast<std::size_t>(clone-joint.clones.begin()),pixel.pixel});
    }
    Eigen::Vector3d point; std::string reason;
    if(!available || !triangulateVisualTrack(joint.clones,msckf_camera_,observations,&point,&reason)) { ++geometry_rejected; continue; }
    ++triangulated;
    // FEJ landmark expressed from its anchor clone; live residual, frozen
    // Jacobians. The nullspace removes landmark nuisance variables.
    const auto& anchor=joint.clones[observations.front().clone];
    const Eigen::Vector3d point_fej=anchor.p_fej+anchor.R_fej*anchor.R.transpose()*(point-anchor.p);
    Eigen::MatrixXd H=Eigen::MatrixXd::Zero(2*observations.size(),dim),Hf(2*observations.size(),3);
    Eigen::VectorXd r(2*observations.size()); bool valid=true;
    for(std::size_t k=0;k<observations.size();++k) {
      const auto& o=observations[k]; auto frozen=joint.clones[o.clone]; frozen.p=frozen.p_fej; frozen.R=frozen.R_fej;
      const auto nominal=linearizeReprojection(joint.clones[o.clone],msckf_camera_,point);
      const auto linear=linearizeReprojection(frozen,msckf_camera_,point_fej);
      if(!nominal.valid || !linear.valid) { valid=false; break; }
      H.block(2*k,15+6*o.clone,2,6)=linear.H_pose; Hf.block(2*k,0,2,3)=linear.H_point;
      r.segment<2>(2*k)=o.pixel-nominal.prediction;
    }
    Eigen::MatrixXd Hp; Eigen::VectorXd rp;
    if(!valid || !eliminateVisualFeature(Hf,H,r,&Hp,&rp) || (Hp*selector).norm()<1e-9) { ++geometry_rejected; continue; }
    const Eigen::MatrixXd U=Hp*joint.factor;
    const Eigen::MatrixXd innovation=U*U.transpose()+msckf_pixel_sigma_*msckf_pixel_sigma_*Eigen::MatrixXd::Identity(Hp.rows(),Hp.rows());
    Eigen::LLT<Eigen::MatrixXd> solver(innovation);
    if(solver.info()!=Eigen::Success || rp.dot(solver.solve(rp))>boost::math::quantile(boost::math::chi_squared(Hp.rows()),0.95)) { ++nis_rejected; continue; }
    max_residual=std::max(max_residual,r.cwiseAbs().maxCoeff());
    rows+=Hp.rows(); Hs.push_back(Hp); rs.push_back(rp); ++accepted;
    if(accepted>=40) break;
  }
  if(!accepted) { status="NO_ACCEPTED_REPROJECTION_TRACK"; log(); return; }
  Eigen::MatrixXd H(rows,dim); Eigen::VectorXd r(rows); int offset=0;
  for(std::size_t k=0;k<Hs.size();++k) { H.middleRows(offset,Hs[k].rows())=Hs[k]; r.segment(offset,rs[k].size())=rs[k]; offset+=Hs[k].rows(); }
  // Orthogonal measurement compression; pixel noise is isotropic. Keep
  // residual-only rows out of the update (per-track NIS already evaluated).
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(H);
  const int rank=qr.rank();
  const Eigen::MatrixXd projected=qr.householderQ().adjoint()*H;
  const Eigen::VectorXd projected_r=qr.householderQ().adjoint()*r;
  H=projected.topRows(rank); r=projected_r.head(rank);
  Eigen::MatrixXd K,updated;
  if(!correlatedVisualJoseph(joint.factor,Eigen::MatrixXd::Zero(rank,joint.factor.cols()),
      msckf_pixel_sigma_*Eigen::MatrixXd::Identity(rank,rank),H,selector,&K,&updated)) {
    status="VISUAL_FACTOR_UPDATE_REJECTED"; log(); return;
  }
  const Eigen::VectorXd dx=K*r;
  if(!dx.allFinite()) { status="NONFINITE_CORRECTION"; log(); return; }
  working.p+=dx.segment<3>(0); working.v+=dx.segment<3>(3);
  working.R=working.R*expRotation(dx.segment<3>(6)); working.ba+=dx.segment<3>(9); working.bg+=dx.segment<3>(12);
  Eigen::MatrixXd reset=Eigen::MatrixXd::Identity(dim,dim);
  reset.block<3,3>(6,6)=rightJacobian(dx.segment<3>(6));
  for(std::size_t k=0;k<joint.clones.size();++k) {
    joint.clones[k].p+=dx.segment<3>(15+6*k);
    joint.clones[k].R=joint.clones[k].R*expRotation(dx.segment<3>(18+6*k));
    reset.block<3,3>(18+6*k,18+6*k)=rightJacobian(dx.segment<3>(18+6*k));
  }
  joint.factor=compressed(reset*updated);
  if(!covarianceRootFromColumns(joint.factor.topRows(15),&working.S)) throw std::runtime_error("MSCKF_UPDATED_CORE_FACTOR");
  before=original.S.squaredNorm(); after=working.S.squaredNorm(); dp_norm=dx.segment<3>(0).norm(); dr_norm=dx.segment<3>(6).norm();
  square_root_diagnostics_.record("MSCKF_UPDATE",target,working.S,(ros::WallTime::now()-begin).toSec()*1000,before);
  eraseStateHistoryAfter(target); restoreStateSnapshot(working); saveStateSnapshot(target);
  for(const auto& imu:imu_history_) {
    if(imu.stamp<=target+1e-9 || imu.stamp>now+1e-9) continue;
    const double replay_dt=imu.stamp-state_stamp_;
    if(!imu.acc.allFinite() || !imu.gyro.allFinite() || !std::isfinite(replay_dt) ||
       replay_dt<=0 || replay_dt>max_imu_dt_+1e-9) throw std::runtime_error("MSCKF_REPLAY_IMU_TIME_GATE");
    propagateImu(imu.acc,imu.gyro,replay_dt); state_stamp_=imu.stamp; last_imu_time_=imu.stamp;
    saveStateSnapshot(state_stamp_); ++replayed;
  }
  if(std::abs(state_stamp_-now)>1e-6 || !p_.allFinite() || !v_.allFinite() || !validCovarianceRoot(S_))
    throw std::runtime_error("MSCKF_REPLAY_CONTRACT_FAILED");
  ++ekf_state_revision_;
  status="APPLIED_REPROJECTION_SELECTED_GAIN"; log();
}
}  // namespace dog_prior_map_localization
