#include "dog_prior_map_localization/core/msckf_visual_measurement.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <cmath>

namespace dog_prior_map_localization {
namespace {
Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& x) {
  Eigen::Matrix3d M; M<<0,-x.z(),x.y(),x.z(),0,-x.x(),-x.y(),x.x(),0; return M;
}
bool properRotation(const Eigen::Matrix3d& R) {
  return R.allFinite() && (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-6 && std::abs(R.determinant()-1)<1e-6;
}
bool legalFactor(const Eigen::MatrixXd& L) {
  return L.rows()>=15 && (L.rows()-15)%6==0 && L.cols()>=15 && L.allFinite();
}
bool compressFactor(const Eigen::MatrixXd& input,Eigen::MatrixXd* output) {
  if(!output || !legalFactor(input)) return false;
  if(input.cols()<input.rows()) { *output=input; return true; }
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(input.transpose());
  Eigen::MatrixXd R=qr.matrixQR().topLeftCorner(input.rows(),input.rows()).triangularView<Eigen::Upper>();
  for(int i=0;i<R.rows();++i) if(R(i,i)<0) R.row(i)*=-1;
  *output=R.transpose(); return output->allFinite();
}
}  // namespace

ReprojectionLinearization linearizeReprojection(const VisualPoseClone& pose,const RectifiedCamera& camera,
                                               const Eigen::Vector3d& point) {
  ReprojectionLinearization out;
  if(!properRotation(pose.R) || !properRotation(camera.R_imu_camera) || !pose.p.allFinite() ||
     !point.allFinite() || !camera.K.allFinite() || !camera.p_imu_camera.allFinite() ||
     camera.K(0,0)<=0 || camera.K(1,1)<=0) return out;
  const Eigen::Vector3d p_imu=pose.R.transpose()*(point-pose.p);
  const Eigen::Matrix3d R_ci=camera.R_imu_camera.transpose();
  const Eigen::Vector3d q=R_ci*(p_imu-camera.p_imu_camera);
  if(!q.allFinite() || q.z()<=0.1 || q.norm()>60.0) return out;
  out.prediction<<camera.K(0,0)*q.x()/q.z()+camera.K(0,2),camera.K(1,1)*q.y()/q.z()+camera.K(1,2);
  Eigen::Matrix<double,2,3> J;
  J<<camera.K(0,0)/q.z(),0,-camera.K(0,0)*q.x()/(q.z()*q.z()),
     0,camera.K(1,1)/q.z(),-camera.K(1,1)*q.y()/(q.z()*q.z());
  out.H_pose.leftCols<3>()=-J*R_ci*pose.R.transpose();
  out.H_pose.rightCols<3>()=J*R_ci*crossMatrix(p_imu);
  out.H_point=J*R_ci*pose.R.transpose();
  out.valid=out.prediction.allFinite() && out.H_pose.allFinite() && out.H_point.allFinite();
  return out;
}

bool triangulateVisualTrack(const std::vector<VisualPoseClone>& clones,const RectifiedCamera& camera,
                           const std::vector<PixelObservation>& observations,Eigen::Vector3d* point,std::string* reason) {
  if(!point || !reason || observations.size()<3 || !properRotation(camera.R_imu_camera) ||
      camera.K(0,0)<=0 || camera.K(1,1)<=0) return false;
  Eigen::Matrix3d A=Eigen::Matrix3d::Zero(); Eigen::Vector3d b=Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> centers;
  for(const auto& observation:observations) {
    if(observation.clone>=clones.size() || !observation.pixel.allFinite()) { *reason="INVALID_TRACK"; return false; }
    const auto& clone=clones[observation.clone];
    if(!properRotation(clone.R)) { *reason="INVALID_CLONE_ROTATION"; return false; }
    Eigen::Vector3d ray((observation.pixel.x()-camera.K(0,2))/camera.K(0,0),
                        (observation.pixel.y()-camera.K(1,2))/camera.K(1,1),1);
    ray=(clone.R*camera.R_imu_camera*ray).normalized();
    const Eigen::Vector3d center=clone.p+clone.R*camera.p_imu_camera;
    const Eigen::Matrix3d projector=Eigen::Matrix3d::Identity()-ray*ray.transpose();
    A+=projector; b+=projector*center; centers.push_back(center);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(A);
  if(eigen.info()!=Eigen::Success || eigen.eigenvalues().minCoeff()<=1e-12 ||
      eigen.eigenvalues().maxCoeff()/eigen.eigenvalues().minCoeff()>10000) { *reason="TRIANGULATION_CONDITION"; return false; }
  *point=A.ldlt().solve(b);
  double baseline=0;
  for(const auto& center:centers) baseline=std::max(baseline,(center-centers.front()).norm());
  if(baseline<=1e-9 || (*point-centers.front()).norm()/baseline>40) { *reason="INSUFFICIENT_PARALLAX"; return false; }
  // Fixed, bounded landmark-only Gauss-Newton, no pose/candidate optimization.
  for(int iteration=0;iteration<5;++iteration) {
    Eigen::Matrix3d normal=Eigen::Matrix3d::Zero(); Eigen::Vector3d rhs=Eigen::Vector3d::Zero();
    for(const auto& observation:observations) {
      const auto linear=linearizeReprojection(clones[observation.clone],camera,*point);
      if(!linear.valid) { *reason="INVALID_DEPTH"; return false; }
      normal+=linear.H_point.transpose()*linear.H_point;
      rhs+=linear.H_point.transpose()*(observation.pixel-linear.prediction);
    }
    Eigen::LDLT<Eigen::Matrix3d> solver(normal);
    if(solver.info()!=Eigen::Success || !solver.isPositive()) { *reason="LANDMARK_SOLVE_FAILED"; return false; }
    const Eigen::Vector3d delta=solver.solve(rhs);
    if(!delta.allFinite()) { *reason="NONFINITE_LANDMARK"; return false; }
    *point+=delta; if(delta.norm()<1e-6) break;
  }
  *reason="TRIANGULATED"; return point->allFinite();
}

bool eliminateVisualFeature(const Eigen::MatrixXd& Hf,const Eigen::MatrixXd& Hx,
                            const Eigen::VectorXd& residual,Eigen::MatrixXd* H_out,Eigen::VectorXd* r_out) {
  if(!H_out || !r_out || Hf.cols()!=3 || Hf.rows()<=3 || Hx.rows()!=Hf.rows() ||
      residual.size()!=Hf.rows() || !Hf.allFinite() || !Hx.allFinite() || !residual.allFinite()) return false;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(Hf);
  if(qr.rank()!=3) return false;
  const Eigen::MatrixXd projected=qr.householderQ().adjoint()*Hx;
  const Eigen::VectorXd projected_r=qr.householderQ().adjoint()*residual;
  *H_out=projected.bottomRows(Hf.rows()-3); *r_out=projected_r.tail(Hf.rows()-3);
  return H_out->allFinite() && r_out->allFinite();
}

bool appendPoseCloneFactor(const Eigen::MatrixXd& L,Eigen::MatrixXd* augmented) {
  if(!augmented || !legalFactor(L)) return false;
  Eigen::MatrixXd candidate(L.rows()+6,L.cols()); candidate.topRows(L.rows())=L;
  candidate.block(L.rows(),0,3,L.cols())=L.topRows<3>();
  candidate.block(L.rows()+3,0,3,L.cols())=L.middleRows<3>(6);
  *augmented=std::move(candidate);
  return augmented->allFinite();
}
bool marginalizeOldestPoseCloneFactor(const Eigen::MatrixXd& L,Eigen::MatrixXd* retained) {
  if(!retained || !legalFactor(L) || L.rows()<21) return false;
  Eigen::MatrixXd selected(L.rows()-6,L.cols()); selected.topRows<15>()=L.topRows<15>();
  selected.bottomRows(L.rows()-21)=L.bottomRows(L.rows()-21);
  return compressFactor(selected,retained);
}
bool propagateClonedFactor(const Eigen::MatrixXd& L,const Matrix15d& F,
                          const Eigen::MatrixXd& Qroot,Eigen::MatrixXd* propagated) {
  if(!propagated || !legalFactor(L) || Qroot.rows()!=15 || !F.allFinite() || !Qroot.allFinite()) return false;
  Eigen::MatrixXd columns=Eigen::MatrixXd::Zero(L.rows(),L.cols()+Qroot.cols());
  columns.leftCols(L.cols())=L; columns.topLeftCorner(15,L.cols())=F*L.topRows<15>();
  columns.topRightCorner(15,Qroot.cols())=Qroot;
  return compressFactor(columns,propagated);
}
bool josephClonedFactor(const Eigen::MatrixXd& L,const Eigen::MatrixXd& K,const Eigen::MatrixXd& H,
                       const Eigen::MatrixXd& Rroot,Eigen::MatrixXd* updated) {
  if(!updated || !legalFactor(L) || K.rows()!=L.rows() || H.cols()!=L.rows() || K.cols()!=H.rows() ||
      Rroot.rows()!=H.rows() || !K.allFinite() || !H.allFinite() || !Rroot.allFinite()) return false;
  Eigen::MatrixXd columns(L.rows(),L.cols()+Rroot.cols());
  columns.leftCols(L.cols())=(Eigen::MatrixXd::Identity(L.rows(),L.rows())-K*H)*L;
  columns.rightCols(Rroot.cols())=K*Rroot;
  return compressFactor(columns,updated);
}
bool causalVisualTime(uint64_t observation_ns,uint64_t target_ns) {
  return observation_ns>0 && observation_ns<=target_ns;
}
void linearizeCausalImu(const Eigen::Matrix3d& R,const Eigen::Vector3d& a,
    const Eigen::Vector3d& w,double d,Matrix15d* F,Eigen::Matrix<double,15,6>* G) {
  const Eigen::Vector3d phi=w*d; const double angle=phi.norm(); const Eigen::Matrix3d X=crossMatrix(phi);
  Eigen::Matrix3d J;
  if(angle<1e-6) J=Eigen::Matrix3d::Identity()-0.5*X+X*X/6.0;
  else J=Eigen::Matrix3d::Identity()-(1-std::cos(angle))/(angle*angle)*X+
      (angle-std::sin(angle))/(angle*angle*angle)*X*X;
  const Eigen::Matrix3d Theta=angle<1e-12 ? Eigen::Matrix3d::Identity() :
      Eigen::AngleAxisd(-angle,phi.normalized()).toRotationMatrix();
  const Eigen::Matrix3d A=-R*crossMatrix(a);
  *F=Matrix15d::Identity(); F->block<3,3>(0,3)=d*Eigen::Matrix3d::Identity();
  F->block<3,3>(6,6)=Theta; F->block<3,3>(6,12)=-J*d;
  F->block<3,3>(3,6)=A*Theta*d; F->block<3,3>(3,9)=-R*d; F->block<3,3>(3,12)=-A*J*d*d;
  F->block<3,3>(0,6)=0.5*d*F->block<3,3>(3,6); F->block<3,3>(0,9)=-0.5*R*d*d;
  F->block<3,3>(0,12)=0.5*d*F->block<3,3>(3,12);
  G->setZero(); G->block<3,3>(3,0)=R*d; G->block<3,3>(0,0)=0.5*R*d*d;
  G->block<3,3>(6,3)=J*d; G->block<3,3>(3,3)=A*J*d*d;
  G->block<3,3>(0,3)=0.5*d*G->block<3,3>(3,3);
}
bool correlatedVisualJoseph(const Eigen::MatrixXd& L,const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& V,const Eigen::MatrixXd& H,
                            const Eigen::MatrixXd& selector,
                            Eigen::MatrixXd* gain,Eigen::MatrixXd* updated) {
  if(!gain || !updated || !legalFactor(L) || H.cols()!=L.rows() || B.cols()!=L.cols() ||
     B.rows()!=H.rows() || V.rows()!=H.rows() || selector.rows()!=L.rows() || selector.cols()!=L.rows() ||
     !B.allFinite() || !V.allFinite() || !H.allFinite() || !selector.allFinite() ||
     (selector-selector.transpose()).norm()>1e-8 || (selector*selector-selector).norm()>1e-8 ||
     (H*selector).norm()<=1e-12) return false;
  const Eigen::MatrixXd G=H*L+B;
  const Eigen::MatrixXd innovation=G*G.transpose()+V*V.transpose();
  Eigen::LLT<Eigen::MatrixXd> solver(innovation);
  if(solver.info()!=Eigen::Success) return false;
  *gain=selector*L*G.transpose()*solver.solve(Eigen::MatrixXd::Identity(H.rows(),H.rows()));
  Eigen::MatrixXd columns(L.rows(),L.cols()+V.cols());
  columns.leftCols(L.cols())=L-*gain*G;
  columns.rightCols(V.cols())=*gain*V;
  return gain->allFinite() && compressFactor(columns,updated);
}
}  // namespace dog_prior_map_localization
