#include "dog_prior_map_localization/core/msckf_visual_measurement.hpp"
#include <iostream>
#include <stdexcept>
#include <Eigen/Eigenvalues>
using namespace dog_prior_map_localization;
void require(bool good,const char* why) { if(!good) throw std::runtime_error(why); }
Eigen::Matrix3d exp(const Eigen::Vector3d& d) { if(d.norm()<1e-15)return Eigen::Matrix3d::Identity(); return Eigen::AngleAxisd(d.norm(),d.normalized()).toRotationMatrix(); }
int main() {
  try {
    RectifiedCamera camera; camera.K<<297.160965789772,0,317.73333375994946,0,265.24651866020713,233.53737010585292,0,0,1;
    camera.R_imu_camera=exp(Eigen::Vector3d(.3,-.7,.2)); camera.p_imu_camera<<.1796755,.04727031,-.01985536;
    VisualPoseClone pose; pose.R=exp(Eigen::Vector3d(.05,.01,.02)); pose.p<<.3,.1,.2;
    const Eigen::Vector3d point=pose.p+pose.R*(camera.p_imu_camera+camera.R_imu_camera*Eigen::Vector3d(.3,.2,5));
    auto analytic=linearizeReprojection(pose,camera,point); require(analytic.valid,"projection invalid");
    double max_error=0; constexpr double epsilon=1e-6;
    for(int j=0;j<6;++j) {
      auto plus=pose,minus=pose;
      if(j<3) {plus.p(j)+=epsilon; minus.p(j)-=epsilon;}
      else {Eigen::Vector3d d=Eigen::Vector3d::Zero(); d(j-3)=epsilon; plus.R=pose.R*exp(d);minus.R=pose.R*exp(-d);}
      const Eigen::Vector2d numerical=(linearizeReprojection(plus,camera,point).prediction-linearizeReprojection(minus,camera,point).prediction)/(2*epsilon);
      max_error=std::max(max_error,(numerical-analytic.H_pose.col(j)).cwiseAbs().maxCoeff());
    }
    for(int j=0;j<3;++j) {
      Eigen::Vector3d d=Eigen::Vector3d::Zero(); d(j)=epsilon;
      const Eigen::Vector2d numerical=(linearizeReprojection(pose,camera,point+d).prediction-linearizeReprojection(pose,camera,point-d).prediction)/(2*epsilon);
      max_error=std::max(max_error,(numerical-analytic.H_point.col(j)).cwiseAbs().maxCoeff());
    }
    require(max_error<1e-6,"reprojection Jacobian mismatch");
    std::vector<VisualPoseClone> clones(3,pose); std::vector<PixelObservation> observations;
    for(std::size_t i=0;i<3;++i) {clones[i].p.x()+=.3*i;observations.push_back(PixelObservation{i,linearizeReprojection(clones[i],camera,point).prediction});}
    Eigen::Vector3d reconstructed; std::string reason;
    require(triangulateVisualTrack(clones,camera,observations,&reconstructed,&reason),"triangulation failed");
    require((reconstructed-point).norm()<1e-7,"triangulation mismatch");
    Eigen::MatrixXd Hf(6,3),Hx=Eigen::MatrixXd::Zero(6,18); Eigen::VectorXd r=Eigen::VectorXd::Zero(6);
    for(int i=0;i<3;++i) {const auto jet=linearizeReprojection(clones[i],camera,point);Hf.middleRows<2>(2*i)=jet.H_point;Hx.block<2,6>(2*i,6*i)=jet.H_pose;}
    Eigen::MatrixXd reduced; Eigen::VectorXd reduced_r;
    require(eliminateVisualFeature(Hf,Hx,r,&reduced,&reduced_r),"nullspace failed");
    Eigen::VectorXd global_translation=Eigen::VectorXd::Zero(18);
    for(int i=0;i<3;++i) global_translation.segment<3>(6*i)=Eigen::Vector3d(.3,.4,.1);
    require((reduced*global_translation).norm()<1e-9,"global translation gauge lost");
    Eigen::VectorXd global_rotation=Eigen::VectorXd::Zero(18);
    const Eigen::Vector3d world_axis(.2,.1,.3);
    for(int i=0;i<3;++i) {
      global_rotation.segment<3>(6*i)=world_axis.cross(clones[i].p);
      global_rotation.segment<3>(6*i+3)=clones[i].R.transpose()*world_axis;
    }
    require((reduced*global_rotation).norm()<1e-9,"global rotation gauge lost");
    Eigen::MatrixXd L=Matrix15d::Identity(),augmented;
    require(appendPoseCloneFactor(L,&augmented),"clone augmentation failed");
    const Eigen::MatrixXd P=augmented*augmented.transpose();
    require((P.block<15,3>(0,15)-L*L.topRows<3>().transpose()).norm()<1e-12,"clone cross covariance missing");
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(P);
    require(eigen.eigenvalues().head(6).cwiseAbs().maxCoeff()<1e-12,"clone fake independent noise");
    Matrix15d F=Matrix15d::Identity();F(0,3)=.005;
    Eigen::MatrixXd propagated;
    require(propagateClonedFactor(augmented,F,Matrix15d::Identity()*.01,&propagated),"joint propagation failed");
    Eigen::MatrixXd Fjoint=Eigen::MatrixXd::Identity(21,21); Fjoint.topLeftCorner<15,15>()=F;
    Eigen::MatrixXd Q=Eigen::MatrixXd::Zero(21,21);Q.topLeftCorner<15,15>()=Matrix15d::Identity()*.0001;
    require((propagated*propagated.transpose()-Fjoint*P*Fjoint.transpose()-Q).norm()<1e-11,"joint propagation differs");
    Eigen::MatrixXd H=Eigen::MatrixXd::Zero(1,21);H(0,15)=1;
    const Eigen::MatrixXd prior=propagated*propagated.transpose();
    Eigen::MatrixXd K=prior*H.transpose()/( (H*prior*H.transpose())(0,0)+1.0);
    Eigen::MatrixXd updated;
    require(josephClonedFactor(propagated,K,H,Eigen::MatrixXd::Identity(1,1),&updated),"joint Joseph failed");
    require(updated.squaredNorm()<=propagated.squaredNorm()+1e-12,"standard visual update inflated covariance");
    Eigen::MatrixXd marginalized;require(marginalizeOldestPoseCloneFactor(updated,&marginalized),"marginalization failed");
    require((marginalized*marginalized.transpose()-(updated*updated.transpose()).topLeftCorner<15,15>()).norm()<1e-11,"marginalized by conditioning");
    // Nonzero interpolation/process cross-noise; an independence shortcut
    // produces a different gain and is not an equivalent update.
    Eigen::MatrixXd common=Eigen::MatrixXd::Zero(1,propagated.cols());common(0,0)=.2;
    Eigen::MatrixXd selected=Eigen::MatrixXd::Zero(21,21);selected(0,0)=selected(15,15)=1;
    Eigen::MatrixXd correlated_gain,correlated_root;
    require(correlatedVisualJoseph(propagated,common,Eigen::MatrixXd::Identity(1,1),H,selected,
                                  &correlated_gain,&correlated_root),"shared-noise Joseph failed");
    const Eigen::MatrixXd C=propagated*common.transpose();
    const Eigen::MatrixXd Rnoise=common*common.transpose()+Eigen::MatrixXd::Identity(1,1);
    const Eigen::MatrixXd innovation=H*prior*H.transpose()+Rnoise+H*C+C.transpose()*H.transpose();
    const Eigen::MatrixXd expected_gain=selected*(prior*H.transpose()+C)/innovation(0,0);
    require((correlated_gain-expected_gain).norm()<1e-12,"cross-noise gain mismatch");
    const Eigen::MatrixXd expected_P=prior-expected_gain*(H*prior+C.transpose())-
        (prior*H.transpose()+C)*expected_gain.transpose()+expected_gain*innovation*expected_gain.transpose();
    require((correlated_root*correlated_root.transpose()-expected_P).norm()<1e-11,"shared-noise covariance mismatch");
    require(causalVisualTime(10,10)&&causalVisualTime(9,10)&&!causalVisualTime(11,10),"future time accepted");
    // The runtime IMU right-error Jacobian, not a second test-only formula.
    const Eigen::Vector3d acceleration(.4,.2,9.5),angular(.2,-.1,.3);
    constexpr double interval=.005;
    const Eigen::Matrix3d initial_R=exp(Eigen::Vector3d(.1,.2,-.1));
    const Eigen::Matrix3d end_R=initial_R*exp(angular*interval);
    Matrix15d transition; Eigen::Matrix<double,15,6> noise_map;
    linearizeCausalImu(end_R,acceleration,angular,interval,&transition,&noise_map);
    auto predicted=[&](const Vector15d& error) {
      Vector15d output=Vector15d::Zero();
      const Eigen::Matrix3d R=initial_R*exp(error.segment<3>(6))*exp((angular-error.segment<3>(12))*interval);
      const Eigen::Vector3d a=R*(acceleration-error.segment<3>(9));
      output.segment<3>(0)=error.segment<3>(0)+error.segment<3>(3)*interval+.5*(a-end_R*acceleration)*interval*interval;
      output.segment<3>(3)=error.segment<3>(3)+(a-end_R*acceleration)*interval;
      Eigen::AngleAxisd aa(end_R.transpose()*R); output.segment<3>(6)=aa.axis()*aa.angle();
      output.tail<6>()=error.tail<6>(); return output;
    };
    double imu_error=0;
    for(int j=0;j<15;++j) {
      Vector15d e=Vector15d::Zero(); e(j)=1e-6;
      const Vector15d derivative=(predicted(e)-predicted(-e))/(2e-6);
      imu_error=std::max(imu_error,(derivative-transition.col(j)).cwiseAbs().maxCoeff());
    }
    require(imu_error<1e-8,"runtime IMU right-error Jacobian failed");
    // A split interval has ONE shared noise driver. Check a cloned partial
    // increment against the full increment and reject the independence shortcut.
    const double sigma=.1,d=.0025,D=.005;
    Eigen::Matrix<double,2,1> shared; shared<<d*sigma,D*sigma;
    const auto shared_P=(shared*shared.transpose()).eval();
    require(std::abs(shared_P(0,1)-sigma*sigma*d*D)<1e-18,"partial/full shared-noise correlation lost");
    FilterStateSnapshot saved; saved.square_root_authoritative=true;
    saved.visual_joint.factor=propagated; saved.visual_joint.clones=clones;
    saved.visual_joint.driver_active=true; saved.visual_joint.driver_head_stamp=12.5;
    const auto restored=saved;
    require((restored.visual_joint.factor-saved.visual_joint.factor).norm()==0 &&
      restored.visual_joint.driver_head_stamp==12.5 && restored.visual_joint.clones[1].p==clones[1].p,
      "joint history snapshot changed");
    std::cout<<"runtime_imu_jacobian_max="<<imu_error<<" shared_interval_cross_covariance="<<shared_P(0,1)<<'\n';
    std::cout<<"MSCKF_MEASUREMENT_CORE_PASS max_pixel_jacobian_error="<<max_error<<"; triangulation nullspace gauge exact_clone_cross_covariance singular_factor joint_prediction Joseph marginalization causal_time\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
