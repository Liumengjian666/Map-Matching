#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include <Eigen/Cholesky>
#include <iostream>
#include <stdexcept>

using namespace dog_prior_map_fastlio2_frontend_exp;
int main() {
  try {
    RuntimeParameters parameters;
    parameters.static_init_samples=20;
    FastLio2IkfomFrontend frontend(parameters);
    std::vector<ImuSample,Eigen::aligned_allocator<ImuSample>> samples;
    for (int i=0;i<20;++i) {
      ImuSample s; s.stamp_ns=1'000'000'000ULL+5'000'000ULL*i;
      s.acceleration=Eigen::Vector3d(0,0,parameters.gravity_mps2);
      samples.push_back(s);
    }
    Pose3d initial, extrinsic;
    initial.position=Eigen::Vector3d(1,2,3);
    initial.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(0.3,Eigen::Vector3d::UnitZ()));
    extrinsic.position=Eigen::Vector3d(0.1,-0.2,0.3);
    std::string reason;
    if (!frontend.initializeStatic(samples,initial,extrinsic,&reason))
      throw std::runtime_error(reason);
    FilterSnapshot snapshot=frontend.getState();
    snapshot.velocity=Eigen::Vector3d(0.3,-0.1,0.2);
    snapshot.gyro_bias=Eigen::Vector3d(0.01,0.02,0.03);
    snapshot.accel_bias=Eigen::Vector3d(0.1,0.2,0.3);
    snapshot.covariance=Eigen::MatrixXd::Identity(23,23);
    for(int i=0;i<23;++i) snapshot.covariance(i,i)=1.0+0.1*i;
    snapshot.covariance(0,21)=snapshot.covariance(21,0)=0.2;
    snapshot.covariance(4,22)=snapshot.covariance(22,4)=-0.3;
    snapshot.covariance(12,0)=snapshot.covariance(0,12)=0.1;
    if(!frontend.setWindowPredictionSeed(snapshot,&reason)) throw std::runtime_error(reason);
    snapshot=frontend.getState();
    FixedLagInitializationSeed seed;
    if(!frontend.makeFixedLagInitializationSeed(&seed,&reason)) throw std::runtime_error(reason);
    Eigen::Matrix<double,15,23> S=Eigen::Matrix<double,15,23>::Zero();
    const int expected_blocks[]={3,0,12,15,18};
    for(int b=0;b<5;++b) S.block<3,3>(3*b,expected_blocks[b]).setIdentity();
    const Eigen::Matrix<double,15,15> Pxx=S*snapshot.covariance*S.transpose();
    const Eigen::Matrix<double,15,2> Pxg=S*snapshot.covariance.block(0,21,23,2);
    const Eigen::Matrix2d Pgg=snapshot.covariance.block<2,2>(21,21);
    const Eigen::Matrix<double,15,15> expected=Pxx-Pxg*Pgg.llt().solve(Pxg.transpose());
    if((expected-seed.covariance15).norm()>1e-12 ||
        (seed.information15*seed.covariance15-Eigen::Matrix<double,15,15>::Identity()).norm()>1e-11 ||
        seed.gravity_conditioning_delta_norm<=0 ||
        (seed.velocity-snapshot.velocity).norm()>1e-12 ||
        (seed.gyro_bias-snapshot.gyro_bias).norm()>1e-12 ||
        (seed.accel_bias-snapshot.accel_bias).norm()>1e-12 ||
        (seed.gravity-snapshot.gravity).norm()>1e-12 || seed.stamp_ns!=snapshot.stamp_ns ||
        (seed.map_T_imu.position-snapshot.map_T_imu.position).norm()>1e-12 ||
        (seed.map_T_imu.orientation.toRotationMatrix()-snapshot.map_T_imu.orientation.toRotationMatrix()).norm()>1e-12 ||
        (frontend.getState().covariance-snapshot.covariance).norm()!=0.0)
      throw std::runtime_error("23D selection/gravity conditional/state mapping failed");
    std::cout<<"IKFOM_23D_TO_15D_MTK_MAPPING_AND_GRAVITY_CONDITIONAL_PASS delta="
             <<seed.gravity_conditioning_delta_norm<<'\n';
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
