#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_directional_covariance.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <iostream>
#include <stdexcept>
namespace p=dog_prior_map_fastlio2_frontend_exp;
void require(bool ok,const char* why) {if(!ok) throw std::runtime_error(why);}
Eigen::Matrix3d rotation(const Eigen::Vector3d& v) {return Eigen::AngleAxisd(v.norm(),v.normalized()).toRotationMatrix();}
int main() {try {
  p::Pose3d lidar,predicted,extr;
  lidar.position<<1,2,3;lidar.orientation=rotation(Eigen::Vector3d(.2,-.4,.6));
  extr.position<<.08,.03,-.12;extr.orientation=rotation(Eigen::Vector3d(.15,.1,-.2));
  p::Pose3d nominal=lidar;
  const Eigen::Vector3d theta(.01,-.015,.02);
  lidar.orientation=rotation(theta)*nominal.orientation;
  auto measured=p::lidarMeasurementToImu(lidar,extr);
  predicted=measured;predicted.orientation=rotation(Eigen::Vector3d(.12,-.1,.05))*measured.orientation;
  p::CoupledVector6 l;l<<.01,.02,1,2,3,4;
  p::CoupledMatrix6 Q=p::CoupledMatrix6::Identity(),R0=Q;
  R0.topLeftCorner<3,3>()*=.04;R0.bottomRightCorner<3,3>()*=.01;
  // Mix translation/rotation, unlike a test where covariance stays diagonal.
  Q(0,0)=Q(3,3)=std::cos(.4);Q(0,3)=-std::sin(.4);Q(3,0)=std::sin(.4);
  auto r=p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,predicted,extr);
  require(r.valid && r.multipliers.minCoeff()==20 && r.minimum_eigenvalue>0,"SPD clipped inflation");
  Eigen::Matrix<double,6,6> fd;
  for(int c=0;c<6;++c) {
    auto sample=[&](double h) {
      p::Pose3d perturbed=lidar;
      if(c<3) perturbed.position(c)+=.8*h;
      else {Eigen::Vector3d d=theta;d(c-3)+=h;perturbed.orientation=rotation(d)*nominal.orientation;}
      auto imu=p::lidarMeasurementToImu(perturbed,extr);
      p::CoupledVector6 residual;residual.head<3>()=imu.position-predicted.position;
      residual.tail<3>()=p::so3Log(predicted.orientation.toRotationMatrix().transpose()*imu.orientation.toRotationMatrix());
      return residual;
    };
    fd.col(c)=(sample(1e-6)-sample(-1e-6))/2e-6;
  }
  require((fd-r.jacobian).cwiseAbs().maxCoeff()<2e-8,"real lidar/imu residual finite difference including lever and nonzero innovation");
  require(r.strong_block_difference<1e-12,"strong chart block retained");
  const auto solver=r.jacobian.colPivHouseholderQr();
  p::CoupledMatrix6 left=solver.solve(r.covariance);
  p::CoupledMatrix6 pulled=solver.solve(left.transpose()).transpose();
  require((Q.rightCols<4>().transpose()*(pulled-r.chart_baseline)*Q.rightCols<4>()).norm()<1e-12,"independent emitted strong block");
  for(int i=0;i<2;++i) require(std::abs(Q.col(i).dot(pulled*Q.col(i))-r.multipliers(i)*r.weak_variances(i))<1e-12,"actual weak variance multiplier");
  Eigen::SelfAdjointEigenSolver<p::CoupledMatrix6> added(r.covariance-R0);
  require(added.eigenvalues().minCoeff()>-1e-12,"inflation PSD not confidence gain");
  nominal=lidar;predicted=measured;r=p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,predicted,extr);
  require((r.jacobian.bottomRightCorner<3,3>()-measured.orientation.toRotationMatrix().transpose()).norm()<1e-12,"zero innovation right tangent");
  extr.position.setZero();measured=p::lidarMeasurementToImu(lidar,extr);
  r=p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,measured,extr);
  require(r.valid && r.jacobian.topRightCorner<3,3>().norm()==0,"zero lever");
  l(0)=-1;require(!p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,predicted,extr).valid,"negative curvature nominal fallback");
  l(0)=.01;Q(0,0)=2;require(!p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,predicted,extr).valid,"invalid basis fallback");
  Q.setIdentity();R0(0,0)=-1;require(!p::buildDirectionalCovariance(l,Q,2,R0,nominal,lidar,predicted,extr).valid,"invalid noise fallback");
  std::cout<<"R7 actual residual Jacobian, SPD, lever, weak inflation/strong retention, fallbacks PASS\n";
  return 0;
}catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}}
