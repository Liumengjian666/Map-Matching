#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace dog_prior_map_fastlio2_frontend_exp;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
MovingInitializationState fixture() {
  MovingInitializationState s;
  s.stamp_ns = 1000000000;
  s.map_T_imu.position << 1., 2., 3.;
  s.map_T_imu.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitY()));
  s.T_imu_lidar.position << .08, .029, .03;
  s.T_imu_lidar.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(.15, Eigen::Vector3d::UnitX()));
  s.velocity_map << .7, -.2, .1;
  s.gyro_bias_imu << .01, -.02, .03;
  s.accel_bias_imu << .04, -.05, .06;
  s.gravity_map << 0, 0, -9.809;
  s.gravity_tangent_basis.col(0) = Eigen::Vector3d::UnitX();
  s.gravity_tangent_basis.col(1) = Eigen::Vector3d::UnitY();
  s.covariance.setIdentity(); s.covariance *= .04;
  s.covariance(16,16)=.09;
  s.covariance(0, 15) = s.covariance(15, 0) = .005;
  s.covariance(0, 16) = s.covariance(16, 0) = -.007;
  return s;
}
int main() {
  try {
    RuntimeParameters p; std::string reason;
    auto s = fixture(); FastLio2IkfomFrontend f(p);
    require(f.initializeMoving(s, &reason), reason.c_str());
    auto x = f.getState();
    require(x.stamp_ns == s.stamp_ns, "stamp");
    require((x.velocity-s.velocity_map).norm()<1e-14 && (x.gyro_bias-s.gyro_bias_imu).norm()<1e-14 &&
            (x.accel_bias-s.accel_bias_imu).norm()<1e-14 && (x.gravity-s.gravity_map).norm()<1e-12, "state");
    require((x.map_T_imu.position-s.map_T_imu.position).norm()<1e-14 &&
        x.map_T_imu.orientation.angularDistance(s.map_T_imu.orientation)<1e-14, "pose");
    require((x.T_imu_lidar_translation-s.T_imu_lidar.position).norm()<1e-14 &&
        (x.T_imu_lidar_rotation-s.T_imu_lidar.orientation.toRotationMatrix()).norm()<1e-14, "extrinsic");
    require(std::abs(x.covariance(21,21)-.04/(9.809*9.809))<1e-14, "S2 covariance units");
    require(std::abs(x.covariance(0,21)+.005/9.809)<1e-14 &&
            std::abs(x.covariance(0,22)+.007/9.809)<1e-14, "signed S2 gravity cross covariance");
    require(std::abs(x.covariance(22,22)-.09/(9.809*9.809))<1e-14, "anisotropic S2 covariance");
    for(int i=6;i<12;++i) {
      require(std::abs(x.covariance(i,i)-1e-12)<1e-20, "extrinsic variance");
      auto row=x.covariance.row(i).eval(); row(i)=0;
      require(row.norm()==0, "fixed extrinsic cross covariance");
    }
    require(!f.initializeMoving(s,&reason) && f.getState().stamp_ns==s.stamp_ns, "reinitialization");
    auto tilted=s;
    const Eigen::Matrix3d tilt=Eigen::AngleAxisd(.7,Eigen::Vector3d(1.,2.,3.).normalized()).toRotationMatrix();
    tilted.gravity_map=tilt*s.gravity_map; tilted.gravity_tangent_basis=tilt*s.gravity_tangent_basis;
    FastLio2IkfomFrontend tilted_filter(p);
    require(tilted_filter.initializeMoving(tilted,&reason),reason.c_str());
    // Independently reconstruct Cartesian covariance through pinned S2 type-1
    // boxplus Jacobian M=-hat(g)*Bx, including signed position/gravity cross.
    const auto g=tilted.gravity_map; const double l=g.norm(); Eigen::Matrix<double,3,2> B;
    B << -g.y(),-g.z(), l-g.y()*g.y()/(l+g.x()),-g.z()*g.y()/(l+g.x()),
         -g.z()*g.y()/(l+g.x()),l-g.z()*g.z()/(l+g.x()); B/=l;
    Eigen::Matrix3d hat; hat<<0,-g.z(),g.y(),g.z(),0,-g.x(),-g.y(),g.x(),0;
    const Eigen::Matrix<double,3,2> M=-hat*B;
    const auto P=tilted_filter.getState().covariance;
    require((M*P.block<2,2>(21,21)*M.transpose()-tilted.gravity_tangent_basis*
        tilted.covariance.block<2,2>(15,15)*tilted.gravity_tangent_basis.transpose()).norm()<1e-13,
        "non-axis Cartesian gravity covariance");
    require((P.block<1,2>(0,21)*M.transpose()-tilted.covariance.block<1,2>(0,15)*
        tilted.gravity_tangent_basis.transpose()).norm()<1e-13, "non-axis gravity cross covariance");
    for(int bad=0;bad<7;++bad) {
      auto invalid=s;
      if(bad==0)invalid.stamp_ns=0;
      if(bad==1)invalid.gravity_map*=.9;
      if(bad==2)invalid.map_T_imu.orientation.coeffs()*=2;
      if(bad==3)invalid.covariance(0,0)=-1;
      if(bad==4)invalid.covariance(0,1)=.01;
      if(bad==5)invalid.gravity_tangent_basis.col(0)=Eigen::Vector3d::UnitZ();
      if(bad==6)invalid.velocity_map.x()=std::numeric_limits<double>::quiet_NaN();
      FastLio2IkfomFrontend fresh(p);
      require(!fresh.initializeMoving(invalid,&reason) && !fresh.initialized(), "invalid state not atomic");
    }
    RuntimeParameters incompatible=p; incompatible.gravity_mps2=9.809+1.5e-8;
    FastLio2IkfomFrontend rejected(incompatible); auto near_boundary=s;
    near_boundary.gravity_map*= (9.809+.75e-8)/9.809;
    const auto before=rejected.getState();
    require(!rejected.initializeMoving(near_boundary,&reason) && !rejected.initialized(),
        "pinned S2/runtime tolerance must reject before mutation");
    require(rejected.getState().stamp_ns==before.stamp_ns &&
        (rejected.getState().covariance-before.covariance).norm()==0, "failed injection mutated state");
    ImuSample a,b; a.stamp_ns=s.stamp_ns; b.stamp_ns=s.stamp_ns+10000000;
    a.acceleration=b.acceleration=s.map_T_imu.orientation.conjugate()*(-s.gravity_map)+s.accel_bias_imu;
    a.angular_velocity=b.angular_velocity=s.gyro_bias_imu;
    require(f.predictInterval(a,b,&reason), reason.c_str());
    require((f.getState().map_T_imu.position-(s.map_T_imu.position+.01*s.velocity_map)).norm()<1e-10,
        "nonzero moving velocity propagation");
    std::cout<<"MOVING_INITIALIZATION_SYNTHETIC_PASS; REAL_STATE_NOT_ACCEPTED_BY_THIS_TEST\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
