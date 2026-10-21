#pragma once
#include "../../p62_r6_dog_shadow_prototype/tools/contract.hpp"

namespace cause {
using namespace shadow;
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v){
  Eigen::Matrix3d s;s<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return s;
}
// xi=(rho[m],phi[rad]), right/body tangent; translation is J(phi)*rho.
inline Pose expSE3(const Vector6& xi){
  require(xi.allFinite(),"nonfinite SE3 exponential");
  const Eigen::Vector3d p=xi.tail<3>();const double a=p.norm(),a2=a*a;
  const Eigen::Matrix3d w=skew(p),w2=w*w;
  double b,c;
  if(a<1e-5){b=.5-a2/24+a2*a2/720;c=1./6-a2/120+a2*a2/5040;}
  else{b=(1-std::cos(a))/a2;c=(a-std::sin(a))/(a2*a);}
  Pose t=Pose::Identity();
  if(a>0)t.block<3,3>(0,0)=Eigen::AngleAxisd(a,p/a).toRotationMatrix();
  t.block<3,1>(0,3)=(Eigen::Matrix3d::Identity()+b*w+c*w2)*xi.head<3>();
  require(rigid(t),"invalid SE3 exponential result");return t;
}
inline Vector6 logSE3(const Pose& t){
  require(rigid(t),"invalid SE3 logarithm input");
  // PCL returns float matrices. Normalized quaternion removes only their
  // tolerated rotational rounding error, not any estimated pose/bias.
  Eigen::Quaterniond q(t.block<3,3>(0,0));q.normalize();if(q.w()<0)q.coeffs()*=-1;
  const double n=q.vec().norm(),a=2*std::atan2(n,q.w());
  Eigen::Vector3d phi=2*q.vec();if(n>1e-12)phi=(a/n)*q.vec();
  const auto w=skew(phi);const double a2=phi.squaredNorm();double v;
  if(a<1e-5)v=1./12+a2/720+a2*a2/30240;
  else v=(1-.5*a/std::tan(.5*a))/a2;
  Vector6 xi;xi.tail<3>()=phi;
  xi.head<3>()=(Eigen::Matrix3d::Identity()-.5*w+v*w*w)*t.block<3,1>(0,3);
  require(xi.allFinite(),"nonfinite SE3 logarithm result");return xi;
}
struct Seeds {
  bool valid=false;std::string status="NO_VALID_DISAGREEMENT_DIRECTION";
  Vector6 xi=Vector6::Constant(NAN);double scale=NAN,reconstruction_error=NAN;
  std::array<Pose,2> poses{{Pose::Constant(NAN),Pose::Constant(NAN)}};
};
inline Seeds disagreement(const Pose& prediction,const Pose& nominal){
  Seeds out;if(!rigid(prediction)||!rigid(nominal))return out;
  const Pose relative=prediction.inverse()*nominal;
  out.xi=logSE3(relative);
  const double t=out.xi.head<3>().norm(),r=out.xi.tail<3>().norm();
  out.reconstruction_error=(expSE3(out.xi)-relative).norm();
  require(out.reconstruction_error<1e-5,"SE3 logarithm reconstruction exceeds native-roundoff tolerance");
  if(t<=1e-10&&r<=1e-10)return out;
  // Coordinate-aware floating-point guard: subtraction of world positions
  // may lose more precision than s itself. Fixed formula, not a search.
  const double eps=std::numeric_limits<double>::epsilon();
  const Eigen::Matrix3d rp=prediction.block<3,3>(0,0);
  const double inflation=std::max(1.,Eigen::JacobiSVD<Eigen::Matrix3d>(rp).singularValues().maxCoeff());
  const double orthogonal_error=(rp.transpose()*rp-Eigen::Matrix3d::Identity()).norm();
  const double t_cap=(.30-64*eps*(1+prediction.block<3,1>(0,3).norm()+.30))/inflation;
  const double r_cap=3*PI/180-64*eps*(1+rp.norm())-8*orthogonal_error;
  require(t_cap>0&&r_cap>0,"insufficient coordinate precision for bounded seed");
  out.scale=1.;if(t>1e-10)out.scale=std::min(out.scale,t_cap/t);
  if(r>1e-10)out.scale=std::min(out.scale,r_cap/r);
  out.scale=std::nextafter(out.scale,0.);
  require(std::isfinite(out.scale)&&out.scale>0,"invalid seed scale");
  for(int i=0;i<2;++i){
    out.poses[i]=prediction*expSE3((i==0?1.:-1.)*out.scale*out.xi);
    require(rigid(out.poses[i]),"invalid disagreement seed");
    require(translation(out.poses[i],prediction)<=.30&&rotation(out.poses[i],prediction)<=3.,"physical seed limit violation");
  }
  out.valid=true;out.status="PREDICTION_DISAGREEMENT_DIRECTION";return out;
}
} // namespace cause
