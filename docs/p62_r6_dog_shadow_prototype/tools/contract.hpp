#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <vector>

namespace shadow {
constexpr double PI=3.1415926535897932384626433832795;
using Pose=Eigen::Matrix4d;
using Vector6=Eigen::Matrix<double,6,1>;
using Matrix6=Eigen::Matrix<double,6,6>;
using Loss=std::array<double,8>;
inline void require(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
inline std::vector<std::string> csvFields(const std::string& text){
  std::stringstream stream(text);std::string v;std::vector<std::string> out;
  while(std::getline(stream,v,',')){if(!v.empty()&&v.back()=='\r')v.pop_back();out.push_back(v);}return out;
}
inline bool rigid(const Pose& t){return t.allFinite() &&
    (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-6 &&
    (t.block<3,3>(0,0).transpose()*t.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()<1e-4 &&
    std::abs(t.block<3,3>(0,0).determinant()-1)<1e-4;}
inline double angle(const Eigen::Matrix3d& r){return Eigen::AngleAxisd(r).angle()*180/PI;}
inline double translation(const Pose& a,const Pose& b){return (a.block<3,1>(0,3)-b.block<3,1>(0,3)).norm();}
inline double rotation(const Pose& a,const Pose& b){return angle(a.block<3,3>(0,0).transpose()*b.block<3,3>(0,0));}
inline bool duplicate(const Pose& a,const Pose& b){return translation(a,b)<=.10 && rotation(a,b)<=1.;}
inline bool bounded(const Pose& t,const Pose& initial){return rigid(t)&&translation(t,initial)<=.8&&rotation(t,initial)<=5.;}
inline int sector(double x,double y){
  require(std::isfinite(x)&&std::isfinite(y),"nonfinite scanner direction");
  double a=std::fmod(std::atan2(y,x)+PI,2*PI);if(a<0)a+=2*PI;
  return std::min(15,int(std::floor(a/(2*PI)*16)));
}
inline bool validLoss(const Loss& a){for(double x:a)if(!std::isfinite(x)||x<0||x>1)return false;return true;}
inline double paired(const Loss& j,const Loss& k){
  require(validLoss(j)&&validLoss(k),"invalid paired block losses");
  Loss differences;for(int b=0;b<8;++b)differences[b]=k[b]-j[b];
  std::sort(differences.begin(),differences.end());
  return .25*(differences[0]+differences[1]+differences[2]+differences[3]);
}
inline double mean(const Loss& a){require(validLoss(a),"invalid block loss");double s=0;for(double x:a)s+=x;return s/8;}
inline double median(Loss a){require(validLoss(a),"invalid block loss");std::sort(a.begin(),a.end());return (a[3]+a[4])/2;}
struct Candidate {
  int id=0,duplicate_of=-1;
  Pose initial=Pose::Identity(),pose=Pose::Identity();
  bool converged=false,finite=false,loss_valid=false,eligible=false;
  int iterations=0;double score=NAN,fitness=NAN,align_ms=0,validation_ms=0,overlap=NAN;
  Loss loss{{NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN}};
};
struct Selection {int native=-1,heldout_mean=-1,robust=-1;};
inline std::vector<int> pool(const std::vector<Candidate>& c){
  std::vector<int> out;for(size_t j=0;j<c.size();++j)
    if(c[j].finite && c[j].loss_valid && c[j].duplicate_of<0)out.push_back(j);return out;
}
inline Selection select(const std::vector<Candidate>& c){
  Selection out;const auto ids=pool(c);
  for(int j:ids){
    if(!c[j].eligible)continue;
    if(out.native<0||c[j].score>c[out.native].score)out.native=j;
    if(out.heldout_mean<0||mean(c[j].loss)<mean(c[out.heldout_mean].loss))out.heldout_mean=j;
    if(j==0)continue;bool dominates=true;
    for(int k:ids)if(j!=k&&!(paired(c[j].loss,c[k].loss)>.02))dominates=false;
    if(dominates){require(out.robust<0,"nonunique strict robust winner");out.robust=j;}
  }
  return out;
}
struct Budget {int used=0;bool consume(){if(used>=20)return false;++used;return true;}};
inline bool trigger(bool converged,int iterations,const Loss& losses){return !converged||iterations>=40||median(losses)>.25;}
inline Pose seed(const Pose& center,const Vector6& direction,double sign){
  require(rigid(center)&&direction.allFinite()&&direction.norm()>1e-12,"invalid seed direction");
  const double t=.8*direction.head<3>().norm(),r=direction.tail<3>().norm();
  double scale=std::numeric_limits<double>::infinity();
  if(t>1e-12)scale=std::min(scale,.30/t);
  if(r>1e-12)scale=std::min(scale,(3*PI/180)/r);
  require(std::isfinite(scale),"zero seed direction");
  Pose result=center;result.block<3,1>(0,3)+=.8*scale*sign*direction.head<3>();
  const Eigen::Vector3d rv=scale*sign*direction.tail<3>();
  if(rv.norm()>0)result.block<3,3>(0,0)=Eigen::AngleAxisd(rv.norm(),rv.normalized()).toRotationMatrix()*center.block<3,3>(0,0);
  return result;
}
inline void deduplicate(std::vector<Candidate>& c){
  for(size_t j=0;j<c.size();++j){c[j].duplicate_of=-1;if(!c[j].finite)continue;
    for(size_t k=0;k<j;++k)if(c[k].finite&&c[k].duplicate_of<0&&duplicate(c[j].pose,c[k].pose)){
      c[j].duplicate_of=k;break;}}
}
} // namespace shadow
